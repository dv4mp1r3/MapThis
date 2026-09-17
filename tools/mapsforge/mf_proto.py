#!/usr/bin/env python3
# Прототип читателя mapsforge .map (file version 3, writer 0.18.0)
# Цель: разобрать заголовок, tile index, декодировать тайл (POI/way блоки),
# отрендерить PNG и замерить плотность данных. Только stdlib (zlib).
#
# Спецификация: https://github.com/mapsforge/mapsforge/blob/master/docs/Specification-Binary-Map-File.md
# Сверено с исходниками writer'а: mapsforge-map-writer/src/main/java/org/mapsforge/map/writer/MapFileWriter.java
#
# Ключевые выводы (проверены по реальному файлу):
#  - tile index: записи 5 байт, порядок y-внешний/x-внутренний (writeSubfile: for tileY{for tileX})
#  - zoom table: строки по ВСЕМ зумам интервала min..max (не до base!)
#  - флаги POI: 0x80 имя, 0x40 housenumber, 0x20 elevation
#  - флаги way: 0x80 имя, 0x40 housenumber, 0x20 ref, 0x10 label position,
#               0x08 несколько way-блоков (мультиполигон: счётчик блоков),
#               0x04 double-delta кодирование координат
#  - счётчик way-блоков пишется ТОЛЬКО при флаге 0x08; внутри каждого блока
#    ВСЕГДА пишется счётчик координатных подблоков (1 + внутренние кольца)
#  - координаты: микроградусы, дельты к северо-западному углу тайла (со знаком)

import math
import struct
import sys
import time
import zlib
from collections import Counter

MAP_FILE = "planet_30.004_59.781_927ddaa9-mapsforge-osm/planet_30.004_59.781_927ddaa9.map"

# флаги way
W_NAME, W_HOUSENUMBER, W_REF, W_LABEL, W_MULTI, W_DOUBLE_DELTA = 0x80, 0x40, 0x20, 0x10, 0x08, 0x04
# флаги POI
P_NAME, P_HOUSENUMBER, P_ELEVATION = 0x80, 0x40, 0x20


# ---------------------------------------------------------------------------
# Низкоуровневые декодеры
# ---------------------------------------------------------------------------

class Buf:
    def __init__(self, data, pos=0, end=None):
        self.d = data
        self.pos = pos
        self.end = len(data) if end is None else end

    def read(self, n):
        if self.pos + n > self.end:
            raise EOFError("want %d bytes at %d, end=%d" % (n, self.pos, self.end))
        r = self.d[self.pos:self.pos + n]
        self.pos += n
        return r

    def u8(self):
        return self.read(1)[0]

    def u16(self):
        return struct.unpack(">H", self.read(2))[0]

    def u32(self):
        return struct.unpack(">I", self.read(4))[0]

    def u64(self):
        return struct.unpack(">Q", self.read(8))[0]

    def vbe_u(self):
        """VBE-U INT: 7 бит на байт, младшие группы вперёд, 0x80 = продолжение."""
        result = 0
        shift = 0
        while True:
            b = self.u8()
            result |= (b & 0x7F) << shift
            if not (b & 0x80):
                return result
            shift += 7

    def vbe_s(self):
        """VBE-S INT: последний байт = 6 бит данных + знаковый бит 0x40."""
        result = 0
        shift = 0
        while True:
            b = self.u8()
            if b & 0x80:
                result |= (b & 0x7F) << shift
                shift += 7
            else:
                result |= (b & 0x3F) << shift
                return -result if (b & 0x40) else result

    def string(self):
        n = self.vbe_u()
        return self.read(n).decode("utf-8", errors="replace")


# ---------------------------------------------------------------------------
# Заголовок файла
# ---------------------------------------------------------------------------

class ZoomInterval:
    __slots__ = ("base", "min", "max", "offset", "size", "index_offset", "index_size",
                 "data_offset", "tiles_x", "tiles_y", "x0", "y0")

    def __repr__(self):
        return "ZoomInterval(z=%d..%d base=%d, subfile=+%d size=%d, grid=%dx%d от x%d y%d)" % (
            self.min, self.max, self.base, self.offset, self.size, self.tiles_x, self.tiles_y,
            self.x0, self.y0)


class MapFile:
    def __init__(self, path):
        with open(path, "rb") as f:
            self.data = f.read()
        self.pois_tags = []
        self.way_tags = []
        self._parse_header()

    # -- Web Mercator (EPSG:3857), как у OSM/Google slippy tiles ----
    @staticmethod
    def lon_to_tile_x(lon, z):
        return (lon + 180.0) / 360.0 * (1 << z)

    @staticmethod
    def lat_to_tile_y(lat, z):
        lat = max(min(lat, 85.05112878), -85.05112878)
        r = math.log(math.tan(math.pi / 4 + math.radians(lat) / 2))
        return (1.0 - r / math.pi) / 2.0 * (1 << z)

    @staticmethod
    def tile_y_to_lat(y, z):
        n = math.pi * (1 - 2.0 * y / (1 << z))
        return math.degrees(math.atan(math.sinh(n)))

    @staticmethod
    def tile_x_to_lon(x, z):
        return x / float(1 << z) * 360.0 - 180.0

    def _parse_header(self):
        b = Buf(self.data)
        assert b.read(20) == b"mapsforge binary OSM"
        self.header_size = b.u32()
        self.version = b.u32()
        self.file_size = b.u64()
        self.creation_date = b.u64()
        self.min_lat = b.u32() / 1e6
        self.min_lon = b.u32() / 1e6
        self.max_lat = b.u32() / 1e6
        self.max_lon = b.u32() / 1e6
        self.tile_size = b.u16()
        self.projection = b.string()
        flags = b.u8()
        if flags & 0x40:  # start position
            self.start_lat = b.u32() / 1e6
            self.start_lon = b.u32() / 1e6
        if flags & 0x20:  # start zoom
            self.start_zoom = b.u8()
        if flags & 0x10:  # languages
            self.languages = b.string()
        if flags & 0x08:  # comment
            self.comment = b.string()
        if flags & 0x04:  # created by
            self.created_by = b.string()
        for _ in range(b.u16()):
            self.pois_tags.append(b.string())
        for _ in range(b.u16()):
            self.way_tags.append(b.string())
        self.intervals = []
        for _ in range(b.u8()):
            zi = ZoomInterval()
            zi.base = b.u8()
            zi.min = b.u8()
            zi.max = b.u8()
            zi.offset = b.u64()
            zi.size = b.u64()
            self.intervals.append(zi)
        self.header_end = b.pos
        for zi in self.intervals:
            zi.x0 = int(math.floor(self.lon_to_tile_x(self.min_lon, zi.base)))
            x1 = int(math.floor(self.lon_to_tile_x(self.max_lon, zi.base)))
            zi.y0 = int(math.floor(self.lat_to_tile_y(self.max_lat, zi.base)))
            y1 = int(math.floor(self.lat_to_tile_y(self.min_lat, zi.base)))
            zi.tiles_x = x1 - zi.x0 + 1
            zi.tiles_y = y1 - zi.y0 + 1
            zi.index_offset = zi.offset
            zi.index_size = zi.tiles_x * zi.tiles_y * 5
            zi.data_offset = zi.offset + zi.index_size

    def entry(self, zi, ix, iy):
        """Запись tile index. Порядок: y-внешний, x-внутренний."""
        p = zi.index_offset + (iy * zi.tiles_x + ix) * 5
        water = bool(self.data[p] & 0x80)
        off = ((self.data[p] & 0x7F) << 32) | struct.unpack_from(">I", self.data, p + 1)[0]
        return water, off

    def all_entries(self, zi):
        out = []
        for iy in range(zi.tiles_y):
            for ix in range(zi.tiles_x):
                out.append(self.entry(zi, ix, iy))
        return out


# ---------------------------------------------------------------------------
# Декодирование тайла
# ---------------------------------------------------------------------------

def tile_block_range(mf, zi, ix, iy):
    """(water, off, block_end) тайла. Блоки пишутся последовательно в порядке
    индекса, поэтому block(i) = [off(i), off(i+1)); у последней записи конец =
    конец subfile. Пустой тайл: off(i) == off(i+1) (off==0 не бывает — writer
    всегда пишет текущую позицию)."""
    water, off = mf.entry(zi, ix, iy)
    k = iy * zi.tiles_x + ix + 1
    if k >= zi.tiles_x * zi.tiles_y:
        block_end = zi.size
    else:
        p = zi.index_offset + k * 5
        block_end = ((mf.data[p] & 0x7F) << 32) | struct.unpack_from(">I", mf.data, p + 1)[0]
    return water, off, block_end


def decode_tile(mf, zi, tile_x, tile_y):
    ix, iy = tile_x - zi.x0, tile_y - zi.y0
    if not (0 <= ix < zi.tiles_x and 0 <= iy < zi.tiles_y):
        raise KeyError("тайл вне сетки")
    water, off, block_end = tile_block_range(mf, zi, ix, iy)
    if block_end <= off:
        return {"empty": True, "water": water}
    b = Buf(mf.data, zi.offset + off, zi.offset + block_end)

    levels = zi.max - zi.min + 1
    zoom_table = [(b.vbe_u(), b.vbe_u()) for _ in range(levels)]
    first_way_offset = b.vbe_u()
    poi_end = b.pos + first_way_offset
    pois = []
    while b.pos < poi_end:
        pois.append(decode_poi(mf, b))
    assert b.pos == poi_end, "POI-секция не сошлась: %d != %d" % (b.pos, poi_end)
    ways = []
    while b.pos < b.end:
        ways.append(decode_way(mf, b))
    assert b.pos == b.end, "way-секция не сошлась: %d != %d" % (b.pos, b.end)
    return {"water": water, "zoom_table": zoom_table, "pois": pois, "ways": ways,
            "tile_x": tile_x, "tile_y": tile_y, "zi": zi}


def decode_poi(mf, b):
    lat_diff = b.vbe_s()   # poi - северо-западный угол (отрицательный к югу)
    lon_diff = b.vbe_s()
    special = b.u8()
    layer = special >> 4  # writer пишет слой как есть, 0-10 (без ±5)
    ntags = special & 0x0F
    tags = [b.vbe_u() for _ in range(ntags)]
    flags = b.u8()
    name = housenum = None
    elevation = None
    if flags & P_NAME:
        name = b.string()
    if flags & P_HOUSENUMBER:
        housenum = b.string()
    if flags & P_ELEVATION:
        elevation = b.vbe_s()
    return {"lat_diff": lat_diff, "lon_diff": lon_diff, "layer": layer, "tags": tags,
            "name": name, "hn": housenum, "elev": elevation}


def decode_way(mf, b):
    data_size = b.vbe_u()
    wend = b.pos + data_size
    subtile_bitmap = b.u16()
    special = b.u8()
    layer = special >> 4  # writer пишет слой как есть, 0-10 (без ±5)
    ntags = special & 0x0F
    tags = [b.vbe_u() for _ in range(ntags)]
    flags = b.u8()
    name = hnum = ref = None
    label_pos = None
    if flags & W_NAME:
        name = b.string()
    if flags & W_HOUSENUMBER:
        hnum = b.string()
    if flags & W_REF:
        ref = b.string()
    if flags & W_LABEL:
        label_pos = (b.vbe_s(), b.vbe_s())
    double_delta = bool(flags & W_DOUBLE_DELTA)

    blocks = []  # каждый элемент: список колец (первое — внешний контур)
    nblocks = b.vbe_u() if flags & W_MULTI else 1
    for _ in range(nblocks):
        nrings = b.vbe_u()   # 1 + внутренние кольца (всегда пишется)
        rings = []
        for _ in range(nrings):
            nnodes = b.vbe_u()
            lat = b.vbe_s()
            lon = b.vbe_s()
            nodes = [(lat, lon)]
            d_lat = d_lon = 0
            for _ in range(nnodes - 1):
                if double_delta:
                    d_lat += b.vbe_s()
                    d_lon += b.vbe_s()
                    lat += d_lat
                    lon += d_lon
                else:
                    lat += b.vbe_s()
                    lon += b.vbe_s()
                nodes.append((lat, lon))
            rings.append(nodes)
        blocks.append(rings)
    assert b.pos == wend, "way не сошёлся: %d != %d" % (b.pos, wend)
    return {"bitmap": subtile_bitmap, "layer": layer, "tags": tags, "name": name,
            "ref": ref, "label": label_pos, "blocks": blocks, "dd": double_delta}


# ---------------------------------------------------------------------------
# Рендер в PNG (без зависимостей)
# ---------------------------------------------------------------------------

def write_png(path, w, h, rgb):
    raw = b"".join(b"\x00" + bytes(rgb[y * w * 3:(y + 1) * w * 3]) for y in range(h))

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    open(path, "wb").write(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b""))


def draw_line(rgb, w, h, x0, y0, x1, y1, color, thick=1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        for ox in range(-(thick // 2), thick // 2 + 1):
            for oy in range(-(thick // 2), thick // 2 + 1):
                x, y = x0 + ox, y0 + oy
                if 0 <= x < w and 0 <= y < h:
                    rgb[(y * w + x) * 3:(y * w + x) * 3 + 3] = color
        if x0 == x1 and y0 == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def way_style(key):
    return {
        "highway=motorway": ((200, 30, 30), 3),
        "highway=trunk": ((220, 60, 40), 3),
        "highway=primary": ((230, 120, 40), 2),
        "highway=secondary": ((240, 180, 60), 2),
        "highway=tertiary": ((250, 220, 120), 1),
        "highway=residential": ((255, 255, 255), 1),
        "highway=unclassified": ((255, 255, 255), 1),
        "highway=service": ((200, 200, 200), 1),
        "highway=footway": ((190, 150, 110), 1),
        "highway=path": ((190, 150, 110), 1),
        "highway=pedestrian": ((210, 190, 170), 1),
        "railway=rail": ((80, 80, 80), 1),
        "waterway=river": ((60, 120, 220), 2),
        "waterway=stream": ((90, 150, 230), 1),
        "natural=coastline": ((20, 60, 180), 2),
        "building=yes": ((190, 150, 130), 1),
    }.get(key, ((100, 170, 100), 1))


def render_tile(mf, tile, out_path):
    zi = tile["zi"]
    tx, ty = tile["tile_x"], tile["tile_y"]
    top_lat = MapFile.tile_y_to_lat(ty, zi.base)
    left_lon = MapFile.tile_x_to_lon(tx, zi.base)
    lat_span = top_lat - MapFile.tile_y_to_lat(ty + 1, zi.base)
    lon_span = MapFile.tile_x_to_lon(tx + 1, zi.base) - left_lon

    W = H = 256
    rgb = bytearray([245, 245, 238] * (W * H))

    def proj(lat_diff, lon_diff):
        # lat_diff = точка - северный край (к югу отрицателен) → y растёт вниз
        x = int((lon_diff / 1e6) / lon_span * W)
        y = int(-(lat_diff / 1e6) / lat_span * H)
        return x, y

    for way in tile["ways"]:
        key = mf.way_tags[way["tags"][0]] if way["tags"] else "?"
        color, thick = way_style(key)
        for rings in way["blocks"]:
            for ring in rings:
                pts = [proj(n[0], n[1]) for n in ring]
                for i in range(len(pts) - 1):
                    draw_line(rgb, W, H, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], color, thick)
    for poi in tile["pois"]:
        x, y = proj(poi["lat_diff"], poi["lon_diff"])
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if 0 <= x + dx < W and 0 <= y + dy < H:
                    i3 = ((y + dy) * W + x + dx) * 3
                    rgb[i3:i3 + 3] = b"\xa0\x1e\xa0"
    write_png(out_path, W, H, rgb)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    t0 = time.time()
    mf = MapFile(MAP_FILE)
    print("Заголовок: v%d, bbox lat %.4f..%.4f lon %.4f..%.4f" %
          (mf.version, mf.min_lat, mf.max_lat, mf.min_lon, mf.max_lon))
    print("POI-тегов %d, way-тегов %d, парсинг заголовка %.1f мс" %
          (len(mf.pois_tags), len(mf.way_tags), (time.time() - t0) * 1000))
    for zi in mf.intervals:
        print("  %r" % zi)

    zi = mf.intervals[2]
    lat, lon = 59.935, 30.32
    tx = int(math.floor(MapFile.lon_to_tile_x(lon, zi.base)))
    ty = int(math.floor(MapFile.lat_to_tile_y(lat, zi.base)))
    print("\n=== Тайл z%d x%d y%d ===" % (zi.base, tx, ty))
    t1 = time.time()
    tile = decode_tile(mf, zi, tx, ty)
    print("декод: %.1f мс" % ((time.time() - t1) * 1000))
    print("zoom table (по уровням z%d..z%d): %s" % (zi.min, zi.max, tile["zoom_table"]))
    print("POI: %d, ways: %d" % (len(tile["pois"]), len(tile["ways"])))

    wc = Counter()
    for way in tile["ways"]:
        for t in way["tags"]:
            wc[mf.way_tags[t]] += 1
    print("топ way-тегов:", wc.most_common(12))
    named = [(w["name"], mf.way_tags[w["tags"][0]]) for w in tile["ways"] if w["name"]][:12]
    print("имена ways (первые 12):")
    for nm, k in named:
        print("   %-42s (%s)" % (nm, k))
    nodes_total = sum(len(r) for w in tile["ways"] for rings in w["blocks"] for r in rings)
    print("узлов в ways: %d" % nodes_total)
    render_tile(mf, tile, "tools/mapsforge/tile_%d_%d_%d.png" % (zi.base, tx, ty))
    print("PNG: tools/mapsforge/tile_%d_%d_%d.png" % (zi.base, tx, ty))

    # --- полная валидация всех тайлов детального интервала ---
    print("\n=== Валидация всех %d тайлов z%d..%d ===" % (zi.tiles_x * zi.tiles_y, zi.min, zi.max))
    t2 = time.time()
    total_ways = total_pois = total_nodes = n_err = n_empty = 0
    max_block = 0
    for iy in range(zi.tiles_y):
        for ix in range(zi.tiles_x):
            water, off, block_end = tile_block_range(mf, zi, ix, iy)
            if block_end <= off:
                n_empty += 1
                continue
            try:
                t = decode_tile(mf, zi, zi.x0 + ix, zi.y0 + iy)
                total_pois += len(t["pois"])
                total_ways += len(t["ways"])
                total_nodes += sum(len(r) for w in t["ways"] for rings in w["blocks"] for r in rings)
            except Exception as e:
                n_err += 1
                if n_err <= 3:
                    print("  ОШИБКА (x%d y%d): %s" % (zi.x0 + ix, zi.y0 + iy, e))
    dt = (time.time() - t2) * 1000
    print("тайлов: %d, пустых: %d, ошибок: %d" % (zi.tiles_x * zi.tiles_y, n_empty, n_err))
    print("всего POI %d, ways %d, узлов %d" % (total_pois, total_ways, total_nodes))
    print("время полной валидации: %.0f мс (Python; на C будет ~в 50-100 раз быстрее)" % dt)


if __name__ == "__main__":
    main()
