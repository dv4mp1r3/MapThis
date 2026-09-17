/*
 * mapsforge.c — читатель векторных карт Mapsforge .map (file version 3)
 * и программный рендер тайлов для MapThis! (PSP).
 *
 * Полностью переносимый C (без PSPSDK): собирается и для PSP (в составе
 * приложения), и на host (тест tools/mapsforge). Спецификация формата и
 * подводные камни — todo_mapsforge_c.md; эталон — tools/mapsforge/mf_proto.py.
 *
 * Ключевые правила формата (проверены по реальному файлу):
 *  - все целые big-endian, координаты — микроградусы (знаковые);
 *  - tile index: 5 байт/запись, старший бит = water, 39 бит = offset
 *    блока ОТ НАЧАЛА SUBFILE; порядок записей y-внешний, x-внутренний;
 *  - границы блока: block(i) = [off(i), off(i+1)); у последней записи
 *    конец = конец subfile; пустой тайл: off(i) == off(i+1);
 *  - слой в special-байте пишется КАК ЕСТЬ (0..10), без смещения ±5;
 *  - счётчик way-блоков пишется ТОЛЬКО при флаге 0x08; счётчик колец — ВСЕГДА;
 *  - label position — дельта к первому узлу way, не к углу тайла;
 *  - объекты в POI-/way-секциях сгруппированы по возрастанию зума
 *    (zoom-фильтрация = прочитать кумулятивный лимит и остановиться).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <dirent.h>
#include "mapsforge.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* флаги way */
#define W_NAME        0x80
#define W_HOUSENUMBER 0x40
#define W_REF         0x20
#define W_LABEL       0x10
#define W_MULTI       0x08
#define W_DOUBLE_DELTA 0x04
/* флаги POI */
#define P_NAME        0x80
#define P_HOUSENUMBER 0x40
#define P_ELEVATION   0x20

/* флаги заголовка */
#define H_DEBUG       0x80
#define H_START_POS   0x40
#define H_START_ZOOM  0x20
#define H_LANGUAGES   0x10
#define H_COMMENT     0x08
#define H_CREATED_BY  0x04

#define MF_TAG_MAXLEN   96     /* с запасом на самый длинный тег "key=value" */
#define MF_POI_DRAW_MAX 4096   /* максимум собранных для отрисовки POI на тайл */
#define MF_LABEL_MAX    24     /* максимум меток имён на тайл */
#define MF_LABEL_TEXT   49     /* обрезка текста метки (байт: 24 симв. UTF-8 + NUL) */
#define MF_HDR_MAX      (8*1024*1024)  /* разумный потолок размера заголовка */

/* ------------------------------------------------------------------ */
/* состояние карты                                                     */
/* ------------------------------------------------------------------ */

typedef struct {
	uint8_t  base, min, max;
	uint64_t offset, size;          /* subfile: абсолютный offset и размер */
	/* вычисляемое: */
	int      x0, y0;                /* СЗ-угол сетки (slippy-тайл на base) */
	int      tiles_x, tiles_y;
	unsigned char *index;           /* кэш индекса: 5 байт/запись */
} MF_INTERVAL;

static FILE *mf_fp = NULL;
static int   mf_opened = 0;
static char  mf_err[256] = "";

static int32_t mf_min_lat, mf_min_lon, mf_max_lat, mf_max_lon; /* мкград */
static int     mf_tile_size;
static int     mf_version;

static int   mf_poi_tag_n = 0, mf_way_tag_n = 0;
static char *mf_tag_blob = NULL;               /* общий блоб строк тегов */
static char *mf_poi_tags[MF_MAX_TAGS];
static char *mf_way_tags[MF_MAX_TAGS];
static uint16_t mf_way_tag_style[MF_MAX_TAGS]; /* idx+1 в mf_styles, 0 = нет стиля */

static MF_INTERVAL mf_iv[MF_MAX_INTERVALS];
static int   mf_iv_n = 0;
static int   mf_maxbase = 0;      /* base самого детального интервала */

/* буфер одного блока тайла: malloc при mf_open (обзорные интервалы дают
 * блоки в сотни КБ — статический 256КБ BSS не хватало, центр города на
 * обзорных зумах оставался фоном) */
static unsigned char *mf_blk = NULL;

/* ------------------------------------------------------------------ */
/* стили отрисовки (перенос way_style из mf_proto.py)                  */
/* ------------------------------------------------------------------ */

typedef struct {
	const char *tag;          /* "key=value" */
	uint8_t r, g, b;
	uint8_t thick;
	uint8_t back;             /* 1 = фоновый слой (здания/вода) — первый проход */
} MF_STYLE;

static const MF_STYLE mf_styles[] = {
	{"highway=motorway",    200,  30,  30, 3, 0},
	{"highway=trunk",       220,  60,  40, 3, 0},
	{"highway=primary",     230, 120,  40, 2, 0},
	{"highway=secondary",   240, 180,  60, 2, 0},
	{"highway=tertiary",    250, 220, 120, 1, 0},
	{"highway=residential", 255, 255, 255, 1, 0},
	{"highway=unclassified",255, 255, 255, 1, 0},
	{"highway=service",     200, 200, 200, 1, 0},
	{"highway=footway",     190, 150, 110, 1, 0},
	{"highway=path",        190, 150, 110, 1, 0},
	{"highway=pedestrian",  210, 190, 170, 1, 0},
	{"railway=rail",         80,  80,  80, 1, 0},
	{"waterway=river",       60, 120, 220, 2, 1},
	{"waterway=stream",      90, 150, 230, 1, 1},
	{"natural=coastline",    20,  60, 180, 2, 1},
	{"building=yes",        190, 150, 130, 1, 1},
};
#define MF_STYLE_N      (int)(sizeof(mf_styles)/sizeof(mf_styles[0]))
#define MF_DEFAULT_STYLE (MF_STYLE_N)  /* неизвестный тег — как в прототипе */

/* палитра (день); ночью инвертируется */
#define MF_BG_R 245
#define MF_BG_G 245
#define MF_BG_B 238
#define MF_WATER_R 120
#define MF_WATER_G 170
#define MF_WATER_B 220
#define MF_POI_R 160
#define MF_POI_G  30
#define MF_POI_B 160

static uint32_t mf_color(int r, int g, int b, int night)
{
	if (night) {
		r = 255 - r; g = 255 - g; b = 255 - b;
	}
	/* нативный формат PSP GU 8888: A<<24 | B<<16 | G<<8 | R */
	return 0xFF000000u | ((uint32_t)b << 16) | ((uint32_t)g << 8) | (uint32_t)r;
}

/* ------------------------------------------------------------------ */
/* чтение буфера: big-endian + VBE                                     */
/* ------------------------------------------------------------------ */

typedef struct {
	const unsigned char *p;
	const unsigned char *end;
} MF_BUF;

static int mf_buf_u8(MF_BUF *b, uint32_t *v)
{
	if (b->end - b->p < 1) return 0;
	*v = *b->p++;
	return 1;
}
static int mf_buf_u16(MF_BUF *b, uint32_t *v)
{
	if (b->end - b->p < 2) return 0;
	*v = ((uint32_t)b->p[0] << 8) | b->p[1];
	b->p += 2;
	return 1;
}
static int mf_buf_u32(MF_BUF *b, uint32_t *v)
{
	if (b->end - b->p < 4) return 0;
	*v = ((uint32_t)b->p[0] << 24) | ((uint32_t)b->p[1] << 16) |
	     ((uint32_t)b->p[2] << 8) | b->p[3];
	b->p += 4;
	return 1;
}
static int mf_buf_u64(MF_BUF *b, uint64_t *v)
{
	uint32_t hi, lo;
	if (!mf_buf_u32(b, &hi) || !mf_buf_u32(b, &lo)) return 0;
	*v = ((uint64_t)hi << 32) | lo;
	return 1;
}

/* VBE-U: 7 бит на байт, младшие группы вперёд, 0x80 = продолжение */
static uint32_t mf_vbe_u(MF_BUF *b, int *err)
{
	uint32_t r = 0;
	int sh = 0;
	if (*err) return 0;
	for (;;) {
		if (b->p >= b->end) { *err = 1; return 0; }
		{
			uint8_t c = *b->p++;
			if (sh < 32)
				r |= (uint32_t)(c & 0x7F) << sh;
			if (!(c & 0x80))
				return r;
			sh += 7;
			if (sh > 35) { *err = 1; return 0; }
		}
	}
}

/* VBE-S: последний байт = 6 бит данных + знаковый бит 0x40.
 * Знак применяется к модулю (НЕ дополнение до двух). */
static int32_t mf_vbe_s(MF_BUF *b, int *err)
{
	uint32_t r = 0;
	int sh = 0;
	if (*err) return 0;
	for (;;) {
		if (b->p >= b->end) { *err = 1; return 0; }
		{
			uint8_t c = *b->p++;
			if (c & 0x80) {
				if (sh < 32)
					r |= (uint32_t)(c & 0x7F) << sh;
				sh += 7;
			} else {
				int32_t v;
				if (sh < 32)
					r |= (uint32_t)(c & 0x3F) << sh;
				v = (int32_t)r;
				return (c & 0x40) ? -v : v;
			}
			if (sh > 35) { *err = 1; return 0; }
		}
	}
}

/* string: vbe_u длина + UTF-8 байты; указатель внутрь буфера */
static const unsigned char *mf_string(MF_BUF *b, int *len, int *err)
{
	uint32_t n = mf_vbe_u(b, err);
	if (*err || (uint32_t)(b->end - b->p) < n) {
		*err = 1;
		if (len) *len = 0;
		return NULL;
	}
	{
		const unsigned char *s = b->p;
		b->p += n;
		if (len) *len = (int)n;
		return s;
	}
}

/* big-endian из FILE (для magic/header_size) */
static int mf_rbytes(FILE *f, void *dst, int n)
{
	return (int)fread(dst, 1, n, f) == n;
}
static int mf_r_u32f(FILE *f, uint32_t *v)
{
	unsigned char c[4];
	if (!mf_rbytes(f, c, 4)) return 0;
	*v = ((uint32_t)c[0] << 24) | ((uint32_t)c[1] << 16) | ((uint32_t)c[2] << 8) | c[3];
	return 1;
}

/* ------------------------------------------------------------------ */
/* Web Mercator (slippy); используется только при открытии карты и      */
/* настройке проекции — двойной точности немного, не жалко              */
/* ------------------------------------------------------------------ */

static double mf_lon_to_tile_x(double lon, int z)
{
	return (lon + 180.0) / 360.0 * (double)(1u << z);
}
static double mf_lat_to_tile_y(double lat, int z)
{
	double r;
	if (lat > 85.05112878) lat = 85.05112878;
	if (lat < -85.05112878) lat = -85.05112878;
	r = log(tan(M_PI / 4 + lat * (M_PI / 180.0) / 2));
	return (1.0 - r / M_PI) / 2.0 * (double)(1u << z);
}
static double mf_tile_y_to_lat(double y, int z)
{
	double n = M_PI * (1.0 - 2.0 * y / (double)(1u << z));
	return atan(sinh(n)) * (180.0 / M_PI);
}

/* ------------------------------------------------------------------ */
/* заголовок                                                            */
/* ------------------------------------------------------------------ */

static void mf_seterr(const char *msg)
{
	snprintf(mf_err, sizeof(mf_err), "%s", msg);
}

void mf_close(void)
{
	int i;
	if (mf_fp != NULL) {
		fclose(mf_fp);
		mf_fp = NULL;
	}
	for (i = 0; i < mf_iv_n; i++) {
		if (mf_iv[i].index != NULL)
			free(mf_iv[i].index);
		mf_iv[i].index = NULL;
	}
	mf_iv_n = 0;
	mf_poi_tag_n = mf_way_tag_n = 0;
	if (mf_tag_blob != NULL) {
		free(mf_tag_blob);
		mf_tag_blob = NULL;
	}
	if (mf_blk != NULL) {
		free(mf_blk);
		mf_blk = NULL;
	}
	mf_opened = 0;
	mf_err[0] = 0;
}

static int mf_has_ext_map(const char *name)
{
	size_t l = strlen(name);
	if (l < 5) return 0;
	return (name[l - 4] == '.' || name[l - 4] == '.') &&
	       (name[l - 3] == 'm' || name[l - 3] == 'M') &&
	       (name[l - 2] == 'a' || name[l - 2] == 'A') &&
	       (name[l - 1] == 'p' || name[l - 1] == 'P');
}

/* найти .map: сам путь или первый *.map в каталоге */
static int mf_resolve_path(const char *path, char *out, int outsz)
{
	DIR *dp;
	struct dirent *de;

	if (mf_has_ext_map(path)) {
		snprintf(out, outsz, "%s", path);
		return 1;
	}
	dp = opendir(path);
	if (dp == NULL)
		return 0;
	while ((de = readdir(dp)) != NULL) {
		if (mf_has_ext_map(de->d_name)) {
			snprintf(out, outsz, "%s/%s", path, de->d_name);
			closedir(dp);
			return 1;
		}
	}
	closedir(dp);
	return 0;
}

/* таблица тегов: u16 count + count строк (vbe-длина); строки — в блоб */
static int mf_read_tags(MF_BUF *h, char **tab, int *count, int *blob_used, int blob_cap)
{
	uint32_t n, i;
	int err = 0;
	if (!mf_buf_u16(h, &n) || n > MF_MAX_TAGS) return 0;
	for (i = 0; i < n; i++) {
		int len = 0;
		const unsigned char *s = mf_string(h, &len, &err);
		if (err || len <= 0 || len > MF_TAG_MAXLEN) return 0;
		if (*blob_used + len + 1 > blob_cap) return 0;
		memcpy(mf_tag_blob + *blob_used, s, len);
		mf_tag_blob[*blob_used + len] = 0;
		tab[i] = mf_tag_blob + *blob_used;
		*blob_used += len + 1;
	}
	*count = (int)n;
	return 1;
}

int mf_open(const char *path)
{
	char full[512];
	char magic[20];
	uint32_t header_size, v, flags;
	unsigned char *hdr = NULL;
	MF_BUF h;
	uint64_t u64;
	int i, blob_used = 0, blob_cap;

	if (mf_opened)
		mf_close();

	if (!mf_resolve_path(path, full, sizeof(full))) {
		mf_seterr("no .map file found");
		return 0;
	}
	mf_fp = fopen(full, "rb");
	if (mf_fp == NULL) {
		mf_seterr("cannot open map file");
		return 0;
	}
	if (!mf_rbytes(mf_fp, magic, 20) || memcmp(magic, "mapsforge binary OSM", 20) != 0) {
		mf_seterr("bad magic");
		goto fail;
	}
	if (!mf_r_u32f(mf_fp, &header_size) || header_size < 32 || header_size > MF_HDR_MAX) {
		mf_seterr("bad header size");
		goto fail;
	}
	hdr = malloc(header_size);
	if (hdr == NULL || fread(hdr, 1, header_size, mf_fp) != header_size) {
		mf_seterr("truncated header");
		goto fail;
	}
	h.p = hdr;
	h.end = hdr + header_size;

	if (!mf_buf_u32(&h, &v)) { mf_seterr("truncated header"); goto fail; }
	mf_version = (int)v;
	if (v != 3) {
		mf_seterr("unsupported file version");
		goto fail;
	}
	if (!mf_buf_u64(&h, &u64)) { mf_seterr("truncated header"); goto fail; }  /* file size */
	if (!mf_buf_u64(&h, &u64)) { mf_seterr("truncated header"); goto fail; }  /* creation date */
	if (!mf_buf_u32(&h, (uint32_t *)&mf_min_lat) ||
	    !mf_buf_u32(&h, (uint32_t *)&mf_min_lon) ||
	    !mf_buf_u32(&h, (uint32_t *)&mf_max_lat) ||
	    !mf_buf_u32(&h, (uint32_t *)&mf_max_lon)) {
		mf_seterr("truncated header");
		goto fail;
	}
	if (!mf_buf_u16(&h, &v)) { mf_seterr("truncated header"); goto fail; }
	mf_tile_size = (int)v;
	{
		int err = 0, len = 0;
		if (mf_string(&h, &len, &err) == NULL || err) {   /* projection */
			mf_seterr("truncated header");
			goto fail;
		}
	}
	if (!mf_buf_u8(&h, &flags)) { mf_seterr("truncated header"); goto fail; }
	if (flags & H_DEBUG) {
		/* с debug-строками формат объектов другой — не поддерживаем */
		mf_seterr("debug maps are not supported");
		goto fail;
	}
	if (flags & H_START_POS) {
		uint32_t a;
		if (!mf_buf_u32(&h, &a) || !mf_buf_u32(&h, &a)) { mf_seterr("truncated header"); goto fail; }
	}
	if (flags & H_START_ZOOM) {
		uint32_t a;
		if (!mf_buf_u8(&h, &a)) { mf_seterr("truncated header"); goto fail; }
	}
	{
		uint32_t f = flags;
		for (i = 0; i < 3; i++) {         /* languages, comment, created by */
			int err = 0, len = 0;
			if (!(f & (H_LANGUAGES >> i))) continue;
			if (mf_string(&h, &len, &err) == NULL || err) {
				mf_seterr("truncated header");
				goto fail;
			}
		}
	}

	blob_cap = MF_MAX_TAGS * (MF_TAG_MAXLEN + 1) * 2;
	mf_tag_blob = malloc(blob_cap);
	if (mf_tag_blob == NULL) { mf_seterr("out of memory"); goto fail; }
	if (!mf_read_tags(&h, mf_poi_tags, &mf_poi_tag_n, &blob_used, blob_cap) ||
	    !mf_read_tags(&h, mf_way_tags, &mf_way_tag_n, &blob_used, blob_cap)) {
		mf_seterr("bad tag table");
		goto fail;
	}

	/* карта стилей по way-тегам */
	for (i = 0; i < mf_way_tag_n; i++) {
		int s;
		mf_way_tag_style[i] = 0;
		for (s = 0; s < MF_STYLE_N; s++) {
			if (strcmp(mf_way_tags[i], mf_styles[s].tag) == 0) {
				mf_way_tag_style[i] = (uint16_t)(s + 1);
				break;
			}
		}
	}

	if (!mf_buf_u8(&h, &v) || v == 0 || v > MF_MAX_INTERVALS) {
		mf_seterr("bad zoom interval count");
		goto fail;
	}
	mf_iv_n = (int)v;
	for (i = 0; i < mf_iv_n; i++) {
		uint32_t base, mn, mx;
		if (!mf_buf_u8(&h, &base) || !mf_buf_u8(&h, &mn) || !mf_buf_u8(&h, &mx) ||
		    !mf_buf_u64(&h, &mf_iv[i].offset) || !mf_buf_u64(&h, &mf_iv[i].size)) {
			mf_seterr("truncated header");
			goto fail;
		}
		mf_iv[i].base = (uint8_t)base;
		mf_iv[i].min = (uint8_t)mn;
		mf_iv[i].max = (uint8_t)mx;
		mf_iv[i].index = NULL;
	}
	free(hdr);
	hdr = NULL;

	/* сетка интервалов из bbox + кэш индексов в RAM */
	mf_maxbase = 0;
	for (i = 0; i < mf_iv_n; i++) {
		MF_INTERVAL *zi = &mf_iv[i];
		double minlat = mf_min_lat / 1e6, minlon = mf_min_lon / 1e6;
		double maxlat = mf_max_lat / 1e6, maxlon = mf_max_lon / 1e6;
		int bx1 = (int)floor(mf_lon_to_tile_x(maxlon, zi->base));
		int by1 = (int)floor(mf_lat_to_tile_y(minlat, zi->base));
		zi->x0 = (int)floor(mf_lon_to_tile_x(minlon, zi->base));
		zi->y0 = (int)floor(mf_lat_to_tile_y(maxlat, zi->base));
		zi->tiles_x = bx1 - zi->x0 + 1;
		zi->tiles_y = by1 - zi->y0 + 1;
		if (zi->tiles_x <= 0 || zi->tiles_y <= 0 ||
		    (double)zi->tiles_x * (double)zi->tiles_y > 4e7) {
			mf_seterr("bad interval grid");
			goto fail;
		}
		if (zi->base > mf_maxbase)
			mf_maxbase = zi->base;
	}
	for (i = 0; i < mf_iv_n; i++) {
		MF_INTERVAL *zi = &mf_iv[i];
		size_t isz = (size_t)zi->tiles_x * zi->tiles_y * 5;
		zi->index = malloc(isz);
		if (zi->index == NULL) { mf_seterr("out of memory"); goto fail; }
		if (fseek(mf_fp, (long)zi->offset, SEEK_SET) != 0 ||
		    fread(zi->index, 1, isz, mf_fp) != isz) {
			mf_seterr("index read failed");
			goto fail;
		}
	}

	mf_blk = malloc(MF_BLOCK_SIZE);       /* буфер блока на всё время работы */
	if (mf_blk == NULL) { mf_seterr("out of memory"); goto fail; }

	mf_opened = 1;
	return 1;

fail:
	if (hdr != NULL)
		free(hdr);
	mf_close();
	return 0;
}

int mf_is_open(void) { return mf_opened; }
const char *mf_strerror(void) { return mf_err[0] ? mf_err : "no error"; }

int mf_interval_count(void) { return mf_iv_n; }
int mf_interval_base(int i) { return (i >= 0 && i < mf_iv_n) ? mf_iv[i].base : -1; }
int mf_interval_min(int i) { return (i >= 0 && i < mf_iv_n) ? mf_iv[i].min : -1; }
int mf_interval_max(int i) { return (i >= 0 && i < mf_iv_n) ? mf_iv[i].max : -1; }
void mf_interval_grid(int i, int *x0, int *y0, int *tiles_x, int *tiles_y)
{
	if (i >= 0 && i < mf_iv_n) {
		*x0 = mf_iv[i].x0; *y0 = mf_iv[i].y0;
		*tiles_x = mf_iv[i].tiles_x; *tiles_y = mf_iv[i].tiles_y;
	}
}
int mf_poi_tag_count(void) { return mf_poi_tag_n; }
int mf_way_tag_count(void) { return mf_way_tag_n; }
const char *mf_poi_tag(int i) { return (i >= 0 && i < mf_poi_tag_n) ? mf_poi_tags[i] : NULL; }
const char *mf_way_tag(int i) { return (i >= 0 && i < mf_way_tag_n) ? mf_way_tags[i] : NULL; }

int mf_max_base_zoom(void) { return mf_maxbase; }

/* Аналог TILE_NUM MapThis: размер КАРТЫ в тайлах на детальном зуме,
 * округлённый вверх до степени двойки (мир MapThis = карта; MapThis
 * центрируется в TILE_NUM/2). Минимум 2 — иначе get_zoom даст 0 и
 * powerOf(zoom-1) уедет в UB. */
int mf_tile_num(void)
{
	int i, tx = 0, ty = 0, m = 2;
	for (i = 0; i < mf_iv_n; i++) {
		if (mf_iv[i].base == mf_maxbase) {
			if (mf_iv[i].tiles_x > tx) tx = mf_iv[i].tiles_x;
			if (mf_iv[i].tiles_y > ty) ty = mf_iv[i].tiles_y;
		}
	}
	while (m < tx || m < ty)
		m <<= 1;
	return m;
}
void mf_base_tile(int *tx, int *ty)
{
	int i;
	*tx = *ty = 0;
	for (i = 0; i < mf_iv_n; i++) {
		if (mf_iv[i].base == mf_maxbase) {
			*tx = mf_iv[i].x0;
			*ty = mf_iv[i].y0;
			return;
		}
	}
}

/* ------------------------------------------------------------------ */
/* tile index                                                           */
/* ------------------------------------------------------------------ */

/* запись индекса: water-бит + 39-битный offset от начала subfile */
static void mf_entry(const MF_INTERVAL *zi, int idx, int *water, uint64_t *off)
{
	const unsigned char *p = zi->index + (size_t)idx * 5;
	*water = p[0] & 0x80;
	*off = ((uint64_t)(p[0] & 0x7F) << 32) |
	       ((uint64_t)p[1] << 24) | ((uint64_t)p[2] << 16) |
	       ((uint64_t)p[3] << 8) | (uint64_t)p[4];
}

/* КЛЮЧЕВОЕ ПРАВИЛО: блоки идут строго последовательно в порядке индекса,
 * поэтому block(i) = [off(i), off(i+1)); последний — до конца subfile. */
static uint64_t mf_block_end(const MF_INTERVAL *zi, int ix, int iy)
{
	int k = iy * zi->tiles_x + ix + 1;
	int water;
	uint64_t off;
	if (k >= zi->tiles_x * zi->tiles_y)
		return zi->size;
	mf_entry(zi, k, &water, &off);
	return off;
}

/* ------------------------------------------------------------------ */
/* декодирование блока (итератор с callback'ами + зум-фильтр)           */
/* ------------------------------------------------------------------ */

int mf_next_node(MF_NODE_ITER *it, int32_t *lat, int32_t *lon)
{
	if (!it->has_cur || it->err)
		return 0;
	*lat = it->lat;
	*lon = it->lon;
	it->has_cur = 0;
	if (it->remaining > 0) {
		int err = 0;
		int32_t dla, dlo;
		MF_BUF b;
		b.p = it->p;
		b.end = it->end;
		dla = mf_vbe_s(&b, &err);
		dlo = mf_vbe_s(&b, &err);
		if (err) {
			it->err = 1;
		} else {
			if (it->double_delta) {
				it->dlat += dla;
				it->dlon += dlo;
				it->lat += it->dlat;
				it->lon += it->dlon;
			} else {
				it->lat += dla;
				it->lon += dlo;
			}
			it->p = b.p;
			it->remaining--;
			it->has_cur = 1;
		}
	}
	return 1;
}

static int mf_decode_poi(MF_BUF *b, MF_POI *out, int *err)
{
	int32_t lat, lon;
	uint8_t special, flags;
	int ntags, i;

	lat = mf_vbe_s(b, err);
	lon = mf_vbe_s(b, err);
	if (*err || b->p >= b->end) { *err = 1; return 0; }
	special = *b->p++;
	out->layer = special >> 4;
	ntags = special & 0x0F;
	out->ntags = ntags;
	for (i = 0; i < ntags; i++)
		out->tags[i] = (uint16_t)mf_vbe_u(b, err);
	if (*err || b->p >= b->end) { *err = 1; return 0; }
	flags = *b->p++;
	out->name = NULL; out->name_len = 0;
	out->housenumber = NULL; out->hn_len = 0;
	out->has_elevation = 0;
	if (flags & P_NAME)
		out->name = (const char *)mf_string(b, &out->name_len, err);
	if (!*err && (flags & P_HOUSENUMBER))
		out->housenumber = (const char *)mf_string(b, &out->hn_len, err);
	if (!*err && (flags & P_ELEVATION)) {
		out->elevation = mf_vbe_s(b, err);
		out->has_elevation = 1;
	}
	if (*err) return 0;
	out->lat = lat;
	out->lon = lon;
	return 1;
}

static int mf_decode_way(MF_BUF *b, MF_WAY *out, MF_WAY_CB cb, void *user, int *err)
{
	uint32_t data_size;
	const unsigned char *wend;
	uint8_t special, flags;
	int ntags, i, nblocks;

	data_size = mf_vbe_u(b, err);
	if (*err || (uint32_t)(b->end - b->p) < data_size) { *err = 1; return 0; }
	wend = b->p + data_size;

	if (b->end - b->p < 3) { *err = 1; return 0; }
	out->subtile_bitmap = (uint16_t)((b->p[0] << 8) | b->p[1]);
	b->p += 2;
	special = *b->p++;
	out->layer = special >> 4;
	ntags = special & 0x0F;
	out->ntags = ntags;
	for (i = 0; i < ntags; i++)
		out->tags[i] = (uint16_t)mf_vbe_u(b, err);
	if (*err || b->p >= b->end) { *err = 1; return 0; }
	flags = *b->p++;
	out->name = NULL; out->name_len = 0;
	out->housenumber = NULL; out->hn_len = 0;
	out->ref = NULL; out->ref_len = 0;
	out->has_label = 0;
	out->double_delta = (flags & W_DOUBLE_DELTA) != 0;
	if (flags & W_NAME)
		out->name = (const char *)mf_string(b, &out->name_len, err);
	if (!*err && (flags & W_HOUSENUMBER))
		out->housenumber = (const char *)mf_string(b, &out->hn_len, err);
	if (!*err && (flags & W_REF))
		out->ref = (const char *)mf_string(b, &out->ref_len, err);
	if (!*err && (flags & W_LABEL)) {
		out->label_lat = mf_vbe_s(b, err);
		out->label_lon = mf_vbe_s(b, err);
		out->has_label = 1;
	}
	if (*err) return 0;

	/* счётчик блоков — ТОЛЬКО при флаге 0x08; счётчик колец — ВСЕГДА */
	nblocks = (flags & W_MULTI) ? (int)mf_vbe_u(b, err) : 1;
	if (*err || nblocks < 1 || nblocks > 100000) { *err = 1; return 0; }

	for (i = 0; i < nblocks; i++) {
		int nrings = (int)mf_vbe_u(b, err);
		int r;
		if (*err || nrings < 1 || nrings > 100000) { *err = 1; return 0; }
		for (r = 0; r < nrings; r++) {
			MF_NODE_ITER it;
			uint32_t nnodes;
			int32_t la, lo;
			nnodes = mf_vbe_u(b, err);
			/* каждый узел после первого — минимум 1 байт (грубая отсечка) */
			if (*err || nnodes < 1 || nnodes - 1 > (uint32_t)(wend - b->p)) {
				*err = 1;
				return 0;
			}
			memset(&it, 0, sizeof(it));
			it.end = b->end;
			it.double_delta = out->double_delta;
			/* первый узел — абсолютная дельта к СЗ углу тайла */
			it.lat = mf_vbe_s(b, err);
			it.lon = mf_vbe_s(b, err);
			if (*err) return 0;
			it.p = b->p;               /* ПОСЛЕ первого узла */
			it.remaining = (int)nnodes - 1;
			it.has_cur = 1;
			out->block_index = i;
			out->ring_index = r;
			if (cb)
				cb(out, &it, user);
			/* пользователь мог не выбрать все узлы — дочитываем молча */
			while (mf_next_node(&it, &la, &lo)) {}
			if (it.err) { *err = 1; return 0; }
			b->p = it.p;
		}
	}
	if (b->p != wend) { *err = 1; return 0; }
	return 1;
}

/* Декод блока из памяти с зум-фильтром render_z.
 * Объекты упорядочены по возрастанию зума: берём ровно столько, сколько
 * задают кумулятивные суммы zoom-таблицы для уровней <= z; POI-секцию
 * допрыгиваем до границы, way-секцию просто не читаем до конца.
 * render_z == zmax включает строгие проверки границ (валидация). */
static int mf_decode_block(const unsigned char *buf, int len, int zmin, int zmax,
                           int render_z, MF_POI_CB poi_cb, MF_WAY_CB way_cb, void *user)
{
	MF_BUF b;
	int levels = zmax - zmin + 1;
	int i, zz, full;
	uint32_t poi_limit = 0, way_limit = 0;
	uint64_t first_way_offset;
	const unsigned char *poi_end;
	int err = 0;

	if (len <= 0 || levels <= 0 || levels > 32)
		return MF_ERR_FORMAT;
	b.p = buf;
	b.end = buf + len;

	zz = render_z;
	if (zz > zmax) zz = zmax;
	if (zz < zmin) zz = zmin;
	full = (zz == zmax);

	for (i = 0; i < levels; i++) {
		uint32_t pc = mf_vbe_u(&b, &err);
		uint32_t wc = mf_vbe_u(&b, &err);
		if (err) return MF_ERR_FORMAT;
		if (zmin + i <= zz) {
			poi_limit += pc;
			way_limit += wc;
		}
	}
	if (poi_limit > 50000000u || way_limit > 50000000u)
		return MF_ERR_FORMAT;
	first_way_offset = mf_vbe_u(&b, &err);
	if (err || (uint64_t)(b.end - b.p) < first_way_offset)
		return MF_ERR_FORMAT;
	poi_end = b.p + first_way_offset;

	for (i = 0; i < (int)poi_limit; i++) {
		MF_POI poi;
		if (!mf_decode_poi(&b, &poi, &err) || err)
			return MF_ERR_FORMAT;
		if (poi_cb)
			poi_cb(&poi, user);
	}
	if (full && b.p != poi_end)
		return MF_ERR_FORMAT;
	b.p = poi_end;

	for (i = 0; i < (int)way_limit; i++) {
		MF_WAY way;
		if (!mf_decode_way(&b, &way, way_cb, user, &err) || err)
			return MF_ERR_FORMAT;
	}
	if (full && b.p != b.end)
		return MF_ERR_FORMAT;
	return MF_OK;
}

/* прочитать блок тайла в mf_blk; >0 = длина, 0 = пустой, <0 = ошибка */
static int mf_read_block(const MF_INTERVAL *zi, int ix, int iy, int *water)
{
	int w;
	uint64_t off, end;
	size_t len;

	*water = 0;
	if (ix < 0 || iy < 0 || ix >= zi->tiles_x || iy >= zi->tiles_y)
		return MF_ERR_FORMAT;
	mf_entry(zi, iy * zi->tiles_x + ix, &w, &off);
	end = mf_block_end(zi, ix, iy);
	*water = w;
	if (end <= off)
		return 0;                    /* пустой тайл (off == off_next) */
	if (end - off > MF_BLOCK_SIZE)
		return MF_ERR_FORMAT;
	if (fseek(mf_fp, (long)(zi->offset + off), SEEK_SET) != 0)
		return MF_ERR_IO;
	len = (size_t)(end - off);
	if (fread(mf_blk, 1, len, mf_fp) != len)
		return MF_ERR_IO;
	return (int)len;
}

int mf_block_zoom_table(int interval, int ix, int iy, uint32_t *pairs, int max_pairs)
{
	MF_INTERVAL *zi;
	int len, levels, i, water, err = 0;
	if (!mf_opened || interval < 0 || interval >= mf_iv_n)
		return MF_ERR_NO_MAP;
	zi = &mf_iv[interval];
	len = mf_read_block(zi, ix, iy, &water);
	if (len <= 0)
		return len;
	levels = zi->max - zi->min + 1;
	if (levels > max_pairs)
		levels = max_pairs;
	{
		MF_BUF b;
		b.p = mf_blk;
		b.end = mf_blk + len;
		for (i = 0; i < levels; i++) {
			pairs[2 * i] = mf_vbe_u(&b, &err);
			pairs[2 * i + 1] = mf_vbe_u(&b, &err);
			if (err) return MF_ERR_FORMAT;
		}
	}
	return levels;
}

int mf_decode_tile_block(int interval, int ix, int iy, int render_z,
                         MF_POI_CB poi_cb, MF_WAY_CB way_cb, void *user)
{
	MF_INTERVAL *zi;
	int len, water, rc;
	if (!mf_opened || interval < 0 || interval >= mf_iv_n)
		return MF_ERR_NO_MAP;
	zi = &mf_iv[interval];
	len = mf_read_block(zi, ix, iy, &water);
	if (len == 0)
		return MF_EMPTY;
	if (len < 0)
		return len;
	rc = mf_decode_block(mf_blk, len, zi->min, zi->max, render_z, poi_cb, way_cb, user);
	return rc;
}

/* ------------------------------------------------------------------ */
/* рендер                                                               */
/* ------------------------------------------------------------------ */

typedef struct {
	uint32_t *pix;
	int    w, h, stride;
	int    night;
	int    pass;                 /* 0 = фоновые стили, 1 = остальные + POI */
	int    k;                    /* base - z: <0 апскейл, >0 даунскейл */
	int64_t ox, oy;              /* окно тайла в базовых пикселях (Q16) */
	int64_t XQ;                  /* px(Q16) за мкград lon (на base) */
	int64_t NYQ;                 /* -px(Q16) за мкград lat (строка блоков) */
	int    cur_bx, cur_by;       /* текущий блок */
	int    npoi;
	/* собранные метки имён (рисуются после всего) */
	int    nlab;
	int    zlab;                 /* метки разрешены (z >= 13) */
} MF_RC;

/* Буферы сбора POI/меток — в BSS, НЕ на стеке: рендер вызывается из
 * cachemngr (стек 8 КБ), а MF_POI_DRAW_MAX точек это ~16 КБ. Рендер
 * выполняется только из одного потока, так что shared-буферы безопасны. */
static int16_t mf_poi_px[MF_POI_DRAW_MAX];
static int16_t mf_poi_py[MF_POI_DRAW_MAX];
static char    mf_lab_text[MF_LABEL_MAX][MF_LABEL_TEXT];
static int16_t mf_lab_x[MF_LABEL_MAX];
static int16_t mf_lab_y[MF_LABEL_MAX];
static int16_t mf_lab_tw[MF_LABEL_MAX];   /* ширина текста для проверки коллизий */

/* точки для выбора середины именованного way (потоковый обход) */
#define MF_MID_MAX 512
static int16_t mf_mid_x[MF_MID_MAX];
static int16_t mf_mid_y[MF_MID_MAX];
static int32_t mf_mid_len[MF_MID_MAX];

/* ------------------------------------------------------------------ */
/* встроенный шрифт меток: 7px, латиница ASCII + кириллица             */
/* (сгенерирован из Arial Unicode: LANCZOS-даунскейл + порог 110;      */
/*  глиф = {width, 7 строк по биту на пиксель, MSB-вперёд})            */
/* ------------------------------------------------------------------ */

static const unsigned char mf_font_latin[95][8] = {
	{3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x80},
	{8, 0xe7, 0xe7, 0xe7, 0xe7, 0x66, 0x66, 0x66},
	{5, 0x08, 0x50, 0xf8, 0x50, 0xf8, 0x90, 0x80},
	{4, 0x60, 0xf0, 0xc0, 0x60, 0x10, 0xf0, 0x60},
	{7, 0xe0, 0xa8, 0xa0, 0x54, 0x0a, 0x2a, 0x0e},
	{6, 0x70, 0x50, 0x70, 0x60, 0x98, 0x98, 0x74},
	{3, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0x60},
	{2, 0x40, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80},
	{2, 0x80, 0x00, 0x40, 0x40, 0x40, 0x40, 0x00},
	{8, 0x18, 0x18, 0xfe, 0x7e, 0x38, 0x2c, 0x24},
	{8, 0x18, 0x18, 0x18, 0xff, 0x18, 0x18, 0x18},
	{5, 0xf0, 0xf0, 0xf0, 0xf0, 0x30, 0x30, 0x70},
	{8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
	{8, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe},
	{3, 0x20, 0x20, 0x40, 0x40, 0x40, 0x80, 0x80},
	{5, 0x70, 0xd8, 0x88, 0x88, 0x88, 0xd8, 0x70},
	{3, 0x20, 0xe0, 0xa0, 0x20, 0x20, 0x20, 0x20},
	{5, 0x70, 0xc8, 0x08, 0x10, 0x20, 0x40, 0xf8},
	{5, 0x70, 0x98, 0x10, 0x30, 0x08, 0x88, 0x70},
	{5, 0x10, 0x30, 0x70, 0x50, 0xf8, 0x38, 0x10},
	{5, 0x78, 0xc0, 0xf0, 0xd8, 0x08, 0x98, 0x70},
	{5, 0x70, 0xc8, 0xb0, 0xd8, 0x88, 0xc8, 0x70},
	{5, 0xf8, 0x10, 0x30, 0x20, 0x60, 0x40, 0x40},
	{5, 0x70, 0xd8, 0xd0, 0x70, 0x88, 0x88, 0x70},
	{5, 0x70, 0x98, 0x88, 0xd8, 0x68, 0x98, 0x70},
	{2, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0},
	{1, 0x80, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80},
	{7, 0x06, 0x1c, 0x70, 0xc0, 0x70, 0x1c, 0x06},
	{8, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff},
	{7, 0xc0, 0x70, 0x1c, 0x06, 0x1c, 0x70, 0xc0},
	{5, 0x70, 0x88, 0x08, 0x30, 0x20, 0x00, 0x20},
	{8, 0x3c, 0x42, 0xbd, 0xa5, 0xa9, 0xbe, 0x41},
	{7, 0x10, 0x38, 0x28, 0x68, 0x7c, 0xc4, 0x86},
	{6, 0xf8, 0x8c, 0x88, 0xf8, 0x84, 0x84, 0xf8},
	{6, 0x78, 0xcc, 0x80, 0x80, 0x84, 0xcc, 0x78},
	{6, 0xf8, 0x8c, 0x84, 0x84, 0x84, 0x8c, 0xf8},
	{6, 0xfc, 0x80, 0x80, 0xf8, 0x80, 0x80, 0xfc},
	{5, 0xf8, 0x80, 0x80, 0xf0, 0x80, 0x80, 0x80},
	{7, 0x3c, 0x46, 0x80, 0x8e, 0x82, 0x42, 0x3c},
	{6, 0x84, 0x84, 0x84, 0xfc, 0x84, 0x84, 0x84},
	{1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{4, 0x10, 0x10, 0x10, 0x10, 0x10, 0x90, 0xe0},
	{6, 0x88, 0x90, 0xa0, 0xe0, 0x90, 0x88, 0x8c},
	{5, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xf8},
	{7, 0xc6, 0xc6, 0xe6, 0xaa, 0xaa, 0x92, 0x92},
	{6, 0xc4, 0xc4, 0xa4, 0xb4, 0x94, 0x8c, 0x84},
	{7, 0x78, 0xc4, 0x82, 0x82, 0x82, 0xc4, 0x78},
	{6, 0xf8, 0x84, 0x84, 0xf8, 0x80, 0x80, 0x80},
	{6, 0x78, 0xcc, 0x84, 0x84, 0x84, 0x58, 0x3c},
	{7, 0xfc, 0x84, 0x84, 0xf8, 0x88, 0x8c, 0x86},
	{6, 0x78, 0xcc, 0xc0, 0x38, 0x04, 0xc4, 0x78},
	{6, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30},
	{6, 0x84, 0x84, 0x84, 0x84, 0x84, 0xcc, 0x78},
	{7, 0x82, 0x44, 0x44, 0x6c, 0x28, 0x38, 0x10},
	{8, 0x99, 0x99, 0x59, 0x42, 0x66, 0x66, 0x66},
	{7, 0x44, 0x6c, 0x38, 0x10, 0x38, 0x6c, 0xc6},
	{7, 0xc6, 0x44, 0x28, 0x38, 0x10, 0x10, 0x10},
	{6, 0x7c, 0x08, 0x10, 0x30, 0x60, 0x40, 0xfc},
	{2, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{3, 0x80, 0x80, 0x40, 0x40, 0x40, 0x20, 0x20},
	{2, 0xc0, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40},
	{8, 0x18, 0x18, 0x3c, 0x24, 0x66, 0x46, 0xc3},
	{3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{8, 0xf8, 0x7c, 0x3c, 0x3c, 0x1e, 0x0e, 0x0f},
	{6, 0x78, 0xcc, 0x0c, 0x7c, 0xcc, 0xcc, 0x7c},
	{5, 0x80, 0x80, 0xf8, 0x88, 0x88, 0x98, 0xf0},
	{6, 0x78, 0x4c, 0xc0, 0x80, 0xc4, 0x4c, 0x78},
	{5, 0x08, 0x08, 0x78, 0x88, 0x88, 0xc8, 0x78},
	{6, 0x78, 0x4c, 0xcc, 0xfc, 0xc0, 0x4c, 0x78},
	{3, 0x60, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x40},
	{5, 0x78, 0xd8, 0x88, 0x88, 0xd8, 0x78, 0x08},
	{4, 0x80, 0x80, 0xf0, 0x90, 0x90, 0x90, 0x90},
	{1, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80},
	{1, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80},
	{4, 0x80, 0x80, 0xa0, 0xc0, 0xc0, 0xa0, 0x90},
	{1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{8, 0xf7, 0xd9, 0x99, 0x99, 0x99, 0x99, 0x99},
	{6, 0xf8, 0xcc, 0x84, 0x84, 0x84, 0x84, 0x84},
	{7, 0x38, 0x4c, 0xc6, 0x86, 0xc6, 0x44, 0x38},
	{5, 0xf0, 0xd8, 0x88, 0x88, 0x98, 0xf0, 0x80},
	{5, 0x78, 0xd8, 0x88, 0x88, 0xd8, 0x78, 0x08},
	{4, 0xf0, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x80},
	{6, 0x78, 0xc8, 0xc0, 0x78, 0x0c, 0xcc, 0x78},
	{3, 0x40, 0x40, 0xe0, 0x40, 0x40, 0x40, 0x60},
	{6, 0x84, 0x84, 0x84, 0x84, 0x8c, 0xcc, 0x7c},
	{7, 0xc6, 0x44, 0x44, 0x6c, 0x28, 0x38, 0x10},
	{8, 0x99, 0xdb, 0x5a, 0x5a, 0x66, 0x66, 0x24},
	{7, 0x44, 0x6c, 0x38, 0x10, 0x38, 0x6c, 0xc6},
	{6, 0xc4, 0x48, 0x48, 0x78, 0x30, 0x30, 0x30},
	{6, 0xfc, 0x08, 0x18, 0x30, 0x60, 0x40, 0xfc},
	{3, 0x60, 0x40, 0x40, 0xc0, 0xc0, 0x40, 0x40},
	{1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{3, 0xc0, 0x40, 0x40, 0x60, 0x60, 0x40, 0x40},
	{8, 0x00, 0x70, 0xfb, 0xff, 0x9e, 0x0e, 0x00},
};

/* кириллица U+0400..U+0451 (А-Я, а-я, Ё, ё; пустые кодпоинты = пробел) */
static const unsigned char mf_font_cyr[82][8] = {
	{7, 0xfe, 0x82, 0x82, 0x82, 0x82, 0x82, 0xfe},
	{5, 0x50, 0xf8, 0x80, 0xf0, 0x80, 0x80, 0xf8},
	{8, 0xfc, 0x20, 0x3c, 0x33, 0x21, 0x21, 0x26},
	{4, 0x40, 0xf0, 0x80, 0x80, 0x80, 0x80, 0x80},
	{6, 0x78, 0xcc, 0x80, 0xe0, 0x84, 0xcc, 0x78},
	{6, 0x78, 0xcc, 0xc0, 0x38, 0x04, 0xc4, 0x78},
	{1, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{2, 0xc0, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80},
	{4, 0x10, 0x10, 0x10, 0x10, 0x10, 0x90, 0xe0},
	{8, 0x78, 0x48, 0x48, 0x4f, 0x49, 0x48, 0xcf},
	{8, 0x90, 0x90, 0x90, 0xfe, 0x91, 0x91, 0x9e},
	{8, 0xfc, 0x30, 0x34, 0x3b, 0x31, 0x31, 0x31},
	{4, 0x00, 0x90, 0xa0, 0xc0, 0xc0, 0xa0, 0x90},
	{7, 0xfe, 0x82, 0x82, 0x82, 0x82, 0x82, 0xfe},
	{5, 0x20, 0x88, 0xd8, 0x50, 0x20, 0x20, 0x40},
	{5, 0x88, 0x88, 0x88, 0x88, 0x88, 0xf8, 0x20},
	{7, 0x10, 0x38, 0x28, 0x68, 0x7c, 0xc4, 0x86},
	{6, 0xf8, 0x80, 0x80, 0xf8, 0x84, 0x84, 0xf8},
	{6, 0xf8, 0x8c, 0x88, 0xf8, 0x84, 0x84, 0xf8},
	{5, 0xf8, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{6, 0x78, 0x4c, 0x4c, 0x4c, 0x4c, 0xfc, 0x84},
	{6, 0xfc, 0x80, 0x80, 0xf8, 0x80, 0x80, 0xfc},
	{8, 0xd3, 0x52, 0x34, 0x3c, 0x76, 0x52, 0x9b},
	{5, 0x70, 0xc8, 0x18, 0x30, 0x08, 0xc8, 0x70},
	{6, 0x84, 0x8c, 0x94, 0xb4, 0xa4, 0xc4, 0xc4},
	{5, 0x30, 0x88, 0x98, 0xb8, 0xa8, 0xc8, 0x88},
	{5, 0x98, 0x90, 0xa0, 0xe0, 0xa0, 0x90, 0x98},
	{6, 0x7c, 0x44, 0x44, 0x44, 0x44, 0x44, 0xc4},
	{7, 0xc6, 0xc6, 0xe6, 0xaa, 0xaa, 0x92, 0x92},
	{6, 0x84, 0x84, 0x84, 0xfc, 0x84, 0x84, 0x84},
	{7, 0x78, 0xc4, 0x82, 0x82, 0x82, 0xc4, 0x78},
	{6, 0xfc, 0x84, 0x84, 0x84, 0x84, 0x84, 0x84},
	{6, 0xf8, 0x84, 0x84, 0xf8, 0x80, 0x80, 0x80},
	{6, 0x78, 0xcc, 0x80, 0x80, 0x84, 0xcc, 0x78},
	{6, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30},
	{6, 0x84, 0x48, 0x48, 0x30, 0x30, 0x20, 0x60},
	{7, 0x10, 0x7c, 0xd6, 0x92, 0xd6, 0x7c, 0x10},
	{7, 0x44, 0x6c, 0x38, 0x10, 0x38, 0x6c, 0xc6},
	{6, 0x8c, 0x8c, 0x8c, 0x8c, 0x8c, 0xfc, 0x04},
	{6, 0x84, 0x84, 0x84, 0xcc, 0x34, 0x04, 0x04},
	{8, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0xff},
	{8, 0x92, 0x92, 0x92, 0x92, 0x92, 0xff, 0x01},
	{8, 0xe0, 0x20, 0x20, 0x3e, 0x23, 0x21, 0x3e},
	{7, 0x82, 0x82, 0x82, 0xfa, 0x8a, 0x8a, 0xfa},
	{6, 0x80, 0x80, 0x80, 0xf8, 0x8c, 0x84, 0xf8},
	{6, 0x78, 0xcc, 0x04, 0x1c, 0x04, 0xcc, 0x78},
	{8, 0x9e, 0x93, 0xa1, 0xe1, 0xa1, 0x93, 0x9e},
	{6, 0x7c, 0x44, 0x44, 0x7c, 0x24, 0x44, 0xc4},
	{6, 0x78, 0xcc, 0x0c, 0x7c, 0xcc, 0xcc, 0x7c},
	{5, 0x78, 0x80, 0xf0, 0x88, 0x88, 0x98, 0x70},
	{6, 0xf8, 0x8c, 0xc8, 0xf8, 0x8c, 0x8c, 0xf8},
	{4, 0xf0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80},
	{7, 0x3c, 0x24, 0x64, 0x44, 0x44, 0xfe, 0x82},
	{6, 0x78, 0x4c, 0xcc, 0xfc, 0xc0, 0x4c, 0x78},
	{8, 0xdb, 0x5a, 0x3c, 0x3c, 0x7e, 0x5a, 0xdb},
	{5, 0x70, 0xd8, 0x18, 0x30, 0x18, 0xd8, 0x70},
	{6, 0x8c, 0x8c, 0x9c, 0xb4, 0xe4, 0xc4, 0xc4},
	{4, 0x60, 0x00, 0x90, 0xb0, 0xf0, 0xd0, 0x90},
	{5, 0x98, 0xb0, 0xa0, 0xe0, 0xa0, 0x90, 0x98},
	{7, 0x7e, 0x66, 0x62, 0x66, 0x66, 0x66, 0xc6},
	{8, 0xc3, 0xe7, 0xe7, 0xa5, 0xb9, 0x99, 0x99},
	{6, 0x8c, 0x84, 0xcc, 0xfc, 0x8c, 0x8c, 0x8c},
	{7, 0x38, 0x4c, 0xc6, 0x86, 0xc6, 0x44, 0x38},
	{6, 0xfc, 0xcc, 0x84, 0x84, 0x84, 0x84, 0x84},
	{5, 0xf0, 0xd8, 0x88, 0x88, 0x98, 0xf0, 0x80},
	{6, 0x78, 0x4c, 0xc0, 0x80, 0xc4, 0x4c, 0x78},
	{6, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30},
	{6, 0xc4, 0x48, 0x48, 0x78, 0x30, 0x30, 0x30},
	{7, 0x10, 0x7c, 0xde, 0x92, 0x92, 0x7e, 0x10},
	{7, 0x44, 0x6c, 0x38, 0x10, 0x38, 0x6c, 0xc6},
	{6, 0x88, 0x88, 0x88, 0x88, 0x88, 0xfc, 0x04},
	{6, 0x84, 0x84, 0xc4, 0xfc, 0x3c, 0x04, 0x04},
	{8, 0x99, 0x99, 0x99, 0x99, 0x99, 0x99, 0xff},
	{8, 0x92, 0x92, 0x92, 0x92, 0x92, 0xff, 0x01},
	{8, 0xe0, 0x20, 0x30, 0x3e, 0x23, 0x33, 0x3e},
	{8, 0x83, 0x83, 0x83, 0xfb, 0x8d, 0x8d, 0xfb},
	{6, 0x80, 0x80, 0xc0, 0xf8, 0x8c, 0x8c, 0xf8},
	{6, 0x78, 0xcc, 0x0c, 0x3c, 0x8c, 0xcc, 0x78},
	{8, 0x9e, 0x92, 0xb3, 0xf1, 0xb3, 0x92, 0x9e},
	{6, 0x7c, 0x44, 0x44, 0x7c, 0x24, 0x64, 0xc4},
	{3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{5, 0x50, 0x20, 0x78, 0xd8, 0xf8, 0xc8, 0x70},
};

/* UTF-8 -> codepoint (без проверки старших байтов: данные из файла) */
static uint32_t mf_next_cp(const unsigned char **ps, const unsigned char *end)
{
	const unsigned char *p = *ps;
	uint32_t c;
	int n, i;

	if (p >= end)
		return 0xFFFFFFFFu;
	if (p[0] < 0x80)              { c = p[0];           n = 1; }
	else if ((p[0] & 0xE0) == 0xC0 && end - p >= 2) { c = p[0] & 0x1F; n = 2; }
	else if ((p[0] & 0xF0) == 0xE0 && end - p >= 3) { c = p[0] & 0x0F; n = 3; }
	else if ((p[0] & 0xF8) == 0xF0 && end - p >= 4) { c = p[0] & 0x07; n = 4; }
	else                          { *ps = p + 1; return '?'; }
	for (i = 1; i < n; i++)
		c = (c << 6) | (p[i] & 0x3F);
	*ps = p + n;
	return c;
}

/* глиф по codepoint: {width, 7 строк}; NULL = символа нет */
static const unsigned char *mf_glyph(uint32_t cp, int *w)
{
	if (cp >= 0x20 && cp < 0x7f) {
		*w = mf_font_latin[cp - 0x20][0];
		return mf_font_latin[cp - 0x20];
	}
	if (cp >= 0x400 && cp <= 0x451) {
		*w = mf_font_cyr[cp - 0x400][0];
		return mf_font_cyr[cp - 0x400];
	}
	return NULL;
}

static int mf_text_width(const char *s, int len)
{
	const unsigned char *p = (const unsigned char *)s, *end = p + len;
	int tw = 0;
	while (p < end) {
		uint32_t cp = mf_next_cp(&p, end);
		int w;
		if (mf_glyph(cp, &w))
			tw += w + 1;
	}
	return tw > 0 ? tw - 1 : 0;
}

static void mf_blit_glyph(MF_RC *R, int x, int y, const unsigned char *g, uint32_t col)
{
	int w = g[0], gx, gy;
	for (gy = 0; gy < 7; gy++) {
		int row = g[1 + gy];
		for (gx = 0; gx < w; gx++)
			if (row & (1 << (7 - gx))) {
				int px = x + gx, py = y + gy;
				if (px >= 0 && px < R->w && py >= 0 && py < R->h)
					R->pix[(size_t)py * R->stride + px] = col;
			}
	}
}

/* текст по центру (cx,cy) с белой обводкой (halo), ночью инвертируется */
static void mf_draw_text(MF_RC *R, int cx, int cy, const char *s, int len)
{
	static const int off[8][2] = {
		{-1,0},{1,0},{0,-1},{0,1},{-1,-1},{1,-1},{-1,1},{1,1}
	};
	const unsigned char *p = (const unsigned char *)s, *end = p + len;
	uint32_t tc = mf_color(40, 40, 40, R->night);
	uint32_t hc = mf_color(255, 255, 255, R->night);
	int tw = mf_text_width(s, len);
	int x = cx - tw / 2, y = cy - 3;
	int i;

	if (tw <= 0)
		return;
	for (i = 0; i < 8; i++) {           /* halo всеми смещениями */
		const unsigned char *q = p;
		int gx = x + off[i][0], gy = y + off[i][1];
		while (q < end) {
			uint32_t cp = mf_next_cp(&q, end);
			int w;
			const unsigned char *g = mf_glyph(cp, &w);
			if (g) {
				mf_blit_glyph(R, gx, gy, g, hc);
				gx += w + 1;
			}
		}
	}
	{
		const unsigned char *q = p;
		int gx = x;
		while (q < end) {
			uint32_t cp = mf_next_cp(&q, end);
			int w;
			const unsigned char *g = mf_glyph(cp, &w);
			if (g) {
				mf_blit_glyph(R, gx, y, g, tc);
				gx += w + 1;
			}
		}
	}
}

/* копия имени в буфер метки: только символы с глифами, обрезка без
 * разрыва UTF-8 последовательности; возвращает длину */
static int mf_copy_label(char *dst, const char *src, int len)
{
	const unsigned char *p = (const unsigned char *)src, *end = p + len;
	int o = 0, cpn = 0;

	while (p < end && cpn < 24) {
		const unsigned char *q = p;
		uint32_t cp = mf_next_cp(&p, end);
		int w, n;
		if (!mf_glyph(cp, &w))
			continue;
		n = (int)(p - q);
		if (o + n > MF_LABEL_TEXT - 1)
			break;
		memcpy(dst + o, q, n);
		o += n;
		cpn++;
	}
	dst[o] = 0;
	return o;
}

static void mf_fill(uint32_t *pix, int stride, int x0, int y0, int w, int h,
                    int cw, int ch, uint32_t col)
{
	int x, y;
	if (x0 < 0) { w += x0; x0 = 0; }
	if (y0 < 0) { h += y0; y0 = 0; }
	if (w <= 0 || h <= 0) return;
	if (x0 + w > cw) w = cw - x0;
	if (y0 + h > ch) h = ch - y0;
	if (w <= 0 || h <= 0) return;
	for (y = 0; y < h; y++) {
		uint32_t *row = pix + (size_t)(y0 + y) * stride + x0;
		for (x = 0; x < w; x++)
			row[x] = col;
	}
}

/* Брезенхем с прямоугольным пером (перенос draw_line из mf_proto.py) */
static void mf_draw_line(MF_RC *R, int x0, int y0, int x1, int y1, uint32_t col, int thick)
{
	int dx = x1 > x0 ? x1 - x0 : x0 - x1;
	int dy = -(y1 > y0 ? y1 - y0 : y0 - y1);
	int sx = x0 < x1 ? 1 : -1;
	int sy = y0 < y1 ? 1 : -1;
	int err = dx + dy;
	int half = thick / 2;
	int ox, oy;

	for (;;) {
		for (oy = -half; oy <= half; oy++) {
			for (ox = -half; ox <= half; ox++) {
				int x = x0 + ox, y = y0 + oy;
				if (x >= 0 && x < R->w && y >= 0 && y < R->h)
					R->pix[(size_t)y * R->stride + x] = col;
			}
		}
		if (x0 == x1 && y0 == y1)
			return;
		{
			int e2 = 2 * err;
			if (e2 >= dy) { err += dy; x0 += sx; }
			if (e2 <= dx) { err += dx; y0 += sy; }
		}
	}
}

/* дельты мкград к СЗ углу базового тайла -> пиксели выходного тайла */
static void mfrc_project(const MF_RC *R, int32_t latd, int32_t lond, int *x, int *y)
{
	int64_t nx = ((int64_t)R->cur_bx << 24) + (int64_t)lond * R->XQ;
	int64_t ny = ((int64_t)R->cur_by << 24) + (int64_t)latd * R->NYQ;
	*x = (int)((nx - R->ox) >> (16 + R->k));
	*y = (int)((ny - R->oy) >> (16 + R->k));
}

/* стиль way: первый по приоритету таблицы из его тегов; иначе дефолт */
static int mf_style_of_way(const MF_WAY *w)
{
	int best = -1, i;
	for (i = 0; i < w->ntags; i++) {
		int t = w->tags[i];
		if (t < mf_way_tag_n && mf_way_tag_style[t]) {
			int s = mf_way_tag_style[t] - 1;
			if (best < 0 || s < best)
				best = s;
		}
	}
	return best < 0 ? MF_DEFAULT_STYLE : best;
}

static void mf_way_draw_cb(const MF_WAY *w, MF_NODE_ITER *it, void *user)
{
	MF_RC *R = (MF_RC *)user;
	int style = mf_style_of_way(w);
	int isback = style < MF_DEFAULT_STYLE && mf_styles[style].back;
	uint32_t col;
	int thick;
	int32_t la, lo;
	int x, y, px = 0, py = 0, have = 0;
	/* метка имени: дорога (не фон), внешний контур, зум достаточно крупный */
	int want_label = (R->pass == 1 && R->zlab && !isback &&
	                  w->name != NULL && w->name_len > 0 &&
	                  w->block_index == 0 && w->ring_index == 0 &&
	                  R->nlab < MF_LABEL_MAX);
	int nmid = 0;
	int32_t tot = 0;

	if (isback != (R->pass == 0))
		return;
	if (style < MF_DEFAULT_STYLE) {
		col = mf_color(mf_styles[style].r, mf_styles[style].g, mf_styles[style].b, R->night);
		thick = mf_styles[style].thick;
	} else {
		col = mf_color(100, 170, 100, R->night);
		thick = 1;
	}
	while (mf_next_node(it, &la, &lo)) {
		mfrc_project(R, la, lo, &x, &y);
		if (have) {
			mf_draw_line(R, px, py, x, y, col, thick);
			if (want_label)
				tot += (int32_t)(x > px ? x - px : px - x) +
				       (int32_t)(y > py ? y - py : py - y);
		}
		px = x; py = y; have = 1;
		if (want_label && nmid < MF_MID_MAX) {
			mf_mid_x[nmid] = (int16_t)x;
			mf_mid_y[nmid] = (int16_t)y;
			mf_mid_len[nmid] = tot;
			nmid++;
		}
	}
	/* имя рисуем, если дорога на экране не короче текста (с допуском) */
	if (want_label && nmid >= 2 && tot >= 12) {
		int tw = mf_text_width(w->name, w->name_len);
		if (tot + 6 >= tw) {
			int32_t target = tot / 2, l0, l1, seg;
			int i2, mx, my;
			for (i2 = 1; i2 < nmid; i2++)
				if (mf_mid_len[i2] >= target)
					break;
			i2--;
			l0 = mf_mid_len[i2];
			l1 = (i2 + 1 < nmid) ? mf_mid_len[i2 + 1] : l0 + 1;
			seg = l1 - l0;
			mx = mf_mid_x[i2];
			my = mf_mid_y[i2];
			if (seg > 0 && i2 + 1 < nmid) {
				int32_t f = target - l0;
				mx += (int)(((int64_t)(mf_mid_x[i2 + 1] - mf_mid_x[i2]) * f) / seg);
				my += (int)(((int64_t)(mf_mid_y[i2 + 1] - mf_mid_y[i2]) * f) / seg);
			}
			if (mf_copy_label(mf_lab_text[R->nlab], w->name, w->name_len) > 0) {
				/* метки не должны перекрывать друг друга: bbox с запасом
				 * на halo, иначе обводка одной затирает текст другой */
				int bx0 = mx - tw / 2 - 2, bx1 = bx0 + tw + 3;
				int by0 = my - 4, by1 = by0 + 9;
				int j, ok = 1;
				for (j = 0; j < R->nlab; j++) {
					int ax0 = mf_lab_x[j] - mf_lab_tw[j] / 2 - 2;
					int ax1 = ax0 + mf_lab_tw[j] + 3;
					int ay0 = mf_lab_y[j] - 4, ay1 = ay0 + 9;
					if (!(bx1 < ax0 || ax1 < bx0 || by1 < ay0 || ay1 < by0)) {
						ok = 0;
						break;
					}
				}
				if (ok) {
					mf_lab_x[R->nlab] = (int16_t)mx;
					mf_lab_y[R->nlab] = (int16_t)my;
					mf_lab_tw[R->nlab] = (int16_t)tw;
					R->nlab++;
				}
			}
		}
	}
}

static void mf_poi_draw_cb(const MF_POI *p, void *user)
{
	MF_RC *R = (MF_RC *)user;
	int x, y;
	if (R->pass != 1 || R->npoi >= MF_POI_DRAW_MAX)
		return;
	mfrc_project(R, p->lat, p->lon, &x, &y);
	mf_poi_px[R->npoi] = (int16_t)x;
	mf_poi_py[R->npoi] = (int16_t)y;
	R->npoi++;
}

/* интервал, содержащий зум z (иначе ближайший по расстоянию) */
static MF_INTERVAL *mf_pick_interval(int z)
{
	int i, best = -1, bestd = 1 << 30;
	for (i = 0; i < mf_iv_n; i++) {
		int d;
		if (z >= mf_iv[i].min && z <= mf_iv[i].max)
			return &mf_iv[i];
		d = z < mf_iv[i].min ? mf_iv[i].min - z : z - mf_iv[i].max;
		if (d < bestd) {
			bestd = d;
			best = i;
		}
	}
	return best >= 0 ? &mf_iv[best] : NULL;
}

int mf_render_tile(int z, int tx, int ty,
                   uint32_t *pix, int w, int h, int stride, int nightmode)
{
	MF_RC R;
	MF_INTERVAL *zi;
	int64_t ox, oy;
	int b0x, b1x, b0y, b1y, pass;
	uint32_t bg, waterc;

	if (pix == NULL || w <= 0 || h <= 0 || stride < w)
		return MF_ERR_FORMAT;
	if (!mf_opened) {
		mf_fill(pix, stride, 0, 0, w, h, w, h, mf_color(MF_BG_R, MF_BG_G, MF_BG_B, nightmode));
		return MF_ERR_NO_MAP;
	}
	zi = mf_pick_interval(z);
	if (zi == NULL)
		return MF_ERR_NO_MAP;

	bg = mf_color(MF_BG_R, MF_BG_G, MF_BG_B, nightmode);
	waterc = mf_color(MF_WATER_R, MF_WATER_G, MF_WATER_B, nightmode);
	mf_fill(pix, stride, 0, 0, w, h, w, h, bg);

	/* Окно тайла в базовых пикселях (Q16): СЗ-угол карты стоит в (0,0)
	 * мира MapThis; k = base - z (может быть < 0 — апскейл). */
	R.k = zi->base - z;
	if (R.k < -8 || R.k > 8)
		return MF_ERR_FORMAT;
	ox = ((int64_t)zi->x0 << 24) + ((int64_t)tx << (24 + R.k));
	oy = ((int64_t)zi->y0 << 24) + ((int64_t)ty << (24 + R.k));
	R.ox = ox;
	R.oy = oy;
	R.pix = pix;
	R.w = w; R.h = h; R.stride = stride;
	R.night = nightmode;
	R.npoi = 0;
	R.nlab = 0;
	R.zlab = (z >= 13);            /* имена улиц только на крупных зумах */

	/* блоки, покрывающие окно (шаг окна в базовых пикселях = 2^k) */
	b0x = (int)(ox >> 24);
	b1x = (int)(((ox + ((int64_t)w << (16 + R.k)) - 1) >> 24) + 1);
	b0y = (int)(oy >> 24);
	b1y = (int)(((oy + ((int64_t)h << (16 + R.k)) - 1) >> 24) + 1);
	if (b0x < zi->x0) b0x = zi->x0;
	if (b0y < zi->y0) b0y = zi->y0;
	if (b1x > zi->x0 + zi->tiles_x) b1x = zi->x0 + zi->tiles_x;
	if (b1y > zi->y0 + zi->tiles_y) b1y = zi->y0 + zi->tiles_y;

	/* XQ: пикселей(Q16) базового зума за мкград долготы */
	R.XQ = (int64_t)(((double)(1LL << (24 + zi->base)) / 360000000.0) + 0.5);

	/* Два прохода: сначала фоновые стили (здания/вода), потом дороги и
	 * остальное. Блок читается с MS один раз, декодируется дважды. */
	for (pass = 0; pass < 2; pass++) {
		int bx, by;
		R.pass = pass;
		for (by = b0y; by < b1y; by++) {
			/* NYQ: пикселей(Q16) за мкград широты строки блоков;
			 * lat к югу отрицателен, y растёт вниз — знак минус */
			double nlat = mf_tile_y_to_lat((double)by, zi->base);
			double slat = mf_tile_y_to_lat((double)by + 1.0, zi->base);
			double span = (nlat - slat) * 1e6;
			if (span <= 0) span = 1;
			R.NYQ = -(int64_t)((65536.0 * 256.0 / span) + 0.5);
			R.cur_by = by;
			for (bx = b0x; bx < b1x; bx++) {
				int water, len;
				R.cur_bx = bx;
				len = mf_read_block(zi, bx - zi->x0, by - zi->y0, &water);
				if (len == 0) {
					/* пустой тайл: вода заливается, суша остаётся фоном */
					if (pass == 0 && R.k >= 0 && water) {
						int qx = (int)((((int64_t)bx << 24) - ox) >> (16 + R.k));
						int qy = (int)((((int64_t)by << 24) - oy) >> (16 + R.k));
						int qs = 256 >> R.k;
						mf_fill(pix, stride, qx, qy, qs, qs, w, h, waterc);
					}
					continue;
				}
				if (len < 0)
					continue;      /* ошибка чтения — остаётся фон */
				mf_decode_block(mf_blk, len, zi->min, zi->max, z,
				                mf_poi_draw_cb, mf_way_draw_cb, &R);
			}
		}
	}

	/* POI поверх дорог — точки 3x3, как в прототипе */
	{
		int i, dx, dy;
		uint32_t col = mf_color(MF_POI_R, MF_POI_G, MF_POI_B, nightmode);
		for (i = 0; i < R.npoi; i++) {
			for (dy = -1; dy <= 1; dy++)
				for (dx = -1; dx <= 1; dx++) {
					int x = mf_poi_px[i] + dx, y = mf_poi_py[i] + dy;
					if (x >= 0 && x < w && y >= 0 && y < h)
						pix[(size_t)y * stride + x] = col;
				}
		}
	}
	/* имена улиц — последним слоем */
	{
		int i;
		for (i = 0; i < R.nlab; i++)
			mf_draw_text(&R, mf_lab_x[i], mf_lab_y[i],
			             mf_lab_text[i], (int)strlen(mf_lab_text[i]));
	}
	return MF_OK;
}
