#ifndef MAPSFORGE_H
#define MAPSFORGE_H

/*
 * Портативный (без PSPSDK) читатель векторных карт Mapsforge .map (file version 3)
 * и программный рендер тайлов 256x256 в ARGB-буфер.
 *
 * Формат пикселя — нативный PSP GU 8888 (как у png/jpeg-загрузчиков graphics.c):
 *   u32 = A<<24 | B<<16 | G<<8 | R  (R в младшем байте)
 *
 * Спецификация формата, подводные камни и план: todo_mapsforge_c.md.
 * Эталонная реализация (проверена, 0 ошибок на всей тестовой карте):
 *   tools/mapsforge/mf_proto.py
 *
 * Тайлы нумеруются в slippy-координатах (Google/OSM); сетка интервала и
 * bbox берутся из заголовка .map. Блоки тайлов читаются точечно из файла,
 * индексы всех интервалов кэшируются в RAM при открытии.
 */

#include <stdint.h>

#define MF_BLOCK_SIZE    (2*1024*1024)  /* буфер одного блока тайла (ОБЗОРНЫЕ интервалы дают
				       * блоки крупнее детальных: центр СПб на base10 =
				       * 659614 байт; 2 МБ с запасом, выделяется malloc
				       * при mf_open) */
#define MF_MAX_TAGS      1024         /* макс. тегов в POI-/way-таблице заголовка */
#define MF_MAX_INTERVALS 8

/* коды возврата mf_render_tile / mf_decode_tile_block (<0 = ошибка) */
#define MF_OK            0
#define MF_EMPTY         1        /* пустой тайл (off == off_next) */
#define MF_ERR_NO_MAP   (-1)
#define MF_ERR_IO       (-2)
#define MF_ERR_FORMAT   (-3)

/* ---- итератор узлов кольца way -------------------------------------- */
/* Выдаёт узлы кольца как абсолютные дельты микроградусов к СЗ-углу
 * базового тайла блока. Callback обязан выбрать все узлы (или обход
 * молча дочитает остаток после возврата). */

typedef struct {
	/* приватное состояние */
	const unsigned char *p;   /* позиция чтения в блоке */
	const unsigned char *end; /* конец блока (граница чтения) */
	int    remaining;         /* непрочитанных узлов после текущего */
	int    double_delta;
	int32_t lat, lon;         /* текущий узел (дельта мкград к СЗ углу) */
	int32_t dlat, dlon;       /* накопленные дельты для double-delta */
	int    has_cur;
	int    err;               /* EOF/повреждение потока */
} MF_NODE_ITER;

int mf_next_node(MF_NODE_ITER *it, int32_t *lat, int32_t *lon); /* 1 = узел выдан */

/* ---- объекты --------------------------------------------------------- */

typedef struct {
	int           layer;                 /* 0..10, writer пишет как есть */
	int           ntags;                 /* 0..15 */
	uint16_t      tags[15];              /* индексы в таблице POI-тегов */
	const char   *name;      int name_len;
	const char   *housenumber; int hn_len;
	int           elevation;             /* валиден при has_elevation */
	int32_t       lat, lon;              /* дельта мкград к СЗ углу базового тайла */
	int           has_elevation;
} MF_POI;

typedef struct {
	int           layer;                 /* 0..10 */
	int           ntags;
	uint16_t      tags[15];              /* индексы в таблице way-тегов */
	const char   *name;      int name_len;
	const char   *housenumber; int hn_len;
	const char   *ref;       int ref_len;
	int32_t       label_lat, label_lon;  /* валидны при has_label:
	                                        дельта к ПЕРВОМУ УЗЛУ way, не к углу! */
	uint16_t      subtile_bitmap;
	int           has_label;
	int           double_delta;
	int           block_index, ring_index; /* текущий блок/кольцо way
	                                        (0/0 — внешний контур; у простых
	                                        way всегда 0/0) */
} MF_WAY;

/* Вызывается на каждое КОЛЬЦО каждого way-блока (у мультиполигонов
 * первый блок/кольцо — внешний контур, далее внутренние). */
typedef void (*MF_WAY_CB)(const MF_WAY *way, MF_NODE_ITER *nodes, void *user);
typedef void (*MF_POI_CB)(const MF_POI *poi, void *user);

/* ---- открытие/закрытие карты ----------------------------------------- */

/* path — путь к файлу .map ИЛИ к каталогу (тогда берётся первый *.map
 * внутри; MapThis передаёт каталог карты "./maps/_ИМЯ"). 1 = успех. */
int  mf_open(const char *path);
void mf_close(void);
int  mf_is_open(void);

const char *mf_strerror(void);          /* текст последней ошибки */

/* ---- параметры карты для интеграции с MapThis ------------------------ */

int  mf_max_base_zoom(void);            /* base самого детального интервала (напр. 14) */
int  mf_tile_num(void);                 /* 1 << max_base_zoom — аналог TILE_NUM */
void mf_base_tile(int *tx, int *ty);    /* СЗ-угол сетки на max_base_zoom — для ftx/fty */

/* ---- низкоуровневый доступ (тесты/валидация) ------------------------- */

int  mf_interval_count(void);
int  mf_interval_base(int i);
int  mf_interval_min(int i);
int  mf_interval_max(int i);
void mf_interval_grid(int i, int *x0, int *y0, int *tiles_x, int *tiles_y);
int  mf_poi_tag_count(void);
int  mf_way_tag_count(void);
const char *mf_poi_tag(int i);
const char *mf_way_tag(int i);

/* zoom-таблица блока: до max_pairs пар (poi_cnt, way_cnt) по уровням
 * min..max. Возвращает число уровней или <0. */
int mf_block_zoom_table(int interval, int ix, int iy,
                        uint32_t *pairs /*[2*max_pairs]*/, int max_pairs);

/* Прочитать блок тайла (ix,iy — координаты ВНУТРИ сетки интервала) и
 * декодировать его с зум-фильтром render_z (берутся объекты уровней <= z).
 * render_z = interval_max означает «без фильтра» + строгие проверки границ. */
int mf_decode_tile_block(int interval, int ix, int iy, int render_z,
                         MF_POI_CB poi_cb, MF_WAY_CB way_cb, void *user);

/* ---- рендер ----------------------------------------------------------- */

/* Отрендерить тайл (tx,ty) на slippy-зуме z в буфер pix (w x h, шаг stride
 * uint32-слов; для Image textureWidth==256 stride=256). tx,ty — «локальные»
 * координаты мира MapThis (0..2^z-1), карта стоит своим СЗ-углом в (0,0).
 * Пустые/водные тайлы и область вне bbox заливаются фоном/водой.
 * Буфер всегда заполняется целиком; код <0 означает внутреннюю ошибку
 * (данные могли быть частично дорисованы). */
int mf_render_tile(int z, int tx, int ty,
                   uint32_t *pix, int w, int h, int stride, int nightmode);

#endif /* MAPSFORGE_H */
