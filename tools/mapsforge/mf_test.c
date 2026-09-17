/*
 * mf_test.c — host-тест C-читателя mapsforge (фазы 1–2 из todo_mapsforge_c.md).
 *
 * Сверяется с эталонным выводом tools/mapsforge/mf_proto.py:
 *   - заголовок/интервалы/теги;
 *   - тайл z14 x9571 y4763 (интервал 2, сетка 14,18), render_z = zmax:
 *     1397 POI / 3742 way / 29970 узлов, zoom-таблица, топ-12 way-тегов,
 *     первые 12 именованных ways, первые координаты;
 *   - полная валидация детального интервала: 1190 тайлов, 57 пустых,
 *     161278 POI, 786172 way, 5723258 узлов, 0 ошибок;
 *   - рендеры в PNG (stored-deflate, без zlib): z14/z13/z12/z10/z8.
 *
 * Сборка (из корня репозитория):
 *   cc -O2 -Wall -o tools/mapsforge/mf_test tools/mapsforge/mf_test.c mapsforge.c -lm
 * Запуск из корня репозитория (пути относительные).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "../../mapsforge.h"

#define MAP_DEFAULT "planet_30.004_59.781_927ddaa9-mapsforge-osm/planet_30.004_59.781_927ddaa9.map"

/* эталон из mf_proto.py */
#define REF_TILE_POIS   1397L
#define REF_TILE_WAYS   3742L
#define REF_TILE_NODES  29970L
#define REF_ALL_TILES   1190L
#define REF_ALL_EMPTY   57L
#define REF_ALL_POIS    161278L
#define REF_ALL_WAYS    786172L
#define REF_ALL_NODES   5723258L

/* эталоны НЕдетальных интервалов (независимо посчитаны mf_proto-парсером):
 * {tiles, empty, poi, way, nodes} — интервалы по порядку, кроме детального */
#define REF_IV0 {1, 0, 1, 2773, 5577}      /* base 5,  1x1  */
#define REF_IV1 {9, 0, 18, 42508, 200111}  /* base 10, 3x3  */

/* ------------------------------------------------------------------ */
/* тайминги                                                            */
/* ------------------------------------------------------------------ */

static double now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* ------------------------------------------------------------------ */
/* PNG writer (RGB, stored-deflate — без zlib)                          */
/* ------------------------------------------------------------------ */

static uint32_t crc_tab[256];

static void crc_init(void)
{
	uint32_t i, k, c;
	for (i = 0; i < 256; i++) {
		c = i;
		for (k = 0; k < 8; k++)
			c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
		crc_tab[i] = c;
	}
}
static uint32_t crc_run(uint32_t crc, const unsigned char *p, size_t n)
{
	size_t i;
	for (i = 0; i < n; i++)
		crc = crc_tab[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
	return crc;
}
static uint32_t adler32(const unsigned char *p, size_t n)
{
	uint32_t a = 1, b = 0;
	size_t i;
	for (i = 0; i < n; i++) {
		a = (a + p[i]) % 65521;
		b = (b + a) % 65521;
	}
	return (b << 16) | a;
}

static void put32(FILE *f, uint32_t v)
{
	fputc((int)(v >> 24), f);
	fputc((int)(v >> 16), f);
	fputc((int)(v >> 8), f);
	fputc((int)v, f);
}

static void png_chunk(FILE *f, const char *tag, const unsigned char *data, size_t n)
{
	uint32_t c = 0xFFFFFFFFu;
	put32(f, (uint32_t)n);
	fwrite(tag, 1, 4, f);
	if (n)
		fwrite(data, 1, n, f);
	c = crc_run(c, (const unsigned char *)tag, 4);
	if (n)
		c = crc_run(c, data, n);
	put32(f, c ^ 0xFFFFFFFFu);
}

/* pix — формат PSP GU 8888: A<<24|B<<16|G<<8|R */
static void write_png(const char *path, int w, int h, const uint32_t *pix, int stride)
{
	FILE *f;
	unsigned char *raw, *zbuf;
	size_t rowlen = 1 + (size_t)w * 3;
	size_t rawlen = rowlen * (size_t)h;
	size_t zlen, pos, i;
	uint32_t ad;
	int x, y;

	raw = malloc(rawlen);
	if (raw == NULL)
		return;
	for (y = 0; y < h; y++) {
		unsigned char *row = raw + (size_t)y * rowlen;
		row[0] = 0;   /* фильтр None */
		for (x = 0; x < w; x++) {
			uint32_t c = pix[(size_t)y * stride + x];
			row[1 + x * 3]     = (unsigned char)(c & 0xFF);         /* R */
			row[1 + x * 3 + 1] = (unsigned char)((c >> 8) & 0xFF);  /* G */
			row[1 + x * 3 + 2] = (unsigned char)((c >> 16) & 0xFF); /* B */
		}
	}

	/* zlib-поток из stored-блоков: 2 байт заголовок + блоки + adler32 */
	zlen = 2 + ((rawlen + 65534) / 65535) * 5 + rawlen + 4;
	zbuf = malloc(zlen);
	if (zbuf == NULL) {
		free(raw);
		return;
	}
	zbuf[0] = 0x78;
	zbuf[1] = 0x01;
	pos = 2;
	i = 0;
	while (i < rawlen) {
		size_t n = rawlen - i;
		if (n > 65535)
			n = 65535;
		zbuf[pos++] = (i + n == rawlen) ? 1 : 0;   /* BFINAL в последнем */
		zbuf[pos++] = (unsigned char)(n & 0xFF);
		zbuf[pos++] = (unsigned char)(n >> 8);
		zbuf[pos++] = (unsigned char)(~n & 0xFF);
		zbuf[pos++] = (unsigned char)((~n >> 8) & 0xFF);
		memcpy(zbuf + pos, raw + i, n);
		pos += n;
		i += n;
	}
	ad = adler32(raw, rawlen);
	zbuf[pos++] = (unsigned char)(ad >> 24);
	zbuf[pos++] = (unsigned char)(ad >> 16);
	zbuf[pos++] = (unsigned char)(ad >> 8);
	zbuf[pos++] = (unsigned char)ad;

	f = fopen(path, "wb");
	if (f != NULL) {
		static const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
		unsigned char ihdr[13];
		ihdr[0] = (unsigned char)(w >> 24); ihdr[1] = (unsigned char)(w >> 16);
		ihdr[2] = (unsigned char)(w >> 8);  ihdr[3] = (unsigned char)w;
		ihdr[4] = (unsigned char)(h >> 24); ihdr[5] = (unsigned char)(h >> 16);
		ihdr[6] = (unsigned char)(h >> 8);  ihdr[7] = (unsigned char)h;
		ihdr[8] = 8;   /* бит на канал */
		ihdr[9] = 2;   /* цвет: RGB */
		ihdr[10] = ihdr[11] = ihdr[12] = 0;
		fwrite(sig, 1, 8, f);
		png_chunk(f, "IHDR", ihdr, 13);
		png_chunk(f, "IDAT", zbuf, pos);
		png_chunk(f, "IEND", NULL, 0);
		fclose(f);
	}
	free(zbuf);
	free(raw);
}

/* ------------------------------------------------------------------ */
/* счётчики декода                                                     */
/* ------------------------------------------------------------------ */

#define MAX_SHOW 12

typedef struct {
	long pois, ways, nodes;
	long tagcnt[MF_MAX_TAGS];
	int   first_poi;
	int32_t fpoi_lat, fpoi_lon;
	int   first_way;
	int32_t fway_lat, fway_lon;
	int   nnames;
	char  names[MAX_SHOW][80];
	const char *name_tags[MAX_SHOW];
} CNT;

static void cnt_poi_cb(const MF_POI *p, void *user)
{
	CNT *c = (CNT *)user;
	c->pois++;
	if (!c->first_poi) {
		c->first_poi = 1;
		c->fpoi_lat = p->lat;
		c->fpoi_lon = p->lon;
	}
}

static void cnt_way_cb(const MF_WAY *w, MF_NODE_ITER *it, void *user)
{
	CNT *c = (CNT *)user;
	int32_t la, lo;

	if (w->block_index == 0 && w->ring_index == 0) {
		int i;
		c->ways++;
		for (i = 0; i < w->ntags; i++)
			if (w->tags[i] < mf_way_tag_count())
				c->tagcnt[w->tags[i]]++;
		if (c->nnames < MAX_SHOW && w->name && w->name_len > 0) {
			int n = w->name_len < 79 ? w->name_len : 79;
			memcpy(c->names[c->nnames], w->name, n);
			c->names[c->nnames][n] = 0;
			c->name_tags[c->nnames] =
				(w->ntags > 0 && w->tags[0] < mf_way_tag_count())
					? mf_way_tag(w->tags[0]) : "?";
			c->nnames++;
		}
	}
	while (mf_next_node(it, &la, &lo)) {
		c->nodes++;
		if (!c->first_way) {
			c->first_way = 1;
			c->fway_lat = la;
			c->fway_lon = lo;
		}
	}
}

/* ------------------------------------------------------------------ */

static int fails = 0;

static void check(const char *what, long got, long ref)
{
	printf("  %-34s %10ld   эталон %10ld   %s\n", what, got, ref,
	       got == ref ? "OK" : "*** РАСХОЖДЕНИЕ ***");
	if (got != ref)
		fails++;
}

int main(int argc, char **argv)
{
	const char *map = argc > 1 ? argv[1] : MAP_DEFAULT;
	CNT cnt;
	double t0, dt;
	int i, rc;
	int x0, y0, tx, ty;
	int iv_detail = -1;

	t0 = now_ms();
	if (!mf_open(map)) {
		printf("mf_open(%s) не удался: %s\n", map, mf_strerror());
		return 2;
	}
	printf("открытие карты: %.1f мс\n", now_ms() - t0);
	printf("интервалов: %d, POI-тегов: %d, way-тегов: %d, max_base_zoom: %d (TILE_NUM=%d)\n",
	       mf_interval_count(), mf_poi_tag_count(), mf_way_tag_count(),
	       mf_max_base_zoom(), mf_tile_num());
	for (i = 0; i < mf_interval_count(); i++) {
		mf_interval_grid(i, &x0, &y0, &tx, &ty);
		printf("  интервал %d: base=%d  z%d..z%d  сетка %dx%d  от x%d y%d\n",
		       i, mf_interval_base(i), mf_interval_min(i), mf_interval_max(i),
		       tx, ty, x0, y0);
		if (mf_interval_base(i) == mf_max_base_zoom()) {
			iv_detail = i;
		}
	}
	if (iv_detail < 0) {
		printf("нет детального интервала\n");
		return 2;
	}
	mf_interval_grid(iv_detail, &x0, &y0, &tx, &ty);
	printf("детальный интервал %d: всего тайлов %d\n\n", iv_detail, tx * ty);

	/* ---- эталонный тайл z14 x9571 y4763 (сетка 14,18), все уровни ---- */
	{
		uint32_t pairs[2 * 40];
		int lv;

		printf("=== Тайл z14 x9571 y4763 (render_z=zmax) ===\n");
		memset(&cnt, 0, sizeof(cnt));
		t0 = now_ms();
		rc = mf_decode_tile_block(iv_detail, 9571 - x0, 4763 - y0,
		                          mf_interval_max(iv_detail),
		                          cnt_poi_cb, cnt_way_cb, &cnt);
		dt = now_ms() - t0;
		printf("декод: %.1f мс, rc=%d\n", dt, rc);
		check("POI", cnt.pois, REF_TILE_POIS);
		check("ways", cnt.ways, REF_TILE_WAYS);
		check("узлов в ways", cnt.nodes, REF_TILE_NODES);
		if (cnt.first_poi)
			printf("первый POI: дельта lat=%d lon=%d мкград к СЗ углу\n",
			       cnt.fpoi_lat, cnt.fpoi_lon);
		if (cnt.first_way)
			printf("первый узел way: lat=%d lon=%d\n", cnt.fway_lat, cnt.fway_lon);

		lv = mf_block_zoom_table(iv_detail, 9571 - x0, 4763 - y0, pairs, 40);
		printf("zoom table (%d уровней):", lv);
		for (i = 0; i < lv; i++)
			printf(" z%d:%u/%u", mf_interval_min(iv_detail) + i,
			       pairs[2 * i], pairs[2 * i + 1]);
		printf("\n");

		printf("топ-12 way-тегов:\n");
		{
			int s, k;
			for (s = 0; s < 12; s++) {
				int best = -1;
				for (k = 0; k < mf_way_tag_count(); k++)
					if (best < 0 || cnt.tagcnt[k] > cnt.tagcnt[best])
						best = k;
				if (best < 0 || cnt.tagcnt[best] == 0)
					break;
				printf("   %-42s %ld\n", mf_way_tag(best), cnt.tagcnt[best]);
				cnt.tagcnt[best] = 0;
			}
		}
		printf("имена ways (первые %d):\n", cnt.nnames);
		for (i = 0; i < cnt.nnames; i++)
			printf("   %-42s (%s)\n", cnt.names[i], cnt.name_tags[i]);
	}

	/* ---- полная валидация детального интервала ---- */
	printf("\n=== Валидация всех тайлов детального интервала ===\n");
	{
		CNT tot;
		long n_empty = 0, n_err = 0;

		memset(&tot, 0, sizeof(tot));
		t0 = now_ms();
		{
			int ix, iy;
			for (iy = 0; iy < ty; iy++) {
				for (ix = 0; ix < tx; ix++) {
					rc = mf_decode_tile_block(iv_detail, ix, iy,
					                          mf_interval_max(iv_detail),
					                          cnt_poi_cb, cnt_way_cb, &tot);
					if (rc == MF_EMPTY)
						n_empty++;
					else if (rc < 0) {
						n_err++;
						if (n_err <= 3)
							printf("  ОШИБКА (x%d y%d): rc=%d\n",
							       x0 + ix, y0 + iy, rc);
					}
				}
			}
		}
		dt = now_ms() - t0;
		check("тайлов (всего)", (long)(tx * ty), REF_ALL_TILES);
		check("пустых", n_empty, REF_ALL_EMPTY);
		check("ошибок", n_err, 0);
		check("всего POI", tot.pois, REF_ALL_POIS);
		check("всего ways", tot.ways, REF_ALL_WAYS);
		check("всего узлов", tot.nodes, REF_ALL_NODES);
		printf("время полной валидации: %.0f мс (C, host)\n", dt);
	}

	/* ---- валидация остальных интервалов (oversize-блоки ловятся тут:
	 * блок base10 659614 байт > 256КБ давал пустой центр на z8..11) ---- */
	printf("\n=== Валидация обзорных интервалов (render_z=zmax) ===\n");
	{
		static const long ref_iv[2][5] = { REF_IV0, REF_IV1 };
		int ivi;

		for (ivi = 0; ivi < mf_interval_count(); ivi++) {
			CNT tot;
			long n_empty = 0, n_err = 0;
			int gx0, gy0, gtx, gty, ix, iy, ri = -1;

			if (mf_interval_base(ivi) == mf_max_base_zoom())
				continue;                     /* детальный проверен выше */
			if (ivi < 2)
				ri = ivi;
			mf_interval_grid(ivi, &gx0, &gy0, &gtx, &gty);
			memset(&tot, 0, sizeof(tot));
			for (iy = 0; iy < gty; iy++)
				for (ix = 0; ix < gtx; ix++) {
					rc = mf_decode_tile_block(ivi, ix, iy,
					                          mf_interval_max(ivi),
					                          cnt_poi_cb, cnt_way_cb, &tot);
					if (rc == MF_EMPTY)
						n_empty++;
					else if (rc < 0) {
						n_err++;
						printf("  ОШИБКА (x%d y%d): rc=%d\n", gx0 + ix, gy0 + iy, rc);
					}
				}
			printf("интервал %d (base %d, %dx%d): пустых %ld, ошибок %ld, "
			       "POI %ld, ways %ld, узлов %ld\n",
			       ivi, mf_interval_base(ivi), gtx, gty,
			       n_empty, n_err, tot.pois, tot.ways, tot.nodes);
			if (ri >= 0 && ri < 2) {
				char lbl[64];
				snprintf(lbl, sizeof(lbl), "ив%d тайлов", ri);
				check(lbl, (long)(gtx * gty), ref_iv[ri][0]);
				snprintf(lbl, sizeof(lbl), "ив%d пустых", ri);
				check(lbl, n_empty, ref_iv[ri][1]);
				snprintf(lbl, sizeof(lbl), "ив%d POI", ri);
				check(lbl, tot.pois, ref_iv[ri][2]);
				snprintf(lbl, sizeof(lbl), "ив%d ways", ri);
				check(lbl, tot.ways, ref_iv[ri][3]);
				snprintf(lbl, sizeof(lbl), "ив%d узлов", ri);
				check(lbl, tot.nodes, ref_iv[ri][4]);
			}
		}
	}

	/* ---- рендеры ---- */
	printf("\n=== Рендеры (256x256, день) ===\n");
	{
		struct { int z, gx, gy; } rends[] = {
			{14, 9571, 4763}, {13, 4785, 2381}, {12, 2392, 1190},
			{10, 597, 296},   {8,  149, 74}
		};
		uint32_t *pix = malloc(256 * 256 * 4);
		uint32_t bg = 0xFF000000u | (238u << 16) | (245u << 8) | 245u;
		int nrend = (int)(sizeof(rends) / sizeof(rends[0]));

		if (pix == NULL) {
			printf("нет памяти на рендер\n");
			return 2;
		}
		crc_init();
		for (i = 0; i < nrend; i++) {
			int iv = -1, k, lx, ly, j, nonbg = 0, ivi;
			int gx0, gy0, gtx, gty;
			char path[128];

			/* интервал, содержащий z */
			for (ivi = 0; ivi < mf_interval_count(); ivi++) {
				if (rends[i].z >= mf_interval_min(ivi) &&
				    rends[i].z <= mf_interval_max(ivi)) {
					iv = ivi;
					break;
				}
			}
			if (iv < 0) {
				printf("z%d: интервал не найден\n", rends[i].z);
				continue;
			}
			mf_interval_grid(iv, &gx0, &gy0, &gtx, &gty);
			k = mf_interval_base(iv) - rends[i].z;
			lx = rends[i].gx - (gx0 >> k);
			ly = rends[i].gy - (gy0 >> k);
			t0 = now_ms();
			rc = mf_render_tile(rends[i].z, lx, ly, pix, 256, 256, 256, 0);
			dt = now_ms() - t0;
			for (j = 0; j < 256 * 256; j++)
				if (pix[j] != bg)
					nonbg++;
			snprintf(path, sizeof(path), "tools/mapsforge/ctest_%d_%d_%d.png",
			         rends[i].z, rends[i].gx, rends[i].gy);
			write_png(path, 256, 256, pix, 256);
			printf("z%d тайл глобальный (%d,%d) локальный (%d,%d): "
			       "%.1f мс, rc=%d, геометрия %.1f%%  -> %s\n",
			       rends[i].z, rends[i].gx, rends[i].gy, lx, ly,
			       dt, rc, nonbg * 100.0 / (256.0 * 256.0), path);
		}
		free(pix);
	}

	mf_close();
	printf("\nИТОГ: %s\n", fails ? "ЕСТЬ РАСХОЖДЕНИЯ С ЭТАЛОНОМ" : "ВСЁ СОВПАДАЕТ С ЭТАЛОНОМ");
	return fails ? 1 : 0;
}
