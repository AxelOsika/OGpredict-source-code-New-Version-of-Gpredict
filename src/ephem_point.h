#ifndef _EPHEM_POINT_H_
#define _EPHEM_POINT_H_

#include "sgpsdp/sgp4sdp4.h"  /* for definition of sat_t */
#include <glib.h>            /* for GSList */
#include <glib/gprintf.h> /* for g_strdup_printf */
#include "predict-tools.h" /* for qth_t and predict_calc() */
/**
 * Holds one “ephemeris sample”:
 *   - epoch_jd:   Julian date (UTC)
 *   - lat_deg:    sub-satellite latitude in degrees
 *   - lon_deg:    sub-satellite longitude in degrees
 */
typedef struct {
    double epoch_jd;
    char   *time_str; 
    double lat_deg;
    double lon_deg;
} EphemPoint;

/* Exported helper to free one Ephemeris point (matches definition in .c) */
void free_ephem_point(gpointer data);

void collect_groundtrack_duration(sat_t *sat, qth_t *qth,
                                  int duration_s, int step_sec);
/**
 * A global, singly-linked list of EphemPoint*.
 *   - g_ephem_buffer: head of the list (newest at head)
 *   - g_ephem_buffer_count: number of nodes currently stored
 */
extern GSList *g_ephem_buffer;
extern guint   g_ephem_buffer_count;

#endif  /* _EPHEM_POINT_H_ */
