/*
  OGpredict — extensions to Gpredict for operations planning

  Copyright (C) 2025 Axel Osika <osikaaxel@gmail.com>

  This file is part of OGpredict, a derivative of Gpredict.

  OGpredict is free software: you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the
  Free Software Foundation; either version 2 of the License, or (at your
  option) any later version.  See the GNU General Public License for details.
*/

/* SPDX-License-Identifier: GPL-2.0-or-later */



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
