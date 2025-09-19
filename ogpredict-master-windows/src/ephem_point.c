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




#include "ephem_point.h"
#include "predict-tools.h"    /* for predict_calc() */
#include <glib/gprintf.h>     /* for g_strdup_printf */
#include <stdlib.h>
#include <math.h>

/* Forward‐declare the Gregorian converter from gtk-sat-map-ground-track.c */
void jd_to_gregorian(double jd,
                    int   *year_out,
                    int   *month_out,
                    int   *day_out,
                    int   *hour_out,
                    int   *min_out,
                    int   *sec_out);


void free_ephem_point(gpointer data) {
    EphemPoint *p = (EphemPoint*)data;
    g_free(p->time_str);
    g_free(p);
}

void collect_groundtrack_duration(sat_t *sat, qth_t *qth,
                                  int duration_s,
                                  int step_sec){
    /* 1) clear the old buffer */
    g_slist_free_full(g_ephem_buffer,
                      (GDestroyNotify)free_ephem_point);
    g_ephem_buffer = NULL;
    g_ephem_buffer_count = 0;

    /* 2) sample at t=0…duration_s seconds _after_ the current Julian date */
    double jul_now = sat->jul_utc;           /* JD of “now” */
    for (int sec = 0; sec <= duration_s; sec += step_sec) {
        double jul_point = jul_now + ((double)sec)/86400.0;

        /* advance the satellite state to that JD */
        predict_calc(sat, qth, jul_point);

        EphemPoint *p = g_new0(EphemPoint, 1);
        p->epoch_jd = jul_point;

        /* build “YYYY/MM/DD HH:MM:SS” from the JD */
        {
            int Y, Mo, D, h, m, s;
            jd_to_gregorian(jul_point, &Y, &Mo, &D, &h, &m, &s);
           p->time_str = g_strdup_printf(
                "%04d/%02d/%02d %02d:%02d:%02d",
                Y, Mo, D, h, m, s
            );
        }

        /* subsatellite lat/lon at that time */
        predict_get_subsatellite_coords(sat,
                                       &p->lat_deg,
                                       &p->lon_deg);

        /* O(1) push-front to avoid O(N^2) appends on Windows */
        g_ephem_buffer = g_slist_prepend(g_ephem_buffer, p);
        g_ephem_buffer_count++;
    }    
    
    /* restore chronological order after prepend */
    g_ephem_buffer = g_slist_reverse(g_ephem_buffer);

}



/* Initialize the global buffer to empty */
GSList *g_ephem_buffer       = NULL;
guint   g_ephem_buffer_count = 0;
