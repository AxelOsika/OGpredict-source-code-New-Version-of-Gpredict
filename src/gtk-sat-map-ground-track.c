/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2017  Alexandru Csete, OZ9AEC.

  Comments, questions and bugreports should be submitted via
  http://sourceforge.net/projects/gpredict/
  More details can be found at the project home page:

  http://gpredict.oz9aec.net/
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, visit http://www.fsf.org/
*/
/**
 * Implementation of the satellite ground tracks.
 *
 * @note The ground track functions should only be called from gtk-sat-map.c
 *       and gtk-sat-map-popup.c.
 *
 */
#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif
#include <gtk/gtk.h>
#include <glib/gi18n.h>

#include "config-keys.h"
#include "gtk-sat-map.h"
#include "gtk-sat-map-ground-track.h"
#include "mod-cfg-get-param.h"
#include "orbit-tools.h"
#include "predict-tools.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "sgpsdp/sgp4sdp4.h"
#include <stdio.h>            /* for printf */
#include "ephem_point.h"      /* our EphemPoint + g_ephem_buffer */
#include <math.h>    /* for floor(), fmod() */

/**
 * jd_to_gregorian():
 *
 *   Given a Julian Date (jd, in UTC), compute the corresponding
 *   Gregorian calendar date and time (year, month, day, hour, minute, second).
 *
 *   Algorithm is from Fliegel & Van Flandern (1968) / Jean Meeus.
 *
 * Inputs:
 *   jd       : Julian Date in UTC (e.g. 2460832.43600475)
 * Outputs (all output pointers must be non-NULL):
 *   year_out : 4-digit year (e.g. 2024)
 *   month_out: month (1–12)
 *   day_out  : day of month (1–31)
 *   hour_out : hour of day (0–23)
 *   min_out  : minute (0–59)
 *   sec_out  : second (0–59, rounded to nearest integer)
 */

/*-------------------------------------------------------------------------------------
 * Helper to free a single EphemPoint, including its time_str.
 * This function matches the GDestroyNotify signature (i.e. takes one gpointer).
 *-----------------------------------------------------------------------------------*/


void
jd_to_gregorian(double jd,
                int   *year_out,
                int   *month_out,
                int   *day_out,
                int   *hour_out,
                int   *min_out,
                int   *sec_out){

    /* 1) Convert JD to “Julian day number” (integer) plus fractional day */
    double Z, F;
    long   J;
    Z = floor(jd + 0.5);
    F = (jd + 0.5) - Z;            /* fractional part of day */
    J = (long) Z;                  /* integer part */

    long   A;
    if (J >= 2299161L) {
        /* Gregorian reform: */
        long alpha = (long) floor((J - 1867216.25) / 36524.25);
        A = J + 1 + alpha - (long)floor(alpha / 4.0);
    } else {
        A = J;
    }

    /* 2) Convert to “B” */
    long B = A + 1524;

    /* 3) Year and month calculations */
    long C = (long) floor((B - 122.1) / 365.25);
    long D = (long) floor(365.25 * C);
    long E = (long) floor((B - D) / 30.6001);

    double day_decimal = B - D - floor(30.6001 * E) + F; 
    /* day_decimal is day-of-month + fractional day */

    int day = (int) floor(day_decimal);  /* integer day-of-month */

    int month;
    if (E < 14) {
        month = (int) (E - 1);
    } else {
        month = (int) (E - 13);
    }

    int year;
    if (month > 2) {
        year = (int) (C - 4716);
    } else {
        year = (int) (C - 4715);
    }

    /* 4) Extract time from fractional part of day_decimal */
    double fractional_day = day_decimal - day; 
    /* fractional_day is in [0,1) of one day (i.e. 24h) */

    double total_seconds = fractional_day * 86400.0; 
    /* total seconds since 00:00:00 of that day */

    int hour = (int) floor(total_seconds / 3600.0);
    double rem = total_seconds - (hour * 3600.0);
    int minute = (int) floor(rem / 60.0);
    double seconds = rem - (minute * 60.0);

    /* Round to nearest integer second (you could also floor) */
    int second = (int) floor(seconds + 0.5);
    if (second >= 60) {
        second -= 60;
        minute += 1;
        if (minute >= 60) {
            minute -= 60;
            hour += 1;
            if (hour >= 24) {
                /* Roll into next day */
                hour -= 24;
                day += 1;
                /* Naïvely increment day without re-checking month boundaries;
                   in practice the JD → Gregorian algorithm above produces
                   day already in correct range, and rounding might only add
                   one second. If it exactly hits 24:00:00, you could adjust
                   more robustly, but this is seldom needed for ground-track. */
            }
        }
    }

    /* 5) Store outputs */
    *year_out  = year;
    *month_out = month;
    *day_out   = day;
    *hour_out  = hour;
    *min_out   = minute;
    *sec_out   = second;
}

/**
 * print_all_ephemeris_points()
 *
 *   Iterates over the global g_ephem_buffer and prints
 *   each (epoch_jd, latitude, longitude) in chronological order.
 *   Since we used g_slist_prepend() when collecting, the newest
 *   entry is at the head. We reverse once, walk, then reverse back.
 */
void
print_all_ephemeris_points(void)
{
    if (g_ephem_buffer_count == 0) {
        printf("No ephemeris points to print.\n");
        fflush(stdout);
        return;
    }

    // buffer is already in chronological order; no reversal needed

    // Walk from oldest -> newest
    for (GSList *walk = g_ephem_buffer; walk; walk = walk->next) {
        EphemPoint *pp = (EphemPoint *) walk->data;

        /* Convert pp->epoch_jd to Gregorian components */
        int Y, Mo, D, h, m, s;
        jd_to_gregorian(pp->epoch_jd, &Y, &Mo, &D, &h, &m, &s);

        

    }

    // no need to reverse back now that we always append in time‐order

    fflush(stdout);
}


/**
 * collect_groundtrack_points()
 *
 *   Revised so that the very first point collected is the satellite’s current
 *   position/time (sat->jul_utc). Then steps forward in fixed 30 s increments
 *   until N_orbits are completed. Each EphemPoint’s time_str is built from
 *   the JD using jd_to_gregorian(), so the “Date / Time” column matches “now.”
 */
void
collect_groundtrack_points(sat_t *sat, qth_t *qth,
                          int    N_orbits,
                          int    step_sec)
{
    double jul_now  = sat->jul_utc;      /* current Julian date */
    long   orbit0   = sat->orbit;        /* “now” orbit number */
    long   max_orbit = orbit0 + N_orbits;
    /* use user’s chosen step (seconds) */
    const double dt_forward = (double)step_sec / 86400.0;

    /* 1) Free any old EphemPoints (and their time_str) */
    if (g_ephem_buffer) {
        g_slist_free_full(g_ephem_buffer, free_ephem_point);
    }
    g_ephem_buffer = NULL;
    g_ephem_buffer_count = 0;

    /* 2) Insert the “now” point as the very first sample */
    {
        EphemPoint *p = g_malloc(sizeof(EphemPoint));
        p->epoch_jd = jul_now;

        /* Convert jul_now to YYYY/MM/DD HH:MM:SS */
        int Y, Mo, D, h, m, s;
        jd_to_gregorian(jul_now, &Y, &Mo, &D, &h, &m, &s);
        p->time_str = g_strdup_printf(
            "%04d/%02d/%02d %02d:%02d:%02d",
            Y, Mo, D, h, m, s
        );

        p->lat_deg = sat->ssplat;
        p->lon_deg = sat->ssplon;

        g_ephem_buffer = g_slist_append(g_ephem_buffer, p);
        g_ephem_buffer_count++;
    }

    /* 3) Step forward from jul_now in 30 s increments until we've done N_orbits */
    double t = jul_now;
    while ((sat->orbit <= max_orbit) && (!decayed(sat))) {
        t += dt_forward;
        predict_calc(sat, qth, t);

        EphemPoint *p = g_malloc(sizeof(EphemPoint));
        p->epoch_jd = sat->jul_utc;

        int Y, Mo, D, h, m, s;
        jd_to_gregorian(sat->jul_utc, &Y, &Mo, &D, &h, &m, &s);
        p->time_str = g_strdup_printf(
            "%04d/%02d/%02d %02d:%02d:%02d",
            Y, Mo, D, h, m, s
        );

        p->lat_deg = sat->ssplat;
        p->lon_deg = sat->ssplon;

        g_ephem_buffer = g_slist_append(g_ephem_buffer, p);
        g_ephem_buffer_count++;
    }

    /* 4) Restore satellite’s live state back to jul_now */
    predict_calc(sat, qth, jul_now);
}




static void     create_polylines(GtkSatMap * satmap, sat_t * sat, qth_t * qth,
                                 sat_map_obj_t * obj);
static gboolean ssp_wrap_detected(GtkSatMap * satmap, gdouble x1, gdouble x2);
static void     free_ssp(gpointer ssp, gpointer data);


/**
 * Create and show ground track for a satellite.
 *
 * @param satmap The satellite map widget.
 * @param sat Pointer to the satellite object.
 * @param qth Pointer to the QTH data.
 * @param obj the satellite object.
 *  
 * Gpredict allows the user to require the ground track for any number of orbits
 * ahead. Therefore, the resulting ground track may cross the map boundaries many
 * times, and using one single polyline for the whole ground track would look very
 * silly. To avoid this, the points will be split into several polylines.
 */
void ground_track_create(GtkSatMap * satmap, sat_t * sat, qth_t * qth,
                         sat_map_obj_t * obj)
{
    long            this_orbit; /* current orbit number */
    long            max_orbit;  /* target orbit number, ie. this + num - 1 */
    double          t0;         /* time when this_orbit starts */
    double          t;
    

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: Creating ground track for %s"),
                __func__, sat->nickname);

    /* just to be safe... if empty GSList is not NULL => segfault */
    obj->track_data.latlon = NULL;

    /* get configuration parameters */
    this_orbit = sat->orbit;
    max_orbit = sat->orbit - 1 + mod_cfg_get_int(satmap->cfgdata,
                                                 MOD_CFG_MAP_SECTION,
                                                 MOD_CFG_MAP_TRACK_NUM,
                                                 SAT_CFG_INT_MAP_TRACK_NUM);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: Start orbit: %d"), __func__, this_orbit);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: End orbit %d"), __func__, max_orbit);

    /* find the time when the current orbit started */

    /* Iterate backwards in time until we reach sat->orbit < this_orbit.
       Use predict_calc from predict-tools.c as SGP/SDP driver.
       As a built-in safety, we stop iteration if the orbit crossing is
       more than 24 hours back in time.
     */
    t0 = satmap->tstamp;        //get_current_daynum ();
    /* use == instead of >= as it is more robust */
    for (t = t0; (sat->orbit == this_orbit) && ((t + 1.0) > t0); t -= 0.0007)
        predict_calc(sat, qth, t);

    /* set it so that we are in the same orbit as this_orbit
       and not a different one */
    t += 2 * 0.0007;
    t0 = t;
    predict_calc(sat, qth, t0);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: T0: %f (%d)"), __func__, t0, sat->orbit);

    /* calculate (lat,lon) for the required orbits */
    while ((sat->orbit <= max_orbit) &&
           (sat->orbit >= this_orbit) && (!decayed(sat)))
    {
        /* We use 30 sec time steps. If resolution is too fine, the
           line drawing routine will filter out unnecessary points
         */
        t += 0.00035;
        predict_calc(sat, qth, t);

        /* store this SSP */

        /* Note: g_slist_append() has to traverse the entire list to find the end, which
           is inefficient when adding multiple elements. Therefore, we use g_slist_prepend()
           and reverse the entire list when we are done.
         */
        ssp_t *this_ssp = g_try_new(ssp_t, 1);

        if (!this_ssp) {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Insufficient memory for ground track!"),
                        __func__);
            return;
        }

        this_ssp->lat = sat->ssplat;
        this_ssp->lon = sat->ssplon;
        obj->track_data.latlon = g_slist_prepend(obj->track_data.latlon,
                                                 this_ssp);

    }

    /* —– Section C: Restore “live” state so sat->jul_utc == real now —– */
    predict_calc(sat, qth, satmap->tstamp);

    /* Temporary debug: collect 1 orbit’s worth of points and print a count */
    /* default to 10 orbits, 30 s step */
    collect_groundtrack_points(sat, qth, 10, 30);
    print_all_ephemeris_points();
    
    fflush(stdout);

    /* Regardless of orbit count, reverse & draw the polylines now */
    obj->track_data.latlon = g_slist_reverse(obj->track_data.latlon);
    create_polylines(satmap, sat, qth, obj);

    /* log if there is a problem with the orbit calculation */
    if (sat->orbit != (max_orbit + 1))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Problem computing ground track for %s"),
                    __func__, sat->nickname);
        return;
    }

    /* Reset satellite structure to eliminate glitches in single sat 
       view and other places when new ground track is laid out */
    predict_calc(sat, qth, satmap->tstamp);

    /* misc book-keeping */
    obj->track_orbit = this_orbit;
}

/**
 * Update the ground track for a satellite.
 *
 * @param satmap The satellite map widget.
 * @param sat Pointer to the satellite object.
 * @param qth Pointer to the QTH data.
 * @param obj the satellite object.
 * @param recalc Flag indicating whether ground track should be recalculated.
 *
 *    If (recalc=TRUE)
 *       call ground_track_delete (clear_ssp=TRUE)
 *       call ground_track_create
 *    Else
 *       call ground_track_delete (clear_ssp=FALSE)
 *       call create_polylines
 *
 *
 * The purpose with the recalc flag is to allow updates of ground track look without having
 * to recalculate the whole ground track (recalc=FALSE). 
 */
void ground_track_update(GtkSatMap * satmap, sat_t * sat, qth_t * qth,
                         sat_map_obj_t * obj, gboolean recalc)
{
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: Updating ground track for %s"),
                __func__, sat->nickname);

    if (decayed(sat))
    {
        ground_track_delete(satmap, sat, qth, obj, TRUE);
        return;
    }

    if (recalc == TRUE)
    {
        ground_track_delete(satmap, sat, qth, obj, TRUE);
        ground_track_create(satmap, sat, qth, obj);
    }
    else
    {
        ground_track_delete(satmap, sat, qth, obj, FALSE);
        create_polylines(satmap, sat, qth, obj);
    }
}

/**
 * Delete the ground track for a satellite.
 *
 * @param satmap The satellite map widget.
 * @param sat Pointer to the satellite object.
 * @param qth Pointer to the QTH data.
 * @param obj the satellite object.
 * @param clear_ssp Flag indicating whether SSP data should be cleared as well (TRUE=yes);
 */
void ground_track_delete(GtkSatMap * satmap, sat_t * sat, qth_t * qth,
                         sat_map_obj_t * obj, gboolean clear_ssp)
{
    guint           i, n;
    gint            j;
    GooCanvasItemModel *line;
    GooCanvasItemModel *root;

    (void)qth;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: Deleting ground track for %s"),
                __func__, sat->nickname);

    root = goo_canvas_get_root_item_model(GOO_CANVAS(satmap->canvas));

    /* remove plylines */
    if (obj->track_data.lines != NULL)
    {
        n = g_slist_length(obj->track_data.lines);

        for (i = 0; i < n; i++)
        {
            /* get line */
            line =
                GOO_CANVAS_ITEM_MODEL(g_slist_nth_data
                                      (obj->track_data.lines, i));

            /* find its ID and remove it */
            j = goo_canvas_item_model_find_child(root, line);
            if (j == -1)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Could not find part %d of ground track"),
                            __func__, j);
            }
            else
            {
                goo_canvas_item_model_remove_child(root, j);
            }
        }

        g_slist_free(obj->track_data.lines);
        obj->track_data.lines = NULL;
    }

    /* clear SSP too? */
    if (clear_ssp == TRUE)
    {
        if (obj->track_data.latlon != NULL)
        {
            /* free allocated ssp_t */
            g_slist_foreach(obj->track_data.latlon, free_ssp, NULL);

            /* free the SList itself */
            g_slist_free(obj->track_data.latlon);
            obj->track_data.latlon = NULL;

        }

        obj->track_orbit = 0;
    }
}

/**
 * Free an ssp_t structure.
 *
 * The ssp_t items in the obj->track_data.latlon GSList are dunamically allocated
 * hence they need to be freed when the cround track is deleted. This function
 * is intended to be called from a g_slist_foreach() iterator.
 */
static void free_ssp(gpointer ssp, gpointer data)
{
    (void)data;
    g_free(ssp);
}

/** Create polylines. */
static void create_polylines(GtkSatMap * satmap, sat_t * sat, qth_t * qth,
                             sat_map_obj_t * obj)
{
    ssp_t          *ssp, *buff; /* map coordinates */
    
    double          lastx, lasty;
    GSList         *points = NULL;
    GooCanvasItemModel *root;
    GooCanvasItemModel *line;
    GooCanvasPoints *gpoints;
    guint           start=0;
    guint           i, j, n, num_points;
    guint32         col;

    (void)sat;
    (void)qth;

    /* initialise parameters */
    lastx = -50.0;
    lasty = -50.0;
    num_points = 0;
    n = g_slist_length(obj->track_data.latlon);
    col = mod_cfg_get_int(satmap->cfgdata,
                          MOD_CFG_MAP_SECTION,
                          MOD_CFG_MAP_TRACK_COL, SAT_CFG_INT_MAP_TRACK_COL);

    /* loop over each SSP */
    for (i = 0; i < n; i++)
    {
        buff = (ssp_t *) g_slist_nth_data(obj->track_data.latlon, i);
        ssp = g_try_new(ssp_t, 1);

        if (!ssp) {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Insufficient memory for ground track!"),
                        __func__);
            return;
        }

        gtk_sat_map_lonlat_to_xy(satmap, 
                                buff->lon,
                                buff->lat, 
                                &ssp->lon,
                                &ssp->lat);

        /* if this is the first point, just add it to the list */
        if (i == start)
        {
            points = g_slist_prepend(points, ssp);
            lastx = ssp->lon;
            lasty = ssp->lat;
        }
        else
        {
            /* if SSP is on the other side of the map */
            if (ssp_wrap_detected(satmap, lastx, ssp->lon))
            {
                points = g_slist_reverse(points);
                num_points = g_slist_length(points);

                /* we need at least 2 points to draw a line */
                if (num_points > 1)
                {
                    /* convert SSPs to GooCanvasPoints */
                    gpoints = goo_canvas_points_new(num_points);
                    for (j = 0; j < num_points; j++)
                    {
                        buff = (ssp_t *) g_slist_nth_data(points, j);
                        gpoints->coords[2 * j] = buff->lon;
                        gpoints->coords[2 * j + 1] = buff->lat;
                    }

                    /* create a new polyline using the current set of points */
                    root =
                        goo_canvas_get_root_item_model(GOO_CANVAS
                                                       (satmap->canvas));

                    line = goo_canvas_polyline_model_new(root, FALSE, 0,
                                                         "points", gpoints,
                                                         "line-width", 1.0,
                                                         "stroke-color-rgba",
                                                         col, "line-cap",
                                                         CAIRO_LINE_CAP_SQUARE,
                                                         "line-join",
                                                         CAIRO_LINE_JOIN_MITER,
                                                         NULL);
                    goo_canvas_points_unref(gpoints);
                    goo_canvas_item_model_lower(line, obj->marker);

                    /* store line in sat object */
                    obj->track_data.lines =
                        g_slist_append(obj->track_data.lines, line);

                }

                /* reset parameters and continue with a new set */
                g_slist_foreach(points, free_ssp, NULL);
                g_slist_free(points);
                points = NULL;
                start = i;
                lastx = ssp->lon;
                lasty = ssp->lat;
                num_points = 0;

                /* Add current SSP to the new list */
                points = g_slist_prepend(points, ssp);
                lastx = ssp->lon;
                lasty = ssp->lat;
            }

            /* else if this SSP is separable from the previous */
            else if ((fabs(lastx - ssp->lon) > 1.0) ||
                     (fabs(lasty - ssp->lat) > 1.0))
            {

                /* add SSP to list */
                points = g_slist_prepend(points, ssp);
                lastx = ssp->lon;
                lasty = ssp->lat;
            }
            /* else do nothing */
            else g_free(ssp);
        }
    }

    /* create (last) line if we have at least two points */
    points = g_slist_reverse(points);
    num_points = g_slist_length(points);

    if (num_points > 1)
    {
        /* convert SSPs to GooCanvasPoints */
        gpoints = goo_canvas_points_new(num_points);
        for (j = 0; j < num_points; j++)
        {
            buff = (ssp_t *) g_slist_nth_data(points, j);
            gpoints->coords[2 * j] = buff->lon;
            gpoints->coords[2 * j + 1] = buff->lat;
        }

        /* create a new polyline using the current set of points */
        root = goo_canvas_get_root_item_model(GOO_CANVAS(satmap->canvas));

        line = goo_canvas_polyline_model_new(root, FALSE, 0,
                                             "points", gpoints,
                                             "line-width", 1.0,
                                             "stroke-color-rgba", col,
                                             "line-cap", CAIRO_LINE_CAP_SQUARE,
                                             "line-join",
                                             CAIRO_LINE_JOIN_MITER, NULL);
        goo_canvas_points_unref(gpoints);
        goo_canvas_item_model_lower(line, obj->marker);

        /* store line in sat object */
        obj->track_data.lines = g_slist_append(obj->track_data.lines, line);

        /* reset parameters and continue with a new set */
        g_slist_foreach(points, free_ssp, NULL);
        g_slist_free(points);
    }
}

/** Check whether ground track wraps around map borders */
static gboolean ssp_wrap_detected(GtkSatMap * satmap, gdouble x1, gdouble x2)
{
    gboolean        retval = FALSE;

    if (fabs(x1 - x2) > satmap->width / 2.0)
        retval = TRUE;

    return retval;
}
