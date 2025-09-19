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

/*
  OGpredict — large-scale modifications for operations planning
  Copyright (C) 2024–2025 Axel Osika <osikaaxel@gmail.com>

  This file is a substantial rewrite derived from the original
  Gpredict file "gtk-sat-map-popup.c" by Alexandru Csete (OZ9AEC).
  Major OGpredict changes include:
   • New 3-tab popup (Ephemeris / Territory / POI) and UI wiring.
   • 1 Hz ephemeris generation with chunked GTK inserts (idle-driven).
   • Asynchronous workers (GTask) with cancellability & progress bars.
   • Country/POI spatial filtering with polygon BBoxes and fast PIP.
   • Parallel POI slicing, detaching/re-attaching models for speed.

  License: GPL-2.0-or-later (same as upstream). This notice adds
  authorship for the modifications and does not remove upstream notices.
*/
/* SPDX-License-Identifier: GPL-2.0-or-later */


#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <glib/gi18n.h>
#include <gtk/gtk.h>
#include <glib.h>
#include <gio/gio.h>   
/* Forward decls for handlers used before their definitions */
static void on_poi_refresh_clicked(GtkButton *button, gpointer user_data);
/* Always provide our own stubs so g_task_report_progress()
// ──────────────────────────────────────────────────────────────
// COMMON — Dialog / Helpers / Shared
// ──────────────────────────────────────────────────────────────
 * @brief Callback / helper function.
 * @function g_task_get_progress_fraction
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param 2.46 ) are always declared */

/* only build these stubs for GLib < 2.46
 * @param !GLIB_CHECK_VERSION(2 which already provides these symbols */

/* only build these stubs for GLib < 2.46, which already provides these symbols */
#if !GLIB_CHECK_VERSION(2,46,0)
static void
g_task_report_progress(GTask        *task,
                       gdouble       fraction,
                       GCancellable *cancellable)
{
    (void)task; (void)fraction; (void)cancellable;
}
/*
 * @brief Callback / helper function.
 * @function g_task_get_progress_fraction
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param task GTask *task
 * @return (gdouble)
 */

static gdouble
g_task_get_progress_fraction(GTask *task)
{
    (void)task;
    return 0.0;
}
#endif

#include "config-keys.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "gtk-sat-data.h"
#include "gtk-sat-map.h"
#include "gtk-sat-map-popup.h"
#include "gtk-sat-map-ground-track.h"
#include "gtk-sat-popup-common.h"
#include "mod-cfg-get-param.h"
#include "orbit-tools.h"
#include "predict-tools.h"
#include "sat-info.h"
#include "sat-pass-dialogs.h"
#include "sgpsdp/sgp4sdp4.h"
#include <glib/gprintf.h>                 /* for g_strdup_printf() */
#include <glib.h>                       /* for g_idle_add_full */
#include <math.h>

/* New Code*/
/* Filters */
#include "Logic_Country_Filter.h"                       /*  filter module for table in tab2 */
#include "Logic_POI_Filter.h"            /*  filter module for table in tab3 */

/* Loading data*/
#include "ephem_point.h"                 /* Loading EphemPoint, buffer */
#include "points_interests.h"           /* Loading points of interests */

/* Helper */
#include "sub_window_ephemeris.h"  /* helper for sub window in tab3 */
#include "countries.h"             /* helper for scroll input in tab2 */

#if !GLIB_CHECK_VERSION(2,68,0)
#define g_memdup2(mem, n_bytes) g_memdup((mem), (n_bytes))
#endif


/* Safe wrapper: avoid GTK_IS_WIDGET assertion on NULL/destroyed widgets */
static inline void safe_set_sensitive(GtkWidget *w, gboolean s) {
    if (GTK_IS_WIDGET(w)) gtk_widget_set_sensitive(w, s);
}


/* ---- Fast bbox for POI polygons (pre-filter before point-in-polygon) ---- */
typedef struct { double min_lat, max_lat, min_lon, max_lon; gboolean wraps; } BBox;
static inline BBox bbox_from_poly(GArray *poly) {
    BBox b = {  90.0, -90.0,  180.0, -180.0, FALSE };
    LP_GeoPoint *p = (LP_GeoPoint*)poly->data;
    for (guint i = 0; i < poly->len; ++i) {
        if (p[i].lat < b.min_lat) b.min_lat = p[i].lat;
        if (p[i].lat > b.max_lat) b.max_lat = p[i].lat;
        if (p[i].lon < b.min_lon) b.min_lon = p[i].lon;
        if (p[i].lon > b.max_lon) b.max_lon = p[i].lon;
    }
    /* crude wrap detection: very wide span implies crossing the dateline */
    b.wraps = (b.max_lon - b.min_lon) > 300.0;
    return b;
}

static inline gboolean bbox_contains(const BBox *b, double lat, double lon) {
    if (lat < b->min_lat || lat > b->max_lat) return FALSE;
    if (b->wraps) return TRUE; /* skip lon check if polygon crosses ±180 */
    return !(lon < b->min_lon || lon > b->max_lon);
}

/*───────────────────────────────────────────────────────────────────────────*/
/* We need this to format YYYY/MM/DD HH:MM:SS from a Julian date:           */
void jd_to_gregorian(double jd,
                    int   *year_out,
                    int   *month_out,
                    int   *day_out,
                    int   *hour_out,
                    int   *min_out,
                    int   *sec_out);
/*───────────────────────────────────────────────────────────────────────────*/


// ──────────────────────────────────────────────────────────────
// TAB 1 — Ephemeris
// ──────────────────────────────────────────────────────────────
/* A small struct to carry our context into the callback */
typedef struct {
    GtkSatMap *satmap;
    sat_t     *sat;
    qth_t     *qth;
} ShowEphemCtx;

typedef struct {
    GtkSatMap    *satmap;
    sat_t        *sat;
    qth_t        *qth;
    GtkListStore *store;
    GtkTreeView  *treeview;
    GtkSpinButton *hours_spin;  /* number‐of‐hours selector */
    GtkSpinButton *step_spin;   /* time‐step selector (seconds) */
    GtkProgressBar *progress_bar;

    /* ── our private popup buffer ───────────────────────────────────────── */
    struct _POISelectionCtx *poi_ctx;  /* NEW: link to the POI context for auto-refresh */
    GSList        *buffer;
    guint          buffer_count;
    GtkLabel      *count_label;    /* NEW: shows total points */    

    guint           pulse_source_id; /* for pulsing progress bar */
    /* +++ add seconds-counter fields +++ */
    GtkLabel       *time_label;      /* shows elapsed seconds */
    guint           timer_source_id; /* id of the 1s timeout */
    guint64         start_time;      /* g_get_monotonic_time() at refresh */
    /* ← Capture these on the main thread before launching the GTask */
    int duration_s;
    int step_sec;
    /* streaming insert */
    GSList         *append_ptr;   /* next node in buffer to append */
    guint           idle_id;       /* idle source id for chunked appends */
    gboolean running;
    guint           inserted_count;   /* running count while streaming */
    gboolean        model_detached;   /* TRUE while tv model is detached */

} EphemUpdateCtx;


// ──────────────────────────────────────────────────────────────
// TAB 2 — Territory / Countries
// ──────────────────────────────────────────────────────────────
/*
 * @brief Callback / helper function.
 * @function zone_row_free
 * @param r ZoneRow *r
 * @return (void)
 */
/* A small struct to carry our context into the callback */

typedef struct {              /* for Tab 2 */
   gchar *time;
   gdouble lat, lon;
   gchar *country;
} ZoneRow;

/* ─── Tab 2: CountrySelectionCtx ───────────────────────────────────────── */
typedef struct _CountrySelectionCtx {
    GtkWidget      *button;        /* “Select Region” button */
    GtkEntry       *entry;         /* country entry */
    char            name[128];     /* selected country name */
    guint           selected_territory_id;
    GtkTreeView    *tv_tab1;       /* Tab 1 ephemeris view */
    GtkWidget      *treeview;      /* Tab 2 filtered list */
    GtkProgressBar *progress_bar;  /* loading bar */
    GtkListStore   *store;         /* Tab 2’s ListStore for filtered rows */
    GtkLabel       *count_label;   /* NEW: total‐points label for Tab 2 */
    guint           pulse_source_id; /* for pulsing progress bar */
    /* +++ add seconds-counter fields +++ */
    GtkLabel       *time_label;      /* shows elapsed seconds */
    guint           timer_source_id; /* id of the 1s timeout */
    guint64         start_time;      /* g_get_monotonic_time() at refresh */
    /* streaming insert + cancel */
    GCancellable   *cancel;
    GPtrArray      *pending_rows;
    guint           next_row;
    guint           idle_id;
    gboolean        model_detached; /* TRUE while tv model is detached */

} CountrySelectionCtx;

static void zone_row_free(ZoneRow *r){ if(!r) return; g_free(r->time); g_free(r->country); g_free(r); }


// ──────────────────────────────────────────────────────────────
// TAB 3 — Points of Interest
// ──────────────────────────────────────────────────────────────
/*
 * @brief Callback / helper function.
 * @function poi_row_free
 * @param r PoiRow *r
 * @return (void)
 */
/* A small struct to carry our context into the callback */

typedef struct {              /* for Tab 3 */
   gchar *time, *dir, *name, *type;
   gdouble lat, lon, range_km;
} PoiRow;

static void poi_row_free(PoiRow *r){ if(!r) return; g_free(r->time); g_free(r->dir); g_free(r->name); g_free(r->type); g_free(r); }

typedef struct _POISelectionCtx {
    GtkTreeView   *tab1_tree;
    GtkEntry      *entry;    /* the type-ahead text input */
    GtkWidget     *progress_bar;    /* +++ our new progress bar +++ */
    gchar          name[128];/* selected POI name */
    GPtrArray     *types;          /*  catched list of point types */
    GPtrArray     *names;    /* cached list from points_interests */
    GtkListStore  *store;    /* POI table model */
    GtkTreeView   *treeview; /* future POI-in-zone view */
    GtkWidget     *button;   /* Tab 3 “Refresh” button */
    guint          pulse_source_id; /* ─── for our pulsing timeout */
    GtkLabel      *time_label;      /* NEW: shows elapsed seconds */
    guint          timer_source_id; /* NEW: id of the 1 s timeout */
    guint64        start_time;      /* NEW: g_get_monotonic_time() at refresh */
    GCancellable *cancel;
    GPtrArray    *pending_rows;   /* from worker thread */
    guint         next_row;       /* next row index to append */
    guint         idle_id;        /* idle source for chunk appends */
    gboolean      model_detached; /* TRUE while tv model is detached */

} POISelectionCtx;

/*
Enums in this file give clear, named integers for UI indices and states.
They define the GtkTree/ListStore column order (COL_* per Tab 1/2/3) and
small state/mode sets (STATE_, MODE_) used by workers and signal handlers.
Using enums removes “magic numbers”, makes switch statements exhaustive, and
allows safe array sizing/looping via the N_* sentinel. Reordering columns or
adding states becomes a single-point change, keeping worker threads and the
GTK main loop in sync and the code self-documenting.
*/


// for the ephemeris table
enum { COL_TIME = 0, COL_LAT, COL_LON, N_COLS };

// for the country pop-over list
enum { COL_COUNTRY = 0, N_COUNTRY_COLS };

/* Columns for Tab 2 (“ephemeris-in-territory” table) */
enum {
    ZONE_COL_TIME = 0,
    ZONE_COL_LAT,
    ZONE_COL_LON,
    ZONE_COL_COUNTRY,
    ZONE_N_COLS
};

enum {
    POI_COL_TIME,
    POI_COL_LAT,
    POI_COL_LON,
    POI_COL_RANGE,
    POI_COL_DIR,
    POI_COL_NAME,
    POI_COL_TYPE,
    POI_N_COLS
};

/*── Struct & idle‐callback to push progress back to the main loop ─────────*/
typedef struct {
    EphemUpdateCtx *ctx;
    double          fraction;
} ProgressUpdate;

/*
 * @brief GtkNotebook "switch-page" handler.
 * @function on_nb_switch_page
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param nb GtkNotebook *nb
 * @param page GtkWidget   *page
 * @param page_num guint        page_num
 * @param user_data gpointer     user_data
 * @return (void)
 */


static void
on_nb_switch_page (GtkNotebook *nb,
                   GtkWidget   *page,
                   guint        page_num,
                   gpointer     user_data)
{
    GtkWidget *dialog  = GTK_WIDGET(user_data);
    GtkWidget *savebtn = gtk_dialog_get_widget_for_response(GTK_DIALOG(dialog),
                                                            GTK_RESPONSE_ACCEPT);
    GtkWidget *addbtn  = gtk_dialog_get_widget_for_response(GTK_DIALOG(dialog),
                                                            1001 /* RESPONSE_ADD */);
    if (!savebtn) return;

    if (page_num == 2)  /* 0=Ephemeris, 1=Territory, 2=POI */
    {
        gtk_widget_show(savebtn);
        if (addbtn) gtk_widget_show(addbtn);
    }
    else
    {
        gtk_widget_hide(savebtn);
        if (addbtn) gtk_widget_hide(addbtn);
    }

    (void)nb; (void)page;
}


/* ── ensure compiler knows the real signature, not int() ───────────── */
static gchar *format_bearing_text(double bearing);



/* ─── timeout callback to pulse the ephemeris progress bar ────────────────────── */
// ──────────────────────────────────────────────────────────────
// TAB 1 — Ephemeris
// ──────────────────────────────────────────────────────────────
/*
 * @brief GLib timeout to pulse an indeterminate GtkProgressBar while work is in progress.
 * @function ephem_pulse_timeout
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */
static gboolean
ephem_pulse_timeout(gpointer data)
{
    EphemUpdateCtx *ctx = data;
    if (!ctx || !GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        return G_SOURCE_REMOVE;
    gtk_progress_bar_pulse(GTK_PROGRESS_BAR(ctx->progress_bar));
    return G_SOURCE_CONTINUE;
}


/* forward decl so Tab 1 idle can call it without implicit int */
static void on_poi_entry_activate(GtkEntry *entry, gpointer user_data);

/*
 * @brief Idle callback that streams batched rows into the GtkListStore to keep UI responsive.
 * @function ephem_append_chunk_idle
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */

static gboolean ephem_append_chunk_idle(gpointer data)
{
    EphemUpdateCtx *ctx = data;
    if (!ctx || !GTK_IS_LIST_STORE(ctx->store) || !ctx->append_ptr)
        return G_SOURCE_REMOVE;
    const guint CHUNK = 20000;  /* ephemeris can be huge */
    guint added = 0;
    while (ctx->append_ptr && added < CHUNK) {
        EphemPoint *pp = ctx->append_ptr->data;
        /* single-call insert is a tad cheaper than append+set */
        gtk_list_store_insert_with_values(ctx->store, NULL, -1,
                           COL_TIME, pp->time_str,
                           COL_LAT,  pp->lat_deg,
                           COL_LON,  pp->lon_deg,
                           -1);
        ctx->append_ptr = ctx->append_ptr->next;
        ++added;
    }
    /* track how many were appended (optional debug), but don't rewrite "Total:" */
    ctx->inserted_count += added;

    if (!ctx->append_ptr) {
            /* finished streaming rows — now stop pulse/timer */
            if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }
            if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
            /* reset bar and re-enable controls */
            if (GTK_IS_PROGRESS_BAR(ctx->progress_bar))
                gtk_progress_bar_set_fraction(ctx->progress_bar, 0.0);
            safe_set_sensitive(GTK_WIDGET(ctx->hours_spin), TRUE);
            safe_set_sensitive(GTK_WIDGET(ctx->step_spin), TRUE);
            /* allow next run */
            ctx->running = FALSE;
            /* reattach model now that it’s full */
            if (ctx->model_detached && GTK_IS_TREE_VIEW(ctx->treeview)) {
                gtk_tree_view_set_model(ctx->treeview, GTK_TREE_MODEL(ctx->store));
                ctx->model_detached = FALSE;
            }
            /* now that Tab 1’s model is fully updated, auto-refresh POI if needed */
            if (ctx->poi_ctx && ctx->poi_ctx->name[0] != '\0')
                on_poi_entry_activate(ctx->poi_ctx->entry, ctx->poi_ctx);
            ctx->idle_id = 0;
            return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}

/*
 * @brief GtkDialog "response" handler.
 * @function on_ephem_dialog_response
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param dlg GtkDialog *dlg
 * @param response gint response
 * @param user_data gpointer user_data
 * @return (void)
 */

static void on_ephem_dialog_response(GtkDialog *dlg, gint response, gpointer user_data)
{
    (void)user_data;

    /* ─── Handle our new “Add” action ─────────────────────────────── */
    if (response == 1001) { /* RESPONSE_ADD */
        GtkWidget *sub = gtk_dialog_new_with_buttons(
            "Add Point of Interest", GTK_WINDOW(dlg),
            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
            "_Cancel", GTK_RESPONSE_CANCEL,
            "_Add",    GTK_RESPONSE_OK,
            NULL);
        GtkWidget *box = gtk_dialog_get_content_area(GTK_DIALOG(sub));
        GtkWidget *grid = gtk_grid_new();
        gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
        gtk_grid_set_column_spacing(GTK_GRID(grid), 8);
        gtk_container_add(GTK_CONTAINER(box), grid);

        GtkWidget *e_name = gtk_entry_new();
        GtkWidget *e_type = gtk_entry_new();
        GtkWidget *e_lat  = gtk_entry_new();
        GtkWidget *e_lon  = gtk_entry_new();
        GtkWidget *e_km   = gtk_entry_new();
        gtk_entry_set_placeholder_text(GTK_ENTRY(e_name), "e.g. My City");
        gtk_entry_set_placeholder_text(GTK_ENTRY(e_type), "e.g. City");
        gtk_entry_set_placeholder_text(GTK_ENTRY(e_lat),  "Center latitude (°)");
        gtk_entry_set_placeholder_text(GTK_ENTRY(e_lon),  "Center longitude (°)");
        gtk_entry_set_placeholder_text(GTK_ENTRY(e_km),   "Tile size (km)");

        gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Name:"),         0,0, 1,1);
        gtk_grid_attach(GTK_GRID(grid), e_name,                          1,0, 1,1);
        gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Type:"),         0,1, 1,1);
        gtk_grid_attach(GTK_GRID(grid), e_type,                          1,1, 1,1);
        gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Center Lat:"),   0,2, 1,1);
        gtk_grid_attach(GTK_GRID(grid), e_lat,                           1,2, 1,1);
        gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Center Lon:"),   0,3, 1,1);
        gtk_grid_attach(GTK_GRID(grid), e_lon,                           1,3, 1,1);
        gtk_grid_attach(GTK_GRID(grid), gtk_label_new("Tile size (km):"),0,4,1,1);
        gtk_grid_attach(GTK_GRID(grid), e_km,                            1,4, 1,1);

        gtk_widget_show_all(sub);
        if (gtk_dialog_run(GTK_DIALOG(sub)) == GTK_RESPONSE_OK) {
            const gchar *name = gtk_entry_get_text(GTK_ENTRY(e_name));
            const gchar *type = gtk_entry_get_text(GTK_ENTRY(e_type));
            gchar *endp = NULL;
            double lat = g_ascii_strtod(gtk_entry_get_text(GTK_ENTRY(e_lat)), &endp);
            double lon = g_ascii_strtod(gtk_entry_get_text(GTK_ENTRY(e_lon)), &endp);
            double km  = g_ascii_strtod(gtk_entry_get_text(GTK_ENTRY(e_km)),  &endp);

            gboolean ok_inputs = (name && *name && isfinite(lat) && isfinite(lon) &&
                                  isfinite(km) && km > 0.0 &&
                                  lat >= -90.0 && lat <= 90.0 &&
                                  lon >= -180.0 && lon <= 180.0);
            if (!ok_inputs) {
                GtkWidget *err = gtk_message_dialog_new(GTK_WINDOW(sub),
                    GTK_DIALOG_MODAL, GTK_MESSAGE_ERROR, GTK_BUTTONS_OK,
                    "Please enter a Name and valid numeric values:\n"
                    "lat ∈ [-90,90], lon ∈ [-180,180], tile > 0.");
                gtk_dialog_run(GTK_DIALOG(err));
                gtk_widget_destroy(err);
            } else {
                GError *err = NULL;
                if (points_interest_add_to_csv(NULL, name, type, km, lat, lon, &err)) {
                    /* Update the completion model with the new name */
                    GtkListStore *comp_model =
                        g_object_get_data(G_OBJECT(dlg), "poi_completion_model");
                    if (GTK_IS_LIST_STORE(comp_model)) {
                        GtkTreeIter it;
                        gtk_list_store_append(comp_model, &it);
                        gtk_list_store_set(comp_model, &it, 0, name, -1);
                    }
                    /* Reload POI polygons and refresh Tab 3 immediately */
                    lp_cleanup();
                    lp_init(POI_CSV_FILE, NULL);
                    POISelectionCtx *poi = g_object_get_data(G_OBJECT(dlg), "poi_ctx");
                    if (poi) on_poi_refresh_clicked(NULL, poi);
                    GtkWidget *okmsg = gtk_message_dialog_new(GTK_WINDOW(sub),
                        GTK_DIALOG_MODAL, GTK_MESSAGE_INFO, GTK_BUTTONS_OK,
                        "Added \"%s\" (%s) to Points_of_Interests.csv.", name, type);
                    gtk_dialog_run(GTK_DIALOG(okmsg));
                    gtk_widget_destroy(okmsg);
                } else {
                    GtkWidget *errmsg = gtk_message_dialog_new(GTK_WINDOW(sub),
                        GTK_DIALOG_MODAL, GTK_MESSAGE_ERROR, GTK_BUTTONS_OK,
                        "Failed to add POI: %s", err ? err->message : "unknown error");
                    gtk_dialog_run(GTK_DIALOG(errmsg));
                    gtk_widget_destroy(errmsg);
                    g_clear_error(&err);
                }
            }
        }
        gtk_widget_destroy(sub);
        return; /* keep main dialog open */
    }

    if (response == GTK_RESPONSE_ACCEPT) {
        SubwinSaveSpec spec = (SubwinSaveSpec){0};
        if (sub_window_ephemeris_run(GTK_WINDOW(dlg), &spec)) {
            POISelectionCtx *poi = g_object_get_data(G_OBJECT(dlg), "poi_ctx");
            if (!poi || !GTK_IS_TREE_VIEW(poi->treeview)) {
                g_warning("No Points of Interest table to export.");
            } else {
                POIColumns cols = {
                    .col_time  = POI_COL_TIME,
                    .col_lat   = POI_COL_LAT,
                    .col_lon   = POI_COL_LON,
                    .col_range = POI_COL_RANGE,
                    .col_dir   = POI_COL_DIR,
                    .col_name  = POI_COL_NAME,
                    .col_type  = POI_COL_TYPE
                };
                GError *err = NULL;
                if (sub_window_ephemeris_export_poi(poi->treeview, &spec, &cols, &err)) {
                    g_message("Saved POI to %s (%s)",
                              spec.filepath,
                              spec.format == SUBWIN_FORMAT_CSV ? "CSV" : "TXT");
                } else {
                    g_warning("Export failed: %s", err ? err->message : "unknown error");
                    g_clear_error(&err);
                }
            }
            sub_window_ephemeris_spec_free(&spec);
        }
        return; /* keep dialog open after save dialog closes */
    }
    if (response == GTK_RESPONSE_CLOSE || response == GTK_RESPONSE_DELETE_EVENT) {
         gtk_widget_destroy(GTK_WIDGET(dlg));
     }
}




// ──────────────────────────────────────────────────────────────
// TAB 2 — Country
// ──────────────────────────────────────────────────────────────
/* Runs in background; filters and reports progress */
/*
 * @brief Background worker thread function. Performs heavy computation off the GTK main loop.
 * @function country_worker
 * @thread Runs in a background GTask thread. Do NOT touch GTK here.
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param task GTask        *task
 * @param source_object gpointer      source_object
 * @param user_data gpointer      user_data
 * @param cancellable GCancellable *cancellable
 * @return (void)
 */
static void
country_worker(GTask        *task,
               gpointer      source_object,
               gpointer      user_data,
               GCancellable *cancellable)
{

    (void)source_object; 
    CountrySelectionCtx *ctx = user_data;
    GtkTreeModel *src = gtk_tree_view_get_model(ctx->tv_tab1);
    GList *pass = tool_list_from_model(src);
    guint done  = 0;

    /* Build a simple array of rows (no GTK in worker thread) */
    GPtrArray *rows = g_ptr_array_new_with_free_func((GDestroyNotify)zone_row_free);

    /* Get static lists once, not per-point */
    GList *all_polys     = tool_get_all_polygons();
    GList *all_countries = tool_get_all_countries();

    for (GList *l = pass; l; l = l->next) {
            /* abort quickly if the user started another run/closed dialog */
            if (g_cancellable_is_cancelled(cancellable)) {
                g_task_return_new_error(task, G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                        "Operation was cancelled");
                g_list_free_full(pass, (GDestroyNotify)g_free);
                g_ptr_array_unref(rows);
                return;
            }
        ToolEphemPoint *pt = l->data;
        gchar *hit_country = NULL;

        /* Find the first polygon that contains this point */
        GList *pp = all_polys;
        GList *cc = all_countries;
        for (; pp && cc; pp = pp->next, cc = cc->next) {
            GArray   *poly = pp->data;
            if (_point_in_poly((GeoPoint*)poly->data,
                               poly->len,
                               pt->lat, pt->lon))
            {
                hit_country = (gchar*)cc->data;
                break;
            }
        }
        /* Only include if:
         *  - user selected "Territory" (all land), OR
         *  - hit_country matches the selected country
         */
        if (hit_country &&
            (g_strcmp0(ctx->name, "Territory") == 0 ||
             g_strcmp0(hit_country, ctx->name) == 0)) {
            ZoneRow *r = g_new0(ZoneRow,1);
            r->time    = g_strdup(pt->time_str);
            r->lat     = pt->lat;
            r->lon     = pt->lon;
            r->country = g_strdup(hit_country);
            g_ptr_array_add(rows, r);
        }


        done++;
    }

    g_list_free_full(pass, (GDestroyNotify)g_free);
    /* hand off rows to main thread */
    g_task_return_pointer(task, rows, (GDestroyNotify)g_ptr_array_unref);
}


/*
 * @brief Idle callback that streams batched rows into the GtkListStore to keep UI responsive.
 * @function country_append_chunk_idle
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */

static gboolean country_append_chunk_idle(gpointer data)
{
    CountrySelectionCtx *ctx = data;
    if (!ctx || !GTK_IS_LIST_STORE(ctx->store)) return G_SOURCE_REMOVE;
    if (!ctx->pending_rows) return G_SOURCE_REMOVE;

    const guint CHUNK = 20000; /* territory lists are big; tune as needed */
    guint added = 0;
    for (; ctx->next_row < ctx->pending_rows->len && added < CHUNK; ++ctx->next_row, ++added) {
        ZoneRow *r = ctx->pending_rows->pdata[ctx->next_row];
        GtkTreeIter it;
        gtk_list_store_append(ctx->store, &it);
        gtk_list_store_set(ctx->store, &it,
            ZONE_COL_TIME,    r->time,
            ZONE_COL_LAT,     r->lat,
            ZONE_COL_LON,     r->lon,
            ZONE_COL_COUNTRY, r->country, -1);
    }

    /* update total counter as we go (cheap) */
    if (GTK_IS_LABEL(ctx->count_label)) {
        gchar *txt = g_strdup_printf("Total: %u", ctx->next_row);
        gtk_label_set_text(ctx->count_label, txt);
        g_free(txt);
    }

    if (ctx->next_row >= ctx->pending_rows->len) {
        /* all rows streamed — stop pulse/timer and finalize UI */
        if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }
        if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
        /* final elapsed seconds update */
        if (GTK_IS_LABEL(ctx->time_label)) {
            guint64 now = g_get_monotonic_time();
            guint   secs = (now - ctx->start_time) / G_USEC_PER_SEC;
            gchar  *txt  = g_strdup_printf("%us", secs);
            gtk_label_set_text(GTK_LABEL(ctx->time_label), txt);
            g_free(txt);
        }
        /* complete the bar at 100% */
        if (GTK_IS_PROGRESS_BAR(ctx->progress_bar)) {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(ctx->progress_bar), 1.0);
            gtk_progress_bar_set_text(GTK_PROGRESS_BAR(ctx->progress_bar), "100%");
        }
        /* re-enable entry now that table is fully rebuilt */
        safe_set_sensitive(GTK_WIDGET(ctx->entry), TRUE);
        safe_set_sensitive(ctx->button, TRUE);  /* re-enable together */
        /* reattach the model for one-shot layout & paint */
        if (ctx->model_detached && GTK_IS_TREE_VIEW(ctx->treeview)) {
            gtk_tree_view_set_model(GTK_TREE_VIEW(ctx->treeview), GTK_TREE_MODEL(ctx->store));
            ctx->model_detached = FALSE;
        }
        g_ptr_array_unref(ctx->pending_rows);
        ctx->pending_rows = NULL;
        ctx->idle_id = 0;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}


/* on_country_progress removed—using idle callbacks instead */

/* Called on main thread when worker finishes */
/* ─── Called on main thread when country_worker finishes ───────────────────── */
/*
 * @brief GTask "finished" handler executed on the GTK main thread.
 * @function on_country_done
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param source GObject      *source
 * @param res GAsyncResult *res
 * @param user_data gpointer      user_data
 * @return (void)
 */
static void
on_country_done(GObject      *source,
                GAsyncResult *res,
                gpointer      user_data){

    (void)source;
    CountrySelectionCtx *ctx = user_data;
    GError *error = NULL;

    GPtrArray *rows = g_task_propagate_pointer(G_TASK(res), &error);
    if (error) {
        if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
            /* expected: user cancelled / restarted */
            if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }
            if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
            safe_set_sensitive(GTK_WIDGET(ctx->entry), TRUE);
            safe_set_sensitive(ctx->button, TRUE);
        } else {
            g_warning("Country filter error: %s", error->message);
        }
        g_clear_error(&error);
        return;
    } else {
        /* create target model, DETACH view, then stream rows in idle */
        ctx->store = gtk_list_store_new(
            ZONE_N_COLS, G_TYPE_STRING, G_TYPE_DOUBLE, G_TYPE_DOUBLE, G_TYPE_STRING);
        gtk_tree_view_set_model(GTK_TREE_VIEW(ctx->treeview), NULL); /* detach for speed */
        ctx->model_detached = TRUE;
        ctx->pending_rows = rows;
        ctx->next_row     = 0;
        if (ctx->idle_id) g_source_remove(ctx->idle_id);
        ctx->idle_id = g_idle_add_full(G_PRIORITY_LOW, country_append_chunk_idle, ctx, NULL);
    }
    /* do NOT stop pulse/timer or re-enable here — we do that after the last chunk */
}

/*
 * @brief Callback / helper function.
 * @function cancel_country_if_running
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param w GtkWidget *w
 * @param user_data gpointer user_data
 * @return (void)
 */
static void cancel_country_if_running(GtkWidget *w, gpointer user_data) {
    (void)w;
    CountrySelectionCtx *ctx = user_data;
    if (ctx && ctx->cancel) g_cancellable_cancel(ctx->cancel);
}



// ──────────────────────────────────────────────────────────────
// TAB 3 — POI
// ──────────────────────────────────────────────────────────────

/* Generic handler for “match-selected” on any entry+completion.
   After GTK copies the selected text into the entry,
   we emit “activate” on that entry to trigger the real lookup. */

static void on_poi_refresh_clicked(GtkButton *button, gpointer user_data);
/*
 * @brief Callback / helper function.
 * @function cancel_poi_if_running
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param w GtkWidget *w
 * @param user_data gpointer user_data
 * @return (void)
 */

static void cancel_poi_if_running(GtkWidget *w, gpointer user_data) {
    (void)w;
    POISelectionCtx *ctx = user_data;
    if (ctx && ctx->cancel) g_cancellable_cancel(ctx->cancel);
}



/* Idle callback to update the POI progress bar fraction and “XX%” text */
typedef struct { POISelectionCtx *ctx; double fraction; } POIProgressUpdate;
/*
 * @brief Idle callback to update the GtkProgressBar fraction/text from a worker thread.
 * @function poi_progress_idle
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */

static gboolean G_GNUC_UNUSED
poi_progress_idle(gpointer data){

    POIProgressUpdate *up = data;

    /* if the bar has been destroyed (dialog closed), stop immediately */
    if (!GTK_IS_PROGRESS_BAR(up->ctx->progress_bar)) {
        g_free(up);
        return G_SOURCE_REMOVE;
    }
    gtk_progress_bar_set_fraction(
        GTK_PROGRESS_BAR(up->ctx->progress_bar), up->fraction);
    /* display “XX%” */
    gchar *pct = g_strdup_printf("%d%%", (int)(up->fraction * 100));
    gtk_progress_bar_set_text(GTK_PROGRESS_BAR(up->ctx->progress_bar), pct);
    g_free(pct);
    return G_SOURCE_REMOVE;
}

/* Background worker: builds a new ListStore and reports progress */
/*
 * @brief Background worker thread function. Performs heavy computation off the GTK main loop.
 * @function poi_worker
 * @thread Runs in a background GTask thread. Do NOT touch GTK here.
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param task GTask        *task
 * @param source_object gpointer      source_object
 * @param user_data gpointer      user_data
 * @param cancellable GCancellable *cancellable
 * @return (void)
 */

typedef struct {
    GList       *begin;     /* first ToolEphemPoint node for this slice */
    guint        count;     /* how many points in this slice */
    GList       *polys;     /* shared list (do not free) */
    GPtrArray   *bboxes;    /* shared bboxes (do not free) */
    gint         filter_idx;
    POISelectionCtx *ctx;   /* shared names/types (do not free) */
    GPtrArray   *out;       /* per-slice result rows */
    GCancellable *cancellable;
} POISlice;

static void poi_slice_worker(gpointer data, gpointer user_data) {
    POISlice *s = data; (void)user_data;
    GList *qfixed = (s->filter_idx >= 0) ? g_list_nth(s->polys, s->filter_idx) : NULL;
    gint   idxfixed = s->filter_idx;
    GList *node = s->begin;
    for (guint k=0; k < s->count && node; ++k, node=node->next) {
        if (g_cancellable_is_cancelled(s->cancellable)) return;
        ToolEphemPoint *t = node->data;
        gint idx = 0;
        for (GList *q = (qfixed ? qfixed : s->polys);
             q; q = (qfixed ? NULL : q->next), ++idx)
        {
            if (qfixed) idx = idxfixed;
            GArray *poly = q->data;
            BBox   *bb   = g_ptr_array_index(s->bboxes, idx);
            if (!bbox_contains(bb, t->lat, t->lon)) continue;
            if (lp_point_in_poly((LP_GeoPoint*)poly->data, poly->len, t->lat, t->lon)) {
                if (s->ctx->name[0] &&
                    g_strcmp0(s->ctx->name, g_ptr_array_index(s->ctx->names, idx)) != 0)
                    break;
                LP_GeoPoint ctr = lp_polygon_center(poly);
                LP_GeoPoint pt  = (LP_GeoPoint){ t->lat, t->lon };
                double dist = lp_compute_distance_km(&ctr, &pt);
                double brg  = lp_compute_bearing_deg(&ctr, &pt);
                gchar *dir  = format_bearing_text(brg);
                gchar *nm   = g_ptr_array_index(s->ctx->names, idx);
                gchar *tp   = g_ptr_array_index(s->ctx->types, idx);
                PoiRow *r   = g_new0(PoiRow,1);
                r->time     = g_strdup(t->time_str);
                r->lat      = t->lat; r->lon = t->lon;
                r->range_km = dist;   r->dir = dir;
                r->name     = g_strdup(nm); r->type = g_strdup(tp);
                g_ptr_array_add(s->out, r);
                break;
            }
        }
    }
}

static void
poi_worker(GTask        *task,
           gpointer      source_object,
           gpointer      user_data,
           GCancellable *cancellable)
{
            
    (void)source_object; 
    POISelectionCtx *ctx = user_data;
    /* 1) snapshot filter name */
    const gchar *poi = gtk_entry_get_text(ctx->entry);

    /* 2) grab ephemeris from Tab 1 */
    GtkTreeModel *m1 = gtk_tree_view_get_model(ctx->tab1_tree);
    GList       *tool_pts = tool_list_from_model(m1);
    guint total = g_list_length(tool_pts);

    /* 3) prepare temporary store (must list all 7 column types!) */
    /* reserve a bit to reduce reallocs; keep your free func */
    GPtrArray *rows = g_ptr_array_sized_new(total > 16384 ? 16384 : total);
    g_ptr_array_set_free_func(rows, (GDestroyNotify)poi_row_free);

    /* 4) load polygons once + precompute bboxes (massive prune) */
    GList *polys = lp_get_all_polygons();
    const guint npolys = g_list_length(polys);
    GPtrArray *bboxes = g_ptr_array_sized_new(npolys);
    g_ptr_array_set_free_func(bboxes, g_free);
    for (GList *q = polys; q; q = q->next) {
        BBox bb = bbox_from_poly((GArray*)q->data);
        g_ptr_array_add(bboxes, g_memdup2(&bb, sizeof(BBox)));
    }
    /* If a specific POI name is typed, resolve its index once */
    gint filter_idx = -1;
    if (poi && poi[0] != '\0' && ctx->names) {
        for (guint i = 0; i < ctx->names->len; ++i)
            if (g_strcmp0(poi, g_ptr_array_index(ctx->names, i)) == 0) { filter_idx = (gint)i; break; }
    }

    /* Split points into slices and process in parallel */
    const guint nthreads = CLAMP((guint)g_get_num_processors(), 2, 8);
    GThreadPool *pool = g_thread_pool_new(poi_slice_worker, NULL, nthreads, FALSE, NULL);
    GPtrArray   *slices = g_ptr_array_new_with_free_func(g_free);
    /* count nodes and slice size */
    guint npts = total, per = (npts + nthreads - 1) / nthreads;
    GList *start = tool_pts;
    for (guint i=0; i<nthreads && start; ++i) {
        POISlice *s = g_new0(POISlice,1);
        s->begin = start; s->count = MIN(per, npts - i*per);
        /* advance 'start' by s->count */
        for (guint k=0; k<s->count && start; ++k) start = start->next;
        s->polys = polys; s->bboxes = bboxes;
        s->filter_idx = filter_idx; s->ctx = ctx; s->cancellable = cancellable;
        s->out = g_ptr_array_new_with_free_func((GDestroyNotify)poi_row_free);
        g_ptr_array_add(slices, s);
        g_thread_pool_push(pool, s, NULL);
    }
    g_thread_pool_free(pool, FALSE, TRUE); /* wait for completion */
    /* Concatenate per-slice outputs into 'rows' (preserve approx order) */
    for (guint i=0; i<slices->len; ++i) {
        POISlice *s = g_ptr_array_index(slices, i);
        for (guint k=0; k<s->out->len; ++k)
            g_ptr_array_add(rows, g_ptr_array_index(s->out, k));
        g_ptr_array_free(s->out, FALSE);
    }
    g_ptr_array_free(slices, TRUE);

    /* cleanup only the list nodes; do NOT free the ToolEphemPoint pointers */
    g_list_free(tool_pts);
    g_ptr_array_free(bboxes, TRUE);

    /* hand back the new store */
    g_task_return_pointer(task, rows, (GDestroyNotify)g_ptr_array_unref);
}
/*
 * @brief Idle callback that streams batched rows into the GtkListStore to keep UI responsive.
 * @function poi_append_chunk_idle
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */

static gboolean poi_append_chunk_idle(gpointer data)
{
    POISelectionCtx *ctx = data;
    if (!ctx || !GTK_IS_LIST_STORE(ctx->store)) return FALSE;
    if (!ctx->pending_rows) return FALSE;

    const guint CHUNK = 20000;            /* tune for snappiness vs speed */
    guint added = 0;

    for (; ctx->next_row < ctx->pending_rows->len && added < CHUNK; ++ctx->next_row, ++added) {
        PoiRow *r = ctx->pending_rows->pdata[ctx->next_row];
        GtkTreeIter it;
        gtk_list_store_append(ctx->store, &it);
        gtk_list_store_set(ctx->store, &it,
            POI_COL_TIME,  r->time,
            POI_COL_LAT,   r->lat,
            POI_COL_LON,   r->lon,
            POI_COL_RANGE, r->range_km,
            POI_COL_DIR,   r->dir,
            POI_COL_NAME,  r->name,
            POI_COL_TYPE,  r->type, -1);
    }

    if (ctx->next_row >= ctx->pending_rows->len) {
        /* all rows streamed — stop pulse/timer and finalize UI */
        if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }
        if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
        if (GTK_IS_LABEL(ctx->time_label)) {
            guint64 now = g_get_monotonic_time();
            guint   secs = (now - ctx->start_time) / G_USEC_PER_SEC;
            gchar  *txt  = g_strdup_printf("%us", secs);
            gtk_label_set_text(GTK_LABEL(ctx->time_label), txt);
            g_free(txt);
        }
        if (GTK_IS_PROGRESS_BAR(ctx->progress_bar)) {
            gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(ctx->progress_bar), 1.0);
            gtk_progress_bar_set_text(GTK_PROGRESS_BAR(ctx->progress_bar), "100%");
        }
        safe_set_sensitive(GTK_WIDGET(ctx->entry), TRUE);
        safe_set_sensitive(ctx->button, TRUE);  /* re-enable together */
        /* reattach model for one-shot layout & paint */
        if (ctx->model_detached && GTK_IS_TREE_VIEW(ctx->treeview)) {
            gtk_tree_view_set_model(GTK_TREE_VIEW(ctx->treeview), GTK_TREE_MODEL(ctx->store));
            ctx->model_detached = FALSE;
        }
        g_ptr_array_unref(ctx->pending_rows);
        ctx->pending_rows = NULL;
        ctx->idle_id = 0;
        return G_SOURCE_REMOVE;
    }
    return G_SOURCE_CONTINUE;
}



/* Runs on the main thread when poi_worker finishes */
/*
 * @brief GTask "finished" handler executed on the GTK main thread.
 * @function on_poi_done
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param source GObject      *source
 * @param res GAsyncResult *res
 * @param user_data gpointer      user_data
 * @return (void)
 */
static void
on_poi_done(GObject      *source,
            GAsyncResult *res,
            gpointer      user_data){

    (void)source;
    POISelectionCtx *ctx = user_data;

    /* if the treeview is gone, don’t try to update */
    if (!GTK_IS_TREE_VIEW(ctx->treeview))
        return;  
    GError *error = NULL;

    GPtrArray *rows = g_task_propagate_pointer(G_TASK(res), &error);
    if (error) {
        if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
            if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }
            if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
            safe_set_sensitive(GTK_WIDGET(ctx->entry), TRUE);
        } else {
            g_warning("POI filter error: %s", error->message);
        }
        g_clear_error(&error);
        return;
    }
    /* Create the target model, but DETACH the view while we stream rows. */
    ctx->store = gtk_list_store_new(
        POI_N_COLS, G_TYPE_STRING, G_TYPE_DOUBLE, G_TYPE_DOUBLE,
        G_TYPE_DOUBLE, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING);
    gtk_tree_view_set_model(ctx->treeview, NULL);   /* <- detach for speed */
    ctx->model_detached = TRUE;
    ctx->pending_rows = rows;
    ctx->next_row     = 0;
    if (ctx->idle_id) g_source_remove(ctx->idle_id);
    ctx->idle_id = g_idle_add_full(G_PRIORITY_LOW, poi_append_chunk_idle, ctx, NULL);


}


/* ─── Handler for POI entry completion: trigger activate to filter ─── */
/*
 * @brief GtkEntryCompletion "match-selected" handler that triggers the corresponding action.
 * @function on_poi_match_selected
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param completion GtkEntryCompletion *completion
 * @param model GtkTreeModel       *model
 * @param iter GtkTreeIter        *iter
 * @param user_data gpointer            user_data
 * @return (gboolean)
 */
static gboolean
on_poi_match_selected(GtkEntryCompletion *completion,
                      GtkTreeModel       *model,
                      GtkTreeIter        *iter,
                      gpointer            user_data)
{
    (void)completion; (void)model; (void)iter;
    POISelectionCtx *ctx = user_data;
    /* emit “activate” on the entry to call on_poi_entry_activate() */
    g_signal_emit_by_name(ctx->entry, "activate");
    return FALSE;
}


/*      Creation of loading bars based on pulse define by the Tab3      */
/* ─── timeout callback to pulse the progress bar Tab 3 ────────────────────── */
/*
 * @brief GLib timeout to pulse an indeterminate GtkProgressBar while work is in progress.
 * @function poi_pulse_timeout
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */

static void stop_all_timeouts(GtkWidget *dialog, gpointer user_data) {
    (void)user_data;
    EphemUpdateCtx *e = g_object_get_data(G_OBJECT(dialog), "update_ctx");
    if (e) {
        if (e->pulse_source_id) { g_source_remove(e->pulse_source_id); e->pulse_source_id = 0; }
        if (e->timer_source_id) { g_source_remove(e->timer_source_id); e->timer_source_id = 0; }
    }
    CountrySelectionCtx *c = g_object_get_data(G_OBJECT(dialog), "country_ctx");
    if (c) {
        if (c->pulse_source_id) { g_source_remove(c->pulse_source_id); c->pulse_source_id = 0; }
        if (c->timer_source_id) { g_source_remove(c->timer_source_id); c->timer_source_id = 0; }
    }
    POISelectionCtx *p = g_object_get_data(G_OBJECT(dialog), "poi_ctx");
    if (p) {
        if (p->pulse_source_id) { g_source_remove(p->pulse_source_id); p->pulse_source_id = 0; }
        if (p->timer_source_id) { g_source_remove(p->timer_source_id); p->timer_source_id = 0; }
    }
}


static gboolean
poi_pulse_timeout(gpointer data)
{
    POISelectionCtx *ctx = data;
    if (!ctx || !GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        return G_SOURCE_REMOVE;
    gtk_progress_bar_pulse(GTK_PROGRESS_BAR(ctx->progress_bar));
    return G_SOURCE_CONTINUE;
}

/* ─── timeout callback to pulse the country progress bar ────────────────────── */
/*
 * @brief GLib timeout to pulse an indeterminate GtkProgressBar while work is in progress.
 * @function country_pulse_timeout
 * @thread Runs on the GTK main thread (GLib main loop).
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param data gpointer data
 * @return (gboolean)
 */
static gboolean
country_pulse_timeout(gpointer data)
{
    CountrySelectionCtx *ctx = data;
    if (!ctx || !GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        return G_SOURCE_REMOVE;
    gtk_progress_bar_pulse(GTK_PROGRESS_BAR(ctx->progress_bar));
    return G_SOURCE_CONTINUE;
}


/* ─── seconds‐counter timeout for Tab 1 ──────────────────────────────── */
/*
 * @brief GLib timeout that updates an "elapsed seconds" label every second.
 * @function update_ephem_timer
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param user_data gpointer user_data
 * @return (gboolean)
 */
static gboolean
update_ephem_timer(gpointer user_data)
{
    EphemUpdateCtx *ctx = user_data;
    if (!ctx || !GTK_IS_LABEL(ctx->time_label))
        return G_SOURCE_REMOVE;
    guint64 now = g_get_monotonic_time();
    guint secs = (now - ctx->start_time) / G_USEC_PER_SEC;
    gchar *txt = g_strdup_printf("%us", secs);
    gtk_label_set_text(GTK_LABEL(ctx->time_label), txt);

    g_free(txt);
    return G_SOURCE_CONTINUE;

}

/* ─── seconds‐counter timeout for Tab 2 ──────────────────────────────── */
/*
 * @brief GLib timeout that updates an "elapsed seconds" label every second.
 * @function update_country_timer
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param user_data gpointer user_data
 * @return (gboolean)
 */
static gboolean
update_country_timer(gpointer user_data)
{
    CountrySelectionCtx *ctx = user_data;
    if (!ctx || !GTK_IS_LABEL(ctx->time_label))
        return G_SOURCE_REMOVE;
    guint64 now = g_get_monotonic_time();
    guint secs = (now - ctx->start_time) / G_USEC_PER_SEC;
    gchar *txt = g_strdup_printf("%us", secs);
    gtk_label_set_text(GTK_LABEL(ctx->time_label), txt);
    g_free(txt);
    return G_SOURCE_CONTINUE;
}


/* ─── timeout callback to update elapsed‐seconds label ──────────────── */
/*
 * @brief GLib timeout that updates an "elapsed seconds" label every second.
 * @function update_poi_timer
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param user_data gpointer user_data
 * @return (gboolean)
 */
static gboolean
update_poi_timer(gpointer user_data)
{
    POISelectionCtx *ctx = user_data;
    if (!ctx || !GTK_IS_LABEL(ctx->time_label))
        return G_SOURCE_REMOVE;
    guint64 now = g_get_monotonic_time();
    guint   secs = (now - ctx->start_time) / G_USEC_PER_SEC;
    gchar  *txt  = g_strdup_printf("%us", secs);
    gtk_label_set_text(GTK_LABEL(ctx->time_label), txt);
    g_free(txt);
    return G_SOURCE_CONTINUE;
}

/* ───────────────────────────────────────────────────────────────────── */
/* wrapper for the “Refresh” button, so we pass the real entry */
/*
 * @brief Callback / helper function.
 * @function on_poi_refresh_clicked
 * @param button GtkButton   *button
 * @param user_data gpointer     user_data
 * @return (void)
 */
static void
on_poi_refresh_clicked(GtkButton   *button,
                       gpointer     user_data){
                        
                        
    (void)button;
    POISelectionCtx *ctx = user_data;

    if (ctx->cancel) {
        g_cancellable_cancel(ctx->cancel);
        g_clear_object(&ctx->cancel);
    }
    /* clear filter entry so Refresh restores all POIs */
    gtk_entry_set_text(GTK_ENTRY(ctx->entry), "");
    
    /* disable input during filter */
    safe_set_sensitive(GTK_WIDGET(ctx->entry), FALSE);

    /* reset bar */
    if (GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(ctx->progress_bar), 0.0);
    /* start the elapsed‐time counter */
    ctx->start_time      = g_get_monotonic_time();
    gtk_label_set_text(GTK_LABEL(ctx->time_label), "0s");
    ctx->timer_source_id = g_timeout_add_seconds(1,
                                                 update_poi_timer,
                                                 ctx);


    /* launch async filter; idle callbacks drive the bar */
    ctx->cancel = g_cancellable_new();
    GTask *task = g_task_new(NULL, ctx->cancel, on_poi_done, ctx);
    g_task_set_check_cancellable(task, TRUE);
    g_task_set_task_data(task, ctx, NULL);
    g_task_run_in_thread(task, poi_worker);
    g_object_unref(task);

    /* ── start the pulse immediately so the bar “moves” right away ── */
    ctx->pulse_source_id = g_timeout_add(100, poi_pulse_timeout, ctx);


}


/* when the user picks a completion in the Country entry */
/*
 * @brief GtkEntryCompletion "match-selected" handler that triggers the corresponding action.
 * @function on_country_match_selected
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param completion GtkEntryCompletion *completion
 * @param model GtkTreeModel       *model
 * @param iter GtkTreeIter        *iter
 * @param user_data gpointer            user_data
 * @return (gboolean)
 */
static gboolean
on_country_match_selected(GtkEntryCompletion *completion,
                          GtkTreeModel       *model,
                          GtkTreeIter        *iter,
                          gpointer            user_data){

    /* suppress unused-parameter warnings */
    (void)completion;
    (void)model;
    (void)iter;

    CountrySelectionCtx *ctx = user_data;
    g_signal_emit_by_name(ctx->entry, "activate");
    return FALSE;
}

/* Tab 3: Returns a newly-allocated string for bearing, e.g. "123.4°" */
/* 
 * @brief Formats a bearing in degrees into a short human-readable string.
 * @function format_bearing_text
 * @param bearing double bearing
 * @return (gchar *)
 */
static gchar *
format_bearing_text(double bearing)
{
    return g_strdup_printf("%.1f°", bearing);
}

/* Helper: format the “Latitude (°)” cell as text from a double. */
/*
 * @brief GtkTreeViewColumn cell-data function to format numeric cells.
 * @function lat_cell_data_func
 * @param column GtkTreeViewColumn *column
 * @param renderer GtkCellRenderer   *renderer
 * @param model GtkTreeModel      *model
 * @param iter GtkTreeIter       *iter
 * @param data gpointer           data
 * @return (void)
 */
static void
lat_cell_data_func(GtkTreeViewColumn *column,
                   GtkCellRenderer   *renderer,
                   GtkTreeModel      *model,
                   GtkTreeIter       *iter,
                   gpointer           data)
{
    double lat;
    gchar buf[32];
    gtk_tree_model_get(model, iter, 1, &lat, -1);  /* column 1 = COL_LAT */
    g_snprintf(buf, sizeof(buf), "% .5f", lat);
    g_object_set(renderer, "text", buf, NULL);
    (void)column;
    (void)data;
}

/* Helper: format the “Longitude (°)” cell as text from a double. */
/*
 * @brief GtkTreeViewColumn cell-data function to format numeric cells.
 * @function lon_cell_data_func
 * @param column GtkTreeViewColumn *column
 * @param renderer GtkCellRenderer   *renderer
 * @param model GtkTreeModel      *model
 * @param iter GtkTreeIter       *iter
 * @param data gpointer           data
 * @return (void)
 */
static void
lon_cell_data_func(GtkTreeViewColumn *column,
                   GtkCellRenderer   *renderer,
                   GtkTreeModel      *model,
                   GtkTreeIter       *iter,
                   gpointer           data)
{
    double lon;
    gchar buf[32];
    gtk_tree_model_get(model, iter, 2, &lon, -1);  /* column 2 = COL_LON */
    g_snprintf(buf, sizeof(buf), "% .5f", lon);
    g_object_set(renderer, "text", buf, NULL);
    (void)column;
    (void)data;
}
/*
 * @brief GtkEntry "activate" handler to launch the associated action.
 * @function on_poi_entry_activate
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param entry GtkEntry *entry
 * @param user_data gpointer user_data
 * @return (void)
 */

static void
on_poi_entry_activate(GtkEntry *entry, gpointer user_data){
    (void)entry;
    POISelectionCtx *ctx = user_data;
    /* Simply kick off the existing asynchronous worker: */
    on_poi_refresh_clicked(NULL, ctx);
}



/* ========================================================================== /
/ TAB 1 — Ephemeris /
/ ========================================================================== /

Collect the ground track.
Moves the freshly computed ephemeris buffer from the worker into the
GtkListStore using a low-priority idle handler (chunked insert to keep the UI
responsive). Updates the “Total” label, resets the progress bar, re-enables
user controls, and stops the pulse and seconds timer. If a POI is currently
selected, it re-issues the same POI query so Tab 3 reflects the new data.

Samples the orbit at fixed “step” seconds over a “duration”, using a private
copy of the satellite state to call Predict. For each sample, creates an
EphemPoint (JD, formatted timestamp, sub-satellite lat/lon) and appends it to
the thread-local buffer. Honors GCancellable, frees the previous run’s buffer,
and returns TRUE/FALSE via g_task_return_boolean().


now takes an extra step‐size argument */
void collect_groundtrack_points(sat_t *sat, qth_t *qth,
                               int n_orbits,
                                int step_sec);

/*
 * @brief GTask "finished" handler executed on the GTK main thread.
 * @function on_ephem_done
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param source GObject      *source
 * @param res GAsyncResult *res
 * @param user_data gpointer    user_data
 * @return (void)
 */
static void
on_ephem_done(GObject      *source,
              GAsyncResult *res,
              gpointer    user_data){

    (void)source;
    EphemUpdateCtx *ctx = user_data;
    gboolean ok = g_task_propagate_boolean(G_TASK(res), NULL);
    if (ok) {
        /* clear old rows; detach view for fast bulk insert; init counters */
        gtk_list_store_clear(ctx->store);
        if (GTK_IS_TREE_VIEW(ctx->treeview)) {
            gtk_tree_view_set_model(ctx->treeview, NULL);   /* BIG speedup */
            ctx->model_detached = TRUE;
        }
        ctx->inserted_count = 0;
        ctx->append_ptr = ctx->buffer;
        if (ctx->idle_id) g_source_remove(ctx->idle_id);
        ctx->idle_id = g_idle_add_full(G_PRIORITY_LOW, ephem_append_chunk_idle, ctx, NULL);

        /* +++ UPDATE the total‐points label +++ */
        {
            gchar *txt = g_strdup_printf("Total: %u", ctx->buffer_count);
            gtk_label_set_text(ctx->count_label, txt);
            g_free(txt);
        }

    }

}

/*
 * @brief Background worker thread function. Performs heavy computation off the GTK main loop.
 * @function ephem_worker
 * @thread Runs in a background GTask thread. Do NOT touch GTK here.
 * @note Performance/UX critical: keeps UI responsive via chunked inserts, progress updates, and cancellability.
 * @param task GTask      *task
 * @param source_object gpointer    source_object
 * @param task_data gpointer    task_data
 * @param cancellable GCancellable *cancellable
 * @return (void)
 */

static void
ephem_worker(GTask      *task,
             gpointer    source_object,
             gpointer    task_data,
             GCancellable *cancellable){

    (void)source_object;
    (void)cancellable;
    EphemUpdateCtx *ctx = task_data;
    /* 1) Snapshot the spin-values on the main thread into locals */
    const int duration = MAX (1, ctx->duration_s);
    const int step     = MAX (1, ctx->step_sec);

    /* 2) Copy the satellite struct for thread-safe Predict calls */
    sat_t sat_copy = *ctx->sat;
    qth_t *qth     = ctx->qth;
    double jul0    = sat_copy.jul_utc;     /* <-- define before using it */

    const double end_jd  = jul0 + ((double)duration) / 86400.0;
    const double step_jd = ((double)step)     / 86400.0;

    /* clear *our* previous run’s buffer (never touch the global!) */
    if (ctx->buffer) {
        g_slist_free_full(ctx->buffer, (GDestroyNotify)free_ephem_point);
    }
    ctx->buffer        = NULL;
    ctx->buffer_count  = 0;


    /* drive by time, not loop count → always stops at end_jd */
    for (double t = jul0; t <= end_jd + 1e-9; t += step_jd) {
        if (g_cancellable_is_cancelled(cancellable)) {
            g_task_return_boolean(task, FALSE);
            return;
        }
        /* advance in our private copy */
        const double jul_pt = t;
        predict_calc(&sat_copy, qth, jul_pt);

        EphemPoint *p = g_new0(EphemPoint,1);
        p->epoch_jd = jul_pt;
        /* format Timestamp = “YYYY/MM/DD HH:MM:SS” */
        {
            int Y,Mo,D,h,m,s;
            jd_to_gregorian(jul_pt, &Y,&Mo,&D,&h,&m,&s);
            p->time_str = g_strdup_printf(
                "%04d/%02d/%02d %02d:%02d:%02d",
                Y,Mo,D,h,m,s
            );
        }
       /* extract subsatellite coords from our private copy */
        predict_get_subsatellite_coords(&sat_copy,
                                       &p->lat_deg,
                                       &p->lon_deg);

        /* use prepend to avoid O(n^2) on Windows builds, reverse later */
        ctx->buffer = g_slist_prepend(ctx->buffer, p);
        ctx->buffer_count++;


    }

    /* reverse once so buffer is in chronological order */
    ctx->buffer = g_slist_reverse(ctx->buffer);
    
    // signal completion
    g_task_return_boolean(task, TRUE);
}



/* Helper: load polygons, filter by ctx->name, and rebuild Tab 2’s treeview */

/* this old, synchronous filter is now superseded—mark it unused */
/*
 * @brief Callback / helper function.
 * @function __attribute__
 * @param ctx (unused))
ephem_update_with_country(CountrySelectionCtx *ctx
 * @return (void)
 */
static void __attribute__((unused))
ephem_update_with_country(CountrySelectionCtx *ctx){


    /* a) Load the master lists (do NOT free these) */
    GList *all_polys     = tool_get_all_polygons();
    GList *all_countries = tool_get_all_countries();

    /* b) Grab raw ephemeris from Tab 1 */
    GtkTreeModel *src = gtk_tree_view_get_model(ctx->tv_tab1);
    GList        *pass = tool_list_from_model(src);

    /* c) Territory = special “all lands” case */
    if (g_strcmp0(ctx->name, "Territory") == 0) {
        GtkListStore *store = gtk_list_store_new(
            ZONE_N_COLS,
            G_TYPE_STRING, G_TYPE_DOUBLE,
            G_TYPE_DOUBLE, G_TYPE_STRING
        );
        GtkTreeIter  tree_iter;
        for (GList *l = pass; l; l = l->next) {
            ToolEphemPoint *pt = l->data;
            for (GList *pp = all_polys, *cc = all_countries;
                 pp && cc;
                 pp = pp->next, cc = cc->next)
            {
                GArray   *poly = pp->data;
                GeoPoint *pts  = (GeoPoint*)poly->data;
                guint     n    = poly->len;
                if (_point_in_poly(pts, n, pt->lat, pt->lon)) {
                    gtk_list_store_append(store, &tree_iter);
                    gtk_list_store_set(store, &tree_iter,
                        ZONE_COL_TIME,    pt->time_str,
                        ZONE_COL_LAT,     pt->lat,
                        ZONE_COL_LON,     pt->lon,
                        ZONE_COL_COUNTRY, (gchar*)cc->data,
                        -1);
                    break;
                }
            }
        }
        gtk_tree_view_set_model(GTK_TREE_VIEW(ctx->treeview),
                                GTK_TREE_MODEL(store));
        g_object_unref(store);

        /* cleanup */
        for (GList *l = pass; l; l = l->next) {
            ToolEphemPoint *pt = l->data;
            g_free(pt->time_str);
            g_free(pt);
        }
        g_list_free(pass);
        return;
    }

    /* d) Country = filter just that country’s polys */
    GList *my_polys = NULL;
    for (GList *pp = all_polys, *cc = all_countries;
         pp && cc;
         pp = pp->next, cc = cc->next)
    {
        if (g_strcmp0((gchar*)cc->data, ctx->name) == 0)
            my_polys = g_list_append(my_polys, pp->data);
    }

    GtkListStore *store = gtk_list_store_new(
        ZONE_N_COLS,
        G_TYPE_STRING, G_TYPE_DOUBLE,
        G_TYPE_DOUBLE, G_TYPE_STRING
    );
    GtkTreeIter  tree_iter;
    for (GList *l = pass; l; l = l->next) {
        ToolEphemPoint *pt = l->data;
        for (GList *pp = my_polys; pp; pp = pp->next) {
            GArray   *poly = pp->data;
            GeoPoint *pts  = (GeoPoint*)poly->data;
            guint     n    = poly->len;
            if (_point_in_poly(pts, n, pt->lat, pt->lon)) {
                gtk_list_store_append(store, &tree_iter);
                gtk_list_store_set(store, &tree_iter,
                    ZONE_COL_TIME,    pt->time_str,
                    ZONE_COL_LAT,     pt->lat,
                    ZONE_COL_LON,     pt->lon,
                    ZONE_COL_COUNTRY, ctx->name,
                    -1);
                break;
            }
        }
    }
    gtk_tree_view_set_model(GTK_TREE_VIEW(ctx->treeview),
                            GTK_TREE_MODEL(store));
    g_object_unref(store);

    /* cleanup */
    for (GList *l = pass; l; l = l->next) {
        ToolEphemPoint *pt = l->data;
        g_free(pt->time_str);
        g_free(pt);
    }
    g_list_free(pass);
    g_list_free(my_polys);
}

/* ========================================================================== /
/ TAB 2 — Territory / Countries  /
/ ========================================================================== /
The next three GTK main-thread callbacks coordinate the asynchronous country
filtering pipeline. They validate the user’s selection, serialize work using
a GCancellable, disable inputs during processing, reset the progress bar,
and then launch country_worker (heavy lifting off the main loop). 
Model updates and progress text are driven by idle/timeout callbacks and 
finalized in on_country_done.

on_country_changed(GtkEntry*, gpointer)
Triggered on every edit. When the entry text exactly matches a known
country, it copies the selection into the context, disables inputs, resets
the progress bar, creates a fresh GCancellable, starts pulse + timer, and
spawns country_worker via GTask. Otherwise, it does nothing.

on_country_entry_activate(GtkEntry*, gpointer)
Invoked when the user presses Enter in the entry. Validates the text
against the master country list. If valid, it cancels any in-flight task
(single-flight), disables inputs, hard-resets the bar to 0% (including
label), starts pulse + timer, and launches country_worker.

on_country_row_activated(GtkTreeView*, GtkTreePath*, GtkTreeViewColumn*, gpointer)
Called when the user picks a country from the popover list. Extracts the
selected country name, closes the popover, disables inputs, cancels any
running task, and launches country_worker. UI/model updates occur via
idle/timeout callbacks and are finalized in on_country_done. */

/* Called on every edit; only updates when the text is a full, known country */
/*
 * @brief Callback / helper function.
 * @function on_country_changed
 * @param entry GtkEntry *entry
 * @param user_data gpointer  user_data
 * @return (void)
 */
static void
on_country_changed(GtkEntry *entry,
                   gpointer  user_data){

    CountrySelectionCtx *ctx = user_data;
    const gchar        *text = gtk_entry_get_text(entry);

    /* Only fire when it’s an exact match */
    GList *all = tool_get_all_countries();
    for (GList *l = all; l; l = l->next) {
        if (g_strcmp0(text, (char*)l->data) == 0) {
            /* copy & refresh */
            g_strlcpy(ctx->name, text, sizeof(ctx->name));
            /* disable inputs */
            safe_set_sensitive(ctx->button, FALSE);
            safe_set_sensitive(GTK_WIDGET(ctx->entry), FALSE);
            /* reset bar to zero */
            gtk_progress_bar_set_fraction(ctx->progress_bar, 0.0);
            /* launch async filter; we’ll only pulse */
            ctx->cancel = g_cancellable_new();
            GTask *task = g_task_new(NULL, ctx->cancel, on_country_done, ctx);
            g_task_set_check_cancellable(task, TRUE);
            g_task_set_task_data(task, ctx, NULL);
            g_task_run_in_thread(task, country_worker);
            /* +++ start elapsed-counter +++ */
            ctx->start_time      = g_get_monotonic_time();
            gtk_label_set_text(ctx->time_label, "0s");
            ctx->timer_source_id = g_timeout_add_seconds(1,
                                                         update_country_timer,
                                                         ctx);
            /* start pulsing */
            ctx->pulse_source_id = g_timeout_add(100,
                                        country_pulse_timeout,
                                        ctx);
            g_object_unref(task);
            break;
        }
    }
}


/*
 * @brief GtkEntry "activate" handler to launch the associated action.
 * @function on_country_entry_activate
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param entry GtkEntry       *entry
 * @param user_data gpointer        user_data
 * @return (void)
 */

static void
on_country_entry_activate(GtkEntry       *entry,
                          gpointer        user_data){
                            

    CountrySelectionCtx *ctx = user_data;
    const gchar *text = gtk_entry_get_text(entry);

    /* Check against master list */
    GList *all = tool_get_all_countries();
    gboolean found = FALSE;
    for (GList *l = all; l; l = l->next) {
        if (g_strcmp0(text, (char*)l->data) == 0) {
            found = TRUE;
            break;
        }
    }

    if (!found) {
        /* silently ignore invalid country */
        return;
    }

    /* Copy selection into ctx and update the ephemeris‐in-zone view */
    g_strlcpy(ctx->name, text, sizeof(ctx->name));


    /* cancel any previous run, disable inputs and reset bar */
    if (ctx->cancel) { g_cancellable_cancel(ctx->cancel); g_clear_object(&ctx->cancel); }
    safe_set_sensitive(ctx->button, FALSE);
    safe_set_sensitive(GTK_WIDGET(ctx->entry), FALSE);

    /* reset progress bar to exactly 0% */
    if (GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        gtk_progress_bar_set_fraction(ctx->progress_bar, 0.0);
    gtk_progress_bar_set_text(
        GTK_PROGRESS_BAR(ctx->progress_bar), "0%");
    
    /* launch async filter; idle callbacks drive the bar */
    ctx->cancel = g_cancellable_new();
    GTask *task = g_task_new(NULL, ctx->cancel, on_country_done, ctx);
    g_task_set_check_cancellable(task, TRUE);
    g_task_set_task_data(task, ctx, NULL);
    g_task_run_in_thread(task, country_worker);

    /* +++ start elapsed-seconds counter +++ */
    ctx->start_time      = g_get_monotonic_time();
    gtk_label_set_text(GTK_LABEL(ctx->time_label), "0s");
    ctx->timer_source_id = g_timeout_add_seconds(1,
                                                 update_country_timer,
                                                 ctx);
    /* start pulsing */
    ctx->pulse_source_id = g_timeout_add(100,
                                        country_pulse_timeout,
                                        ctx);
    g_object_unref(task);
    return;

}


// ----------------------------------------------------------------------------
// Callback for when the user picks a country from the pop-over list
/*
 * @brief Callback / helper function.
 * @function on_country_row_activated
 * @thread Runs on the GTK main thread (GLib main loop).
 * @param tree GtkTreeView       *tree
 * @param path GtkTreePath       *path
 * @param G_GNUC_UNUSED GtkTreeViewColumn *col G_GNUC_UNUSED
 * @param user_data gpointer           user_data
 * @return (void)
 */
static void
on_country_row_activated(GtkTreeView       *tree,
                         GtkTreePath       *path,
                         GtkTreeViewColumn *col G_GNUC_UNUSED,
                         gpointer           user_data){
                            

    CountrySelectionCtx *ctx = user_data;
    gchar *sel;

    /* 1) Get selected country name */
    GtkTreeIter iter;
    GtkTreeModel *model = gtk_tree_view_get_model(tree);
    if (!gtk_tree_model_get_iter(model, &iter, path)) return;
    gtk_tree_model_get(model, &iter, COL_COUNTRY, &sel, -1);
    if (!sel) return;
    g_strlcpy(ctx->name, sel, sizeof(ctx->name));
    g_free(sel);

    /* 2) Tear down the pop-over */
    GtkWidget *dialog = gtk_widget_get_toplevel(GTK_WIDGET(ctx->button));
    GtkWidget *popover = g_object_get_data(G_OBJECT(dialog),
                                           "active-country-popover");
    if (popover) {
        gtk_widget_destroy(popover);
        g_object_set_data(G_OBJECT(dialog),
                          "active-country-popover", NULL);
    }

    /* 3) Delegate all rebuild logic to the new helper */
    /* disable UI */
    safe_set_sensitive(ctx->button, FALSE);
    safe_set_sensitive(GTK_WIDGET(ctx->entry), FALSE);

    /* run the country filter in background; idle callbacks drive the bar */
    if (ctx->cancel) { g_cancellable_cancel(ctx->cancel); g_clear_object(&ctx->cancel); }
    ctx->cancel = g_cancellable_new();
    GTask *task = g_task_new(NULL, ctx->cancel, on_country_done, ctx);
    g_task_set_check_cancellable(task, TRUE);
    g_task_set_task_data(task, ctx, NULL);
    g_task_run_in_thread(task, country_worker);
    /* reset bar and start pulse + 1 Hz timer */
    if (GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(ctx->progress_bar), 0.0);
    ctx->start_time      = g_get_monotonic_time();
    gtk_label_set_text(GTK_LABEL(ctx->time_label), "0s");
    ctx->timer_source_id = g_timeout_add_seconds(1, update_country_timer, ctx);
    ctx->pulse_source_id = g_timeout_add(100, country_pulse_timeout, ctx);
    g_object_unref(task);

}


/*
 * @brief Callback / helper function.
 * @function on_dialog_destroy_cleanup
 * @param dialog GtkWidget *dialog
 * @param user_data gpointer user_data
 * @return (void)
 */
static void on_dialog_destroy_cleanup(GtkWidget *dialog, gpointer user_data) {
    (void)user_data;
    /* Kill any live popover */
    GtkWidget *popover = g_object_get_data(G_OBJECT(dialog), "active-country-popover");
    if (popover && GTK_IS_WIDGET(popover)) {
        g_object_set_data(G_OBJECT(dialog), "active-country-popover", NULL);
        gtk_widget_destroy(popover);
    }

    /* --- Tab 1: Ephemeris --- */
    EphemUpdateCtx *e = g_object_get_data(G_OBJECT(dialog), "update_ctx");
    if (e) {
        if (e->idle_id)         { g_source_remove(e->idle_id);         e->idle_id = 0; }
        if (e->pulse_source_id) { g_source_remove(e->pulse_source_id); e->pulse_source_id = 0; }
        if (e->timer_source_id) { g_source_remove(e->timer_source_id); e->timer_source_id = 0; }
        /* Make late callbacks harmless by invalidating widget pointers */
        e->hours_spin   = NULL;
        e->step_spin    = NULL;
        e->progress_bar = NULL;
        e->treeview     = NULL;
        e->time_label   = NULL;
        e->count_label  = NULL;
    }

    /* --- Tab 2: Countries --- */
    CountrySelectionCtx *c = g_object_get_data(G_OBJECT(dialog), "country_ctx");
    if (c) {
        if (c->idle_id)         { g_source_remove(c->idle_id);         c->idle_id = 0; }
        if (c->pulse_source_id) { g_source_remove(c->pulse_source_id); c->pulse_source_id = 0; }
        if (c->timer_source_id) { g_source_remove(c->timer_source_id); c->timer_source_id = 0; }
        if (c->cancel)          { g_cancellable_cancel(c->cancel); g_clear_object(&c->cancel); }
        c->entry = NULL; c->button = NULL; c->treeview = NULL;
        c->progress_bar = NULL; c->count_label = NULL; c->time_label = NULL;
    }

    /* --- Tab 3: POIs --- */
    POISelectionCtx *p = g_object_get_data(G_OBJECT(dialog), "poi_ctx");
    if (p) {
        if (p->idle_id)         { g_source_remove(p->idle_id);         p->idle_id = 0; }
        if (p->pulse_source_id) { g_source_remove(p->pulse_source_id); p->pulse_source_id = 0; }
        if (p->timer_source_id) { g_source_remove(p->timer_source_id); p->timer_source_id = 0; }
        if (p->cancel)          { g_cancellable_cancel(p->cancel); g_clear_object(&p->cancel); }
        p->entry = NULL; p->treeview = NULL; p->progress_bar = NULL;
        p->time_label = NULL; /* POISelectionCtx has no count_label */
    }
}


// SIGNAL HANDLER MUST TAKE TWO PARAMS: the spin widget + user_data
/*
 * @brief Callback / helper function.
 * @function on_orbits_value_changed
 * @param spin GtkSpinButton *spin
 * @param user_data gpointer user_data
 * @return (void)
 */

static void
on_orbits_value_changed(GtkSpinButton *spin, gpointer user_data){

    EphemUpdateCtx *ctx = (EphemUpdateCtx*) user_data;
    (void)spin;  /* we ignore which spin triggered it */
    if (ctx->running) return;            /* ignore while one job is running */
    ctx->running = TRUE;

    /* ensure no stale sources are running */
    if (ctx->timer_source_id) { g_source_remove(ctx->timer_source_id); ctx->timer_source_id = 0; }
    if (ctx->pulse_source_id) { g_source_remove(ctx->pulse_source_id); ctx->pulse_source_id = 0; }

    // Reset progress bar + label
    if (GTK_IS_PROGRESS_BAR(ctx->progress_bar)) {
        gtk_progress_bar_set_fraction(ctx->progress_bar, 0.0);

    }

    // Start elapsed-seconds timer (1 Hz)
    ctx->start_time = g_get_monotonic_time();
    if (GTK_IS_LABEL(ctx->time_label)) gtk_label_set_text(ctx->time_label, "0s");
    ctx->timer_source_id = g_timeout_add_seconds(1, update_ephem_timer, ctx);  // uses your Tab1 helper

    // Start pulsing animation (~10 Hz)
    ctx->pulse_source_id = g_timeout_add(100, ephem_pulse_timeout, ctx);
    /* disable controls while computing */
    safe_set_sensitive(GTK_WIDGET(ctx->hours_spin), FALSE);
    safe_set_sensitive(GTK_WIDGET(ctx->step_spin),  FALSE);

    /* cancel previous run if any */
    static GCancellable *ephem_cancel = NULL;
    if (ephem_cancel) { g_cancellable_cancel(ephem_cancel); g_clear_object(&ephem_cancel); }
    /* regenerate ephemeris buffer in a background task */

    /* ── Read the two spin-buttons *right here* on the main thread ── */
    {
      int hours = gtk_spin_button_get_value_as_int(ctx->hours_spin);
      ctx->duration_s = hours * 3600;
      ctx->step_sec   = gtk_spin_button_get_value_as_int(ctx->step_spin);
    }

    /* now spawn the background job using those stored ints */

    ephem_cancel = g_cancellable_new();
    GTask *task = g_task_new(NULL, ephem_cancel, on_ephem_done, ctx);
    g_task_set_check_cancellable(task, TRUE);
    g_task_set_task_data(task, ctx, NULL);

    g_task_run_in_thread(task, ephem_worker);
    g_object_unref(task);

}


/*
 * @brief Callback / helper function.
 * @function on_territory_clicked
 * @param button GtkButton *button
 * @param user_data gpointer user_data
 * @return (void)
 */

static void
on_territory_clicked(GtkButton *button, gpointer user_data){

    (void)button;  /* suppress unused‐parameter warning */
    CountrySelectionCtx *ctx = user_data;
    /* set to “Territory” */
    g_strlcpy(ctx->name, "Territory", sizeof(ctx->name));
    /* disable inputs */
    safe_set_sensitive(ctx->button, FALSE);
    safe_set_sensitive(GTK_WIDGET(ctx->entry), FALSE);
    /* reset and run the async filter */
    if (GTK_IS_PROGRESS_BAR(ctx->progress_bar))
        gtk_progress_bar_set_fraction(ctx->progress_bar, 0.0);
    ctx->cancel = g_cancellable_new();
    GTask *task = g_task_new(NULL, ctx->cancel, on_country_done, ctx);
    g_task_set_check_cancellable(task, TRUE);
    g_task_set_task_data(task, ctx, NULL);
    g_task_run_in_thread(task, country_worker);

    /* +++ start elapsed-seconds counter +++ */
    ctx->start_time      = g_get_monotonic_time();
    gtk_label_set_text(GTK_LABEL(ctx->time_label), "0s");
    ctx->timer_source_id = g_timeout_add_seconds(1,
                                                 update_country_timer,
                                                 ctx);
    /* start pulsing */
    ctx->pulse_source_id = g_timeout_add(100,
                                        country_pulse_timeout,
                                        ctx);
    g_object_unref(task);
}

// Handler function for button click — dynamique et robuste
/*
 * @brief Callback / helper function.
 * @function on_select_clicked
 * @param button GtkButton *button
 * @param user_data gpointer user_data
 * @return (void)
 */
void on_select_clicked(GtkButton *button, gpointer user_data) {

    CountrySelectionCtx *ctx = (CountrySelectionCtx *)user_data;
    // Création dynamique du popover
    GtkWidget *popover = gtk_popover_new(GTK_WIDGET(button));
    g_object_set_data(G_OBJECT(gtk_widget_get_toplevel(GTK_WIDGET(ctx->button))),
                  "active-country-popover", popover);
    gtk_popover_set_relative_to(GTK_POPOVER(popover), GTK_WIDGET(button));
    gtk_popover_set_position(GTK_POPOVER(popover), GTK_POS_BOTTOM);

    // ScrolledWindow
    GtkWidget *sw = gtk_scrolled_window_new(NULL, NULL);
    gtk_widget_set_size_request(sw, 180, 200);
    gtk_container_add(GTK_CONTAINER(popover), sw);

    // Store + TreeView
    GtkListStore *country_store = gtk_list_store_new(N_COUNTRY_COLS, G_TYPE_STRING);
    populate_country_liststore(country_store);

    GtkWidget *country_tv = gtk_tree_view_new_with_model(GTK_TREE_MODEL(country_store));
    gtk_tree_view_insert_column_with_attributes(
        GTK_TREE_VIEW(country_tv), -1,
        "Countries/Zone",
        gtk_cell_renderer_text_new(),
        "text", COL_COUNTRY,
        NULL);
    gtk_container_add(GTK_CONTAINER(sw), country_tv);
    g_object_unref(country_store);

    gtk_widget_show_all(popover);

    // Fermer la popover ET la détruire après sélection
    g_signal_connect(country_tv, "row-activated", G_CALLBACK(on_country_row_activated), ctx);

    

    gtk_popover_popup(GTK_POPOVER(popover));
}


/*
=================================================================================
CORE ENTRYPOINT — BUILDS AND ORCHESTRATES THE “EPHEMERIS DATA” DIALOG Main Window
=================================================================================

This function is the heart of the feature. It creates the three-tab dialog,
wires every control and signal, and defines the concurrency model that keeps
the UI responsive while heavy computations run in the background. It owns
the cross-tab contracts (who reads what, who refreshes whom), installs all
cleanup hooks, and sets the UX policy (disable inputs while running, pulse
progress bars, show elapsed time, defer “Save” until POI data exists).


TAB 1 — EPHEMERIS (DATA SOURCE)
Builds the primary data source for the dialog. It constructs the Time/Lat/Lon
list model and its TreeView, adds “Hours” and “Step (s)” spin controls, and
places a progress bar with an elapsed-seconds label directly on the page.

An EphemUpdateCtx captures widgets and parameters; both spins are connected
to a single value-changed handler that launches ephem_worker via GTask. The
worker samples the orbit and fills a thread-local buffer; results are streamed
back into the model through a low-priority idle callback (chunked inserts) to
avoid blocking the main loop. When an update completes, the function resets
the progress/pulse, re-enables inputs, updates the “Total” label, and—if a
POI is selected—auto-refreshes Tab 3 so it reflects the new ephemerides.


TAB 2 — TERRITORY / COUNTRIES (GEOSPATIAL FILTER OVER TAB 1)
Provides a country selector (button + entry with completion) and a dedicated
TreeView that displays only the ephemeris rows falling on land for the chosen
country (or “Territory” = any land). A CountrySelectionCtx binds the widgets,
progress pulse bar, and elapsed timer. Edits, Enter presses, or row activation
validate the selection, cancel any in-flight job (single-flight via
GCancellable), disable inputs, and launch country_worker. 

Model updates and progress text are driven from idle/timeout callbacks; 
formatted Lat/Lon cell-data functions ensure consistent numeric rendering.
Cleanup and cancel happen automatically when the dialog is destroyed.


TAB 3 — POINTS OF INTEREST (SEMANTIC FILTER WITH RANGE/BEARING)
Initializes the POI index from CSV, builds an entry with completion and a
“Refresh” action, and prepares a seven-column model (Time, Lat, Lon, Range,
Direction, Name, Type). A POISelectionCtx keeps the entry, progress pulse,
elapsed label, and table together. Actions (activation, completion, refresh)
trigger the background POI computation; results stream in via idle callbacks.

The dialog keeps “Save” hidden until this tab is active to prevent exporting
empty or irrelevant data. Tab 3 is also linked back to Tab 1’s updater so
POIs automatically recompute after any ephemeris change.


CONCURRENCY AND UX CONTRACT
All heavy work runs in GTask threads; the GTK main thread only builds UI,
connects signals, and schedules chunked model updates via idle/timeout. Each
tab manages its own GCancellable (fast aborts and single-flight). Progress is
shown with pulse bars and a live elapsed-seconds label. Destroy handlers tear
down background tasks and temporary indexes deterministically to avoid leaks
and races (e.g., polygon/tiles cleanup and per-tab cancellation).


INTEGRATION POINTS AND EXTENSIBILITY
Context structs (EphemUpdateCtx, CountrySelectionCtx, POISelectionCtx) are
stored on the dialog with g_object_set_data for easy cross-tab access. New
columns, filters, or export actions should follow the same pattern: collect
inputs on the main thread, compute in a worker, and stream results via idle
callbacks. This central builder is where additional tabs or data flows are
registered and coordinated.
*/

static void
on_show_ephemeris_activate(GtkMenuItem *menuitem, gpointer user_data)

{
    ShowEphemCtx *ctx = user_data;
    GtkSatMap    *satmap = ctx->satmap;
    sat_t        *sat    = ctx->sat;
    qth_t        *qth    = ctx->qth;

    g_signal_handlers_disconnect_by_data(menuitem, ctx);
    g_free(ctx);

    if (g_ephem_buffer_count == 0) {
        GtkWindow *parent = GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(menuitem)));
        GtkWidget *warn = gtk_message_dialog_new(
            parent,
            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
            GTK_MESSAGE_WARNING,
            GTK_BUTTONS_OK,
            "No ephemeris data available.\n"
            "Please generate a ground track first, then try again.");
        gtk_window_set_title(GTK_WINDOW(warn), "No Data");
        gtk_dialog_run(GTK_DIALOG(warn));
        gtk_widget_destroy(warn);
        return;
    }

    GtkWindow *parent = GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(menuitem)));
    GtkWidget *dialog = gtk_dialog_new_with_buttons(
        "Ephemeris Data", parent,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        "_Add",  1001,
        "_Save", GTK_RESPONSE_ACCEPT,
        "_Close", GTK_RESPONSE_CLOSE,
        NULL);
    
    g_signal_connect(dialog, "destroy", G_CALLBACK(on_dialog_destroy_cleanup), NULL);
    /* +++ free loaded polygons when popup closes +++ */
    g_signal_connect(dialog, "destroy", G_CALLBACK(lp_cleanup), NULL);
    g_signal_connect(dialog, "destroy", G_CALLBACK(stop_all_timeouts), NULL);


    gtk_window_set_default_size(GTK_WINDOW(dialog), 1200, 600); /* width, height of the window*/

    /* hard-code default to 7 days */
    int default_hours = 7*24;
    if (default_hours < 1) default_hours = 1;

    /* assume default step_sec = cfg or 30 s;
    * @brief Callback / helper function.
    * @function on_orbits_value_changed
    * @param (!lp_init(POI_CSV_FILE ) */
    
    /* initialize our tile filter (once per popup) 
    if (!lp_init(POI_CSV_FILE
    * @param NULL) NULL)
    * @return (actual collection is done by)
    
    actual collection is done by on_orbits_value_changed() */   
    
    /* initialize our tile filter (once per popup) */
    if (!lp_init(POI_CSV_FILE, NULL)) {
        g_warning("Logic_Point: failed to load tiles CSV '%s'", POI_CSV_FILE);
    }


    /* --- build notebook --- */
    GtkWidget *content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    GtkWidget *notebook     = gtk_notebook_new();
    gtk_box_pack_start(GTK_BOX(content_area), notebook, TRUE, TRUE, 0);

    /* ─── Tab 1: Ephemeris ─── (add ORBITS + STEP selectors) */
    GtkWidget *hbox_orbits = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(hbox_orbits), gtk_label_new("Hours:"), FALSE, FALSE, 0);
    GtkAdjustment *hour_adj = gtk_adjustment_new(
            (double)default_hours,  /* value */
            1.0,   /* min 1 h */
            1000,  /* max 999 h */
            1.0,   /* step 1 h */
            1.0,   /* page 1 h */
            0.0);  /* page‐overflow */
    GtkWidget *spin_hours = gtk_spin_button_new(hour_adj, 1.0, 0);
    gtk_box_pack_start(GTK_BOX(hbox_orbits), spin_hours, FALSE, FALSE, 0);


    /* +++ NEW: step‐size selector +++ */
    gtk_box_pack_start(GTK_BOX(hbox_orbits), gtk_label_new(" Step (s):"), FALSE, FALSE, 0);
    GtkAdjustment *step_adj = gtk_adjustment_new(
            30.0,    /* default 30 s */
            1.0,    /* min 1 s   */
        3600.0,    /* max 1 h   */
            1.0,    /* step 1 s  */
            10.0,    /* page 10 s */
            0.0     /* page‐overflow */
        );
    GtkWidget *spin_step = gtk_spin_button_new(step_adj, 1.0, 0);
    gtk_box_pack_start(GTK_BOX(hbox_orbits), spin_step, FALSE, FALSE, 0);


    GtkWidget *scrolled = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled),
                                   GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
    gtk_widget_set_vexpand(scrolled, TRUE);
    gtk_widget_set_hexpand(scrolled, TRUE);

    GtkWidget *page1 = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_box_pack_start(GTK_BOX(page1), hbox_orbits, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(page1), scrolled, TRUE, TRUE, 0);

    /* Build the ListStore for Tab 1 (use global COL_* and N_COLS) */
    GtkListStore *store = gtk_list_store_new(N_COLS,
                                             G_TYPE_STRING,
                                             G_TYPE_DOUBLE,
                                             G_TYPE_DOUBLE);
    /* Fill it from your g_ephem_buffer (already “now → next → …”) */
    for (GSList *l = g_ephem_buffer; l; l = l->next) {
        EphemPoint *pp = (EphemPoint*)l->data;
        GtkTreeIter iter;
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter,
                           COL_TIME, pp->time_str,
                           COL_LAT,  pp->lat_deg,
                           COL_LON,  pp->lon_deg,
                           -1);
    }


    /* Create the TreeView */
    GtkWidget *tv_ephem = gtk_tree_view_new_with_model(GTK_TREE_MODEL(store));
    gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(tv_ephem), TRUE);
    gtk_container_add(GTK_CONTAINER(scrolled), tv_ephem);

    /* --- Tab 1: column definitions --- */
    {
        /* Time column */
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Time (UTC)", r,
            "text", COL_TIME,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv_ephem), c);
        
    }
    {
        /* Latitude column */
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Lat (°)", r,
            "text", COL_LAT,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv_ephem), c);
    }
    {
        /* Longitude column */
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Lon (°)", r,
            "text", COL_LON,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv_ephem), c);
    }

    /* —————————————————————————— */
    /* Hook the “Orbits” spin to live-update Tab 1 ephemeris */
    {
        EphemUpdateCtx *update_ctx = g_new0(EphemUpdateCtx, 1);
        /* mark that no POI is yet linked */
        update_ctx->poi_ctx = NULL; 

        update_ctx->satmap   = satmap;
        update_ctx->sat      = sat;
        update_ctx->qth      = qth;
        update_ctx->store    = store;             /* the Tab 1 ListStore */
        update_ctx->treeview = GTK_TREE_VIEW(tv_ephem);
        update_ctx->hours_spin  = GTK_SPIN_BUTTON(spin_hours);
        update_ctx->step_spin   = GTK_SPIN_BUTTON(spin_step);

        // After you create spin_hours and spin_step, before the scrolled window:
        GtkWidget *progress = gtk_progress_bar_new();
        gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(progress), FALSE);

        /* +++ pack elapsed-time label and bar into page1, not hbox_orbits +++ */
        GtkWidget *time_label = gtk_label_new("0s");
        gtk_widget_set_halign(time_label, GTK_ALIGN_CENTER);
        gtk_box_pack_start(GTK_BOX(page1), time_label, FALSE, FALSE, 6);
        update_ctx->time_label     = GTK_LABEL(time_label);

        gtk_widget_set_hexpand(progress, TRUE);
        gtk_widget_set_halign(progress, GTK_ALIGN_FILL);
        gtk_box_pack_start(GTK_BOX(page1), progress, FALSE, FALSE, 0);

        // And store it in your update‐ctx so you can update it later:
        update_ctx->progress_bar = GTK_PROGRESS_BAR(progress);

        /* +++ NEW: total‐points label +++ */
        GtkWidget *count_label = gtk_label_new("Total: 0");
        gtk_box_pack_start(GTK_BOX(hbox_orbits), count_label, FALSE, FALSE, 6);
        update_ctx->count_label = GTK_LABEL(count_label);        

        /* wire both controls into the same live-update callback */
         g_signal_connect(spin_hours, "value-changed",
                         G_CALLBACK(on_orbits_value_changed),
                         update_ctx);
        g_signal_connect(spin_step, "value-changed",
                         G_CALLBACK(on_orbits_value_changed),
                         update_ctx);
        /* immediately populate Tab 1 exactly as the spin-handler does */
        on_orbits_value_changed(GTK_SPIN_BUTTON(spin_hours), update_ctx);


        /* Persist update_ctx on the dialog so we can find it later */
        g_object_set_data(G_OBJECT(dialog), "update_ctx", update_ctx);
    }
    /* —————————————————————————— */

     /* Remember Tab 1’s TreeView for Tab 2’s callbacks */
    g_object_set_data(G_OBJECT(dialog), "tv_ephemeris", tv_ephem);

    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), page1,
                             gtk_label_new("Ephemeris"));

    /* ─────── Tab 2: initialize container & country‐selection context ─────── */
    /* create the Tab 2 “Territory” page */
    GtkWidget *page2 = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);

    /* allocate our selection context for the “Show Table” callback */
    CountrySelectionCtx *country_ctx = g_new0(CountrySelectionCtx, 1);

    /* +++ pack “Select Region” and entry side-by-side +++ */
    GtkWidget *hbox_country = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(page2), hbox_country, FALSE, FALSE, 0);

    /* remember Tab 1’s TreeView so Tab 2 can re-read ephemeris data */
    country_ctx->tv_tab1 = GTK_TREE_VIEW(tv_ephem);

    /* Create the “Territory” button and hook it up */
    GtkWidget *territory_button = gtk_button_new_with_label("Territory");
    gtk_box_pack_start(GTK_BOX(hbox_country), territory_button, FALSE, FALSE, 0);
    country_ctx->button = territory_button;  /* ensure non-NULL before entry signals */
    /* Clicking it filters for all land (“Territory”) */
    g_signal_connect(territory_button, "clicked",
                     G_CALLBACK(on_territory_clicked),
                     country_ctx);

    /* +++ NEW: free-text entry with completion +++ */
    GtkWidget *entry = gtk_entry_new();
    gtk_entry_set_placeholder_text(GTK_ENTRY(entry),
                                   "Type a country…");
    /* Build completion model from all countries, but only unique names */
    GtkListStore *m = gtk_list_store_new(1, G_TYPE_STRING);
    /* hash to remember which names we’ve already seen */
    GHashTable *seen = g_hash_table_new_full(g_str_hash, g_str_equal, NULL, NULL);
    for (GList *l = tool_get_all_countries(); l; l = l->next) {
        const gchar *country = (const gchar*)l->data;
        if (!g_hash_table_contains(seen, country)) {
            /* first time we see this name, add it */
            g_hash_table_add(seen, (gpointer)country);

            GtkTreeIter iter2;
            gtk_list_store_append(m, &iter2);
            gtk_list_store_set(m, &iter2, 0, country, -1);
        }
    }
    g_hash_table_destroy(seen);
    GtkEntryCompletion *comp = gtk_entry_completion_new();
    gtk_entry_completion_set_model(comp, GTK_TREE_MODEL(m));
    gtk_entry_completion_set_text_column(comp, 0);
    /* ensure we pop up after one character */
    gtk_entry_completion_set_minimum_key_length(comp, 1);
    
    /* Make the pop-up the same width as the entry */
    g_object_set(comp, "popup-set-width", TRUE, NULL);


    /* pack the entry into our new hbox */
    gtk_box_pack_start(GTK_BOX(hbox_country), entry, TRUE, TRUE, 0);

    /* 4) Record it in our ctx and hook signals */
    country_ctx->entry = GTK_ENTRY(entry);
    g_signal_connect(comp, "match-selected",
                     G_CALLBACK(on_country_match_selected),
                     country_ctx);
    g_signal_connect(entry, "activate",
                     G_CALLBACK(on_country_entry_activate),
                     country_ctx);
    g_signal_connect(entry, "changed",
                     G_CALLBACK(on_country_changed),
                     country_ctx);

    /* +++ NEW: total‐points label +++ */
    GtkWidget *count_label = gtk_label_new("Total: 0");
    gtk_box_pack_start(GTK_BOX(hbox_country), count_label, FALSE, FALSE, 6);
    country_ctx->count_label = GTK_LABEL(count_label);




    /* +++ pulsing progress + elapsed-counter like Tab 3 +++ */
    GtkWidget *country_pb    = gtk_progress_bar_new();
    gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(country_pb), FALSE);

    /* +++ elapsed-seconds label above the pulse bar +++ */
    GtkWidget *time_label2    = gtk_label_new("0s");
    gtk_box_pack_start(GTK_BOX(page2), time_label2, FALSE, FALSE, 6);
    country_ctx->time_label   = GTK_LABEL(time_label2);
    /* pack the pulse bar */
    gtk_box_pack_start(GTK_BOX(page2), country_pb, FALSE, FALSE, 0);
    country_ctx->progress_bar = GTK_PROGRESS_BAR(country_pb);

                             
    /* ─────── Tab 2: Filtered Ephemeris (“Territory”) ─────── */

    /* Columns for Tab 2’s TreeView */
    enum {
        ZONE_COL_TIME = 0,
        ZONE_COL_LAT,
        ZONE_COL_LON,
        ZONE_COL_COUNTRY,
        ZONE_N_COLS
    };

    /* Inside on_show_ephemeris_activate(), after setting up page2 = gtk_box_new(): */

    /* 1) Create an empty ListStore with four columns */
    GtkListStore *empty2 = gtk_list_store_new(
        ZONE_N_COLS,
        G_TYPE_STRING,  /* ZONE_COL_TIME    */
        G_TYPE_DOUBLE,  /* ZONE_COL_LAT     */
        G_TYPE_DOUBLE,  /* ZONE_COL_LON     */
        G_TYPE_STRING   /* ZONE_COL_COUNTRY */
    );
    country_ctx->store  = empty2;

    /* 2) Create the TreeView for Tab 2 */
    GtkWidget *tv2 = gtk_tree_view_new_with_model(GTK_TREE_MODEL(empty2));
    g_object_unref(empty2);
    gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(tv2), TRUE);

    /* 3) Add the “Time” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Time", r,
            "text", ZONE_COL_TIME,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv2), c);
        
    }

    /* 4) Add the “Latitude” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Latitude", r,
            "text", ZONE_COL_LAT,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv2), c);
        /* +++ bind our custom formatter so lat_cell_data_func is used +++ */
        gtk_tree_view_column_set_cell_data_func(c, r,
            lat_cell_data_func, NULL, NULL);
    }

    /* 5) Add the “Longitude” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Longitude", r,
            "text", ZONE_COL_LON,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv2), c);
        /* +++ bind our custom formatter so lon_cell_data_func is used +++ */
        gtk_tree_view_column_set_cell_data_func(c, r,
            lon_cell_data_func, NULL, NULL);
    }

    /* 6) Add the “Country” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Country", r,
            "text", ZONE_COL_COUNTRY,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(tv2), c);
    }

    /* 7) Wrap the TreeView in a scrolled window */
    GtkWidget *scrolled2 = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled2),
                                GTK_POLICY_AUTOMATIC,
                                GTK_POLICY_AUTOMATIC);
    gtk_widget_set_vexpand(scrolled2, TRUE);
    gtk_widget_set_hexpand(scrolled2, TRUE);
    gtk_container_add(GTK_CONTAINER(scrolled2), tv2);

    /* 8) Pack into the Tab 2 page box */
    gtk_box_pack_start(GTK_BOX(page2), scrolled2, TRUE, TRUE, 0);
    

    /* 9) Store the TreeView pointer for the “Show Table” callback */
    country_ctx->treeview = tv2;
    g_object_set_data(G_OBJECT(dialog), "country_ctx", country_ctx);

    /* cancel any running territory task if the dialog is closed */
  g_signal_connect(dialog, "destroy",
                   G_CALLBACK(cancel_country_if_running),
                   country_ctx);

    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), page2,
                             gtk_label_new("Territory"));

    
    /* ──────────────────────── Tab 3: Points of Interest ──────────────────────── */
    GtkWidget *vbox_poi = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_box_set_homogeneous(GTK_BOX(vbox_poi), FALSE);

    /* Initialize POI names */
    points_interest_init(POI_CSV_FILE);
    GPtrArray *names = points_interest_get_names();

    GPtrArray *types = points_interest_get_types();  /* NEW: pull types in same order as names */

    /* (build completion model from `names`) */
    GtkListStore *poi_model = gtk_list_store_new(1, G_TYPE_STRING);
    for (guint i = 0; i < names->len; i++) {
        GtkTreeIter iter;
        gtk_list_store_append(poi_model, &iter);
        gtk_list_store_set(poi_model, &iter,
                        0, g_ptr_array_index(names, i),
                        -1);
    }

    /* Create the entry + completion */
    GtkWidget *poi_entry = gtk_entry_new();
    gtk_entry_set_placeholder_text(GTK_ENTRY(poi_entry),
                                "Type a point…");
    GtkEntryCompletion *poi_comp = gtk_entry_completion_new();
    gtk_entry_completion_set_model(poi_comp, GTK_TREE_MODEL(poi_model));
    gtk_entry_completion_set_text_column(poi_comp, 0);
    g_object_set(poi_comp, "popup-set-width", TRUE, NULL);
    /* Keep a ref to the completion model on the dialog for updates after Add */
    g_object_set_data_full(G_OBJECT(dialog), "poi_completion_model",
                           g_object_ref(poi_model), (GDestroyNotify)g_object_unref);


    /* Make our context and hook signals */
    POISelectionCtx *poi_ctx = g_new0(POISelectionCtx, 1);
    poi_ctx->tab1_tree = GTK_TREE_VIEW(tv_ephem);
    poi_ctx->names    = names;
    poi_ctx->types    = types;

    /* ── Build a horizontal row: [Refresh] [POI entry] ───────────────── */
    GtkWidget *hbox_poi = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_box_pack_start(GTK_BOX(vbox_poi), hbox_poi, FALSE, FALSE, 0);

    /* 1) “Refresh” button — user must click to rebuild the table */
    GtkWidget *refresh_btn = gtk_button_new_from_icon_name(
        "view-refresh", GTK_ICON_SIZE_BUTTON);
    gtk_box_pack_start(GTK_BOX(hbox_poi), refresh_btn,
                       FALSE, FALSE, 2);

    poi_ctx->button = refresh_btn;  /* so we can disable/enable it during work */

    g_signal_connect(refresh_btn, "clicked",
                     G_CALLBACK(on_poi_refresh_clicked),
                     poi_ctx);

    /* 2) POI text entry (with completion), purely for filter text */
    gtk_box_pack_start(GTK_BOX(hbox_poi), poi_entry, TRUE, TRUE, 0);
    poi_ctx->entry = GTK_ENTRY(poi_entry);
    /* hooked “match-selected” and “activate” to drive filtering: */
    g_signal_connect(poi_comp, "match-selected",
                     G_CALLBACK(on_poi_match_selected),
                     poi_ctx);
    g_signal_connect(poi_entry, "activate",
                     G_CALLBACK(on_poi_entry_activate),
                     poi_ctx);

    /* NEW: insert elapsed‐time label just under the controls */
    poi_ctx->time_label = GTK_LABEL(gtk_label_new("0s"));
    gtk_box_pack_start(GTK_BOX(vbox_poi),
                       GTK_WIDGET(poi_ctx->time_label),
                       FALSE, FALSE, 2);

    /* 3) Loading bar: always visible under Refresh+Entry */
    poi_ctx->progress_bar = gtk_progress_bar_new();
    /* we show elapsed time in the label, not % on the bar */
    gtk_progress_bar_set_show_text(GTK_PROGRESS_BAR(poi_ctx->progress_bar), FALSE);
    /* no hide() or no_show_all() here – leave it in the layout */
    gtk_box_pack_start(GTK_BOX(vbox_poi),
                       poi_ctx->progress_bar,
                       FALSE, FALSE, 2);

    /* ─── reserve space for the future POI table ─── */
    GtkWidget *sw3 = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(sw3),
                                GTK_POLICY_AUTOMATIC,
                                GTK_POLICY_AUTOMATIC);
    gtk_widget_set_vexpand(sw3, TRUE);
    gtk_widget_set_hexpand(sw3, TRUE);
    gtk_box_pack_start(GTK_BOX(vbox_poi),
                    sw3, TRUE, TRUE, 0);
                    
    /* ─── Tab 3: empty 6-col table ─── */
    enum {
        POI_COL_TIME   = 0,
        POI_COL_LAT,
        POI_COL_LON,
        POI_COL_RANGE,
        POI_COL_DIR,
        POI_COL_NAME,      /* existing: point-of-interest name */
        POI_COL_TYPE,      /* point-of-interest type */
        POI_N_COLS
    };

    /* build the store: Time, Lat, Lon, Range, Direction, Name, Type */
    GtkListStore *poi_store = gtk_list_store_new(
        POI_N_COLS,
        G_TYPE_STRING,  /* Time */
        G_TYPE_DOUBLE,  /* Lat */
        G_TYPE_DOUBLE,  /* Lon */
        G_TYPE_DOUBLE,  /* Range */
        G_TYPE_STRING,  /* Direction */
        G_TYPE_STRING,  /* Name */
        G_TYPE_STRING   /* Type */
    );  

    /* 2) Create the TreeView */
    GtkWidget *poi_tree = gtk_tree_view_new_with_model(GTK_TREE_MODEL(poi_store));
    
    gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(poi_tree), TRUE);

    /* 3) Add “Time” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Time", r,
            "text", POI_COL_TIME,
            NULL);
       gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 4) Add “Latitude” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new();
        gtk_tree_view_column_set_title(c, "Latitude");
        gtk_tree_view_column_pack_start(c, r, TRUE);
        /* reuse your lat_cell_data_func to format */
        gtk_tree_view_column_set_cell_data_func(c, r,
            lat_cell_data_func, GINT_TO_POINTER(POI_COL_LAT), NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 5) Add “Longitude” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new();
        gtk_tree_view_column_set_title(c, "Longitude");
        gtk_tree_view_column_pack_start(c, r, TRUE);
        /* reuse your lon_cell_data_func */
        gtk_tree_view_column_set_cell_data_func(c, r,
            lon_cell_data_func, GINT_TO_POINTER(POI_COL_LON), NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 6) Add “Range” column */
   {
        /* simple numeric renderer; we’ll format it later in your update logic */
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Range (km)", r,
            "text", POI_COL_RANGE,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 7) Add “Direction” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Direction (N, S, E, W)", r,
            "text", POI_COL_DIR,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 8) Add “Name” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Name", r,
            "text", POI_COL_NAME,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }

    /* 9) Add “Type” column */
    {
        GtkCellRenderer *r = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *c = gtk_tree_view_column_new_with_attributes(
            "Type", r,
            "text", POI_COL_TYPE,
            NULL);
        gtk_tree_view_append_column(GTK_TREE_VIEW(poi_tree), c);
    }


    /* 10) Add it into your scrolled window and remember in ctx */
    gtk_container_add(GTK_CONTAINER(sw3), poi_tree);

    
    /* record the store/treeview in our ctx */
    poi_ctx->store    = poi_store;
    poi_ctx->treeview = GTK_TREE_VIEW(poi_tree);

    /* keep a handle so the Save handler can find Tab 3’s model */
    g_object_set_data(G_OBJECT(dialog), "poi_ctx", poi_ctx);


    /* now that entry/store/treeview are set, hook auto‐refresh */
    {
        EphemUpdateCtx *update_ctx =
            g_object_get_data(G_OBJECT(dialog), "update_ctx");
        if (update_ctx)
            update_ctx->poi_ctx = poi_ctx;
    }

    /* Tab 3: initially populate with ALL points inside any tile */
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook),
                             vbox_poi,
                             gtk_label_new("Points of Interest"));



    
    /* toggle Save button when user changes tab */
    g_signal_connect(notebook, "switch-page",
                     G_CALLBACK(on_nb_switch_page), dialog);
    g_signal_connect(dialog, "response",
                     G_CALLBACK(on_ephem_dialog_response), NULL);
    g_signal_connect(dialog, "destroy",
                     G_CALLBACK(cancel_poi_if_running),
                     poi_ctx);

    // Final display
    gtk_widget_show_all(dialog);

    /* hide Save/Add until Points-of-Interest tab is active */
    {
      GtkWidget *save_btn = gtk_dialog_get_widget_for_response(
                             GTK_DIALOG(dialog),
                              GTK_RESPONSE_ACCEPT);

      gtk_widget_hide(save_btn);
      GtkWidget *add_btn = gtk_dialog_get_widget_for_response(
                              GTK_DIALOG(dialog), 1001);
      if (add_btn) gtk_widget_hide(add_btn);
    }

    /* Now that the dialog (and its GtkEntry widgets) are realized,
       attach both completions so GTK can anchor them properly. */
    gtk_entry_set_completion(GTK_ENTRY(country_ctx->entry), comp);
    gtk_entry_set_completion(GTK_ENTRY(poi_ctx->entry),   poi_comp);

    /* Now that completions are attached, we can drop our refs
       (the dialog holds an extra ref as “poi_completion_model”) */
    g_object_unref(comp);
    g_object_unref(m);
    g_object_unref(poi_comp);
    g_object_unref(poi_model);

    /* no modal run; we control lifetime via our response handler */
    
}



static void     coverage_toggled(GtkCheckMenuItem * item, gpointer data);
static void     track_toggled(GtkCheckMenuItem * item, gpointer data);

/* static void target_toggled (GtkCheckMenuItem *item, gpointer data); */



/*
 * Show satellite popup menu.
 *
 * @param sat Pointer to the satellite data.
 * @param qth The current location.
 * @param event The mouse-click related event info
 * @param toplevel Pointer to toplevel window.
 *
 */
/*
 * @brief Callback / helper function.
 * @function gtk_sat_map_popup_exec
 * @param sat sat_t * sat
 * @param qth qth_t * qth
 * @param satmap GtkSatMap * satmap
 * @param event GdkEventButton * event
 * @param toplevel GtkWidget * toplevel
 * @return (void)
 */
void gtk_sat_map_popup_exec(sat_t * sat, 
                            qth_t * qth,
                            GtkSatMap * satmap,
                            GdkEventButton * event, 
                            GtkWidget * toplevel)
{
    GtkWidget      *menu, *menuitem;
    sat_map_obj_t  *obj = NULL;
    gint           *catnum;

    menu = gtk_menu_new();

    /* first menu item is the satellite name, centered */
    menuitem = gtk_menu_item_new();
    gtk_menu_item_set_label(GTK_MENU_ITEM(menuitem), _("Satellite info"));

    /* attach data to menuitem and connect callback */
    g_object_set_data(G_OBJECT(menuitem), "sat", sat);
    g_object_set_data(G_OBJECT(menuitem), "qth", qth);
    g_signal_connect(menuitem, "activate", G_CALLBACK(show_sat_info_menu_cb),
                     toplevel);

    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);

    /* separator */
    menuitem = gtk_separator_menu_item_new();
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);

    /* add the menu items for current,next, and future passes. */
    add_pass_menu_items(menu, sat, qth, &satmap->tstamp, GTK_WIDGET(satmap));

    /* separator */
    menuitem = gtk_separator_menu_item_new();
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);

    /* get sat obj since we'll need it for the remaining items */
    catnum = g_new0(gint, 1);
    *catnum = sat->tle.catnr;
    obj = SAT_MAP_OBJ(g_hash_table_lookup(satmap->obj, catnum));
    g_free(catnum);

    /* highlight cov. area */
    menuitem = gtk_check_menu_item_new_with_label(_("Highlight footprint"));
    g_object_set_data(G_OBJECT(menuitem), "sat", sat);
    g_object_set_data(G_OBJECT(menuitem), "obj", obj);
    g_object_set_data(G_OBJECT(menuitem), "qth", qth);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuitem),
                                   obj->showcov);
    g_signal_connect(menuitem, "activate", G_CALLBACK(coverage_toggled),
                     satmap);

    /* show track */
    menuitem = gtk_check_menu_item_new_with_label(_("Ground Track"));
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuitem),
                                   obj->showtrack);
    g_object_set_data(G_OBJECT(menuitem), "sat", sat);
    g_object_set_data(G_OBJECT(menuitem), "qth", qth);
    g_object_set_data(G_OBJECT(menuitem), "obj", obj);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuitem),
                                   obj->showtrack);
    g_signal_connect(menuitem, "activate", G_CALLBACK(track_toggled), satmap);

    /* ── Step 5.1: Create “Show Ephemeris” and store our context pointers ── */
    menuitem = gtk_menu_item_new_with_label(_("Show Ephemeris"));

    ShowEphemCtx *ctx = g_new0(ShowEphemCtx, 1);
    ctx->satmap = satmap;
    ctx->sat    = sat;
    ctx->qth    = qth;

    /* Connect the callback, passing ctx as user_data */
    g_signal_connect(menuitem, "activate",
                     G_CALLBACK(on_show_ephemeris_activate),
                     ctx);

    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);
    /* ── end insertion ── */
    gtk_widget_show_all(menu);



#if 0
    /* target */
    menuitem = gtk_check_menu_item_new_with_label(_("Set Target"));
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuitem),
                                   obj->istarget);
    g_object_set_data(G_OBJECT(menuitem), "sat", sat);
    g_object_set_data(G_OBJECT(menuitem), "qth", qth);
    g_object_set_data(G_OBJECT(menuitem), "obj", obj);
    gtk_menu_shell_append(GTK_MENU_SHELL(menu), menuitem);
    gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(menuitem),
                                   obj->istarget);
    g_signal_connect(menuitem, "activate", G_CALLBACK(target_toggled), satmap);
    gtk_widget_set_sensitive(menuitem, FALSE);
#endif

    gtk_widget_show_all(menu);

    /* gtk_menu_popup got deprecated in 3.22, first available in Ubuntu 18.04 */
#if GTK_MINOR_VERSION < 22
    gtk_menu_popup(GTK_MENU(menu), NULL, NULL, NULL, NULL,
                   (event != NULL) ? event->button : 0,
                   gdk_event_get_time((GdkEvent *) event));
#else
    gtk_menu_popup_at_pointer(GTK_MENU(menu), (GdkEvent*)event);
#endif
}



/*
 * Manage toggling of Area Coverage.
 *
 * @param item The menu item that was toggled.
 * @param data Pointer to the GtkSatMap structure.
 *
 */
/*
 * @brief Callback / helper function.
 * @function coverage_toggled
 * @param item GtkCheckMenuItem * item
 * @param data gpointer data
 * @return (void)
 */
static void coverage_toggled(GtkCheckMenuItem * item, gpointer data)
{
    sat_map_obj_t  *obj = NULL;
    sat_t          *sat;
    GtkSatMap      *satmap = GTK_SAT_MAP(data);
    guint32         covcol;

    /* get satellite object */
    obj = SAT_MAP_OBJ(g_object_get_data(G_OBJECT(item), "obj"));
    sat = SAT(g_object_get_data(G_OBJECT(item), "sat"));

    if (obj == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to get satellite object."),
                    __FILE__, __LINE__);
        return;
    }

    /* toggle flag */
    obj->showcov = !obj->showcov;
    gtk_check_menu_item_set_active(item, obj->showcov);

    if (obj->showcov)
    {
        /* remove it from the storage structure */
        g_hash_table_remove(satmap->hidecovs, &(sat->tle.catnr));

    }
    else
    {
        g_hash_table_insert(satmap->hidecovs,
                            &(sat->tle.catnr), (gpointer) 0x1);
    }

    /* set or clear coverage colour */
    if (obj->showcov)
    {
        covcol = mod_cfg_get_int(satmap->cfgdata,
                                 MOD_CFG_MAP_SECTION,
                                 MOD_CFG_MAP_SAT_COV_COL,
                                 SAT_CFG_INT_MAP_SAT_COV_COL);
    }
    else
    {
        covcol = 0x00000000;
    }

    g_object_set(obj->range1, "fill-color-rgba", covcol, NULL);

    if (obj->newrcnum == 2)
    {
        g_object_set(obj->range2, "fill-color-rgba", covcol, NULL);
    }
}



/*
 * Manage toggling of Ground Track.
 *
 * @param item The menu item that was toggled.
 * @param data Pointer to the GtkSatMap structure.
 *
 */
/*
 * @brief Callback / helper function.
 * @function track_toggled
 * @param item GtkCheckMenuItem * item
 * @param data gpointer data
 * @return (void)
 */
static void track_toggled(GtkCheckMenuItem * item, gpointer data)
{
    sat_map_obj_t  *obj = NULL;
    sat_t          *sat = NULL;
    qth_t          *qth = NULL;
    GtkSatMap      *satmap = GTK_SAT_MAP(data);

    /* get satellite object */
    obj = SAT_MAP_OBJ(g_object_get_data(G_OBJECT(item), "obj"));
    sat = SAT(g_object_get_data(G_OBJECT(item), "sat"));
    qth = (qth_t *) (g_object_get_data(G_OBJECT(item), "qth"));

    if (obj == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to get satellite object."),
                    __FILE__, __LINE__);
        return;
    }

    /* toggle flag */
    obj->showtrack = !obj->showtrack;
    gtk_check_menu_item_set_active(item, obj->showtrack);

    if (obj->showtrack)
    {
        /* create ground track */
        ground_track_create(satmap, sat, qth, obj);

        /* add it to the storage structure */
        g_hash_table_insert(satmap->showtracks,
                            &(sat->tle.catnr), (gpointer) 0x1);

    }
    else
    {
        /* delete ground track with clear_ssp = TRUE */
        ground_track_delete(satmap, sat, qth, obj, TRUE);

        /* remove it from the storage structure */
        g_hash_table_remove(satmap->showtracks, &(sat->tle.catnr));
    }
}

#if 0
/*
 * Manage toggling of Set Target.
 *
 * @param item The menu item that was toggled.
 * @param data Pointer to the GtkSatMap structure.
 *
 */
/*
 * @brief Callback / helper function.
 * @function target_toggled
 * @param item GtkCheckMenuItem * item
 * @param data gpointer data
 * @return (void)
 */
static void target_toggled(GtkCheckMenuItem * item, gpointer data)
{
    sat_map_obj_t  *obj = NULL;

    /* get satellite object */
    obj = SAT_MAP_OBJ(g_object_get_data(G_OBJECT(item), "obj"));

    if (obj == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to get satellite object."),
                    __FILE__, __LINE__);
        return;
    }

    /* toggle flag */
    obj->istarget = !obj->istarget;
    gtk_check_menu_item_set_active(item, obj->istarget);
}
#endif
