/* src/sub_window_ephemeris.c */
#include "sub_window_ephemeris.h"
#include <time.h>
#include <string.h> /* strpbrk */

static void set_default_name(GtkFileChooser *fc, SubwinFormat fmt) {
     /* poi_YYYYMMDD_HHMMSS.ext */
     time_t t = time(NULL);
     struct tm *tm = gmtime(&t);
     gchar *stamp = g_strdup_printf("poi_%04d%02d%02d_%02d%02d%02d.%s",
         tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
         tm->tm_hour, tm->tm_min, tm->tm_sec,
         fmt == SUBWIN_FORMAT_CSV ? "csv" : "txt");
     gtk_file_chooser_set_current_name(fc, stamp);
     g_free(stamp);
}

gboolean sub_window_ephemeris_run(GtkWindow *parent, SubwinSaveSpec *out_spec) {
     g_return_val_if_fail(out_spec != NULL, FALSE);

     GtkFileChooserNative *native = gtk_file_chooser_native_new(
         "Save Points of Interest",
         parent,
         GTK_FILE_CHOOSER_ACTION_SAVE,
         "_Confirm",
         "_Cancel");

     GtkFileChooser *fc = GTK_FILE_CHOOSER(native);
     gtk_file_chooser_set_do_overwrite_confirmation(fc, TRUE);

     /* Filters for CSV and TXT */
     GtkFileFilter *f_csv = gtk_file_filter_new();
     gtk_file_filter_set_name(f_csv, "CSV file (*.csv)");
     gtk_file_filter_add_pattern(f_csv, "*.csv");
     gtk_file_chooser_add_filter(fc, f_csv);

     GtkFileFilter *f_txt = gtk_file_filter_new();
     gtk_file_filter_set_name(f_txt, "Text file (*.txt)");
     gtk_file_filter_add_pattern(f_txt, "*.txt");
     gtk_file_chooser_add_filter(fc, f_txt);

     /* Default to CSV filename */
     set_default_name(fc, SUBWIN_FORMAT_CSV);
     gtk_file_chooser_set_filter(fc, f_csv);

     gboolean ok = FALSE;
     gint resp = gtk_native_dialog_run(GTK_NATIVE_DIALOG(native));
     if (resp == GTK_RESPONSE_ACCEPT) {
         GFile *file = gtk_file_chooser_get_file(fc);
         gchar *path = g_file_get_path(file);
         GtkFileFilter *sel = gtk_file_chooser_get_filter(fc);
         SubwinFormat fmt = (sel == f_txt || g_str_has_suffix(path, ".txt"))
                            ? SUBWIN_FORMAT_TXT : SUBWIN_FORMAT_CSV;

         out_spec->filepath = g_strdup(path);
         out_spec->format   = fmt;
         ok = TRUE;

         g_free(path);
         g_object_unref(file);
     }
     g_object_unref(native);
     return ok;
 }

void sub_window_ephemeris_spec_free(SubwinSaveSpec *spec) {
     if (!spec) return;
     g_clear_pointer(&spec->filepath, g_free);
 }

/* ---- CSV/TXT export --------------------------------------------------- */
static gchar *csv_escape(const gchar *s) {
    if (!s) return g_strdup("");
    if (!strpbrk(s, ",\"\n\r")) return g_strdup(s);
    GString *g = g_string_new("\"");
    for (const char *p = s; *p; ++p) {
        if (*p == '"') g_string_append_c(g, '"'); /* double quotes */
        g_string_append_c(g, *p);
    }
    g_string_append_c(g, '"');
    return g_string_free(g, FALSE);
}

static gchar *ensure_ext(const gchar *path, gboolean is_csv) {
    if (!path) return NULL;
    if (g_str_has_suffix(path, is_csv ? ".csv" : ".txt")) return g_strdup(path);
    return g_strconcat(path, is_csv ? ".csv" : ".txt", NULL);
}

gboolean sub_window_ephemeris_export_poi(GtkTreeView *tv,
                                         const SubwinSaveSpec *spec,
                                         const POIColumns *c,
                                         GError **err)
{
    GtkTreeModel *m = gtk_tree_view_get_model(tv);
    if (!m) {
        g_set_error(err, g_quark_from_static_string("export"), 1,
                    "No model to export");
        return FALSE;
    }

    gboolean csv = (spec->format == SUBWIN_FORMAT_CSV);
    GString *out = g_string_new(NULL);
    /* Excel hint: BOM makes it detect UTF-8, avoiding 'Â°' */
    if (csv) g_string_append(out, "\xEF\xBB\xBF");

    if (csv) {
        g_string_append(out, "Time,Latitude,Longitude,Range_km,Direction,Name,Type\n");
    } else {
        g_string_append(out, "Time\tLatitude\tLongitude\tRange (km)\tDirection\tName\tType\n");
    }

    GtkTreeIter it;
    gboolean ok = gtk_tree_model_get_iter_first(m, &it);
    while (ok) {
        gchar *time = NULL, *dir = NULL, *name = NULL, *type = NULL;
        gdouble lat = 0, lon = 0, range = 0;
        gtk_tree_model_get(m, &it,
            c->col_time,  &time,
            c->col_lat,   &lat,
            c->col_lon,   &lon,
            c->col_range, &range,
            c->col_dir,   &dir,
            c->col_name,  &name,
            c->col_type,  &type,
            -1);

        char slat[32], slon[32], srange[32];
        g_ascii_formatd(slat,  sizeof(slat),  "%.5f", lat);
        g_ascii_formatd(slon,  sizeof(slon),  "%.5f", lon);
        g_ascii_formatd(srange,sizeof(srange),"%.3f", range);

        if (csv) {
            gchar *qtime = csv_escape(time);
            gchar *qdir  = csv_escape(dir);
            gchar *qname = csv_escape(name);
            gchar *qtype = csv_escape(type);
            g_string_append_printf(out, "%s,%s,%s,%s,%s,%s,%s\n",
                qtime, slat, slon, srange, qdir, qname, qtype);
            g_free(qtime); g_free(qdir); g_free(qname); g_free(qtype);
        } else {
            g_string_append_printf(out, "%s\t%s\t%s\t%s\t%s\t%s\t%s\n",
                time ? time : "", slat, slon, srange,
                dir ? dir : "", name ? name : "", type ? type : "");
        }

        g_free(time); g_free(dir); g_free(name); g_free(type);
        ok = gtk_tree_model_iter_next(m, &it);
    }

    gchar *final_path = ensure_ext(spec->filepath, csv);
    gboolean wrote = g_file_set_contents(final_path, out->str, out->len, err);
    g_free(final_path);
    g_string_free(out, TRUE);
    return wrote;
}
