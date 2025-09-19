/* win32-compat-time.h */
#pragma once
#ifdef _WIN32
  #include <time.h>
  #include <stdio.h>
  #include <string.h>

  static inline int gmtime_r(const time_t *tp, struct tm *tm) {
      struct tm *tmp = gmtime(tp);
      if (!tmp) return -1;
      *tm = *tmp;
      return 0;
  }

  #ifndef timegm
  #define timegm _mkgmtime
  #endif

  static inline char *strptime(const char *s, const char *fmt, struct tm *tm) {
      if (sscanf(s, "%d-%d-%d %d:%d:%d",
                 &tm->tm_year, &tm->tm_mon, &tm->tm_mday,
                 &tm->tm_hour, &tm->tm_min, &tm->tm_sec) != 6)
          return NULL;
      tm->tm_year -= 1900;
      tm->tm_mon  -= 1;
      return (char*)(s + strlen(fmt)); /* dummy return */
  }
#endif
