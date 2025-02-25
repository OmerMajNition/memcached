#include "config.h"
#include <time.h>
#include <stdarg.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
enum { LOG_TRACE, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };

static int log_level = LOG_TRACE;

static const char *level_strings[] = {
  "TRACE", "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

#define log_trace(...) logging(LOG_TRACE, __FILENAME__, __LINE__, __VA_ARGS__)
#define log_debug(...) logging(LOG_DEBUG, __FILENAME__, __LINE__, __VA_ARGS__)
#define log_info(...)  logging(LOG_INFO,  __FILENAME__, __LINE__, __VA_ARGS__)
#define log_warn(...)  logging(LOG_WARN,  __FILENAME__, __LINE__, __VA_ARGS__)
#define log_error(...) logging(LOG_ERROR, __FILENAME__, __LINE__, __VA_ARGS__)
#define log_fatal(...) logging(LOG_FATAL, __FILENAME__, __LINE__, __VA_ARGS__)

#define snprintf_trunc(dst, size, ...)  \
do {                                    \
    volatile size_t n = size;           \
    snprintf (dst, n, __VA_ARGS__);     \
} while (0)

static inline void logging(int level, const char *file, int line, const char *fmt, ...)
{
    if (level >= log_level)
    {
        va_list ap;
        struct timespec l_time;
        clock_gettime(CLOCK_REALTIME, &l_time);
        va_start(ap, fmt);
        
        time_t seconds;
        char timestring[32];
        char timebuffer[32] = { 0 };
        char nanosec[32];

        seconds = l_time.tv_sec;
        snprintf_trunc (nanosec, 10, "%09ld", l_time.tv_nsec);
        strftime(timebuffer, sizeof (timebuffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
        snprintf_trunc(timestring, sizeof (timestring), "%s,%.3s", timebuffer , nanosec);
        
        fprintf(
            stderr, "%s %d %d %-5s %s:%d: ",
            timestring, getpid(), getppid(), level_strings[level], file, line);

        vfprintf(stderr, fmt, ap);
        fprintf(stderr, "\n");
        fflush(stderr);
        
        va_end(ap);
    }
}
/* fast-enough functions for uriencoding strings. */
void uriencode_init(void);
bool uriencode(const char *src, char *dst, const size_t srclen, const size_t dstlen);

/*
 * Wrappers around strtoull/strtoll that are safer and easier to
 * use.  For tests and assumptions, see internal_tests.c.
 *
 * str   a NULL-terminated base decimal 10 unsigned integer
 * out   out parameter, if conversion succeeded
 *
 * returns true if conversion succeeded.
 */
bool safe_strtoull(const char *str, uint64_t *out);
bool safe_strtoull_hex(const char *str, uint64_t *out);
bool safe_strtoll(const char *str, int64_t *out);
bool safe_strtoul(const char *str, uint32_t *out);
bool safe_strtol(const char *str, int32_t *out);
bool safe_strtod(const char *str, double *out);
bool safe_strcpy(char *dst, const char *src, const size_t dstmax);
bool safe_memcmp(const void *a, const void *b, size_t len);

#ifndef HAVE_HTONLL
extern uint64_t htonll(uint64_t);
extern uint64_t ntohll(uint64_t);
#endif

#ifdef __GCC
# define __gcc_attribute__ __attribute__
#else
# define __gcc_attribute__(x)
#endif

/**
 * Vararg variant of perror that makes for more useful error messages
 * when reporting with parameters.
 *
 * @param fmt a printf format
 */
void vperror(const char *fmt, ...)
    __gcc_attribute__ ((format (printf, 1, 2)));
