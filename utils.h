#ifndef __UTILS__
#define __UTILS__

#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <stdint.h>
#include <err.h>
#include <getopt.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>
#include <dirent.h>
#include <poll.h>
#include <errno.h>
#include <pthread.h>
#include <assert.h>

#define KB  1024
#define MB  1024 * KB

#define CKH_MIN_SIZE 512
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define BATCH_SIZE      0x10000

#define SECTOR_BYTES        512
#define SUPERBLOCK_SIZE     16ULL * 1024ULL * 1024ULL

#define SUB_BLOCKSIZE_BITS      3
#define SUB_BLOCKSIZE_MASK      ((1 << SUB_BLOCKSIZE_BITS) - 1)

#define MAX_TEST            2

#define MAX_LBN_BITS        42
#define SUPER_BITS          15
#define TRACE_OFFSET        36

extern double achievedSampleRate;
extern uint8_t *map;

typedef struct {
  uint64_t ts;
  uint32_t len;
  uint64_t lbn;
  uint16_t cmd;
} trace_item_t;

typedef struct {
  uint32_t sn;
  uint32_t len;
  uint32_t nSG;
  uint16_t cmd;
  uint16_t ver;
  uint64_t lbn;
  uint64_t ts;
} trace_v1_record_t;

typedef struct {
  uint16_t cmd;
  uint16_t ver;
  uint32_t sn;
  uint32_t len;
  uint32_t nSG;
  uint64_t lbn;
  uint64_t ts;
  uint64_t rt;
} trace_v2_record_t;

typedef enum {
  VSCSI1 = 1,
  VSCSI2 = 2,
  UNKNOWN
} vscsi_version_t;

typedef enum {
  WL_START = 0,
  WL_CONTINUE,
  WL_TERMINATE
} sync_response_type;

typedef enum {
    FORMAT_FILENOTFOUND = -2,
    FORMAT_TOOSMALL = -1,
    FORMAT_PARDA = 0,
    FORMAT_VSCSI1 = 1,
    FORMAT_VSCSI2 = 2,
} trace_format_t;

typedef struct epoch_buffer {
  int format;
  size_t elem_size;
  uint8_t *buffer;
  size_t buffer_len;
} epoch_buffer;

typedef struct epoch_result {
    int epoch_index;
    unsigned int config_index;
    unsigned int read_io_index;
    unsigned int write_io_index;

    float miss_ratio;
    uint64_t mini_sim_ptr;
} epoch_result;

typedef struct epoch_config {
    int epoch_index;
    int mini_sim_index;
    unsigned int max_refs;
    unsigned int config_index;
    unsigned int read_io_index;
    unsigned int write_io_index;
    uint64_t cache_size;
} epoch_config;

typedef struct epoch_sampling_rate
{
    unsigned int epoch_index;
    uint64_t sampled;
    uint64_t skipped;
    uint64_t epochMaxRefs;
    double sampleRate;
    uint32_t instSampled;
} epoch_sampling_rate;

typedef struct trace_context {
    char * trace_name;
    FILE* trace_file;
    size_t trace_size;
    size_t tsz;
    trace_format_t trace_format;
    uint64_t start_ts;
    size_t total_offset;
    size_t got;
    size_t batch_offset;
    uint8_t *batch;
    size_t epoch_batch_size;
    size_t epoch_buffer_len;
    uint8_t *epoch_batch;
    size_t epoch_buffer_itr;
    size_t abs_trace_itr;
    size_t abs_req_trace_itr;
    unsigned int epoch_index;
    unsigned int max_epoch_refs;
} trace_context;

#define MAX_FILE_NAME_LEN   1024

typedef struct ts_sync_req {
    unsigned int workload_index;
    uint64_t workload_ptr;
    uint64_t start_ts;
    char file_name[MAX_FILE_NAME_LEN];
} ts_sync_req;

typedef struct sync_response {
    unsigned int workload_index;
    sync_response_type type;
    uint64_t ref_ts;
    int miss_percent;
} sync_response;

typedef struct reference_req {
    int epoch_index;
    unsigned int workload_index;
    unsigned int msim_index;
    uint64_t workload_ptr;
    bool last_reference;
    uint64_t logical_ts;
    uint64_t physical_ts;
    uint32_t instSampled;
    trace_item_t reference;
} reference_req;

typedef struct reference_response {
    reference_req request;
    int hits;
} reference_response;

typedef struct cache_request {
    uint16_t type;    // cachelab_io_type_t
    uint32_t ioSize; // IO Length to help decide IO partition
    uint32_t len;    // io length in sectors (512 bytes)
    uint64_t lbn;    // logical offset in sectors (512 bytes)
    uint64_t ts;     // timestamp in nanoseconds
} cache_request;

typedef struct cache_response {
    cache_request request;
    int hits;
} cache_response;

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
        char nanosec[10];

        seconds = l_time.tv_sec;
        snprintf(nanosec, 10 , "%09ld", l_time.tv_nsec);
        strftime(timebuffer, sizeof (timebuffer), "%Y-%m-%d %H:%M:%S", localtime(&seconds));
        snprintf(timestring, sizeof (timestring), "%s,%.3s", timebuffer , nanosec);
        
        fprintf(
            stderr, "%s %d %d %-5s %s:%d: ",
            timestring, getpid(), getppid(), level_strings[level], file, line);

        vfprintf(stderr, fmt, ap);
        fprintf(stderr, "\n");
        fflush(stderr);
        
        va_end(ap);
    }
}

trace_format_t get_trace_version(const char *traceName, size_t *size);
int extractTraceItem(trace_format_t format, uint8_t *p,
                                   bool readsOnly, bool writesOnly, int *type, uint64_t *ts, uint64_t *lbn,
                                   uint32_t *numSectors);
void get_trace_version_ctx (trace_context *ctx);
void trace_read_setup_ctx (trace_context *ctx);
bool trace_read_ctx (trace_context *ctx);
void extractTraceItem_ctx(trace_format_t format, uint8_t *p, trace_item_t *trace_ref);
bool is_sampled (reference_req *request, epoch_sampling_rate *sampling_stats);

#endif
