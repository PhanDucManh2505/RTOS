#include "rts_log.h"
#include "rts_util.h"
#include "rts_timing.h"

#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define LOG_SLOTS 256
#define LOG_LINE  192

static char            g_ring[LOG_SLOTS][LOG_LINE];
static int             g_head;          /* next slot the writer reads  */
static int             g_tail;          /* next slot a producer fills  */
static int             g_count;
static unsigned        g_drops;
static int             g_started;
static pthread_mutex_t g_lk;
static pthread_cond_t  g_cv;
static pthread_t       g_tid;
static FILE           *g_fp;
static char            g_path[160];

static void *log_writer(void *arg)
{
    char line[LOG_LINE];
    (void)arg;

    while (1) {
        pthread_mutex_lock(&g_lk);
        while (g_count == 0 && g_started) {
            pthread_cond_wait(&g_cv, &g_lk);
        }
        if (g_count == 0 && !g_started) {
            pthread_mutex_unlock(&g_lk);
            break;                       /* asked to stop, nothing left */
        }
        memcpy(line, g_ring[g_head], LOG_LINE);
        g_head = (g_head + 1) % LOG_SLOTS;
        g_count--;
        pthread_mutex_unlock(&g_lk);

        if (g_fp != NULL) {
            fputs(line, g_fp);
            fputc('\n', g_fp);
            fflush(g_fp);
        }
    }
    return NULL;
}

void rts_log_start(const char *proc_name)
{
    rts_mutex_init(&g_lk);
    pthread_cond_init(&g_cv, NULL);

    /* /fs is the persistent file system on the QNX target. If it is not
       there, fall back to the current directory so logging never fails. */
    mkdir("/fs/rts", 0777);
    snprintf(g_path, sizeof(g_path), "/fs/rts/%s.log", proc_name);
    g_fp = fopen(g_path, "a");
    if (g_fp == NULL) {
        snprintf(g_path, sizeof(g_path), "./%s.log", proc_name);
        g_fp = fopen(g_path, "a");
    }

    g_started = 1;
    rts_thread(&g_tid, log_writer, NULL, 5);   /* lowest priority */
    rts_log("=== %s started, speed %.1fx ===", proc_name, rts_speed());
}

void rts_log(const char *fmt, ...)
{
    char    body[LOG_LINE - 24];
    va_list ap;

    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    pthread_mutex_lock(&g_lk);
    if (g_count >= LOG_SLOTS) {
        /* Buffer full: drop the line rather than make the caller wait. */
        g_drops++;
    } else {
        snprintf(g_ring[g_tail], LOG_LINE, "[%9.3f] %s", rts_uptime_s(), body);
        g_tail = (g_tail + 1) % LOG_SLOTS;
        g_count++;
        pthread_cond_signal(&g_cv);
    }
    pthread_mutex_unlock(&g_lk);
}

void rts_log_stop(void)
{
    pthread_mutex_lock(&g_lk);
    g_started = 0;
    pthread_cond_signal(&g_cv);
    pthread_mutex_unlock(&g_lk);

    pthread_join(g_tid, NULL);
    if (g_fp != NULL) {
        fclose(g_fp);
        g_fp = NULL;
    }
}

unsigned rts_log_drops(void) { return g_drops; }
const char *rts_log_path(void) { return g_path; }
