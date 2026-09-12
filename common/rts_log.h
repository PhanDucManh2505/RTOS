/*
 * rts_log.h - asynchronous logging to the persistent file system.
 *
 * The brief allows test vectors and debug information to be stored
 * under /fs. Writing a file takes far longer than anything else these
 * processes do, so no thread is allowed to wait for it. Callers drop a
 * line into a ring buffer and carry on; a low priority writer thread
 * empties the buffer into the file.
 *
 * The buffer is shared by several threads, so it is protected by a
 * mutex, and the writer sleeps on a condition variable instead of
 * polling. Those are the two synchronisation primitives this part of
 * the system demonstrates.
 */
#ifndef RTS_LOG_H
#define RTS_LOG_H

void rts_log_start(const char *proc_name);
void rts_log(const char *fmt, ...);
void rts_log_stop(void);
/* How many lines had to be dropped because the buffer was full. */
unsigned rts_log_drops(void);
/* Full path of the file being written, for the demonstration. */
const char *rts_log_path(void);

#endif /* RTS_LOG_H */
