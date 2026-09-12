/*
 * rts_shm.h - the corridor region shared by the six intersection
 * processes on VM2.
 *
 * Qnet carries messages between nodes but it cannot carry a shared
 * memory region, so shared memory is only useful inside one machine.
 * All six local controllers run on VM2, which makes this the right
 * place for it: each controller publishes when its cycle started, and
 * its neighbour reads that to hold the green wave offset without any
 * message traffic at all.
 *
 * There is one writer per slot and several readers, so no lock is
 * used. Instead each slot carries a sequence number that is odd while
 * a write is in progress. A reader that sees an odd number, or a
 * different number before and after, simply reads again. This is the
 * standard seqlock pattern: readers never block a writer, and a writer
 * never blocks at all, which is what a real-time system wants.
 */
#ifndef RTS_SHM_H
#define RTS_SHM_H

#include "rts_proto.h"
#include <stdint.h>

#define RTS_SHM_NAME  "/rts_corridor"
#define RTS_SHM_MAGIC 0x52545332u   /* "RTS2": slots now carry the pattern */

typedef struct {
    volatile uint32_t seq;      /* odd = being written, even = stable */
    uint8_t  present;           /* 1 once the controller has started  */
    uint8_t  phase;
    uint8_t  state;
    uint8_t  train_hold;
    uint8_t  pattern;           /* pattern_t, so a partner knows the cycle */
    uint32_t cycle_count;
    uint64_t cycle_start_ns;
    uint64_t updated_ns;
} rts_slot_t;

typedef struct {
    uint32_t   magic;
    uint32_t   pad;
    rts_slot_t slot[RTS_N_INTERSECTIONS];
} rts_corridor_t;

/* Map the region, creating it if this is the first process to start. */
rts_corridor_t *rts_shm_open(void);
void            rts_shm_close(rts_corridor_t *c);

/* Publish this controller's own slot. id is 1..6. */
void rts_slot_write(rts_corridor_t *c, int id,
                    uint8_t phase, uint8_t state, uint8_t train_hold,
                    uint8_t pattern, uint32_t cycle_count,
                    uint64_t cycle_start_ns);

/* Read a neighbour's slot. Returns 0 on success, -1 if no stable read
   was possible or the neighbour has not started yet. */
int rts_slot_read(const rts_corridor_t *c, int id, rts_slot_t *out);

#endif /* RTS_SHM_H */
