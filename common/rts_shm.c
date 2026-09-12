#include "rts_shm.h"
#include "rts_util.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

/* Stops the compiler moving memory accesses across this point. It costs
   nothing at run time but keeps the seqlock correct. */
#define BARRIER() __asm__ __volatile__("" ::: "memory")

rts_corridor_t *rts_shm_open(void)
{
    int             fd;
    rts_corridor_t *c;

    fd = shm_open(RTS_SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (fd == -1) {
        return NULL;
    }
    if (ftruncate(fd, (off_t)sizeof(rts_corridor_t)) == -1) {
        close(fd);
        return NULL;
    }

    c = (rts_corridor_t *)mmap(NULL, sizeof(rts_corridor_t),
                               PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);                 /* the mapping stays valid after close */
    if (c == MAP_FAILED) {
        return NULL;
    }

    if (c->magic != RTS_SHM_MAGIC) {
        memset(c, 0, sizeof(*c));
        c->magic = RTS_SHM_MAGIC;
    }
    return c;
}

void rts_shm_close(rts_corridor_t *c)
{
    if (c != NULL) {
        munmap(c, sizeof(rts_corridor_t));
    }
}

void rts_slot_write(rts_corridor_t *c, int id,
                    uint8_t phase, uint8_t state, uint8_t train_hold,
                    uint8_t pattern, uint32_t cycle_count,
                    uint64_t cycle_start_ns)
{
    rts_slot_t *s;

    if (c == NULL || id < 1 || id > RTS_N_INTERSECTIONS) {
        return;
    }
    s = &c->slot[id - 1];

    s->seq++;                 /* now odd: readers know to try again */
    BARRIER();
    s->present        = 1;
    s->phase          = phase;
    s->state          = state;
    s->train_hold     = train_hold;
    s->pattern        = pattern;
    s->cycle_count    = cycle_count;
    s->cycle_start_ns = cycle_start_ns;
    s->updated_ns     = rts_now_ns();
    BARRIER();
    s->seq++;                 /* even again: the slot is consistent */
}

int rts_slot_read(const rts_corridor_t *c, int id, rts_slot_t *out)
{
    const rts_slot_t *s;
    uint32_t          before;
    int               tries;

    if (c == NULL || id < 1 || id > RTS_N_INTERSECTIONS || out == NULL) {
        return -1;
    }
    s = &c->slot[id - 1];

    for (tries = 0; tries < 64; tries++) {
        before = s->seq;
        if (before & 1u) {
            continue;                     /* a write is in progress */
        }
        memcpy(out, s, sizeof(*out));
        BARRIER();
        if (s->seq == before) {
            return out->present ? 0 : -1; /* nothing changed while reading */
        }
    }
    return -1;
}
