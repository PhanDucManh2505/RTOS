/*
 * rts_safety.h - the one place that decides whether a set of lamp
 * values is legal.
 *
 * Three rules drive the whole design:
 *   1. No two conflicting movements may show green at the same time.
 *   2. Every green ends in a fixed amber and all-red that nothing may
 *      shorten.
 *   3. While a train is expected, no movement that ends in the arm the
 *      tracks cross may show anything but red.
 *
 * Rules 1 and 3 are checked here, on every single lamp change, just
 * before the lamps are committed. If a check ever fails the controller
 * falls back to all-red rather than showing an unsafe combination.
 */
#ifndef RTS_SAFETY_H
#define RTS_SAFETY_H

#include "rts_proto.h"
#include <stdint.h>

/* Which phase a movement belongs to. Movements in different phases
   conflict; movements in the same phase do not. */
int rts_mv_phase(int mv);
int rts_ped_phase(int pd);

/* The two ends of a movement, as arm_t: MV_NW for instance comes from
   the north arm and leaves by the west one. */
int rts_mv_from(int mv);
int rts_mv_to(int mv);

/* Every movement of one phase, as a bitmask of (1u << mv). */
uint8_t rts_phase_mask(int phase);

/*
 * The movements a train forbids: the ones that end in the arm the
 * tracks cross, whatever phase they belong to. That is one through
 * movement plus one right turn off the other road. Everything else may
 * keep running, because it drives away from the tracks.
 *
 * rail_arm == ARM_NONE means the geometry was never configured; then
 * the whole of phases C and D is blocked, which is the safe answer.
 */
uint8_t rts_rail_block_mask(int rail_arm);

/*
 * The movement that empties the road between the gates and the stop
 * line: the through movement that comes out of the rail-side arm. It is
 * the one movement given green during the clearing green.
 * -1 when the geometry is not configured.
 */
int rts_rail_exit_mv(int rail_arm);

/* 1 = safe, 0 = unsafe. 'why' is filled with a REJ_* code when unsafe.
   'blocked' is the mask above, or 0 when no train is expected. */
int rts_lamps_safe(const uint8_t veh[MV_COUNT],
                   const uint8_t ped[PD_COUNT],
                   uint8_t blocked,
                   uint8_t *why);

/* Set every vehicle lamp red and every pedestrian lamp DON'T WALK. */
void rts_lamps_all_red(uint8_t veh[MV_COUNT], uint8_t ped[PD_COUNT]);

/* Give the movements of one phase a colour, everything else stays red. */
void rts_lamps_phase(uint8_t veh[MV_COUNT], int phase, uint8_t colour);

/* The same, but every movement in 'blocked' is held at red. A phase
   with some of its movements blocked still runs, it just runs with
   fewer lamps lit. */
void rts_lamps_phase_masked(uint8_t veh[MV_COUNT], int phase,
                            uint8_t colour, uint8_t blocked);

#endif /* RTS_SAFETY_H */
