/*
 * rts_proto.h - everything the three nodes need to agree on.
 *
 * EEET2588 Real-Time Systems - Traffic Light Control System
 * Team ANK : Phan Duc Manh (S4124156), Mai Quy Anh (S4118973),
 *            Vu Minh Khanh (S4117146)
 *
 * Rules for this file:
 *  - Fixed-width types only. Two nodes may be built differently, so an
 *    "int" is not guaranteed to be the same size on both sides.
 *  - Never put a pointer inside a message. The other end is a different
 *    process, usually on a different machine, so an address means nothing.
 *  - Every struct here is fixed size, so no memory is allocated at run time.
 */
#ifndef RTS_PROTO_H
#define RTS_PROTO_H

#include <stdint.h>
#include <sys/iomsg.h>     /* gives us _IO_BASE and _IO_MAX          */
#include <sys/neutrino.h>  /* gives us struct _pulse and pulse codes */

#define RTS_N_INTERSECTIONS 6
#define RTS_N_CROSSINGS     3

/* ------------------------------------------------------------------ */
/* Who sent a message (goes in the header)                            */
/* ------------------------------------------------------------------ */
enum {
    SND_UNKNOWN = 0,
    SND_CENTRAL = 1,
    SND_I1 = 11, SND_I2 = 12, SND_I3 = 13,
    SND_I4 = 14, SND_I5 = 15, SND_I6 = 16,
    SND_RAILWAY = 30
};

/* ------------------------------------------------------------------ */
/* Message types                                                       */
/*                                                                     */
/* They start above _IO_MAX so one of our messages can never be        */
/* confused with a system message such as _IO_CONNECT, which the       */
/* kernel sends to a server when a client calls name_open().           */
/* ------------------------------------------------------------------ */
#define MSG_BASE          (_IO_MAX + 1)

#define MSG_HEARTBEAT     (MSG_BASE + 1)   /* central -> intersection / railway */
#define MSG_SET_PATTERN   (MSG_BASE + 2)   /* central -> intersection           */
#define MSG_OVERRIDE      (MSG_BASE + 3)   /* central -> intersection           */
#define MSG_PED_BUTTON    (MSG_BASE + 4)   /* central -> intersection           */
#define MSG_QUERY_STATE   (MSG_BASE + 5)   /* central -> intersection           */
#define MSG_STATUS        (MSG_BASE + 6)   /* intersection -> central           */
#define MSG_XING_STATE    (MSG_BASE + 7)   /* railway -> intersection / central */
#define MSG_GATE_CMD      (MSG_BASE + 8)   /* central -> railway                */
#define MSG_GATE_FAULT    (MSG_BASE + 9)   /* railway -> central                */

/* ------------------------------------------------------------------ */
/* Pulse codes (a pulse carries one byte of code and four of value)    */
/* ------------------------------------------------------------------ */
#define PULSE_PHASE_TICK    (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_PED_TICK      (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_SENSOR_TICK   (_PULSE_CODE_MINAVAIL + 3)
#define PULSE_WAKE          (_PULSE_CODE_MINAVAIL + 4)
#define PULSE_XING_CHANGED  (_PULSE_CODE_MINAVAIL + 5)
#define PULSE_WATCHDOG      (_PULSE_CODE_MINAVAIL + 6)

/* ------------------------------------------------------------------ */
/* Lights                                                              */
/* ------------------------------------------------------------------ */
typedef enum { LAMP_RED = 0, LAMP_AMBER = 1, LAMP_GREEN = 2 } lamp_t;
typedef enum { PED_DONT = 0, PED_WALK = 1, PED_FLASH = 2 } pedlamp_t;

/*
 * Vehicle movements, named "from arm -> to arm".
 * NS means traffic that entered from the North arm and leaves South.
 * The four right turns get their own phase because they cross the
 * opposing stream (Australian rules, traffic keeps left).
 */
enum {
    MV_NS = 0, MV_SN,          /* through movements on R1 (the vertical road) */
    MV_EW,     MV_WE,          /* through movements on R3 (the rail-side road) */
    MV_NW,     MV_SE,          /* right turns out of R1 */
    MV_WS,     MV_EN,          /* right turns out of R3 */
    MV_COUNT
};

/*
 * The four arms of an intersection. Every movement above is named
 * "from arm -> to arm", so the arm a movement ends in is what says
 * whether it drives into the railway area. Each controller knows which
 * of its own arms the tracks cross (inter_cfg_t.rail_arm).
 */
typedef enum { ARM_NONE = 0, ARM_N, ARM_S, ARM_E, ARM_W } arm_t;

/* Pedestrian crossings, one on each arm. */
enum { PD_N = 0, PD_S, PD_E, PD_W, PD_COUNT };

/*
 * Phases. The order never changes: A -> B -> C -> D -> A.
 *   A : R1 through   + pedestrians on the E and W arms
 *   B : R1 right turns, no pedestrians
 *   C : R3 through   + pedestrians on the N and S arms
 *   D : R3 right turns, no pedestrians
 * R3 is the road that runs over the railway tracks, so phases C and D
 * are the ones that must be shut down when a train is coming.
 */
typedef enum { PH_A = 0, PH_B, PH_C, PH_D, PH_COUNT } phase_t;

/* State of the light state machine inside one local controller. */
typedef enum {
    ST_STARTUP = 0,   /* everything red while the process settles */
    ST_GREEN,
    ST_AMBER,
    ST_ALLRED,
    ST_PRE_CLEAR,     /* railway pre-emption: green to flush the tracks */
    ST_COUNT
} fsm_state_t;

/* Light sequence patterns the central controller can select. */
typedef enum {
    PAT_FIXED = 0,    /* peak hour: fixed times, pedestrian buttons ignored */
    PAT_SENSOR,       /* off peak: react to vehicle demand and to buttons   */
    PAT_UPDATED,      /* advanced: green times supplied by the control room */
    PAT_COUNT
} pattern_t;

/* State published by a level crossing. */
typedef enum {
    XS_CLEAR = 0,     /* no train, gates up            */
    XS_WARNING,       /* train coming, gates still up  */
    XS_CLOSED,        /* gates down, train on the road */
    XS_FAULT          /* a gate did not reach position */
} xing_state_t;

/* Boom gate position. */
typedef enum { GATE_UP = 0, GATE_MOVING = 1, GATE_DOWN = 2 } gate_pos_t;

/* Actions the control room can ask the railway controller to take. */
enum {
    GC_NORMAL = 0,      /* release manual control            */
    GC_FORCE_DOWN,      /* hold the gates down               */
    GC_FORCE_UP,        /* hold the gates up (maintenance)   */
    GC_INJECT_FAULT,    /* simulate a gate that will not move */
    GC_CLEAR_FAULT,
    GC_REQUEST_TRAIN_A, /* send a test train on track A      */
    GC_REQUEST_TRAIN_B  /* send a test train on track B      */
};

/* Why a command was refused. */
enum {
    REJ_NONE = 0,
    REJ_UNKNOWN_PATTERN,
    REJ_MIN_GREEN,
    REJ_MAX_GREEN,
    REJ_CYCLE_TOO_LONG,
    REJ_CONFLICT,
    REJ_BAD_PHASE,
    REJ_PREEMPT_ACTIVE,
    REJ_RAIL_ENTRY,     /* would drive into the arm the tracks cross */
    REJ_COUNT
};

/* ------------------------------------------------------------------ */
/* Message header, shared by every message                             */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t type;     /* one of the MSG_* codes above                 */
    uint16_t sender;   /* one of the SND_* codes above                 */
    uint32_t seq;      /* +1 for every message, a gap means one was lost */
    uint64_t t_ns;     /* CLOCK_MONOTONIC time the sender stamped it   */
} rts_hdr_t;

/* Full picture of one intersection. Sent on every lamp change. */
typedef struct {
    uint8_t  inter_id;             /* 1..6                            */
    uint8_t  phase;                /* phase_t                         */
    uint8_t  state;                /* fsm_state_t                     */
    uint8_t  pattern;              /* pattern_t                       */
    uint8_t  veh[MV_COUNT];        /* lamp_t per movement             */
    uint8_t  ped[PD_COUNT];        /* pedlamp_t per crossing          */
    uint8_t  ped_req;              /* bitmask of buttons still waiting */
    uint8_t  train_hold;           /* 1 = rail road is being held red  */
    uint8_t  xing_state;           /* what this controller last read   */
    uint8_t  central_online;       /* 1 = heartbeat is arriving        */
    uint8_t  override_active;
    uint8_t  pad0;
    uint16_t green_s[PH_COUNT];    /* green time in force per phase    */
    uint32_t cycle_count;
} inter_status_t;

/* Full picture of one level crossing. */
typedef struct {
    uint8_t  xing_id;      /* 1..3                                   */
    uint8_t  state;        /* xing_state_t                           */
    uint8_t  track_busy;   /* bit0 = track A, bit1 = track B         */
    uint8_t  gate_pos;     /* gate_pos_t                             */
    uint8_t  gate_fault;   /* 1 = a gate is stuck                    */
    uint8_t  train_signal; /* 0 = red to the train, 1 = green        */
    uint8_t  manual;       /* 1 = the control room is holding it     */
    uint8_t  pad0;
    uint32_t seq;          /* +1 on every published change           */
    uint32_t trains_served;
    uint64_t t_ns;
} xing_status_t;

/* The one message struct that travels on every link. */
typedef struct {
    rts_hdr_t hdr;
    union {
        struct {
            uint8_t  pattern;              /* pattern_t                */
            uint8_t  pad0;
            uint16_t green_s[PH_COUNT];    /* 0 = keep the current value */
            uint16_t offset_s;
        } set_pattern;

        struct {
            uint8_t  phase;                /* phase group to hold green */
            uint8_t  cancel;               /* 1 = drop the override     */
            uint16_t timeout_s;
        } override_cmd;

        struct { uint8_t ped_id; } button; /* PD_N .. PD_W */

        struct { uint8_t xing_id; uint8_t action; } gate_cmd;

        struct { uint8_t xing_id; uint8_t gate; uint8_t train_stopped; } gate_fault;

        inter_status_t status;
        xing_status_t  xing;
    } u;
} rts_msg_t;

/* Every message gets exactly one reply. */
typedef struct {
    rts_hdr_t      hdr;
    int32_t        result;   /* 0 = accepted, negative = refused */
    uint8_t        reason;   /* one of the REJ_* codes           */
    uint8_t        pad[3];
    inter_status_t status;   /* only filled in for MSG_QUERY_STATE */
} rts_reply_t;

/*
 * MsgReceive() may hand us either a pulse or a real message, so the
 * receive buffer has to be big enough for both.
 */
typedef union {
    struct _pulse pulse;
    rts_msg_t     msg;
} rts_rcv_t;

const char *rts_lamp_name(uint8_t lamp);
const char *rts_phase_name(uint8_t phase);
const char *rts_state_name(uint8_t state);
const char *rts_pattern_name(uint8_t pattern);
const char *rts_xing_name(uint8_t state);
const char *rts_reject_name(uint8_t reason);
const char *rts_mv_name(int mv);
const char *rts_gate_name(uint8_t pos);
const char *rts_arm_name(int arm);

#endif /* RTS_PROTO_H */
