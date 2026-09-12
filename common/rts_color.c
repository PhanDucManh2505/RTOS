#include "rts_color.h"
#include "rts_proto.h"
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

int rts_color_on(void)
{
    static int cached = -1;
    if (cached < 0) {
        const char *v = getenv("RTS_COLOR");
        cached = (v != NULL && v[0] == '0') ? 0 : 1;
    }
    return cached;
}

const char *C(const char *seq)
{
    return rts_color_on() ? seq : "";
}

const char *rts_lamp_color(uint8_t lamp)
{
    switch (lamp) {
        case LAMP_GREEN: return C(A_GREEN);
        case LAMP_AMBER: return C(A_AMBER);
        default:         return C(A_LAMP_RED);
    }
}

const char *rts_ped_color(uint8_t ped)
{
    switch (ped) {
        case PED_WALK:  return C(A_GREEN);
        case PED_FLASH: return C(A_AMBER);
        default:        return C(A_LAMP_RED);
    }
}

const char *rts_state_color(uint8_t state)
{
    switch (state) {
        case ST_GREEN:
        case ST_PRE_CLEAR: return C(A_GREEN);    /* both show a green */
        case ST_AMBER:     return C(A_AMBER);
        case ST_ALLRED:    return C(A_LAMP_RED);
        default:           return C(A_GREY);     /* STARTUP */
    }
}

const char *rts_xing_color(uint8_t state)
{
    switch (state) {
        case XS_CLEAR:   return C(A_GREEN);
        case XS_WARNING: return C(A_AMBER);
        case XS_CLOSED:  return C(A_RED);
        default:         return C(A_BG_RED);   /* FAULT stands out */
    }
}

const char *rts_gate_color(uint8_t pos)
{
    switch (pos) {
        case GATE_UP:   return C(A_GREEN);
        case GATE_DOWN: return C(A_RED);
        default:        return C(A_AMBER);     /* MOVING */
    }
}

/* ------------------------------------------------------------------ */
/* strings built in the caller's buffer                                */
/* ------------------------------------------------------------------ */

size_t rts_vappend(char *buf, size_t n, size_t used, const char *fmt,
                   va_list ap)
{
    int w;

    if (n == 0 || used + 1 >= n) {
        return used;
    }
    w = vsnprintf(buf + used, n - used, fmt, ap);
    if (w < 0) {
        buf[used] = '\0';
        return used;
    }
    used += (size_t)w;
    return (used < n) ? used : n - 1;
}

size_t rts_append(char *buf, size_t n, size_t used, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    used = rts_vappend(buf, n, used, fmt, ap);
    va_end(ap);
    return used;
}

char *rts_lamps_str(char *buf, size_t n, const uint8_t veh[MV_COUNT])
{
    /* Phase order, so the green steps left to right as the cycle runs. */
    static const int order[MV_COUNT] = {
        MV_NS, MV_SN,   MV_NW, MV_SE,   MV_EW, MV_WE,   MV_WS, MV_EN
    };
    size_t used = 0;
    int    i;

    if (n == 0) return buf;
    buf[0] = '\0';
    for (i = 0; i < MV_COUNT; i++) {
        const char *name = rts_mv_name(order[i]);
        uint8_t     lamp = veh[order[i]];
        char        low[3];

        if (!rts_color_on() && lamp == LAMP_RED) {
            /* No colour to carry the lamp, so the case does: a movement
               held at red is written in lower case. */
            low[0] = (char)tolower((unsigned char)name[0]);
            low[1] = (char)tolower((unsigned char)name[1]);
            low[2] = '\0';
            name   = low;
        }
        used = rts_append(buf, n, used, "%s%s%s%s",
                          i == 0 ? "" : (i % 2 == 0 ? "  " : " "),
                          rts_lamp_color(lamp), name, C(A_RESET));
    }
    return buf;
}

char *rts_peds_str(char *buf, size_t n, const uint8_t ped[PD_COUNT])
{
    static const char arm[PD_COUNT] = { 'N', 'S', 'E', 'W' };
    size_t used = 0;
    int    i;

    if (n == 0) return buf;
    buf[0] = '\0';
    for (i = 0; i < PD_COUNT; i++) {
        char c = arm[i];

        if (!rts_color_on()) {
            /* the letters the plain log has always used */
            c = (ped[i] == PED_WALK)  ? 'W' :
                (ped[i] == PED_FLASH) ? 'F' : '-';
        }
        used = rts_append(buf, n, used, "%s%s%c%s", i == 0 ? "" : " ",
                          rts_ped_color(ped[i]), c, C(A_RESET));
    }
    return buf;
}

char *rts_tracks_str(char *buf, size_t n, uint8_t busy)
{
    size_t used = 0;
    int    t;

    if (n == 0) return buf;
    buf[0] = '\0';
    for (t = 0; t < 2; t++) {
        int  on = (busy & (1u << t)) != 0;
        char c  = (on || rts_color_on()) ? (char)('A' + t) : '-';

        used = rts_append(buf, n, used, "%s%s%c%s", t == 0 ? "" : " ",
                          on ? C(A_RED) : C(A_GREY), c, C(A_RESET));
    }
    return buf;
}

void rts_out(const char *s, size_t n)
{
    while (n > 0) {
        ssize_t w = write(STDOUT_FILENO, s, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            return;              /* the terminal has gone, nothing to do */
        }
        s += w;
        n -= (size_t)w;
    }
}

/* ------------------------------------------------------------------ */
/* full-screen panels                                                  */
/* ------------------------------------------------------------------ */

void rts_frame_begin(rts_frame_t *f)
{
    f->len    = 0;
    f->buf[0] = '\0';
    rts_frame_add(f, "%s%s%s", C(A_HIDE), C(A_NOWRAP), C(A_HOME));
}

void rts_frame_add(rts_frame_t *f, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    f->len = rts_vappend(f->buf, sizeof(f->buf), f->len, fmt, ap);
    va_end(ap);
}

void rts_frame_eol(rts_frame_t *f)
{
    rts_frame_add(f, "%s%s\n", C(A_RESET), C(A_ERASE));
}

void rts_frame_line(rts_frame_t *f, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    f->len = rts_vappend(f->buf, sizeof(f->buf), f->len, fmt, ap);
    va_end(ap);
    rts_frame_eol(f);
}

void rts_frame_bar(rts_frame_t *f, const char *bg, const char *left,
                   const char *right, int width)
{
    int pad = width - (int)strlen(left) - (int)strlen(right);
    if (pad < 1) pad = 1;
    rts_frame_line(f, "%s%s%*s%s", C(bg), left, pad, "", right);
}

void rts_frame_key(rts_frame_t *f, const char *key, const char *what)
{
    if (rts_color_on()) {
        rts_frame_add(f, "%s %s %s %s", A_KEY, key, A_RESET, what);
    } else {
        rts_frame_add(f, "[%s] %s", key, what);
    }
}

void rts_frame_keys(rts_frame_t *f, int label_w, const char *section, ...)
{
    va_list     ap;
    const char *key;

    rts_frame_add(f, "  %s%-*s%s", C(A_GREY), label_w, section, C(A_RESET));
    va_start(ap, section);
    while ((key = va_arg(ap, const char *)) != NULL) {
        rts_frame_key(f, key, va_arg(ap, const char *));
        rts_frame_add(f, "  ");
    }
    va_end(ap);
    rts_frame_eol(f);
}

void rts_frame_end(rts_frame_t *f)
{
    rts_frame_add(f, "%s%s", C(A_RESET), C(A_ERASE_DOWN));
    rts_out(f->buf, f->len);
}
