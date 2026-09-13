/*
 * rts_color.h - ANSI escape codes so the terminal output is readable.
 *
 * These work in an SSH session from Windows PowerShell or Windows
 * Terminal. Set RTS_COLOR=0 in the environment to turn colour off, for
 * example when the output is being piped into a file.
 *
 * The signal colours are 256-colour codes rather than the basic eight,
 * so a lamp shows its real colour whatever palette the terminal uses:
 * the basic "yellow" comes out pale cream in Windows Terminal, not amber.
 * One meaning per colour, everywhere:
 *   green  go, up, clear        amber  changing, warned, refused
 *   red    stop, down, fault    magenta  the operator has taken over
 *   cyan   names (I1, X1)       grey   headings and secondary text
 */
#ifndef RTS_COLOR_H
#define RTS_COLOR_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include "rts_proto.h"

#define A_RESET    "\033[0m"
#define A_BOLD     "\033[1m"
#define A_DIM      "\033[2m"
#define A_RED      "\033[1;38;5;196m"    /* alarms: DOWN, FAULT, CLOSED  */
#define A_AMBER    "\033[1;38;5;214m"    /* amber lamps, WARNING         */
#define A_GREEN    "\033[1;38;5;40m"     /* green lamps, up, CLEAR       */
#define A_LAMP_RED "\033[38;5;160m"      /* a red lamp: normal, not news */
#define A_BLUE     "\033[1;38;5;33m"
#define A_MAGENTA  "\033[1;38;5;170m"    /* override                     */
#define A_CYAN     "\033[1;38;5;45m"     /* names: I1, X1                */
#define A_WHITE    "\033[1;97m"
#define A_GREY     "\033[38;5;245m"      /* secondary text               */
#define A_HEAD     "\033[4;38;5;245m"    /* table headings, underlined   */
#define A_KEY      "\033[30;48;5;250m"   /* a key cap, dark on light     */
#define A_BG_RED   "\033[1;97;48;5;160m"
#define A_BG_BLUE  "\033[1;97;48;5;25m"

#define A_CLEAR   "\033[2J\033[H"   /* clear screen, cursor to top left */
#define A_HOME    "\033[H"          /* cursor to top left, no clear     */
#define A_ERASE   "\033[K"          /* erase to end of line             */
#define A_ERASE_DOWN "\033[J"       /* erase to end of screen           */
#define A_HIDE    "\033[?25l"
#define A_SHOW    "\033[?25h"
#define A_NOWRAP  "\033[?7l"        /* cut long lines instead of wrapping */
#define A_WRAP    "\033[?7h"

/* 1 unless RTS_COLOR=0. */
int rts_color_on(void);
/* Returns the escape code, or an empty string when colour is off. */
const char *C(const char *seq);

/* Colours that match a lamp_t, pedlamp_t, fsm_state_t, crossing state
   and gate position. */
const char *rts_lamp_color(uint8_t lamp);
const char *rts_ped_color(uint8_t ped);
const char *rts_state_color(uint8_t state);
const char *rts_xing_color(uint8_t state);
const char *rts_gate_color(uint8_t pos);

/*
 * The strings below are written into the caller's buffer, so any number
 * of them can go into one printf(). Each one is always the same width on
 * screen, whatever the escape codes add, so columns stay straight.
 */

/* The eight vehicle lamps, grouped by the phase that releases them,
   each movement name printed in its lamp colour:
       NS SN  NW SE  EW WE  WS EN        (phase A, B, C, D)            */
#define RTS_LAMPS_W 26
char *rts_lamps_str(char *buf, size_t n, const uint8_t veh[MV_COUNT]);

/* The four pedestrian lamps, "N S E W", each arm in its lamp colour. */
#define RTS_PEDS_W 7
char *rts_peds_str(char *buf, size_t n, const uint8_t ped[PD_COUNT]);

/* The two tracks of a crossing, "A B", red where a train is on it. */
#define RTS_TRACKS_W 3
char *rts_tracks_str(char *buf, size_t n, uint8_t busy);

/* Append printf-style text to a string being built in buf[n]. Returns
   the new length; a full buffer truncates instead of overflowing. */
size_t rts_append(char *buf, size_t n, size_t used, const char *fmt, ...);
size_t rts_vappend(char *buf, size_t n, size_t used, const char *fmt,
                   va_list ap);

/* Write all of it to stdout in as few write() calls as the terminal
   allows, so another process or thread cannot cut into the middle. */
void rts_out(const char *s, size_t n);

/*
 * A full-screen panel, built in memory and written in one go, so the
 * terminal never shows half of one frame and half of the next.
 *
 *   rts_frame_begin(&f);
 *   rts_frame_line(&f, "...");      one line, erased to the right edge
 *   rts_frame_end(&f);              erase what is left below, write it
 *
 * Every line ends with a colour reset before the erase. An erase paints
 * with the current background, so a badge left open would bleed.
 */
typedef struct {
    char   buf[16384];
    size_t len;
} rts_frame_t;

void rts_frame_begin(rts_frame_t *f);
void rts_frame_add(rts_frame_t *f, const char *fmt, ...);   /* no newline */
void rts_frame_eol(rts_frame_t *f);
void rts_frame_line(rts_frame_t *f, const char *fmt, ...);
/* A title bar `width` columns wide: left text, right text flush right. */
void rts_frame_bar(rts_frame_t *f, const char *bg, const char *left,
                   const char *right, int width);
/* " f " as a key cap, then " fixed". Takes strlen(key) + 3 + strlen(what)
   columns. */
void rts_frame_key(rts_frame_t *f, const char *key, const char *what);
/* A whole line of key help: the section name in grey, padded to
   label_w, then key / description pairs until a NULL key. */
void rts_frame_keys(rts_frame_t *f, int label_w, const char *section, ...);
void rts_frame_end(rts_frame_t *f);

#endif /* RTS_COLOR_H */
