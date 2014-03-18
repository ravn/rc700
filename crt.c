/*
 * RC700  -  a Regnecentralen RC700 simulator
 *
 * Copyright (C) 2012 by Michael Ringgaard
 *
 * Intel 8275 - Programmable CRT Controller
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "sim.h"
#include "simglb.h"

#define W(x) x
#define LL(x)
#define L(x)
#define D(x)

#define MAX_PARAMS 5

#define CRT_CMD_RESET           0
#define CRT_CMD_START_DISPLAY   1
#define CRT_CMD_STOP_DISPLAY    2
#define CRT_CMD_READ_LIGHT_PEN  3
#define CRT_CMD_LOAD_CURSOR     4
#define CRT_CMD_INTR_ENABLE     5
#define CRT_CMD_INTR_DISABLE    6
#define CRT_CMD_PRESET          7

#define CRT_STAT_FO 0x01 /* FIFO Overrun */
#define CRT_STAT_DU 0x02 /* DMA Underrun */
#define CRT_STAT_VE 0x04 /* Video Enabled */
#define CRT_STAT_IC 0x08 /* Improper Command */
#define CRT_STAT_LP 0x10 /* Light Pen */
#define CRT_STAT_IR 0x20 /* Interrupt Request */
#define CRT_STAT_IE 0x40 /* Interrupt Enabled */

static int crt_parameter_count[] = {
  5,  /*  0 */
  1,  /*  1 */
  1,  /*  2 */
  3,  /*  3 */
  3,  /*  4 */
  1,  /*  5 */
  1,  /*  6 */
  1,  /*  7 */
};

struct crt_controller {
  BYTE status;
  BYTE param[MAX_PARAMS];
  int param_index;
  int x, y;
  int rows, cols;
  BYTE *screen;
  BYTE *prev;
};

struct crt_controller crt;

void crt_command(int cmd) {
  int x, y;
  int intr = 0;
  switch (cmd) {
    case CRT_CMD_RESET:
      crt.status = 0;
      crt.cols = (crt.param[1] & 0x7F) + 1;
      crt.rows = (crt.param[2] & 0x3F) + 1;
      L(printf("crt: reset, cols=%d, rows=%d\n", crt.cols, crt.rows));
      crt.screen = NULL;
      crt.prev = realloc(crt.prev, crt.cols * crt.rows);
      memset(crt.prev, ' ', crt.cols * crt.rows);
      break;
    case CRT_CMD_START_DISPLAY: 
      L(printf("crt: start display\n"));
      crt.status |= CRT_STAT_IE | CRT_STAT_VE;
      rcterm_clear_screen(crt.cols, crt.rows);
      intr = 1;
      break;
    case CRT_CMD_STOP_DISPLAY: 
      L(printf("crt: stop display\n"));
      crt.status &= ~CRT_STAT_VE;
      break;
    case CRT_CMD_READ_LIGHT_PEN: 
      L(printf("crt: read light pen\n"));
      break;
    case CRT_CMD_LOAD_CURSOR: 
      // Use cursor movement to simulate simple console.
      x = crt.param[1];
      y = crt.param[2];
      crt.x = x;
      crt.y = y;
      rcterm_gotoxy(x, y);
      break;
    case CRT_CMD_INTR_ENABLE: 
      L(printf("crt: intr enable\n"));
      crt.status |= CRT_STAT_IE; 
      break;
    case CRT_CMD_INTR_DISABLE: 
      L(printf("crt: intr disable\n"));
      crt.status &= ~CRT_STAT_IE; 
      break;
    case CRT_CMD_PRESET: 
      L(printf("crt: preset\n"));
      break;
  }

  if (intr && (crt.status & CRT_STAT_IE)) {
    // In RC702, the CRT INT pin is connected to the CLK/TRG2 on the CTC
    ctc_trigger(CTC_CHANNEL_CRT);
  }
}

BYTE crt_param_in(int dev)  {
  LL(printf("crt: param in\n"));
  return 0; 
}

void crt_param_out(BYTE data, int dev) {
  LL(printf("crt: param out %02X\n", data));
  crt.param[crt.param_index++] = data;
  if (crt.param_index == crt_parameter_count[crt.param[0] >> 5]) {
    crt_command(crt.param[0] >> 5);
    crt.param_index = 0;
  }
}

BYTE crt_status_in(int dev) {
  L(printf("crt: read status %02X\n", crt.status));
  return crt.status;
}

void crt_command_out(BYTE data, int dev) {
  int cmd;

  LL(printf("crt: command %02X\n", data));
  cmd = data >> 5;
  crt.param[0] = data;
  crt.param_index = 1;
  if (crt.param_index == crt_parameter_count[cmd]) {
    crt_command(cmd);
    crt.param_index = 0;
  }
}

int crt_poll(void) {
  WORD cnt;
  WORD adr;

  // Only update screen if interrupts and video are enabled.
  if (!IFF) return 0;
  if (!(crt.status & CRT_STAT_VE)) return 0;

  // Fetch screen buffer from DMA channel.
  adr = dma_fetch(2, &cnt);
  if (!cnt) return 0;
  cnt++;
  crt.screen = ram + adr;

  LL(printf("crt: refresh %04X, %d bytes\n", adr, cnt));

  // Update screen if screen buffer has changed.
  if (crt.prev && cnt == crt.cols * crt.rows &&
      memcmp(crt.screen, crt.prev, cnt) != 0) {
    rcterm_screen(crt.screen, crt.prev, crt.cols, crt.rows);
    memcpy(crt.prev, crt.screen, cnt);
  }

  // Trigger interrupt.
  if (!crt.status & CRT_STAT_IE) return 0;
  ctc_trigger(CTC_CHANNEL_CRT);
  return 1;
}

void dump_screen(void) {
  int x, y, lastx;
  BYTE *p, *start, *end;

  if (!crt.screen) {
    printf("<no display buffer>\n");
  } else {
    printf("%dx%d screen buffer at %04X:\n", crt.cols, crt.rows, crt.screen - ram);
    p = crt.screen;
    for (y = 0; y < crt.rows; ++y) {
      start = p;
      end = NULL;
      for (x = 0; x < crt.cols; ++x) {
        if (*p != ' ') end = p;
        p++;
      }
      if (end) fwrite(start, end - start + 1, 1, stdout);
      putchar('\n');
    }
  }
}

void init_crt(void) {
  register_port(0x00, crt_param_in, crt_param_out, 0);
  register_port(0x01, crt_status_in, crt_command_out, 0);
}

