/*
 * RC700  -  a Regnecentralen RC700 simulator
 *
 * Copyright (C) 2012 by Michael Ringgaard
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include "sim.h"
#include "simglb.h"
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

extern void init_io(void);
extern void exit_io(void);
extern int poll_crt(void);
extern int poll_pio(void);

#define MAX_FLOPPIES 4
char *floppy[MAX_FLOPPIES];
int num_floppies = 0;

int refresh_ticks = 100000;
int active_delay  =   0; //5;
int idle_delay    = 0; //200;

void delay(int ms) {
#ifdef WIN32
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

void cpu_poll() {
  static int tick = 0;
  static int active = 1000;

  if (--tick > 0) return;
  tick = refresh_ticks;

  if (!pio_poll() && !crt_poll()) {
    if (!active) {
      delay(idle_delay);
    } else {
      --active;
      delay(active_delay);
    }
  } else {
    active = 1000;
  }
}

static void init_rc700(void) {
  int i;

  wrk_ram = PC = STACK = ram;
  memset(ram, 0, 65536);

  init_rom();
  init_pio();
  init_sio();
  init_ctc();
  init_dma();
  init_crt();
  init_fdc();
  init_ftp();
  
  for (i = 0; i < num_floppies; ++i) {
    fdc_mount_disk(i, floppy[i]);
  }
}

static void simbreak(int sig) {
  cpu_error = USERINT;
  cpu_state = STOPPED;
}

int main(int argc, char *argv[]) {
#ifndef WIN32
  static struct sigaction sa;
#endif
  int suspend = 0;
  int monitor = 0;
  int i;

  char *s, *p;
  char *pn = argv[0];

  printf("%s Release %s, %s\n", USR_COM, USR_REL, USR_CPR);
  printf("Z80-SIM Release %s, %s\n\n", RELEASE, COPYR);
  fflush(stdout);

  for (i = 1; i < argc; ++i) {
    if (argv[i][0] == '-') {
      char *s = argv[i] + 1;
      while (s && *s) {
        switch (*s) {
          case 'm':
            monitor = 1;
            s++;
            break;

          case 's':
            suspend = 1;
            s++;
            break;

          case '?':
            goto usage;

          default:
            printf("illegal option %c\n", *s);
          usage:  
            printf("usage:\t%s -ms [IMD FILES...]\n", argv[0]);
            printf("-m (exit into monitor)\n");
            printf("-s (start suspended)\n");
            exit(1);
        }
      }
    } else {
      if (num_floppies == MAX_FLOPPIES) {
        fprintf(stderr, "Too many floppies\n");
        exit(1);
      }
      floppy[num_floppies++] = argv[i];
    }
  }

#ifndef WIN32
  sa.sa_handler = simbreak;
  sigaction(SIGINT, &sa, NULL);
#endif

  init_io();
  init_rc700();

  if (suspend) {
    mon();
    rcterm_init();
    cpu();
    rcterm_exit();
  } else if (monitor) {
    rcterm_init();
    cpu_state = CONTIN_RUN;
    cpu_error = NONE;
    cpu();
    rcterm_exit();
    mon();
  } else {
    rcterm_init();
    cpu_state = CONTIN_RUN;
    cpu_error = NONE;
    cpu();
    rcterm_exit();
  }

  exit_io();

#ifndef WIN32
  sa.sa_handler = SIG_DFL;
  sigaction(SIGINT, &sa, NULL);
#endif
  
  for (i = 0; i < num_floppies; ++i) {
    if (floppy[i]) fdc_flush_disk(i, floppy[i]);
  }

  return 0;
}

