/*
 *
 * Copyright (C) 2009-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file arch/linux/mcu_arch.h
 * linux arch dependant microcontroller initialisation functions.
 *
 * Because Linux runs on omap, we don't have to initialize the MCU ourselves.
 */

#include "mcu_arch.h"

#if USE_LINUX_SIGNAL
#include "message_pragmas.h"
PRINT_CONFIG_MSG("Catching SIGINT. Press CTRL-C twice to stop program.")

/*
 * Handle linux signals by hand if the program is not launch
 * by a shell already doing it
 */

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

/**
 * catch SIGINT signal two times to stop the main program
 */
static void sig_handler(int signo)
{
  static int nb_signal = 0;

  if (signo == SIGINT) {
    printf("Received SIGINT\n");
    if (nb_signal == 0) {
      printf("Press Ctrl-C again to stop the program\n");
      nb_signal++;
    } else {
      printf("Leaving now\n");
      exit(0);
    }
  }
}

void mcu_arch_init(void)
{
  struct sigaction sa;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sa.sa_handler = &sig_handler;
  if (sigaction(SIGINT, &sa, NULL) == -1) {
    printf("Can't catch SIGINT\n");
  }
}

#else

void mcu_arch_init(void) { }

#endif

