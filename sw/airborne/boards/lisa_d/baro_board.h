/*
 * board specific functions for the elle0 board
 *
 */

#ifndef BOARDS_LISA_D_BARO_H
#define BOARDS_LISA_D_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_MS5611_SPI
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_LISA_D_BARO_H */
