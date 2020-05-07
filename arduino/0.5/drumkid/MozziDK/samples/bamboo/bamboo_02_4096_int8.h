#ifndef BAMBOO_02_4096_H_
#define BAMBOO_02_4096_H_

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "mozzi_pgmspace.h"

#define BAMBOO_02_4096_NUM_CELLS 4096
#define BAMBOO_02_4096_SAMPLERATE 16384

CONSTTABLE_STORAGE(int8_t) BAMBOO_02_4096_DATA []  = {0, 0, -1, 0, -1, 1, 1, -1, 0, -1,
                -1, -6, -26, -68, -95, -66, -4, 11, 2, 43, 113, 126, 67, 6, -21, -26, -5, 23,
                23, -7, -61, -90, -47, 5, 8, -1, -5, -45, -90, -37, 69, 90, 41, 2, -20, -25, -7,
                18, 36, 44, 19, -27, -19, 37, 50, 25, 0, -30, -43, -11, 35, 36, 8, -26, -68,
                -68, -10, 39, 33, -1, -36, -60, -36, 29, 53, 21, -14, -36, -34, 3, 52, 73, 52,
                -2, -54, -48, 17, 60, 47, 16, -8, -29, -30, 2, 31, 11, -33, -54, -43, -8, 22,
                20, -9, -36, -47, -30, 12, 40, 24, -5, -21, -24, -7, 28, 49, 31, -4, -16, 0, 25,
                37, 26, 4, -14, -27, -25, 0, 27, 22, -8, -33, -37, -26, -14, -4, -1, -8, -16,
                -18, -7, 6, 10, 0, -9, -7, 0, 13, 29, 32, 19, 4, 1, 6, 12, 22, 24, 8, -12, -20,
                -12, 5, 9, -3, -22, -28, -25, -17, -1, 7, -2, -23, -35, -27, -5, 13, 18, 9, -3,
                -7, 0, 19, 33, 29, 13, 2, 4, 16, 23, 20, 7, -9, -20, -14, 0, 7, 0, -9, -20, -27,
                -24, -12, 0, -4, -19, -27, -21, -7, 6, 11, 9, -1, -7, -3, 14, 28, 32, 26, 10,
                -2, 0, 13, 22, 19, 6, -8, -16, -15, -8, -1, -4, -17, -24, -21, -14, -2, 1, -6,
                -19, -23, -18, -5, 10, 15, 9, -1, -3, 7, 25, 33, 26, 10, -1, -1, 10, 23, 25, 13,
                -8, -21, -17, -3, 10, 6, -11, -28, -33, -24, -6, 1, -4, -14, -22, -19, -4, 10,
                14, 7, -5, -8, 5, 24, 30, 27, 14, 3, -2, 7, 19, 24, 12, -7, -17, -14, 0, 10, 5,
                -12, -26, -31, -24, -11, -3, -5, -13, -20, -18, -8, 5, 11, 7, -1, 0, 7, 15, 19,
                18, 15, 11, 9, 9, 15, 15, 7, -4, -10, -9, -4, -1, -3, -9, -15, -21, -23, -16,
                -8, -8, -13, -15, -12, -6, 3, 9, 9, 6, 1, 4, 10, 15, 17, 17, 11, 8, 7, 9, 10, 9,
                4, -1, -5, -5, -7, -6, -6, -11, -16, -16, -14, -13, -10, -12, -12, -11, -8, -2,
                4, 9, 9, 7, 6, 8, 9, 13, 16, 11, 7, 8, 8, 7, 9, 9, 1, -4, -7, -9, -8, -6, -8,
                -12, -14, -14, -12, -10, -7, -8, -10, -9, -4, -1, 4, 6, 7, 7, 8, 9, 10, 12, 10,
                7, 6, 6, 7, 7, 8, 6, -1, -5, -5, -6, -8, -9, -11, -15, -16, -14, -10, -6, -4,
                -7, -9, -8, -3, 2, 6, 6, 3, 5, 8, 9, 12, 13, 9, 2, 1, 5, 8, 11, 7, 0, -5, -7,
                -3, -1, -3, -9, -15, -16, -14, -10, -6, -5, -8, -11, -8, -3, 2, 5, 6, 2, 0, 3,
                8, 12, 13, 13, 7, 5, 5, 8, 8, 6, 1, -2, -3, -5, -4, -5, -6, -10, -15, -13, -10,
                -9, -10, -9, -7, -7, -5, -1, 3, 4, 4, 5, 6, 9, 11, 10, 10, 8, 8, 9, 8, 5, 2, 0,
                -1, -3, -5, -6, -7, -8, -9, -9, -9, -8, -9, -9, -8, -7, -5, -4, -1, 0, 3, 4, 5,
                7, 8, 7, 8, 9, 11, 8, 6, 6, 6, 4, 2, -1, -4, -5, -7, -7, -6, -8, -11, -12, -9,
                -8, -7, -6, -6, -7, -5, -3, 0, 2, 5, 5, 6, 6, 7, 9, 9, 8, 7, 6, 4, 3, 3, 2, 1,
                -2, -3, -5, -6, -6, -8, -8, -8, -8, -7, -8, -7, -6, -5, -4, -1, 0, 2, 1, 2, 5,
                8, 9, 7, 7, 5, 5, 5, 5, 5, 4, 1, 0, 0, 0, -1, -2, -5, -7, -8, -8, -8, -8, -8,
                -9, -9, -6, -4, -3, 0, 1, 2, 1, 4, 6, 6, 8, 7, 6, 6, 5, 5, 5, 5, 2, -1, -1, -2,
                -3, -4, -4, -4, -6, -9, -10, -9, -8, -6, -7, -6, -4, -2, -2, 2, 3, 3, 4, 5, 4,
                5, 6, 6, 5, 5, 4, 4, 5, 4, 0, -1, -2, -5, -4, -4, -4, -6, -7, -9, -7, -5, -5,
                -6, -5, -4, -2, -1, 2, 4, 4, 3, 2, 4, 6, 7, 7, 4, 3, 4, 4, 4, 4, 3, 0, -2, -3,
                -3, -2, -3, -6, -9, -9, -7, -5, -4, -5, -5, -5, -4, -1, 1, 4, 3, 3, 3, 4, 5, 6,
                6, 4, 3, 1, 3, 4, 4, 2, 0, -1, -1, -3, -3, -4, -5, -6, -7, -7, -5, -5, -4, -5,
                -4, -3, -2, 0, 1, 3, 3, 1, 3, 4, 4, 6, 5, 5, 3, 2, 2, 2, 1, 1, 0, -3, -5, -5,
                -4, -4, -5, -6, -8, -7, -5, -3, -2, -3, -4, -3, 0, 1, 2, 3, 2, 2, 3, 4, 5, 6, 4,
                4, 1, 2, 3, 2, 2, -1, -2, -4, -3, -3, -3, -4, -6, -7, -6, -4, -4, -2, -3, -3,
                -2, 0, 1, 3, 3, 3, 2, 3, 3, 5, 7, 6, 3, 2, 2, 2, 2, 0, -2, -4, -5, -4, -4, -3,
                -4, -4, -4, -6, -5, -4, -2, -2, -3, -2, 0, 2, 3, 3, 2, 3, 3, 3, 3, 3, 3, 3, 1,
                1, 1, 0, 1, 0, -2, -3, -4, -4, -4, -4, -4, -4, -4, -4, -3, -2, -1, -3, -2, -1,
                -1, 1, 2, 3, 2, 3, 2, 3, 3, 4, 3, 2, 1, 1, 0, 1, -1, -1, -4, -4, -4, -4, -3, -3,
                -4, -5, -4, -3, -2, -1, -1, -1, -1, 0, 1, 2, 3, 3, 2, 3, 3, 4, 4, 3, 2, 0, -1,
                0, -1, 0, 0, -2, -3, -4, -4, -4, -4, -3, -4, -4, -3, -3, -2, -1, -1, 0, 0, 1, 1,
                3, 3, 3, 3, 3, 2, 1, 1, 2, 1, 1, 0, -1, -1, -2, -2, -2, -4, -3, -4, -4, -3, -2,
                -2, -3, -3, -3, -1, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 1, 2, 2, 2, 1, -1, -1, -2,
                -1, -2, -2, -3, -3, -4, -3, -3, -3, -3, -3, -3, -2, -1, 0, 1, 0, 0, 0, 1, 2, 3,
                4, 2, 2, 1, 2, 1, 2, 1, 0, -1, -2, -2, -3, -2, -2, -3, -3, -3, -3, -3, -1, -2,
                -3, -3, -1, 0, -1, 1, 2, 1, 2, 2, 2, 2, 2, 2, 0, 2, 1, 1, 1, 0, -2, -2, -3, -2,
                -2, -2, -3, -3, -4, -3, -2, -1, -2, -2, -2, -1, 0, 0, 2, 2, 2, 1, 2, 2, 2, 2, 2,
                2, 0, 1, 0, 0, -1, 0, -2, -2, -2, -2, -3, -3, -3, -3, -3, -3, -3, -3, -2, -1,
                -1, 0, 0, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 0, -1, 0, 0, -1, -2, -2, -3, -3,
                -3, -3, -2, -2, -3, -2, -2, -1, 0, -1, 0, 0, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0,
                0, 0, 0, 0, 0, -2, -3, -3, -3, -3, -2, -3, -3, -3, -3, -2, -1, -1, 0, 0, 0, 1,
                2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 0, 0, -1, 0, 0, -2, -1, -3, -3, -3, -2, -3, -2,
                -3, -3, -2, -3, -1, -1, 0, -1, 0, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0,
                -1, -1, -2, -2, -3, -3, -3, -3, -2, -3, -3, -3, -2, -1, 0, -1, 0, 0, 0, 1, 2, 2,
                2, 2, 2, 2, 2, 1, 2, 1, 0, -1, 0, -1, -2, -2, -3, -3, -3, -3, -2, -3, -3, -2,
                -2, -1, -1, -1, 0, -1, 0, 0, 1, 1, 2, 2, 2, 2, 2, 2, 2, 1, 1, 0, -1, 0, 0, -1,
                -2, -2, -3, -3, -3, -3, -3, -3, -3, -2, -2, -1, -1, 0, -1, 0, 0, 2, 2, 2, 2, 2,
                1, 2, 1, 1, 1, 0, 0, -1, -1, -1, -1, -2, -2, -3, -3, -3, -2, -3, -2, -2, -2, -1,
                -1, 0, 0, 0, -1, 0, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, 0, -1, -2, -2, -2,
                -3, -2, -3, -3, -2, -2, -2, -2, -2, -2, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 0, -1, -1, -2, -2, -1, -2, -2, -3, -2, -2, -2, -2, -2, -2, -2, -2, -1,
                0, -1, 1, 1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0, -1, -1, -1, -2, -2, -2, -3,
                -3, -3, -2, -2, -2, -2, -2, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
                0, 0, -1, -1, -1, -2, -2, -2, -2, -3, -2, -2, -2, -2, -2, -2, -2, 0, -1, 0, 0,
                1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 0, 0, 0, -1, -2, -1, -2, -2, -2, -2, -2, -3,
                -2, -3, -2, -1, -1, -2, -1, -1, 0, 0, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1, 0, 0,
                -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -2, -1, -1, 0, 0, 0, 1,
                1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, -1, 0, -1, -1, -2, -1, -2, -2, -2, -2, -3, -2,
                -2, -2, -1, -2, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0,
                -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1, 0, 0, 0, -1, 0, -1, -1, -2, -2, -2, -2, -2, -2, -3, -2, -2,
                -2, -1, -1, -1, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -2,
                -1, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, 0, -1, 0, 0, 1, 0, 1, 1, 1,
                1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -2, -2, -2, -1, -2, -2, -2, -2, -2, -1, -2,
                -1, 0, 0, -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, 0, -2, -2, -2,
                -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1,
                1, 1, 0, 0, -1, -1, -1, 0, -2, -2, -2, -1, -2, -2, -2, -2, -2, -2, -2, -1, -1,
                0, -1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, -1, 0, -1, -2, -1, -2, -2,
                -2, -2, -2, -2, -2, -1, -1, -1, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0,
                1, 0, 0, -1, -1, -1, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1, 0, -1, 0, 0, 0,
                0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, -1, -1, -1, -2, -1, -2, -2, -1, -2, -2, -2,
                -2, -1, -1, -2, -1, -1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, -1,
                -1, -1, -1, -1, -1, -2, -2, -2, -2, -1, -2, -1, -2, -1, -1, 0, -1, 0, 0, 1, 0,
                1, 0, 1, 0, 1, 1, 0, 1, 0, -1, 0, -1, -1, -1, -1, -2, -2, -2, -1, -2, -2, -2,
                -2, -2, -1, -1, -1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, -1,
                -1, -1, -2, -1, -1, -2, -2, -2, -2, -2, -2, -2, 0, -1, -1, -1, 0, 0, 1, 0, 1, 0,
                1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, -2, -2, -1, -2, -1, -2, -2, -2, -1, -1, -1,
                -1, -1, 0, -1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, -1, -1, -1, -1,
                -2, -1, -2, -1, -1, -1, -1, -2, -1, -1, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                -1, 0, 0, 0, -1, 0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0,
                -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1,
                0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, 0, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, -1, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, -1, -1, -1,
                -1, -1, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, -1, 0, 0, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1,
                0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, -1, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0,
                0, 0, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, 0, 0,
                0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0,
                0, -1, 0, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0,
                0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1,
                -1, -1, 0, -1, 0, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, 0, 0,
                -1, 0, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, -1, -1, 0, -1, 0, -1,
                -1, -1, 0, 0, -1, -1, -1, 0, 0, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
                0, -1, 0, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, -1,
                0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, -1, -1, -1, -1, -1,
                -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0,
                -1, 0, -1, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0,
                -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, -1, 0, -1, -1, 0, -1, -1, 0,
                -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1,
                0, -1, -1, -1, 0, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
                0, 0, 0, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, -1, -1, -1, -1, 0, -1, 0, 0,
                0, 0, 0, 0, -1, 0, 0, 0, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0, -1, -1, -1, -1,
                -1, -1, -1, 0, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
                0, 0, 0, -1, 0, -1, -1, -1, 0, 0, 0, -1, -1, -1, -1, 0, -1, -1, 0, -1, 0, -1, 0,
                -1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1,
                -1, -1, 0, -1, -1, -1, 0, 0, -1, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0, 0, -1, 0, 0,
                -1, 0, -1, -1, 0, -1, 0, -1, -1, -1, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
                0, -1, 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0,
                -1, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, -1, -1,
                -1, -1, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, 0,
                0, 0, 0, 0, 0, 0, -1, -1, -1, 0, -1, 0, -1, -1, -1, 0, 0, -1, -1, -1, 0, -1, 0,
                -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, -1, -1, 0, -1, -1,
                -1, -1, -1, -1, 0, -1, 0, -1, -1, -1, 0, -1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0,
                0, -1, 0, -1, -1, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, -1, -1, 0, -1, 0, -1,
                0, 0, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, -1, -1, -1, -1,
                -1, -1, 0, -1, 0, -1, 0, 0, -1, -1, -1, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, -1,
                0, 0, 0, -1, 0, -1, 0, -1, -1, -1, 0, -1, 0, -1, -1, 0, 0, 0, -1, 0, -1, 0, -1,
                0, -1, 0, -1, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0, -1, -1, -1, 0, -1,
                0, -1, 0, 0, -1, 0, 0, -1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, -1, 0, -1, 0,
                -1, 0, -1, -1, 0, -1, -1, -1, 0, -1, -1, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0,
                -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1,
                0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, -1, -1, -1, 0,
                -1, 0, -1, -1, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0,
                -1, 0, 0, -1, -1, 0, -1, -1, -1, 0, -1, -1, 0, -1, 0, -1, -1, -1, 0, -1, -1, 0,
                0, -1, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, -1, -1, -1, 0, -1,
                0, -1, 0, -1, 0, 0, -1, -1, -1, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0, 0, -1, 0,
                0, -1, -1, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0,
                0, 0, -1, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, -1, -1, 0, -1, 0, 0, -1, 0, 0,
                0, -1, -1, -1, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0,
                -1, 0, -1, -1, -1, -1, -1, 0, -1, -1, -1, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1,
                0, 0, -1, 0, -1, 0, -1, -1, 0, -1, 0, -1, 0, 0, 0, 0, -1, 0, 0, -1, 0, -1, 0,
                -1, 0, -1, 0, -1, 0, 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0,
                -1, 0, 0, -1, 0, -1, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, -1, 0, -1, 0, 0,
                0, -1, 0, -1, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0,
                0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, -1, 0, -1, 0, 0, -1, 0, 0, -1, 0,
                -1, 0, -1, 0, -1, 0, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, 0, 0, -1, 0, -1, 0, -1,
                0, -1, 0, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0,
                -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, -1, 0, -1,
                0, 0, 0, -1, 0, -1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, -1, 0, -1, 0, -1, 0,
                -1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0,
                -1, 0, 0, 0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1,
                0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0,
                -1, 0, 0, 0, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, 0, -1, 0, -1, 0, -1,
                0, 0, 0, 0, 0, -1, 0, -1, 0, 0, -1, 0 };

#endif /* BAMBOO_02_4096_H_ */