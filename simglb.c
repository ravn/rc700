//
// Z80SIM  -  a Z80-CPU simulator
//
// Copyright (C) 1987-2006 by Udo Munk
//

#include "sim.h"

//
//  CPU registers.
//

// Z80 primary registers.
BYTE A, B, C, D, E, H, L;   
WORD IX, IY;
BYTE F;

// Z80 secondary registers.
BYTE A_, B_, C_, D_, E_, H_, L_;
BYTE F_;

// Z80 program counter.
BYTE *PC;

// Z80 stack pointer.
BYTE *STACK;

// Z80 interrupt register.
BYTE I;

// Z80 interrupt flags.
BYTE IFF;

// Z80 refresh register.
long R;       

//
// RAM memory.
//

BYTE ram[65536];   // 64KB RAM

//
// Trace history.
//

#ifdef HISIZE
struct history his[HISIZE]; // Trace information.
int h_next;                 // Index into trace history.
int h_flag;                 // Flag for trace history overrun.
#endif

//
// Breakpoints.
//

#ifdef SBSIZE
struct softbreak soft[SBSIZE];
int sb_next;
#endif

//
// Runtime measurement.
//

#ifdef WANT_TIM
long t_states;                // Number of counted T states.
int t_flag;
BYTE *t_start = ram + 65535;  // Start address for measurement.
BYTE *t_end = ram + 65535;    // End address for measurement.
#endif

//
// Simulator control flags.
//

int s_flag;
int l_flag;
int m_flag;
int x_flag;
int i_flag = 1;
int f_flag;
int break_flag = 0; // 1 = break at HALT, 0 = execute HALT

//
// Simulator state.
//

int cpu_state;      // Status of CPU emulation.
int cpu_error;      // Error status of CPU emulation.

int int_mode;       // CPU interrupt mode (IM 0, IM 1, IM 2)
int int_type;       // Interrupt type.
int int_vec;        // Interrupt vector.

float freq;         // CPU clock in usec.
int clk_ctr = 0;    // Counter for generating ticks from the system clock.

// Interrupt daisy chain for interrupt priority.
int int_chain[8] = {-1, -1, -1, -1, -1, -1, -1, -1};

//  Pre-computed parity table.
int parity[256] = {
    0 /* 00000000 */, 1 /* 00000001 */, 1 /* 00000010 */,
    0 /* 00000011 */, 1 /* 00000100 */, 0 /* 00000101 */,
    0 /* 00000110 */, 1 /* 00000111 */, 1 /* 00001000 */,
    0 /* 00001001 */, 0 /* 00001010 */, 1 /* 00001011 */,
    0 /* 00001100 */, 1 /* 00001101 */, 1 /* 00001110 */,
    0 /* 00001111 */, 1 /* 00010000 */, 0 /* 00010001 */,
    0 /* 00010010 */, 1 /* 00010011 */, 0 /* 00010100 */,
    1 /* 00010101 */, 1 /* 00010110 */, 0 /* 00010111 */,
    0 /* 00011000 */, 1 /* 00011001 */, 1 /* 00011010 */,
    0 /* 00011011 */, 1 /* 00011100 */, 0 /* 00011101 */,
    0 /* 00011110 */, 1 /* 00011111 */, 1 /* 00100000 */,
    0 /* 00100001 */, 0 /* 00100010 */, 1 /* 00100011 */,
    0 /* 00100100 */, 1 /* 00100101 */, 1 /* 00100110 */,
    0 /* 00100111 */, 0 /* 00101000 */, 1 /* 00101001 */,
    1 /* 00101010 */, 0 /* 00101011 */, 1 /* 00101100 */,
    0 /* 00101101 */, 0 /* 00101110 */, 1 /* 00101111 */,
    0 /* 00110000 */, 1 /* 00110001 */, 1 /* 00110010 */,
    0 /* 00110011 */, 1 /* 00110100 */, 0 /* 00110101 */,
    0 /* 00110110 */, 1 /* 00110111 */, 1 /* 00111000 */,
    0 /* 00111001 */, 0 /* 00111010 */, 1 /* 00111011 */,
    0 /* 00111100 */, 1 /* 00111101 */, 1 /* 00111110 */,
    0 /* 00111111 */, 1 /* 01000000 */, 0 /* 01000001 */,
    0 /* 01000010 */, 1 /* 01000011 */, 0 /* 01000100 */,
    1 /* 01000101 */, 1 /* 01000110 */, 0 /* 01000111 */,
    0 /* 01001000 */, 1 /* 01001001 */, 1 /* 01001010 */,
    0 /* 01001011 */, 1 /* 01001100 */, 0 /* 01001101 */,
    0 /* 01001110 */, 1 /* 01001111 */, 0 /* 01010000 */,
    1 /* 01010001 */, 1 /* 01010010 */, 0 /* 01010011 */,
    1 /* 01010100 */, 0 /* 01010101 */, 0 /* 01010110 */,
    1 /* 01010111 */, 1 /* 01011000 */, 0 /* 01011001 */,
    0 /* 01011010 */, 1 /* 01011011 */, 0 /* 01011100 */,
    1 /* 01011101 */, 1 /* 01011110 */, 0 /* 01011111 */,
    0 /* 01100000 */, 1 /* 01100001 */, 1 /* 01100010 */,
    0 /* 01100011 */, 1 /* 01100100 */, 0 /* 01100101 */,
    0 /* 01100110 */, 1 /* 01100111 */, 1 /* 01101000 */,
    0 /* 01101001 */, 0 /* 01101010 */, 1 /* 01101011 */,
    0 /* 01101100 */, 1 /* 01101101 */, 1 /* 01101110 */,
    0 /* 01101111 */, 1 /* 01110000 */, 0 /* 01110001 */,
    0 /* 01110010 */, 1 /* 01110011 */, 0 /* 01110100 */,
    1 /* 01110101 */, 1 /* 01110110 */, 0 /* 01110111 */,
    0 /* 01111000 */, 1 /* 01111001 */, 1 /* 01111010 */,
    0 /* 01111011 */, 1 /* 01111100 */, 0 /* 01111101 */,
    0 /* 01111110 */, 1 /* 01111111 */,
    1 /* 10000000 */, 0 /* 10000001 */, 0 /* 10000010 */,
    1 /* 10000011 */, 0 /* 10000100 */, 1 /* 10000101 */,
    1 /* 10000110 */, 0 /* 10000111 */, 0 /* 10001000 */,
    1 /* 10001001 */, 1 /* 10001010 */, 0 /* 10001011 */,
    1 /* 10001100 */, 0 /* 10001101 */, 0 /* 10001110 */,
    1 /* 10001111 */, 0 /* 10010000 */, 1 /* 10010001 */,
    1 /* 10010010 */, 0 /* 10010011 */, 1 /* 10010100 */,
    0 /* 10010101 */, 0 /* 10010110 */, 1 /* 10010111 */,
    1 /* 10011000 */, 0 /* 10011001 */, 0 /* 10011010 */,
    1 /* 10011011 */, 0 /* 10011100 */, 1 /* 10011101 */,
    1 /* 10011110 */, 0 /* 10011111 */, 0 /* 10100000 */,
    1 /* 10100001 */, 1 /* 10100010 */, 0 /* 10100011 */,
    1 /* 10100100 */, 0 /* 10100101 */, 0 /* 10100110 */,
    1 /* 10100111 */, 1 /* 10101000 */, 0 /* 10101001 */,
    0 /* 10101010 */, 1 /* 10101011 */, 0 /* 10101100 */,
    1 /* 10101101 */, 1 /* 10101110 */, 0 /* 10101111 */,
    1 /* 10110000 */, 0 /* 10110001 */, 0 /* 10110010 */,
    1 /* 10110011 */, 0 /* 10110100 */, 1 /* 10110101 */,
    1 /* 10110110 */, 0 /* 10110111 */, 0 /* 10111000 */,
    1 /* 10111001 */, 1 /* 10111010 */, 0 /* 10111011 */,
    1 /* 10111100 */, 0 /* 10111101 */, 0 /* 10111110 */,
    1 /* 10111111 */, 0 /* 11000000 */, 1 /* 11000001 */,
    1 /* 11000010 */, 0 /* 11000011 */, 1 /* 11000100 */,
    0 /* 11000101 */, 0 /* 11000110 */, 1 /* 11000111 */,
    1 /* 11001000 */, 0 /* 11001001 */, 0 /* 11001010 */,
    1 /* 11001011 */, 0 /* 11001100 */, 1 /* 11001101 */,
    1 /* 11001110 */, 0 /* 11001111 */, 1 /* 11010000 */,
    0 /* 11010001 */, 0 /* 11010010 */, 1 /* 11010011 */,
    0 /* 11010100 */, 1 /* 11010101 */, 1 /* 11010110 */,
    0 /* 11010111 */, 0 /* 11011000 */, 1 /* 11011001 */,
    1 /* 11011010 */, 0 /* 11011011 */, 1 /* 11011100 */,
    0 /* 11011101 */, 0 /* 11011110 */, 1 /* 11011111 */,
    1 /* 11100000 */, 0 /* 11100001 */, 0 /* 11100010 */,
    1 /* 11100011 */, 0 /* 11100100 */, 1 /* 11100101 */,
    1 /* 11100110 */, 0 /* 11100111 */, 0 /* 11101000 */,
    1 /* 11101001 */, 1 /* 11101010 */, 0 /* 11101011 */,
    1 /* 11101100 */, 0 /* 11101101 */, 0 /* 11101110 */,
    1 /* 11101111 */, 0 /* 11110000 */, 1 /* 11110001 */,
    1 /* 11110010 */, 0 /* 11110011 */, 1 /* 11110100 */,
    0 /* 11110101 */, 0 /* 11110110 */, 1 /* 11110111 */,
    1 /* 11111000 */, 0 /* 11111001 */, 0 /* 11111010 */,
    1 /* 11111011 */, 0 /* 11111100 */, 1 /* 11111101 */,
    1 /* 11111110 */, 0 /* 11111111 */
};
