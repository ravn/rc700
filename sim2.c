//
// Z80SIM  -  a Z80-CPU simulator
//
// Copyright (C) 1987-2006 by Udo Munk
// Modified for RC700 simulator by Michael Ringgaard
//
// Emulation of multi byte opcodes starting with 0xcb
//

#include "simglb.h"

// Trap for illegal 0xcb multi byte opcodes.
static int trap_cb() {
  cpu_error = OPTRAP2;
  cpu_state = STOPPED;
  return 0;
}

// SRL A
static int op_srla() {
  (A & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  A >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL B
static int op_srlb() {
  (B & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  B >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL C
static int op_srlc() {
  (C & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  C >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL D
static int op_srld() {
  (D & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  D >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL E
static int op_srle() {
  (E & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  E >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) :(F |= P_FLAG);
  return 8;
}

// SRL H
static int op_srlh() {
  (H & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  H >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL L
static int op_srll() {
  (L & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  L >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRL (HL)
static int op_srlhl() {
  BYTE *p;

  p = ram + (H << 8) + L;
  (*p & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  *p >>= 1;
  F &= ~(H_FLAG | N_FLAG);
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// SLA A
static int op_slaa() {
  (A & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  A <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA B
static int op_slab() {
  (B & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  B <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA C
static int op_slac() {
  (C & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  C <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA D
static int op_slad() {
  (D & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  D <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA E
static int op_slae() {
  (E & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  E <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA H
static int op_slah() {
  (H & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  H <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA L
static int op_slal() {
  (L & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  L <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SLA (HL)
static int op_slahl() {
  BYTE *p;

  p = ram + (H << 8) + L;
  (*p & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  *p <<= 1;
  F &= ~(H_FLAG | N_FLAG);
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// RL A
static int op_rlra() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (A & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  A <<= 1;
  if (old_c_flag) A |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL B
static int op_rlb() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (B & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  B <<= 1;
  if (old_c_flag) B |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL C
static int op_rlc() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (C & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  C <<= 1;
  if (old_c_flag) C |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL D
static int op_rld() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (D & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  D <<= 1;
  if (old_c_flag) D |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL E
static int op_rle() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (E & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  E <<= 1;
  if (old_c_flag) E |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL H
static int op_rlh() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (H & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  H <<= 1;
  if (old_c_flag) H |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) :(F |= P_FLAG);
  return 8;
}

// RL L
static int op_rll() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (L & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  L <<= 1;
  if (old_c_flag) L |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RL (HL)
static int op_rlhl() {
  int old_c_flag;
  BYTE *p;

  p = ram + (H << 8) + L;
  old_c_flag = F & C_FLAG;
  (*p & 128) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  *p <<= 1;
  if (old_c_flag) *p |= 1;
  F &= ~(H_FLAG | N_FLAG);
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// RR A
static int op_rrra() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (A & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  A >>= 1;
  if (old_c_flag) A |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR B
static int op_rrb() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (B & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  B >>= 1;
  if (old_c_flag) B |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR C
static int op_rrc() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (C & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  C >>= 1;
  if (old_c_flag) C |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR D
static int op_rrd() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (D & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  D >>= 1;
  if (old_c_flag) D |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR E
static int op_rre() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (E & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  E >>= 1;
  if (old_c_flag) E |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR H
static int op_rrh() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (H & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  H >>= 1;
  if (old_c_flag) H |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR L
static int op_rrl() {
  int old_c_flag;

  old_c_flag = F & C_FLAG;
  (L & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  L >>= 1;
  if (old_c_flag) L |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RR (HL)
static int op_rrhl() {
  int old_c_flag;
  BYTE *p;

  old_c_flag = F & C_FLAG;
  p = ram + (H << 8) + L;
  (*p & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  *p >>= 1;
  if (old_c_flag) *p |= 128;
  F &= ~(H_FLAG | N_FLAG);
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// RRC A
static int op_rrcra() {
  int i;

  i = A & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  A >>= 1;
  if (i) A |= 128;
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC B
static int op_rrcb() {
  int i;

  i = B & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  B >>= 1;
  if (i) B |= 128;
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC C
static int op_rrcc() {
  int i;

  i = C & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  C >>= 1;
  if (i) C |= 128;
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC D
static int op_rrcd() {
  int i;

  i = D & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  D >>= 1;
  if (i) D |= 128;
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC E
static int op_rrce() {
  int i;

  i = E & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  E >>= 1;
  if (i) E |= 128;
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC H
static int op_rrch() {
  int i;

  i = H & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  H >>= 1;
  if (i) H |= 128;
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC L
static int op_rrcl() {
  int i;

  i = L & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  L >>= 1;
  if (i) L |= 128;
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RRC (HL)
static int op_rrchl() {
  int i;
  BYTE *p;

  p = ram + (H << 8) + L;
  i = *p & 1;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  *p >>= 1;
  if (i) *p |= 128;
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// RLC A
static int op_rlcra() {
  int i;

  i = A & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  A <<= 1;
  if (i) A |= 1;
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC B
static int op_rlcb() {
  int i;

  i = B & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  B <<= 1;
  if (i) B |= 1;
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC C
static int op_rlcc() {
  int i;

  i = C & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  C <<= 1;
  if (i) C |= 1;
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC D
static int op_rlcd() {
  int i;

  i = D & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  D <<= 1;
  if (i) D |= 1;
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC E
static int op_rlce() {
  int i;

  i = E & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  E <<= 1;
  if (i) E |= 1;
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC H
static int op_rlch() {
  int i;

  i = H & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  H <<= 1;
  if (i) H |= 1;
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC L
static int op_rlcl() {
  int i;

  i = L & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  L <<= 1;
  if (i) L |= 1;
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// RLC (HL)
static int op_rlchl() {
  int i;
  BYTE *p;

  p = ram + (H << 8) + L;
  i = *p & 128;
  (i) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  F &= ~(H_FLAG | N_FLAG);
  *p <<= 1;
  if (i) *p |= 1;
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// SRA A
static int op_sraa() {
  int i;

  i = A & 128;
  (A & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  A >>= 1;
  A |= i;
  F &= ~(H_FLAG | N_FLAG);
  (A) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (A & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[A]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA B
static int op_srab() {
  int i;

  i = B & 128;
  (B & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  B >>= 1;
  B |= i;
  F &= ~(H_FLAG | N_FLAG);
  (B) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (B & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[B]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA C
static int op_srac() {
  int i;

  i = C & 128;
  (C & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  C >>= 1;
  C |= i;
  F &= ~(H_FLAG | N_FLAG);
  (C) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (C & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[C]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA D
static int op_srad() {
  int i;

  i = D & 128;
  (D & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  D >>= 1;
  D |= i;
  F &= ~(H_FLAG | N_FLAG);
  (D) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (D & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[D]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA E
static int op_srae() {
  int i;

  i = E & 128;
  (E & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  E >>= 1;
  E |= i;
  F &= ~(H_FLAG | N_FLAG);
  (E) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (E & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[E]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA H
static int op_srah() {
  int i;

  i = H & 128;
  (H & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  H >>= 1;
  H |= i;
  F &= ~(H_FLAG | N_FLAG);
  (H) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (H & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[H]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA L
static int op_sral() {
  int i;

  i = L & 128;
  (L & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  L >>= 1;
  L |= i;
  F &= ~(H_FLAG | N_FLAG);
  (L) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (L & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[L]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 8;
}

// SRA (HL)
static int op_srahl() {
  int i;
  BYTE *p;

  p = ram + (H << 8) + L;
  i = *p & 128;
  (*p & 1) ? (F |= C_FLAG) : (F &= ~C_FLAG);
  *p >>= 1;
  *p |= i;
  F &= ~(H_FLAG | N_FLAG);
  (*p) ? (F &= ~Z_FLAG) : (F |= Z_FLAG);
  (*p & 128) ? (F |= S_FLAG) : (F &= ~S_FLAG);
  (parity[*p]) ? (F &= ~P_FLAG) : (F |= P_FLAG);
  return 15;
}

// SET 0,A
static int op_sb0a() {
  A |= 1;
  return 8;
}

// SET 1,A
static int op_sb1a() {
  A |= 2;
  return 8;
}

// SET 2,A
static int op_sb2a() {
  A |= 4;
  return 8;
}

// SET 3,A
static int op_sb3a() {
  A |= 8;
  return 8;
}

// SET 4,A
static int op_sb4a() {
  A |= 16;
  return 8;
}

// SET 5,A
static int op_sb5a() {
  A |= 32;
  return 8;
}

// SET 6,A
static int op_sb6a() {
  A |= 64;
  return 8;
}

// SET 7,A
static int op_sb7a() {
  A |= 128;
  return 8;
}

// SET 0,B
static int op_sb0b() {
  B |= 1;
  return 8;
}

// SET 1,B
static int op_sb1b() {
  B |= 2;
  return 8;
}

// SET 2,B
static int op_sb2b() {
  B |= 4;
  return 8;
}

// SET 3,B
static int op_sb3b() {
  B |= 8;
  return 8;
}

// SET 4,B
static int op_sb4b() {
  B |= 16;
  return 8;
}

// SET 5,B
static int op_sb5b() {
  B |= 32;
  return 8;
}

// SET 6,B
static int op_sb6b() {
  B |= 64;
  return 8;
}

// SET 7,B
static int op_sb7b() {
  B |= 128;
  return 8;
}

// SET 0,C
static int op_sb0c() {
  C |= 1;
  return 8;
}

// SET 1,C
static int op_sb1c() {
  C |= 2;
  return 8;
}

// SET 2,C
static int op_sb2c() {
  C |= 4;
  return 8;
}

// SET 3,C
static int op_sb3c() {
  C |= 8;
  return 8;
}

// SET 4,C
static int op_sb4c() {
  C |= 16;
  return 8;
}

// SET 5,C
static int op_sb5c() {
  C |= 32;
  return 8;
}

// SET 6,C
static int op_sb6c() {
  C |= 64;
  return 8;
}

// SET 7,C
static int op_sb7c() {
  C |= 128;
  return 8;
}

// SET 0,D
static int op_sb0d() {
  D |= 1;
  return 8;
}

// SET 1,D
static int op_sb1d() {
  D |= 2;
  return 8;
}

// SET 2,D
static int op_sb2d() {
  D |= 4;
  return 8;
}

// SET 3,D
static int op_sb3d() {
  D |= 8;
  return 8;
}

// SET 4,D
static int op_sb4d() {
  D |= 16;
  return 8;
}

// SET 5,D
static int op_sb5d() {
  D |= 32;
  return 8;
}

// SET 6,D
static int op_sb6d() {
  D |= 64;
  return 8;
}

// SET 7,D
static int op_sb7d() {
  D |= 128;
  return 8;
}

// SET 0,E
static int op_sb0e() {
  E |= 1;
  return 8;
}

// SET 1,E
static int op_sb1e() {
  E |= 2;
  return 8;
}

// SET 2,E
static int op_sb2e() {
  E |= 4;
  return 8;
}

// SET 3,E
static int op_sb3e() {
  E |= 8;
  return 8;
}

// SET 4,E
static int op_sb4e() {
  E |= 16;
  return 8;
}

// SET 5,E
static int op_sb5e() {
  E |= 32;
  return 8;
}

// SET 6,E
static int op_sb6e() {
  E |= 64;
  return 8;
}

// SET 7,E
static int op_sb7e() {
  E |= 128;
  return 8;
}

// SET 0,H
static int op_sb0h() {
  H |= 1;
  return 8;
}

// SET 1,H
static int op_sb1h() {
  H |= 2;
  return 8;
}

// SET 2,H
static int op_sb2h() {
  H |= 4;
  return 8;
}

// SET 3,H
static int op_sb3h() {
  H |= 8;
  return 8;
}

// SET 4,H
static int op_sb4h() {
  H |= 16;
  return 8;
}

// SET 5,H
static int op_sb5h() {
  H |= 32;
  return 8;
}

// SET 6,H
static int op_sb6h() {
  H |= 64;
  return 8;
}

// SET 7,H
static int op_sb7h() {
  H |= 128;
  return 8;
}

// SET 0,L
static int op_sb0l() {
  L |= 1;
  return 8;
}

// SET 1,L
static int op_sb1l() {
  L |= 2;
  return 8;
}

// SET 2,L
static int op_sb2l() {
  L |= 4;
  return 8;
}

// SET 3,L
static int op_sb3l() {
  L |= 8;
  return 8;
}

// SET 4,L
static int op_sb4l() {
  L |= 16;
  return 8;
}

// SET 5,L
static int op_sb5l() {
  L |= 32;
  return 8;
}

// SET 6,L
static int op_sb6l() {
  L |= 64;
  return 8;
}

// SET 7,L
static int op_sb7l() {
  L |= 128;
  return 8;
}

// SET 0,(HL)
static int op_sb0hl() {
  *(ram + (H << 8) + L) |= 1;
  return 15;
}

// SET 1,(HL)
static int op_sb1hl() {
  *(ram + (H << 8) + L) |= 2;
  return 15;
}

// SET 2,(HL)
static int op_sb2hl() {
  *(ram + (H << 8) + L) |= 4;
  return 15;
}

// SET 3,(HL)
static int op_sb3hl() {
  *(ram + (H << 8) + L) |= 8;
  return 15;
}

// SET 4,(HL)
static int op_sb4hl() {
  *(ram + (H << 8) + L) |= 16;
  return 15;
}

// SET 5,(HL)
static int op_sb5hl() {
  *(ram + (H << 8) + L) |= 32;
  return 15;
}

// SET 6,(HL)
static int op_sb6hl() {
  *(ram + (H << 8) + L) |= 64;
  return 15;
}

// SET 7,(HL)
static int op_sb7hl() {
  *(ram + (H << 8) + L) |= 128;
  return 15;
}

// RES 0,A
static int op_rb0a() {
  A &= ~1;
  return 8;
}

// RES 1,A
static int op_rb1a() {
  A &= ~2;
  return 8;
}

// RES 2,A
static int op_rb2a() {
  A &= ~4;
  return 8;
}

// RES 3,A
static int op_rb3a() {
  A &= ~8;
  return 8;
}

// RES 4,A
static int op_rb4a() {
  A &= ~16;
  return 8;
}

// RES 5,A
static int op_rb5a() {
  A &= ~32;
  return 8;
}

// RES 6,A
static int op_rb6a() {
  A &= ~64;
  return 8;
}

// RES 7,A
static int op_rb7a() {
  A &= ~128;
  return 8;
}

// RES 0,B
static int op_rb0b() {
  B &= ~1;
  return 8;
}

// RES 1,B
static int op_rb1b() {
  B &= ~2;
  return 8;
}

// RES 2,B
static int op_rb2b() {
  B &= ~4;
  return 8;
}

// RES 3,B
static int op_rb3b() {
  B &= ~8;
  return 8;
}

// RES 4,B
static int op_rb4b() {
  B &= ~16;
  return 8;
}

// RES 5,B
static int op_rb5b() {
  B &= ~32;
  return 8;
}

// RES 6,B
static int op_rb6b() {
  B &= ~64;
  return 8;
}

// RES 7,B
static int op_rb7b() {
  B &= ~128;
  return 8;
}

// RES 0,C
static int op_rb0c() {
  C &= ~1;
  return 8;
}

// RES 1,C
static int op_rb1c() {
  C &= ~2;
  return 8;
}

// RES 2,C
static int op_rb2c() {
  C &= ~4;
  return 8;
}

// RES 3,C
static int op_rb3c() {
  C &= ~8;
  return 8;
}

// RES 4,C
static int op_rb4c() {
  C &= ~16;
  return 8;
}

// RES 5,C
static int op_rb5c() {
  C &= ~32;
  return 8;
}

// RES 6,C
static int op_rb6c() {
  C &= ~64;
  return 8;
}

// RES 7,C
static int op_rb7c() {
  C &= ~128;
  return 8;
}

// RES 0,D
static int op_rb0d() {
  D &= ~1;
  return 8;
}

// RES 1,D
static int op_rb1d() {
  D &= ~2;
  return 8;
}

// RES 2,D
static int op_rb2d() {
  D &= ~4;
  return 8;
}

// RES 3,D
static int op_rb3d() {
  D &= ~8;
  return 8;
}

// RES 4,D
static int op_rb4d() {
  D &= ~16;
  return 8;
}

// RES 5,D
static int op_rb5d() {
  D &= ~32;
  return 8;
}

// RES 6,D
static int op_rb6d() {
  D &= ~64;
  return 8;
}

// RES 7,D
static int op_rb7d() {
  D &= ~128;
  return 8;
}

// RES 0,E
static int op_rb0e() {
  E &= ~1;
  return 8;
}

// RES 1,E
static int op_rb1e() {
  E &= ~2;
  return 8;
}

// RES 2,E
static int op_rb2e() {
  E &= ~4;
  return 8;
}

// RES 3,E
static int op_rb3e() {
  E &= ~8;
  return 8;
}

// RES 4,E
static int op_rb4e() {
  E &= ~16;
  return 8;
}

// RES 5,E
static int op_rb5e() {
  E &= ~32;
  return 8;
}

// RES 6,E
static int op_rb6e() {
  E &= ~64;
  return 8;
}

// RES 7,E
static int op_rb7e() {
  E &= ~128;
  return 8;
}

// RES 0,H
static int op_rb0h() {
  H &= ~1;
  return 8;
}

// RES 1,H
static int op_rb1h() {
  H &= ~2;
  return 8;
}

// RES 2,H
static int op_rb2h() {
  H &= ~4;
  return 8;
}

// RES 3,H
static int op_rb3h() {
  H &= ~8;
  return 8;
}

// RES 4,H
static int op_rb4h() {
  H &= ~16;
  return 8;
}

// RES 5,H
static int op_rb5h() {
  H &= ~32;
  return 8;
}

// RES 6,H
static int op_rb6h() {
  H &= ~64;
  return 8;
}

// RES 7,H
static int op_rb7h() {
  H &= ~128;
  return 8;
}

// RES 0,L
static int op_rb0l() {
  L &= ~1;
  return 8;
}

// RES 1,L
static int op_rb1l() {
  L &= ~2;
  return 8;
}

// RES 2,L
static int op_rb2l() {
  L &= ~4;
  return 8;
}

// RES 3,L
static int op_rb3l() {
  L &= ~8;
  return 8;
}

// RES 4,L
static int op_rb4l() {
  L &= ~16;
  return 8;
}

// RES 5,L
static int op_rb5l() {
  L &= ~32;
  return 8;
}

// RES 6,L
static int op_rb6l() {
  L &= ~64;
  return 8;
}

// RES 7,L
static int op_rb7l() {
  L &= ~128;
  return 8;
}

// RES 0,(HL)
static int op_rb0hl() {
  *(ram + (H << 8) + L) &= ~1;
  return 15;
}

// RES 1,(HL)
static int op_rb1hl() {
  *(ram + (H << 8) + L) &= ~2;
  return 15;
}

// RES 2,(HL)
static int op_rb2hl() {
  *(ram + (H << 8) + L) &= ~4;
  return 15;
}

// RES 3,(HL)
static int op_rb3hl() {
  *(ram + (H << 8) + L) &= ~8;
  return 15;
}

// RES 4,(HL)
static int op_rb4hl() {
  *(ram + (H << 8) + L) &= ~16;
  return 15;
}

// RES 5,(HL)
static int op_rb5hl() {
  *(ram + (H << 8) + L) &= ~32;
  return 15;
}

// RES 6,(HL)
static int op_rb6hl() {
  *(ram + (H << 8) + L) &= ~64;
  return 15;
}

// RES 7,(HL)
static int op_rb7hl() {
  *(ram + (H << 8) + L) &= ~128;
  return 15;
}

// BIT 0,A
static int op_tb0a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,A
static int op_tb1a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,A
static int op_tb2a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,A
static int op_tb3a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,A
static int op_tb4a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,A
static int op_tb5a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,A
static int op_tb6a() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (A & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,A
static int op_tb7a() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (A & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,B
static int op_tb0b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,B
static int op_tb1b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,B
static int op_tb2b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,B
static int op_tb3b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,B
static int op_tb4b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,B
static int op_tb5b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,B
static int op_tb6b() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (B & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,B
static int op_tb7b() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (B & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,C
static int op_tb0c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,C
static int op_tb1c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,C
static int op_tb2c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,C
static int op_tb3c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,C
static int op_tb4c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,C
static int op_tb5c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,C
static int op_tb6c() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (C & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,C
static int op_tb7c() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (C & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,D
static int op_tb0d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,D
static int op_tb1d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,D
static int op_tb2d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,D
static int op_tb3d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,D
static int op_tb4d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,D
static int op_tb5d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,D
static int op_tb6d() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (D & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,D
static int op_tb7d() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (D & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,E
static int op_tb0e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,E
static int op_tb1e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,E
static int op_tb2e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,E
static int op_tb3e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,E
static int op_tb4e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,E
static int op_tb5e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,E
static int op_tb6e() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (E & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,E
static int op_tb7e() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (E & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,H
static int op_tb0h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,H
static int op_tb1h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,H
static int op_tb2h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,H
static int op_tb3h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,H
static int op_tb4h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,H
static int op_tb5h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,H
static int op_tb6h() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (H & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,H
static int op_tb7h() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (H & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,L
static int op_tb0l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 1) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 1,L
static int op_tb1l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 2) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 2,L
static int op_tb2l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 4) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 3,L
static int op_tb3l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 8) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 4,L
static int op_tb4l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 16) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 5,L
static int op_tb5l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 32) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 6,L
static int op_tb6l() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (L & 64) ? (F &= ~(Z_FLAG | P_FLAG)) : (F |= (Z_FLAG | P_FLAG));
  return 8;
}

// BIT 7,L
static int op_tb7l() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (L & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 8;
}

// BIT 0,(HL)
static int op_tb0hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 1) ? (F &= ~(Z_FLAG | P_FLAG))
            : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 1,(HL)
static int op_tb1hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 2) ? (F &= ~(Z_FLAG | P_FLAG))
            : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 2,(HL)
static int op_tb2hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 4) ? (F &= ~(Z_FLAG | P_FLAG))
            : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 3,(HL)
static int op_tb3hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 8) ? (F &= ~(Z_FLAG | P_FLAG))
            : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 4,(HL)
static int op_tb4hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 16) ? (F &= ~(Z_FLAG | P_FLAG))
             : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 5,(HL)
static int op_tb5hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 32) ? (F &= ~(Z_FLAG | P_FLAG))
             : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 6,(HL)
static int op_tb6hl() {
  F &= ~(N_FLAG | S_FLAG);
  F |= H_FLAG;
  (*(ram + (H << 8) + L) & 64) ? (F &= ~(Z_FLAG | P_FLAG))
             : (F |= (Z_FLAG | P_FLAG));
  return 12;
}

// BIT 7,(HL)
static int op_tb7hl() {
  F &= ~N_FLAG;
  F |= H_FLAG;
  if (*(ram + (H << 8) + L) & 128) {
    F &= ~(Z_FLAG | P_FLAG);
    F |= S_FLAG;
  } else {
    F |= (Z_FLAG | P_FLAG);
    F &= ~S_FLAG;
  }
  return 12;
}

// Execute 0xcb multi byte opcode.
int op_cb_handler() {
  int t;

  static int (*op_cb[256])() = {
    op_rlcb,      /* 0x00 */
    op_rlcc,      /* 0x01 */
    op_rlcd,      /* 0x02 */
    op_rlce,      /* 0x03 */
    op_rlch,      /* 0x04 */
    op_rlcl,      /* 0x05 */
    op_rlchl,     /* 0x06 */
    op_rlcra,     /* 0x07 */
    op_rrcb,      /* 0x08 */
    op_rrcc,      /* 0x09 */
    op_rrcd,      /* 0x0a */
    op_rrce,      /* 0x0b */
    op_rrch,      /* 0x0c */
    op_rrcl,      /* 0x0d */
    op_rrchl,     /* 0x0e */
    op_rrcra,     /* 0x0f */
    op_rlb,       /* 0x10 */
    op_rlc,       /* 0x11 */
    op_rld,       /* 0x12 */
    op_rle,       /* 0x13 */
    op_rlh,       /* 0x14 */
    op_rll,       /* 0x15 */
    op_rlhl,      /* 0x16 */
    op_rlra,      /* 0x17 */
    op_rrb,       /* 0x18 */
    op_rrc,       /* 0x19 */
    op_rrd,       /* 0x1a */
    op_rre,       /* 0x1b */
    op_rrh,       /* 0x1c */
    op_rrl,       /* 0x1d */
    op_rrhl,      /* 0x1e */
    op_rrra,      /* 0x1f */
    op_slab,      /* 0x20 */
    op_slac,      /* 0x21 */
    op_slad,      /* 0x22 */
    op_slae,      /* 0x23 */
    op_slah,      /* 0x24 */
    op_slal,      /* 0x25 */
    op_slahl,     /* 0x26 */
    op_slaa,      /* 0x27 */
    op_srab,      /* 0x28 */
    op_srac,      /* 0x29 */
    op_srad,      /* 0x2a */
    op_srae,      /* 0x2b */
    op_srah,      /* 0x2c */
    op_sral,      /* 0x2d */
    op_srahl,     /* 0x2e */
    op_sraa,      /* 0x2f */
    trap_cb,      /* 0x30 */
    trap_cb,      /* 0x31 */
    trap_cb,      /* 0x32 */
    trap_cb,      /* 0x33 */
    trap_cb,      /* 0x34 */
    trap_cb,      /* 0x35 */
    trap_cb,      /* 0x36 */
    trap_cb,      /* 0x37 */
    op_srlb,      /* 0x38 */
    op_srlc,      /* 0x39 */
    op_srld,      /* 0x3a */
    op_srle,      /* 0x3b */
    op_srlh,      /* 0x3c */
    op_srll,      /* 0x3d */
    op_srlhl,     /* 0x3e */
    op_srla,      /* 0x3f */
    op_tb0b,      /* 0x40 */
    op_tb0c,      /* 0x41 */
    op_tb0d,      /* 0x42 */
    op_tb0e,      /* 0x43 */
    op_tb0h,      /* 0x44 */
    op_tb0l,      /* 0x45 */
    op_tb0hl,     /* 0x46 */
    op_tb0a,      /* 0x47 */
    op_tb1b,      /* 0x48 */
    op_tb1c,      /* 0x49 */
    op_tb1d,      /* 0x4a */
    op_tb1e,      /* 0x4b */
    op_tb1h,      /* 0x4c */
    op_tb1l,      /* 0x4d */
    op_tb1hl,     /* 0x4e */
    op_tb1a,      /* 0x4f */
    op_tb2b,      /* 0x50 */
    op_tb2c,      /* 0x51 */
    op_tb2d,      /* 0x52 */
    op_tb2e,      /* 0x53 */
    op_tb2h,      /* 0x54 */
    op_tb2l,      /* 0x55 */
    op_tb2hl,     /* 0x56 */
    op_tb2a,      /* 0x57 */
    op_tb3b,      /* 0x58 */
    op_tb3c,      /* 0x59 */
    op_tb3d,      /* 0x5a */
    op_tb3e,      /* 0x5b */
    op_tb3h,      /* 0x5c */
    op_tb3l,      /* 0x5d */
    op_tb3hl,     /* 0x5e */
    op_tb3a,      /* 0x5f */
    op_tb4b,      /* 0x60 */
    op_tb4c,      /* 0x61 */
    op_tb4d,      /* 0x62 */
    op_tb4e,      /* 0x63 */
    op_tb4h,      /* 0x64 */
    op_tb4l,      /* 0x65 */
    op_tb4hl,     /* 0x66 */
    op_tb4a,      /* 0x67 */
    op_tb5b,      /* 0x68 */
    op_tb5c,      /* 0x69 */
    op_tb5d,      /* 0x6a */
    op_tb5e,      /* 0x6b */
    op_tb5h,      /* 0x6c */
    op_tb5l,      /* 0x6d */
    op_tb5hl,     /* 0x6e */
    op_tb5a,      /* 0x6f */
    op_tb6b,      /* 0x70 */
    op_tb6c,      /* 0x71 */
    op_tb6d,      /* 0x72 */
    op_tb6e,      /* 0x73 */
    op_tb6h,      /* 0x74 */
    op_tb6l,      /* 0x75 */
    op_tb6hl,     /* 0x76 */
    op_tb6a,      /* 0x77 */
    op_tb7b,      /* 0x78 */
    op_tb7c,      /* 0x79 */
    op_tb7d,      /* 0x7a */
    op_tb7e,      /* 0x7b */
    op_tb7h,      /* 0x7c */
    op_tb7l,      /* 0x7d */
    op_tb7hl,     /* 0x7e */
    op_tb7a,      /* 0x7f */
    op_rb0b,      /* 0x80 */
    op_rb0c,      /* 0x81 */
    op_rb0d,      /* 0x82 */
    op_rb0e,      /* 0x83 */
    op_rb0h,      /* 0x84 */
    op_rb0l,      /* 0x85 */
    op_rb0hl,     /* 0x86 */
    op_rb0a,      /* 0x87 */
    op_rb1b,      /* 0x88 */
    op_rb1c,      /* 0x89 */
    op_rb1d,      /* 0x8a */
    op_rb1e,      /* 0x8b */
    op_rb1h,      /* 0x8c */
    op_rb1l,      /* 0x8d */
    op_rb1hl,     /* 0x8e */
    op_rb1a,      /* 0x8f */
    op_rb2b,      /* 0x90 */
    op_rb2c,      /* 0x91 */
    op_rb2d,      /* 0x92 */
    op_rb2e,      /* 0x93 */
    op_rb2h,      /* 0x94 */
    op_rb2l,      /* 0x95 */
    op_rb2hl,     /* 0x96 */
    op_rb2a,      /* 0x97 */
    op_rb3b,      /* 0x98 */
    op_rb3c,      /* 0x99 */
    op_rb3d,      /* 0x9a */
    op_rb3e,      /* 0x9b */
    op_rb3h,      /* 0x9c */
    op_rb3l,      /* 0x9d */
    op_rb3hl,     /* 0x9e */
    op_rb3a,      /* 0x9f */
    op_rb4b,      /* 0xa0 */
    op_rb4c,      /* 0xa1 */
    op_rb4d,      /* 0xa2 */
    op_rb4e,      /* 0xa3 */
    op_rb4h,      /* 0xa4 */
    op_rb4l,      /* 0xa5 */
    op_rb4hl,     /* 0xa6 */
    op_rb4a,      /* 0xa7 */
    op_rb5b,      /* 0xa8 */
    op_rb5c,      /* 0xa9 */
    op_rb5d,      /* 0xaa */
    op_rb5e,      /* 0xab */
    op_rb5h,      /* 0xac */
    op_rb5l,      /* 0xad */
    op_rb5hl,     /* 0xae */
    op_rb5a,      /* 0xaf */
    op_rb6b,      /* 0xb0 */
    op_rb6c,      /* 0xb1 */
    op_rb6d,      /* 0xb2 */
    op_rb6e,      /* 0xb3 */
    op_rb6h,      /* 0xb4 */
    op_rb6l,      /* 0xb5 */
    op_rb6hl,     /* 0xb6 */
    op_rb6a,      /* 0xb7 */
    op_rb7b,      /* 0xb8 */
    op_rb7c,      /* 0xb9 */
    op_rb7d,      /* 0xba */
    op_rb7e,      /* 0xbb */
    op_rb7h,      /* 0xbc */
    op_rb7l,      /* 0xbd */
    op_rb7hl,     /* 0xbe */
    op_rb7a,      /* 0xbf */
    op_sb0b,      /* 0xc0 */
    op_sb0c,      /* 0xc1 */
    op_sb0d,      /* 0xc2 */
    op_sb0e,      /* 0xc3 */
    op_sb0h,      /* 0xc4 */
    op_sb0l,      /* 0xc5 */
    op_sb0hl,     /* 0xc6 */
    op_sb0a,      /* 0xc7 */
    op_sb1b,      /* 0xc8 */
    op_sb1c,      /* 0xc9 */
    op_sb1d,      /* 0xca */
    op_sb1e,      /* 0xcb */
    op_sb1h,      /* 0xcc */
    op_sb1l,      /* 0xcd */
    op_sb1hl,     /* 0xce */
    op_sb1a,      /* 0xcf */
    op_sb2b,      /* 0xd0 */
    op_sb2c,      /* 0xd1 */
    op_sb2d,      /* 0xd2 */
    op_sb2e,      /* 0xd3 */
    op_sb2h,      /* 0xd4 */
    op_sb2l,      /* 0xd5 */
    op_sb2hl,     /* 0xd6 */
    op_sb2a,      /* 0xd7 */
    op_sb3b,      /* 0xd8 */
    op_sb3c,      /* 0xd9 */
    op_sb3d,      /* 0xda */
    op_sb3e,      /* 0xdb */
    op_sb3h,      /* 0xdc */
    op_sb3l,      /* 0xdd */
    op_sb3hl,     /* 0xde */
    op_sb3a,      /* 0xdf */
    op_sb4b,      /* 0xe0 */
    op_sb4c,      /* 0xe1 */
    op_sb4d,      /* 0xe2 */
    op_sb4e,      /* 0xe3 */
    op_sb4h,      /* 0xe4 */
    op_sb4l,      /* 0xe5 */
    op_sb4hl,     /* 0xe6 */
    op_sb4a,      /* 0xe7 */
    op_sb5b,      /* 0xe8 */
    op_sb5c,      /* 0xe9 */
    op_sb5d,      /* 0xea */
    op_sb5e,      /* 0xeb */
    op_sb5h,      /* 0xec */
    op_sb5l,      /* 0xed */
    op_sb5hl,     /* 0xee */
    op_sb5a,      /* 0xef */
    op_sb6b,      /* 0xf0 */
    op_sb6c,      /* 0xf1 */
    op_sb6d,      /* 0xf2 */
    op_sb6e,      /* 0xf3 */
    op_sb6h,      /* 0xf4 */
    op_sb6l,      /* 0xf5 */
    op_sb6hl,     /* 0xf6 */
    op_sb6a,      /* 0xf7 */
    op_sb7b,      /* 0xf8 */
    op_sb7c,      /* 0xf9 */
    op_sb7d,      /* 0xfa */
    op_sb7e,      /* 0xfb */
    op_sb7h,      /* 0xfc */
    op_sb7l,      /* 0xfd */
    op_sb7hl,     /* 0xfe */
    op_sb7a       /* 0xff */
  };

  // Execute next opcode
  t = (*op_cb[*PC++])();

#ifdef ENABLE_PCC
  // Correct PC overrun
  if (PC > ram + 65535) PC = ram;
#endif

  return t;
}

