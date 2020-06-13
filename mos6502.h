
#pragma once

#define u32 unsigned int
#define u16 unsigned short
#define u8 unsigned char
#define s8 char



void cpu_init(u8(*cpu_rd)(u16 addr), void(*cpu_wr)(u16 addr, u8 val));
//void cpu_rst();
void cpu_cycle();

extern u8 cpu_int_irq;
extern u8 cpu_int_nmi;
extern u32 cpu_cycle_ctr;

