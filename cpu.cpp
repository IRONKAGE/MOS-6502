// cpu.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <Windows.h>
#include <time.h>
#include "mos6502.h"


u8 cpu_r(u16 addr);
void cpu_w(u16 addr, u8 val);



void cpu_test();
u8 memory[65536];
int main()
{

	u16 addr = 0x8000;

	FILE *file;
	fopen_s(&file, "mos-app.bin", "rb");
	fread(memory, 1, sizeof(memory), file);
	fclose(file);

	cpu_init(cpu_r, cpu_w);
	
	for (;;)cpu_test();

	for (;;) {
		cpu_cycle();
	}

    return 0;
}




void cpu_test() {

	u32 test_len = 2000;
	printf("Press enter to start");
	getchar();
	printf("testing... \n");

	SYSTEMTIME time_ms;
	GetSystemTime(&time_ms);
	LONG time_a = ((time_ms.wSecond + time_ms.wMinute * 60) * 1000) + time_ms.wMilliseconds;
	for (unsigned int i = 0; i < test_len * 1000000; i++) {
		cpu_cycle();
	}
	GetSystemTime(&time_ms);
	LONG time_b = ((time_ms.wSecond + time_ms.wMinute * 60) * 1000) + time_ms.wMilliseconds - time_a;
	printf("2 billion cpu cycles executed! time: %dms\n", time_b);
	float mhz = (float)test_len *1000 / time_b;
	printf("Your CPU emulation speed equal to mos-6502 at %fMHz\n", mhz);
}


u8 cpu_r(u16 addr) {

	return memory[addr];
}

void cpu_w(u16 addr, u8 val) {
	memory[addr] = val;
}

