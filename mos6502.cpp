

// MOS-6502 cycle accurate core. Undocumented opcodes isn't supported
// Copyright (c) 2017 by Igor Gulubovskiy (krikzz)
// biokrik@gmail.com

#include "stdafx.h"
#include "mos6502.h"

#define SR_C 0x01
#define SR_Z 0x02
#define SR_I 0x04
#define SR_D 0x08
#define SR_B 0x10
#define SR_5 0x20
#define SR_V 0x40
#define SR_N 0x80

#define VECTOR_NMI 0xFFFA
#define VECTOR_RST 0xFFFC
#define VECTOR_IRQ 0xFFFE


void cpu_init(u8(*cpu_rd)(u16 addr), void(*cpu_wr)(u16 addr, u8 val));
void cpu_cycle();
u8(*cpu_read)(u16 addr);
void(*cpu_write)(u16 addr, u8 val);


u8 addr_null();
u8 addr_x_ind();
u8 addr_ind_y();
u8 addr_abs();
u8 addr_abs_x();
u8 addr_abs_x5();
u8 addr_abs_y();
u8 addr_imm();
u8 addr_zpg();
u8 addr_zpg_x();
u8 addr_zpg_y();



//math
void adc_mad();
void and_mad();
void asl_acc();
void asl_mad();
void bit_mad();
void sbc_mad();
void dec_mad();
void dex_imp();
void dey_imp();
void inc_mad();
void inx_imp();
void iny_imp();
void lsr_acc();
void lsr_mad();
void ora_mad();
void rol_acc();
void rol_mad();
void ror_acc();
void ror_mad();
void eor_mad();

//branch
void bcc_rel();
void bcs_rel();
void beq_rel();
void bmi_rel();
void bne_rel();
void bpl_rel();
void bvc_rel();
void bvs_rel();
void jmp_abs();
void jmp_ind();
void jsr_abs();
void rti_imp();
void rts_imp();

//interrupts
void brk_imp();

//status reg
void clc_imp();
void cld_imp();
void cli_imp();
void clv_imp();
void sec_imp();
void sed_imp();
void sei_imp();

//data transfer
void lda_mad();
void ldx_mad();
void ldy_mad();
void sta_mad();
void sta_abx();
void sta_aby();
void sta_iny();
void stx_mad();
void sty_mad();

void tax_imp();
void tay_imp();
void tsx_imp();
void txa_imp();
void txs_imp();
void tya_imp();


//stack
void pha_imp();
void php_imp();
void pla_imp();
void plp_imp();

//var
void x_____x();
void nop_mad();
void cmp_mad();
void cpx_mad();
void cpy_mad();

void cpu_nmi();
void cpu_irq();
void cpu_rst();


u16 reg_pc;
u16 reg_op_addr;
u8 reg_a;
u8 reg_x;
u8 reg_y;
u8 reg_sr;
u8 reg_sp;

u8 cpu_busy;
u8 cpu_opcode;

u8 cpu_int_irq;
u8 cpu_int_nmi;
u8 cpu_int_nmi_old;
u32 cpu_cycle_ctr;

//void(*cpu_instructions[256])();
u8(*cpu_address_mode[256])();

void(*cpu_inst_tbl[256])() = {
    //0x00		0x01		0x02		0x03		0x04		0x05		0x06		0x07		0x08		0x09		0x0A		0x0B		0x0C		0x0D		0x0E		0x0F
    brk_imp, ora_mad, x_____x, x_____x, x_____x, ora_mad, asl_mad, x_____x, php_imp, ora_mad, asl_acc, x_____x, x_____x, ora_mad, asl_mad, x_____x, //0x00
    bpl_rel, ora_mad, x_____x, x_____x, x_____x, ora_mad, asl_mad, x_____x, clc_imp, ora_mad, x_____x, x_____x, x_____x, ora_mad, asl_mad, x_____x, //0x10
    jsr_abs, and_mad, x_____x, x_____x, bit_mad, and_mad, rol_mad, x_____x, plp_imp, and_mad, rol_acc, x_____x, bit_mad, and_mad, rol_mad, x_____x, //0x20
    bmi_rel, and_mad, x_____x, x_____x, x_____x, and_mad, rol_mad, x_____x, sec_imp, and_mad, x_____x, x_____x, x_____x, and_mad, rol_mad, x_____x, //0x30
    rti_imp, eor_mad, x_____x, x_____x, x_____x, eor_mad, lsr_mad, x_____x, pha_imp, eor_mad, lsr_acc, x_____x, jmp_abs, eor_mad, lsr_mad, x_____x, //0x40
    bvc_rel, eor_mad, x_____x, x_____x, x_____x, eor_mad, lsr_mad, x_____x, cli_imp, eor_mad, x_____x, x_____x, x_____x, eor_mad, lsr_mad, x_____x, //0x50
    rts_imp, adc_mad, x_____x, x_____x, x_____x, adc_mad, ror_mad, x_____x, pla_imp, adc_mad, ror_acc, x_____x, jmp_ind, adc_mad, ror_mad, x_____x, //0x60
    bvs_rel, adc_mad, x_____x, x_____x, x_____x, adc_mad, ror_mad, x_____x, sei_imp, adc_mad, x_____x, x_____x, x_____x, adc_mad, ror_mad, x_____x, //0x70
    x_____x, sta_mad, x_____x, x_____x, sty_mad, sta_mad, stx_mad, x_____x, dey_imp, x_____x, txa_imp, x_____x, sty_mad, sta_mad, stx_mad, x_____x, //0x80
    bcc_rel, sta_iny, x_____x, x_____x, sty_mad, sta_mad, stx_mad, x_____x, tya_imp, sta_aby, txs_imp, x_____x, x_____x, sta_abx, x_____x, x_____x, //0x90
    ldy_mad, lda_mad, ldx_mad, x_____x, ldy_mad, lda_mad, ldx_mad, x_____x, tay_imp, lda_mad, tax_imp, x_____x, ldy_mad, lda_mad, ldx_mad, x_____x, //0xA0
    bcs_rel, lda_mad, x_____x, x_____x, ldy_mad, lda_mad, ldx_mad, x_____x, clv_imp, lda_mad, tsx_imp, x_____x, ldy_mad, lda_mad, ldx_mad, x_____x, //0xB0
    cpy_mad, cmp_mad, x_____x, x_____x, cpy_mad, cmp_mad, dec_mad, x_____x, iny_imp, cmp_mad, dex_imp, x_____x, cpy_mad, cmp_mad, dec_mad, x_____x, //0xC0
    bne_rel, cmp_mad, x_____x, x_____x, x_____x, cmp_mad, dec_mad, x_____x, cld_imp, cmp_mad, x_____x, x_____x, x_____x, cmp_mad, dec_mad, x_____x, //0xD0 
    cpx_mad, sbc_mad, x_____x, x_____x, cpx_mad, sbc_mad, inc_mad, x_____x, inx_imp, sbc_mad, nop_mad, x_____x, cpx_mad, sbc_mad, inc_mad, x_____x, //0xE0
    beq_rel, sbc_mad, x_____x, x_____x, x_____x, sbc_mad, inc_mad, x_____x, sed_imp, sbc_mad, x_____x, x_____x, x_____x, sbc_mad, inc_mad, x_____x //0xF0
};



/*
s8 *dbg_cmd[] = {
        //0x00		0x01		0x02		0x03		0x04		0x05		0x06		0x07		0x08		0x09		0x0A		0x0B		0x0C		0x0D		0x0E		0x0F
        "brk_imp",	"ora_mad",	"x_____x",	"x_____x",	"x_____x",	"ora_mad",	"asl_mad",	"x_____x",	"php_imp",	"ora_mad",	"asl_acc",	"x_____x",	"x_____x",	"ora_mad",	"asl_mad",	"x_____x",	//0x00
        "bpl_rel",	"ora_mad",	"x_____x",	"x_____x",	"x_____x",	"ora_mad",	"asl_mad",	"x_____x",	"clc_imp",	"ora_mad",	"x_____x",	"x_____x",	"x_____x",	"ora_mad",	"asl_mad",	"x_____x",	//0x10
        "jsr_abs",	"and_mad",	"x_____x",	"x_____x",	"bit_mad",	"and_mad",	"rol_mad",	"x_____x",	"plp_imp",	"and_mad",	"rol_acc",	"x_____x",	"bit_mad",	"and_mad",	"rol_mad",	"x_____x",	//0x20
        "bmi_rel",	"and_mad",	"x_____x",	"x_____x",	"x_____x",	"and_mad",	"rol_mad",	"x_____x",	"sec_imp",	"and_mad",	"x_____x",	"x_____x",	"x_____x",	"and_mad",	"rol_mad",	"x_____x",	//0x30
        "rti_imp",	"eor_mad",	"x_____x",	"x_____x",	"x_____x",	"eor_mad",	"lsr_mad",	"x_____x",	"pha_imp",	"eor_mad",	"lsr_acc",	"x_____x",	"jmp_abs",	"eor_mad",	"lsr_mad",	"x_____x",	//0x40
        "bvc_rel",	"eor_mad",	"x_____x",	"x_____x",	"x_____x",	"eor_mad",	"lsr_mad",	"x_____x",	"cli_imp",	"eor_mad",	"x_____x",	"x_____x",	"x_____x",	"eor_mad",	"lsr_mad",	"x_____x",	//0x50
        "rts_imp",	"adc_mad",	"x_____x",	"x_____x",	"x_____x",	"adc_mad",	"ror_mad",	"x_____x",	"pla_imp",	"adc_mad",	"ror_acc",	"x_____x",	"jmp_ind",	"adc_mad",	"ror_mad",	"x_____x",	//0x60
        "bvs_rel",	"adc_mad",	"x_____x",	"x_____x",	"x_____x",	"adc_mad",	"ror_mad",	"x_____x",	"sei_imp",	"adc_mad",	"x_____x",	"x_____x",	"x_____x",	"adc_mad",	"ror_mad",	"x_____x",	//0x70
        "x_____x",	"sta_mad",	"x_____x",	"x_____x",	"sty_mad",	"sta_mad",	"stx_mad",	"x_____x",	"dey_imp",	"x_____x",	"txa_imp",	"x_____x",	"sty_mad",	"sta_mad",	"stx_mad",	"x_____x",	//0x80
        "bcc_rel",	"sta_iny",	"x_____x",	"x_____x",	"sty_mad",	"sta_mad",	"stx_mad",	"x_____x",	"tya_imp",	"sta_aby",	"txs_imp",	"x_____x",	"x_____x",	"sta_abx",	"x_____x",	"x_____x",	//0x90
        "ldy_mad",	"lda_mad",	"ldx_mad",	"x_____x",	"ldy_mad",	"lda_mad",	"ldx_mad",	"x_____x",	"tay_imp",	"lda_mad",	"tax_imp",	"x_____x",	"ldy_mad",	"lda_mad",	"ldx_mad",	"x_____x",	//0xA0
        "bcs_rel",	"lda_mad",	"x_____x",	"x_____x",	"ldy_mad",	"lda_mad",	"ldx_mad",	"x_____x",	"clv_imp",	"lda_mad",	"tsx_imp",	"x_____x",	"ldy_mad",	"lda_mad",	"ldx_mad",	"x_____x",	//0xB0
        "cpy_mad",	"cmp_mad",	"x_____x",	"x_____x",	"cpy_mad",	"cmp_mad",	"dec_mad",	"x_____x",	"inx_imp",	"cmp_mad",	"dex_imp",	"x_____x",	"cpy_mad",	"cmp_mad",	"dec_mad",	"x_____x",	//0xC0
        "bne_rel",	"cmp_mad",	"x_____x",	"x_____x",	"x_____x",	"cmp_mad",	"dec_mad",	"x_____x",	"cld_imp",	"cmp_mad",	"x_____x",	"x_____x",	"x_____x",	"cmp_mad",	"dec_mad",	"x_____x",	//0xD0 
        "cpx_mad",	"sbc_mad",	"x_____x",	"x_____x",	"cpx_mad",	"sbc_mad",	"inc_mad",	"x_____x",	"inx_imp",	"sbc_mad",	"nop_mad",	"x_____x",	"cpx_mad",	"sbc_mad",	"inc_mad",	"x_____x",	//0xE0
        "beq_rel",	"sbc_mad",	"x_____x",	"x_____x",	"x_____x",	"sbc_mad",	"inc_mad",	"x_____x",	"sed_imp",	"sbc_mad",	"x_____x",	"x_____x",	"x_____x",	"sbc_mad",	"inc_mad",	"x_____x"	//0xF0
};


void dbg_print() {

printf("reg_pc: %04x, ", reg_pc);
printf("cmd: [0x%02x.", cpu_read(reg_pc));
printf(dbg_cmd[cpu_read(reg_pc)]);
printf("], ");
printf("[%04x], ", reg_op_addr);
printf("A: %02x, ", reg_a);
printf("X: %02x, ", reg_x);
printf("Y: %02x, ", reg_y);
printf("M: %02x, ", memory[0x32]);
//printf("reg_sr: %02x, ", reg_sr);
printf("reg_sr: ");
for (int i = 0; i < 8; i++) {
if (i == 2)printf("-");
if (i != 2)printf("%d", (reg_sr >> (7 - i) & 1));
}
printf(", ");

printf("cycle  : %d ", cpu_cycle_ctr - 1);
getchar();


FILE *file;
fopen_s(&file, "d:/memory.bin", "wb");
fwrite(memory, 1, sizeof(memory), file);
fclose(file);
}
 */

//*************************************************************************************

void cpu_init(u8(*cpu_rd)(u16 addr), void(*cpu_wr)(u16 addr, u8 val)) {

    cpu_read = cpu_rd;
    cpu_write = cpu_wr;

    for (int i = 0; i < 256; i++) {
        cpu_address_mode[i] = addr_null;
    }


    for (int i = 0; i < 16; i += 2) {

        cpu_address_mode[0x01 + (i + 0) * 16] = addr_x_ind;
        cpu_address_mode[0x01 + (i + 1) * 16] = addr_ind_y;

        cpu_address_mode[0x04 + (i + 0) * 16] = addr_zpg;
        cpu_address_mode[0x04 + (i + 1) * 16] = addr_zpg_x;

        cpu_address_mode[0x05 + (i + 0) * 16] = addr_zpg;
        cpu_address_mode[0x05 + (i + 1) * 16] = addr_zpg_x;

        cpu_address_mode[0x06 + (i + 0) * 16] = addr_zpg;
        cpu_address_mode[0x06 + (i + 1) * 16] = addr_zpg_x;

        cpu_address_mode[0x09 + (i + 0) * 16] = addr_imm;
        cpu_address_mode[0x09 + (i + 1) * 16] = addr_abs_y;

        cpu_address_mode[0x0C + (i + 0) * 16] = addr_abs;

        cpu_address_mode[0x0D + (i + 0) * 16] = addr_abs;
        cpu_address_mode[0x0D + (i + 1) * 16] = addr_abs_x;

        cpu_address_mode[0x0E + (i + 0) * 16] = addr_abs;
        cpu_address_mode[0x0E + (i + 1) * 16] = addr_abs_x;
    }

    cpu_address_mode[0xA2] = addr_imm; //ldx
    cpu_address_mode[0x96] = addr_zpg_y; //stx
    cpu_address_mode[0xB6] = addr_zpg_y; //ldx
    cpu_address_mode[0xBC] = addr_abs_x; //ldy
    cpu_address_mode[0xBE] = addr_abs_y; //ldx

    cpu_address_mode[0xA0] = addr_imm; //ldy
    cpu_address_mode[0xC0] = addr_imm; //cpy
    cpu_address_mode[0xE0] = addr_imm; //cpx

    cpu_address_mode[0x1E] = addr_abs_x5; //asl
    cpu_address_mode[0xDE] = addr_abs_x5; //dec
    cpu_address_mode[0xFE] = addr_abs_x5; //inc
    cpu_address_mode[0x5E] = addr_abs_x5; //lsr
    cpu_address_mode[0x3E] = addr_abs_x5; //rol
    cpu_address_mode[0x7E] = addr_abs_x5; //ror

    cpu_int_irq = 1;
    cpu_int_nmi = 1;
    cpu_int_nmi_old = 1;
    cpu_rst();
}

void cpu_cycle() {

    cpu_cycle_ctr++;
    if (cpu_busy != 0) {
        cpu_busy--;
        return;
    }


    if (cpu_int_nmi != cpu_int_nmi_old) {

        cpu_int_nmi_old = cpu_int_nmi;
        if (cpu_int_nmi == 0) {
            cpu_nmi();
            return;
        }
    }

    if (cpu_int_irq == 0 && (reg_sr & SR_I) == 0) {
        cpu_irq();
        return;
    }


    //dbg_print();
    cpu_opcode = cpu_read(reg_pc++);
    cpu_inst_tbl[cpu_opcode]();
    cpu_busy--;


}


//************************************************************************************* addressing

u8 addr_null() {
    printf("addr null: %02x\n", cpu_opcode);
    getchar();
    return 0;
}

u8 addr_x_ind() {//(indirect,X)
    u8 zp_addr = cpu_read(reg_pc++) + reg_x;
    reg_op_addr = cpu_read(zp_addr++);
    reg_op_addr |= cpu_read(zp_addr++) << 8;
    return 6;
}

u8 addr_ind_y() {//(indirect),Y
    u8 zp_addr = cpu_read(reg_pc++);
    reg_op_addr = cpu_read(zp_addr++);
    reg_op_addr |= cpu_read(zp_addr++) << 8;
    reg_op_addr += reg_y;
    if ((reg_op_addr & 0xff) < reg_y)return 6;
    return 5;
}

u8 addr_abs() {
    reg_op_addr = cpu_read(reg_pc++);
    reg_op_addr |= cpu_read(reg_pc++) << 8;
    return 4;
}

u8 addr_abs_x() {
    reg_op_addr = cpu_read(reg_pc++);
    reg_op_addr |= cpu_read(reg_pc++) << 8;
    reg_op_addr += reg_x;
    if ((reg_op_addr & 0xff) < reg_x)return 5;
    return 4;
}

u8 addr_abs_x5() {
    reg_op_addr = cpu_read(reg_pc++);
    reg_op_addr |= cpu_read(reg_pc++) << 8;
    reg_op_addr += reg_x;
    return 5;
}

u8 addr_abs_y() {

    reg_op_addr = cpu_read(reg_pc++);
    reg_op_addr |= cpu_read(reg_pc++) << 8;
    reg_op_addr += reg_y;
    if ((reg_op_addr & 0xff) < reg_y)return 5;
    return 4;
}

u8 addr_imm() {
    reg_op_addr = reg_pc;
    reg_pc++;
    return 2;
}

u8 addr_zpg() {
    reg_op_addr = cpu_read(reg_pc++);
    return 3;
}

u8 addr_zpg_x() {//zeropage,X
    reg_op_addr = (cpu_read(reg_pc++) + reg_x) & 0xff;
    return 4;
}

u8 addr_zpg_y() {
    reg_op_addr = (cpu_read(reg_pc++) + reg_y) & 0xff;
    return 4;
}

//************************************************************************************* var stuff

inline void status_nz(u8 val) {

    reg_sr &= ~(SR_Z | SR_N);
    if (val == 0)reg_sr |= SR_Z;
    reg_sr |= val & SR_N;
}

inline void status_nzc(u8 val, u8 car) {

    reg_sr &= ~(SR_Z | SR_N | SR_C);
    if (val == 0)reg_sr |= SR_Z;
    reg_sr |= (val & SR_N);
    if (car)reg_sr |= SR_C;
}

inline void status_cmp(u8 reg, u8 mem) {

    reg_sr &= ~(SR_Z | SR_N | SR_C);

    if (reg == mem) {
        reg_sr |= SR_Z | SR_C;
    } else {
        if (reg > mem)reg_sr |= SR_C;
        reg_sr |= (reg - mem) & SR_N;
    }
}

inline void push_8(u8 val) {
    cpu_write(0x100 + reg_sp--, val);
}

inline void push_16(u16 val) {
    cpu_write(0x100 + reg_sp--, val >> 8);
    cpu_write(0x100 + reg_sp--, val & 0xff);
}

inline u8 pull_8() {
    return cpu_read(0x100 + ++reg_sp);
}

inline u16 pull_16() {
    u16 val;
    val = cpu_read(0x100 + ++reg_sp);
    val |= cpu_read(0x100 + ++reg_sp) << 8;
    return val;
}

inline void fetch_vector(u16 addr) {
    reg_pc = cpu_read(addr) | (cpu_read(addr + 1) << 8);
}

void x_____x() {
    printf("inst null: %02x\n", cpu_opcode);
    getchar();
}



//***************************************************************************** math

void adc_mad() {

    u8 arg;
    u16 sum;
    cpu_busy = cpu_address_mode[cpu_opcode]();
    arg = cpu_read(reg_op_addr);
    sum = arg + reg_a;
    if ((reg_sr & SR_C))sum++;

    reg_sr &= ~(SR_Z | SR_N | SR_C | SR_V);

    if ((reg_a ^ sum) & (arg ^ sum) & 0x80)reg_sr |= SR_V;

    reg_a = (u8) sum;
    if (sum > 255)reg_sr |= SR_C;
    if (reg_a == 0)reg_sr |= SR_Z;
    reg_sr |= reg_a & SR_N;

}

void sbc_mad() {

    u8 arg;
    u16 sum;
    cpu_busy = cpu_address_mode[cpu_opcode]();
    arg = 256 - (cpu_read(reg_op_addr) + 1);
    sum = arg + reg_a;
    if ((reg_sr & SR_C))sum++;

    reg_sr &= ~(SR_Z | SR_N | SR_C | SR_V);

    if ((reg_a ^ sum) & (arg ^ sum) & 0x80)reg_sr |= SR_V;

    reg_a = (u8) sum;
    if (sum > 255)reg_sr |= SR_C;
    if (reg_a == 0)reg_sr |= SR_Z;
    reg_sr |= reg_a & SR_N;
}

void and_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_a &= cpu_read(reg_op_addr);
    status_nz(reg_a);
}

void bit_mad() {

    u8 mem;
    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_sr &= ~(SR_N | SR_V | SR_Z);

    mem = cpu_read(reg_op_addr);
    if ((mem & 0x80))reg_sr |= SR_N;
    if ((mem & 0x40))reg_sr |= SR_V;
    if ((mem & reg_a) == 0)reg_sr |= SR_Z;
}

void dec_mad() {

    u8 mem;
    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;
    mem = cpu_read(reg_op_addr) - 1;
    cpu_write(reg_op_addr, mem);
    status_nz(mem);
}

void dex_imp() {

    cpu_busy = 2;
    reg_x--;
    status_nz(reg_x);
}

void dey_imp() {

    cpu_busy = 2;
    reg_y--;
    status_nz(reg_y);
}

void inc_mad() {

    u8 mem;
    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;
    mem = cpu_read(reg_op_addr) + 1;
    cpu_write(reg_op_addr, mem);
    status_nz(mem);
}

void inx_imp() {

    cpu_busy = 2;
    reg_x++;
    status_nz(reg_x);
}

void iny_imp() {

    cpu_busy = 2;
    reg_y++;
    status_nz(reg_y);
}

void asl_acc() {

    cpu_busy = 2;

    reg_sr &= ~(SR_Z | SR_N | SR_C);

    if ((reg_a & 0x80))reg_sr |= SR_C;
    reg_a <<= 1;
    if (reg_a == 0)reg_sr |= SR_Z;
    reg_sr |= reg_a & SR_N;
}

void asl_mad() {

    u8 mem;
    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;
    mem = cpu_read(reg_op_addr);

    reg_sr &= ~(SR_Z | SR_N | SR_C);

    if ((mem & 0x80))reg_sr |= SR_C;
    mem <<= 1;
    if (mem == 0)reg_sr |= SR_Z;
    reg_sr |= mem & SR_N;

    cpu_write(reg_op_addr, mem);
}

void lsr_acc() {

    cpu_busy = 2;
    reg_sr &= ~(SR_Z | SR_N | SR_C);
    if ((reg_a & 1))reg_sr |= SR_C;
    reg_a >>= 1;
    if (reg_a == 0)reg_sr |= SR_Z;
}

void lsr_mad() {

    u8 mem;
    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;
    mem = cpu_read(reg_op_addr);

    reg_sr &= ~(SR_Z | SR_N | SR_C);
    if ((mem & 1))reg_sr |= SR_C;
    mem >>= 1;
    if (mem == 0)reg_sr |= SR_Z;
    cpu_write(reg_op_addr, mem);
}

void ora_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_a |= cpu_read(reg_op_addr);
    status_nz(reg_a);
}

void rol_acc() {

    u8 car;

    cpu_busy = 2;
    car = reg_a & 0x80;
    reg_a <<= 1;
    if ((reg_sr & SR_C))reg_a |= 1;

    status_nzc(reg_a, car);
}

void rol_mad() {

    u8 car;
    u8 mem;

    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;

    mem = cpu_read(reg_op_addr);
    car = mem & 0x80;
    mem <<= 1;
    if ((reg_sr & SR_C))mem |= 1;
    cpu_write(reg_op_addr, mem);

    status_nzc(mem, car);
}

void ror_acc() {

    u8 car;

    cpu_busy = 2;
    car = reg_a & 0x01;
    reg_a >>= 1;
    if ((reg_sr & SR_C))reg_a |= 0x80;

    status_nzc(reg_a, car);
}

void ror_mad() {

    u8 car;
    u8 mem;

    cpu_busy = cpu_address_mode[cpu_opcode]() + 2;

    mem = cpu_read(reg_op_addr);
    car = mem & 0x01;
    mem >>= 1;
    if ((reg_sr & SR_C))mem |= 0x80;
    cpu_write(reg_op_addr, mem);

    status_nzc(mem, car);
}

void eor_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_a ^= cpu_read(reg_op_addr);
    status_nz(reg_a);
}

//***************************************************************************** data transfer

void lda_mad() {
    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_a = cpu_read(reg_op_addr);
    status_nz(reg_a);
}

void ldx_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_x = cpu_read(reg_op_addr);
    status_nz(reg_x);
}

void ldy_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    reg_y = cpu_read(reg_op_addr);
    status_nz(reg_y);
}

void sta_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    cpu_write(reg_op_addr, reg_a);
}

void sta_abx() {

    cpu_busy = 5;
    addr_abs_x();
    cpu_write(reg_op_addr, reg_a);
}

void sta_aby() {

    cpu_busy = 5;
    addr_abs_y();
    cpu_write(reg_op_addr, reg_a);
}

void sta_iny() {

    cpu_busy = 6;
    addr_ind_y();
    cpu_write(reg_op_addr, reg_a);
}

void stx_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    cpu_write(reg_op_addr, reg_x);
}

void sty_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    cpu_write(reg_op_addr, reg_y);
}

void tax_imp() {

    cpu_busy = 2;
    reg_x = reg_a;
    status_nz(reg_a);
}

void tay_imp() {

    cpu_busy = 2;
    reg_y = reg_a;
    status_nz(reg_a);
}

void tsx_imp() {

    cpu_busy = 2;
    reg_x = reg_sp;
    status_nz(reg_x);
}

void txa_imp() {

    cpu_busy = 2;
    reg_a = reg_x;
    status_nz(reg_x);
}

void txs_imp() {

    cpu_busy = 2;
    reg_sp = reg_x;
    //status_nz(reg_x);
}

void tya_imp() {

    cpu_busy = 2;
    reg_a = reg_y;
    status_nz(reg_a);
}


//***************************************************************************** branch

inline u8 bra() {

    u16 old_pc = reg_pc;
    s8 offset = cpu_read(reg_pc++);
    reg_pc += offset;
    if (((reg_pc ^ old_pc) & 0x80))return 4;
    return 3;
}

void bcc_rel() {

    if ((reg_sr & SR_C) == 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bcs_rel() {

    if ((reg_sr & SR_C) != 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void beq_rel() {

    if ((reg_sr & SR_Z) != 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bmi_rel() {

    if ((reg_sr & SR_N) != 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bne_rel() {

    if ((reg_sr & SR_Z) == 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bpl_rel() {

    if ((reg_sr & SR_N) == 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bvc_rel() {

    if ((reg_sr & SR_V) == 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void bvs_rel() {

    if ((reg_sr & SR_V) != 0) {
        cpu_busy = bra();
        return;
    }
    reg_pc++;
    cpu_busy = 2;
}

void jmp_abs() {

    cpu_busy = 3;
    reg_pc = cpu_read(reg_pc) | (cpu_read(reg_pc + 1) << 8);
}

void jmp_ind() {

    u8 in_page;
    cpu_busy = 5;
    reg_op_addr = cpu_read(reg_pc) | (cpu_read(reg_pc + 1) << 8);
    reg_pc = cpu_read(reg_op_addr);
    in_page = reg_op_addr + 1;
    reg_pc |= cpu_read((reg_op_addr & 0xff00) | in_page) << 8;
}

void jsr_abs() {

    cpu_busy = 6;
    push_16(reg_pc + 1);
    fetch_vector(reg_pc);
}

void rti_imp() {

    cpu_busy = 6;
    reg_sr = pull_8() | SR_B;
    reg_pc = pull_16();
}

void rts_imp() {

    cpu_busy = 6;
    reg_pc = pull_16() + 1;
}

//***************************************************************************** interrupts

void brk_imp() {

    cpu_busy = 7;
    push_16(reg_pc + 1);
    push_8(reg_sr);
    reg_sr |= SR_I;
    fetch_vector(VECTOR_IRQ);
}

void cpu_nmi() {

    cpu_busy = 7;
    push_16(reg_pc);
    push_8(reg_sr & ~SR_B);
    reg_sr |= SR_I;
    fetch_vector(VECTOR_NMI);
}

void cpu_irq() {

    cpu_busy = 7;
    push_16(reg_pc);
    push_8(reg_sr & ~SR_B);
    reg_sr |= SR_I;
    fetch_vector(VECTOR_IRQ);
}

void cpu_rst() {

    cpu_cycle_ctr = 0;
    cpu_busy = 0; // 6;
    fetch_vector(VECTOR_RST);
    reg_sr = SR_5 | SR_I | SR_B;
}

//***************************************************************************** statu reg

void clc_imp() {

    cpu_busy = 2;
    reg_sr &= ~SR_C;
}

void cld_imp() {

    cpu_busy = 2;
    reg_sr &= ~SR_D;
}

void cli_imp() {

    cpu_busy = 2;
    reg_sr &= ~SR_C;
}

void clv_imp() {

    cpu_busy = 2;
    reg_sr &= ~SR_V;
}

void sec_imp() {

    cpu_busy = 2;
    reg_sr |= SR_C;
}

void sed_imp() {

    cpu_busy = 2;
    reg_sr |= SR_D;
}

void sei_imp() {

    cpu_busy = 2;
    reg_sr |= SR_I;
}
//***************************************************************************** stack

void pha_imp() {

    cpu_busy = 3;
    push_8(reg_a);
}

void php_imp() {

    cpu_busy = 3;
    push_8(reg_sr);
}

void pla_imp() {

    cpu_busy = 4;
    reg_a = pull_8();
    status_nz(reg_a);
}

void plp_imp() {

    cpu_busy = 4;
    reg_sr = pull_8() | SR_B | SR_5;
}

//***************************************************************************** var instructions

void nop_mad() {

    cpu_busy = 2;
}

void cmp_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    status_cmp(reg_a, cpu_read(reg_op_addr));
}

void cpx_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    status_cmp(reg_x, cpu_read(reg_op_addr));
}

void cpy_mad() {

    cpu_busy = cpu_address_mode[cpu_opcode]();
    status_cmp(reg_y, cpu_read(reg_op_addr));
}