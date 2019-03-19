/*
 Reinette, the french Apple 1 emulator
 Last modified 19th of March 2019

 Copyright (c) 2018, 2019 Arthur Ferreira

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 */

#include <ncurses.h>
#include <unistd.h>      // for usleep()

#include "woz.h"


// modified to run the klaus Test suite : romless
// #define RAMSIZE 0xC000  // 48KB
#define RAMSIZE 0x10000  // 64KB


uint8_t ram[RAMSIZE];

#define CARRY 0x01
#define ZERO 0x02
#define INTERRUPT 0x04
#define DECIMAL 0x08
#define BREAK 0x10
#define UNDEFINED 0x20
#define OVERFLOW 0x40
#define SIGN 0x80

struct Operand{
  bool setAcc;
  uint16_t value, address;
}ope;

struct Register{
  uint8_t A,X,Y,SR,SP;
  uint16_t PC;
}reg;

uint8_t key, keyRdy;


// MEMORY AND I/O

static uint8_t readMem(uint16_t address){
  static uint8_t queries=0; // slow down emulation when waiting for a keypress

  if (address < RAMSIZE)   return (ram[address]);
  if (address >= ROMSTART) return (rom[address - ROMSTART]);

  if (address == 0xD011 ){        // is there a keypressed ?
    if (keyRdy) return(keyRdy);   // yes
    if (! ++queries) usleep(100); // else sleep 100ms every 256 iterations
    return(0);                    // and return 0 (no keypressed)
  }
  if ((address == 0xD010) && keyRdy){  // is there a key waiting us ?
    keyRdy = 0;                        // yes, reset the keyRdy flag
    return(key | 0x80);                // and return the key
  }
  return(0);  // catch all
}

static void writeMem(uint16_t address, uint8_t value){
  if (address < RAMSIZE) ram[address] = value;

  else if (address == 0xD012){  // DSP, display one char
    value &= 0x7F;
    if (value == 0x7F) value = '@';    // make DEL printable
    if (value == 0x0D) value = 0x0A;   // CR (\r) to LF (\n)
    if (value == 0x5F)                 // erase the previous character
      printw("%c%c%c",0x08,0x20,0x08); // BackSpace, Space , BackSpace
    else printw("%c",value);
  }
}


// RESET

static void reset(){
  reg.PC = readMem(0xFFFC) | (readMem(0xFFFD) << 8);
  reg.SP = 0xFF;
  reg.SR |= UNDEFINED;
  ope.setAcc = false;
  ope.value = 0;
  ope.address = 0;
  keyRdy = 0;
}


// STACK, SIGN AND ZERO FLAGS ROUTINES

static void push(uint8_t value){
  writeMem(0x100 + reg.SP--, value);
}

uint8_t pull(){
  return(readMem(0x100 + ++reg.SP));
}

static void setSZ(uint8_t value){  //  updates both the Sign & Zero FLAGS
  if (value & 0x00FF) reg.SR &= ~ZERO;
  else reg.SR |= ZERO;
  if (value & 0x80) reg.SR |= SIGN;
  else reg.SR &= ~SIGN;
}


// ADDRESSING MODES

static void IMP(){  // Implicit
}

static void ACC(){  // ACCumulator
  ope.value = reg.A;
  ope.setAcc = true;
}

static void IMM(){  // IMMediate
  ope.address = reg.PC++;
  ope.value = readMem(ope.address);
}

static void ZPG(){  // Zero PaGe
  ope.address = readMem(reg.PC++);
  ope.value = readMem(ope.address);
}

static void ZPX(){  // Zero PaGe,X
  ope.address = (readMem(reg.PC++) + reg.X) & 0xFF;
  ope.value = readMem(ope.address);
}

static void ZPY(){  // Zero PaGe,Y
  ope.address = (readMem(reg.PC++) + reg.Y) & 0xFF;
  ope.value = readMem(ope.address);
}

static void REL(){  // RELative (for branch instructions)
  ope.address = readMem(reg.PC++);
  if (ope.address & 0x80) ope.address |= 0xFF00;  // branch backward
}

static void ABS(){  // ABSolute
  ope.address = readMem(reg.PC) | (readMem(reg.PC + 1) << 8);
  ope.value = readMem(ope.address);
  reg.PC += 2;
}

static void ABX(){  // ABsolute,X
  ope.address = (readMem(reg.PC) | (readMem(reg.PC + 1) << 8)) + reg.X;
  ope.value = readMem(ope.address);
  reg.PC += 2;
}

static void ABY(){  // ABsolute,Y
  ope.address = (readMem(reg.PC) | (readMem(reg.PC + 1) << 8)) + reg.Y;
  ope.value = readMem(ope.address);
  reg.PC += 2;
}

static void IND(){  // INDirect - JMP ($ABCD) with page-boundary wraparound bug
  uint16_t vector1 = readMem(reg.PC) | (readMem(reg.PC + 1) << 8);
  uint16_t vector2 = (vector1 & 0xFF00) | ((vector1 + 1) & 0x00FF);
  ope.address  = readMem(vector1) | (readMem(vector2) << 8);
  ope.value = readMem(ope.address);
  reg.PC += 2;
}

static void IDX(){  // InDexed indirect X
  uint16_t vector1 = ((readMem(reg.PC++) + reg.X) & 0xFF);
  ope.address = readMem(vector1 & 0x00FF)|(readMem((vector1+1) & 0x00FF) << 8);
  ope.value = readMem(ope.address);
}

static void IDY(){  // InDirect Indexed Y
  uint16_t vector1 = readMem(reg.PC++);
  uint16_t vector2 = (vector1 & 0xFF00) | ((vector1 + 1) & 0x00FF);
  ope.address = (readMem(vector1) | (readMem(vector2) << 8)) + reg.Y;
  ope.value = readMem(ope.address);
}


// INSTRUCTIONS

static void NOP(){  // NO Operation
}

static void BRK(){  // BReaK
  push(((++reg.PC) >> 8) & 0xFF);
  push(reg.PC & 0xFF);
  push(reg.SR | BREAK);
  reg.SR |= INTERRUPT;
  reg.PC = readMem(0xFFFE) | (readMem(0xFFFF) << 8);
}

static void CLD(){  // CLear Decimal
  reg.SR &= ~DECIMAL;
}

static void SED(){  // SEt Decimal
  reg.SR |= DECIMAL;
}

static void CLC(){  // CLear Carry
  reg.SR &= ~CARRY;
}

static void SEC(){  // SEt Carry
  reg.SR |= CARRY;
}

static void CLI(){  // CLear Interrupt
  reg.SR &= ~INTERRUPT;
}

static void SEI(){  // SEt Interrupt
  reg.SR |= INTERRUPT;
}

static void CLV(){  // CLear oVerflow
  reg.SR &= ~OVERFLOW;
}

static void LDA(){  // LoaD Accumulator
  reg.A = ope.value;
  setSZ(reg.A);
}

static void LDX(){  // LoaD X
  reg.X = ope.value;
  setSZ(reg.X);
}

static void LDY(){  // LoaD Y
  reg.Y = ope.value;
  setSZ(reg.Y);
}

static void STA(){  // STore Accumulator
  writeMem(ope.address, reg.A);
}

static void STX(){  // STore X
  writeMem(ope.address, reg.X);
}

static void STY(){  // STore Y
  writeMem(ope.address, reg.Y);
}

static void DEC(){  // DECrement
  writeMem(ope.address, --ope.value);
  setSZ(ope.value);
}

static void DEX(){  // DEcrement X
  setSZ(--reg.X);
}

static void DEY(){  // DEcrement Y
  setSZ(--reg.Y);
}

static void INC(){  // INCrement
  writeMem(ope.address, ++ope.value);
  setSZ(ope.value);
}

static void INX(){  // INcrement X
  setSZ(++reg.X);
}

static void INY(){  // INcrement Y
  setSZ(++reg.Y);
}

static void TAX(){  // Transfer Accumulator to X
  reg.X = reg.A;
  setSZ(reg.X);
}

static void TAY(){  // Transfer Accumulator to Y
  reg.Y = reg.A;
  setSZ(reg.Y);
}

static void TXA(){  // Transfer X to Accumulator
  reg.A = reg.X;
  setSZ(reg.A);
}

static void TYA(){  // Transfer Y to Accumulator
  reg.A = reg.Y;
  setSZ(reg.A);
}

static void TSX(){  // Transfer Sp to X
  reg.X = reg.SP;
  setSZ(reg.X);
}

static void TXS(){  // Transfer X to Sp
  reg.SP = reg.X;
}

static void BEQ(){  // Branch on EQual (zero set)
  if (reg.SR & ZERO) reg.PC += ope.address;
}

static void BNE(){  // Branch on Not Equal (zero clear)
  if (!(reg.SR & ZERO)) reg.PC += ope.address;
}

static void BMI(){  // Branch if MInus (ie when negative, when SIGN is set)
  if (reg.SR & SIGN) reg.PC += ope.address;
}

static void BPL(){  // Branch if PLus (ie when positive, when SIGN is clear)
  if (!(reg.SR & SIGN)) reg.PC += ope.address;
}

static void BVS(){  // Branch on oVerflow Set
  if (reg.SR & OVERFLOW) reg.PC += ope.address;
}

static void BVC(){  // Branch on oVerflow Clear
  if (!(reg.SR & OVERFLOW)) reg.PC += ope.address;
}

static void BCS(){  // Branch on Carry Set
  if (reg.SR & CARRY) reg.PC +=ope.address;
}

static void BCC(){  // Branch on Carry Clear
  if (!(reg.SR & CARRY)) reg.PC += ope.address;
}

static void PHA(){  // PusH A to the stack
  push(reg.A);
}

static void PLA(){  // PulL stack into A
  reg.A = pull();
  setSZ(reg.A);
}

static void PHP(){  // PusH Programm (Status) register to the stack
  push(reg.SR | BREAK);
}

static void PLP(){  // PulL stack into Programm (SR) register
  reg.SR = pull() | UNDEFINED;
}

static void JMP(){  // JuMP
  reg.PC = ope.address;
}

static void JSR(){  // Jump Sub-Routine
  push((--reg.PC >> 8) & 0xFF);
  push(reg.PC & 0xFF);
  reg.PC = ope.address;
}

static void RTS(){  // ReTurn from Sub-routine
  reg.PC = (pull() | (pull() << 8)) + 1;
}

static void RTI(){  // ReTurn from Interrupt
  reg.SR = pull();
  reg.PC = pull() | (pull() << 8);
}

static void CMP(){  // Compare with A
  setSZ(reg.A - ope.value);
  if (reg.A >= ope.value) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
}

static void CPX(){  // Compare with X
  setSZ(reg.X - ope.value);
  if (reg.X >= ope.value) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
}

static void CPY(){  // Compare with Y
  setSZ(reg.Y - ope.value);
  if (reg.Y >= ope.value) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
}

static void AND(){  // AND with A
  reg.A &= ope.value;
  setSZ(reg.A);
}

static void ORA(){  // OR with A
  reg.A |= ope.value;
  setSZ(reg.A);
}

static void EOR(){  // Exclusive Or with A
  reg.A ^= ope.value;
  setSZ(reg.A);
}

static void BIT(){  // BIT with A - http://www.6502.org/tutorials/vflag.html
  if (reg.A & ope.value) reg.SR &= ~ZERO;
  else reg.SR |= ZERO;
  reg.SR = (reg.SR & 0x3F) | (ope.value & 0xC0);  // update SIGN & OVERFLOW
}

static void makeUpdates(uint8_t val){
  if (ope.setAcc) reg.A = val;
  else writeMem(ope.address, val);
  ope.setAcc = false;
  setSZ(val);
}

static void ASL(){  // Arithmetic Shift Left
  uint16_t result = (ope.value << 1);
  if (result & 0xFF00) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  makeUpdates((uint8_t)(result & 0xFF));
}

static void LSR(){  // Logical Shift Right
  if (ope.value & 1) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  makeUpdates((uint8_t)((ope.value >> 1) & 0xFF));
}

static void ROL(){  // ROtate Left
  uint16_t result = ((ope.value << 1) | (reg.SR & CARRY));
  if (result & 0x100) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  makeUpdates((uint8_t)(result & 0xFF));
}

static void ROR(){  // ROtate Right
  uint16_t result = (ope.value >> 1) | ((reg.SR & CARRY) << 7);
  if (ope.value & 0x1) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  makeUpdates((uint8_t)(result & 0xFF));
}

static void ADC(){  // ADd with Carry
  uint16_t result = reg.A + ope.value + (reg.SR & CARRY);
  setSZ(result);
  if (((result)^(reg.A ))&((result)^(ope.value))&0x0080) reg.SR |= OVERFLOW;
  else reg.SR &= ~OVERFLOW;
  if (reg.SR&DECIMAL) result += ((((result+0x66)^reg.A^ope.value)>>3)&0x22)*3;
  if (result & 0xFF00) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  reg.A = (result & 0xFF);
}

static void SBC(){  // SuBtract with Carry
  ope.value ^= 0xFF;
  if (reg.SR & DECIMAL) ope.value -= 0x0066;
  uint16_t result = reg.A + ope.value + (reg.SR & CARRY);
  setSZ(result);
  if (((result)^(reg.A ))&((result)^(ope.value))&0x0080) reg.SR |= OVERFLOW;
  else reg.SR &= ~OVERFLOW;
  if (reg.SR&DECIMAL) result += ((((result+0x66)^reg.A^ope.value)>>3)&0x22)*3;
  if (result & 0xFF00) reg.SR |= CARRY;
  else reg.SR &= ~CARRY;
  reg.A = (result & 0xFF);
}

static void UND(){  // UNDefined (not a valid or supported 6502 opcode)
  printw("\n\n~ Illegal Instruction At Address $%04X ~\n", reg.PC - 1);
  BRK();
}


// JUMP TABLES

static void (*instruction[])(void) = {
 BRK, ORA, UND, UND, UND, ORA, ASL, UND, PHP, ORA, ASL, UND, UND, ORA, ASL, UND,
 BPL, ORA, UND, UND, UND, ORA, ASL, UND, CLC, ORA, UND, UND, UND, ORA, ASL, UND,
 JSR, AND, UND, UND, BIT, AND, ROL, UND, PLP, AND, ROL, UND, BIT, AND, ROL, UND,
 BMI, AND, UND, UND, UND, AND, ROL, UND, SEC, AND, UND, UND, UND, AND, ROL, UND,
 RTI, EOR, UND, UND, UND, EOR, LSR, UND, PHA, EOR, LSR, UND, JMP, EOR, LSR, UND,
 BVC, EOR, UND, UND, UND, EOR, LSR, UND, CLI, EOR, UND, UND, UND, EOR, LSR, UND,
 RTS, ADC, UND, UND, UND, ADC, ROR, UND, PLA, ADC, ROR, UND, JMP, ADC, ROR, UND,
 BVS, ADC, UND, UND, UND, ADC, ROR, UND, SEI, ADC, UND, UND, UND, ADC, ROR, UND,
 UND, STA, UND, UND, STY, STA, STX, UND, DEY, UND, TXA, UND, STY, STA, STX, UND,
 BCC, STA, UND, UND, STY, STA, STX, UND, TYA, STA, TXS, UND, UND, STA, UND, UND,
 LDY, LDA, LDX, UND, LDY, LDA, LDX, UND, TAY, LDA, TAX, UND, LDY, LDA, LDX, UND,
 BCS, LDA, UND, UND, LDY, LDA, LDX, UND, CLV, LDA, TSX, UND, LDY, LDA, LDX, UND,
 CPY, CMP, UND, UND, CPY, CMP, DEC, UND, INY, CMP, DEX, UND, CPY, CMP, DEC, UND,
 BNE, CMP, UND, UND, UND, CMP, DEC, UND, CLD, CMP, UND, UND, UND, CMP, DEC, UND,
 CPX, SBC, UND, UND, CPX, SBC, INC, UND, INX, SBC, NOP, UND, CPX, SBC, INC, UND,
 BEQ, SBC, UND, UND, UND, SBC, INC, UND, SED, SBC, UND, UND, UND, SBC, INC, UND
};

static void (*addressing[])(void) = {
 IMP, IDX, IMP, IMP, IMP, ZPG, ZPG, IMP, IMP, IMM, ACC, IMP, IMP, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP,
 ABS, IDX, IMP, IMP, ZPG, ZPG, ZPG, IMP, IMP, IMM, ACC, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP,
 IMP, IDX, IMP, IMP, IMP, ZPG, ZPG, IMP, IMP, IMM, ACC, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP,
 IMP, IDX, IMP, IMP, IMP, ZPG, ZPG, IMP, IMP, IMM, ACC, IMP, IND, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP,
 IMP, IDX, IMP, IMP, ZPG, ZPG, ZPG, IMP, IMP, IMP, IMP, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, ZPX, ZPX, ZPY, IMP, IMP, ABY, IMP, IMP, IMP, ABX, IMP, IMP,
 IMM, IDX, IMM, IMP, ZPG, ZPG, ZPG, IMP, IMP, IMM, IMP, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, ZPX, ZPX, ZPY, IMP, IMP, ABY, IMP, IMP, ABX, ABX, ABY, IMP,
 IMM, IDX, IMP, IMP, ZPG, ZPG, ZPG, IMP, IMP, IMM, IMP, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP,
 IMM, IDX, IMP, IMP, ZPG, ZPG, ZPG, IMP, IMP, IMM, IMP, IMP, ABS, ABS, ABS, IMP,
 REL, IDY, IMP, IMP, IMP, ZPX, ZPX, IMP, IMP, ABY, IMP, IMP, IMP, ABX, ABX, IMP
};


// PROGRAM ENTRY POINT

int main(int argc, char *argv[]) {
  int i = 0;//, ch = 0;
  uint8_t opcode = 0;

  // ncurses initialization
  initscr();
  cbreak();
  noecho();
  qiflush();
  scrollok(stdscr, TRUE);
  nodelay(stdscr, TRUE);

  // processor reset
  reset();


         // load the Klaus Test Suite into the 64KB of RAM
         FILE *f=fopen("6502_functional_test.bin","rb");
         while(fread(ram+i, 1, 1, f)) i++;
         // set the Program Counter to 0x400
         reg.PC=0x400;


  // main loop
  while(1){
    for (i=0; i<100; i++){        // execute 100 instructions before a kbd scan
      opcode = readMem(reg.PC++); // FETCH and increment the Program Counter
      addressing[opcode]();       // DECODE operands against the addressing mode
      instruction[opcode]();      // EXEC the instruction
    }


        // print the Program Counter every 100 instructions to detect faults
        move(0,0);
        printw("PC = $%04X",reg.PC);
        refresh();
        }
        }

        // Alter a few seconds, PC is stuck at $3469 => all the tests passed

        /*  commented out to run the klaus Test Suite

    // keyboard controller
    if (!keyRdy){                       // only if not already a key in wait
      if ((ch = getch()) != ERR){       // non blocking keybd read from ncurses
        key = (uint8_t)ch;              // getch() returns an int
        if (key == 0x12) reset();       // CTRL-R, reset
        else if (key == 0x02) BRK();    // CTRL-B, break
        else {
          if (key == 0x0A) key = 0x0D;  // LF (\n) to CR (\r)
          if ((key == 0x7F) || (key == 0x08)) key = 0x5F;  // DEL and BS to _
          if ((key >= 0x61) && (key <= 0x7A)) key &= 0xDF; // to upper case
          keyRdy = 0x80;
        }
      }
    }
  }
}
        commented out to run the klaus Test Suite */
