#include <iostream>
#include <bitset>
#include <limits>
#include <memory>
#include <cassert>
#include <map>

#include <bits/stdc++.h>

enum {
      OP_BR = 0,
      OP_ADD,
      OP_LD,
      OP_ST,
      OP_JSR,
      OP_AND,
      OP_LDR,
      OP_STR,
      OP_RTI,
      OP_NOT,
      OP_LDI,
      OP_STI,
      OP_JMP,
      OP_RES,
      OP_LEA,
      OP_TRAP,
};

enum {
      R_R0 = 0,
      R_R1,
      R_R2,
      R_R3,
      R_R4,
      R_R5,
      R_R6,
      R_R7,
      R_PC,
      R_COND,
      R_COUNT
};



enum {
      FL_POS = 1 << 0,
      FL_ZRO = 1 << 1,
      FL_NEG = 1 << 2
};

uint16_t reg[R_COUNT];
uint16_t memory[UINT16_MAX];


uint16_t sign_extend(uint16_t x, int bit_count) {
  if ((x >> (bit_count - 1)) & 1) {
    x |= (0xFFFF << bit_count);
  }
  return x;
}


bool read_image(char * img) {

  return true;
}

uint16_t mem_read(uint16_t x) {
  return x;
}

void update_flags(uint16_t r) {
  if (reg[r] == 0) {
    reg[R_COND] = FL_ZRO;
  } else if (reg[r] >> 15) {
    reg[R_COND] = FL_NEG;
  } else {
    reg[R_COND] = FL_POS;
  }
}

std::string humanize_opcode() {

};

void unused_opcode(uint16_t opcode) {
  std::cout << "[-] Met bad opcode" 
}

int main(int argc, char * argv[]) {

  if (argc < 2) {
    std::cout << "[*] Usage: " << argv[0] << " " << "[image]" << std::endl;
    std::exit(2);
  }

  if (!read_image(argv[1])) {
    std::cout << "[-] Failed to load image: " << argv[1] << std::endl;
    std::exit(2);
  }

  enum { PC_START = 0x3000};

  reg[R_PC] = PC_START;

  bool running = true;

  while (running) {

    uint16_t instr = mem_read(reg[R_PC]++);
    uint16_t opcode = instr >> 12;
    
    switch(opcode) {
    case OP_ADD:
      {
	uint16_t r0 = (instr >> 9) & 0x7; // Dest
	uint16_t r1 = (instr >> 6) & 0x7; // Src
	bool imm_flag = (instr >> 5) & 0x1; // IMM

	if (imm_flag) {
	  uint16_t imm5 = sign_extend(instr & 0x1F, 5);
	  reg[r0] = reg[r1] + imm5;
	} else {
	  uint16_t r2 = instr & 0x7;
	  reg[r0] = reg[r1] + reg[r2];
	}
	update_flags(r0);
      }
      break;
    case OP_AND:
      {
	
      }
      break;
    case OP_NOT:
      break;
    case OP_BR:
      break;
    case OP_JMP:
      break;
    case OP_JSR:
      break;
    case OP_LD:
      break;
    case OP_LDI:
      {
	uint16_t r0 = (instr >> 9) & 0x7; // DR
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	reg[r0] = mem_read(mem_read(reg[R_PC] + pc_offset));
	update_flags(r0);
      }
      break;
    case OP_LDR:
      break;
    case OP_LEA:
      break;
    case OP_ST:
      break;
    case OP_STI:
      break;
    case OP_STR:
      break;
    case OP_TRAP:
      break;
    case OP_RES:
    case OP_RTI:
      unused_opcode(opcode);
      break;
    default:
      // Bad opcode
      break;
    }
  }
  
  return 0;
}


