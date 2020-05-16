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

std::unordered_map<uint16_t, std::string> opcode_mnemonics =
  {
   {OP_BR, "BR"},
   {OP_ADD, "ADD"},
   {OP_LD, "LD"},
   {OP_ST, "ST"},
   {OP_JSR, "JSR"},
   {OP_AND, "AND"},
   {OP_LDR, "LDR"},
   {OP_STR, "STR"},
   {OP_RTI, "RTI"},
   {OP_NOT, "NOT"},
   {OP_LDI, "LDI"},
   {OP_STI, "STI"},
   {OP_JMP, "JMP"},
   {OP_RES, "RES"},
   {OP_LEA, "LEA"},
   {OP_TRAP, "TRAP"},
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

// IO interaction
enum {
      TRAP_GETC = 0x20, // Get char, no echo
      TRAP_OUT = 0x21,  // Output char
      TRAP_PUTS = 0x22, // Output string
      TRAP_IN = 0x23,   // Get char, echo 
      TRAP_PUTSP = 0x24,// Output bin
      TRAP_HALT = 0x25  // Halt and stfu
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

void unused_opcode(uint16_t opcode) {
  std::cout << "[-] Met bad opcode" <<  opcode_mnemonics[opcode] << std::endl;
  abort();
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
    uint16_t opcode =  instr >> 12;
    
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
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t r1 = (instr >> 6) & 0x7;
	bool imm_flag = (instr >> 5) & 0x1;

	if (imm_flag) {
	  uint16_t imm5 = sign_extend(instr & 0x1F, 5);
	  reg[r0] = reg[r1] & imm5;
	} else {
	  uint16_t r2 = instr & 0x7;
	  reg[r0] = reg[r1] & reg[r2];
	}
	update_flags(r0);
      }
      break;
    case OP_NOT:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t r1 = (instr >> 6) & 0x7;

	reg[r0] = ~reg[r1];
	update_flags(r0);
      }
      break;
    case OP_BR:
      {
	// Fix pc relative address
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	uint16_t cond_flag = (instr >> 9) & 0x7;

	if (cond_flag & reg[R_COND]) {
	  reg[R_PC] += pc_offset;
	}
      }
      break;
    case OP_JMP:
      {
	// Don't account for validity, just YUMP.
	uint16_t r1 = (instr >> 6) & 0x7;
	reg[R_PC] = reg[R1];
      }
      break;
    case OP_JSR:
      {
	bool long_flag = (instr >> 11) & 1;
	reg[R_R7] = reg[R_PC];

	if (long_flag) {
	  uint16_t long_pc_offset = sign_extend(instr & 0x7FF, 11);
	  reg[R_PC] += long_pc_offset;
	    } else {
	  uint16_t r1 = (instr >> 6) & 0x7;
	  reg[R_PC] = reg[r1];
	}
      }
      break;
    case OP_LD:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	reg[r0] = mem_read(reg[R_PC] + pc_offset);
	update_flags(r0);
      }
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
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t r1 = (instr >> 6) & 0x7;
	uint16_t offset = sign_extend(instr & 0x3F, 6);
	reg[r0] = mem_read(reg[r1] + offset);
	update_flags(r0);
      }
      break;
    case OP_LEA:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	reg[r0] = reg[R_PC] + pc_offset;
	update_flags(r0);
      }
      break;
    case OP_ST:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	mem_write(reg[R_PC] + pc_offset, reg[r0]);
      }
      break;
    case OP_STI:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t pc_offset = sign_extend(instr & 0x1FF, 9);
	mem_write(mem_read(reg[R_PC] + pc_offset), reg[r0]);
      }
      break;
    case OP_STR:
      {
	uint16_t r0 = (instr >> 9) & 0x7;
	uint16_t r1 = (instr >> 6) & 0x7;
	uint16_t offset = sign_extend(instr & 0x3F, 6);
	mem_write(reg[r1] + offset, reg[r0]);
      }
      break;
    case OP_TRAP:
      {
	switch(instr & 0xFF) {
	case TRAP_GETC:
	  break;
	case TRAP_OUT:
	  break;
	case TRAP_PUTS:
	  {
	    uint16_t * c = memory + reg[R_R0];
	    while (*c) {
	      std::putc((char) * c, stdout);
	    }
	  }
	  break;
	case TRAP_IN:
	  break;
	case TRAP_PUTSP:
	  break;
	case TRAP_HALT:
	  break;
	default:
	  break;
	}
      }
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


