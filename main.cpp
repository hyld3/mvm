#include <iostream>
#include <bitset>
#include <limits>
#include <memory>
#include <cassert>
#include <map>
#include <ctime>
#include <csignal>

#include <termios.h>

#include <sys/select.h>

#define _STDIN_FILENO 0
#define _STDOUT_FILENO 1
#define _STDERR_FILENO 2

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

std::map<uint16_t, std::string> opcode_mnemonics =
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

enum {
      MR_KBSR = 0xFE00, // Keyboard status
      MR_KBDR = 0xFE02  // Keyboard data
};


// VM memory and registers
uint16_t reg[R_COUNT];
uint16_t memory[UINT16_MAX];
//

struct termios original_tio;

void disable_input_buffering() {
  tcgetattr(_STDIN_FILENO, &original_tio);

  struct termios new_tio = original_tio;
  new_tio.c_lflag &= ~ICANON & ~ECHO;

  tcsetattr(_STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
  tcsetattr(_STDIN_FILENO, TCSANOW, &original_tio);
}

void handle_interrupt(int signal) {
  restore_input_buffering();
  std::cout << std::endl;
  std::exit(-2);
}

uint16_t sign_extend(uint16_t x, int bit_count) {
  if ((x >> (bit_count - 1)) & 1) {
    x |= (0xFFFF << bit_count);
  }
  return x;
}

uint16_t swap16(uint16_t x) {
  return (x << 8) | (x >> 8);
}

void read_image_file(FILE * file) {
  uint16_t origin;

  fread(&origin, sizeof(origin), 1, file);
  origin = swap16(origin);

  uint16_t max_read = UINT16_MAX - origin;
  uint16_t * p = memory + origin;

  size_t read = fread(p, sizeof(uint16_t), max_read, file);

  while (read-- > 0) {
    *p = swap16(*p);
    ++p;
  }
}

bool read_image(char * imgp) {
  FILE * file = fopen(imgp, "rb");
  if (!file) {return false;}
  read_image_file(file);

  fclose(file);
  return true;
}

uint16_t check_key() {

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(0, &readfds);

  struct timeval timeout = { .tv_usec = 0x0, .tv_sec = 0x0 };

  return select(1, &readfds, NULL, NULL, &timeout) != 0; 
}

void mem_write(uint16_t addr, uint16_t val) {
  memory[addr] = val;
}

uint16_t mem_read(uint16_t addr) {
  if (addr == MR_KBSR) {
    if (check_key()) {
      memory[MR_KBSR] = (1 << 15);
      memory[MR_KBDR] = std::getchar();
    } else {
      memory[MR_KBSR] = 0x0;
    }
  }
  return memory[addr];
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

  signal(SIGINT, handle_interrupt);
  disable_input_buffering();

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
	reg[R_PC] = reg[r1];
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
	  {
	    reg[R_R0] = (uint16_t) std::getchar();
	  }
	  break;
	case TRAP_OUT:
	  {
	    std::putc((char)reg[R_R0], stdout);
	    std::fflush(stdout);
	  }
	  break;
	case TRAP_PUTS:
	  {
	    uint16_t * c = memory + reg[R_R0];
	    while (*c) {
	      std::putc((char) * c, stdout);
	      ++c;
	    }
	    std::fflush(stdout);
	  }
	  break;
	case TRAP_IN:
	  {
	    std::cout << "[*] Enter character: " << std::endl;
	    char c = std::getchar();
	    std::putc(c, stdout);
	    reg[R_R0] = (uint16_t) c;
	  }
	  break;
	case TRAP_PUTSP:
	  {
	    uint16_t * c = memory + reg[R_R0];

	    while(*c) {
	      char o[2] = {0x0};
	      o[0] = (*c) & 0xFF;
	      std::putc(o[0], stdout);
	      o[1] = (*c) >> 8;
	      if (o[1])
		std::putc(o[1], stdout);
	      ++c;
	    }
	    std::fflush(stdout);
	  }
	  break;
	case TRAP_HALT:
	  {
	    std::cout << "[*] HALTING" << std::endl;
	    std::fflush(stdout);
	    running ^= 1;
	  }
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
  restore_input_buffering();
  
  return 0;
}


