/* VM simulates LC-3 computer */
#include <stdlib.h>
#include <stdio.h>

// each memory address holds 16 bits, total: 128Kb
uint16_t memory[UINT16_MAX];

// 10 registers, with 8 general purpose used to perform
// calculations
enum {
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,   // program counter holds address of next instruction to execute
    R_COND, // condition flag stores information about previous calculation
    R_COUNT
};

uint16_t registers[R_COUNT];

// instruction: command for CPU to perform, consist of opcode
// which identifies task and parameters for input

// LC-3 is a RISC since it has few instructions

// left 4 bits of an instruction form opcode
enum {
    OP_BR = 0,  // branch
    OP_ADD,     // add
    OP_LD,      // load
    OP_ST,      // store
    OP_JSR,     // jump register
    OP_AND,     // bitwise and
    OP_LDR,     // load register
    OP_STR,     // store register
    OP_RTI,     // unused
    OP_NOT,     // bitwise not
    OP_LDI,     // load indirect
    OP_STI,     // store indirect
    OP_JMP,     // jump
    OP_RES,     // reserved, unused
    OP_LEA,     // load effective address
    OP_TRAP     // run trap
};

// condition flags indicate sign of previous calculation
enum {
    FL_POS = 1 << 0,
    FL_ZRO = 1 << 1,
    FL_NEG = 1 << 2
};

// trap routines allow for I/O and other functionality
// each has an identifier, its trap code
enum {
    TRAP_GETC = 0x20,   // get character from stdin, not echoed
    TRAP_OUT = 0x21,    // output character
    TRAP_PUTS = 0x22,   // output string
    TRAP_IN = 0x23,     // get character from stdin, echoed onto terminal
    TRAP_PUTSP = 0x24,  // output byte string
    TRAP_HALT = 0x25    // halt program
};

// extend bits in number val, consisting of bit_count bits,
// so it has 16 like all other values in memory
uint16_t sign_extend(uint16_t val, int bit_count) {
    // extract leftmost bit to check whether val is negative
    // since negative values need additional 1 bits
    if ((val >> (bit_count - 1)) & 0x1) {
        val |= (0xFFFF << bit_count);
    }
    return val;
}

// record the sign of the value that was just written to a
// register
void update_cond_flags(uint16_t reg) {
    if (registers[reg] == 0) {
        registers[R_COND] = FL_ZRO;
    } else if (registers[reg] >> 15) {
        // leftmost bit = 1 means value is negative
        registers[R_COND] = FL_NEG;
    } else {
        registers[R_COND] = FL_POS;
    }
}

int main(int argc, const char *argv[]) {
    // set initial program counter address
    enum { PC_START = 0x3000 };
    registers[R_PC] = PC_START;
    
    int running = 1;
    while (running) {
        // fetch instruction from memory and update PC
        uint16_t instr = mem_read(registers[R_PC]++);
        uint16_t op = instr >> 12;

        switch(op) {
            case OP_ADD:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t src1 = (instr >> 6) & 0x7;

                // determine instruction mode: register or immediate
                uint16_t mode_flag = (instr >> 5) & 0x1;
                if (mode_flag) {
                    // immediate mode: 2nd operand to add supplied in
                    // instruction (limited to 5 bit numbers)
                    uint16_t imm5 = sign_extend(instr & 0x1F, 5);
                    registers[dest] = registers[src1] + imm5;
                } else {
                    // register mode: 2nd operand is another source
                    // register
                    uint16_t src2 = instr & 0x7;
                    registers[dest] = registers[src1] + registers[src2];
                }

                update_cond_flags(dest);
                break;
            case OP_AND:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t src1 = (instr >> 6) & 0x7;

                uint16_t mode_flag = (instr >> 5) & 0x1;
                if (mode_flag) {
                    uint16_t imm5 = sign_extend(instr & 0x1F, 5);
                    registers[dest] = registers[src1] & imm5;
                } else {
                    uint16_t src2 = instr & 0x7;
                    registers[dest] = registers[src1] & registers[src2];
                }

                update_cond_flags(dest);
                break;
            case OP_NOT:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t src = (instr >> 6) & 0x7;
                registers[dest] = ~registers[src];
                update_cond_flags(dest);
                break;
            case OP_BR:
                // check whether any of the bits corresponding to
                // a condition flag is set, updating pc if so
                uint16_t n = (instr >> 11) & 0x1;
                uint16_t z = (instr >> 10) & 0x1;
                uint16_t p = (instr >> 9) & 0x1;
                if ((n && registers[R_COND] == FL_NEG) ||
                    (p && registers[R_COND] == FL_POS) ||
                    (z && registers[R_COND] == FL_ZRO)) {
                    registers[R_PC] += sign_extend(instr & 0x1FF, 9);
                }
                break;
            case OP_JMP:
                uint16_t base_reg = (instr >> 6) & 0x7;
                registers[R_PC] = registers[base_reg];
                break;
            case OP_JSR:
                // store current pc address (location of current
                // routine) in register 7 and then jump to address
                // of subroutine
                registers[R_R7] = registers[R_PC];
                uint16_t mode_flag = (instr >> 11) & 0x1;
                if (mode_flag) {
                    registers[R_PC] += sign_extend(instr & 0x7FF, 11);
                } else {
                    uint16_t base_reg = (instr >> 6) & 0x7;
                    registers[R_PC] = registers[base_reg];
                }
                break;
            case OP_LD:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t addr = registers[R_PC] + sign_extend(instr & 0x1FF, 9);
                registers[dest] = memory[addr];
                update_cond_flags(dest);
                break;
            case OP_LDI:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x7FF, 9);
                
                uint16_t addr = registers[R_PC] + pc_offset;
                reg[dest] = mem_read(mem_read(addr));
                update_cond_flags(dest);
                break;
            case OP_LDR:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t base_reg = (instr >> 6) & 0x7;
                uint16_t addr = registers[base_reg] + sign_extend(instr & 0x3F, 6);
                registers[dest] = memory[addr];
                update_cond_flags(dest);
                break;
            case OP_LEA:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t addr = registers[R_PC] + sign_extend(instr & 0x1FF, 9);
                registers[dest] = addr;
                update_cond_flags(dest);
                break;
            case OP_ST:
                uint16_t src = (instr >> 9) & 0x7;
                uint16_t addr = registers[R_PC] + sign_extend(instr & 0x1FF, 9);
                memory[addr] = registers[src];
                break;
            case OP_STI:
                uint16_t src = (instr >> 9) & 0x7;
                uint16_t addr = memory[registers[R_PC] + sign_extend(instr & 0x1FF, 9)];
                memory[addr] = registers[src];
                break;
            case OP_STR:
                uint16_t src = (instr >> 9) & 0x7;
                uint16_t base_reg = (instr >> 6) & 0x7;
                uint16_t addr = registers[base_reg] + sign_extend(instr & 0x3F, 6);
                memory[addr] = registers[src];
                break;
            case OP_TRAP:
                registers[R_R7] = registers[R_PC];
                
                // condition on last 8 bits, trap code, of instruction
                switch(instr & 0xFF) {
                    case TRAP_GETC:
                        registers[R_R0] = (uint16_t)getchar();
                        break;
                    case TRAP_OUT:
                        break;
                    case TRAP_PUTS:
                        // first char of string located at address stored in R0
                        // each char occupies 16 bits
                        uint16_t *c = memory + registers[R_R0];
                        while (*c) {
                            putc((char)*c, stdout);
                            ++c;
                        }
                        // write all buffered data for stdout
                        fflush(stdout);
                        break;
                    case TRAP_IN:
                        printf("> ");
                        char user_input = getchar();
                        putc(user_input, stdout);
                        registers[R_R0] = (uint16_t)user_input;
                        break;
                    case TRAP_PUTSP:
                        break;
                    case TRAP_HALT:
                        break;
                }
                break;
            case OP_RES:
            case OP_RTI:
            default:
                abort();
                break;
        }
    }
}
