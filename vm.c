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

// memory mapped registers: access through the address assigned
// to each
enum {
    MR_KBSR = 0xFE00,   // keyboard status: indicate if a key has been pressed
    MR_KBDR = 0xFE02    // keyboard data: record key pressed
};

uint16_t check_key() {
    fd_set readfds;     // fixed size buffer holding file descriptor set
    FD_ZERO(&readfds);  // initialize readfds to be empty set
    FD_SET(STDIN_FILENO, &readfds); // add file descriptor to set

    // timeout: min. interval select should block waiting for file descriptor
    // to become ready
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    // select: monitor file descriptors to see if they are ready for 
    // returns num. of file descriptors
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

// adding memory mapped registers requires following functions to
// access memory
uint16_t mem_read(uint16_t addr) {
    if (addr == MR_KBSR) {
        if (check_key()) {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        } else {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[addr];
}

void mem_write(uint16_t addr, uint16_t val) {
    memory[addr] = val;
}

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

// swap order of bytes in 16 bit integer, which
// changes endianness
uint16_t swap16(uint16_t x) {
    return (x << 8) | (x >> 8);
}

void read_image_file(FILE *file) {
    // origin: 1st 16 bits indicate address to load program into
    uint16_t origin;
    // read 1 element from file, storing the result in origin
    fread(&origin, sizeof(origin), 1, file);
    // swap order of bytes since LC-3 programs are big endian
    // while most computers are little endian
    origin = swap16(origin);

    uint16_t max_file_size = UINT16_MAX - origin;
    uint16_t *p = memory + origin;
    size_t num_addr_read = fread(p, sizeof(uint16_t), max_file_size, file);
    // change endianness of remaining addresses
    while (num_addr_read-- > 0) {
        *p = swap16(*p);
        ++p;
    }
}

int read_image(const char *image_path) {
    FILE *file = fopen(image_path, "rb");
    if (!file) return 0;
    read_image_file(file);
    fclose(file);
    return 1;
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
                registers[dest] = mem_read(addr);
                update_cond_flags(dest);
                break;
            case OP_LDI:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t pc_offset = sign_extend(instr & 0x7FF, 9);
                
                uint16_t addr = registers[R_PC] + pc_offset;
                registers[dest] = mem_read(mem_read(addr));
                update_cond_flags(dest);
                break;
            case OP_LDR:
                uint16_t dest = (instr >> 9) & 0x7;
                uint16_t base_reg = (instr >> 6) & 0x7;
                uint16_t addr = registers[base_reg] + sign_extend(instr & 0x3F, 6);
                registers[dest] = mem_read(addr);
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
                mem_write(addr, registers[src]);
                break;
            case OP_STI:
                uint16_t src = (instr >> 9) & 0x7;
                uint16_t addr = memory[registers[R_PC] + sign_extend(instr & 0x1FF, 9)];
                mem_write(addr, registers[src]);
                break;
            case OP_STR:
                uint16_t src = (instr >> 9) & 0x7;
                uint16_t base_reg = (instr >> 6) & 0x7;
                uint16_t addr = registers[base_reg] + sign_extend(instr & 0x3F, 6);
                mem_write(addr, registers[src]);
                break;
            case OP_TRAP:
                registers[R_R7] = registers[R_PC];
                
                // condition on last 8 bits, trap code, of instruction
                switch(instr & 0xFF) {
                    case TRAP_GETC:
                        registers[R_R0] = (uint16_t)getchar();
                        break;
                    case TRAP_OUT:
                        putc((char)(registers[R_R0] & 0xFF), stdout);
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
                        // each char occupies 1 byte, so each memory
                        // location holds 2 chars - output char stored
                        // in rightmost portion of each address before
                        // char in leftmost
                        uint16_t *c = memory + registers[R_R0];
                        while (*c) {
                            uint16_t first = (*c) & 0xFF;
                            uint16_t second = (*c) >> 8;
                            putc((char)first, stdout);
                            // if string has odd number of chars, leftmost
                            // byte of some address will hold 0x00
                            if (second) putc((char)second, stdout);
                            ++c;
                        }
                        fflush(stdout);
                        break;
                    case TRAP_HALT:
                        printf("Halting program..");
                        running = 0;
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
