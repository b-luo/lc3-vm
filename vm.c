/* VM simulates LC-3 computer */

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
                break;
            case OP_AND:
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
            default:
                break;
        }
    }
}
