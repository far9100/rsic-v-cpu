# add_test.s
# Target: Test basic addition (ADD), immediate addition (ADDI),
# and store word (SW) instructions.
# Expected: 10 + 20 = 30. The result 30 (0x1E) will be stored
# to memory address 0x100.

.global _start

_start:
    # Load the first operand into x5 (t0)
    addi x5, x0, 10      # x5 = 0 + 10 (x5 = 10)

    # Load the second operand into x6 (t1)
    addi x6, x0, 20      # x6 = 0 + 20 (x6 = 20)

    # Perform addition: x7 = x5 + x6
    add  x7, x5, x6      # x7 = 10 + 20 (x7 = 30)

    # Prepare to store the result to memory address 0x100
    # The testbench's data_mem is accessed starting from word address 64
    # (which is byte address 0x100).
    addi x8, x0, 0x100   # x8 = 0x100 (base memory address)

    # Store the content of x7 (result 30) to memory address 0x100
    # sw rs2, offset(rs1)  => sw x7, 0(x8)
    sw   x7, 0(x8)       # Mem[0x100] = x7 (value 30)

    # Program end: enter an infinite loop.
    # Your testbench waits for PC to reach 0x20.
    # If this program is shorter, you might need to adjust
    # the testbench's halt condition or pad with NOPs here.
    # For now, a simple infinite loop.
    # Assuming this program is loaded at address 0x0.
    # Instruction 1: addi x5, x0, 10   (addr 0x00)
    # Instruction 2: addi x6, x0, 20   (addr 0x04)
    # Instruction 3: add  x7, x5, x6   (addr 0x08)
    # Instruction 4: addi x8, x0, 0x100 (addr 0x0C)
    # Instruction 5: sw   x7, 0(x8)    (addr 0x10)
    # Instruction 6: jal x0, halt_loop (addr 0x14)
    # To make halt_loop at 0x20, we need (0x20 - 0x14) / 4 = 3 NOPs
    
    nop                      # addr 0x14 (NOP is addi x0, x0, 0)
    nop                      # addr 0x18
    nop                      # addr 0x1C

halt_loop:                   # addr 0x20
    jal x0, halt_loop   # Jump to self, creating an infinite loop
