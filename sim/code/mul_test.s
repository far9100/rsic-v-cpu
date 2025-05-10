# mul_test.s
# Software multiplication test: 5 * 3 = 15 (0xF)
# Result will be stored at memory address 0x104 (word address 65)

.global _start

_start:
    # Initialize operands and result
    addi x5, x0, 5     # x5 (multiplicand) = 5
    addi x6, x0, 3     # x6 (multiplier, also loop counter) = 3
    addi x7, x0, 0     # x7 (result) = 0
    addi x10, x0, 0    # x10 (constant zero) for comparison

loop:
    # Check if counter (x6) is zero. If so, branch to end_loop.
    # beq rs1, rs2, offset
    # beq x6, x10, end_loop
    beq x6, x10, end_loop  # If x6 == x10 (i.e., x6 == 0), jump to end_loop

    # Accumulate result
    add x7, x7, x5     # result = result + multiplicand (x7 = x7 + x5)

    # Decrement counter
    addi x6, x6, -1    # counter = counter - 1 (x6 = x6 - 1)

    # Jump back to the beginning of the loop
    # jal rd, offset
    # jal x0, loop (rd=x0 means don't save return address)
    jal x0, loop

end_loop:
    # Store the final result (x7) into memory address 0x104
    # Word address 65 corresponds to byte address 0x104 (65 * 4)
    addi x8, x0, 0x104 # x8 = address 0x104
    sw x7, 0(x8)       # Mem[0x104] = x7 (should be 15)

    # Program halt loop
    # Ensure this address is what the testbench expects for program completion,
    # or update the testbench's halt condition.
    # For this test, let's make it distinct from the add_test halt address.
    # _start: 0x00
    # addi x5: 0x00
    # addi x6: 0x04
    # addi x7: 0x08
    # addi x10:0x0C
    # loop:   (target of beq and jal)
    # beq:    0x10
    # add:    0x14
    # addi x6:0x18
    # jal loop:0x1C
    # end_loop: (target of beq)
    # addi x8:0x20
    # sw:     0x24
    # halt_loop:0x28
halt_loop:
    jal x0, halt_loop  # Infinite loop
