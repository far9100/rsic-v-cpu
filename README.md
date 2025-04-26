# RISC-V CPU Implementation in Verilog

## Description

This project implements a basic RISC-V CPU core using the Verilog Hardware Description Language. It aims to provide a functional processor based on the RISC-V instruction set architecture (ISA).

## Architecture

The CPU utilizes a classic **five-stage pipeline** architecture to improve instruction throughput:

1.  **IF (Instruction Fetch):** Fetches the next instruction from instruction memory based on the Program Counter (PC).
    *   Module: `src/if_stage.v`
2.  **ID (Instruction Decode):** Decodes the fetched instruction, reads required operands from the register file, and generates control signals.
    *   Module: `src/id_stage.v`
3.  **EX (Execute):** Performs the main computation, typically involving the Arithmetic Logic Unit (ALU), and calculates branch/jump target addresses.
    *   Module: `src/ex_stage.v`
    *   ALU: `src/alu.v`
4.  **MEM (Memory Access):** Handles load and store instructions by accessing the data memory. Results from other instructions pass through this stage.
    *   Module: `src/mem_stage.v`
5.  **WB (Write Back):** Writes the final result (either from the ALU or data memory) back into the register file.
    *   Module: `src/wb_stage.v`

## Key Modules

*   `src/riscv_cpu.v`: The top-level module that integrates all pipeline stages and the control unit.
*   `src/control_unit.v`: Generates control signals based on the instruction's opcode and function fields.
*   `src/if_stage.v`: Instruction Fetch stage logic.
*   `src/id_stage.v`: Instruction Decode stage logic, including the register file access.
*   `src/ex_stage.v`: Execute stage logic, including ALU instantiation and branch/jump calculations.
*   `src/alu.v`: Arithmetic Logic Unit performing calculations like addition, subtraction, logical operations, etc.
*   `src/mem_stage.v`: Memory Access stage logic for load/store operations.
*   `src/wb_stage.v`: Write Back stage logic for updating the register file.

## Features

*   Implements a subset of the RV32I base integer instruction set.
*   Pipelined execution for improved performance.
*   Handles basic arithmetic, logical, load, store, branch, and jump instructions.

## Status

The project is currently under development. Some features or optimizations (like full hazard detection/forwarding or complete control signal pipelining) might still be in progress.

## Simulation and Building

*(Instructions on how to simulate the design using tools like ModelSim, Verilator, or Vivado, and how to synthesize it for an FPGA would go here.)*
