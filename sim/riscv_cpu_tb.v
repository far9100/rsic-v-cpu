`timescale 1ns / 1ps

module riscv_cpu_tb;

    // Parameters
    parameter CLK_PERIOD = 10; // Clock period in ns (100 MHz)
    parameter INSTR_MEM_SIZE = 1024; // Instruction memory size in words (4KB)
    parameter DATA_MEM_SIZE = 1024; // Data memory size in words (4KB)
    parameter DATA_MEM_ADDR_WIDTH = $clog2(DATA_MEM_SIZE);
    parameter INSTR_MEM_ADDR_WIDTH = $clog2(INSTR_MEM_SIZE);

    // Signals
    logic clk;
    logic rst;

    // Instruction Memory Interface
    logic [31:0] instr_addr;
    logic [31:0] instr_data;

    // Data Memory Interface
    logic [31:0] data_addr;
    logic [31:0] data_wdata;
    logic [3:0]  data_we;
    logic [31:0] data_rdata;

    // Instantiate the CPU (DUT - Design Under Test)
    riscv_cpu dut (
        .clk(clk),
        .rst(rst),

        // Instruction Memory Interface
        .instr_addr_o(instr_addr),
        .instr_data_i(instr_data),

        // Data Memory Interface
        .data_addr_o(data_addr),
        .data_wdata_o(data_wdata),
        .data_we_o(data_we),
        .data_rdata_i(data_rdata)
    );

    // --- Clock Generation ---
    initial begin
        clk = 0;
        forever #(CLK_PERIOD / 2) clk = ~clk;
    end

    // --- Reset Generation ---
    initial begin
        rst = 1;
        #(CLK_PERIOD * 5); // Assert reset for 5 clock cycles
        rst = 0;
        $display("Reset released at time %0t", $time);
    end

    // --- Simple Memory Models ---

    // Instruction Memory (ROM)
    logic [31:0] instr_mem [0:INSTR_MEM_SIZE-1];

    // Read from Instruction Memory (combinational read)
    // Assuming word-aligned access for simplicity
    assign instr_data = instr_mem[instr_addr >> 2]; // Divide by 4 for word address

    // Data Memory (RAM)
    logic [31:0] data_mem [0:DATA_MEM_SIZE-1];

    // Read from Data Memory (combinational read)
    // Assuming word-aligned access for simplicity
    assign data_rdata = data_mem[data_addr >> 2]; // Divide by 4 for word address

    // Write to Data Memory (synchronous write)
    always_ff @(posedge clk) begin
        if (!rst) begin // Only write when not in reset
            // Byte-level write enable handling
            if (data_we[0]) data_mem[data_addr >> 2][7:0]   <= data_wdata[7:0];
            if (data_we[1]) data_mem[data_addr >> 2][15:8]  <= data_wdata[15:8];
            if (data_we[2]) data_mem[data_addr >> 2][23:16] <= data_wdata[23:16];
            if (data_we[3]) data_mem[data_addr >> 2][31:24] <= data_wdata[31:24];
        end
    end

    // --- Simulation Control ---
    initial begin
        integer file;
        integer c;
        integer status;
        integer instr_index;
        logic [31:0] expected_result; // Moved declaration
        logic [31:0] actual_result;   // Moved declaration
        
        // Initialize memories to a default value (e.g., NOP for instruction, 0 for data)
        for (int i = 0; i < INSTR_MEM_SIZE; i++) begin
            instr_mem[i] = 32'h00000013; // Default to NOP
        end
        for (int i = 0; i < DATA_MEM_SIZE; i++) begin
            data_mem[i] = 32'h00000000;
        end

        // Load program and data from hex files
        $display("Loading instruction memory from sim/code/mul_test.hex"); // Changed to mul_test.hex
        $readmemh("sim/code/mul_test.hex", instr_mem);
        
        // Display some instructions to verify loading
        $display("Instruction at address 0x00: %h", instr_mem[0]); // ADDI x5, x0, 5
        $display("Instruction at address 0x04: %h", instr_mem[1]); // ADDI x6, x0, 3
        $display("Instruction at address 0x10 (loop): %h", instr_mem[4]); // BEQ x6, x10, end_loop
        $display("Instruction at address 0x20 (end_loop): %h", instr_mem[8]); // ADDI x8, x0, 0x104
        $display("Instruction at address 0x28 (halt_loop): %h", instr_mem[10]); // JAL x0, halt_loop
        
        // $display("Loading data memory from sim/code/data.hex"); // Temporarily commented out to avoid error if data.hex is missing
        // $readmemh("sim/code/data.hex", data_mem, 64); 
        
        // Display initial data array
        $display("Initial data array (before sorting):");
        dump_data_memory(64, 10);

        $display("Starting Simulation...");

        // Wait for reset to deassert
        wait (rst == 0);

        // Run simulation until the program counter reaches the halt address (0x28 for mul_test.s)
        // Or a timeout occurs
        fork
            begin : timeout_block
                #(CLK_PERIOD * 1_000_000); // Timeout after 1,000,000 cycles
                $display("Simulation timed out! PC did not reach halt address 0x28.");
                $display("Current PC = 0x%h", dut.instr_addr_o);
                dump_data_memory(64, 10); // Dump data memory on timeout
                $finish;
            end
            begin : run_block
                wait (dut.instr_addr_o == 32'h00000028); // Wait until PC reaches the halt loop address for mul_test
                #(CLK_PERIOD * 2); // Wait a couple more cycles for pipeline to settle
                $display("PC reached halt address 0x28 at time %0t", $time);
                disable timeout_block; // Disable timeout if halt is reached
            end
        join

        // --- Simplified Verification ---
        if (dut.instr_addr_o == 32'h00000028) begin // Check for new halt address
            $display("Verification PASSED: PC reached the halt loop at 0x28.");
        end else begin
            $error("Verification FAILED: PC did not reach the halt loop (PC = 0x%h).", dut.instr_addr_o);
        end

        // Add check for multiplication result
        // Declarations moved to the top of the initial block

        expected_result = 32'd15; // 5 * 3
        actual_result = data_mem[65]; // Word address 65 (0x104)

        if (actual_result == expected_result) begin
            $display("Multiplication Result PASSED: Mem[0x104] = 0x%h (Expected: 0x%h)", actual_result, expected_result);
        end else begin
            $error("Multiplication Result FAILED: Mem[0x104] = 0x%h (Expected: 0x%h)", actual_result, expected_result);
        end

        // Dump final data memory state (first 10 elements of the array)
        $display("Dumping data memory (word address 64 for 10 words):");
        dump_data_memory(64, 10); // Word address 64 is 0x40

        $display("Simulation finished at time %0t", $time);
        $finish; // End simulation
    end

    // Task to dump a portion of data memory
    task dump_data_memory (input [31:0] start_word_addr, input [31:0] num_words); // Use integer for compatibility
        $display("--- Data Memory Dump (Word Addr: %0d to %0d) ---", start_word_addr, start_word_addr + num_words - 1);
        for (integer i = 0; i < num_words; i++) begin // Use integer for compatibility
            $display("Mem[0x%h (Word %0d)]: 0x%h (%0d)", (start_word_addr + i) * 4, start_word_addr + i, data_mem[start_word_addr + i], data_mem[start_word_addr + i]);
        end
        $display("--------------------------------------------------");
    endtask

    // --- Monitoring (Optional) ---
    // Example: Monitor register file writes (requires access inside the CPU or specific outputs)
    // initial begin
    //     forever @(posedge clk) begin
    //         if (dut.wb_stage_inst.wb_reg_we) begin // Accessing internal signal - requires modification or specific output
    //             $display("Time %0t: Reg Write: Addr=%d, Data=0x%h", $time, dut.wb_stage_inst.wb_rd_addr, dut.wb_stage_inst.wb_rd_data);
    //         end
    //     end
    // end

endmodule
