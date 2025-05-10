// Memory Access (MEM) Stage
module mem_stage (
    input  logic        clk,          // Clock
    input  logic        rst,          // Reset


    
    // Inputs from EX Stage (or EX/MEM pipeline register)
    input  logic [31:0] alu_result_i, // ALU result (used as memory address)
    input  logic [31:0] rs2_data_i,   // Data from rs2 (data to be stored)
    input  logic [4:0]  rd_addr_i,    // Destination register address
    input  logic [31:0] pc_plus4_i,   // PC + 4 from EX stage
    // Control signals from EX stage (placeholders for now)
    input  logic        mem_read_i,   // Memory read enable
    input  logic        mem_write_i,  // Memory write enable
    input  logic [1:0]  mem_op_size_i,// Memory operation size (byte, half, word) - Placeholder

    // Data Memory Interface Inputs/Outputs
    output logic [31:0] data_addr_o,  // Data memory address
    output logic [31:0] data_wdata_o, // Data to write to memory
    output logic [3:0]  data_we_o,    // Data write enable (byte level)
    input  logic [31:0] data_rdata_i, // Data read from memory

    // Outputs to WB Stage (or MEM/WB pipeline register)
    output logic [31:0] mem_rdata_o,  // Data read from memory (potentially processed)
    output logic [31:0] alu_result_o, // Pass through ALU result
    output logic [4:0]  rd_addr_o,    // Pass through destination register address
    output logic [31:0] pc_plus4_o    // Pass through PC+4
    // Pass through control signals for WB stage
);

    // Memory Address and Write Data
    assign data_addr_o = alu_result_i; // ALU result is used as the memory address
    assign data_wdata_o = rs2_data_i;  // rs2 data is the data to write for stores

    // Memory Write Enable (Byte-level) - Simplified Placeholder
    // This needs to be generated based on mem_write_i, mem_op_size_i, and lower bits of address
    assign data_we_o = mem_write_i ? 4'b1111 : 4'b0000; // Placeholder: Write full word if mem_write is asserted

    // Memory Read Data Processing - Simplified Placeholder
    // This needs to handle sign/zero extension based on mem_op_size_i for loads (LB, LH, LW, LBU, LHU)
    assign mem_rdata_o = data_rdata_i; // Placeholder: Pass through raw read data

    // Pass through other signals
    assign alu_result_o = alu_result_i;
    assign rd_addr_o = rd_addr_i;
    assign pc_plus4_o = pc_plus4_i;

    // Pass through control signals (to be added)

endmodule
