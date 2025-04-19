// Write Back (WB) Stage
module wb_stage (
    input  logic        clk,          // Clock
    input  logic        rst,          // Reset

    // Inputs from MEM Stage (or MEM/WB pipeline register)
    input  logic [31:0] mem_rdata_i,  // Data read from memory
    input  logic [31:0] alu_result_i, // Result from ALU
    input  logic [4:0]  rd_addr_i,    // Destination register address
    input  logic [31:0] pc_plus4_i,   // PC + 4 from MEM stage (for JAL/JALR link address)
    // Control signals from MEM stage (placeholders for now)
    input  logic        reg_we_i,     // Register write enable signal
    input  logic [1:0]  wb_mux_sel_i, // Write back data select (00: ALU, 01: Mem, 10: PC+4) - Placeholder

    // Outputs to ID Stage (for register file write)
    output logic        wb_reg_we_o,  // Register write enable
    output logic [4:0]  wb_rd_addr_o, // Destination register address
    output logic [31:0] wb_rd_data_o  // Data to write back
);

    logic [31:0] write_data;

    // Select data to write back to the register file
    // Selects between ALU result, Memory read data, or PC+4 (for JAL/JALR link address)
    always_comb begin
        case (wb_mux_sel_i)
            2'b00:  write_data = alu_result_i; // Select ALU result
            2'b01:  write_data = mem_rdata_i;  // Select Memory data
            2'b10:  write_data = pc_plus4_i;   // Select PC+4
            default: write_data = alu_result_i; // Default to ALU result (or handle as error)
        endcase
    end

    // Output Assignments
    assign wb_reg_we_o  = reg_we_i;     // Pass through write enable signal
    assign wb_rd_addr_o = rd_addr_i;    // Pass through destination register address
    assign wb_rd_data_o = write_data;   // Selected data to write back

endmodule
