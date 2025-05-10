// Top-level module for the RISC-V CPU
module riscv_cpu (
    input clk,          // Clock signal
    input rst,          // Reset signal

    // Instruction Memory Interface
    output logic [31:0] instr_addr_o, // Instruction address
    input  logic [31:0] instr_data_i, // Instruction data

    // Data Memory Interface
    output logic [31:0] data_addr_o,  // Data memory address
    output logic [31:0] data_wdata_o, // Data to write
    output logic [3:0]  data_we_o,    // Data write enable (byte level)
    input  logic [31:0] data_rdata_i  // Data read from memory
);

    // --- Signals connecting stages ---

    // IF Stage Outputs -> ID Stage Inputs
    logic [31:0] if_pc;
    logic [31:0] if_pc_plus4;

    // ID Stage Outputs -> EX Stage Inputs
    logic [31:0] id_rs1_data;
    logic [31:0] id_rs2_data;
    logic [31:0] id_imm;
    logic [4:0]  id_rd_addr;
    logic [6:0]  id_opcode;
    logic [2:0]  id_funct3;
    logic [6:0]  id_funct7;
    logic [31:0] id_pc;          // PC value from ID stage (PC of instruction in ID)
    logic [31:0] id_pc_plus4;    // PC+4 of instruction in ID

    // Control Unit Signals (generated based on instruction in ID)
    logic        ctrl_alu_src_a_sel;
    logic [1:0]  ctrl_alu_src_b_sel;
    logic [3:0]  ctrl_alu_op;
    logic        ctrl_mem_read;
    logic        ctrl_mem_write;
    // logic [1:0] ctrl_mem_op_size; // Add when control unit outputs it
    logic        ctrl_reg_we;
    logic [1:0]  ctrl_wb_mux_sel;
    logic        ctrl_branch; // Control signal: Is branch?
    logic        ctrl_jump;   // Control signal: Is jump?

    // EX Stage Outputs -> MEM Stage Inputs / IF Stage Inputs
    logic [31:0] ex_alu_result;
    logic        ex_alu_zero;
    logic [31:0] ex_rs2_data;
    logic [4:0]  ex_rd_addr;
    logic [31:0] ex_pc_plus4;
    logic [31:0] ex_branch_target_addr; // Calculated branch target from EX
    logic [31:0] ex_jalr_target_addr;   // Calculated JALR target from EX
    logic        ex_branch_condition_met; // Branch condition result from EX
    // EX Control Signals (Outputs of ID/EX Register, Inputs to EX Stage for control)
    // These are the control signals that travel with the instruction from ID to EX
    logic        id_ex_alu_src_a_sel_reg;
    logic [1:0]  id_ex_alu_src_b_sel_reg;
    logic [3:0]  id_ex_alu_op_reg;
    logic        id_ex_mem_read_reg;
    logic        id_ex_mem_write_reg;
    logic        id_ex_reg_we_reg;
    logic [1:0]  id_ex_wb_mux_sel_reg;
    logic        id_ex_branch_reg; // Added for pipelining branch control signal
    // Data path signals for ID/EX register
    logic [31:0] id_ex_pc_reg;
    logic [31:0] id_ex_pc_plus4_reg;
    logic [31:0] id_ex_rs1_data_reg;
    logic [31:0] id_ex_rs2_data_reg;
    logic [31:0] id_ex_imm_reg;
    logic [4:0]  id_ex_rd_addr_reg;
    logic [2:0]  id_ex_funct3_reg; // For branch condition in EX

    // MEM Stage Inputs (Outputs of EX/MEM Register)
    // Control signals for MEM stage
    logic        ex_mem_mem_read_reg;
    logic        ex_mem_mem_write_reg;
    logic        ex_mem_reg_we_reg;
    logic [1:0]  ex_mem_wb_mux_sel_reg;
    // Data path signals for EX/MEM register
    logic [31:0] ex_mem_alu_result_reg;
    logic [31:0] ex_mem_rs2_data_reg; // For SW
    logic [4:0]  ex_mem_rd_addr_reg;
    logic [31:0] ex_mem_pc_plus4_reg; // For JAL/JALR link address

    // MEM Stage Outputs -> MEM/WB Register Inputs
    logic [31:0] mem_rdata;
    logic [31:0] mem_alu_result;
    logic [4:0]  mem_rd_addr;
    logic [31:0] mem_pc_plus4;

    // WB Stage Inputs (Outputs of MEM/WB Register)
    // Control signals for WB stage
    logic        mem_wb_reg_we_reg;
    logic [1:0]  mem_wb_wb_mux_sel_reg;
    // Data path signals for MEM/WB register
    logic [31:0] mem_wb_mem_rdata_reg;
    logic [31:0] mem_wb_alu_result_reg;
    logic [4:0]  mem_wb_rd_addr_reg;
    logic [31:0] mem_wb_pc_plus4_reg;

    // WB Stage Outputs -> ID Stage Inputs (Register File Write)
    logic        wb_reg_we;
    logic [4:0]  wb_rd_addr;
    logic [31:0] wb_rd_data;

    // --- Stage Instantiations ---

    // --- PC Update Logic Signals ---
    logic [31:0] jal_target_addr; // Calculated JAL target address (PC + J-imm)
    logic        is_jalr;         // Flag indicating if the current instruction is JALR

    // --- Stage Instantiations ---

    // IF Stage
    if_stage if_stage_inst (
        .clk                    (clk),
        .rst                    (rst),
        .jump_i                 (ctrl_jump),                 // From Control Unit
        .branch_i               (ctrl_branch),               // From Control Unit
        .branch_condition_met_i (ex_branch_condition_met), // From EX Stage
        .jal_target_addr_i      (jal_target_addr),         // Calculated below
        .jalr_target_addr_i     (ex_jalr_target_addr),     // From EX Stage
        .branch_target_addr_i   (ex_branch_target_addr),   // From EX Stage
        .is_jalr_i              (is_jalr),                 // Determined below
        .pc_o                   (if_pc),
        .pc_plus4_o             (if_pc_plus4)
    );

    // ID Stage (includes Register File)
    id_stage id_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .pc_i           (if_pc),        // Pass current PC to ID stage
        .instr_i        (instr_data_i), // Instruction comes from memory
        .pc_plus4_i     (if_pc_plus4),  // PC+4 from IF
        .wb_reg_we_i    (wb_reg_we),    // Write enable from WB
        .wb_rd_addr_i   (wb_rd_addr),   // Write address from WB
        .wb_rd_data_i   (wb_rd_data),   // Write data from WB
        .rs1_data_o     (id_rs1_data),
        .rs2_data_o     (id_rs2_data),
        .imm_o          (id_imm),
        .rd_addr_o      (id_rd_addr),
        .opcode_o       (id_opcode),
        .funct3_o       (id_funct3),
        .funct7_o       (id_funct7),
        .pc_o           (id_pc),        // Output PC from ID stage
        .pc_plus4_o     (id_pc_plus4)
        // Control signal outputs need to be added and connected
        // Note: Control unit is placed after ID stage as it depends on opcode from ID
    );

    // Control Unit
    control_unit control_unit_inst (
        .opcode_i       (id_opcode),
        .funct3_i       (id_funct3),     // Connect funct3 from ID stage
        .funct7_b5_i    (id_funct7[5]),  // Connect funct7[5] from ID stage
        .alu_src_a_sel_o(ctrl_alu_src_a_sel),
        .alu_src_b_sel_o(ctrl_alu_src_b_sel),
        .alu_op_o       (ctrl_alu_op),
        .mem_read_o     (ctrl_mem_read),
        .mem_write_o    (ctrl_mem_write),
        // .mem_op_size_o(ctrl_mem_op_size),
        .reg_we_o       (ctrl_reg_we),
        .wb_mux_sel_o   (ctrl_wb_mux_sel),
        .branch_o       (ctrl_branch), // Connect branch signal output
        .jump_o         (ctrl_jump)    // Connect jump signal output
    );

    // Calculate JAL target address and is_jalr flag
    // JAL target = PC_of_JAL_instruction + J-immediate
    // PC_of_JAL_instruction is id_pc (output from id_stage, which is the pc_i input to id_stage)
    // id_imm is the sign-extended byte offset for JAL from id_stage
    assign jal_target_addr = id_pc + id_imm; // Corrected: Use id_pc (PC of instruction in ID)
    assign is_jalr = (id_opcode == 7'b1100111); // Check if opcode is JALR

    // --- Pipeline Registers ---

    // ID/EX Pipeline Register
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            id_ex_pc_reg            <= 32'b0;
            id_ex_pc_plus4_reg      <= 32'b0;
            id_ex_rs1_data_reg      <= 32'b0;
            id_ex_rs2_data_reg      <= 32'b0;
            id_ex_imm_reg           <= 32'b0;
            id_ex_rd_addr_reg       <= 5'b0;
            id_ex_funct3_reg        <= 3'b0;
            // Control signals reset to NOP-like state
            id_ex_alu_src_a_sel_reg <= 1'b1; // rs1
            id_ex_alu_src_b_sel_reg <= 2'b00; // rs2
            id_ex_alu_op_reg        <= 4'b0000; // ADD (NOP if rd=x0)
            id_ex_mem_read_reg      <= 1'b0;
            id_ex_mem_write_reg     <= 1'b0;
            id_ex_reg_we_reg        <= 1'b0; // No register write
            id_ex_wb_mux_sel_reg    <= 2'b00; // ALU result
            id_ex_branch_reg        <= 1'b0; // Default branch to false
        end else begin // Changed { to begin
            // Latch values from ID stage and Control Unit
            id_ex_pc_reg            <= id_pc;
            id_ex_pc_plus4_reg      <= id_pc_plus4;
            id_ex_rs1_data_reg      <= id_rs1_data;
            id_ex_rs2_data_reg      <= id_rs2_data;
            id_ex_imm_reg           <= id_imm;
            id_ex_rd_addr_reg       <= id_rd_addr;
            id_ex_funct3_reg        <= id_funct3;
            // Latch control signals
            id_ex_alu_src_a_sel_reg <= ctrl_alu_src_a_sel;
            id_ex_alu_src_b_sel_reg <= ctrl_alu_src_b_sel;
            id_ex_alu_op_reg        <= ctrl_alu_op;
            id_ex_mem_read_reg      <= ctrl_mem_read;
            id_ex_mem_write_reg     <= ctrl_mem_write;
            id_ex_reg_we_reg        <= ctrl_reg_we;
            id_ex_wb_mux_sel_reg    <= ctrl_wb_mux_sel;
            id_ex_branch_reg        <= ctrl_branch; // Latch branch signal
        end
    end

    // EX Stage (includes ALU)
    ex_stage ex_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .pc_i           (id_ex_pc_reg),        // From ID/EX Reg
        .rs1_data_i     (id_ex_rs1_data_reg),  // From ID/EX Reg
        .rs2_data_i     (id_ex_rs2_data_reg),  // From ID/EX Reg
        .imm_i          (id_ex_imm_reg),       // From ID/EX Reg
        .rd_addr_i      (id_ex_rd_addr_reg),   // From ID/EX Reg
        .pc_plus4_i     (id_ex_pc_plus4_reg),  // From ID/EX Reg
        .alu_src_a_sel_i(id_ex_alu_src_a_sel_reg), // From ID/EX Reg
        .alu_src_b_sel_i(id_ex_alu_src_b_sel_reg), // From ID/EX Reg
        .alu_op_i       (id_ex_alu_op_reg),       // From ID/EX Reg
        .funct3_i       (id_ex_funct3_reg),       // From ID/EX Reg (for branch)
        .branch_i       (id_ex_branch_reg),       // Corrected: Use pipelined branch signal from ID/EX Reg
        .alu_result_o   (ex_alu_result),
        .alu_zero_o     (ex_alu_zero),
        .rs2_data_o     (ex_rs2_data),         // Data to forward to MEM for SW
        .rd_addr_o      (ex_rd_addr),          // rd_addr to forward
        .pc_plus4_o     (ex_pc_plus4),         // pc_plus4 to forward
        .branch_target_addr_o (ex_branch_target_addr),
        .jalr_target_addr_o   (ex_jalr_target_addr),
        .branch_condition_met_o(ex_branch_condition_met)
    );

    // EX/MEM Pipeline Register
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ex_mem_alu_result_reg <= 32'b0;
            ex_mem_rs2_data_reg   <= 32'b0;
            ex_mem_rd_addr_reg    <= 5'b0;
            ex_mem_pc_plus4_reg   <= 32'b0;
            // Control signals
            ex_mem_mem_read_reg   <= 1'b0;
            ex_mem_mem_write_reg  <= 1'b0;
            ex_mem_reg_we_reg     <= 1'b0;
            ex_mem_wb_mux_sel_reg <= 2'b00;
        end else begin // Changed { to begin
            ex_mem_alu_result_reg <= ex_alu_result;
            ex_mem_rs2_data_reg   <= ex_rs2_data; // Pass rs2_data from EX for SW in MEM
            ex_mem_rd_addr_reg    <= ex_rd_addr;  // Pass rd_addr from EX
            ex_mem_pc_plus4_reg   <= ex_pc_plus4; // Pass pc_plus4 from EX
            // Control signals from ID/EX register, passed through EX (or re-evaluated if EX modifies them)
            // For simplicity, assume EX stage doesn't change these control signals, just passes from ID/EX
            ex_mem_mem_read_reg   <= id_ex_mem_read_reg;
            ex_mem_mem_write_reg  <= id_ex_mem_write_reg;
            ex_mem_reg_we_reg     <= id_ex_reg_we_reg;
            ex_mem_wb_mux_sel_reg <= id_ex_wb_mux_sel_reg;
        end
    end

    // MEM Stage
    mem_stage mem_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .alu_result_i   (ex_mem_alu_result_reg), // From EX/MEM Reg
        .rs2_data_i     (ex_mem_rs2_data_reg),   // From EX/MEM Reg (for SW)
        .rd_addr_i      (ex_mem_rd_addr_reg),    // From EX/MEM Reg
        .pc_plus4_i     (ex_mem_pc_plus4_reg),   // From EX/MEM Reg
        .mem_read_i     (ex_mem_mem_read_reg),   // From EX/MEM Reg
        .mem_write_i    (ex_mem_mem_write_reg),  // From EX/MEM Reg
        .data_addr_o    (data_addr_o),
        .data_wdata_o   (data_wdata_o),
        .data_we_o      (data_we_o),
        .data_rdata_i   (data_rdata_i),
        .mem_rdata_o    (mem_rdata),             // Output to MEM/WB Reg
        .alu_result_o   (mem_alu_result),        // Pass through ALU result to MEM/WB Reg
        .rd_addr_o      (mem_rd_addr),           // Pass through rd_addr to MEM/WB Reg
        .pc_plus4_o     (mem_pc_plus4)           // Pass through pc_plus4 to MEM/WB Reg
    );

    // MEM/WB Pipeline Register
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            mem_wb_mem_rdata_reg    <= 32'b0;
            mem_wb_alu_result_reg <= 32'b0;
            mem_wb_rd_addr_reg    <= 5'b0;
            mem_wb_pc_plus4_reg   <= 32'b0;
            // Control signals
            mem_wb_reg_we_reg     <= 1'b0;
            mem_wb_wb_mux_sel_reg <= 2'b00;
        end else begin // Changed { to begin
            mem_wb_mem_rdata_reg    <= mem_rdata;      // Data from memory read
            mem_wb_alu_result_reg <= mem_alu_result;  // ALU result from MEM stage
            mem_wb_rd_addr_reg    <= mem_rd_addr;     // rd_addr from MEM stage
            mem_wb_pc_plus4_reg   <= mem_pc_plus4;    // pc_plus4 from MEM stage
            // Control signals from EX/MEM register, passed through MEM
            mem_wb_reg_we_reg     <= ex_mem_reg_we_reg;
            mem_wb_wb_mux_sel_reg <= ex_mem_wb_mux_sel_reg;
        end
    end

    // WB Stage
    wb_stage wb_stage_inst (
        .clk            (clk),
        .rst            (rst),
        .mem_rdata_i    (mem_wb_mem_rdata_reg),    // From MEM/WB Reg
        .alu_result_i   (mem_wb_alu_result_reg), // From MEM/WB Reg
        .rd_addr_i      (mem_wb_rd_addr_reg),    // From MEM/WB Reg
        .pc_plus4_i     (mem_wb_pc_plus4_reg),   // From MEM/WB Reg
        .reg_we_i       (mem_wb_reg_we_reg),     // From MEM/WB Reg
        .wb_mux_sel_i   (mem_wb_wb_mux_sel_reg), // From MEM/WB Reg
        .wb_reg_we_o    (wb_reg_we),
        .wb_rd_addr_o   (wb_rd_addr),
        .wb_rd_data_o   (wb_rd_data)
    );

    // Connect IF stage PC output to instruction memory address
    assign instr_addr_o = if_pc;

endmodule
