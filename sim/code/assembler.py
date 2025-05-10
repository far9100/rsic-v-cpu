import sys
import re

# RV32I Opcodes and Funct codes
OPCODE_LUI    = 0b0110111
OPCODE_AUIPC  = 0b0010111
OPCODE_JAL    = 0b1101111
OPCODE_JALR   = 0b1100111
OPCODE_BRANCH = 0b1100011
OPCODE_LOAD   = 0b0000011
OPCODE_STORE  = 0b0100011
OPCODE_IMM    = 0b0010011  # ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
OPCODE_REG    = 0b0110011  # ADD, SUB, SLL, SLT, SLTU, XOR, SRL, SRA, OR, AND
OPCODE_FENCE  = 0b0001111
OPCODE_SYSTEM = 0b1110011

# Funct3 for IMM instructions
FUNCT3_ADDI  = 0b000
FUNCT3_SLTI  = 0b010
FUNCT3_SLTIU = 0b011
FUNCT3_XORI  = 0b100
FUNCT3_ORI   = 0b101
FUNCT3_ANDI  = 0b110
FUNCT3_SLLI  = 0b001
FUNCT3_SRLI  = 0b101 # Also SRAI

# Funct3 for REG instructions
FUNCT3_ADD  = 0b000 # Also SUB
FUNCT3_SLL  = 0b001
FUNCT3_SLT  = 0b010
FUNCT3_SLTU = 0b011
FUNCT3_XOR  = 0b100
FUNCT3_SRL  = 0b101 # Also SRA
FUNCT3_OR   = 0b110
FUNCT3_AND  = 0b111

# Funct3 for STORE instructions
FUNCT3_SB = 0b000
FUNCT3_SH = 0b001
FUNCT3_SW = 0b010

# Funct7 for some REG instructions
FUNCT7_ADD   = 0b0000000
FUNCT7_SUB   = 0b0100000
FUNCT7_SRA  = 0b0100000 # Differentiates from SRLI
FUNCT7_SRLI = 0b0000000 # Differentiates from SRAI

# Register mapping
REG_MAP = {
    'x0': 0, 'zero': 0,
    'x1': 1, 'ra': 1,
    'x2': 2, 'sp': 2,
    'x3': 3, 'gp': 3,
    'x4': 4, 'tp': 4,
    'x5': 5, 't0': 5,
    'x6': 6, 't1': 6,
    'x7': 7, 't2': 7,
    'x8': 8, 's0': 8, 'fp': 8,
    'x9': 9, 's1': 9,
    'x10': 10, 'a0': 10,
    'x11': 11, 'a1': 11,
    'x12': 12, 'a2': 12,
    'x13': 13, 'a3': 13,
    'x14': 14, 'a4': 14,
    'x15': 15, 'a5': 15,
    'x16': 16, 'a6': 16,
    'x17': 17, 'a7': 17,
    'x18': 18, 's2': 18,
    'x19': 19, 's3': 19,
    'x20': 20, 's4': 20,
    'x21': 21, 's5': 21,
    'x22': 22, 's6': 22,
    'x23': 23, 's7': 23,
    'x24': 24, 's8': 24,
    'x25': 25, 's9': 25,
    'x26': 26, 's10': 26,
    'x27': 27, 's11': 27,
    'x28': 28, 't3': 28,
    'x29': 29, 't4': 29,
    'x30': 30, 't5': 30,
    'x31': 31, 't6': 31,
}

def get_reg_num(reg_str):
    reg_str = reg_str.lower()
    if reg_str not in REG_MAP:
        raise ValueError(f"Unknown register: {reg_str}")
    return REG_MAP[reg_str]

def to_signed_imm(val, bits):
    """Converts an integer to its signed representation if negative, within bit limits."""
    if val < 0:
        return (1 << bits) + val  # 2's complement
    return val

def parse_immediate(imm_str):
    imm_str = imm_str.lower()
    if imm_str.startswith('0x'):
        return int(imm_str, 16)
    elif imm_str.startswith('0b'):
        return int(imm_str, 2)
    else:
        return int(imm_str)

# --- Encoding Functions ---

def encode_r_type(funct7, rs2, rs1, funct3, rd, opcode):
    return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def encode_i_type(imm, rs1, funct3, rd, opcode):
    imm_val = to_signed_imm(imm, 12) & 0xFFF # Ensure 12-bit immediate
    return (imm_val << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

def encode_s_type(imm, rs2, rs1, funct3, opcode):
    imm_val = to_signed_imm(imm, 12) & 0xFFF # Ensure 12-bit immediate
    imm11_5 = (imm_val >> 5) & 0x7F
    imm4_0  = imm_val & 0x1F
    return (imm11_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (imm4_0 << 7) | opcode

def encode_j_type(imm, rd, opcode):
    imm_val = to_signed_imm(imm, 21) & 0x1FFFFF # Ensure 21-bit immediate (actually 20 bits for JAL, offset is imm/2)
    # J-type immediate: imm[20] imm[10:1] imm[11] imm[19:12]
    imm_20   = (imm_val >> 20) & 0x1
    imm_10_1 = (imm_val >> 1)  & 0x3FF
    imm_11   = (imm_val >> 11) & 0x1
    imm_19_12= (imm_val >> 12) & 0xFF
    encoded_imm = (imm_20 << 19) | (imm_19_12 << 11) | (imm_11 << 10) | imm_10_1 # This is for JAL's specific imm field structure
    # For JAL, the immediate field in the instruction is bits 31:12
    # The structure is imm[20] (at bit 31), imm[10:1] (at bits 30:21), imm[11] (at bit 20), imm[19:12] (at bits 19:12)
    # The value 'encoded_imm' here is already shifted and structured for bits 19:0 of the J-type immediate field.
    # So, it should be shifted by 12 to align with instruction bits 31:12
    return (encoded_imm << 12) | (rd << 7) | opcode # Corrected: J-type imm is bits 31:12. rd is 11:7.

def encode_b_type(imm, rs2, rs1, funct3, opcode):
    # B-type immediate: offset is 13-bit, signed, multiple of 2. imm[0] is always 0.
    # Instruction format: imm[12|10:5] rs2 rs1 funct3 imm[4:1|11] opcode
    imm_val = to_signed_imm(imm, 13) & 0x1FFF # Ensure 13-bit (offset is imm, scaled by 1 internally by CPU)
    
    imm_12   = (imm_val >> 12) & 0x1
    imm_10_5 = (imm_val >> 5)  & 0x3F
    imm_4_1  = (imm_val >> 1)  & 0xF
    imm_11   = (imm_val >> 11) & 0x1
    
    encoded_imm_p1 = (imm_12 << 6) | imm_10_5 # imm[12|10:5] for bits 31:25
    encoded_imm_p2 = (imm_4_1 << 1) | imm_11   # imm[4:1|11] for bits 11:7 (imm[11] is at LSB here for encoding)

    return (encoded_imm_p1 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (encoded_imm_p2 << 7) | opcode

# --- Assembler ---

def assemble(input_asm_file, output_hex_file):
    raw_instructions = []
    labels = {}
    current_address = 0

    # First Pass: Read lines, find labels, and store raw instructions
    with open(input_asm_file, 'r') as f:
        for line_num, line_content in enumerate(f):
            line = line_content.strip()
            
            # Remove comments
            if '#' in line:
                line = line.split('#', 1)[0].strip()

            if not line:
                continue

            # Skip assembler directives (lines starting with '.')
            if line.startswith('.'):
                continue

            # Handle labels
            match_label = re.match(r'([a-zA-Z_][a-zA-Z0-9_]*):(.*)', line)
            if match_label:
                label_name = match_label.group(1)
                remaining_line = match_label.group(2).strip()
                if label_name in labels:
                    raise ValueError(f"Duplicate label '{label_name}' at line {line_num + 1}")
                labels[label_name] = current_address
                line = remaining_line # Continue processing if instruction is on the same line

            if line: # If there's an instruction (or remaining part after label)
                raw_instructions.append({'asm': line, 'addr': current_address, 'line_num': line_num + 1})
                current_address += 4

    # Second Pass: Encode instructions
    machine_code_hex = []
    for instr_info in raw_instructions:
        asm_line = instr_info['asm']
        addr = instr_info['addr']
        line_n = instr_info['line_num']
        
        parts = re.split(r'[,\s()]+', asm_line) # Split by comma, space, or parentheses
        parts = [p for p in parts if p] # Remove empty strings

        mnemonic = parts[0].lower()
        mc = 0

        try:
            if mnemonic == "addi":
                rd = get_reg_num(parts[1])
                rs1 = get_reg_num(parts[2])
                imm = parse_immediate(parts[3])
                mc = encode_i_type(imm, rs1, FUNCT3_ADDI, rd, OPCODE_IMM)
            elif mnemonic == "add":
                rd = get_reg_num(parts[1])
                rs1 = get_reg_num(parts[2])
                rs2 = get_reg_num(parts[3])
                mc = encode_r_type(FUNCT7_ADD, rs2, rs1, FUNCT3_ADD, rd, OPCODE_REG)
            elif mnemonic == "sw": # sw rs2, offset(rs1)
                rs2_reg = get_reg_num(parts[1]) # Source register to store
                imm = parse_immediate(parts[2]) # Offset
                rs1_reg = get_reg_num(parts[3]) # Base address register
                mc = encode_s_type(imm, rs2_reg, rs1_reg, FUNCT3_SW, OPCODE_STORE)
            elif mnemonic == "jal":
                rd = get_reg_num(parts[1])
                target = parts[2]
                if target in labels:
                    target_addr = labels[target]
                else:
                    target_addr = parse_immediate(target) # Allow jumping to absolute address (less common for JAL)
                
                offset = target_addr - addr
                if offset % 2 != 0:
                    raise ValueError(f"JAL offset must be multiple of 2. Target: {target_addr}, Current: {addr} on line {line_n}")
                
                # JAL immediate is sign-extended, 20-bit, and scaled by 2
                # The encode_j_type expects the actual byte offset.
                # JAL immediate field in instruction is already scaled by assembler (offset / 2 implicitly by structure)
                # The immediate for JAL is 20 bits, representing a signed offset in multiples of 2 bytes.
                # So, the 'offset' value passed to encode_j_type should be the direct byte offset.
                # The encode_j_type will then structure it into imm[20], imm[10:1], imm[11], imm[19:12]
                # The JAL immediate is sign-extended, bits 20. imm[0] is always 0.
                # The value stored in instruction is offset >> 1, but structured.
                # Let's ensure encode_j_type handles the raw byte offset correctly.
                # The RISC-V spec shows the J-immediate as Jimm[20:1]
                # imm_val in encode_j_type is the byte offset.
                # The fields are: imm[20] at inst[31], imm[10:1] at inst[30:21], imm[11] at inst[20], imm[19:12] at inst[19:12]
                # This means the immediate is structured from the byte offset.
                mc = encode_j_type(offset, rd, OPCODE_JAL) 
            
            elif mnemonic == "beq": # beq rs1, rs2, label
                rs1_reg = get_reg_num(parts[1])
                rs2_reg = get_reg_num(parts[2])
                target_label = parts[3]
                if target_label not in labels:
                    raise ValueError(f"Undefined label '{target_label}' on line {line_n}")
                target_addr = labels[target_label]
                offset = target_addr - addr
                if offset % 2 != 0:
                    raise ValueError(f"Branch offset must be multiple of 2. Target: {target_addr}, Current: {addr} on line {line_n}")
                # B-type immediate is 13-bit signed byte offset. encode_b_type takes this byte offset.
                mc = encode_b_type(offset, rs2_reg, rs1_reg, 0b000, OPCODE_BRANCH) # funct3 for BEQ is 000

            elif mnemonic == "nop": # Pseudo-instruction: nop is addi x0, x0, 0
                mc = encode_i_type(0, REG_MAP['x0'], FUNCT3_ADDI, REG_MAP['x0'], OPCODE_IMM)
            else:
                raise ValueError(f"Unsupported mnemonic '{mnemonic}' on line {line_n}")

            machine_code_hex.append(f"{mc:08x}")

        except Exception as e:
            print(f"Error assembling line {line_n} ('{instr_info['asm']}'): {e}")
            sys.exit(1)


    with open(output_hex_file, 'w') as f:
        for hex_instr in machine_code_hex:
            f.write(hex_instr + '\n')

    print(f"Assembly successful: '{input_asm_file}' -> '{output_hex_file}'")
    print(f"Labels found: {labels}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python assembler.py <input_asm_file.s> <output_hex_file.hex>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    
    assemble(input_file, output_file)
