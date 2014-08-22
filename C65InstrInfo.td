//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 basic instruction formats
//===----------------------------------------------------------------------===//

// Basic instruction format
class InstC65<dag outs, dag ins, string asmstr, list<dag> pattern>
        : Instruction {
  field bits<8> Inst;

  let Namespace = "C65";
  let Size = 1;

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  let DecoderNamespace = "C65";
}

// Format cc instructions
// 6502 instruction format generally follows: aaabbbcc
class Fcc<bits<3> op, bits<3> addrmode, bits<2> cc,
          dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst{7-5} = op;
  let Inst{4-2} = addrmode;
  let Inst{1-0} = cc;
}

//===----------------------------------------------------------------------===//
// C65-specific node definitions
//===----------------------------------------------------------------------===//

def SDT_C65cmp : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisSameAs<0, 1>]>;
def SDT_C65ret : SDTypeProfile<0, 0, []>;

def C65cmp : SDNode<"C65ISD::CMP", SDT_C65cmp,
                    [SDNPOutGlue]>;
def C65ret : SDNode<"C65ISD::RET", SDTNone,
                    [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

//  These are target-independent nodes, but have target-specific formats.
def SDT_C65CallSeqStart : SDCallSeqStart<[SDTCisVT<0, i32>]>;
def SDT_C65CallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;

def callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_C65CallSeqStart,
                           [SDNPHasChain, SDNPOutGlue]>;
def callseq_end   : SDNode<"ISD::CALLSEQ_END",   SDT_C65CallSeqEnd,
                           [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

//===----------------------------------------------------------------------===//
// C65 assembler operand definitions
//===----------------------------------------------------------------------===//

// 8-bit PC-relative branch target assembler operand
def PCRelAsmOperand : AsmOperandClass {
  let Name = "PCRel";
  let RenderMethod = "addImmOperands";
  let ParserMethod = "parsePCRel";
}

// 16-bit address assembler operand
def MEM16i16AsmOperand : AsmOperandClass {
  let Name = "MEM16i16";
  let ParserMethod = "parseMEMOperand";
}

// 16-bit X-indexed address assembler operand
def MEM16ix16AsmOperand : AsmOperandClass {
  let Name = "MEM16ix16";
  let ParserMethod = "parseMEMOperand";
}

// 16-bit Y-indexed address assembler operand
def MEM16iy16AsmOperand : AsmOperandClass {
  let Name = "MEM16iy16";
  let ParserMethod = "parseMEMOperand";
}

// 16-bit S-indexed address assembler operand
def MEM16is16AsmOperand : AsmOperandClass {
  let Name = "MEM16is16";
  let ParserMethod = "parseMEMOperand";
}

//===----------------------------------------------------------------------===//
// C65 complex pattern definitions
//===----------------------------------------------------------------------===//

def addr_ix : ComplexPattern<iPTR, 2, "SelectAddrXY", [], []>;
def addr_iy : ComplexPattern<iPTR, 2, "SelectAddrXY", [], []>;
def addr_is : ComplexPattern<iPTR, 2, "SelectAddrS", [frameindex], []>;

//===----------------------------------------------------------------------===//
// C65 operand class definitions
//===----------------------------------------------------------------------===//

// 8-bit PC-relative branch target operand
def brtarget8 : Operand<OtherVT> {
  let EncoderMethod = "getPCEncoding";
  let DecoderMethod = "decodePCOperand";
  let ParserMatchClass = PCRelAsmOperand;
  let MIOperandInfo = (ops i8imm);
}

// 16-bit absolute branch target operand
def brtarget16 : Operand<OtherVT> {
  let PrintMethod = "printMemOperandAbs";
  let MIOperandInfo = (ops i16imm);
}

multiclass C65AddressingMode {
  let PrintMethod = "printMemOperandAbs" in {
    def _a16 : Operand<iPTR> {
      let MIOperandInfo = (ops i16imm);
    }
  }
  let PrintMethod = "printMemOperandIndex" in {
    def _ix16 : Operand<iPTR> {
      let MIOperandInfo = (ops IX16, i16imm);
    }
    def _iy16 : Operand<iPTR> {
      let MIOperandInfo = (ops IY16, i16imm);
    }
    def _is16 : Operand<iPTR> {
      let MIOperandInfo = (ops IS16, i16imm);
    }
  }
}

defm i8mem : C65AddressingMode;
defm i16mem : C65AddressingMode;

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern>;

multiclass F01_binary<bits<3> op, string asmstr, SDPatternOperator operator> {
  // Binary operation on 16-bit accumulator
  def imm  : F01<op, 0b010, (outs ACC16:$dst),
                 (ins ACC16:$src, i16imm:$imm),
                 !strconcat(asmstr, " $imm"),
                 [(set ACC16:$dst, (operator ACC16:$src, imm:$imm))]>;

  let mayLoad = 1 in {
    def i  : F01<op, 0b011, (outs ACC16:$dst),
                 (ins ACC16:$src, i16mem_a16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set ACC16:$dst,
                    (operator ACC16:$src, (load tglobaladdr:$addr)))]>;

    def ix : F01<op, 0b110, (outs ACC16:$dst),
                 (ins ACC16:$src, i16mem_ix16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set ACC16:$dst,
                    (operator ACC16:$src, (load addr_ix:$addr)))]>;
  }
}

let Defs = [P], Constraints = "$src = $dst" in {
  defm ORA : F01_binary<0b000, "ORA", or>;
  defm AND : F01_binary<0b001, "AND", and>;
  defm EOR : F01_binary<0b010, "EOR", xor>;
  let Uses = [P] in {
    defm ADC : F01_binary<0b011, "ADC", add>; //TODO: Change to adde!
    defm SBC : F01_binary<0b111, "SBC", sub>; // sube
  }
}

let mayStore = 1 in {
  def STAi   : F01<0b100, 0b011, (outs), (ins ACC16:$src, i16mem_a16:$addr),
                   "STA $addr",
                   [(store ACC16:$src, tglobaladdr:$addr)]>;

  def STAix  : F01<0b100, 0b110, (outs), (ins ACC16:$src, i16mem_ix16:$addr),
                   "STA $addr",
                   [(store ACC16:$src, addr_ix:$addr)]>;
}

let Defs = [P] in {
  let isMoveImm = 1 in {
    def LDAimm : F01<0b101, 0b010, (outs ACC16:$dst), (ins i16imm:$imm),
                     "LDA $imm",
                     [(set ACC16:$dst, imm:$imm)]>;
  }

  def CMPimm   : F01<0b110, 0b010, (outs), (ins ACC16:$src, i16imm:$imm),
                     "CMP $imm",
                     [(C65cmp ACC16:$src, imm:$imm)]>;

  let mayLoad = 1 in {
    def LDAi   : F01<0b101, 0b011, (outs ACC16:$dst), (ins i16mem_a16:$addr),
                     "LDA $addr",
                     [(set ACC16:$dst, (load tglobaladdr:$addr))]>;

    def LDAix  : F01<0b101, 0b110, (outs ACC16:$dst), (ins i16mem_ix16:$addr),
                     "LDA $addr",
                     [(set ACC16:$dst, (load addr_ix:$addr))]>;

    def CMPi   : F01<0b110, 0b011, (outs), (ins ACC16:$src, i16mem_a16:$addr),
                     "CMP $addr",
                     [(C65cmp ACC16:$src, (load tglobaladdr:$addr))]>;

    def CMPix  : F01<0b110, 0b011, (outs), (ins ACC16:$src, i16mem_ix16:$addr),
                     "CMP $addr",
                     [(C65cmp ACC16:$src, (load addr_ix:$addr))]>;
  }
}

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc
//===----------------------------------------------------------------------===//

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern>;

multiclass F10_unary<bits<3> op, string asmstr, SDPatternOperator operator> {
  // Unary operator on 16-bit memory
  let mayLoad = 1, mayStore = 1 in {
    def i  : F10<op, 0b011, (outs), (ins i16mem_a16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store (operator (i16 (load tglobaladdr:$addr)), (i16 1)),
                     tglobaladdr:$addr)]>;

    def ix : F10<op, 0b111, (outs), (ins i16mem_ix16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store (operator (i16 (load addr_ix:$addr)), (i16 1)),
                     addr_ix:$addr)]>;
  }
}

let Defs = [P] in {
  defm ASL : F10_unary<0b000, "ASL", shl>;
  defm ROL : F10_unary<0b001, "ROL", rotl>;
  defm LSR : F10_unary<0b010, "LSR", srl>;
  defm ROR : F10_unary<0b011, "ROR", rotr>;
  defm DEC : F10_unary<0b110, "DEC", add>;
  defm INC : F10_unary<0b111, "INC", sub>;
}

let mayStore = 1 in {
  def STXi   : F01<0b100, 0b011, (outs), (ins IX16:$src, i16mem_a16:$addr),
                   "STX $addr",
                   [(store IX16:$src, tglobaladdr:$addr)]>;
}

let Defs = [P], Uses = [P], Constraints = "$src = $dst" in {
  def ASLa : F10<0b000, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                 "ASL A",
                 [(set ACC16:$dst, (shl ACC16:$src, (i8 1)))]>;

  def ROLa : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                 "ROL A",
                 [(set ACC16:$dst, (rotl ACC16:$src, (i8 1)))]>;

  def LSRa : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                 "LSR A",
                 [(set ACC16:$dst, (srl ACC16:$src, (i8 1)))]>;

  def RORa : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                 "ROR A",
                 [(set ACC16:$dst, (rotr ACC16:$src, (i8 1)))]>;
}

let Defs = [P] in {
  let isMoveImm = 1 in {
    def LDXimm : F10<0b101, 0b000, (outs IX16:$dst), (ins i16imm:$imm),
                     "LDX $imm",
                     [(set IX16:$dst, imm:$imm)]>;
  }

  let mayLoad = 1 in {
    def LDXi  : F10<0b101, 0b011, (outs IX16:$dst), (ins i16mem_a16:$addr),
                    "LDX $addr",
                    [(set IX16:$dst, (load tglobaladdr:$addr))]>;

    def LDXiy : F10<0b101, 0b111, (outs IX16:$dst), (ins i16mem_iy16:$addr),
                    "LDX $addr",
                    [(set IX16:$dst, (load addr_iy:$addr))]>;
  }
}

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, (jmp), sty, ldy, cpy, cpx
//===----------------------------------------------------------------------===//

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern>;

// TODO: Decide if Y register is required for bare-bones operation
let mayStore = 1 in {
  def STYi : F00<0b100, 0b011, (outs), (ins IY16:$src, i16mem_a16:$addr),
                 "STY $addr",
                 [(store IY16:$src, tglobaladdr:$addr)]>;
}

let Defs = [P] in {
  let isMoveImm = 1 in {
    def LDYimm : F00<0b101, 0b000, (outs IY16:$dst), (ins i16imm:$imm),
                     "LDY $imm",
                     [(set IY16:$dst, imm:$imm)]>;
  }

  let mayLoad = 1 in {
    def LDYi   : F00<0b101, 0b011, (outs IY16:$dst), (ins i16mem_a16:$addr),
                     "LDY $addr",
                     [(set IY16:$dst, (load tglobaladdr:$addr))]>;

    def LDYix  : F00<0b101, 0b111, (outs IY16:$dst), (ins i16mem_ix16:$addr),
                     "LDY $addr",
                     [(set IY16:$dst, (load addr_ix:$addr))]>;
  }
}

let Defs = [P] in {
  def CPYimm : F01<0b110, 0b000, (outs), (ins IY16:$src, i16imm:$imm),
                   "CPY $imm",
                   [(C65cmp IY16:$src, imm:$imm)]>;

  def CPXimm : F01<0b111, 0b000, (outs), (ins IX16:$src, i16imm:$imm),
                   "CPX $imm",
                   [(C65cmp IX16:$src, imm:$imm)]>;

  let mayLoad = 1 in {
    def CPYi   : F01<0b110, 0b011, (outs), (ins IY16:$src, i16mem_a16:$addr),
                     "CPY $addr",
                     [(C65cmp IY16:$src, (load tglobaladdr:$addr))]>;

    def CPXi   : F01<0b111, 0b011, (outs), (ins IX16:$src, i16mem_a16:$addr),
                     "CPX $addr",
                     [(C65cmp IX16:$src, (load tglobaladdr:$addr))]>;
  }
}

//===----------------------------------------------------------------------===//
// Format 11 instructions : Extended ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F11<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b11, outs, ins, asmstr, pattern>;

multiclass F11_binary<bits<3> op, string asmstr, SDPatternOperator operator> {
  // Binary operation on 16-bit accumulator
  let mayLoad = 1 in {
    def is : F11<op, 0b000, (outs ACC16:$dst),
                 (ins ACC16:$src, i16mem_is16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set ACC16:$dst,
                    (operator ACC16:$src, (load addr_is:$addr)))]>;
  }
}

let Defs = [P], Constraints = "$src = $dst" in {
  defm ORA : F11_binary<0b000, "ORA", or>;
  defm AND : F11_binary<0b001, "AND", and>;
  defm EOR : F11_binary<0b010, "EOR", xor>;
  let Uses = [P] in {
    defm ADC : F11_binary<0b011, "ADC", add>; //TODO: Change to adde!
    defm SBC : F11_binary<0b111, "SBC", sub>; // sube
  }
}

let mayStore = 1 in {
  def STAis : F11<0b100, 0b000, (outs), (ins ACC16:$src, i16mem_is16:$addr),
                  "STA $addr",
                  [(store ACC16:$src, addr_is:$addr)]>;
}

let Defs = [P], mayLoad = 1 in {
  def LDAis  : F11<0b101, 0b000, (outs ACC16:$dst), (ins i16mem_is16:$addr),
                   "LDA $addr",
                   [(set ACC16:$dst, (load addr_is:$addr))]>;

  def CMPis  : F11<0b110, 0b000, (outs), (ins ACC16:$src, i16mem_is16:$addr),
                   "CMP $addr",
                   [(C65cmp ACC16:$src, (load addr_is:$addr))]>;
}

//===----------------------------------------------------------------------===//
// Conditional branch instructions : bpl, bmi, bvc, bvs, bcc, bcs, bne, beq
//===----------------------------------------------------------------------===//

class CondFlag<bits<2> ix> { bits<2> flagIndex = ix; }
def CondNegative : CondFlag<0b00>;
def CondOverflow : CondFlag<0b01>;
def CondCarry    : CondFlag<0b10>;
def CondZero     : CondFlag<0b11>;

// Branch if the value of the flag 'cf' equals 'value'
class FC<CondFlag cf, bit value, string asmstr>
    : InstC65<(outs), (ins brtarget8:$dst), asmstr, []> {
  let Inst{7-6} = cf.flagIndex;
  let Inst{5} = value;
  let Inst{4-0} = 0b10000;
}

let Uses = [P], isBranch = 1, isTerminator = 1 in {
  def BPL : FC<CondNegative, 0, "BPL">;
  def BMI : FC<CondNegative, 1, "BMI">;
  def BVC : FC<CondOverflow, 0, "BVC">;
  def BVS : FC<CondOverflow, 1, "BVS">;
  def BCC : FC<CondCarry,    0, "BCC">;
  def BCS : FC<CondCarry,    1, "BCS">;
  def BNE : FC<CondZero,     0, "BNE">;
  def BEQ : FC<CondZero,     1, "BEQ">;
}

// def C65_COND_PL : PatLeaf<(i8 0)>;  // alt. COND_NBE
// def C65_COND_MI : PatLeaf<(i8 1)>;  // alt. COND_NC
// def C65_COND_  : PatLeaf<(i8 2)>;  // alt. COND_C
// def C65_COND_BE  : PatLeaf<(i8 3)>;  // alt. COND_NA
// def C65_COND_E   : PatLeaf<(i8 4)>;  // alt. COND_Z
// def C65_COND_G   : PatLeaf<(i8 5)>;  // alt. COND_NLE
// def C65_COND_GE  : PatLeaf<(i8 6)>;  // alt. COND_NL
// def C65_COND_L   : PatLeaf<(i8 7)>;  // alt. COND_NGE
// def C65_COND_LE  : PatLeaf<(i8 8)>;  // alt. COND_NG
// def C65_COND_NE  : PatLeaf<(i8 9)>;  // alt. COND_NZ
// def C65_COND_NO  : PatLeaf<(i8 10)>;
// def C65_COND_NP  : PatLeaf<(i8 11)>; // alt. COND_PO

//===----------------------------------------------------------------------===//
// Free format instructions
//===----------------------------------------------------------------------===//

class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst = op;
}

let neverHasSideEffects = 1 in {
  def NOOP : FF<0xea, (outs), (ins), "NOP", []>;
}

let Defs = [P] in {
  def BITimm : FF<0x89, (outs), (ins i16imm:$imm), "BIT $imm",
                  []>;
  def BITi   : FF<0x24, (outs), (ins i16mem_a16:$addr), "BIT $addr",
                  []>;
  def BITix  : FF<0x3c, (outs), (ins i16mem_ix16:$addr), "BIT $addr",
                  []>;
}

def STZi  : FF<0x9c, (outs), (ins i16mem_a16:$addr), "STZ $addr",
               [(store (i16 0), tglobaladdr:$addr)]>;
def STZix : FF<0x9e, (outs), (ins i16mem_ix16:$addr), "STZ $addr",
               [(store (i16 0), addr_ix:$addr)]>;

// Push and pull stack operations
let neverHasSideEffects = 1 in {
  let Defs = [P], mayLoad = 1 in {
    def PLA : FF<0x68, (outs ACC16:$dst), (ins), "PLA", []>;
    def PLX : FF<0xfa, (outs IX16:$dst), (ins), "PLX", []>;
    def PLY : FF<0x7a, (outs IY16:$dst), (ins), "PLY", []>;
    def PLD : FF<0x2b, (outs ID16:$dst), (ins), "PLD", []>;
    def PLP : FF<0x08, (outs CCR:$dst), (ins), "PLP", []>;
  }
  let mayStore = 1 in {
    def PHA : FF<0x48, (outs), (ins ACC16:$src), "PHA", []>;
    def PHX : FF<0xda, (outs), (ins IX16:$src), "PHX", []>;
    def PHY : FF<0x5a, (outs), (ins IY16:$src), "PHY", []>;
    def PHD : FF<0x0b, (outs), (ins ID16:$src), "PLD", []>;
    def PHP : FF<0x28, (outs), (ins CCR:$src), "PLP", []>;
    def PEA : FF<0xf4, (outs), (ins i16mem_a16:$addr), "PEA $addr", []>;
    def PER : FF<0x62, (outs), (ins brtarget8:$addr), "PER $addr", []>;
  }
  let mayLoad = 1, mayStore = 1 in {
    def PEI : FF<0xd4, (outs), (ins i16mem_a16:$addr), "PEI $addr", []>;
  }
}

// Branch relative and jump instructions
let isBranch = 1, isTerminator = 1, isBarrier = 1 in {
  def BRA : FF<0x80, (outs), (ins brtarget8:$addr),
               "BRA $addr",
               [(br bb:$addr)]>;

  def JMP : FF<0x6c, (outs), (ins brtarget16:$addr),
               "JMP $addr",
               [(br bb:$addr)]>;
}

// Jump subroutine (call)
let Defs = [A, X, Y, P], isCall = 1 in {
  def JSR : FF<0x20, (outs), (ins i16mem_a16:$addr),
               "JSR $addr",
               []>;
}

// Return from subroutine
let isReturn = 1, isTerminator = 1, isBarrier = 1,
     DecoderMethod = "DecodeReturn" in {
  def RTS : FF<0x60, (outs), (ins), "RTS", [(C65ret)]>;
}

// Transfer between registers
let Defs = [P], neverHasSideEffects = 1, isCodeGenOnly = 1 in {
  def TAX : FF<0xaa, (outs IX16:$dst), (ins ACC16:$src), "TAX",
               [(set IX16:$dst, ACC16:$src)]>;

  def TAY : FF<0xa8, (outs IY16:$dst), (ins ACC16:$src), "TAY",
               [(set IY16:$dst, ACC16:$src)]>;

  def TXA : FF<0x8a, (outs ACC16:$dst), (ins IX16:$src), "TXA",
               [(set ACC16:$dst, IX16:$src)]>;

  def TYA : FF<0xa8, (outs ACC16:$dst), (ins IY16:$src), "TYA",
               [(set ACC16:$dst, IY16:$src)]>;

  def TXY : FF<0x9b, (outs IY16:$dst), (ins IX16:$src), "TXY",
               [(set IY16:$dst, IX16:$src)]>;

  def TYX : FF<0xbb, (outs IX16:$dst), (ins IY16:$src), "TYX",
               [(set IX16:$dst, IY16:$src)]>;

  def TXS : FF<0x9b, (outs IS16:$dst), (ins IX16:$src), "TXS",
               [(set IS16:$dst, IX16:$src)]>;

  def TSX : FF<0xbb, (outs IX16:$dst), (ins IS16:$src), "TSX",
               [(set IX16:$dst, IS16:$src)]>;
}

// Clear and set flag instructions
let Defs = [P], Uses = [P] in {
  def CLC : FF<0x18, (outs), (ins), "CLC", []>;
  def SEC : FF<0x38, (outs), (ins), "STC", []>;
  def CLD : FF<0xd8, (outs), (ins), "CLD", []>;
  def SED : FF<0xf8, (outs), (ins), "STD", []>;
  def CLI : FF<0x58, (outs), (ins), "CLI", []>;
  def SEI : FF<0x78, (outs), (ins), "STI", []>;
  def CLV : FF<0xb8, (outs), (ins), "CLV", []>;
  def REP : FF<0xc2, (outs), (ins i8imm:$imm), "REP $imm", []>;
  def SEP : FF<0xe2, (outs), (ins i8imm:$imm), "SEP $imm", []>;
}

// Increment and decrement instructions
let Defs = [P], Constraints = "$src = $dst" in {
  def INCa : FF<0x1a, (outs ACC16:$dst), (ins ACC16:$src), "INC A",
                [(set ACC16:$dst, (add ACC16:$src, 1))]>;

  def INX  : FF<0xe8, (outs IX16:$dst), (ins IX16:$src), "INX",
                [(set IX16:$dst, (add IX16:$src, 1))]>;

  def INY  : FF<0xc8, (outs IY16:$dst), (ins IY16:$src), "INY",
                [(set IY16:$dst, (add IY16:$src, 1))]>;

  def DECa : FF<0x3a, (outs ACC16:$dst), (ins ACC16:$src), "DEC A",
                [(set ACC16:$dst, (sub ACC16:$src, 1))]>;

  def DEX  : FF<0xca, (outs IX16:$dst), (ins IX16:$src), "DEX",
                [(set IX16:$dst, (sub IX16:$src, 1))]>;

  def DEY  : FF<0x88, (outs IY16:$dst), (ins IY16:$src), "DEY",
                [(set IY16:$dst, (sub IY16:$src, 1))]>;
}

//===----------------------------------------------------------------------===//
// Pseudo-instructions
//===----------------------------------------------------------------------===//

class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
}

// ADJCALLSTACKDOWN, ADJCALLSTACKUP are mandatory pseudo-instructions
let Defs = [S, P], Uses = [S], isCodeGenOnly = 1 in {
  def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                                 ";!ADJCALLSTACKDOWN $amt",
                                 [(callseq_start timm:$amt)]>;
  def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                              ";!ADJCALLSTACKUP $amt1",
                              [(callseq_end timm:$amt1, timm:$amt2)]>;
}

// let Defs = [P], Constraints = "$src = $dst" in {
//   def SHLimm  : Pseudo<(outs ACC16:$dst), (ins ACC16:$src, i16imm:$amt),
//                        ";!SHL A, $amt",
//                        [(set ACC16:$dst, (shl ACC16:$src, timm:$amt))]>;
//   def LSHRimm : Pseudo<(outs ACC16:$dst), (ins ACC16:$src, i16imm:$amt),
//                        ";!LSHR A, $amt",
//                        [(set ACC16:$dst, (lshr ACC16:$src, timm:$amt))]>;
//   def ASHRimm : Pseudo<(outs ACC16:$dst), (ins ACC16:$src, i16imm:$amt),
//                        ";!ASHR A, $amt",
//                        [(set ACC16:$dst, (ashr ACC16:$src, timm:$amt))]>;
// }
