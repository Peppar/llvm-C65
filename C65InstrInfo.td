//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 predicates
//===----------------------------------------------------------------------===//

def Has65C02  : Predicate<"Subtarget->has65C02()">;
def Has65C816 : Predicate<"Subtarget->has65C816()">;

class AccSize<bits<2> val> {
  bits<2> ASize = val;
}
class AccSizeNA : AccSize<0>; // Not affected by accumulator flag
class AccSize8  : AccSize<1>; // Requires 8-bit accumulator
class AccSize16 : AccSize<2>; // Requires 16-bit accumulator (M flag clear)

class IndexSize<bits<2> val> {
  bits<2> IxSize = val;
}
class IndexSizeNA : IndexSize<0>; // Not affected by index flag
class IndexSize8  : IndexSize<1>; // Requires 8-bit index
class IndexSize16 : IndexSize<2>; // Requires 16-bit index (X flag clear)

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

def SDT_C65cmp   : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisSameAs<0, 1>]>;
def SDT_C65spill : SDTypeProfile<0, 1, [SDTCisInt<0>]>;

def C65cmp   : SDNode<"C65ISD::CMP", SDT_C65cmp,
                      [SDNPOutGlue]>;
def C65ret   : SDNode<"C65ISD::RET", SDTNone,
                      [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def C65spill : SDNode<"C65ISD::SPILL", SDT_C65spill,
                      [SDNPHasChain, SDNPMayStore,
                       SDNPMayLoad, SDNPMemOperand]>;

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

// 16-bit zero page assembler operand
def MEM16zpAsmOperand : AsmOperandClass {
  let Name = "MEM16zp";
  let ParserMethod = "parseMEMOperand";
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

def addr_zp  : ComplexPattern<iPTR, 1, "SelectAddrZP", [], []>;
def addr_izy : ComplexPattern<iPTR, 2, "SelectAddrZY", [], []>;
def addr_ix  : ComplexPattern<iPTR, 2, "SelectAddrXY", [], []>;
def addr_iy  : ComplexPattern<iPTR, 2, "SelectAddrXY", [], []>;
def addr_is  : ComplexPattern<iPTR, 2, "SelectAddrS", [frameindex], []>;

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

let PrintMethod = "printMemOperandAbs" in {
  def mem_zp : Operand<iPTR> {
    let MIOperandInfo = (ops i8imm);
  }
  def mem_a16 : Operand<iPTR> {
    let MIOperandInfo = (ops i16imm);
  }
  // def mem_a24 : Operand<iPTR> {
  //   let MIOperandInfo = (ops i24imm);
  // }
}
let PrintMethod = "printMemOperandIndex" in {
  def mem_izy16 : Operand<iPTR> {
    let MIOperandInfo = (ops ZRC16, IY16);
  }
  def mem_ix16 : Operand<iPTR> {
    let MIOperandInfo = (ops IX16, i16imm);
  }
  def mem_iy16 : Operand<iPTR> {
    let MIOperandInfo = (ops IY16, i16imm);
  }
  def mem_is16 : Operand<iPTR> {
    let MIOperandInfo = (ops IS16, i16imm);
  }
}

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern>;

// Requires<[Has65C02]>;
// Requires<[Has65C816]>;

multiclass F01_binary<bits<3> op, string asmstr,
                      SDPatternOperator operator,
                      RegisterClass RC, Operand immOp> {
  def imm : F01<op, 0b010, (outs RC:$dst), (ins RC:$src, immOp:$imm),
                !strconcat(asmstr, " $imm"),
                [(set RC:$dst, (operator RC:$src, imm:$imm)), (implicit P)]>;

  let mayLoad = 1 in {
    def zp   : F01<op, 0b001, (outs RC:$dst), (ins RC:$src, mem_zp:$addr),
                   !strconcat(asmstr, " $addr"),
                   [(set RC:$dst,
                      (operator RC:$src, (load addr_zp:$addr))),
                    (implicit P)]>;

    def i    : F01<op, 0b011, (outs RC:$dst), (ins RC:$src, mem_a16:$addr),
                   !strconcat(asmstr, " $addr"),
                   [(set RC:$dst,
                      (operator RC:$src, (load tglobaladdr:$addr))),
                    (implicit P)]>;

    def ix   : F01<op, 0b110, (outs RC:$dst), (ins RC:$src, mem_ix16:$addr),
                   !strconcat(asmstr, " $addr"),
                   [(set RC:$dst,
                      (operator RC:$src, (load addr_ix:$addr))),
                    (implicit P)]>;
  }
}

// multiclass F01_binary<bits<3> op, string asmstr, SDPatternOperator operator> {
//   defm i8  : F01_binary_p<op, asmstr, operator, ACC8,  i8imm>;
//   defm i16 : F01_binary_p<op, asmstr, operator, ACC16, i16imm>;
// }

// SDNode - shl
// ValueType - i64
// RCClass - I64Regs
// Operand - simm130p

let Defs = [P], Constraints = "$src = $dst" in {
  defm ORA8  : F01_binary<0b000, "ORA", or,  ACC8,  i8imm>,  AccSize8;
  defm ORA16 : F01_binary<0b000, "ORA", or,  ACC16, i16imm>, AccSize16;
  defm AND8  : F01_binary<0b001, "AND", and, ACC8,  i8imm>,  AccSize8;
  defm AND16 : F01_binary<0b001, "AND", and, ACC16, i16imm>, AccSize16;
  defm EOR8  : F01_binary<0b010, "EOR", xor, ACC8,  i8imm>,  AccSize8;
  defm EOR16 : F01_binary<0b010, "EOR", xor, ACC16, i16imm>, AccSize16;
  let Uses = [P] in {
    // FIXME: Not really add/sub, but adde/sube
    defm ADC8  : F01_binary<0b011, "ADC", add, ACC8,  i8imm>,  AccSize8;
    defm ADC16 : F01_binary<0b011, "ADC", add, ACC16, i16imm>, AccSize16;
    defm SBC8  : F01_binary<0b111, "SBC", sub, ACC8,  i8imm>,  AccSize8;
    defm SBC16 : F01_binary<0b111, "SBC", sub, ACC16, i16imm>, AccSize16;
  }
}

let mayStore = 1 in {
  def STAzp  : F01<0b100, 0b001, (outs), (ins ACC16:$src, mem_zp:$addr),
                   "STA $addr",
                   [(store ACC16:$src, addr_zp:$addr)]>;

  def STAi   : F01<0b100, 0b011, (outs), (ins ACC16:$src, mem_a16:$addr),
                   "STA $addr",
                   [(store ACC16:$src, tglobaladdr:$addr)]>;

  def STAix  : F01<0b100, 0b110, (outs), (ins ACC16:$src, mem_ix16:$addr),
                   "STA $addr",
                   [(store ACC16:$src, addr_ix:$addr)]>;
}

let Defs = [P] in {
  let isMoveImm = 1 in {
    def LDAimm : F01<0b101, 0b010, (outs ACC16:$dst), (ins i16imm:$imm),
                     "LDA $imm",
                     [(set ACC16:$dst, imm:$imm), (implicit P)]>;
  }

  def CMPimm   : F01<0b110, 0b010, (outs), (ins ACC16:$src, i16imm:$imm),
                     "CMP $imm",
                     [(C65cmp ACC16:$src, imm:$imm), (implicit P)]>;

  let mayLoad = 1 in {
    def LDAzp  : F01<0b101, 0b011, (outs ACC16:$dst), (ins mem_zp:$addr),
                     "LDA $addr",
                     [(set ACC16:$dst, (load addr_zp:$addr)),
                      (implicit P)]>;

    def LDAi   : F01<0b101, 0b011, (outs ACC16:$dst), (ins mem_a16:$addr),
                     "LDA $addr",
                     [(set ACC16:$dst, (load tglobaladdr:$addr)),
                      (implicit P)]>;

    def LDAix  : F01<0b101, 0b110, (outs ACC16:$dst), (ins mem_ix16:$addr),
                     "LDA $addr",
                     [(set ACC16:$dst, (load addr_ix:$addr)),
                      (implicit P)]>;

    def CMPzp  : F01<0b110, 0b001, (outs), (ins ACC16:$src, mem_zp:$addr),
                     "CMP $addr",
                     [(C65cmp ACC16:$src, (load addr_zp:$addr)),
                      (implicit P)]>;

    def CMPi   : F01<0b110, 0b011, (outs), (ins ACC16:$src, mem_a16:$addr),
                     "CMP $addr",
                     [(C65cmp ACC16:$src, (load tglobaladdr:$addr)),
                      (implicit P)]>;

    def CMPix  : F01<0b110, 0b011, (outs), (ins ACC16:$src, mem_ix16:$addr),
                     "CMP $addr",
                     [(C65cmp ACC16:$src, (load addr_ix:$addr)),
                      (implicit P)]>;
  }
}

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc
//===----------------------------------------------------------------------===//

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern>;

multiclass F10_unary<bits<3> op, string asmstr, SDPatternOperator operator>,
                     ValueType Ty> {
  // Unary operator on memory
  let mayLoad = 1, mayStore = 1 in {
    def zp : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store (operator (Ty (load addr_zp:$addr)), (Ty 1)),
                     addr_zp:$addr), (implicit P)]>;

    def i  : F10<op, 0b011, (outs), (ins mem_a16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store (operator (Ty (load tglobaladdr:$addr)), (Ty 1)),
                     tglobaladdr:$addr), (implicit P)]>;

    def ix : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store (operator (Ty (load addr_ix:$addr)), (Ty 1)),
                     addr_ix:$addr), (implicit P)]>;
  }
}

let Defs = [P] in {
  defm ASL8  : F10_unary<0b000, "ASL", shl,  i8>,  AccSize8;
  defm ASL16 : F10_unary<0b000, "ASL", shl,  i16>, AccSize16;
  defm ROL8  : F10_unary<0b001, "ROL", rotl, i8>,  AccSize8;
  defm ROL16 : F10_unary<0b001, "ROL", rotl, i16>, AccSize16;
  defm LSR8  : F10_unary<0b010, "LSR", srl,  i8>,  AccSize8;
  defm LSR16 : F10_unary<0b010, "LSR", srl,  i16>, AccSize16;
  defm ROR8  : F10_unary<0b011, "ROR", rotr, i8>,  AccSize8;
  defm ROR16 : F10_unary<0b011, "ROR", rotr, i16>, AccSize16;
  defm DEC8  : F10_unary<0b110, "DEC", add,  i8>,  AccSize8;
  defm DEC16 : F10_unary<0b110, "DEC", add,  i16>, AccSize16;
  defm INC8  : F10_unary<0b111, "INC", sub,  i8>,  AccSize8;
  defm INC16 : F10_unary<0b111, "INC", sub,  i16>, AccSize16;
}

let mayStore = 1 in {
  def STXzp  : F01<0b100, 0b001, (outs), (ins IX16:$src, mem_zp:$addr),
                   "STX $addr",
                   [(store IX16:$src, addr_zp:$addr)]>;

  def STXi   : F01<0b100, 0b011, (outs), (ins IX16:$src, mem_a16:$addr),
                   "STX $addr",
                   [(store IX16:$src, tglobaladdr:$addr)]>;
}

let Defs = [P], Uses = [P], Constraints = "$src = $dst" in {
  def ASL8a  : F10<0b000, 0b010, (outs ACC8:$dst), (ins ACC8:$src),
                   "ASL A",
                   [(set ACC8:$dst, (shl ACC8:$src, (i8 1))),
                    (implicit P)]>, AccSize8;

  def ASL16a : F10<0b000, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                   "ASL A",
                   [(set ACC16:$dst, (shl ACC16:$src, (i16 1))),
                    (implicit P)]>, AccSize16;

  def ROL8a  : F10<0b001, 0b010, (outs ACC8:$dst), (ins ACC8:$src),
                   "ROL A",
                   [(set ACC8:$dst, (rotl ACC8:$src, (i8 1))),
                    (implicit P)]>, AccSize8;

  def ROL16a : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                   "ROL A",
                   [(set ACC16:$dst, (rotl ACC16:$src, (i16 1))),
                    (implicit P)]>, AccSize16;

  def LSR8a  : F10<0b001, 0b010, (outs ACC8:$dst), (ins ACC8:$src),
                   "LSR A",
                   [(set ACC8:$dst, (srl ACC8:$src, (i8 1))),
                    (implicit P)]>, AccSize8;

  def LSR16a : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                   "LSR A",
                   [(set ACC16:$dst, (srl ACC16:$src, (i16 1))),
                    (implicit P)]>, AccSize16;

  def ROR8a  : F10<0b001, 0b010, (outs ACC8:$dst), (ins ACC8:$src),
                   "ROR A",
                   [(set ACC8:$dst, (rotr ACC8:$src, (i8 1))),
                    (implicit P)]>, AccSize8;

  def ROR16a : F10<0b001, 0b010, (outs ACC16:$dst), (ins ACC16:$src),
                   "ROR A",
                   [(set ACC16:$dst, (rotr ACC16:$src, (i16 1))),
                    (implicit P)]>, AccSize16;
}

let Defs = [P] in {
  let isMoveImm = 1 in {
    def LDXimm : F10<0b101, 0b000, (outs IX16:$dst), (ins i16imm:$imm),
                     "LDX $imm",
                     [(set IX16:$dst, imm:$imm)]>;
  }

  let mayLoad = 1 in {
    def LDXzp : F10<0b101, 0b001, (outs IX16:$dst), (ins mem_zp:$addr),
                    "LDX $addr",
                    [(set IX16:$dst, (load addr_zp:$addr))]>;

    def LDXi  : F10<0b101, 0b011, (outs IX16:$dst), (ins mem_a16:$addr),
                    "LDX $addr",
                    [(set IX16:$dst, (load tglobaladdr:$addr))]>;

    def LDXiy : F10<0b101, 0b111, (outs IX16:$dst), (ins mem_iy16:$addr),
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

let mayStore = 1 in {
  def STYzp : F00<0b100, 0b001, (outs), (ins IY16:$src, mem_zp:$addr),
                  "STY $addr",
                  [(store IY16:$src, addr_zp:$addr)]>;

  def STYi  : F00<0b100, 0b011, (outs), (ins IY16:$src, mem_a16:$addr),
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
    def LDYzp  : F00<0b101, 0b001, (outs IY16:$dst), (ins mem_zp:$addr),
                     "LDY $addr",
                     [(set IY16:$dst, (load addr_zp:$addr))]>;

    def LDYi   : F00<0b101, 0b011, (outs IY16:$dst), (ins mem_a16:$addr),
                     "LDY $addr",
                     [(set IY16:$dst, (load tglobaladdr:$addr))]>;

    def LDYix  : F00<0b101, 0b111, (outs IY16:$dst), (ins mem_ix16:$addr),
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
    def CPYzp  : F01<0b110, 0b001, (outs), (ins IY16:$src, mem_zp:$addr),
                     "CPY $addr",
                     [(C65cmp IY16:$src, (load addr_zp:$addr))]>;

    def CPYi   : F01<0b110, 0b011, (outs), (ins IY16:$src, mem_a16:$addr),
                     "CPY $addr",
                     [(C65cmp IY16:$src, (load tglobaladdr:$addr))]>;

    def CPXiz  : F01<0b111, 0b001, (outs), (ins IX16:$src, mem_zp:$addr),
                     "CPX $addr",
                     [(C65cmp IX16:$src, (load addr_zp:$addr))]>;

    def CPXi   : F01<0b111, 0b011, (outs), (ins IX16:$src, mem_a16:$addr),
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
                 (ins ACC16:$src, mem_is16:$addr),
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
  def STAis : F11<0b100, 0b000, (outs), (ins ACC16:$src, mem_is16:$addr),
                  "STA $addr",
                  [(store ACC16:$src, addr_is:$addr)]>;
}

let Defs = [P], mayLoad = 1 in {
  def LDAis  : F11<0b101, 0b000, (outs ACC16:$dst), (ins mem_is16:$addr),
                   "LDA $addr",
                   [(set ACC16:$dst, (load addr_is:$addr))]>;

  def CMPis  : F11<0b110, 0b000, (outs), (ins ACC16:$src, mem_is16:$addr),
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
  def NOP : FF<0xea, (outs), (ins), "NOP", []>;
}

let Defs = [P] in {
  def BITimm : FF<0x89, (outs), (ins i16imm:$imm), "BIT $imm",
                  []>;
  def BITi   : FF<0x24, (outs), (ins mem_a16:$addr), "BIT $addr",
                  []>;
  def BITix  : FF<0x3c, (outs), (ins mem_ix16:$addr), "BIT $addr",
                  []>;
}

// 65C02 instructions
def STZzp : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr",
               [(store (i16 0), addr_zp:$addr)]>;
def STZi  : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr",
               [(store (i16 0), tglobaladdr:$addr)]>;
def STZix : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr",
               [(store (i16 0), addr_ix:$addr)]>;

// Push and pull stack operations
let neverHasSideEffects = 1 in {
  let Defs = [P], mayLoad = 1 in {
    def PLA : FF<0x68, (outs ACC16:$dst), (ins), "PLA", []>;
    def PLP : FF<0x08, (outs CCR:$dst), (ins), "PLP", []>;
    // 65C02 instructions
    def PLX : FF<0xfa, (outs IX16:$dst), (ins), "PLX", []>;
    def PLY : FF<0x7a, (outs IY16:$dst), (ins), "PLY", []>;
    // 65C816 instructions
    def PLD : FF<0x2b, (outs ID16:$dst), (ins), "PLD", []>;
  }
  let mayStore = 1 in {
    def PHA : FF<0x48, (outs), (ins ACC16:$src), "PHA", []>;
    def PHP : FF<0x28, (outs), (ins CCR:$src), "PLP", []>;
    // 65C02 instructions
    def PHX : FF<0xda, (outs), (ins IX16:$src), "PHX", []>;
    def PHY : FF<0x5a, (outs), (ins IY16:$src), "PHY", []>;
    // 65C816 instructions
    def PHD : FF<0x0b, (outs), (ins ID16:$src), "PLD", []>;
    def PEA : FF<0xf4, (outs), (ins mem_a16:$addr), "PEA $addr", []>;
    def PER : FF<0x62, (outs), (ins brtarget8:$addr), "PER $addr", []>;
  }
  let mayLoad = 1, mayStore = 1 in {
    // 65C816 instruction
    def PEI : FF<0xd4, (outs), (ins mem_a16:$addr), "PEI $addr", []>;
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
  def JSR : FF<0x20, (outs), (ins mem_a16:$addr),
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

  def TXS : FF<0x9b, (outs IS16:$dst), (ins IX16:$src), "TXS",
               [(set IS16:$dst, IX16:$src)]>;

  def TSX : FF<0xbb, (outs IX16:$dst), (ins IS16:$src), "TSX",
               [(set IX16:$dst, IS16:$src)]>;

  // 65C816-only instructions
  def TXY : FF<0x9b, (outs IY16:$dst), (ins IX16:$src), "TXY",
               [(set IY16:$dst, IX16:$src)]>;

  def TYX : FF<0xbb, (outs IX16:$dst), (ins IY16:$src), "TYX",
               [(set IX16:$dst, IY16:$src)]>;

  def TCS : FF<0x1b, (outs IS16:$dst), (ins ACC16:$src), "TCS",
               [(set IS16:$dst, ACC16:$src)]>;

  def TSC : FF<0x3b, (outs ACC16:$dst), (ins IS16:$src), "TSC",
               [(set ACC16:$dst, IS16:$src)]>;

  def TCD : FF<0x5b, (outs ACC16:$dst), (ins ID16:$src), "TCD",
               [(set ACC16:$dst, ID16:$src)]>;

  def TDC : FF<0x7b, (outs ID16:$dst), (ins ACC16:$src), "TDC",
               [(set ID16:$dst, ACC16:$src)]>;
}

// 65C02-only instructions

def TSBzp : FF<0x04, (outs), (ins ACC16:$src, mem_zp:$addr),
               "TSB $addr", []>;
def TSBi  : FF<0x0c, (outs), (ins ACC16:$src, mem_a16:$addr),
               "TSB $addr", []>;
def TRBzp : FF<0x14, (outs), (ins ACC16:$src, mem_zp:$addr),
               "TRB $addr", []>;
def TRBi  : FF<0x1c, (outs), (ins ACC16:$src, mem_a16:$addr),
               "TRB $addr", []>;

// 65C816-only instructions

let Constraints = "$src = $dst" in {
  def XBA : FF<0xeb, (outs ACC16:$dst), (ins ACC16:$src), "XBA",
               [(set ACC16:$dst, (bswap ACC16:$src))]>;
}
let Uses = [P] in {
  def XCE : FF<0xfb, (outs), (ins), "XCE", []>;
}
def WAI : FF<0xcb, (outs), (ins), "WAI", []>;
def STP : FF<0xfb, (outs), (ins), "STP", []>;
def COP : FF<0x02, (outs), (ins i8imm:$sig), "COP $sig", []>;
def WDM : FF<0x42, (outs), (ins), "WDM", []>;
def MVP : FF<0x44, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "MVP", []>;
def MVN : FF<0x54, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "WDM", []>;

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
  def INX  : FF<0xe8, (outs IX16:$dst), (ins IX16:$src), "INX",
                [(set IX16:$dst, (add IX16:$src, 1))]>;

  def INY  : FF<0xc8, (outs IY16:$dst), (ins IY16:$src), "INY",
                [(set IY16:$dst, (add IY16:$src, 1))]>;

  def DEX  : FF<0xca, (outs IX16:$dst), (ins IX16:$src), "DEX",
                [(set IX16:$dst, (sub IX16:$src, 1))]>;

  def DEY  : FF<0x88, (outs IY16:$dst), (ins IY16:$src), "DEY",
                [(set IY16:$dst, (sub IY16:$src, 1))]>;

  // 65C02-only instructions

  def INCa : FF<0x1a, (outs ACC16:$dst), (ins ACC16:$src), "INC A",
                [(set ACC16:$dst, (add ACC16:$src, 1))]>;

  def DECa : FF<0x3a, (outs ACC16:$dst), (ins ACC16:$src), "DEC A",
                [(set ACC16:$dst, (sub ACC16:$src, 1))]>;

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

//===----------------------------------------------------------------------===//
// Register spill patterns
//===----------------------------------------------------------------------===//

//def : Pat<(and ACC16:$src, ACC16:$dst),
//          (and ACC16:$src, (C65spill ACC16:$dst))>;
//def : Pat<(or ACC16:$src, ACC16:$dst),
//          (or ACC16:$src, (C65spill ACC16:$dst))>;

//===----------------------------------------------------------------------===//
// Zero-page register operations
//===----------------------------------------------------------------------===//

class ZPInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
}

def STZz  : ZPInstr<(outs ZRC16:$dst), (ins),
                   ";STZ %dst",
                   [(set ZRC16:$dst, (i16 0))]>;

let Defs = [P] in {
  def MOVza  : ZPInstr<(outs ZRC16:$dst), (ins ACC16:$src),
                      ";MOV %dst,$src",
                      [(set ZRC16:$dst, ACC16:$src)]>;
  def MOVzx  : ZPInstr<(outs ZRC16:$dst), (ins IX16:$src),
                      ";MOV %dst,$src",
                      [(set ZRC16:$dst, IX16:$src)]>;
  def MOVzy  : ZPInstr<(outs ZRC16:$dst), (ins IY16:$src),
                      ";MOV %dst,$src",
                      [(set ZRC16:$dst, IY16:$src)]>;
}

def MOVaz  : ZPInstr<(outs ACC16:$dst), (ins ZRC16:$src),
                    ";MOV %dst,$src",
                    [(set ACC16:$dst, ZRC16:$src)]>;
def MOVxz  : ZPInstr<(outs IX16:$dst), (ins ZRC16:$src),
                    ";MOV %dst,$src",
                    [(set IX16:$dst, ZRC16:$src)]>;
def MOVyz  : ZPInstr<(outs IY16:$dst), (ins ZRC16:$src),
                    ";MOV %dst,$src",
                    [(set IY16:$dst, ZRC16:$src)]>;

let Defs = [P], Constraints = "$src = $dst" in {
  def INCz : ZPInstr<(outs ZRC16:$dst), (ins ZRC16:$src),
                    ";ADD %dst,$src,#1",
                    [(set ZRC16:$dst, (add ZRC16:$src, (i16 1)))]>;
  def DECz : ZPInstr<(outs ZRC16:$dst), (ins ZRC16:$src),
                    ";SUB %dst,$src,#1",
                    [(set ZRC16:$dst, (sub ZRC16:$src, (i16 1)))]>;
}

// let Defs = [P, A] in {
//   def MOV32zz  : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src),
//                         ";MOV $dst,$src",
//                         [(set ZRC32:$dst, ZRC32:$src)]>;
//   def ADD32zzz : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src1, ZRC32:$src2),
//                         ";ADD $dst,$src1,$src2",
//                         [(set ZRC32:$dst, (add ZRC32:$src1, ZRC32:$src2))]>;
//   def SUB32zzz : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src1, ZRC32:$src2),
//                         ";SUB $dst,$src1,$src2",
//                         [(set ZRC32:$dst, (sub ZRC32:$src1, ZRC32:$src2))]>;
//   def AND32zzz : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src1, ZRC32:$src2),
//                         ";AND $dst,$src1,$src2",
//                         [(set ZRC32:$dst, (and ZRC32:$src1, ZRC32:$src2))]>;
//   def OR32zzz  : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src1, ZRC32:$src2),
//                         ";OR $dst,$src1,$src2",
//                         [(set ZRC32:$dst, (or ZRC32:$src1, ZRC32:$src2))]>;
//   def XOR32zzz : ZPInstr<(outs ZRC32:$dst), (ins ZRC32:$src1, ZRC32:$src2),
//                         ";XOR $dst,$src1,$src2",
//                         [(set ZRC32:$dst, (xor ZRC32:$src1, ZRC32:$src2))]>;
// }

let Defs = [P] in {
  let mayLoad = 1, Constraints = "$src1 = $dst" in {
    def ADDaz : ZPInstr<(outs ACC16:$dst), (ins ACC16:$src1, ZRC16:$src2),
                       ";ADD $dst,$src1,$src2",
                       [(set ACC16:$dst, (add ACC16:$src1, ZRC16:$src2))]>;
    def SUBaz : ZPInstr<(outs ACC16:$dst), (ins ACC16:$src1, ZRC16:$src2),
                       ";SUB $dst,$src1,$src2",
                       [(set ACC16:$dst, (sub ACC16:$src1, ZRC16:$src2))]>;
    def ANDaz : ZPInstr<(outs ACC16:$dst), (ins ACC16:$src1, ZRC16:$src2),
                       ";AND $dst,$src1,$src2",
                       [(set ACC16:$dst, (and ACC16:$src1, ZRC16:$src2))]>;
    def ORaz  : ZPInstr<(outs ACC16:$dst), (ins ACC16:$src1, ZRC16:$src2),
                       ";OR $dst,$src1,$src2",
                       [(set ACC16:$dst, (or ACC16:$src1, ZRC16:$src2))]>;
    def XORaz : ZPInstr<(outs ACC16:$dst), (ins ACC16:$src1, ZRC16:$src2),
                       ";XOR $dst,$src1,$src2",
                       [(set ACC16:$dst, (or ACC16:$src1, ZRC16:$src2))]>;
  }
}


// multiclass F11_binary<bits<3> op, string asmstr, SDPatternOperator operator> {
//   // Binary operation on 16-bit accumulator
//   let mayLoad = 1 in {
//     def is : F11<op, 0b000, (outs ACC16:$dst),
//                  (ins ACC16:$src, mem_is16:$addr),
//                  !strconcat(asmstr, " $addr"),
//                  [(set ACC16:$dst,
//                     (operator ACC16:$src, (load addr_is:$addr)))]>;
//   }
// }

// let Defs = [P], Constraints = "$src = $dst" in {
//   def ORAzpr : F11_binary<0b000, "ORA", or>;
//   def ANDzpr : F11_binary<0b001, "AND", and>;
//   def EORzpr : F11_binary<0b010, "EOR", xor>;
//   def ADCzpr : F11_binary<0b011, "ADC", add>; //TODO: Change to adde!
//   def SBCzpr : F11_binary<0b111, "SBC", sub>; // sube
//   }
// }
