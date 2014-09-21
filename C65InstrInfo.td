//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 predicates
//===----------------------------------------------------------------------===//

def Has65C02  : Predicate<"Subtarget->has65C02()">;
def Has65C816 : Predicate<"Subtarget->has65C816()">;

//===----------------------------------------------------------------------===//
// C65 basic instruction formats
//===----------------------------------------------------------------------===//

// Basic instruction format
class InstC65<dag outs, dag ins, string asmstr, list<dag> pattern>
        : Instruction {
  field bits<8> Inst = 0;

  let Namespace = "C65";
  let Size = 1;

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  let DecoderNamespace = "C65";

  bits<2> AccSize = 0;
  bits<2> IxSize = 0;
  bit is_ZRInstr = 0;
  bits<2> ZRSize = 0;
  bits<2> OpSize = 0;
  bit OpPCRel    = 0;

  let TSFlags{1-0}  = AccSize;
  let TSFlags{3-2}  = IxSize;
  let TSFlags{4}    = is_ZRInstr;
  let TSFlags{6-5}  = ZRSize;
  let TSFlags{8-7}  = OpSize;
  let TSFlags{16-9} = Inst;
  let TSFlags{17}   = OpPCRel;
}

class ZROpSize8 { bits<2> ZRSize = 0; }
class ZROpSize16 { bits<2> ZRSize = 1; }
class ZROpSize32 { bits<2> ZRSize = 2; }
class ZROpSize64 { bits<2> ZRSize = 3; }

class Acc8Bit { bits<2> AccSize = 0b10; }
class Acc16Bit { bits<2> AccSize = 0b11; }
class Ix8Bit { bits<2> IxSize = 0b10; }
class Ix16Bit { bits<2> IxSize = 0b11; }

class OpSize8 { bits<2> OpSize = 1; }
class OpSize16 { bits<2> OpSize = 2; }
class OpSize24 { bits<2> OpSize = 3; }

class OpPCRel { bit OpPCRel = 1; }

// Format cc instructions
// 6502 instruction format generally follows: aaabbbcc
class Fcc<bits<3> op, bits<3> addrmode, bits<2> cc,
          dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst{7-5} = op;
  let Inst{4-2} = addrmode;
  let Inst{1-0} = cc;
}

class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
}

//===----------------------------------------------------------------------===//
// C65-specific node definitions
//===----------------------------------------------------------------------===//

def SDT_C65cmp         : SDTypeProfile<0, 2, [SDTCisInt<0>,
                                              SDTCisSameAs<0, 1>]>;
def SDT_C65br_cc       : SDTypeProfile<0, 4, [SDTCisVT<0, i32>,
                                              SDTCisSameAs<1, 2>,
                                              SDTCisVT<3, OtherVT>]>;
def SDT_C65select_cc   : SDTypeProfile<1, 4, [SDTCisSameAs<0, 1>,
                                              SDTCisSameAs<1, 2>,
                                              SDTCisVT<3, i32>,
                                              SDTCisVT<4, i32>]>;
def SDT_C65call        : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
//def SDT_C65ret         : SDTypeProfile<0, -1, [SDTCisVT<0, i16>]>;

def C65cmp             : SDNode<"C65ISD::CMP", SDT_C65cmp,
                                [SDNPOutGlue]>;
def C65br_cc           : SDNode<"C65ISD::BR_CC", SDT_C65br_cc,
                                [SDNPHasChain, SDNPInGlue]>;
def C65select_cc       : SDNode<"C65ISD::SELECT_CC", SDT_C65select_cc,
    		                [SDNPInGlue]>;
def C65call            : SDNode<"C65ISD::CALL", SDT_C65call,
                                [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                                 SDNPVariadic]>;
def C65ret             : SDNode<"C65ISD::RET", SDTNone,
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

//===----------------------------------------------------------------------===//
// C65 complex pattern definitions
//===----------------------------------------------------------------------===//

def addr_zp   : ComplexPattern<iPTR, 1, "SelectAddrZP", [], []>;
//def addr_izpy : ComplexPattern<iPTR, 2, "SelectAddrZY", [], []>;
def addr_rr   : ComplexPattern<iPTR, 2, "SelectAddrRR", [], []>;
def addr_ri   : ComplexPattern<iPTR, 2, "SelectAddrRI", [], []>;
def addr_is   : ComplexPattern<iPTR, 2, "SelectAddrS", [frameindex], []>;

//===----------------------------------------------------------------------===//
// C65 operand class definitions
//===----------------------------------------------------------------------===//

// 8-bit PC-relative branch target operand
def brtarget8 : Operand<OtherVT> {
//  let PrintMethod = "printPCRelOperand";
  let EncoderMethod = "getPCRelEncoding";
//  let DecoderMethod = "decodePCRelOperand";
//  let ParserMatchClass = PCRelAsmOperand;
//  let MIOperandInfo = (ops i8imm);
}

// 16-bit absolute branch target operand
def brtarget16 : Operand<OtherVT> {
  let PrintMethod = "printMemOperandAbs";
//  let MIOperandInfo = (ops i16imm);
}

let PrintMethod = "printMemOperandI" in {
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
let PrintMethod = "printMemOperandRI" in {
  // def mem_izy16 : Operand<iPTR> {
  //   let MIOperandInfo = (ops ZRC16, IY16);
  // }
  def mem_ix16 : Operand<iPTR> {
    let MIOperandInfo = (ops i16imm);
  }
  def mem_iy16 : Operand<iPTR> {
    let MIOperandInfo = (ops i16imm);
  }
  def mem_is16 : Operand<iPTR> {
    let MIOperandInfo = (ops i16imm);
  }
  def mem_iz16 : Operand<iPTR> {
    let MIOperandInfo = (ops ZRC16, i16imm);
  }
}
let PrintMethod = "printMemOperandRI" in {
  def mem_zz16 : Operand<iPTR> {
    let MIOperandInfo = (ops ZRC16, ZRC16);
  }
}

// FIXME

def uimm6 : Operand<i32>;
def immZExt6 : ImmLeaf<i32, [{return Imm == (Imm & 0x3f);}]>;

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F01_binary<bits<3> op, string asmstr> {
  def _8imm  : F01<op, 0b010, (outs), (ins i8imm:$imm),
                   !strconcat(asmstr, " $imm"), []>, Acc8Bit, OpSize8;

  def _16imm : F01<op, 0b010, (outs), (ins i16imm:$imm),
                   !strconcat(asmstr, " $imm"), []>, Acc16Bit, OpSize16;

  def _8zp   : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8;

  def _16zp  : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8;

  def _8i    : F01<op, 0b011, (outs), (ins mem_a16:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;

  def _16i   : F01<op, 0b011, (outs), (ins mem_a16:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;

  def _8ix16  : F01<op, 0b110, (outs), (ins mem_ix16:$addr),
                   !strconcat(asmstr, " $addr"), []>,
                Acc8Bit, Ix16Bit, OpSize16;

  def _16ix16 : F01<op, 0b110, (outs), (ins mem_ix16:$addr),
                   !strconcat(asmstr, " $addr"), []>,
                Acc16Bit, Ix16Bit, OpSize16;
}

defm ORA   : F01_binary<0b000, "ORA">;

defm AND   : F01_binary<0b001, "AND">;

defm EOR   : F01_binary<0b010, "EOR">;

defm ADC   : F01_binary<0b011, "ADC">;

defm SBC   : F01_binary<0b111, "SBC">;

def STA_8zp    : F01<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize8;
def STA_16zp   : F01<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize8;
def STA_8i     : F01<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize16;
def STA_16i    : F01<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize16;
def STA_8ix16  : F01<0b100, 0b110, (outs), (ins mem_ix16:$addr),
                     "STA $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def STA_16ix16 : F01<0b100, 0b110, (outs), (ins mem_ix16:$addr),
                     "STA $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

def CMP_8imm   : F01<0b110, 0b010, (outs), (ins i8imm:$imm),
                     "CMP $imm", []>, Acc8Bit, OpSize8;
def CMP_16imm  : F01<0b110, 0b010, (outs), (ins i16imm:$imm),
                     "CMP $imm", []>, Acc16Bit, OpSize16;
def CMP_8zp    : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize8;
def CMP_16zp   : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize8;
def CMP_8i     : F01<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize16;
def CMP_16i    : F01<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize16;
def CMP_8ix16  : F01<0b110, 0b011, (outs), (ins mem_ix16:$addr),
                     "CMP $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def CMP_16ix16 : F01<0b110, 0b011, (outs), (ins mem_ix16:$addr),
                     "CMP $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

def LDA_8imm   : F01<0b101, 0b010, (outs), (ins i8imm:$imm),
                     "LDA $imm", []>, Acc8Bit, OpSize8;
def LDA_16imm  : F01<0b101, 0b010, (outs), (ins i16imm:$imm),
                     "LDA $imm", []>, Acc16Bit, OpSize16;
def LDA_8zp    : F01<0b101, 0b011, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize8;
def LDA_16zp   : F01<0b101, 0b011, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize8;
def LDA_8i     : F01<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize16;
def LDA_16i    : F01<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize16;
def LDA_8ix16  : F01<0b101, 0b110, (outs), (ins mem_ix16:$addr),
                     "LDA $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def LDA_16ix16 : F01<0b101, 0b110, (outs), (ins mem_ix16:$addr),
                     "LDA $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc
//===----------------------------------------------------------------------===//

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F10_unary<bits<3> op, string asmstr> {
  def _8zp    : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8;
  def _16zp   : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8;

  def _8i     : F10<op, 0b011, (outs), (ins mem_a16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;
  def _16i    : F10<op, 0b011, (outs), (ins mem_a16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;

  def _8ix16  : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
                    !strconcat(asmstr, " $addr"), []>,
                    Acc8Bit, Ix16Bit, OpSize16;
  def _16ix16 : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
                    !strconcat(asmstr, " $addr"), []>,
                    Acc16Bit, Ix16Bit, OpSize16;
}

defm ASL   : F10_unary<0b000, "ASL">;

def ASL_8a   : F10<0b000, 0b010, (outs), (ins),
                    "ASL A", []>, Acc8Bit;
def ASL_16a  : F10<0b000, 0b010, (outs), (ins),
                    "ASL A", []>, Acc16Bit;

defm ROL   : F10_unary<0b001, "ROL">;

def ROL_8a   : F10<0b001, 0b010, (outs), (ins),
                    "ROL A", []>, Acc8Bit;
def ROL_16a  : F10<0b001, 0b010, (outs), (ins),
                    "ROL A", []>, Acc16Bit;

defm LSR   : F10_unary<0b010, "LSR">;

def LSR_8a   : F10<0b001, 0b010, (outs), (ins),
                "LSR A", []>, Acc8Bit;
def LSR_16a  : F10<0b001, 0b010, (outs), (ins),
                "LSR A", []>, Acc16Bit;

defm ROR   : F10_unary<0b011, "ROR">;

def ROR_8a   : F10<0b001, 0b010, (outs), (ins),
                "ROR A", []>, Acc8Bit;
def ROR_16a  : F10<0b001, 0b010, (outs), (ins),
                "ROR A", []>, Acc16Bit;

defm DEC   : F10_unary<0b110, "DEC">;

defm INC   : F10_unary<0b111, "INC">;

def STX_8zp    : F10<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STX $addr", []>, Ix8Bit, OpSize8;
def STX_16zp   : F10<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STX $addr", []>, Ix16Bit, OpSize8;
def STX_8i     : F10<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STX $addr", []>, Ix8Bit, OpSize16;
def STX_16i    : F10<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STX $addr", []>, Ix16Bit, OpSize16;

def LDX_8imm   : F10<0b101, 0b000, (outs), (ins i8imm:$imm),
                     "LDX $imm", []>, Ix8Bit, OpSize8;
def LDX_16imm  : F10<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDX $imm", []>, Ix16Bit, OpSize16;
def LDX_8zp    : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix8Bit, OpSize8;
def LDX_16zp   : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix16Bit, OpSize8;
def LDX_8i     : F10<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDX $addr", []>, Ix8Bit, OpSize16;
def LDX_16i    : F10<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDX $addr", []>, Ix16Bit, OpSize16;
def LDX_8iy16  : F10<0b101, 0b111, (outs), (ins mem_iy16:$addr),
                     "LDX $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def LDX_16iy16 : F10<0b101, 0b111, (outs), (ins mem_iy16:$addr),
                     "LDX $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, (jmp), sty, ldy, cpy, cpx
//===----------------------------------------------------------------------===//

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

def STY_8zp    : F00<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STY $addr", []>, Ix8Bit, OpSize8;
def STY_16zp   : F00<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STY $addr", []>, Ix16Bit, OpSize8;
def STY_8i     : F00<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STY $addr", []>, Ix8Bit, OpSize16;
def STY_16i    : F00<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STY $addr", []>, Ix16Bit, OpSize16;

def LDY_8imm   : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix8Bit, OpSize16;
def LDY_16imm  : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix16Bit, OpSize16;
def LDY_8zp    : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize8;
def LDY_16zp   : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize8;
def LDY_8i     : F00<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize16;
def LDY_16i    : F00<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize16;
def LDY_8ix16  : F00<0b101, 0b111, (outs), (ins mem_ix16:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize16;
def LDY_16ix16 : F00<0b101, 0b111, (outs), (ins mem_ix16:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize16;

def CPY_8imm   : F00<0b110, 0b000, (outs), (ins i8imm:$imm),
                     "CPY $imm", []>, Ix8Bit, OpSize8;
def CPY_16imm  : F00<0b110, 0b000, (outs), (ins i16imm:$imm),
                     "CPY $imm", []>, Ix16Bit, OpSize16;
def CPY_8zp    : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix8Bit, OpSize8;
def CPY_16zp   : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix16Bit, OpSize8;
def CPY_8i     : F00<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CPY $addr", []>, Ix8Bit, OpSize16;
def CPY_16i    : F00<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CPY $addr", []>, Ix16Bit, OpSize16;

def CPX_8imm   : F00<0b111, 0b000, (outs), (ins i8imm:$imm),
                     "CPX $imm", []>, Ix8Bit, OpSize8;
def CPX_16imm  : F00<0b111, 0b000, (outs), (ins i16imm:$imm),
                     "CPX $imm", []>, Ix16Bit, OpSize16;
def CPX_8zp    : F00<0b111, 0b001, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix8Bit, OpSize16;
def CPX_16zp   : F00<0b111, 0b001, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix16Bit, OpSize16;
def CPX_8i     : F00<0b111, 0b011, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix8Bit, OpSize16;
def CPX_16i    : F00<0b111, 0b011, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix16Bit, OpSize16;

//===----------------------------------------------------------------------===//
// Format 11 instructions : Extended ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F11<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b11, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F11_binary<bits<3> op, string asmstr> {
  def _8is16  : F11<op, 0b000, (outs), (ins mem_is16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;
  def _16is16 : F11<op, 0b000, (outs), (ins mem_is16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;
}

defm ORA : F11_binary<0b000, "ORA">;

defm AND : F11_binary<0b001, "AND">;

defm EOR : F11_binary<0b010, "EOR">;

defm ADC : F11_binary<0b011, "ADC">;

defm SBC : F11_binary<0b111, "SBC">;

def STA_8is16  : F11<0b100, 0b000, (outs), (ins mem_is16:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize16;

def STA_16is16 : F11<0b100, 0b000, (outs), (ins mem_is16:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize16;

def LDA_8is16  : F11<0b101, 0b000, (outs), (ins mem_is16:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize16;

def LDA_16is16 : F11<0b101, 0b000, (outs), (ins mem_is16:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize16;

def CMP_8is16  : F11<0b110, 0b000, (outs), (ins mem_is16:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize16;

def CMP_16is16 : F11<0b110, 0b000, (outs), (ins mem_is16:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize16;

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
    : InstC65<(outs), (ins brtarget8:$dst),
              !strconcat(asmstr, " $dst"), []>, OpSize8, OpPCRel {
  let Inst{7-6} = cf.flagIndex;
  let Inst{5} = value;
  let Inst{4-0} = 0b10000;
  let isBranch = 1;
  let isTerminator = 1;
}

let Uses = [P] in {
  def BPL : FC<CondNegative, 0, "BPL">;
  def BMI : FC<CondNegative, 1, "BMI">;
  def BVC : FC<CondOverflow, 0, "BVC">;
  def BVS : FC<CondOverflow, 1, "BVS">;
  def BCC : FC<CondCarry,    0, "BCC">;
  def BCS : FC<CondCarry,    1, "BCS">;
  def BNE : FC<CondZero,     0, "BNE">;
  def BEQ : FC<CondZero,     1, "BEQ">;
}

//===----------------------------------------------------------------------===//
// Free format instructions
//===----------------------------------------------------------------------===//

class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst = op;
  let hasSideEffects = 1;
}

def NOP      : FF<0xea, (outs), (ins), "NOP", []>;

def BIT_8imm   : FF<0x89, (outs), (ins i8imm:$imm), "BIT $imm", []>,
                 Acc8Bit, OpSize8;
def BIT_16imm  : FF<0x89, (outs), (ins i16imm:$imm), "BIT $imm", []>,
                 Acc16Bit, OpSize16;
def BIT_8zp    : FF<0x24, (outs), (ins mem_a16:$addr), "BIT $addr", []>,
                 Acc8Bit, OpSize8;
def BIT_16zp   : FF<0x24, (outs), (ins mem_a16:$addr), "BIT $addr", []>,
                 Acc16Bit, OpSize8;
def BIT_8ix16  : FF<0x3c, (outs), (ins mem_ix16:$addr), "BIT $addr", []>,
                 Acc8Bit, Ix16Bit, OpSize16;
def BIT_16ix16 : FF<0x3c, (outs), (ins mem_ix16:$addr), "BIT $addr", []>,
                 Acc16Bit, Ix16Bit, OpSize16;

def STZ_8zp    : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc8Bit, OpSize8;
def STZ_16zp   : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc16Bit, OpSize8;
def STZ_8i     : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr", []>,
                 Acc8Bit, OpSize16;
def STZ_16i    : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr", []>,
                 Acc16Bit, OpSize16;
def STZ_8ix16  : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr", []>,
                 Acc8Bit, Ix16Bit, OpSize16;
def STZ_16ix16 : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr", []>,
                 Acc16Bit, Ix16Bit, OpSize16;

def PLA_8  : FF<0x68, (outs), (ins), "PLA", []>, Acc8Bit;
def PLA_16 : FF<0x68, (outs), (ins), "PLA", []>, Acc16Bit;

def PLP    : FF<0x08, (outs), (ins), "PLP", []>;

def PLX_8  : FF<0xfa, (outs), (ins), "PLX", []>, Ix8Bit;
def PLX_16 : FF<0xfa, (outs), (ins), "PLX", []>, Ix16Bit;

def PLY_8  : FF<0x7a, (outs), (ins), "PLY", []>, Ix8Bit;
def PLY_16 : FF<0x7a, (outs), (ins), "PLY", []>, Ix16Bit;

def PLD    : FF<0x2b, (outs), (ins), "PLD", []>;

def PHA_8  : FF<0x48, (outs), (ins), "PHA", []>, Acc8Bit;
def PHA_16 : FF<0x48, (outs), (ins), "PHA", []>, Acc16Bit;

def PHP    : FF<0x28, (outs), (ins), "PLP", []>;

def PHX_8  : FF<0xda, (outs), (ins), "PHX", []>, Ix8Bit;
def PHX_16 : FF<0xda, (outs), (ins), "PHX", []>, Ix16Bit;

def PHY_8  : FF<0x5a, (outs), (ins), "PHY", []>, Ix8Bit;
def PHY_16 : FF<0x5a, (outs), (ins), "PHY", []>, Ix16Bit;

def PHD    : FF<0x0b, (outs), (ins), "PLD", []>;

def PEA    : FF<0xf4, (outs), (ins mem_a16:$addr), "PEA $addr", []>,
             OpSize16;

def PER    : FF<0x62, (outs), (ins brtarget8:$addr), "PER $addr", []>,
             OpSize8;

def PEI    : FF<0xd4, (outs), (ins mem_zp:$addr), "PEI $addr", []>,
             OpSize8;

def BRA    : FF<0x80, (outs), (ins brtarget8:$addr), "BRA $addr", []>,
             OpSize8, OpPCRel {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JMP    : FF<0x6c, (outs), (ins brtarget16:$addr), "JMP $addr",
                [(br bb:$addr)]>, OpSize16 {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JSR    : FF<0x20, (outs), (ins mem_a16:$addr), "JSR $addr",
                [(C65call tglobaladdr:$addr)]>, OpSize16 {
  let isCall = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def RTS    : FF<0x60, (outs), (ins), "RTS", [(C65ret)]> {
  let isReturn = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def TAX_8  : FF<0xaa, (outs), (ins), "TAX", []>, Acc8Bit, Ix8Bit;
def TAX_16 : FF<0xaa, (outs), (ins), "TAX", []>, Acc16Bit, Ix16Bit;

def TAY_8  : FF<0xa8, (outs), (ins), "TAY", []>, Acc8Bit, Ix8Bit;
def TAY_16 : FF<0xa8, (outs), (ins), "TAY", []>, Acc16Bit, Ix16Bit;

def TXA_8  : FF<0x8a, (outs), (ins), "TXA", []>, Acc8Bit, Ix8Bit;
def TXA_16 : FF<0x8a, (outs), (ins), "TXA", []>, Acc16Bit, Ix16Bit;

def TYA_8  : FF<0xa8, (outs), (ins), "TYA", []>, Acc8Bit, Ix8Bit;
def TYA_16 : FF<0xa8, (outs), (ins), "TYA", []>, Acc16Bit, Ix16Bit;

def TXS    : FF<0x9a, (outs), (ins), "TXS", []>;
def TSX    : FF<0xba, (outs), (ins), "TSX", []>;

def TXY_8  : FF<0x9b, (outs), (ins), "TXY", []>, Ix8Bit;
def TXY_16 : FF<0x9b, (outs), (ins), "TXY", []>, Ix16Bit;

def TYX_8  : FF<0xbb, (outs), (ins), "TYX", []>, Ix8Bit;
def TYX_16 : FF<0xbb, (outs), (ins), "TYX", []>, Ix16Bit;

def TCS    : FF<0x1b, (outs), (ins), "TCS", []>;
def TSC    : FF<0x3b, (outs), (ins), "TSC", []>;

def TCD    : FF<0x5b, (outs), (ins), "TCD", []>;
def TDC    : FF<0x7b, (outs), (ins), "TDC", []>;

def TSB_8zp  : FF<0x04, (outs), (ins mem_zp:$addr), "TSB $addr", []>,
               Acc8Bit, OpSize8;
def TSB_16zp : FF<0x04, (outs), (ins mem_zp:$addr), "TSB $addr", []>,
               Acc16Bit, OpSize8;
def TSB_8i   : FF<0x0c, (outs), (ins mem_a16:$addr), "TSB $addr", []>,
               Acc8Bit, OpSize16;
def TSB_16i  : FF<0x0c, (outs), (ins mem_a16:$addr), "TSB $addr", []>,
               Acc16Bit, OpSize16;

def TRB_8zp  : FF<0x04, (outs), (ins mem_zp:$addr), "TRB $addr", []>,
               Acc8Bit, OpSize8;
def TRB_16zp : FF<0x04, (outs), (ins mem_zp:$addr), "TRB $addr", []>,
               Acc16Bit, OpSize8;
def TRB_8i   : FF<0x0c, (outs), (ins mem_a16:$addr), "TRB $addr", []>,
               Acc8Bit, OpSize16;
def TRB_16i  : FF<0x0c, (outs), (ins mem_a16:$addr), "TRB $addr", []>,
               Acc16Bit, OpSize16;

def XBA    : FF<0xeb, (outs), (ins), "XBA", []>;

def XCE    : FF<0xfb, (outs), (ins), "XCE", []>;

def WAI    : FF<0xcb, (outs), (ins), "WAI", []>;

def STP    : FF<0xfb, (outs), (ins), "STP", []>;

def COP    : FF<0x02, (outs), (ins i8imm:$sig), "COP $sig", []>;

def WDM    : FF<0x42, (outs), (ins), "WDM", []>;

def MVP    : FF<0x44, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "MVP", []>,
             OpSize8;

def MVN    : FF<0x54, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "WDM", []>,
             OpSize8;

def CLC    : FF<0x18, (outs), (ins), "CLC", []>;

def SEC    : FF<0x38, (outs), (ins), "STC", []>;

def CLD    : FF<0xd8, (outs), (ins), "CLD", []>;

def SED    : FF<0xf8, (outs), (ins), "STD", []>;

def CLI    : FF<0x58, (outs), (ins), "CLI", []>;

def SEI    : FF<0x78, (outs), (ins), "STI", []>;

def CLV    : FF<0xb8, (outs), (ins), "CLV", []>;

def REP    : FF<0xc2, (outs), (ins i8imm:$imm), "REP $imm", []>,
             OpSize8;

def SEP    : FF<0xe2, (outs), (ins i8imm:$imm), "SEP $imm", []>,
             OpSize8;

def INX_8  : FF<0xe8, (outs), (ins), "INX", []>, Ix8Bit;
def INX_16 : FF<0xe8, (outs), (ins), "INX", []>, Ix16Bit;

def INY_8  : FF<0xc8, (outs), (ins), "INY", []>, Ix8Bit;
def INY_16 : FF<0xc8, (outs), (ins), "INY", []>, Ix16Bit;

def DEX_8  : FF<0xca, (outs), (ins), "DEX", []>, Ix8Bit;
def DEX_16 : FF<0xca, (outs), (ins), "DEX", []>, Ix16Bit;

def DEY_8  : FF<0x88, (outs), (ins), "DEY", []>, Ix8Bit;
def DEY_16 : FF<0x88, (outs), (ins), "DEY", []>, Ix16Bit;

def INC_8  : FF<0x1a, (outs), (ins), "INC A", []>, Acc8Bit;
def INC_16 : FF<0x1a, (outs), (ins), "INC A", []>, Acc16Bit;

def DEC_8  : FF<0x3a, (outs), (ins), "DEC A", []>, Acc8Bit;
def DEC_16 : FF<0x3a, (outs), (ins), "DEC A", []>, Acc16Bit;

//===----------------------------------------------------------------------===//
// Pseudo-instructions
//===----------------------------------------------------------------------===//

// ADJCALLSTACKDOWN, ADJCALLSTACKUP are mandatory pseudo-instructions
let Defs = [S, P], Uses = [S], isCodeGenOnly = 1 in {
  def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                                 ";!ADJCALLSTACKDOWN $amt",
                                 [(callseq_start timm:$amt)]>;
  def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                              ";!ADJCALLSTACKUP $amt1",
                              [(callseq_end timm:$amt1, timm:$amt2)]>;
}

//===----------------------------------------------------------------------===//
// Zero-page register operations
//===----------------------------------------------------------------------===//

class ZRInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  bit is_ZRInstr = 1;
  let usesCustomInserter = 1;
}

// Form: (outs R1), (ins R2)
// class ZRI_R_R_Sized<string mnemonic, RegisterClass, PatFrag frag>
//       : ZRInstr<(outs RC:$reg1), (ins RC:$reg2),
//                 !strconcat(mnemonic, ",$reg2"),
//                 [(frag RC:$reg1, RC:$reg2)]>;

// Form: (outs R1), (ins mem)
//   LD
multiclass ZRI_R_M_Sized<string mnemonic, RegisterClass RC, PatFrag frag> {
  def i16  : ZRInstr<(outs RC:$reg1), (ins mem_a16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, tglobaladdr:$mem)]>;
  def iz16 : ZRInstr<(outs RC:$reg1), (ins mem_iz16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_ri:$mem)]>;
  def zz16 : ZRInstr<(outs RC:$reg1), (ins mem_zz16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_rr:$mem)]>;
}

multiclass ZRI_R_M<string mnemonic, PatFrag frag> {
  defm _8 : ZRI_R_M_Sized<mnemonic, ZRC8, frag>, ZROpSize8;
  defm _16 : ZRI_R_M_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
  defm _32 : ZRI_R_M_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
  defm _64 : ZRI_R_M_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
}

def load_frag : PatFrag<(ops node:$dst, node:$mem),
                        (set node:$dst, (load node:$mem))>;
let mayLoad = 1 in {
  defm LDz : ZRI_R_M<"LD", load_frag>;
}

// Form: (outs), (ins R1, mem)
//   ST
multiclass ZRI__RM_Sized<string mnemonic, RegisterClass RC, PatFrag frag> {
  def i16  : ZRInstr<(outs), (ins RC:$reg1, mem_a16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, tglobaladdr:$mem)]>;
  def iz16 : ZRInstr<(outs), (ins RC:$reg1, mem_iz16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_ri:$mem)]>;
  def zz16 : ZRInstr<(outs), (ins RC:$reg1, mem_zz16:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_rr:$mem)]>;
}

multiclass ZRI__RM<string mnemonic, PatFrag frag> {
  defm _8 : ZRI__RM_Sized<mnemonic, ZRC8, frag>, ZROpSize8;
  defm _16 : ZRI__RM_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
  defm _32 : ZRI__RM_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
  defm _64 : ZRI__RM_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
}

def store_frag : PatFrag<(ops node:$src, node:$mem),
                         (store node:$src, node:$mem)>;
let mayStore = 1 in {
  defm STz : ZRI__RM<"ST", store_frag>;
}

// Form: (outs R1), (ins R2, R3)
//   ADD, SUB, AND, OR, XOR, SHL, SRL
class ZRI_R_RR_Sized<string mnemonic, RegisterClass RC, PatFrag frag>
  : ZRInstr<(outs RC:$reg1), (ins RC:$reg2, RC:$reg3),
            !strconcat(mnemonic, " $reg1,$reg2,$reg3"),
            [(frag RC:$reg1, RC:$reg2, RC:$reg3)]>;

multiclass ZRI_R_RR<string mnemonic, PatFrag frag> {
  def _8 : ZRI_R_RR_Sized<mnemonic, ZRC8, frag>, ZROpSize8;
  def _16 : ZRI_R_RR_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
  def _32 : ZRI_R_RR_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
  def _64 : ZRI_R_RR_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
}

def add_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (add node:$src1, node:$src2))>;
defm ADD : ZRI_R_RR<"ADD", add_frag>;

def sub_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (sub node:$src1, node:$src2))>;
defm SUB : ZRI_R_RR<"SUB", sub_frag>;

def and_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (and node:$src1, node:$src2))>;
defm AND : ZRI_R_RR<"SUB", and_frag>;

def or_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                      (set node:$dst, (or node:$src1, node:$src2))>;
defm OR : ZRI_R_RR<"OR", or_frag>;

def xor_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (xor node:$src1, node:$src2))>;
defm XOR : ZRI_R_RR<"XOR", xor_frag>;

def shl_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (shl node:$src1, node:$src2))>;
defm SHL : ZRI_R_RR<"SHL", shl_frag>;

def srl_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                        (set node:$dst, (srl node:$src1, node:$src2))>;
defm SRL : ZRI_R_RR<"SRL", srl_frag>;

// def STz_8i  : STz_i<ZRC8,  i8>, ZROpSize8;
// def STz_16i : STz_i<ZRC16, i16>, ZROpSize16;
// def STz_32i : STz_i<ZRC32, i32>, ZROpSize32;
// def STz_64i : STz_i<ZRC64, i64>, ZROpSize64;

// class STz_iz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs), (ins RC:$src, mem_iz16:$mem), "ST $src,$mem",
//             [(store Ty:$src, addr_ri:$mem)]> {
//   let mayStore = 1;
// }

// def STz_8iz  : STz_iz<ZRC8,  i8>, ZROpSize8;
// def STz_16iz : STz_iz<ZRC16, i16>, ZROpSize16;
// def STz_32iz : STz_iz<ZRC32, i32>, ZROpSize32;
// def STz_64iz : STz_iz<ZRC64, i64>, ZROpSize64;

// class STz_zz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs), (ins RC:$src, mem_zz16:$mem), "ST $src,$mem",
//             [(store Ty:$src, addr_rr:$mem)]> {
//   let mayLoad = 1;
// }

// def STz_8zz  : STz_zz<ZRC8,  i8>, ZROpSize8;
// def STz_16zz : STz_zz<ZRC16, i16>, ZROpSize16;
// def STz_32zz : STz_zz<ZRC32, i32>, ZROpSize32;
// def STz_64zz : STz_zz<ZRC64, i64>, ZROpSize64;

// class LDz_i<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins mem_a16:$mem), "LD $dst,$mem",
//             [(set Ty:$dst, (load tglobaladdr:$mem))]> {
//   let mayLoad = 1;
// }

// def LDz_8i  : LDz_i<ZRC8,  i8>, ZROpSize8;
// def LDz_16i : LDz_i<ZRC16, i16>, ZROpSize16;
// def LDz_32i : LDz_i<ZRC32, i32>, ZROpSize32;
// def LDz_64i : LDz_i<ZRC64, i64>, ZROpSize64;

// class LDz_iz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs), (ins RC:$src, mem_iz16:$mem), "LD $dst,$mem",
//             [(set Ty:$src, (load addr_iz:$mem))]> {
//   let mayLoad = 1;
// }

// def LDz_8iz  : LDziz<ZRC8>, ZROpSize8;
// def LDz_16iz : LDziz<ZRC16>, ZROpSize16;
// def LDz_32iz : LDziz<ZRC32>, ZROpSize32;
// def LDz_64iz : LDziz<ZRC64>, ZROpSize64;

// class LDzi<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins mem_a16:$mem), ";LOAD $dst,$mem",
//             [(set Ty:$dst, (load tglobaladdr:$mem))]> {
//   let mayLoad = 1;
// }

// class LDZEXTzi<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins mem_a16:$mem), ";LOAD $dst,$mem",
//             [(set Ty:$dst, (load tglobaladdr:$mem))]> {
//   let mayLoad = 1;
// }

// class LDzimm<RegisterClass RC, ValueType Ty, Operand immOp>
//   : ZRInstr<(outs RC:$dst), (ins immOp:$imm), ";LOAD $dst,$imm",
//             [(set Ty:$dst, imm:$imm)]> {
//   let isMoveImm = 1;
// }

// def LD8zimm  : LDzimm<ZRC8,  i8,  i8imm>, ZROpSize8;
// def LD16zimm : LDzimm<ZRC16, i16, i16imm>, ZROpSize16;
// def LD32zimm : LDzimm<ZRC32, i32, i32imm>, ZROpSize32;
// def LD64zimm : LDzimm<ZRC64, i64, i64imm>, ZROpSize64;

// class ANDzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";AND $dst,$src1,$src2",
//             [(set Ty:$dst, (and Ty:$src1, Ty:$src2))]> {
//   let isCommutable = 1;
// }

// def AND8zz  : ANDzz<ZRC8,  i8>, ZROpSize8;
// def AND16zz : ANDzz<ZRC16, i16>, ZROpSize16;
// def AND32zz : ANDzz<ZRC32, i32>, ZROpSize32;
// def AND64zz : ANDzz<ZRC64, i64>, ZROpSize64;

// class ORzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";OR $dst,$src1,$src2",
//             [(set Ty:$dst, (or Ty:$src1, Ty:$src2))]> {
//   let isCommutable = 1;
// }

// def OR8zz  : ORzz<ZRC8,  i8>, ZROpSize8;
// def OR16zz : ORzz<ZRC16, i16>, ZROpSize16;
// def OR32zz : ORzz<ZRC32, i32>, ZROpSize32;
// def OR64zz : ORzz<ZRC64, i64>, ZROpSize64;

// class XORzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";XOR $dst,$src1,$src2",
//             [(set Ty:$dst, (xor Ty:$src1, Ty:$src2))]> {
//   let isCommutable = 1;
// }

// def XOR8zz  : XORzz<ZRC8,  i8>, ZROpSize8;
// def XOR16zz : XORzz<ZRC16, i16>, ZROpSize16;
// def XOR32zz : XORzz<ZRC32, i32>, ZROpSize32;
// def XOR64zz : XORzz<ZRC64, i64>, ZROpSize64;

// class ADDzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";ADD $dst,$src1,$src2",
//             [(set Ty:$dst, (add Ty:$src1, Ty:$src2))]> {
//   let isCommutable = 1;
// }

// def ADD8zz  : ADDzz<ZRC8,  i8>, ZROpSize8;
// def ADD16zz : ADDzz<ZRC16, i16>, ZROpSize16;
// def ADD32zz : ADDzz<ZRC32, i32>, ZROpSize32;
// def ADD64zz : ADDzz<ZRC64, i64>, ZROpSize64;

// class SUBzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";SUB $dst,$src1,$src2",
//             [(set Ty:$dst, (sub Ty:$src1, Ty:$src2))]>;

// def SUB8zz  : SUBzz<ZRC8,  i8>, ZROpSize8;
// def SUB16zz : SUBzz<ZRC16, i16>, ZROpSize16;
// def SUB32zz : SUBzz<ZRC32, i32>, ZROpSize32;
// def SUB64zz : SUBzz<ZRC64, i64>, ZROpSize64;

// class SHLzimm<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, uimm6:$imm), ";SHL $dst,$src,$imm",
//             [(set Ty:$dst, (shl Ty:$src, immZExt6:$imm))]> {
// }

// def SHL8zimm  : SHLzimm<ZRC8,  i8>, ZROpSize8;
// def SHL16zimm : SHLzimm<ZRC16, i16>, ZROpSize16;
// def SHL32zimm : SHLzimm<ZRC32, i32>, ZROpSize32;
// def SHL64zimm : SHLzimm<ZRC64, i64>, ZROpSize64;

// class LSHRzimm<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, uimm6:$imm), ";LSHR $dst,$src,$imm",
//             [(set Ty:$dst, (srl Ty:$src, immZExt6:$imm))]>;

// def LSHR8zimm  : LSHRzimm<ZRC8,  i8>, ZROpSize8;
// def LSHR16zimm : LSHRzimm<ZRC16, i16>, ZROpSize16;
// def LSHR32zimm : LSHRzimm<ZRC32, i32>, ZROpSize32;
// def LSHR64zimm : LSHRzimm<ZRC64, i64>, ZROpSize64;

// class SHLzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), ";SHL $dst,$src,$amt",
//             [(set Ty:$dst, (shl Ty:$src, i16:$amt))]>;

// def SHL8zz  : SHLzz<ZRC8,  i8>, ZROpSize8;
// def SHL16zz : SHLzz<ZRC16, i16>, ZROpSize16;
// def SHL32zz : SHLzz<ZRC32, i32>, ZROpSize32;
// def SHL64zz : SHLzz<ZRC64, i64>, ZROpSize64;

// class LSHRzz<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), ";LSHR $dst,$src,$amt",
//             [(set Ty:$dst, (srl Ty:$src, i16:$amt))]>;

// def LSHR8zz  : LSHRzz<ZRC8,  i8>, ZROpSize8;
// def LSHR16zz : LSHRzz<ZRC16, i16>, ZROpSize16;
// def LSHR32zz : LSHRzz<ZRC32, i32>, ZROpSize32;
// def LSHR64zz : LSHRzz<ZRC64, i64>, ZROpSize64;

class MOVz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src), "MOV $dst,$src",
            []>;

def MOVz_8  : MOVz<ZRC8,  i8>, ZROpSize8;
def MOVz_16 : MOVz<ZRC16, i16>, ZROpSize16;
def MOVz_32 : MOVz<ZRC32, i32>, ZROpSize32;
def MOVz_64 : MOVz<ZRC64, i64>, ZROpSize64;

class STZz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins), "STZ $dst",
            [(set Ty:$dst, (Ty 0))]> {
  let isMoveImm = 1;
}

def STZz_8  : STZz<ZRC8,  i8>, ZROpSize8;
def STZz_16 : STZz<ZRC16, i16>, ZROpSize16;
def STZz_32 : STZz<ZRC32, i32>, ZROpSize32;
def STZz_64 : STZz<ZRC64, i64>, ZROpSize64;

class BRCCzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs), (ins i32imm:$cc, RC:$op0, RC:$op1, brtarget8:$dst),
            "BRCC $cc,$op0,$op1,$dst",
            [(C65br_cc imm:$cc, Ty:$op0, Ty:$op1, bb:$dst)]> {
  let isBranch = 1;
  let isTerminator = 1;
}

def BRCC8zz  : BRCCzz<ZRC8,  i8>, ZROpSize8;
def BRCC16zz : BRCCzz<ZRC16, i16>, ZROpSize16;
def BRCC32zz : BRCCzz<ZRC32, i32>, ZROpSize32;
def BRCC64zz : BRCCzz<ZRC64, i64>, ZROpSize64;
