//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 predicates
//===----------------------------------------------------------------------===//

def Has65C02  : Predicate<"Subtarget->has65C02()">;
def Has65C816 : Predicate<"Subtarget->has65C816()">;

//===----------------------------------------------------------------------===//
// C65 stuff
//===----------------------------------------------------------------------===//

// class AccSize<bits<2> val> {
//   bits<2> Value = val;
// }
// def AccSizeNC : OperandSize<0>; // Don't care for accumulator size
// def AccSize8  : OperandSize<1>; // Needs 8-bit accumulator
// def AccSize16 : OperandSize<2>; // Needs 16-bit accumulator

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

  bit ZRShift = 0;
  bit ZRCtrl = 0;
  bits<2> ZRSize = 0;
  bit is_ZRInstr = 0;
  bits<2> AccSize = 0;
  bits<2> IxSize = 0;

  let TSFlags{0}   = ZRShift;
  let TSFlags{2-1} = ZRSize;
  let TSFlags{3}   = is_ZRInstr;
  let TSFlags{4}   = ZRCtrl;
  let TSFlags{6-5} = AccSize;
  let TSFlags{8-7} = IxSize;
}

class ZRShift { bit ZRShift = 1; }
class ZRCtrl { bit ZRCtrl = 1; }

class ZROpSize8 { bits<2> ZRSize = 0; }
class ZROpSize16 { bits<2> ZRSize = 1; }
class ZROpSize32 { bits<2> ZRSize = 2; }
class ZROpSize64 { bits<2> ZRSize = 3; }

class Acc8Bit { bits<2> AccSize = 0b10; }
class Acc16Bit { bits<2> AccSize = 0b11; }
class Ix8Bit { bits<2> IxSize = 0b10; }
class Ix16Bit { bits<2> IxSize = 0b11; }

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
                   !strconcat(asmstr, " $imm"), []>, Acc8Bit;

  def _16imm : F01<op, 0b010, (outs), (ins i16imm:$imm),
                   !strconcat(asmstr, " $imm"), []>, Acc16Bit;

  def _8zp   : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit;

  def _16zp  : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc16Bit;

  def _8i    : F01<op, 0b011, (outs), (ins mem_a16:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit;

  def _16i   : F01<op, 0b011, (outs), (ins mem_a16:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc16Bit;

  def _8ix16  : F01<op, 0b110, (outs), (ins mem_ix16:$addr),
                   !strconcat(asmstr, " $addr"), []>,
                Acc8Bit, Ix16Bit;

  def _16ix16 : F01<op, 0b110, (outs), (ins mem_ix16:$addr),
                   !strconcat(asmstr, " $addr"), []>,
                Acc16Bit, Ix16Bit;

  def _8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
  def _16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
  def _16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                          !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
}

defm ORA   : F01_binary<0b000, "ORA">;

defm AND   : F01_binary<0b001, "AND">;

defm EOR   : F01_binary<0b010, "EOR">;

defm ADC   : F01_binary<0b011, "ADC">;

defm SBC   : F01_binary<0b111, "SBC">;

def STA_8zp    : F01<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STA $addr", []>, Acc8Bit;
def STA_16zp   : F01<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STA $addr", []>, Acc16Bit;
def STA_8i     : F01<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STA $addr", []>, Acc8Bit;
def STA_16i    : F01<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STA $addr", []>, Acc16Bit;
def STA_8ix16  : F01<0b100, 0b110, (outs), (ins mem_ix16:$addr),
                     "STA $addr", []>, Acc8Bit, Ix16Bit;
def STA_16ix16 : F01<0b100, 0b110, (outs), (ins mem_ix16:$addr),
                     "STA $addr", []>, Acc16Bit, Ix16Bit;
def STA_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc8Bit;
def STA_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc8Bit;
def STA_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc8Bit;
def STA_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc8Bit;
def STA_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc16Bit;
def STA_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc16Bit;
def STA_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STA $reg+$offset", []>, Acc16Bit;

def CMP_8imm   : F01<0b110, 0b010, (outs), (ins i8imm:$imm),
                     "CMP $imm", []>, Acc8Bit;
def CMP_16imm  : F01<0b110, 0b010, (outs), (ins i16imm:$imm),
                     "CMP $imm", []>, Acc16Bit;
def CMP_8zp    : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc8Bit;
def CMP_16zp   : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc16Bit;
def CMP_8i     : F01<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CMP $addr", []>, Acc8Bit;
def CMP_16i    : F01<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CMP $addr", []>, Acc16Bit;
def CMP_8ix16  : F01<0b110, 0b011, (outs), (ins mem_ix16:$addr),
                     "CMP $addr", []>, Acc8Bit, Ix16Bit;
def CMP_16ix16 : F01<0b110, 0b011, (outs), (ins mem_ix16:$addr),
                     "CMP $addr", []>, Acc16Bit, Ix16Bit;
def CMP_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc8Bit;
def CMP_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc8Bit;
def CMP_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc8Bit;
def CMP_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc8Bit;
def CMP_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc16Bit;
def CMP_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc16Bit;
def CMP_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CMP $reg+$offset", []>, Acc16Bit;

def LDA_8imm   : F01<0b101, 0b010, (outs), (ins i8imm:$imm),
                     "LDA $imm", []>, Acc8Bit;
def LDA_16imm  : F01<0b101, 0b010, (outs), (ins i16imm:$imm),
                     "LDA $imm", []>, Acc16Bit;
def LDA_8zp    : F01<0b101, 0b011, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc8Bit;
def LDA_16zp   : F01<0b101, 0b011, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc16Bit;
def LDA_8i     : F01<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDA $addr", []>, Acc8Bit;
def LDA_16i    : F01<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDA $addr", []>, Acc8Bit;
def LDA_8ix16  : F01<0b101, 0b110, (outs), (ins mem_ix16:$addr),
                     "LDA $addr", []>, Acc8Bit, Ix16Bit;
def LDA_16ix16 : F01<0b101, 0b110, (outs), (ins mem_ix16:$addr),
                     "LDA $addr", []>, Acc16Bit, Ix16Bit;
def LDA_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc8Bit;
def LDA_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc8Bit;
def LDA_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc8Bit;
def LDA_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc8Bit;
def LDA_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc16Bit;
def LDA_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc16Bit;
def LDA_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDA $reg+$offset", []>, Acc16Bit;

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
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit;
  def _16zp   : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit;

  def _8i     : F10<op, 0b011, (outs), (ins mem_a16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit;
  def _16i    : F10<op, 0b011, (outs), (ins mem_a16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit;

  def _8ix16  : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, Ix16Bit;
  def _16ix16 : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, Ix16Bit;

  def _8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc8Bit;
  def _16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
  def _16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
  def _16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                    !strconcat(asmstr, " $reg+$offset"), []>, Acc16Bit;
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
                     "STX $addr", []>, Ix8Bit;
def STX_16zp   : F10<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STX $addr", []>, Ix16Bit;
def STX_8i     : F10<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STX $addr", []>, Ix8Bit;
def STX_16i    : F10<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STX $addr", []>, Ix16Bit;
def STX_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix8Bit;
def STX_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix8Bit;
def STX_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix8Bit;
def STX_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix8Bit;
def STX_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix16Bit;
def STX_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix16Bit;
def STX_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STX $reg+$offset", []>, Ix16Bit;

def LDX_8imm   : F10<0b101, 0b000, (outs), (ins i8imm:$imm),
                     "LDX $imm", []>, Ix8Bit;
def LDX_16imm  : F10<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDX $imm", []>, Ix16Bit;
def LDX_8zp    : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix8Bit;
def LDX_16zp   : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix16Bit;
def LDX_8i     : F10<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDX $addr", []>, Ix8Bit;
def LDX_16i    : F10<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDX $addr", []>, Ix16Bit;
def LDX_8iy16  : F10<0b101, 0b111, (outs), (ins mem_iy16:$addr),
                     "LDX $addr", []>, Acc8Bit, Ix16Bit;
def LDX_16iy16 : F10<0b101, 0b111, (outs), (ins mem_iy16:$addr),
                     "LDX $addr", []>, Acc16Bit, Ix16Bit;
def LDX_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix8Bit;
def LDX_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix8Bit;
def LDX_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix8Bit;
def LDX_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix8Bit;
def LDX_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix16Bit;
def LDX_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix16Bit;
def LDX_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDX $reg+$offset", []>, Ix16Bit;

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, (jmp), sty, ldy, cpy, cpx
//===----------------------------------------------------------------------===//

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

def STY_8zp    : F00<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STY $addr", []>, Ix8Bit;
def STY_16zp   : F00<0b100, 0b001, (outs), (ins mem_zp:$addr),
                     "STY $addr", []>, Ix16Bit;
def STY_8i     : F00<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STY $addr", []>, Ix8Bit;
def STY_16i    : F00<0b100, 0b011, (outs), (ins mem_a16:$addr),
                     "STY $addr", []>, Ix16Bit;
def STY_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix8Bit;
def STY_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix8Bit;
def STY_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix8Bit;
def STY_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix8Bit;
def STY_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix16Bit;
def STY_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix16Bit;
def STY_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "STY $reg+$offset", []>, Ix16Bit;

def LDY_8imm   : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix8Bit;
def LDY_16imm  : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix16Bit;
def LDY_8zp    : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix8Bit;
def LDY_16zp   : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix16Bit;
def LDY_8i     : F00<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDY $addr", []>, Ix8Bit;
def LDY_16i    : F00<0b101, 0b011, (outs), (ins mem_a16:$addr),
                     "LDY $addr", []>, Ix16Bit;
def LDY_8ix16  : F00<0b101, 0b111, (outs), (ins mem_ix16:$addr),
                     "LDY $addr", []>, Ix8Bit;
def LDY_16ix16 : F00<0b101, 0b111, (outs), (ins mem_ix16:$addr),
                     "LDY $addr", []>, Ix16Bit;
def LDY_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix8Bit;
def LDY_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix8Bit;
def LDY_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix8Bit;
def LDY_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix8Bit;
def LDY_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix16Bit;
def LDY_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix16Bit;
def LDY_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "LDY $reg+$offset", []>, Ix16Bit;

def CPY_8imm   : F00<0b110, 0b000, (outs), (ins i8imm:$imm),
                     "CPY $imm", []>, Ix8Bit;
def CPY_16imm  : F00<0b110, 0b000, (outs), (ins i16imm:$imm),
                     "CPY $imm", []>, Ix16Bit;
def CPY_8zp    : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix8Bit;
def CPY_16zp   : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix16Bit;
def CPY_8i     : F00<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CPY $addr", []>, Ix8Bit;
def CPY_16i    : F00<0b110, 0b011, (outs), (ins mem_a16:$addr),
                     "CPY $addr", []>, Ix16Bit;
def CPY_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix8Bit;
def CPY_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix8Bit;
def CPY_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix8Bit;
def CPY_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix8Bit;
def CPY_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix16Bit;
def CPY_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix16Bit;
def CPY_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CPY $reg+$offset", []>, Ix16Bit;

def CPX_8imm   : F00<0b111, 0b000, (outs), (ins i8imm:$imm),
                     "CPX $imm", []>, Ix8Bit;
def CPX_16imm  : F00<0b111, 0b000, (outs), (ins i16imm:$imm),
                     "CPX $imm", []>, Ix16Bit;
def CPX_8zp    : F00<0b111, 0b001, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix8Bit;
def CPX_16zp   : F00<0b111, 0b001, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix16Bit;
def CPX_8i     : F00<0b111, 0b011, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix8Bit;
def CPX_16i    : F00<0b111, 0b011, (outs), (ins mem_a16:$addr),
                     "CPX $addr", []>, Ix16Bit;
def CPX_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix8Bit;
def CPX_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix8Bit;
def CPX_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix8Bit;
def CPX_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix8Bit;
def CPX_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix16Bit;
def CPX_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix16Bit;
def CPX_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                     "CPX $reg+$offset", []>, Ix16Bit;

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
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit;
  def _16is16 : F11<op, 0b000, (outs), (ins mem_is16:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit;
}

defm ORA : F11_binary<0b000, "ORA">;

defm AND : F11_binary<0b001, "AND">;

defm EOR : F11_binary<0b010, "EOR">;

defm ADC : F11_binary<0b011, "ADC">;

defm SBC : F11_binary<0b111, "SBC">;

def STA_8is16  : F11<0b100, 0b000, (outs), (ins mem_is16:$addr),
                     "STA $addr", []>, Acc8Bit;

def STA_16is16 : F11<0b100, 0b000, (outs), (ins mem_is16:$addr),
                     "STA $addr", []>, Acc16Bit;

def LDA_8is16  : F11<0b101, 0b000, (outs), (ins mem_is16:$addr),
                     "LDA $addr", []>, Acc8Bit;

def LDA_16is16 : F11<0b101, 0b000, (outs), (ins mem_is16:$addr),
                     "LDA $addr", []>, Acc16Bit;

def CMP_8is16  : F11<0b110, 0b000, (outs), (ins mem_is16:$addr),
                     "CMP $addr", []>, Acc8Bit;

def CMP_16is16 : F11<0b110, 0b000, (outs), (ins mem_is16:$addr),
                     "CMP $addr", []>, Acc16Bit;

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
              !strconcat(asmstr, " $dst"), []> {
  let Inst{7-6} = cf.flagIndex;
  let Inst{5} = value;
  let Inst{4-0} = 0b10000;
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
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

def BIT_imm  : FF<0x89, (outs), (ins i16imm:$imm), "BIT $imm", []>;

def BIT_i    : FF<0x24, (outs), (ins mem_a16:$addr), "BIT $addr", []>;

def BIT_ix   : FF<0x3c, (outs), (ins mem_ix16:$addr), "BIT $addr", []>;

def STZ_8zp    : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc8Bit;
def STZ_16zp   : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc16Bit;
def STZ_8i     : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr", []>,
                 Acc8Bit;
def STZ_16i    : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr", []>,
                 Acc16Bit;
def STZ_8ix16  : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr", []>,
                 Acc8Bit, Ix16Bit;
def STZ_16ix16 : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr", []>,
                 Acc16Bit, Ix16Bit;
def STZ_8zr8   : Pseudo<(outs), (ins ZRC8:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc8Bit;
def STZ_8zr16  : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc8Bit;
def STZ_8zr32  : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc8Bit;
def STZ_8zr64  : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc8Bit;
def STZ_16zr16 : Pseudo<(outs), (ins ZRC16:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc16Bit;
def STZ_16zr32 : Pseudo<(outs), (ins ZRC32:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc16Bit;
def STZ_16zr64 : Pseudo<(outs), (ins ZRC64:$reg, i8imm:$offset),
                    "STZ $reg+$offset", []>, Acc16Bit;

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

def PEA    : FF<0xf4, (outs), (ins mem_a16:$addr), "PEA $addr", []>;

def PER    : FF<0x62, (outs), (ins brtarget8:$addr), "PER $addr", []>;

def PEI    : FF<0xd4, (outs), (ins mem_zp:$addr), "PEI $addr", []>;

def BRA    : FF<0x80, (outs), (ins brtarget8:$addr), "BRA $addr", []> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JMP    : FF<0x6c, (outs), (ins brtarget16:$addr), "JMP $addr",
                [(br bb:$addr)]> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JSR    : FF<0x20, (outs), (ins mem_a16:$addr), "JSR $addr",
                [(C65call tglobaladdr:$addr)]> {
  let isCall = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def RTS    : FF<0x60, (outs), (ins), "RTS", [(C65ret)]> {
  let isReturn = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def TAX    : FF<0xaa, (outs), (ins), "TAX", []>;

def TAY    : FF<0xa8, (outs), (ins), "TAY", []>;

def TXA    : FF<0x8a, (outs), (ins), "TXA", []>;

def TYA    : FF<0xa8, (outs), (ins), "TYA", []>;

def TXS    : FF<0x9b, (outs), (ins), "TXS", []>;

def TSX    : FF<0xbb, (outs), (ins), "TSX", []>;

def TXY    : FF<0x9b, (outs), (ins), "TXY", []>;

def TYX    : FF<0xbb, (outs), (ins), "TYX", []>;

def TCS    : FF<0x1b, (outs), (ins), "TCS", []>;

def TSC    : FF<0x3b, (outs), (ins), "TSC", []>;

def TCD    : FF<0x5b, (outs), (ins), "TCD", []>;

def TDC    : FF<0x7b, (outs), (ins), "TDC", []>;

def TSBzp  : FF<0x04, (outs), (ins mem_zp:$addr), "TSB $addr", []>;

def TSBi   : FF<0x0c, (outs), (ins mem_a16:$addr), "TSB $addr", []>;

def TRBzp  : FF<0x14, (outs), (ins mem_zp:$addr), "TRB $addr", []>;

def TRBi   : FF<0x1c, (outs), (ins mem_a16:$addr), "TRB $addr", []>;

def XBA    : FF<0xeb, (outs), (ins), "XBA", []>;

def XCE    : FF<0xfb, (outs), (ins), "XCE", []>;

def WAI    : FF<0xcb, (outs), (ins), "WAI", []>;

def STP    : FF<0xfb, (outs), (ins), "STP", []>;

def COP    : FF<0x02, (outs), (ins i8imm:$sig), "COP $sig", []>;

def WDM    : FF<0x42, (outs), (ins), "WDM", []>;

def MVP    : FF<0x44, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "MVP", []>;

def MVN    : FF<0x54, (outs), (ins i8imm:$srcbank, i8imm:$dstbank), "WDM", []>;

def CLC    : FF<0x18, (outs), (ins), "CLC", []>;

def SEC    : FF<0x38, (outs), (ins), "STC", []>;

def CLD    : FF<0xd8, (outs), (ins), "CLD", []>;

def SED    : FF<0xf8, (outs), (ins), "STD", []>;

def CLI    : FF<0x58, (outs), (ins), "CLI", []>;

def SEI    : FF<0x78, (outs), (ins), "STI", []>;

def CLV    : FF<0xb8, (outs), (ins), "CLV", []>;

def REP    : FF<0xc2, (outs), (ins i8imm:$imm), "REP $imm", []>;

def SEP    : FF<0xe2, (outs), (ins i8imm:$imm), "SEP $imm", []>;

def INX    : FF<0xe8, (outs), (ins), "INX", []>;

def INY    : FF<0xc8, (outs), (ins), "INY", []>;

def DEX    : FF<0xca, (outs), (ins), "DEX", []>;

def DEY    : FF<0x88, (outs), (ins), "DEY", []>;

def INCa_8   : FF<0x1a, (outs), (ins), "INC A", []>, Acc8Bit;
def INCa_16  : FF<0x1a, (outs), (ins), "INC A", []>, Acc16Bit;

def DECa_8   : FF<0x3a, (outs), (ins), "DEC A", []>, Acc8Bit;
def DECa_16  : FF<0x3a, (outs), (ins), "DEC A", []>, Acc16Bit;

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

class MOVzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src), ";MOV $dst,$src",
            []>;

def MOV8zz  : MOVzz<ZRC8,  i8>, ZROpSize8;
def MOV16zz : MOVzz<ZRC16, i16>, ZROpSize16;
def MOV32zz : MOVzz<ZRC32, i32>, ZROpSize32;
def MOV64zz : MOVzz<ZRC64, i64>, ZROpSize64;

class STZz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins), ";STZ $dst",
            [(set Ty:$dst, (Ty 0))]> {
  let isMoveImm = 1;
}

def STZ8z  : STZz<ZRC8,  i8>, ZROpSize8;
def STZ16z : STZz<ZRC16, i16>, ZROpSize16;
def STZ32z : STZz<ZRC32, i32>, ZROpSize32;
def STZ64z : STZz<ZRC64, i64>, ZROpSize64;

class STzi<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs), (ins RC:$src, mem_a16:$mem), ";STORE $src,$mem",
            [(store Ty:$src, tglobaladdr:$mem)]> {
  let mayStore = 1;
}

def ST8zi  : STzi<ZRC8,  i8>, ZROpSize8;
def ST16zi : STzi<ZRC16, i16>, ZROpSize16;
def ST32zi : STzi<ZRC32, i32>, ZROpSize32;
def ST64zi : STzi<ZRC64, i64>, ZROpSize64;

class LDzi<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins mem_a16:$mem), ";LOAD $dst,$mem",
            [(set Ty:$dst, (load tglobaladdr:$mem))]> {
  let mayLoad = 1;
}

def LD8zi  : LDzi<ZRC8,  i8>, ZROpSize8;
def LD16zi : LDzi<ZRC16, i16>, ZROpSize16;
def LD32zi : LDzi<ZRC32, i32>, ZROpSize32;
def LD64zi : LDzi<ZRC64, i64>, ZROpSize64;

class LDZEXTzi<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins mem_a16:$mem), ";LOAD $dst,$mem",
            [(set Ty:$dst, (load tglobaladdr:$mem))]> {
  let mayLoad = 1;
}

class LDzimm<RegisterClass RC, ValueType Ty, Operand immOp>
  : ZRInstr<(outs RC:$dst), (ins immOp:$imm), ";LOAD $dst,$imm",
            [(set Ty:$dst, imm:$imm)]> {
  let isMoveImm = 1;
}

def LD8zimm  : LDzimm<ZRC8,  i8,  i8imm>, ZROpSize8;
def LD16zimm : LDzimm<ZRC16, i16, i16imm>, ZROpSize16;
def LD32zimm : LDzimm<ZRC32, i32, i32imm>, ZROpSize32;
def LD64zimm : LDzimm<ZRC64, i64, i64imm>, ZROpSize64;

class ANDzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";AND $dst,$src1,$src2",
            [(set Ty:$dst, (and Ty:$src1, Ty:$src2))]>;

def AND8zz  : ANDzz<ZRC8,  i8>, ZROpSize8;
def AND16zz : ANDzz<ZRC16, i16>, ZROpSize16;
def AND32zz : ANDzz<ZRC32, i32>, ZROpSize32;
def AND64zz : ANDzz<ZRC64, i64>, ZROpSize64;

class ORzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";OR $dst,$src1,$src2",
            [(set Ty:$dst, (or Ty:$src1, Ty:$src2))]>;

def OR8zz  : ORzz<ZRC8,  i8>, ZROpSize8;
def OR16zz : ORzz<ZRC16, i16>, ZROpSize16;
def OR32zz : ORzz<ZRC32, i32>, ZROpSize32;
def OR64zz : ORzz<ZRC64, i64>, ZROpSize64;

class XORzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";XOR $dst,$src1,$src2",
            [(set Ty:$dst, (xor Ty:$src1, Ty:$src2))]>;

def XOR8zz  : XORzz<ZRC8,  i8>, ZROpSize8;
def XOR16zz : XORzz<ZRC16, i16>, ZROpSize16;
def XOR32zz : XORzz<ZRC32, i32>, ZROpSize32;
def XOR64zz : XORzz<ZRC64, i64>, ZROpSize64;

class ADDzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";ADD $dst,$src1,$src2",
            [(set Ty:$dst, (add Ty:$src1, Ty:$src2))]>;

def ADD8zz  : ADDzz<ZRC8,  i8>, ZROpSize8;
def ADD16zz : ADDzz<ZRC16, i16>, ZROpSize16;
def ADD32zz : ADDzz<ZRC32, i32>, ZROpSize32;
def ADD64zz : ADDzz<ZRC64, i64>, ZROpSize64;

class SUBzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";SUB $dst,$src1,$src2",
            [(set Ty:$dst, (sub Ty:$src1, Ty:$src2))]>;

def SUB8zz  : SUBzz<ZRC8,  i8>, ZROpSize8;
def SUB16zz : SUBzz<ZRC16, i16>, ZROpSize16;
def SUB32zz : SUBzz<ZRC32, i32>, ZROpSize32;
def SUB64zz : SUBzz<ZRC64, i64>, ZROpSize64;

class SHLzimm<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, uimm6:$imm), ";SHL $dst,$src,$imm",
            [(set Ty:$dst, (shl Ty:$src, immZExt6:$imm))]>;

def SHL8zimm  : SHLzimm<ZRC8,  i8>, ZROpSize8, ZRShift;
def SHL16zimm : SHLzimm<ZRC16, i16>, ZROpSize16, ZRShift;
def SHL32zimm : SHLzimm<ZRC32, i32>, ZROpSize32, ZRShift;
def SHL64zimm : SHLzimm<ZRC64, i64>, ZROpSize64, ZRShift;

class LSHRzimm<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, uimm6:$imm), ";LSHR $dst,$src,$imm",
            [(set Ty:$dst, (srl Ty:$src, immZExt6:$imm))]>;

def LSHR8zimm  : LSHRzimm<ZRC8,  i8>, ZROpSize8, ZRShift;
def LSHR16zimm : LSHRzimm<ZRC16, i16>, ZROpSize16, ZRShift;
def LSHR32zimm : LSHRzimm<ZRC32, i32>, ZROpSize32, ZRShift;
def LSHR64zimm : LSHRzimm<ZRC64, i64>, ZROpSize64, ZRShift;

class SHLzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), ";SHL $dst,$src,$amt",
            [(set Ty:$dst, (shl Ty:$src, i16:$amt))]>;

def SHL8zz  : SHLzz<ZRC8,  i8>, ZROpSize8, ZRShift;
def SHL16zz : SHLzz<ZRC16, i16>, ZROpSize16, ZRShift;
def SHL32zz : SHLzz<ZRC32, i32>, ZROpSize32, ZRShift;
def SHL64zz : SHLzz<ZRC64, i64>, ZROpSize64, ZRShift;

class LSHRzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), ";LSHR $dst,$src,$amt",
            [(set Ty:$dst, (srl Ty:$src, i16:$amt))]>;

def LSHR8zz  : LSHRzz<ZRC8,  i8>, ZROpSize8, ZRShift;
def LSHR16zz : LSHRzz<ZRC16, i16>, ZROpSize16, ZRShift;
def LSHR32zz : LSHRzz<ZRC32, i32>, ZROpSize32, ZRShift;
def LSHR64zz : LSHRzz<ZRC64, i64>, ZROpSize64, ZRShift;

class BRCCzz<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs), (ins i32imm:$cc, RC:$op0, RC:$op1, brtarget8:$dst),
            ";BRCC $cc,$op0,$op1,$dst",
            [(C65br_cc imm:$cc, Ty:$op0, Ty:$op1, bb:$dst)]>;

def BRCC8zz  : BRCCzz<ZRC8,  i8>, ZROpSize8, ZRCtrl;
def BRCC16zz : BRCCzz<ZRC16, i16>, ZROpSize16, ZRCtrl;
def BRCC32zz : BRCCzz<ZRC32, i32>, ZROpSize32, ZRCtrl;
def BRCC64zz : BRCCzz<ZRC64, i64>, ZROpSize64, ZRCtrl;
