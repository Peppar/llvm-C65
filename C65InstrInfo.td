//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 predicates
//===----------------------------------------------------------------------===//

def Has65C02  : Predicate<"Subtarget->has65C02()">;

def Has65802  : Predicate<"Subtarget->has65802()">;

def Has65816 : Predicate<"Subtarget->has65816()">;

def Not65816 : Predicate<"!Subtarget->has65816()">;

// def In8BitMode  : Predicate<"Subtarget->is8Bit()">,
//                   AssemblerPredicate<"Mode8Bit", "8-bit mode">;

// def In16BitMode  : Predicate<"Subtarget->is16Bit()">,
//                    AssemblerPredicate<"Mode16Bit", "16-bit mode">;

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

// Program Counter Relative Addressing (6502, 65C02, 65802, 65816)
//   BRA label
def asmop_pcrel8 : AsmOperandClass {
  let Name = "asmop_pcrel8";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parsePCRel8Operand";
}
// Program Counter Relative Long Addressing (65802, 65816)
//   BRL label
def asmop_pcrel16 : AsmOperandClass {
  let Name = "asmop_pcrel16";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parsePCRel16Operand";
}
// Absolute Addressing (6502, 65C02, 65802, 65816)
//   LDA $1245 ; With DBR
//   JMP $1245 ; With PBR
def asmop_abs : AsmOperandClass {
  let Name = "asmop_abs";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsOperand";
}
// Absolute Indexed X (6502, 65C02, 65802, 65816)
//   LDA $1245,X
def asmop_absx : AsmOperandClass {
  let Name = "asmop_absx";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsXOperand";
}
// Absolute Indexed Y (6502, 65C02, 65802, 65816)
//   LDA $1245,Y
def asmop_absy : AsmOperandClass {
  let Name = "asmop_absy";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsYOperand";
}
// Absolute Indexed Indirect Addressing (Absolute X Preindexed) (65816)
//   JMP ($1000,X) ; With PBR (65C02, 65802, 65816)
//   JSR ($1000,X) ; With PBR (65802, 65816)
def asmop_abspreix : AsmOperandClass {
  let Name = "asmop_abspreix";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsPreIXOperand";
}
// Absolute Indirect Addressing (6502, 65C02, 65802)
// Direct Page Indirect (65816)
//   JMP ($1000)
def asmop_absind : AsmOperandClass {
  let Name = "asmop_absind";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsIndOperand";
}
// Absolute Indirect Long Addressing (65802, 65816)
//   JMP [$1245] ; Indirect from bank 0
def asmop_absindl : AsmOperandClass {
  let Name = "asmop_absindl";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsIndLOperand";
}
// Absolute Long Addressing (65802, 65816)
//   LDA $112233
//   JMP $112233
//   (65802: Data bank value is not propagated to bus)
def asmop_absl : AsmOperandClass {
  let Name = "asmop_absl";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsLOperand";
}
// Absolute Long Indexed, X Addressing (65802, 65816)
//   LDA $112233,X
//   (65802: Data bank value is not propagated to bus)
def asmop_absxl : AsmOperandClass {
  let Name = "asmop_absxl";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseAbsXLOperand";
}
// Zero Page Addressing (6502, 65C02)
// Direct Page Addressing (65816)
//   LDA $21
def asmop_zp : AsmOperandClass {
  let Name = "asmop_zp";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPOperand";
}
// Zero Page Indexed with X Addressing (6502, 65C02)
// Direct Page Indexed with X Addressing (65816)
//   LDA $21,X
def asmop_zpx : AsmOperandClass {
  let Name = "asmop_zpx";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPXOperand";
}
// Zero Page Indexed with Y Addressing (6502, 65C02)
// Direct Page Indexed with Y Addressing (65816)
//   LDA  $21,Y
def asmop_zpy : AsmOperandClass {
  let Name = "asmop_zpy";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPYOperand";
}
// Zero Page Indexing Indirect X Addressing
// Direct Page Indexing Indirect X Addressing
//   (Direct Page Preindexing)
//   STA ($50,X)
def asmop_zppreix : AsmOperandClass {
  let Name = "asmop_zppreix";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPPreIXOperand";
}
// Zero Page Indirect (65C02)
// Direct Page Indirect (65816)
//   STA ($10)
def asmop_zpind : AsmOperandClass {
  let Name = "asmop_zpind";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPIndOperand";
}
// Direct Page Indirect Long (65802, 65816)
//   LDA [$15]
def asmop_dpindl : AsmOperandClass {
  let Name = "asmop_dpindl";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseDPIndLOperand";
}
// Zero Page Indirect Indexed with Y Addressing (6502, 65C02, 65802)
// Direct Page Indirect Indexed with Y Addressing
//   (Direct Page Y Postindexed) (65816)
//   LDA ($15),Y
def asmop_zppostiy : AsmOperandClass {
  let Name = "asmop_zppostiy";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseZPPostIYOperand";
}
// Direct Page Indirect Long Indexed with Y Addressing
//   (Direct Page Y Postindexed Long) (65802, 65816)
//   LDA [$15],Y
//   (65802: Data bank value is not propagated to bus)
def asmop_dppostiyl : AsmOperandClass {
  let Name = "asmop_dppostiyl";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseDPPostIYLOperand";
}
// Stack Relative Addressing (65802, 65816)
//   LDA 3,S
def asmop_srel : AsmOperandClass {
  let Name = "asmop_srel";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseSRelOperand";
}
// Stack Relative Indirect Indexed, Y Addressing
//   (Stack Postindexed) (65802, 65816)
//   LDA (1,S),Y ; With DBR
def asmop_spostiy : AsmOperandClass {
  let Name = "asmop_spostiy";
  let RenderMethod = "addMemOperands";
  let ParserMethod = "parseSPostIYOperand";
}
// def asmop_iz : AsmOperandClass {
//   let Name = "asmop_iz";
// //  let RenderMethod = "printIZOperand";
// //  let ParserMethod = "parseIZOperand";
// }
// def asmop_zz : AsmOperandClass {
//   let Name = "asmop_zz";
// //  let RenderMethod = "printZZOperand";
// //  let ParserMethod = "parseZZOperand";
// }

//===----------------------------------------------------------------------===//
// C65 complex pattern definitions
//===----------------------------------------------------------------------===//

def addr_zp : ComplexPattern<iPTR, 1, "SelectAddrZP", [], []>;
def addr_i  : ComplexPattern<iPTR, 1, "SelectAddrI", [], []>;
def addr_l  : ComplexPattern<iPTR, 1, "SelectAddrL", [], []>;
def addr_rr : ComplexPattern<iPTR, 2, "SelectAddrRR", [], []>;
def addr_ri : ComplexPattern<iPTR, 2, "SelectAddrRI", [], []>;
def addr_is : ComplexPattern<iPTR, 2, "SelectAddrS", [frameindex], []>;

//===----------------------------------------------------------------------===//
// C65 operand class definitions
//===----------------------------------------------------------------------===//

def brtarget_pcrel8 : Operand<OtherVT> {
  let PrintMethod = "printPCRel8Operand";
  let DecoderMethod = "decodePCRel8Operand";
  let MIOperandInfo = (ops i8imm);
  let ParserMatchClass = asmop_pcrel8;
}
def brtarget_pcrel16 : Operand<OtherVT> {
  let PrintMethod = "printPCRel16Operand";
  let DecoderMethod = "decodePCRel16Operand";
  let MIOperandInfo = (ops i16imm);
  let ParserMatchClass = asmop_pcrel16;
}
def brtarget_abs : Operand<OtherVT> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsOperand";
  let PrintMethod = "printAbsOperand";
  let ParserMatchClass = asmop_abs;
}
def mem_abs : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsOperand";
  let PrintMethod = "printAbsOperand";
  let ParserMatchClass = asmop_abs;
}
def mem_absx : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsXOperand";
  let PrintMethod = "printAbsXOperand";
  let ParserMatchClass = asmop_absx;
}
def mem_absy : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsYOperand";
  let PrintMethod = "printAbsYOperand";
  let ParserMatchClass = asmop_absy;
}
def mem_abspreix : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodePreIXOperand";
  let PrintMethod = "printPreIXOperand";
  let ParserMatchClass = asmop_abspreix;
}
def mem_absind : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsIndOperand";
  let PrintMethod = "printAbsIndOperand";
  let ParserMatchClass = asmop_absind;
}
def mem_absindl : Operand<iPTR> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsIndLOperand";
  let PrintMethod = "printAbsIndLOperand";
  let ParserMatchClass = asmop_absindl;
}
def mem_absl : Operand<iPTR> {
  let MIOperandInfo = (ops i32imm);
  let DecoderMethod = "decodeAbsLOperand";
  let PrintMethod = "printAbsLOperand";
  let ParserMatchClass = asmop_absl;
}
def mem_absxl : Operand<iPTR> {
  let MIOperandInfo = (ops i32imm);
  let DecoderMethod = "decodeAbsXLOperand";
  let PrintMethod = "printAbsXLOperand";
  let ParserMatchClass = asmop_absxl;
}
def mem_zp : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPOperand";
  let PrintMethod = "printZPOperand";
  let ParserMatchClass = asmop_zp;
}
def mem_zpx : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPXOperand";
  let PrintMethod = "printZPXOperand";
  let ParserMatchClass = asmop_zpx;
}
def mem_zpy : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPYOperand";
  let PrintMethod = "printZPYOperand";
  let ParserMatchClass = asmop_zpy;
}
def mem_zppreix : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPPreIXOperand";
  let PrintMethod = "printZPPreIXOperand";
  let ParserMatchClass = asmop_zppreix;
}
def mem_zpind : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPIndOperand";
  let PrintMethod = "printZPIndOperand";
  let ParserMatchClass = asmop_zpind;
}
def mem_dpindl : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeDPIndLOperand";
  let PrintMethod = "printDPIndLOperand";
  let ParserMatchClass = asmop_dpindl;
}
def mem_zppostiy : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeZPPostIYOperand";
  let PrintMethod = "printZPPostIYOperand";
  let ParserMatchClass = asmop_zppostiy;
}
def mem_dppostiyl : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeDPPostIYLOperand";
  let PrintMethod = "printDPPostIYLOperand";
  let ParserMatchClass = asmop_dppostiyl;
}
def mem_srel : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeSRelOperand";
  let PrintMethod = "printSRelOperand";
  let ParserMatchClass = asmop_srel;
}
def mem_spostiy : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeSPostIYOperand";
  let PrintMethod = "printSPostIYOperand";
  let ParserMatchClass = asmop_spostiy;
}

// Zero-page register memory operands
def mem_iz : Operand<iPTR> {
//  let PrintMethod = "printMemOperandRI";
  let MIOperandInfo = (ops ZRC16, i16imm);
//  let ParserMatchClass = asmop_iz;
}
def mem_zz : Operand<iPTR> {
//  let PrintMethod = "printMemOperandRR";
  let MIOperandInfo = (ops ZRC16, ZRC16);
//  let ParserMatchClass = asmop_zz;
}

// FIXME

// def uimm6 : Operand<i32>;
// def immZExt6 : ImmLeaf<i32, [{return Imm == (Imm & 0x3f);}]>;

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

  def _8zp   : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8;

  def _8i    : F01<op, 0b011, (outs), (ins mem_abs:$addr),
                   !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;

  def _8zpy16 : F01<op, 0b100, (outs), (ins mem_zpy:$addr),
                    !strconcat(asmstr, " $addr"), []>,
                Acc8Bit, Ix16Bit, OpSize8;

  def _8ix16  : F01<op, 0b110, (outs), (ins mem_absx:$addr),
                   !strconcat(asmstr, " $addr"), []>,
                Acc8Bit, Ix16Bit, OpSize16;

  let Predicates = [Has65816] in {
    def _16imm : F01<op, 0b010, (outs), (ins i16imm:$imm),
                     !strconcat(asmstr, " $imm"), []>, Acc16Bit, OpSize16;

    def _16zp  : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                     !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8;

    def _16i   : F01<op, 0b011, (outs), (ins mem_abs:$addr),
                     !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;

    def _16zpy16 : F01<op, 0b100, (outs), (ins mem_zpy:$addr),
                      !strconcat(asmstr, " $addr"), []>,
                  Acc16Bit, Ix16Bit, OpSize8;

    def _16ix16 : F01<op, 0b110, (outs), (ins mem_absx:$addr),
                     !strconcat(asmstr, " $addr"), []>,
                  Acc16Bit, Ix16Bit, OpSize16;
  }
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
def STA_8i     : F01<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize16;
def STA_16i    : F01<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize16;
def STA_8zpy16 : F01<0b100, 0b100, (outs), (ins mem_zpy:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize8;
def STA_16zpy16 : F01<0b100, 0b100, (outs), (ins mem_zpy:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize8;
def STA_8ix16  : F01<0b100, 0b110, (outs), (ins mem_absx:$addr),
                     "STA $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def STA_16ix16 : F01<0b100, 0b110, (outs), (ins mem_absx:$addr),
                     "STA $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

def CMP_8zp    : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize8;
def CMP_16zp   : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize8;
def CMP_8imm   : F01<0b110, 0b010, (outs), (ins i8imm:$imm),
                     "CMP $imm", []>, Acc8Bit, OpSize8;
def CMP_16imm  : F01<0b110, 0b010, (outs), (ins i16imm:$imm),
                     "CMP $imm", []>, Acc16Bit, OpSize16;
def CMP_8i     : F01<0b110, 0b011, (outs), (ins mem_abs:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize16;
def CMP_16i    : F01<0b110, 0b011, (outs), (ins mem_abs:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize16;
def CMP_8zpy16 : F01<0b110, 0b100, (outs), (ins mem_zpy:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize8;
def CMP_16zpy16 : F01<0b110, 0b100, (outs), (ins mem_zpy:$addr),
                     "CMP $addr", []>, Acc16Bit, OpSize8;
def CMP_8ix16  : F01<0b110, 0b011, (outs), (ins mem_absx:$addr),
                     "CMP $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def CMP_16ix16 : F01<0b110, 0b011, (outs), (ins mem_absx:$addr),
                     "CMP $addr", []>, Acc16Bit, Ix16Bit, OpSize16;

def LDA_8zp    : F01<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize8;
def LDA_16zp   : F01<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize8;
def LDA_8imm   : F01<0b101, 0b010, (outs), (ins i8imm:$imm),
                     "LDA $imm", []>, Acc8Bit, OpSize8;
def LDA_16imm  : F01<0b101, 0b010, (outs), (ins i16imm:$imm),
                     "LDA $imm", []>, Acc16Bit, OpSize16;
def LDA_8i     : F01<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize16;
def LDA_16i    : F01<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize16;
def LDA_8zpy16 : F01<0b101, 0b100, (outs), (ins mem_zpy:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize8;
def LDA_16zpy16 : F01<0b101, 0b100, (outs), (ins mem_zpy:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize8;
def LDA_8ix16  : F01<0b101, 0b110, (outs), (ins mem_absx:$addr),
                     "LDA $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def LDA_16ix16 : F01<0b101, 0b110, (outs), (ins mem_absx:$addr),
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
  // def _8zppreix  : F10<op, 0b001, (outs), (ins mem_zppreix:$addr),
  //                   !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8;
  // def _16zppreix   : F10<op, 0b001, (outs), (ins mem_zppreix:$addr),
  //                   !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8;

  def _8zp    : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8;
  def _16zp   : F10<op, 0b001, (outs), (ins mem_zp:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8;

  def _8i     : F10<op, 0b011, (outs), (ins mem_abs:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;
  def _16i    : F10<op, 0b011, (outs), (ins mem_abs:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;

  def _8ix16  : F10<op, 0b111, (outs), (ins mem_absx:$addr),
                    !strconcat(asmstr, " $addr"), []>,
                    Acc8Bit, Ix16Bit, OpSize16;
  def _16ix16 : F10<op, 0b111, (outs), (ins mem_absx:$addr),
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
def STX_8i     : F10<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STX $addr", []>, Ix8Bit, OpSize16;
def STX_16i    : F10<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STX $addr", []>, Ix16Bit, OpSize16;

def LDX_8imm   : F10<0b101, 0b000, (outs), (ins i8imm:$imm),
                     "LDX $imm", []>, Ix8Bit, OpSize8;
def LDX_16imm  : F10<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDX $imm", []>, Ix16Bit, OpSize16;
def LDX_8zp    : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix8Bit, OpSize8;
def LDX_16zp   : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDX $addr", []>, Ix16Bit, OpSize8;
def LDX_8i     : F10<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDX $addr", []>, Ix8Bit, OpSize16;
def LDX_16i    : F10<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDX $addr", []>, Ix16Bit, OpSize16;
def LDX_8iy16  : F10<0b101, 0b111, (outs), (ins mem_absy:$addr),
                     "LDX $addr", []>, Acc8Bit, Ix16Bit, OpSize16;
def LDX_16iy16 : F10<0b101, 0b111, (outs), (ins mem_absy:$addr),
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
def STY_8i     : F00<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STY $addr", []>, Ix8Bit, OpSize16;
def STY_16i    : F00<0b100, 0b011, (outs), (ins mem_abs:$addr),
                     "STY $addr", []>, Ix16Bit, OpSize16;

def LDY_8imm   : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix8Bit, OpSize16;
def LDY_16imm  : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                     "LDY $imm", []>, Ix16Bit, OpSize16;
def LDY_8zp    : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize8;
def LDY_16zp   : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize8;
def LDY_8i     : F00<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize16;
def LDY_16i    : F00<0b101, 0b011, (outs), (ins mem_abs:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize16;
def LDY_8ix16  : F00<0b101, 0b111, (outs), (ins mem_absx:$addr),
                     "LDY $addr", []>, Ix8Bit, OpSize16;
def LDY_16ix16 : F00<0b101, 0b111, (outs), (ins mem_absx:$addr),
                     "LDY $addr", []>, Ix16Bit, OpSize16;

def CPY_8imm   : F00<0b110, 0b000, (outs), (ins i8imm:$imm),
                     "CPY $imm", []>, Ix8Bit, OpSize8;
def CPY_16imm  : F00<0b110, 0b000, (outs), (ins i16imm:$imm),
                     "CPY $imm", []>, Ix16Bit, OpSize16;
def CPY_8zp    : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix8Bit, OpSize8;
def CPY_16zp   : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                     "CPY $addr", []>, Ix16Bit, OpSize8;
def CPY_8i     : F00<0b110, 0b011, (outs), (ins mem_abs:$addr),
                     "CPY $addr", []>, Ix8Bit, OpSize16;
def CPY_16i    : F00<0b110, 0b011, (outs), (ins mem_abs:$addr),
                     "CPY $addr", []>, Ix16Bit, OpSize16;

def CPX_8imm   : F00<0b111, 0b000, (outs), (ins i8imm:$imm),
                     "CPX $imm", []>, Ix8Bit, OpSize8;
def CPX_16imm  : F00<0b111, 0b000, (outs), (ins i16imm:$imm),
                     "CPX $imm", []>, Ix16Bit, OpSize16;
def CPX_8zp    : F00<0b111, 0b001, (outs), (ins mem_abs:$addr),
                     "CPX $addr", []>, Ix8Bit, OpSize16;
def CPX_16zp   : F00<0b111, 0b001, (outs), (ins mem_abs:$addr),
                     "CPX $addr", []>, Ix16Bit, OpSize16;
def CPX_8i     : F00<0b111, 0b011, (outs), (ins mem_abs:$addr),
                     "CPX $addr", []>, Ix8Bit, OpSize16;
def CPX_16i    : F00<0b111, 0b011, (outs), (ins mem_abs:$addr),
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
  def _8is16  : F11<op, 0b000, (outs), (ins mem_srel:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize16;
  def _16is16 : F11<op, 0b000, (outs), (ins mem_srel:$addr),
                    !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize16;
}

defm ORA : F11_binary<0b000, "ORA">;

defm AND : F11_binary<0b001, "AND">;

defm EOR : F11_binary<0b010, "EOR">;

defm ADC : F11_binary<0b011, "ADC">;

defm SBC : F11_binary<0b111, "SBC">;

def STA_8is16  : F11<0b100, 0b000, (outs), (ins mem_srel:$addr),
                     "STA $addr", []>, Acc8Bit, OpSize16;

def STA_16is16 : F11<0b100, 0b000, (outs), (ins mem_srel:$addr),
                     "STA $addr", []>, Acc16Bit, OpSize16;

def LDA_8is16  : F11<0b101, 0b000, (outs), (ins mem_srel:$addr),
                     "LDA $addr", []>, Acc8Bit, OpSize16;

def LDA_16is16 : F11<0b101, 0b000, (outs), (ins mem_srel:$addr),
                     "LDA $addr", []>, Acc16Bit, OpSize16;

def CMP_8is16  : F11<0b110, 0b000, (outs), (ins mem_srel:$addr),
                     "CMP $addr", []>, Acc8Bit, OpSize16;

def CMP_16is16 : F11<0b110, 0b000, (outs), (ins mem_srel:$addr),
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
    : InstC65<(outs), (ins brtarget_pcrel8:$dst),
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
def BIT_8zp    : FF<0x24, (outs), (ins mem_abs:$addr), "BIT $addr", []>,
                 Acc8Bit, OpSize8;
def BIT_16zp   : FF<0x24, (outs), (ins mem_abs:$addr), "BIT $addr", []>,
                 Acc16Bit, OpSize8;
def BIT_8ix16  : FF<0x3c, (outs), (ins mem_absx:$addr), "BIT $addr", []>,
                 Acc8Bit, Ix16Bit, OpSize16;
def BIT_16ix16 : FF<0x3c, (outs), (ins mem_absx:$addr), "BIT $addr", []>,
                 Acc16Bit, Ix16Bit, OpSize16;

def STZ_8zp    : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc8Bit, OpSize8;
def STZ_16zp   : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>,
                 Acc16Bit, OpSize8;
def STZ_8i     : FF<0x9c, (outs), (ins mem_abs:$addr), "STZ $addr", []>,
                 Acc8Bit, OpSize16;
def STZ_16i    : FF<0x9c, (outs), (ins mem_abs:$addr), "STZ $addr", []>,
                 Acc16Bit, OpSize16;
def STZ_8ix16  : FF<0x9e, (outs), (ins mem_absx:$addr), "STZ $addr", []>,
                 Acc8Bit, Ix16Bit, OpSize16;
def STZ_16ix16 : FF<0x9e, (outs), (ins mem_absx:$addr), "STZ $addr", []>,
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

def PEA    : FF<0xf4, (outs), (ins mem_abs:$addr), "PEA $addr", []>,
             OpSize16;

def PER    : FF<0x62, (outs), (ins brtarget_pcrel8:$addr), "PER $addr", []>,
             OpSize8;

def PEI    : FF<0xd4, (outs), (ins mem_zp:$addr), "PEI $addr", []>,
             OpSize8;

def BRA    : FF<0x80, (outs), (ins brtarget_pcrel8:$addr), "BRA $addr", []>,
             OpSize8, OpPCRel {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def BRL    : FF<0x82, (outs), (ins brtarget_pcrel16:$addr), "BRL $addr", []>,
             OpSize16, OpPCRel {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JMP    : FF<0x6c, (outs), (ins brtarget_abs:$addr), "JMP $addr",
                [(br bb:$addr)]>, OpSize16 {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JSR    : FF<0x20, (outs), (ins mem_abs:$addr), "JSR $addr",
                [(C65call addr_i:$addr)]>, OpSize16 {
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
def TSB_8i   : FF<0x0c, (outs), (ins mem_abs:$addr), "TSB $addr", []>,
               Acc8Bit, OpSize16;
def TSB_16i  : FF<0x0c, (outs), (ins mem_abs:$addr), "TSB $addr", []>,
               Acc16Bit, OpSize16;

def TRB_8zp  : FF<0x04, (outs), (ins mem_zp:$addr), "TRB $addr", []>,
               Acc8Bit, OpSize8;
def TRB_16zp : FF<0x04, (outs), (ins mem_zp:$addr), "TRB $addr", []>,
               Acc16Bit, OpSize8;
def TRB_8i   : FF<0x0c, (outs), (ins mem_abs:$addr), "TRB $addr", []>,
               Acc8Bit, OpSize16;
def TRB_16i  : FF<0x0c, (outs), (ins mem_abs:$addr), "TRB $addr", []>,
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
//   ZLD
multiclass ZRI_R_M_Sized<string mnemonic, RegisterClass RC, PatFrag frag> {
  def zp   : ZRInstr<(outs RC:$reg1), (ins mem_zp:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_zp:$mem)]>;
  def i16  : ZRInstr<(outs RC:$reg1), (ins mem_abs:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_i:$mem)]>;
  def iz16 : ZRInstr<(outs RC:$reg1), (ins mem_iz:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_ri:$mem)]>;
  def zz16 : ZRInstr<(outs RC:$reg1), (ins mem_zz:$mem),
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
  defm ZLD : ZRI_R_M<"ZLD", load_frag>;
}

// Form: (outs), (ins R1, mem)
//   ZST
multiclass ZRI__RM_Sized<string mnemonic, RegisterClass RC, PatFrag frag> {
  def zp   : ZRInstr<(outs), (ins  RC:$reg1, mem_zp:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_zp:$mem)]>;
  def i16  : ZRInstr<(outs), (ins RC:$reg1, mem_abs:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_i:$mem)]>;
  def iz16 : ZRInstr<(outs), (ins RC:$reg1, mem_iz:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(frag RC:$reg1, addr_ri:$mem)]>;
  def zz16 : ZRInstr<(outs), (ins RC:$reg1, mem_zz:$mem),
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
  defm ZST : ZRI__RM<"ZST", store_frag>;
}

// Form: (outs R1), (ins R2, R3)
//   ZADD, ZSUB, ZAND, ZOR, ZXOR, ZSHL, ZLSHR
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
defm ZADD : ZRI_R_RR<"ZADD", add_frag>;

def sub_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (sub node:$src1, node:$src2))>;
defm ZSUB : ZRI_R_RR<"ZSUB", sub_frag>;

def and_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (and node:$src1, node:$src2))>;
defm ZAND : ZRI_R_RR<"ZAND", and_frag>;

def or_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                      (set node:$dst, (or node:$src1, node:$src2))>;
defm ZOR : ZRI_R_RR<"ZOR", or_frag>;

def xor_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (xor node:$src1, node:$src2))>;
defm ZXOR : ZRI_R_RR<"ZXOR", xor_frag>;

class ZSHL<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), "ZSHL $dst,$src,$amt",
            [(set Ty:$dst, (shl Ty:$src, i16:$amt))]>;

def ZSHL_8  : ZSHL<ZRC8,  i8>, ZROpSize8;
def ZSHL_16 : ZSHL<ZRC16, i16>, ZROpSize16;
def ZSHL_32 : ZSHL<ZRC32, i32>, ZROpSize32;
def ZSHL_64 : ZSHL<ZRC64, i64>, ZROpSize64;

class ZLSHR<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), "ZLSHR $dst,$src,$amt",
            [(set Ty:$dst, (srl Ty:$src, i16:$amt))]>;

def ZLSHR_8  : ZLSHR<ZRC8,  i8>, ZROpSize8;
def ZLSHR_6  : ZLSHR<ZRC16, i16>, ZROpSize16;
def ZLSHR_32 : ZLSHR<ZRC32, i32>, ZROpSize32;
def ZLSHR_64 : ZLSHR<ZRC64, i64>, ZROpSize64;

class ZMOV<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src), "ZMOV $dst,$src",
            []>;

def ZMOV_8  : ZMOV<ZRC8,  i8>, ZROpSize8;
def ZMOV_16 : ZMOV<ZRC16, i16>, ZROpSize16;
def ZMOV_32 : ZMOV<ZRC32, i32>, ZROpSize32;
def ZMOV_64 : ZMOV<ZRC64, i64>, ZROpSize64;

class ZBRCC<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs), (ins i32imm:$cc, RC:$op0, RC:$op1, brtarget_pcrel8:$dst),
            "ZBRCC $cc,$op0,$op1,$dst",
            [(C65br_cc imm:$cc, Ty:$op0, Ty:$op1, bb:$dst)]> {
  let isBranch = 1;
  let isTerminator = 1;
}

def ZBRCC_8  : ZBRCC<ZRC8,  i8>, ZROpSize8;
def ZBRCC_16 : ZBRCC<ZRC16, i16>, ZROpSize16;
def ZBRCC_32 : ZBRCC<ZRC32, i32>, ZROpSize32;
def ZBRCC_64 : ZBRCC<ZRC64, i64>, ZROpSize64;

class ZLD_imm<RegisterClass RC, ValueType Ty, Operand immOp>
  : ZRInstr<(outs RC:$dst), (ins immOp:$imm), "ZLD $dst,$imm",
            [(set Ty:$dst, imm:$imm)]> {
  let isMoveImm = 1;
}

def ZLD_8imm  : ZLD_imm<ZRC8,  i8,  i8imm>, ZROpSize8;
def ZLD_16imm : ZLD_imm<ZRC16, i16, i16imm>, ZROpSize16;
def ZLD_32imm : ZLD_imm<ZRC32, i32, i32imm>, ZROpSize32;
def ZLD_64imm : ZLD_imm<ZRC64, i64, i64imm>, ZROpSize64;
