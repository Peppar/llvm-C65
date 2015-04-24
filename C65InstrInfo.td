//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

//===----------------------------------------------------------------------===//
// C65 predicates
//===----------------------------------------------------------------------===//

def Has65C02  : Predicate<"Subtarget->has65C02()">;

def Has65802  : Predicate<"Subtarget->has65802()">;

def Has65816 : Predicate<"Subtarget->has65816()">;

def IsAcc8Bit  : Predicate<"Subtarget->isAcc8Bit()">,
                 AssemblerPredicate<"ModeAcc8Bit",
                                    "8-bit accumulator mode">;
def IsAcc16Bit : Predicate<"Subtarget->isAcc16Bit()">,
                 AssemblerPredicate<"ModeAcc16Bit",
                                    "16-bit accumulator mode">;
def IsIx8Bit   : Predicate<"Subtarget->isIx8Bit()">,
                 AssemblerPredicate<"ModeIx8Bit",
                                    "8-bit index mode">;
def IsIx16Bit  : Predicate<"Subtarget->isIx16Bit()">,
                 AssemblerPredicate<"ModeIx16Bit",
                                    "16-bit index mode">;

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

class ZROpSize8  { bits<2> ZRSize = 0; }
class ZROpSize16 { bits<2> ZRSize = 1; }
class ZROpSize32 { bits<2> ZRSize = 2; }
class ZROpSize64 { bits<2> ZRSize = 3; }

class Acc8Bit  { bits<2> AccSize = 0b10; }
class Acc16Bit { bits<2> AccSize = 0b11; }
class Ix8Bit   { bits<2> IxSize  = 0b10; }
class Ix16Bit  { bits<2> IxSize  = 0b11; }

class OpSize8  { bits<2> OpSize = 1; }
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

def SDTC65CMP         : SDTypeProfile<0, 2,  [SDTCisInt<0>,
                                              SDTCisSameAs<0, 1>]>;
def SDTC65FRAME_ADDR  : SDTypeProfile<1, 1,  [SDTCisPtrTy<0>,
                                              SDTCisVT<1, i8>]>;
def SDTC65BR_CC       : SDTypeProfile<0, 4,  [SDTCisVT<0, i32>,
                                              SDTCisSameAs<1, 2>,
                                              SDTCisVT<3, OtherVT>]>;
def SDTC65SELECT_CC   : SDTypeProfile<1, 5,  [SDTCisSameAs<0, 3>,
                                              SDTCisSameAs<0, 4>,
                                              SDTCisSameAs<1, 2>,
                                              SDTCisVT<5, i32>]>;
def SDTC65CALL        : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDTC65PUSH        : SDTypeProfile<0, 1,  []>;
def SDTC65PULL        : SDTypeProfile<1, 0,  []>;
def SDTC65Wrapper     : SDTypeProfile<1, 1,  [SDTCisSameAs<0, 1>,
                                              SDTCisPtrTy<0>]>;

def C65CMP             : SDNode<"C65ISD::CMP", SDTC65CMP,
                                [SDNPOutGlue]>;
def C65FRAME_ADDR      : SDNode<"C65ISD::FRAME_ADDR", SDTC65FRAME_ADDR>;
def C65BR_CC           : SDNode<"C65ISD::BR_CC", SDTC65BR_CC,
                                [SDNPHasChain, SDNPInGlue]>;
def C65SELECT_CC       : SDNode<"C65ISD::SELECT_CC", SDTC65SELECT_CC,
    		                [SDNPInGlue]>;
def C65CALL            : SDNode<"C65ISD::CALL", SDTC65CALL,
                                [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue,
                                 SDNPVariadic]>;
def C65RET             : SDNode<"C65ISD::RET", SDTNone,
                                [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def C65PUSH            : SDNode<"C65ISD::PUSH", SDTC65PUSH,
                                [SDNPHasChain, SDNPOutGlue]>;
def C65PULL            : SDNode<"C65ISD::PULL", SDTC65PULL,
                                [SDNPHasChain, SDNPOutGlue]>;
def C65Wrapper         : SDNode<"C65ISD::Wrapper", SDTC65Wrapper>;
def C65FarWrapper      : SDNode<"C65ISD::FarWrapper", SDTC65Wrapper>;

//  These are target-independent nodes, but have target-specific formats.
def SDTC65CALLSEQ_START : SDCallSeqStart<[SDTCisVT<0, i32>]>;
def SDTC65CALLSEQ_END   : SDCallSeqEnd<[SDTCisVT<0, i32>, SDTCisVT<1, i32>]>;

def C65CALLSEQ_START : SDNode<"ISD::CALLSEQ_START", SDTC65CALLSEQ_START,
                              [SDNPHasChain, SDNPOutGlue]>;
def C65CALLSEQ_END   : SDNode<"ISD::CALLSEQ_END",   SDTC65CALLSEQ_END,
                              [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

//===----------------------------------------------------------------------===//
// C65 assembler operand definitions
//===----------------------------------------------------------------------===//

// Immediate operand
//   LDA #123   ; 8-bit (6502, 65C02, 65802, 65816)
//   LDA #12345 ; 16-bit (65802, 65816)
def asmop_imm : AsmOperandClass {
  let Name = "asmop_imm";
  let RenderMethod = "addImmOperands";
  let ParserMethod = "parseImmOperand";
}
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

//===----------------------------------------------------------------------===//
// C65 complex pattern definitions
//===----------------------------------------------------------------------===//

def addr_zp   : ComplexPattern<iPTR, 1, "SelectAddrZP", [], []>;
def addr_abs  : ComplexPattern<iPTR, 1, "SelectAddrAbs", [], []>;
def addr_absl : ComplexPattern<iPTR, 1, "SelectAddrAbsL", [], []>;
def addr_rr   : ComplexPattern<iPTR, 2, "SelectAddrRR", [], []>;
def addr_ri   : ComplexPattern<iPTR, 2, "SelectAddrRI", [], []>;
def addr_rrf  : ComplexPattern<iPTR, 2, "SelectAddrRRF", [], []>;
def addr_rif  : ComplexPattern<iPTR, 2, "SelectAddrRIF", [], []>;
def addr_s    : ComplexPattern<iPTR, 2, "SelectAddrS", [frameindex], []>;

//===----------------------------------------------------------------------===//
// C65 operand class definitions
//===----------------------------------------------------------------------===//

// Immediate operands
def imm8 : Operand<i8> {
  let MIOperandInfo = (ops i8imm);
  let DecoderMethod = "decodeImm8Operand";
  let PrintMethod = "printImmOperand";
  let ParserMatchClass = asmop_imm;
}
def imm16 : Operand<i16> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeImm16Operand";
  let PrintMethod = "printImmOperand";
  let ParserMatchClass = asmop_imm;
}

// Branch target operands
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
def brtarget_abspreix : Operand<OtherVT> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodePreIXOperand";
  let PrintMethod = "printAbsPreIXOperand";
  let ParserMatchClass = asmop_abspreix;
}
def brtarget_absind : Operand<OtherVT> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsIndOperand";
  let PrintMethod = "printAbsIndOperand";
  let ParserMatchClass = asmop_absind;
}
def brtarget_absindl : Operand<OtherVT> {
  let MIOperandInfo = (ops i16imm);
  let DecoderMethod = "decodeAbsIndLOperand";
  let PrintMethod = "printAbsIndLOperand";
  let ParserMatchClass = asmop_absindl;
}
def brtarget_absl : Operand<OtherVT> {
  let MIOperandInfo = (ops i32imm);
  let DecoderMethod = "decodeAbsLOperand";
  let PrintMethod = "printAbsLOperand";
  let ParserMatchClass = asmop_absl;
}

// Memory operands
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
  let PrintMethod = "printAbsPreIXOperand";
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
  let MIOperandInfo = (ops i8imm, i8imm);
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
def mem_ri : Operand<iPTR> {
  let MIOperandInfo = (ops ZRC16, i16imm);
}
def mem_rr : Operand<iPTR> {
  let MIOperandInfo = (ops ZRC16, ZRC16);
}
def mem_rif : Operand<iPTR> {
  let MIOperandInfo = (ops ZRC32, i16imm);
}
def mem_rrf : Operand<iPTR> {
  let MIOperandInfo = (ops ZRC32, ZRC16);
}

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F01_noimm_sized<bits<3> op, string asmstr,
                           list<Predicate> preds = []> {
  let Predicates = preds in {
    def zp         : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                         !strconcat(asmstr, " $addr"), []>, OpSize8;
    def abs        : F01<op, 0b011, (outs), (ins mem_abs:$addr),
                         !strconcat(asmstr, " $addr"), []>, OpSize16;
  }
  let Predicates = !listconcat(preds, [IsIx8Bit]) in {
    def zppreix8   : F01<op, 0b000, (outs), (ins mem_zppreix:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
    def zppostiy8  : F01<op, 0b100, (outs), (ins mem_zppostiy:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
    def zpx8       : F01<op, 0b101, (outs), (ins mem_zpx:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
    def absy8      : F01<op, 0b110, (outs), (ins mem_absy:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16;
    def absx8      : F01<op, 0b111, (outs), (ins mem_absx:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16;
  }
  let Predicates = !listconcat(preds, [IsIx16Bit]) in {
    def zppreix16  : F01<op, 0b000, (outs), (ins mem_zppreix:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
    def zppostiy16 : F01<op, 0b100, (outs), (ins mem_zppostiy:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
    def zpx16      : F01<op, 0b101, (outs), (ins mem_zpx:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
    def absy16     : F01<op, 0b110, (outs), (ins mem_absy:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16;
    def absx16     : F01<op, 0b111, (outs), (ins mem_absx:$addr),
                         !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16;
  }
}

multiclass F01_noimm<bits<3> op, string asmstr> {
  defm NAME#8 : F01_noimm_sized<op, asmstr, [IsAcc8Bit]>, Acc8Bit;
  defm NAME#16 : F01_noimm_sized<op, asmstr, [IsAcc16Bit]>, Acc16Bit;
}

multiclass F01_all<bits<3> op, string asmstr>
             : F01_noimm<op, asmstr> {
  def NAME#8imm : F01<op, 0b010, (outs), (ins imm8:$imm),
                      !strconcat(asmstr, " $imm"), []>, Acc8Bit, OpSize8,
                    Requires<[IsAcc8Bit]>;
  def NAME#16imm : F01<op, 0b010, (outs), (ins imm16:$imm),
                       !strconcat(asmstr, " $imm"), []>, Acc16Bit, OpSize16,
                     Requires<[IsAcc16Bit]>;
}

defm ORA : F01_all<0b000, "ora">;
defm AND : F01_all<0b001, "and">;
defm EOR : F01_all<0b010, "eor">;
defm ADC : F01_all<0b011, "adc">;
defm STA : F01_noimm<0b100, "sta">;
defm LDA : F01_all<0b101, "lda">;
defm CMP : F01_all<0b110, "cmp">;
defm SBC : F01_all<0b111, "sbc">;

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc
//===----------------------------------------------------------------------===//

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F10_acc<bits<3> op, string asmstr> {
  def NAME#8acc : F10<op, 0b010, (outs), (ins),
                      !strconcat(asmstr, "a"), []>, Acc8Bit,
                    Requires<[IsAcc8Bit]>;
  def NAME#16acc : F10<op, 0b010, (outs), (ins),
                       !strconcat(asmstr, "a"), []>, Acc16Bit,
                     Requires<[IsAcc16Bit]>;
}

multiclass F10_zp_abs_sized<bits<3> op, string asmstr> {
  def zp : F10<op, 0b001, (outs), (ins mem_zp:$addr),
               !strconcat(asmstr, " $addr"), []>, OpSize8;
  def abs : F10<op, 0b011, (outs), (ins mem_abs:$addr),
                !strconcat(asmstr, " $addr"), []>, OpSize16;
}

multiclass F10_zp_abs<bits<3> op, string asmstr> {
  let Predicates = [IsAcc8Bit] in
    defm NAME#8 : F10_zp_abs_sized<op, asmstr>, Acc8Bit;
  let Predicates = [IsAcc16Bit] in
    defm NAME#16 : F10_zp_abs_sized<op, asmstr>, Acc16Bit;
}

multiclass F10_zpx_absx_sized<bits<3> op, string asmstr,
                             list<Predicate> preds> {
  let Predicates = !listconcat(preds, [IsIx8Bit]) in {
    def zpx8 : F10<op, 0b101, (outs), (ins mem_zpx:$addr),
                   !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
    def absx8 : F10<op, 0b111, (outs), (ins mem_absx:$addr),
                    !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16;
  }
  let Predicates = !listconcat(preds, [IsIx16Bit]) in {
    def zpx16 : F10<op, 0b101, (outs), (ins mem_zpx:$addr),
                    !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
    def absx16 : F10<op, 0b111, (outs), (ins mem_absx:$addr),
                     !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16;
  }
}

multiclass F10_zpx_absx<bits<3> op, string asmstr> {
  defm NAME#8 : F10_zpx_absx_sized<op, asmstr, [IsAcc8Bit]>, Acc8Bit;
  defm NAME#16 : F10_zpx_absx_sized<op, asmstr, [IsAcc16Bit]>, Acc16Bit;
}

multiclass F10_zpy_sized<bits<3> op, string asmstr,
                         list<Predicate> preds> {
  let Predicates = !listconcat(preds, [IsIx8Bit]) in
    def zpy8 : F10<op, 0b101, (outs), (ins mem_zpy:$addr),
                   !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
  let Predicates = !listconcat(preds, [IsIx16Bit]) in
    def zpy16 : F10<op, 0b101, (outs), (ins mem_zpy:$addr),
                    !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
}

multiclass F10_zpy<bits<3> op, string asmstr> {
  defm NAME#8 : F10_zpy_sized<op, asmstr, [IsAcc8Bit]>, Acc8Bit;
  defm NAME#16 : F10_zpy_sized<op, asmstr, [IsAcc16Bit]>, Acc16Bit;
}

multiclass F10_absy_sized<bits<3> op, string asmstr,
                          list<Predicate> preds> {
  let Predicates = !listconcat(preds, [IsIx8Bit]) in
    def absy8 : F10<op, 0b111, (outs), (ins mem_absy:$addr),
                    !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16;
  let Predicates = !listconcat(preds, [IsIx16Bit]) in
    def absy16 : F10<op, 0b111, (outs), (ins mem_absy:$addr),
                    !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16;
}

multiclass F10_absy<bits<3> op, string asmstr> {
  defm NAME#8 : F10_absy_sized<op, asmstr, [IsAcc8Bit]>, Acc8Bit;
  defm NAME#16 : F10_absy_sized<op, asmstr, [IsAcc16Bit]>, Acc16Bit;
}

defm ASL : F10_acc<0b000, "asl">;
defm ASL : F10_zp_abs<0b000, "asl">;
defm ASL : F10_zpx_absx<0b000, "asl">;

defm ROL : F10_acc<0b001, "rol">;
defm ROL : F10_zp_abs<0b001, "rol">;
defm ROL : F10_zpx_absx<0b001, "rol">;

defm LSR : F10_acc<0b010, "lsr">;
defm LSR : F10_zp_abs<0b010, "lsr">;
defm LSR : F10_zpx_absx<0b010, "lsr">;

defm ROR : F10_acc<0b011, "ror">;
defm ROR : F10_zp_abs<0b011, "ror">;
defm ROR : F10_zpx_absx<0b011, "ror">;

defm STX : F10_zp_abs<0b100, "stx">;
defm STX : F10_zpy<0b100, "stx">;

def LDX8imm : F10<0b101, 0b000, (outs), (ins imm8:$imm),
                  "ldx $imm", []>, Ix8Bit, OpSize8,
                Requires<[IsIx8Bit]>;
def LDX16imm : F10<0b101, 0b000, (outs), (ins imm16:$imm),
                   "ldx $imm", []>, Ix16Bit, OpSize16,
                 Requires<[IsIx16Bit]>;
defm LDX : F10_zp_abs<0b101, "ldx">;
defm LDX : F10_zpy<0b101, "ldx">;
defm LDX : F10_absy<0b101, "ldx">;

defm DEC : F10_zp_abs<0b110, "dec">;
defm DEC : F10_zpx_absx<0b110, "dec">;

defm INC : F10_zp_abs<0b111, "inc">;
defm INC : F10_zpx_absx<0b111, "inc">;

// 65C02 added addressing modes for ora, and, eor, adc, sta, lda, cmp, sbc

multiclass F10_zpind<bits<3> op, string asmstr> {
  def NAME#8zpind : F10<op, 0b100, (outs), (ins mem_zpind:$addr),
                        !strconcat(asmstr, " $addr"), []>, Acc8Bit, OpSize8,
                      Requires<[Has65C02, IsAcc8Bit]>;
  def NAME#16zpind : F10<op, 0b100, (outs), (ins mem_zpind:$addr),
                         !strconcat(asmstr, " $addr"), []>, Acc16Bit, OpSize8,
                       Requires<[Has65C02, IsAcc16Bit]>;
}

defm ORA : F10_zpind<0b000, "ora">;
defm AND : F10_zpind<0b001, "and">;
defm EOR : F10_zpind<0b010, "eor">;
defm ADC : F10_zpind<0b011, "adc">;
defm STA : F10_zpind<0b100, "sta">;
defm LDA : F10_zpind<0b101, "lda">;
defm CMP : F10_zpind<0b110, "cmp">;
defm SBC : F10_zpind<0b111, "sbc">;

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, (jmp), sty, ldy, cpy, cpx
//===----------------------------------------------------------------------===//

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F00_imm<bits<3> op, string asmstr> {
  def NAME#8imm : F00<op, 0b000, (outs), (ins imm8:$imm),
                      !strconcat(asmstr, " $imm"), []>, Ix8Bit, OpSize8,
                    Requires<[IsIx8Bit]>;
  def NAME#16imm : F00<op, 0b000, (outs), (ins imm16:$imm),
                       !strconcat(asmstr, " $imm"), []>, Ix16Bit, OpSize16,
                     Requires<[IsIx16Bit]>;
}

multiclass F00_zp_abs<bits<3> op, string asmstr> {
  def NAME#8zp : F00<op, 0b001, (outs), (ins mem_zp:$addr),
                     !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8,
                   Requires<[IsIx8Bit]>;
  def NAME#8abs : F00<op, 0b011, (outs), (ins mem_abs:$addr),
                      !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16,
                    Requires<[IsIx8Bit]>;
  def NAME#16zp : F00<op, 0b001, (outs), (ins mem_zp:$addr),
                      !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8,
                    Requires<[IsIx16Bit]>;
  def NAME#16abs : F00<op, 0b011, (outs), (ins mem_abs:$addr),
                       !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16,
                     Requires<[IsIx16Bit]>;
}

multiclass F00_zpx<bits<3> op, string asmstr> {
  def NAME#8zpx : F00<op, 0b101, (outs), (ins mem_zpx:$addr),
                      !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8,
                    Requires<[IsIx8Bit]>;
  def NAME#16zpx : F00<op, 0b101, (outs), (ins mem_zpx:$addr),
                       !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8,
                     Requires<[IsIx16Bit]>;
}

multiclass F00_absx<bits<3> op, string asmstr> {
  def NAME#8absx : F00<op, 0b111, (outs), (ins mem_absx:$addr),
                       !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize16,
                     Requires<[IsIx8Bit]>;
  def NAME#16absx : F00<op, 0b111, (outs), (ins mem_absx:$addr),
                        !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize16,
                      Requires<[IsIx16Bit]>;
}

// The F00 multiclasses above are only for index registers. Since BIT
// uses the accumulator, we cannot use them.
def BIT8zp : F00<0b001, 0b001, (outs), (ins mem_zp:$addr),
                 "bit $addr", []>, Acc8Bit, OpSize8,
               Requires<[IsAcc8Bit]>;
def BIT16zp : F00<0b001, 0b001, (outs), (ins mem_zp:$addr),
                  "bit $addr", []>, Acc16Bit, OpSize8,
                Requires<[IsAcc16Bit]>;
def BIT8abs : F00<0b001, 0b011, (outs), (ins mem_abs:$addr),
                  "bit $addr", []>, Acc8Bit, OpSize16,
                Requires<[IsAcc8Bit]>;
def BIT16abs : F00<0b001, 0b011, (outs), (ins mem_abs:$addr),
                    "bit $addr", []>, Acc16Bit, OpSize16,
                 Requires<[IsAcc16Bit]>;

defm STY : F00_zp_abs<0b100, "sty">;
defm STY : F00_zpx<0b100, "sty">;

defm LDY : F00_imm<0b101, "ldy">;
defm LDY : F00_zp_abs<0b101, "ldy">;
defm LDY : F00_zpx<0b101, "ldy">;
defm LDY : F00_absx<0b101, "ldy">;

defm CPY : F00_imm<0b110, "cpy">;
defm CPY : F00_zp_abs<0b110, "cpy">;

defm CPX : F00_imm<0b111, "cpx">;
defm CPX : F00_zp_abs<0b111, "cpx">;

//===----------------------------------------------------------------------===//
// Format 11 instructions : Extended ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F11<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b11, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F11_binary_sized<bits<3> op, string asmstr,
                            list<Predicate> preds> {
  let Predicates = preds in
    def srel        : F11<op, 0b000, (outs), (ins mem_srel:$addr),
                          !strconcat(asmstr, " $addr"), []>, OpSize8;
  let Predicates = !listconcat(preds, [IsIx8Bit]) in
    def spostiy8    : F11<op, 0b100, (outs), (ins mem_spostiy:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
  let Predicates = !listconcat(preds, [IsIx16Bit]) in
    def spostiy16   : F11<op, 0b100, (outs), (ins mem_spostiy:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
  let Predicates = !listconcat(preds, [Has65816]) in {
    def absl        : F11<op, 0b011, (outs), (ins mem_absl:$addr),
                          !strconcat(asmstr, " $addr"), []>, OpSize24;
    def dpindl      : F11<op, 0b001, (outs), (ins mem_dpindl:$addr),
                          !strconcat(asmstr, " $addr"), []>, OpSize8;
  }
  let Predicates = !listconcat(preds, [Has65816, IsIx8Bit]) in {
    def dppostiyl8  : F11<op, 0b101, (outs), (ins mem_dppostiyl:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize8;
    def absxl8      : F11<op, 0b111, (outs), (ins mem_absxl:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix8Bit, OpSize24;
  }
  let Predicates = !listconcat(preds, [Has65816, IsIx16Bit]) in {
    def dppostiyl16 : F11<op, 0b101, (outs), (ins mem_dppostiyl:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize8;
    def absxl16     : F11<op, 0b111, (outs), (ins mem_absxl:$addr),
                          !strconcat(asmstr, " $addr"), []>, Ix16Bit, OpSize24;
  }
}

multiclass F11_binary<bits<3> op, string asmstr> {
  defm NAME#8 : F11_binary_sized<op, asmstr, [Has65802, IsAcc8Bit]>, Acc8Bit;
  defm NAME#16 : F11_binary_sized<op, asmstr, [Has65802, IsAcc16Bit]>, Acc16Bit;
}

defm ORA : F11_binary<0b000, "ora">;
defm AND : F11_binary<0b001, "and">;
defm EOR : F11_binary<0b010, "eor">;
defm ADC : F11_binary<0b011, "adc">;
defm STA : F11_binary<0b100, "sta">;
defm LDA : F11_binary<0b101, "lda">;
defm CMP : F11_binary<0b110, "cmp">;
defm SBC : F11_binary<0b111, "sbc">;

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
  def BPL : FC<CondNegative, 0, "bpl">;
  def BMI : FC<CondNegative, 1, "bmi">;
  def BVC : FC<CondOverflow, 0, "bvc">;
  def BVS : FC<CondOverflow, 1, "bvs">;
  def BCC : FC<CondCarry,    0, "bcc">;
  def BCS : FC<CondCarry,    1, "bcs">;
  def BNE : FC<CondZero,     0, "bne">;
  def BEQ : FC<CondZero,     1, "beq">;
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

def NOP : FF<0xea, (outs), (ins), "nop", []>;

def BIT8imm     : FF<0x89, (outs), (ins imm8:$imm), "bit $imm", []>,
                    Acc8Bit, OpSize8,
                    Requires<[Has65C02, IsAcc8Bit]>;
def BIT16imm    : FF<0x89, (outs), (ins imm16:$imm), "bit $imm", []>,
                    Acc16Bit, OpSize16,
                    Requires<[Has65C02, IsAcc16Bit]>;
def BIT8zpx8    : FF<0x34, (outs), (ins mem_zpx:$addr), "bit $addr", []>,
                    Acc8Bit, Ix8Bit, OpSize8,
                    Requires<[Has65C02, IsAcc8Bit, IsIx8Bit]>;
def BIT8zpx16   : FF<0x34, (outs), (ins mem_zpx:$addr), "bit $addr", []>,
                    Acc8Bit, Ix16Bit, OpSize8,
                    Requires<[Has65C02, IsAcc8Bit, IsIx16Bit]>;
def BIT16zpx8   : FF<0x34, (outs), (ins mem_zpx:$addr), "bit $addr", []>,
                    Acc16Bit, Ix8Bit, OpSize8,
                    Requires<[Has65C02, IsAcc16Bit, IsIx8Bit]>;
def BIT16zpx16  : FF<0x34, (outs), (ins mem_zpx:$addr), "bit $addr", []>,
                    Acc16Bit, Ix16Bit, OpSize8,
                    Requires<[Has65C02, IsAcc16Bit, IsIx16Bit]>;
def BIT8absx8   : FF<0x3c, (outs), (ins mem_absx:$addr), "bit $addr", []>,
                    Acc8Bit, Ix8Bit, OpSize16,
                    Requires<[Has65C02, IsAcc8Bit, IsIx8Bit]>;
def BIT8absx16  : FF<0x3c, (outs), (ins mem_absx:$addr), "bit $addr", []>,
                    Acc8Bit, Ix16Bit, OpSize16,
                    Requires<[Has65C02, IsAcc8Bit, IsIx16Bit]>;
def BIT16absx8  : FF<0x3c, (outs), (ins mem_absx:$addr), "bit $addr", []>,
                    Acc16Bit, Ix8Bit, OpSize16,
                    Requires<[Has65C02, IsAcc16Bit, IsIx8Bit]>;
def BIT16absx16 : FF<0x3c, (outs), (ins mem_absx:$addr), "bit $addr", []>,
                    Acc16Bit, Ix16Bit, OpSize16,
                    Requires<[Has65C02, IsAcc16Bit, IsIx16Bit]>;
def STZ8zp      : FF<0x64, (outs), (ins mem_zp:$addr), "stz $addr", []>,
                    Acc8Bit, OpSize8,
                    Requires<[IsAcc8Bit]>;
def STZ8abs     : FF<0x9c, (outs), (ins mem_abs:$addr), "stz $addr", []>,
                    Acc8Bit, OpSize16,
                    Requires<[IsAcc8Bit]>;
def STZ16zp     : FF<0x64, (outs), (ins mem_zp:$addr), "stz $addr", []>,
                    Acc16Bit, OpSize8,
                    Requires<[IsAcc16Bit]>;
def STZ16abs    : FF<0x9c, (outs), (ins mem_abs:$addr), "stz $addr", []>,
                    Acc16Bit, OpSize16,
                    Requires<[IsAcc16Bit]>;
def STZ8zpx8    : FF<0x74, (outs), (ins mem_zpx:$addr), "stz $addr", []>,
                    Acc8Bit, Ix8Bit, OpSize8,
                    Requires<[Has65802, IsAcc8Bit, IsIx8Bit]>;
def STZ8zpx16   : FF<0x74, (outs), (ins mem_zpx:$addr), "stz $addr", []>,
                    Acc8Bit, Ix16Bit, OpSize8,
                    Requires<[Has65802, IsAcc8Bit, IsIx16Bit]>;
def STZ16zpx8   : FF<0x74, (outs), (ins mem_zpx:$addr), "stz $addr", []>,
                    Acc16Bit, Ix8Bit, OpSize8,
                    Requires<[Has65802, IsAcc16Bit, IsIx8Bit]>;
def STZ16zpx16  : FF<0x74, (outs), (ins mem_zpx:$addr), "stz $addr", []>,
                    Acc16Bit, Ix16Bit, OpSize8,
                    Requires<[Has65802, IsAcc16Bit, IsIx16Bit]>;
def STZ8absx8   : FF<0x9e, (outs), (ins mem_absx:$addr), "stz $addr", []>,
                    Acc8Bit, Ix8Bit, OpSize16,
                    Requires<[Has65802, IsAcc8Bit, IsIx8Bit]>;
def STZ8absx16  : FF<0x9e, (outs), (ins mem_absx:$addr), "stz $addr", []>,
                    Acc8Bit, Ix16Bit, OpSize16,
                    Requires<[Has65802, IsAcc8Bit, IsIx16Bit]>;
def STZ16absx8  : FF<0x9e, (outs), (ins mem_absx:$addr), "stz $addr", []>,
                    Acc16Bit, Ix8Bit, OpSize16,
                    Requires<[Has65802, IsAcc16Bit, IsIx8Bit]>;
def STZ16absx16 : FF<0x9e, (outs), (ins mem_absx:$addr), "stz $addr", []>,
                    Acc16Bit, Ix16Bit, OpSize16,
                    Requires<[Has65802, IsAcc16Bit, IsIx16Bit]>;

def PHA8  : FF<0x48, (outs), (ins), "pha", []>, Acc8Bit,
              Requires<[IsAcc8Bit]>;
def PLA8  : FF<0x68, (outs), (ins), "pla", []>, Acc8Bit,
              Requires<[IsAcc8Bit]>;
def PHX8  : FF<0xda, (outs), (ins), "phx", []>, Ix8Bit,
              Requires<[IsIx8Bit]>;
def PLX8  : FF<0xfa, (outs), (ins), "plx", []>, Ix8Bit,
              Requires<[IsIx8Bit]>;
def PHY8  : FF<0x5a, (outs), (ins), "phy", []>, Ix8Bit,
              Requires<[IsIx8Bit]>;
def PLY8  : FF<0x7a, (outs), (ins), "ply", []>, Ix8Bit,
              Requires<[IsIx8Bit]>;

def PHA16 : FF<0x48, (outs), (ins), "pha", []>, Acc16Bit,
              Requires<[IsAcc16Bit]>;
def PLA16 : FF<0x68, (outs), (ins), "pla", []>, Acc16Bit,
              Requires<[IsAcc16Bit]>;
def PHX16 : FF<0xda, (outs), (ins), "phx", []>, Ix16Bit,
              Requires<[IsIx16Bit]>;
def PLX16 : FF<0xfa, (outs), (ins), "plx", []>, Ix16Bit,
              Requires<[IsIx16Bit]>;
def PHY16 : FF<0x5a, (outs), (ins), "phy", []>, Ix16Bit,
              Requires<[IsIx16Bit]>;
def PLY16 : FF<0x7a, (outs), (ins), "ply", []>, Ix16Bit,
              Requires<[IsIx16Bit]>;

def PHP   : FF<0x08, (outs), (ins), "php", []>;
def PLP   : FF<0x28, (outs), (ins), "plp", []>;
def PHD   : FF<0x0b, (outs), (ins), "phd", []>, Requires<[Has65802]>;
def PLD   : FF<0x2b, (outs), (ins), "pld", []>, Requires<[Has65802]>;
def PHK   : FF<0x4b, (outs), (ins), "phk", []>, Requires<[Has65816]>;
def RTL   : FF<0x6b, (outs), (ins), "rtl", []>, Requires<[Has65816]>;
def PHB   : FF<0x8b, (outs), (ins), "phb", []>, Requires<[Has65816]>;
def PLB   : FF<0xab, (outs), (ins), "plb", []>, Requires<[Has65816]>;
def PEA   : FF<0xf4, (outs), (ins mem_abs:$addr), "pea $addr", []>,
              OpSize16, Requires<[Has65802]>;
def PER   : FF<0x62, (outs), (ins brtarget_pcrel8:$addr), "per $addr", []>,
              OpSize8, Requires<[Has65802]>;
def PEI   : FF<0xd4, (outs), (ins mem_zp:$addr), "pei $addr", []>,
              OpSize8, Requires<[Has65802]>;

let isBranch = 1, isTerminator = 1, isBarrier = 1 in {
  def BRA           : FF<0x80, (outs), (ins brtarget_pcrel8:$addr),
                         "bra $addr", []>, OpSize8, OpPCRel,
                        Requires<[Has65C02]>;
  def BRL           : FF<0x82, (outs), (ins brtarget_pcrel16:$addr),
                         "brl $addr", []>, OpSize16, OpPCRel,
                        Requires<[Has65802]>;
  def JMPabs        : FF<0x4c, (outs), (ins brtarget_abs:$addr),
                         "jmp $addr",
                         [(br bb:$addr)]>, OpSize16;
  def JMPabsind     : FF<0x6c, (outs), (ins brtarget_absind:$addr),
                         "jmp $addr",
                         [(br bb:$addr)]>, OpSize16;
  def JMPabspreix8  : FF<0x7c, (outs), (ins brtarget_abspreix:$addr),
                         "jmp $addr",
                         [(br bb:$addr)]>, OpSize16, Ix8Bit,
                        Requires<[Has65C02, IsIx8Bit]>;
  def JMPabspreix16 : FF<0x7c, (outs), (ins brtarget_abspreix:$addr),
                         "jmp $addr",
                         [(br bb:$addr)]>, OpSize16, Ix16Bit,
                        Requires<[Has65802, IsIx16Bit]>;
  def JMLabsindl    : FF<0xdc, (outs), (ins brtarget_absindl:$addr),
                         "jml $addr",
                         [(br bb:$addr)]>, OpSize16,
                        Requires<[Has65816]>;
  def JMLabsl       : FF<0x5c, (outs), (ins brtarget_absl:$addr),
                         "jml $addr",
                         [(br bb:$addr)]>, OpSize24,
                        Requires<[Has65816]>;
}

let isCall = 1 in {
  def JSRabs        : FF<0x20, (outs), (ins mem_abs:$addr, variable_ops),
                         "jsr $addr",
                         [(C65CALL addr_abs:$addr)]>, OpSize16;
  def JSLabsl       : FF<0x22, (outs), (ins mem_absl:$addr, variable_ops),
                         "jsl $addr",
                         [(C65CALL addr_absl:$addr)]>, OpSize24,
                        Requires<[Has65816]>;
  def JSRabspreix8  : FF<0xfc, (outs), (ins mem_abspreix:$addr, variable_ops),
                         "jsr $addr",
                         [(C65CALL addr_abs:$addr)]>, Ix8Bit, OpSize16,
                        Requires<[Has65802, IsIx8Bit]>;
  def JSRabspreix16 : FF<0xfc, (outs), (ins mem_abspreix:$addr, variable_ops),
                         "jsr $addr",
                         [(C65CALL addr_abs:$addr)]>, Ix16Bit, OpSize16,
                        Requires<[Has65802, IsIx16Bit]>;
}

let isReturn = 1, isTerminator = 1, isBarrier = 1 in {
  def RTS    : FF<0x60, (outs), (ins), "rts", [(C65RET)]>;
  def RTI    : FF<0x40, (outs), (ins), "rti", []>;
}

let Predicates = [IsAcc8Bit, IsIx8Bit] in {
  def TAX8   : FF<0xaa, (outs), (ins), "tax", []>, Acc8Bit, Ix8Bit;
  def TAY8   : FF<0xa8, (outs), (ins), "tay", []>, Acc8Bit, Ix8Bit;
  def TXA8   : FF<0x8a, (outs), (ins), "txa", []>, Acc8Bit, Ix8Bit;
  def TYA8   : FF<0xa8, (outs), (ins), "tya", []>, Acc8Bit, Ix8Bit;
}
let Predicates = [IsIx8Bit] in {
  def TXS8 : FF<0x9a, (outs), (ins), "txs", []>, Ix8Bit;
  def TSX8 : FF<0xba, (outs), (ins), "tsx", []>, Ix8Bit;
}
let Predicates = [Has65802, IsIx8Bit] in {
  def TXY8   : FF<0x9b, (outs), (ins), "txy", []>, Ix8Bit;
  def TYX8   : FF<0xbb, (outs), (ins), "tyx", []>, Ix8Bit;
}
let Predicates = [Has65802, IsIx8Bit] in {
  def TXS816 : FF<0x9a, (outs), (ins), "txs", []>, Ix8Bit;
  def TSX168 : FF<0xba, (outs), (ins), "tsx", []>, Ix8Bit;
}
let Predicates = [Has65802, IsIx16Bit] in {
  def TAX16  : FF<0xaa, (outs), (ins), "tax", []>, Ix16Bit;
  def TAY16  : FF<0xa8, (outs), (ins), "tay", []>, Ix16Bit;
  def TXY16  : FF<0x9b, (outs), (ins), "txy", []>, Ix16Bit;
  def TYX16  : FF<0xbb, (outs), (ins), "tyx", []>, Ix16Bit;
  def TXS16  : FF<0x9a, (outs), (ins), "txs", []>, Ix16Bit;
  def TSX16  : FF<0xba, (outs), (ins), "tsx", []>, Ix16Bit;
}
let Predicates = [Has65802, IsAcc16Bit, IsIx8Bit] in {
  def TAX168 : FF<0xaa, (outs), (ins), "tax", []>, Acc16Bit, Ix8Bit;
  def TAY168 : FF<0xa8, (outs), (ins), "tay", []>, Acc16Bit, Ix8Bit;
  def TXA816 : FF<0x8a, (outs), (ins), "txa", []>, Acc16Bit, Ix8Bit;
  def TYA816 : FF<0xa8, (outs), (ins), "tya", []>, Acc16Bit, Ix8Bit;
}
let Predicates = [Has65802, IsAcc8Bit, IsIx16Bit] in {
  def TXA168 : FF<0x8a, (outs), (ins), "txa", []>, Acc8Bit, Ix16Bit;
  def TYA168 : FF<0xa8, (outs), (ins), "tya", []>, Acc8Bit, Ix16Bit;
}
let Predicates = [Has65802, IsAcc16Bit, IsIx16Bit] in {
  def TXA16  : FF<0x8a, (outs), (ins), "txa", []>, Acc16Bit, Ix16Bit;
  def TYA16  : FF<0xa8, (outs), (ins), "tya", []>, Acc16Bit, Ix16Bit;
}
let Predicates = [Has65802] in {
  def TCS    : FF<0x1b, (outs), (ins), "tcs", []>;
  def TSC    : FF<0x3b, (outs), (ins), "tsc", []>;
  def TCD    : FF<0x5b, (outs), (ins), "tcd", []>;
  def TDC    : FF<0x7b, (outs), (ins), "tdc", []>;
}

let Predicates = [Has65C02, IsAcc8Bit] in {
  def TSB8zp   : FF<0x04, (outs), (ins mem_zp:$addr), "tsb $addr", []>,
                   Acc8Bit, OpSize8;
  def TSB8abs  : FF<0x0c, (outs), (ins mem_abs:$addr), "tsb $addr", []>,
                   Acc8Bit, OpSize16;
  def TRB8zp   : FF<0x04, (outs), (ins mem_zp:$addr), "trb $addr", []>,
                   Acc8Bit, OpSize8;
  def TRB8abs  : FF<0x0c, (outs), (ins mem_abs:$addr), "trb $addr", []>,
                   Acc8Bit, OpSize16;
}
let Predicates = [Has65C02, IsAcc16Bit] in {
  def TSB16zp  : FF<0x04, (outs), (ins mem_zp:$addr), "tsb $addr", []>,
                   Acc16Bit, OpSize8;
  def TSB16abs : FF<0x0c, (outs), (ins mem_abs:$addr), "tsb $addr", []>,
                  Acc16Bit, OpSize16;
  def TRB16zp  : FF<0x04, (outs), (ins mem_zp:$addr), "trb $addr", []>,
                   Acc16Bit, OpSize8;
  def TRB16abs : FF<0x0c, (outs), (ins mem_abs:$addr), "trb $addr", []>,
                   Acc16Bit, OpSize16;
}

def WAI     : FF<0xcb, (outs), (ins), "wai", []>,
                Requires<[Has65802]>;
def STP     : FF<0xfb, (outs), (ins), "stp", []>,
                Requires<[Has65802]>;
def XBA     : FF<0xeb, (outs), (ins), "xba", []>,
                Requires<[Has65802]>;
def XCE     : FF<0xfb, (outs), (ins), "xce", []>,
                Requires<[Has65802]>;
def COP     : FF<0x02, (outs), (ins imm8:$sig), "cop $sig", []>,
                Requires<[Has65802]>;
def WDM     : FF<0x42, (outs), (ins), "wdm", []>,
                Requires<[Has65802]>;
def MVP8    : FF<0x44, (outs), (ins imm8:$dstbank, imm8:$srcbank),
                 "mvp $dstbank,$srcbank", []>, OpSize8,
                Requires<[Has65802, IsIx8Bit]>;
def MVP16   : FF<0x44, (outs), (ins imm8:$dstbank, imm8:$srcbank),
                 "mvp $dstbank,$srcbank", []>, OpSize8,
                Requires<[Has65802, IsIx16Bit]>;
def MVN8    : FF<0x54, (outs), (ins imm8:$dstbank, imm8:$srcbank),
                 "mvn $dstbank,$srcbank", []>, OpSize8,
                Requires<[Has65802, IsIx8Bit]>;
def MVN16   : FF<0x54, (outs), (ins imm8:$dstbank, imm8:$srcbank),
                 "mvn $dstbank,$srcbank", []>, OpSize8,
                Requires<[Has65802, IsIx16Bit]>;
def REP     : FF<0xc2, (outs), (ins imm8:$imm), "rep $imm", []>,
                OpSize8, Requires<[Has65802]>;
def SEP     : FF<0xe2, (outs), (ins imm8:$imm), "sep $imm", []>,
                OpSize8, Requires<[Has65802]>;

def CLC     : FF<0x18, (outs), (ins), "clc", []>;
def SEC     : FF<0x38, (outs), (ins), "sec", []>;
def CLD     : FF<0xd8, (outs), (ins), "cld", []>;
def SED     : FF<0xf8, (outs), (ins), "sed", []>;
def CLI     : FF<0x58, (outs), (ins), "cli", []>;
def SEI     : FF<0x78, (outs), (ins), "sei", []>;
def CLV     : FF<0xb8, (outs), (ins), "clv", []>;

let Predicates = [IsIx8Bit] in {
  def INX8  : FF<0xe8, (outs), (ins), "inx", []>, Ix8Bit;
  def INY8  : FF<0xc8, (outs), (ins), "iny", []>, Ix8Bit;
  def DEX8  : FF<0xca, (outs), (ins), "dex", []>, Ix8Bit;
  def DEY8  : FF<0x88, (outs), (ins), "dey", []>, Ix8Bit;
}
let Predicates = [IsAcc8Bit] in {
  def INA8  : FF<0x1a, (outs), (ins), "ina", []>, Acc8Bit;
  def DEA8  : FF<0x3a, (outs), (ins), "dea", []>, Acc8Bit;
}
let Predicates = [IsIx16Bit] in {
  def INX16 : FF<0xe8, (outs), (ins), "inx", []>, Ix16Bit;
  def INY16 : FF<0xc8, (outs), (ins), "iny", []>, Ix16Bit;
  def DEX16 : FF<0xca, (outs), (ins), "dex", []>, Ix16Bit;
  def DEY16 : FF<0x88, (outs), (ins), "dey", []>, Ix16Bit;
}
let Predicates = [IsAcc16Bit] in {
  def INA16 : FF<0x1a, (outs), (ins), "ina", []>, Acc16Bit;
  def DEA16 : FF<0x3a, (outs), (ins), "dea", []>, Acc16Bit;
}

//===----------------------------------------------------------------------===//
// Pseudo-instructions
//===----------------------------------------------------------------------===//

// ADJCALLSTACKDOWN, ADJCALLSTACKUP are mandatory pseudo-instructions
let Defs = [S, P], Uses = [S], isCodeGenOnly = 1 in {
  def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                                ";!ADJCALLSTACKDOWN $amt",
                                [(C65CALLSEQ_START timm:$amt)]>;
  def ADJCALLSTACKUP : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                              ";!ADJCALLSTACKUP $amt1",
                              [(C65CALLSEQ_END timm:$amt1, timm:$amt2)]>;
}

// Directive instruction format
class C65Directive<string asmstr> : InstC65<(outs), (ins), asmstr, []>;

//let isCodeGenOnly = 1 in {
  def LONGA_ON  : C65Directive<"longa on">;
  def LONGA_OFF : C65Directive<"longa off">;
  def LONGI_ON  : C65Directive<"longi on">;
  def LONGI_OFF : C65Directive<"longi off">;
//}

//===----------------------------------------------------------------------===//
// Zero-page register operations
//===----------------------------------------------------------------------===//

class ZRInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  bit is_ZRInstr = 1;
  let usesCustomInserter = 1;
}

// Form: (outs R1), (ins mem)
//   ZLD
multiclass ZRI_ZLD<string mnemonic, RegisterClass RC, ValueType Ty,
                   SDPatternOperator LoadNode> {
  let Constraints = "@earlyclobber $reg1" in {
  def zp   : ZRInstr<(outs RC:$reg1), (ins mem_zp:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_zp:$mem)))]>;
  def abs  : ZRInstr<(outs RC:$reg1), (ins mem_abs:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_abs:$mem)))]>;
  def absl : ZRInstr<(outs RC:$reg1), (ins mem_absl:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_absl:$mem)))]>;
  def s    : ZRInstr<(outs RC:$reg1), (ins mem_srel:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_s:$mem)))]>;
  def ri   : ZRInstr<(outs RC:$reg1), (ins mem_ri:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_ri:$mem)))]>;
  def rr   : ZRInstr<(outs RC:$reg1), (ins mem_rr:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_rr:$mem)))]>;
  def rif  : ZRInstr<(outs RC:$reg1), (ins mem_rif:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_rif:$mem)))]>;
  def rrf  : ZRInstr<(outs RC:$reg1), (ins mem_rrf:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(set RC:$reg1, (Ty (LoadNode addr_rrf:$mem)))]>;
  }
}

// multiclass ZRI_R_M<string mnemonic, PatFrag frag> {
//   defm NAME#8 : ZRI_R_M_Sized<mnemonic, ZRC8, frag>, ZROpSize8;
//   defm NAME#16 : ZRI_R_M_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
//   defm NAME#32 : ZRI_R_M_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
//   defm NAME#64 : ZRI_R_M_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
// }


//def load_frag : PatFrag<(ops node:$dst, node:$mem),
//                        (set node:$dst, (extload node:$mem))>;
let mayLoad = 1 in {
  defm ZLD8        : ZRI_ZLD<"zld", ZRC8,  i8,  load>,        ZROpSize8;
  defm ZLD16       : ZRI_ZLD<"zld", ZRC16, i16, load>,        ZROpSize16;
  defm ZLD32       : ZRI_ZLD<"zld", ZRC32, i32, load>,        ZROpSize32;
  defm ZLD64       : ZRI_ZLD<"zld", ZRC64, i64, load>,        ZROpSize64;
  defm ZLD16ext8   : ZRI_ZLD<"zld", ZRC16, i16, extloadi8>,   ZROpSize16;
  defm ZLD16sext8  : ZRI_ZLD<"zld", ZRC16, i16, sextloadi8>,  ZROpSize16;
  defm ZLD16zext8  : ZRI_ZLD<"zld", ZRC16, i16, zextloadi8>,  ZROpSize16;
  defm ZLD32ext8   : ZRI_ZLD<"zld", ZRC32, i32, extloadi8>,   ZROpSize32;
  defm ZLD32sext8  : ZRI_ZLD<"zld", ZRC32, i32, sextloadi8>,  ZROpSize32;
  defm ZLD32zext8  : ZRI_ZLD<"zld", ZRC32, i32, zextloadi8>,  ZROpSize32;
  defm ZLD32ext16  : ZRI_ZLD<"zld", ZRC32, i32, extloadi16>,  ZROpSize32;
  defm ZLD32sext16 : ZRI_ZLD<"zld", ZRC32, i32, sextloadi16>, ZROpSize32;
  defm ZLD32zext16 : ZRI_ZLD<"zld", ZRC32, i32, zextloadi16>, ZROpSize32;
  defm ZLD64ext8   : ZRI_ZLD<"zld", ZRC64, i64, extloadi8>,   ZROpSize64;
  defm ZLD64sext8  : ZRI_ZLD<"zld", ZRC64, i64, sextloadi8>,  ZROpSize64;
  defm ZLD64zext8  : ZRI_ZLD<"zld", ZRC64, i64, zextloadi8>,  ZROpSize64;
  defm ZLD64ext16  : ZRI_ZLD<"zld", ZRC64, i64, extloadi16>,  ZROpSize64;
  defm ZLD64sext16 : ZRI_ZLD<"zld", ZRC64, i64, sextloadi16>, ZROpSize64;
  defm ZLD64zext16 : ZRI_ZLD<"zld", ZRC64, i64, zextloadi16>, ZROpSize64;
  defm ZLD64ext32  : ZRI_ZLD<"zld", ZRC64, i64, extloadi32>,  ZROpSize64;
  defm ZLD64sext32 : ZRI_ZLD<"zld", ZRC64, i64, sextloadi32>, ZROpSize64;
  defm ZLD64zext32 : ZRI_ZLD<"zld", ZRC64, i64, zextloadi32>, ZROpSize64;
}

// Form: (outs), (ins R1, mem)
//   ZST
multiclass ZRI_ZST<string mnemonic, RegisterClass RC,
                   SDPatternOperator StoreNode> {
  def zp   : ZRInstr<(outs), (ins RC:$reg1, mem_zp:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_zp:$mem)]>;
  def abs  : ZRInstr<(outs), (ins RC:$reg1, mem_abs:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_abs:$mem)]>;
  def absl : ZRInstr<(outs), (ins RC:$reg1, mem_absl:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_absl:$mem)]>;
  def s    : ZRInstr<(outs), (ins RC:$reg1, mem_srel:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_s:$mem)]>;
  def ri   : ZRInstr<(outs), (ins RC:$reg1, mem_ri:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_ri:$mem)]>;
  def rr   : ZRInstr<(outs), (ins RC:$reg1, mem_rr:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_rr:$mem)]>;
  def rif  : ZRInstr<(outs), (ins RC:$reg1, mem_rif:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_rif:$mem)]>;
  def rrf  : ZRInstr<(outs), (ins RC:$reg1, mem_rrf:$mem),
                     !strconcat(mnemonic, " $reg1,$mem"),
                     [(StoreNode RC:$reg1, addr_rrf:$mem)]>;
}

// multiclass ZRI__RM<string mnemonic, PatFrag frag> {
//   defm NAME#8 : ZRI__RM_Sized<mnemonic, ZRC8, frag>, ZROpSize8;
//   defm NAME#16 : ZRI__RM_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
//   defm NAME#32 : ZRI__RM_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
//   defm NAME#64 : ZRI__RM_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
// }

//def store_frag : PatFrag<(ops node:$src, node:$mem),
//                         (unindexedstore node:$src, node:$mem)>;
let mayStore = 1 in {
  defm ZST8         : ZRI_ZST<"zst",   ZRC8,  store>,         ZROpSize8;
  defm ZST16        : ZRI_ZST<"zst",   ZRC16, store>,         ZROpSize16;
  defm ZST32        : ZRI_ZST<"zst",   ZRC32, store>,         ZROpSize32;
  defm ZST64        : ZRI_ZST<"zst",   ZRC64, store>,         ZROpSize64;
  defm ZST16trunc8  : ZRI_ZST<"zst8",  ZRC16, truncstorei8>,  ZROpSize16;
  defm ZST32trunc8  : ZRI_ZST<"zst8",  ZRC32, truncstorei8>,  ZROpSize32;
  defm ZST32trunc16 : ZRI_ZST<"zst16", ZRC32, truncstorei16>, ZROpSize32;
  defm ZST64trunc8  : ZRI_ZST<"zst8",  ZRC64, truncstorei8>,  ZROpSize64;
  defm ZST64trunc16 : ZRI_ZST<"zst16", ZRC64, truncstorei16>, ZROpSize64;
  defm ZST64trunc32 : ZRI_ZST<"zst32", ZRC64, truncstorei32>, ZROpSize64;
}

// Form: (outs R1), (ins R2, R3)
//   ZADD, ZSUB, ZAND, ZOR, ZXOR, ZSHL, ZLSHR
class ZRI_R_RR_Sized<string mnemonic, RegisterClass RC, PatFrag frag>
  : ZRInstr<(outs RC:$reg1), (ins RC:$reg2, RC:$reg3),
            !strconcat(mnemonic, " $reg1,$reg2,$reg3"),
            [(frag RC:$reg1, RC:$reg2, RC:$reg3)]>;

multiclass ZRI_R_RR<string mnemonic, PatFrag frag> {
  def NAME#8  : ZRI_R_RR_Sized<mnemonic, ZRC8,  frag>, ZROpSize8;
  def NAME#16 : ZRI_R_RR_Sized<mnemonic, ZRC16, frag>, ZROpSize16;
  def NAME#32 : ZRI_R_RR_Sized<mnemonic, ZRC32, frag>, ZROpSize32;
  def NAME#64 : ZRI_R_RR_Sized<mnemonic, ZRC64, frag>, ZROpSize64;
}

def add_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (add node:$src1, node:$src2))>;
defm ZADD : ZRI_R_RR<"zadd", add_frag>;

def sub_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (sub node:$src1, node:$src2))>;
defm ZSUB : ZRI_R_RR<"zsub", sub_frag>;

def and_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (and node:$src1, node:$src2))>;
defm ZAND : ZRI_R_RR<"zand", and_frag>;

def or_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                      (set node:$dst, (or node:$src1, node:$src2))>;
defm ZOR : ZRI_R_RR<"zor", or_frag>;

def xor_frag : PatFrag<(ops node:$dst, node:$src1, node:$src2),
                       (set node:$dst, (xor node:$src1, node:$src2))>;
defm ZXOR : ZRI_R_RR<"zxor", xor_frag>;

// class ZSHL<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), "zshl $dst,$src,$amt",
//             [(set Ty:$dst, (shl Ty:$src, i16:$amt))]>;

// def ZSHL8  : ZSHL<ZRC8,  i8>, ZROpSize8;
// def ZSHL16 : ZSHL<ZRC16, i16>, ZROpSize16;
// def ZSHL32 : ZSHL<ZRC32, i32>, ZROpSize32;
// def ZSHL64 : ZSHL<ZRC64, i64>, ZROpSize64;

// class ZLSHR<RegisterClass RC, ValueType Ty>
//   : ZRInstr<(outs RC:$dst), (ins RC:$src, ZRC16:$amt), "zlshr $dst,$src,$amt",
//             [(set Ty:$dst, (srl Ty:$src, i16:$amt))]>;

// def ZLSHR8  : ZLSHR<ZRC8,  i8>, ZROpSize8;
// def ZLSHR6  : ZLSHR<ZRC16, i16>, ZROpSize16;
// def ZLSHR32 : ZLSHR<ZRC32, i32>, ZROpSize32;
// def ZLSHR64 : ZLSHR<ZRC64, i64>, ZROpSize64;

class ZRI_ZMOV<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs RC:$dst), (ins RC:$src), "zmov $dst,$src",
            []>;

def ZMOV8  : ZRI_ZMOV<ZRC8,  i8>,  ZROpSize8;
def ZMOV16 : ZRI_ZMOV<ZRC16, i16>, ZROpSize16;
def ZMOV32 : ZRI_ZMOV<ZRC32, i32>, ZROpSize32;
def ZMOV64 : ZRI_ZMOV<ZRC64, i64>, ZROpSize64;

class ZRI_ZMOV_EXT<string mnemonic, RegisterClass RC1, RegisterClass RC2,
                   SDPatternOperator ExtendNode>
  : ZRInstr<(outs RC1:$dst), (ins RC2:$src),
            !strconcat(mnemonic, " $dst,$src"),
            [(set RC1:$dst, (ExtendNode RC2:$src))]>;

def ZMOV16ext8   : ZRI_ZMOV_EXT<"zmov", ZRC16, ZRC8,  anyext>, ZROpSize16;
def ZMOV16sext8  : ZRI_ZMOV_EXT<"zmov", ZRC16, ZRC8,  sext>,   ZROpSize16;
def ZMOV16zext8  : ZRI_ZMOV_EXT<"zmov", ZRC16, ZRC8,  zext>,   ZROpSize16;
def ZMOV32ext8   : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC8,  anyext>, ZROpSize32;
def ZMOV32sext8  : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC8,  sext>,   ZROpSize32;
def ZMOV32zext8  : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC8,  zext>,   ZROpSize32;
def ZMOV32ext16  : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC16, anyext>, ZROpSize32;
def ZMOV32sext16 : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC16, sext>,   ZROpSize32;
def ZMOV32zext16 : ZRI_ZMOV_EXT<"zmov", ZRC32, ZRC16, zext>,   ZROpSize32;
def ZMOV64ext8   : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC8,  anyext>, ZROpSize64;
def ZMOV64sext8  : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC8,  sext>,   ZROpSize64;
def ZMOV64zext8  : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC8,  zext>,   ZROpSize64;
def ZMOV64ext16  : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC16, anyext>, ZROpSize64;
def ZMOV64sext16 : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC16, sext>,   ZROpSize64;
def ZMOV64zext16 : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC16, zext>,   ZROpSize64;
def ZMOV64ext32  : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC32, anyext>, ZROpSize64;
def ZMOV64sext32 : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC32, sext>,   ZROpSize64;
def ZMOV64zext32 : ZRI_ZMOV_EXT<"zmov", ZRC64, ZRC32, zext>,   ZROpSize64;

class ZBRCC<RegisterClass RC, ValueType Ty>
  : ZRInstr<(outs), (ins i32imm:$cc, RC:$op0, RC:$op1, brtarget_pcrel8:$dst),
            "zbrcc $cc,$op0,$op1,$dst",
            [(C65BR_CC imm:$cc, Ty:$op0, Ty:$op1, bb:$dst)]> {
  let isBranch = 1;
  let isTerminator = 1;
}

def ZBRCC8  : ZBRCC<ZRC8,  i8>,  ZROpSize8;
def ZBRCC16 : ZBRCC<ZRC16, i16>, ZROpSize16;
def ZBRCC32 : ZBRCC<ZRC32, i32>, ZROpSize32;
def ZBRCC64 : ZBRCC<ZRC64, i64>, ZROpSize64;

class ZSELECTCC<RegisterClass RC1, RegisterClass RC2>
  : ZRInstr<(outs RC1:$out), (ins RC2:$lhs, RC2:$rhs,
                                  RC1:$true, RC1:$false, i32imm:$cc),
            "zselectcc $out,$lhs,$rhs,$true,$false,$cc",
            [(set RC1:$out, (C65SELECT_CC RC2:$lhs, RC2:$rhs,
                                          RC1:$true, RC1:$false, imm:$cc))]>;

multiclass ZSELECTCC_A<RegisterClass RC1> {
  def _8  : ZSELECTCC<RC1, ZRC8>,  ZROpSize8;
  def _16 : ZSELECTCC<RC1, ZRC16>, ZROpSize16;
  def _32 : ZSELECTCC<RC1, ZRC32>, ZROpSize32;
  def _64 : ZSELECTCC<RC1, ZRC64>, ZROpSize64;
}

defm ZSELECTCC8  : ZSELECTCC_A<ZRC8>;
defm ZSELECTCC16 : ZSELECTCC_A<ZRC16>;
defm ZSELECTCC32 : ZSELECTCC_A<ZRC32>;
defm ZSELECTCC64 : ZSELECTCC_A<ZRC64>;

class ZLD_imm<RegisterClass RC, ValueType Ty, Operand immOp>
  : ZRInstr<(outs RC:$dst), (ins immOp:$imm), "zld $dst,$imm",
            [(set Ty:$dst, imm:$imm)]> {
  let isMoveImm = 1;
}

def ZLD8imm  : ZLD_imm<ZRC8,  i8,  i8imm>,  ZROpSize8;
def ZLD16imm : ZLD_imm<ZRC16, i16, i16imm>, ZROpSize16;
def ZLD32imm : ZLD_imm<ZRC32, i32, i32imm>, ZROpSize32;
def ZLD64imm : ZLD_imm<ZRC64, i64, i64imm>, ZROpSize64;

class ZPUSH_imm<ValueType Ty, Operand immOp>
  : ZRInstr<(outs), (ins immOp:$imm), "zpush $imm",
            [(C65PUSH (Ty imm:$imm))]> {
  let hasSideEffects = 1;
}

def ZPUSH8imm  : ZPUSH_imm<i8,  i8imm>,  ZROpSize8;
def ZPUSH16imm : ZPUSH_imm<i16, i16imm>, ZROpSize16;
def ZPUSH32imm : ZPUSH_imm<i32, i32imm>, ZROpSize32;
def ZPUSH64imm : ZPUSH_imm<i64, i64imm>, ZROpSize64;

class ZPUSH<RegisterClass RC>
  : ZRInstr<(outs), (ins RC:$reg), "zpush $reg",
            [(C65PUSH RC:$reg)]> {
  let hasSideEffects = 1;
}

def ZPUSH8  : ZPUSH<ZRC8>,  ZROpSize8;
def ZPUSH16 : ZPUSH<ZRC16>, ZROpSize16;
def ZPUSH32 : ZPUSH<ZRC32>, ZROpSize32;
def ZPUSH64 : ZPUSH<ZRC64>, ZROpSize64;

// "Load effective address"; allowing FrameIndex nodes to be resolved.
def ZLEA16s : ZRInstr<(outs ZRC16:$reg1), (ins mem_srel:$mem),
                      "zlea16s $reg1,$mem",
                      [(set ZRC16:$reg1, addr_s:$mem)]>;

//===----------------------------------------------------------------------===//
// Non-Instruction Patterns
//===----------------------------------------------------------------------===//

// Calls
def : Pat<(C65CALL tglobaladdr:$dst),
          (JSRabs tglobaladdr:$dst)>;
def : Pat<(C65CALL texternalsym:$dst),
          (JSRabs texternalsym:$dst)>;

// Truncating patterns
def : Pat<(i8 (trunc ZRC16:$src)),
          (EXTRACT_SUBREG ZRC16:$src, sub_8bit)>;

def : Pat<(i8 (trunc ZRC32:$src)),
          (EXTRACT_SUBREG ZRC32:$src, sub_8bit)>;
def : Pat<(i16 (trunc ZRC32:$src)),
          (EXTRACT_SUBREG ZRC32:$src, sub_16bit)>;

def : Pat<(i8 (trunc ZRC64:$src)),
          (EXTRACT_SUBREG ZRC64:$src, sub_8bit)>;
def : Pat<(i16 (trunc ZRC64:$src)),
          (EXTRACT_SUBREG ZRC64:$src, sub_16bit)>;
def : Pat<(i32 (trunc ZRC64:$src)),
          (EXTRACT_SUBREG ZRC64:$src, sub_32bit)>;

//def : Pat<(i16 (anyext ZRC8:$src)), (EXTRACT

// An 'srl' node with a single use.
def srl_su : PatFrag<(ops node:$lhs, node:$rhs),
                     (srl node:$lhs, node:$rhs), [{
  return N->hasOneUse();
}]>;

// h-register tricks
def : Pat<(i8 (trunc (srl_su ZRC16:$src, (i8 8)))),
          (EXTRACT_SUBREG ZRC16:$src, sub_8bit_hi)>;

def : Pat<(i16 (C65FRAME_ADDR imm:$imm)), (ZLEA16s imm:$imm, 0)>;

// def : Pat<(i8 (trunc (srl_su ZRC32:$src, (i8 8)))),
//           (EXTRACT_SUBREG
//             (EXTRACT_SUBREG ZRC32:$src, sub_16bit), sub_8bit_hi)>;
// def : Pat<(i8 (trunc (srl_su ZRC32:$src, (i8 16)))),
//           (EXTRACT_SUBREG
//             (EXTRACT_SUBREG ZRC32:$src, sub_16bit_hi), sub_8bit)>;
// def : Pat<(i8 (trunc (srl_su ZRC32:$src, (i8 24)))),
//           (EXTRACT_SUBREG
//             (EXTRACT_SUBREG ZRC32:$src, sub_16bit_hi), sub_8bit_hi)>;

// def : Pat<(i8 (trunc (srl_su ZRC64:$src, (i8 8)))),
//           (EXTRACT_SUBREG
//             (EXTRACT_SUBREG
//               (EXTRACT_SUBREG ZRC64:$src, sub_32bit), sub_16bit), sub_8bit_hi)>;

// def : Pat<(i8 (C65Wrapper tconstpool:$dst)), (ZLD8imm tconstpool:$dst)>;
// def : Pat<(i8 (C65Wrapper tjumptable:$dst)), (ZLD8imm tjumptable:$dst)>;
// def : Pat<(i8 (C65Wrapper tglobaladdr:$dst)), (ZLD8imm tglobaladdr:$dst)>;
// def : Pat<(i8 (C65Wrapper texternalsym:$dst)), (ZLD8imm texternalsym:$dst)>;
// def : Pat<(i8 (C65Wrapper tblockaddress:$dst)), (ZLD8imm tblockaddress:$dst)>;
def : Pat<(i16 (C65Wrapper tconstpool:$dst)), (ZLD16imm tconstpool:$dst)>;
def : Pat<(i16 (C65Wrapper tjumptable:$dst)), (ZLD16imm tjumptable:$dst)>;
def : Pat<(i16 (C65Wrapper tglobaladdr:$dst)), (ZLD16imm tglobaladdr:$dst)>;
def : Pat<(i16 (C65Wrapper texternalsym:$dst)), (ZLD16imm texternalsym:$dst)>;
def : Pat<(i16 (C65Wrapper tblockaddress:$dst)), (ZLD16imm tblockaddress:$dst)>;
def : Pat<(i32 (C65FarWrapper tconstpool:$dst)), (ZLD32imm tconstpool:$dst)>;
def : Pat<(i32 (C65FarWrapper tjumptable:$dst)), (ZLD32imm tjumptable:$dst)>;
def : Pat<(i32 (C65FarWrapper tglobaladdr:$dst)), (ZLD32imm tglobaladdr:$dst)>;
def : Pat<(i32 (C65FarWrapper texternalsym:$dst)), (ZLD32imm texternalsym:$dst)>;
def : Pat<(i32 (C65FarWrapper tblockaddress:$dst)), (ZLD32imm tblockaddress:$dst)>;
// def : Pat<(i32 (C65Wrapper tconstpool:$dst)), (ZLD32imm tconstpool:$dst)>;
// def : Pat<(i32 (C65Wrapper tjumptable:$dst)), (ZLD32imm tjumptable:$dst)>;
// def : Pat<(i32 (C65Wrapper tglobaladdr:$dst)), (ZLD32imm tglobaladdr:$dst)>;
// def : Pat<(i32 (C65Wrapper texternalsym:$dst)), (ZLD32imm texternalsym:$dst)>;
// def : Pat<(i32 (C65Wrapper tblockaddress:$dst)), (ZLD32imm tblockaddress:$dst)>;
// def : Pat<(i64 (C65Wrapper tconstpool:$dst)), (ZLD64imm tconstpool:$dst)>;
// def : Pat<(i64 (C65Wrapper tjumptable:$dst)), (ZLD64imm tjumptable:$dst)>;
// def : Pat<(i64 (C65Wrapper tglobaladdr:$dst)), (ZLD64imm tglobaladdr:$dst)>;
// def : Pat<(i64 (C65Wrapper texternalsym:$dst)), (ZLD64imm texternalsym:$dst)>;
// def : Pat<(i64 (C65Wrapper tblockaddress:$dst)), (ZLD64imm tblockaddress:$dst)>;

// def : Pat<(store (i8 (C65Wrapper tglobaladdr:$src)), addr:$dst),
//           (ZST8 addr:$dst, tglobaladdr:$src)>;
