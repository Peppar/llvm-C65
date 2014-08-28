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

def C65cmp   : SDNode<"C65ISD::CMP", SDT_C65cmp,
                      [SDNPOutGlue]>;
def C65ret   : SDNode<"C65ISD::RET", SDTNone,
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

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F01_binary<bits<3> op, string asmstr> {
  def imm : F01<op, 0b010, (outs), (ins i16imm:$imm),
                !strconcat(asmstr, " $imm"), []>;

  def zp  : F01<op, 0b001, (outs), (ins mem_zp:$addr),
                !strconcat(asmstr, " $addr"), []>;

  def i   : F01<op, 0b011, (outs), (ins mem_a16:$addr),
                !strconcat(asmstr, " $addr"), []>;

  def ix  : F01<op, 0b110, (outs), (ins mem_ix16:$addr),
                 !strconcat(asmstr, " $addr"), []>;
}

defm ORA   : F01_binary<0b000, "ORA">;

defm AND   : F01_binary<0b001, "AND">;

defm EOR   : F01_binary<0b010, "EOR">;

defm ADC   : F01_binary<0b011, "ADC">;

defm SBC   : F01_binary<0b111, "SBC">;

def STAzp  : F01<0b100, 0b001, (outs), (ins mem_zp:$addr),
                 "STA $addr", []>;

def STAi   : F01<0b100, 0b011, (outs), (ins mem_a16:$addr),
                 "STA $addr", []>;

def STAix  : F01<0b100, 0b110, (outs), (ins mem_ix16:$addr),
                 "STA $addr", []>;

def LDAimm : F01<0b101, 0b010, (outs), (ins i16imm:$imm),
                 "LDA $imm", []>;

def CMPimm : F01<0b110, 0b010, (outs), (ins i16imm:$imm),
                 "CMP $imm", []>;

def LDAzp  : F01<0b101, 0b011, (outs), (ins mem_zp:$addr),
                 "LDA $addr", []>;

def LDAi   : F01<0b101, 0b011, (outs), (ins mem_a16:$addr),
                 "LDA $addr", []>;

def LDAix  : F01<0b101, 0b110, (outs), (ins mem_ix16:$addr),
                 "LDA $addr", []>;

def CMPzp  : F01<0b110, 0b001, (outs), (ins mem_zp:$addr),
                 "CMP $addr", []>;

def CMPi   : F01<0b110, 0b011, (outs), (ins mem_a16:$addr),
                 "CMP $addr", []>;

def CMPix  : F01<0b110, 0b011, (outs), (ins mem_ix16:$addr),
                 "CMP $addr", []>;

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc
//===----------------------------------------------------------------------===//

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F10_unary<bits<3> op, string asmstr> {
  def zp : F10<op, 0b001, (outs), (ins mem_zp:$addr),
               !strconcat(asmstr, " $addr"), []>;

  def i  : F10<op, 0b011, (outs), (ins mem_a16:$addr),
               !strconcat(asmstr, " $addr"), []>;

  def ix : F10<op, 0b111, (outs), (ins mem_ix16:$addr),
               !strconcat(asmstr, " $addr"), []>;
}

defm ASL   : F10_unary<0b000, "ASL">;

defm ROL   : F10_unary<0b001, "ROL">;

defm LSR   : F10_unary<0b010, "LSR">;

defm ROR   : F10_unary<0b011, "ROR">;

defm DEC   : F10_unary<0b110, "DEC">;

defm INC   : F10_unary<0b111, "INC">;

def STXzp  : F10<0b100, 0b001, (outs), (ins mem_zp:$addr),
                "STX $addr", []>;

def STXi   : F10<0b100, 0b011, (outs), (ins mem_a16:$addr),
                "STX $addr", []>;

def ASL    : F10<0b000, 0b010, (outs), (ins),
                "ASL A", []>;

def ROL    : F10<0b001, 0b010, (outs), (ins),
                "ROL A", []>;

def LSR    : F10<0b001, 0b010, (outs), (ins),
                "LSR A", []>;

def ROR    : F10<0b001, 0b010, (outs), (ins),
                "ROR A", []>;

def LDXimm : F10<0b101, 0b000, (outs), (ins i16imm:$imm),
                 "LDX $imm", []>;

def LDXzp  : F10<0b101, 0b001, (outs), (ins mem_zp:$addr),
                 "LDX $addr", []>;

def LDXi   : F10<0b101, 0b011, (outs), (ins mem_a16:$addr),
                 "LDX $addr", []>;

def LDXiy  : F10<0b101, 0b111, (outs), (ins mem_iy16:$addr),
                 "LDX $addr", []>;

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, (jmp), sty, ldy, cpy, cpx
//===----------------------------------------------------------------------===//

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

def STYzp  : F00<0b100, 0b001, (outs), (ins mem_zp:$addr),
                "STY $addr", []>;

def STYi   : F00<0b100, 0b011, (outs), (ins mem_a16:$addr),
                "STY $addr", []>;

def LDYimm : F00<0b101, 0b000, (outs), (ins i16imm:$imm),
                 "LDY $imm", []>;

def LDYzp  : F00<0b101, 0b001, (outs), (ins mem_zp:$addr),
                 "LDY $addr", []>;

def LDYi   : F00<0b101, 0b011, (outs), (ins mem_a16:$addr),
                 "LDY $addr", []>;

def LDYix  : F00<0b101, 0b111, (outs), (ins mem_ix16:$addr),
                 "LDY $addr", []>;

def CPYimm : F00<0b110, 0b000, (outs), (ins i16imm:$imm),
                 "CPY $imm", []>;

def CPXimm : F00<0b111, 0b000, (outs), (ins i16imm:$imm),
                 "CPX $imm", []>;

def CPYzp  : F00<0b110, 0b001, (outs), (ins mem_zp:$addr),
                 "CPY $addr", []>;

def CPYi   : F00<0b110, 0b011, (outs), (ins mem_a16:$addr),
                 "CPY $addr", []>;

def CPXiz  : F00<0b111, 0b001, (outs), (ins mem_zp:$addr),
                 "CPX $addr", []>;

def CPXi   : F00<0b111, 0b011, (outs), (ins mem_a16:$addr),
                 "CPX $addr", []>;

//===----------------------------------------------------------------------===//
// Format 11 instructions : Extended ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F11<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b11, outs, ins, asmstr, pattern> {
  let hasSideEffects = 1;
}

multiclass F11_binary<bits<3> op, string asmstr> {
  // Binary operation on 16-bit accumulator
  def is : F11<op, 0b000, (outs), (ins mem_is16:$addr),
               !strconcat(asmstr, " $addr"), []>;
}

defm ORA : F11_binary<0b000, "ORA">;

defm AND : F11_binary<0b001, "AND">;

defm EOR : F11_binary<0b010, "EOR">;

defm ADC : F11_binary<0b011, "ADC">;

defm SBC : F11_binary<0b111, "SBC">;

def STAis : F11<0b100, 0b000, (outs), (ins mem_is16:$addr),
                "STA $addr", []>;

def LDAis  : F11<0b101, 0b000, (outs), (ins mem_is16:$addr),
                 "LDA $addr", []>;

def CMPis  : F11<0b110, 0b000, (outs), (ins mem_is16:$addr),
                 "CMP $addr", []>;

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

//===----------------------------------------------------------------------===//
// Free format instructions
//===----------------------------------------------------------------------===//

class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst = op;
  let hasSideEffects = 1;
}

def NOP    : FF<0xea, (outs), (ins), "NOP", []>;

def BITimm : FF<0x89, (outs), (ins i16imm:$imm), "BIT $imm", []>;

def BITi   : FF<0x24, (outs), (ins mem_a16:$addr), "BIT $addr", []>;

def BITix  : FF<0x3c, (outs), (ins mem_ix16:$addr), "BIT $addr", []>;

def STZzp  : FF<0x64, (outs), (ins mem_zp:$addr), "STZ $addr", []>;

def STZi   : FF<0x9c, (outs), (ins mem_a16:$addr), "STZ $addr", []>;

def STZix  : FF<0x9e, (outs), (ins mem_ix16:$addr), "STZ $addr", []>;

def PLA    : FF<0x68, (outs), (ins), "PLA", []>;

def PLP    : FF<0x08, (outs), (ins), "PLP", []>;

def PLX    : FF<0xfa, (outs), (ins), "PLX", []>;

def PLY    : FF<0x7a, (outs), (ins), "PLY", []>;

def PLD    : FF<0x2b, (outs), (ins), "PLD", []>;

def PHA    : FF<0x48, (outs), (ins), "PHA", []>;

def PHP    : FF<0x28, (outs), (ins), "PLP", []>;

def PHX    : FF<0xda, (outs), (ins), "PHX", []>;

def PHY    : FF<0x5a, (outs), (ins), "PHY", []>;

def PHD    : FF<0x0b, (outs), (ins), "PLD", []>;

def PEA    : FF<0xf4, (outs), (ins mem_a16:$addr), "PEA $addr", []>;

def PER    : FF<0x62, (outs), (ins brtarget8:$addr), "PER $addr", []>;

def PEI    : FF<0xd4, (outs), (ins mem_zp:$addr), "PEI $addr", []>;

def BRA    : FF<0x80, (outs), (ins brtarget8:$addr), "BRA $addr", []> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JMP    : FF<0x6c, (outs), (ins brtarget16:$addr), "JMP $addr", []> {
  let isBranch = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

def JSR    : FF<0x20, (outs), (ins mem_a16:$addr), "JSR $addr", []> {
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

def INCa   : FF<0x1a, (outs), (ins), "INC A", []>;

def DECa   : FF<0x3a, (outs), (ins), "DEC A", []>;

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

//===----------------------------------------------------------------------===//
// Zero-page register operations
//===----------------------------------------------------------------------===//

class ZPInstr<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern>;

class MOVzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src), ";MOV $dst,$src",
            [(set RC:$dst, RC:$src)]>;

def MOV8zz  : MOVzz<ZRC8,  i8>;
def MOV16zz : MOVzz<ZRC16, i16>;
def MOV32zz : MOVzz<ZRC32, i32>;
def MOV64zz : MOVzz<ZRC64, i64>;

class STZz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins), ";STZ $dst",
            [(set RC:$dst, (Ty 0))]> {
  let isMoveImm = 1;
}

def STZ8z  : STZz<ZRC8,  i8>;
def STZ16z : STZz<ZRC16, i16>;
def STZ32z : STZz<ZRC32, i32>;
def STZ64z : STZz<ZRC64, i64>;

class STzi<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs), (ins RC:$src, mem_a16:$mem), ";STORE $src,$mem",
            [(store RC:$src, tglobaladdr:$mem)]> {
  let mayStore = 1;
}

def ST8zi  : STzi<ZRC8,  i8>;
def ST16zi : STzi<ZRC16, i16>;
def ST32zi : STzi<ZRC32, i32>;
def ST64zi : STzi<ZRC64, i64>;

class LDzi<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins mem_a16:$mem), ";LOAD $dst,$mem",
            [(set RC:$dst, (load tglobaladdr:$mem))]> {
  let mayLoad = 1;
}

def LD8zi  : LDzi<ZRC8,  i8>;
def LD16zi : LDzi<ZRC16, i16>;
def LD32zi : LDzi<ZRC32, i32>;
def LD64zi : LDzi<ZRC64, i64>;

// SDNode - shl
// ValueType - i64
// RCClass - I64Regs
// Operand - simm130p

class LDzimm<RegisterClass RC, ValueType Ty, Operand immOp>
  : ZPInstr<(outs RC:$dst), (ins immOp:$imm), ";LOAD $dst,$imm",
            [(set RC:$dst, imm:$imm)]> {
  let isMoveImm = 1;
}

def LD8zimm  : LDzimm<ZRC8,  i8,  i8imm>;
def LD16zimm : LDzimm<ZRC16, i16, i16imm>;
def LD32zimm : LDzimm<ZRC32, i32, i32imm>;
def LD64zimm : LDzimm<ZRC64, i64, i64imm>;

class ANDzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";AND $dst,$src1,$src2",
            [(set RC:$dst, (and RC:$src1, RC:$src2))]>;

def AND8zz  : ANDzz<ZRC8,  i8>;
def AND16zz : ANDzz<ZRC16, i16>;
def AND32zz : ANDzz<ZRC32, i32>;
def AND64zz : ANDzz<ZRC64, i64>;

class ORzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";OR $dst,$src1,$src2",
            [(set RC:$dst, (or RC:$src1, RC:$src2))]>;

def OR8zz  : ORzz<ZRC8,  i8>;
def OR16zz : ORzz<ZRC16, i16>;
def OR32zz : ORzz<ZRC32, i32>;
def OR64zz : ORzz<ZRC64, i64>;

class XORzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";XOR $dst,$src1,$src2",
            [(set RC:$dst, (xor RC:$src1, RC:$src2))]>;

def XOR8zz  : XORzz<ZRC8,  i8>;
def XOR16zz : XORzz<ZRC16, i16>;
def XOR32zz : XORzz<ZRC32, i32>;
def XOR64zz : XORzz<ZRC64, i64>;

class ADDzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";ADD $dst,$src1,$src2",
            [(set RC:$dst, (add RC:$src1, RC:$src2))]>;

def ADD8zz  : ADDzz<ZRC8,  i8>;
def ADD16zz : ADDzz<ZRC16, i16>;
def ADD32zz : ADDzz<ZRC32, i32>;
def ADD64zz : ADDzz<ZRC64, i64>;

class SUBzz<RegisterClass RC, ValueType Ty>
  : ZPInstr<(outs RC:$dst), (ins RC:$src1, RC:$src2), ";SUB $dst,$src1,$src2",
            [(set RC:$dst, (sub RC:$src1, RC:$src2))]>;

def SUB8zz  : SUBzz<ZRC8,  i8>;
def SUB16zz : SUBzz<ZRC16, i16>;
def SUB32zz : SUBzz<ZRC32, i32>;
def SUB64zz : SUBzz<ZRC64, i64>;
