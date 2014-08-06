//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

def SDTUnaryArithWithFlags :

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

//class F0<dag outs, dag ins, string asmstr, list<dag> pattern>
//        : Inst65<outs, ins, asmstr, pattern> {
//
//}

//===----------------------------------------------------------------------===//
// C65


def ADDRi : ComplexPattern<iPTR, 2, "SelectADDRi", [], []>;
def ADDRix : ComplexPattern<iPTR, 2, "SelectADDRix", [], []>;

def MEMi : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops i16imm);
  let ParserMatchClass = "C65MEMiAsmOperand";
}

def MEMix : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops ptr_rc, i16imm);
  let ParserMatchClass = "C65MEMixAsmOperand";
}

// 6502 instruction format generally follows:
// aaabbbcc

// Free format instructions, does not follow pattern
class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern> {
  let Inst = op;
}

// Format 01 instructions (cc = 01)
class F01<bits<3> op2, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : InstC65<outs, ins, asmstr, pattern> {
  let Inst{7-5} = op2;
  let Inst{4-2} = addrmode;
  let Inst{1-0} = 0b01;
}

multiclass F01_binary<bits<3> op2, string asmstr, SDPatternOperator operator> {
  // Binary operation on accumulator
  def imm  : F01<op2, 0b010, (outs GR16:A), (ins imm:$I1),
                 !strconcat(asmstr, " #$I1"),
                 [(set GR16:A, (operator GR16:A, imm:$I1))]>;
  def i  : F01<op2, 0b011, (outs GR16:A), (ins MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set GR16:A, (operator GR16:A, ADDRi:$addr))]>;
  def ix : F01<op2, 0b110, (outs GR16:A), (ins MEMxi:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(set GR16:A, (operator GR16:A, ADDRix:$addr))]>;
}

multiclass F01_load<bits<3> op2, string asmstr> {
  // Load accumulator
  def imm  : F01<op2, 0b010, (outs GR16:A), (ins imm:$I1),
                 !strconcat(asmstr, " #$I1"),
                 [(set GR16:A, imm:$I1)]>;
  def i  : F01<op2, 0b011, (outs GR16:A), (ins MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set GR16:A, (load ADDRi:$addr))]>;
  def ix : F01<op2, 0b110, (outs GR16:A), (ins MEMxi:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(set GR16:A, (load ADDRix:$addr))]>;
}

multiclass F01_store<bits<3> op2, string asmstr> {
  // Store accumulator, no immediate
  def i  : F01<op2, 0b011, (outs GR16:A), (ins MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store GR16:A, ADDRi:$addr)]>;
  def ix : F01<op2, 0b110, (outs GR16:A), (ins MEMxi:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(store GR16:A, ADDRix:$addr)]>;
}

// Format 10 instructions (cc = 10)
//class F10<bits<3> op2, bits<3> addrmode, dag outs, dag ins,
//          string asmstr, list<dag> pattern>
//    : InstC65<outs, ins, asmstr, patterm> {
//  let Inst{7-5} = op2;
//  let Inst{4-2} = addrmode;
//  let Inst{1-0} = 0b10;
//}

defm ORA : F01_binary<0b000, "ora", or>;
defm AND : F01_binary<0b001, "and", and>;
defm EOR : F01_binary<0b010, "eor", xor>;
defm ADC : F01_binary<0b011, "adc", add>;
defm STA : F01_store<0b100, "sta">;
defm LDA : F01_load<0b100, "lda">;
defm SBC : F01_binary<0b111, "adc", add>;

//===----------------------------------------------------------------------===//
// C65 Operand Definitions.

def C65

let neverHasSideEffects = 1, SchedRW = [WriteZero] in {
  def NOOP : FF<0xea, (outs), (ins), "nop", []>;
}
let Defs = [SP], Uses = [SP, SR], neverHasSideEffects = 1 in {
let mayLoad, SchedRW = [WriteLoad] in {
  def PLA : FF<0x68, AddRegFrm, (outs GR16:A), (ins), "pla", []>, OpSize8;
  def PLX : I<0xfa, AddRegFrm, (outs GR16:X), (ins), "plx", []>, OpSize8;
  def PLY : I<0x7a, AddRegFrm, (outs GR16:Y), (ins), "ply", []>, OpSize8;
}
let mayStore, SchedRW = [WriteStore] in {
  def PHA : I<0x48, AddRegFrm, (outs), (ins GR16:A), "pha", []>, OpSize8;
  def PHX : I<0xda, AddRegFrm, (outs), (ins GR16:X), "phx", []>, OpSize8;
  def PHY : I<0x5a, AddRegFrm, (outs), (ins GR16:Y), "phy", []>, OpSize8;
}
}

//===----------------------------------------------------------------------===//
//  Move Instructions.
//
let Uses = [SR], SchedRW = [WriteMove] in {
let neverHasSideEffects = 1 in {
  def TAX : I<0xaa, AddRegFrm, (outs GR16:X), (ins GR16:A), "tax", []>, OpSize8;
  def TAY : I<0xa8, AddRegFrm, (outs GR16:Y), (ins GR16:A), "tay", []>, OpSize8;
  def TXA : I<0x8a, AddRegFrm, (outs GR16:A), (ins GR16:X), "txa", []>, OpSize8;
  def TYA : I<0xa8, AddRegFrm, (outs GR16:A), (ins GR16:Y), "tya", []>, OpSize8;
  def TXY : I<0x9b, AddRegFrm, (outs GR16:Y), (ins GR16:X), "txy", []>, OpSize8;
  def TYX : I<0xbb, AddRegFrm, (outs GR16:X), (ins GR16:Y), "tyx", []>, OpSize8;
}
}

//===----------------------------------------------------------------------===//
// Flag instructions
//
let Uses = [SR], SchedRW = [WriteALU] in {
  def CLC : I<0x18, RawFrm, (outs), (ins), "clc", []>, OpSize8;
  def SEC : I<0x38, RawFrm, (outs), (ins), "stc", []>, OpSize8;
  def CLD : I<0xd8, RawFrm, (outs), (ins), "cld", []>, OpSize8;
  def SED : I<0xf8, RawFrm, (outs), (ins), "std", []>, OpSize8;
  def CLI : I<0x58, RawFrm, (outs), (ins), "cli", []>, OpSize8;
  def SEI : I<0x78, RawFrm, (outs), (ins), "sti", []>, OpSize8;
  def CLV : I<0xb8, RawFrm, (outs), (ins), "clv", []>, OpSize8;
}

//===----------------------------------------------------------------------===//
//  Arithmetic Instructions.
//
let Uses = [SR] in {
let Constraints = "$src1 = $dst", SchedRW = [WriteALU] in {
  def INA : I<0x1a, RawFrm, (outs GR16:A), (ins GR16:A), "ina", []>, OpSize8;
  def INX : I<0xe8, RawFrm, (outs GR16:A), (ins GR16:A), "inx", []>, OpSize8;
  def INY : I<0xc8, RawFrm, (outs GR16:A), (ins GR16:A), "iny", []>, OpSize8;
  def DEA : I<0x3a, RawFrm, (outs GR16:A), (ins GR16:A), "dea", []>, OpSize8;
  def DEX : I<0xca, RawFrm, (outs GR16:A), (ins GR16:A), "dex", []>, OpSize8;
  def DEY : I<0x88, RawFrm, (outs GR16:A), (ins GR16:A), "dey", []>, OpSize8;
}
}

class Op
