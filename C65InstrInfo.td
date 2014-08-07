//===-- C65InstrInfo.td - Main C65 Instruction Definition --*- tablegen -*-===//

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

//===----------------------------------------------------------------------===//
// C65-specific node definitions
//===----------------------------------------------------------------------===//

//def C65cmp : SDNode<"C65ISD::CMP", SDT_C65cmp, [SDNPOutGlue]>;
//def C65jsr : SDNode<"C65ISD::JSR", SDT_54

def SDTC65cmp : SDTypeProfile<0, 2, [SDTCisInt<0>, SDTCisSameAs<0, 1>]>;

def C65cmp : SDNode<"C65ISD::CMP", SDTC65cmp, [SDNPOutGlue]>;


//===----------------------------------------------------------------------===//
// C65 operand definitions
//===----------------------------------------------------------------------===//

// Immediate
def MEMi : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops i16imm);
}

// Immediate + X
def MEMix : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops IX16, i16imm);
}

def brtarget : Operand<OtherVT> {
  let PrintMethod = "printMemOperand";
}

def calltarget : Operand<OtherVT> {
  let PrintMethod = "printMemOperand";
}

//===----------------------------------------------------------------------===//
// C65 complex pattern definitions
//===----------------------------------------------------------------------===//

// Address pattern definitions
def ADDRi : ComplexPattern<iPTR, 2, "SelectADDRi", [], []>;
def ADDRix : ComplexPattern<iPTR, 2, "SelectADDRix", [], []>;

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
// C65 instruction predicate definitions
//===----------------------------------------------------------------------===//

def Has65C02 : Predicate<"Subtarget->has65C02()">;
def Has65C816 : Predicate<"Subtarget->has65C816()">;

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern>;

multiclass F01_binary<bits<3> op, string asmstr, SDPatternOperator operator> {
  // Binary operation on 16-bit accumulator
  def imm  : F01<op, 0b010, (outs ACC16:$dst), (ins ACC16:$src, imm:$I1),
                 !strconcat(asmstr, " #$I1"),
                 [(set ACC16:$dst, (operator ACC16:$src, imm:$I1))]>;

  def i  : F01<op, 0b011, (outs ACC16:$dst), (ins ACC16:$src, MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set ACC16:$dst, (operator ACC16:$src, ADDRi:$addr))]>;

  def ix : F01<op, 0b110, (outs ACC16:$dst), (ins ACC16:$src, MEMix:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(set ACC16:$dst, (operator ACC16:$src, ADDRix:$addr))]>;
}

multiclass F01_load<bits<3> op, string asmstr> {
  // Load accumulator
  def imm  : F01<op, 0b010, (outs ACC16:$dst), (ins imm:$I1),
                 !strconcat(asmstr, " #$I1"),
                 [(set ACC16:$dst, imm:$I1)]>;

  def i  : F01<op, 0b011, (outs ACC16:$dst), (ins MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(set ACC16:$dst, (load ADDRi:$addr))]>;

  def ix : F01<op, 0b110, (outs ACC16:$dst), (ins MEMix:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(set ACC16:$dst, (load ADDRix:$addr))]>;
}

multiclass F01_store<bits<3> op, string asmstr> {
  // Store accumulator
  def i  : F01<op, 0b011, (outs ACC16:$dst), (ins MEMi:$addr),
                 !strconcat(asmstr, " $addr"),
                 [(store ACC16:$dst, ADDRi:$addr)]>;

  def ix : F01<op, 0b110, (outs ACC16:$dst), (ins MEMix:$addr),
                 !strconcat(asmstr, " $addr,X"),
                 [(store ACC16:$dst, ADDRix:$addr)]>;
}

def CMPimm : F01<0b110, 0b010, (outs), (ins imm:$I1),
                 "cmp #$I1",
                 [(C65cmp ACC16:$src, imm:$I1)]>;

def CMPi   : F01<0b110, 0b011, (outs), (ins MEMi:$addr),
                 "cmp $addr",
                 [(C65cmp ACC16:$src, ADDRi:$addr)]>;

def CMPix  : F01<0b110, 0b011, (outs), (ins MEMix:$addr),
                 "cmp $addr,X",
                 [(C65cmp ACC16:$src, ADDRix:$addr)]>;

let Defs = [SR] in {
  defm STA : F01_store<0b100, "sta">;
  defm LDA : F01_load<0b101, "lda">;
  let Constraints = "$src = $dst" in {
    defm ORA : F01_binary<0b000, "ora", or>;
    defm AND : F01_binary<0b001, "and", and>;
    defm EOR : F01_binary<0b010, "eor", xor>;
    let Uses = [SR] in {
      defm ADC : F01_binary<0b011, "adc", addc>;
      defm SBC : F01_binary<0b111, "adc", subc>;
    }
  }
}

//===----------------------------------------------------------------------===//
// Format 10 instructions : asl, rol, lsr, ror, stx, ldx, dec, inc

class F10<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b10, outs, ins, asmstr, pattern>;

multiclass F10_unary<bits<3> op, string asmstr, SDPatternOperator operator> {
  // Unary operation
  def i  : F10<op, 0b011, (outs MEMi:$addr), (ins MEMi:$addr),
               !strconcat(asmstr, " $addr"),
               [(set ADDRi:$addr, (operator ADDRi:$addr))]>;

  def ix : F10<op, 0b110, (outs MEMix:$addr), (ins MEMix:$addr),
               !strconcat(asmstr, " $addr,X"),
               [(set ADDRix:addr, (operator ADDRix:$addr))]>;
}

let Defs = [SR], Uses = [SR] in {
  defm ASL : F10_unary<0b000, "asl", shl>;
  defm ROL : F10_unary<0b001, "rol", rotl>;
  defm LSR : F10_unary<0b010, "lsr", srl>;
  defm ROR : F10_unary<0b011, "ror", rotr>;

  //let Constraints = "$src = $dst" in {
  //def ASLa : F10<0b000, 0b010, (outs ACC16:$dst, "asl a",
  //}
}

let Defs = [SR] in {
  def LDXimm : F10<0b101, 0b000, (outs IX16:$dst), (ins imm:$I1),
                   "ldx #$I1",
                   [(set IX16:$dst, imm:$I1)]>;

  def LDXi   : F10<0b101, 0b011, (outs IX16:$dst), (ins MEMi:$addr),
                   "ldx $addr",
                   [(set IX16:$dst, (load ADDRi:$addr))]>;

  def LDXiy  : F10<0b101, 0b111, (outs IX16:$dst), (ins MEMiy:$addr),
                   "ldx $addr,Y",
                   [(set IX16:$dst, (load ADDRiy:$addr))]>;
}

def STXi   : F10<0b100, 0b011, (outs MEMi:$addr), (ins IX16:$src),
                 "stx $addr",
                 [(store IX16:$src, ADDRi:$addr)]>;

//===----------------------------------------------------------------------===//
// Format 00 instructions : bit, jmp, sty, ldy, cpy, cpx

class F00<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b00, outs, ins, asmstr, pattern>;

let Defs = [SR] in {
  def LDYimm : F00<0b101, 0b000, (outs IY16:$dst), (ins imm:$I1),
                   "ldy #$I1",
                   [(set IY16:$dst, imm:$I1)]>;

  def LDYi   : F00<0b101, 0b011, (outs IY16:$dst), (ins MEMi:$addr),
                   "ldy $addr",
                   [(set IY16:$dst, ADDRi:$addr)]>;

  def LDYix  : F00<0b101, 0b111, (outs IY16:$dst), (ins MEMix:$addr),
                   "ldy $addr,X",
                   [(set IY16:$dst, ADDRix:$addr)]>;
}

def STYi : F00<0b100, 0b011, (outs MEMi:$addr), (ins IY16:$src),
               "sty $addr",
               [(store IY16:$src, ADDRi:$addr)]>;

let isBranch = 1, isTerminator = 1, isBarrier = 1 in {
  def JMPi : F00<0b011, 0b011, (outs), (ins brtarget:$dst),
                 "jmp $dst",
                 [(br bb:$dst)]>;
}

//===----------------------------------------------------------------------===//
// Conditional branch instructions : bpl, bmi, bvc, bvs, bcc, bcs, bne, beq

class CondFlag<bits<2> ix> { bits<2> flagIndex = ix; }
def CondNegative : CondFlag<0b00>;
def CondOverflow : CondFlag<0b01>;
def CondCarry    : CondFlag<0b10>;
def CondZero     : CondFlag<0b11>;

class FC<CondFlag cf, bit value, string asmstr, list<dag> pattern>
    : InstC65<(outs), (ins brtarget:$dst), asmstr, pattern> {
  let Inst{7-6} = cf.flagIndex;
  let Inst{5} = value;
  let Inst{4-0} = 0b10000;
}

let Uses = [SR] in {
  def BPL : FC<CondNegative, 0>;
  def BMI : FC<CondNegative, 1>;
  def BVC : FC<CondOverflow, 0>;
  def BVS : FC<CondOverflow, 1>;
  def BCC : FC<CondCarry, 0>;
  def BCS : FC<CondCarry, 1>;
  def BNE : FC<CondZero, 0>;
  def BEQ : FC<CondZero, 1>;
}

//===----------------------------------------------------------------------===//
// Format 11 instructions : Extended ora, and, eor, adc, sta, lda, cmp, sbc

class F11<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
   : Fcc<op, addrmode, 0b11, outs, ins, asmstr, pattern>;

//===----------------------------------------------------------------------===//
// Free format instructions

class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst = op;
}

let neverHasSideEffects = 1 in {
  def NOOP : FF<0xea, (outs), (ins), "nop", []>;
}
let Defs = [SP], Uses = [SP, SR], neverHasSideEffects = 1 in {
  let mayLoad = 1 in {
    def PLA : FF<0x68, (outs ACC16:$dst), (ins), "pla",
                 []>;
    def PLX : FF<0xfa, (outs IX16:$dst), (ins), "plx",
                 []>;
    def PLY : FF<0x7a, (outs IY16:$dst), (ins), "ply",
                 []>;
  }
  let mayStore = 1 in {
    def PHA : FF<0x48, (outs), (ins ACC16:$src), "pha",
                 []>;
    def PHX : FF<0xda, (outs), (ins IX16:$src), "phx",
                 []>;
    def PHY : FF<0x5a, (outs), (ins IY16:$src), "phy",
                 []>;
  }
}

//let isBranch = 1, isTerminator = 1, isBarrier = 1 in {
//  def JMP : FF<0x6c, (outs), (ins brtarget:$imm22), "jmp",
//               [(br bb:$imm22)]>;
//}

let isCall = 1 in {
  let Defs = [A, X, Y] in {
    def JSR : FF<0x20, (outs), (ins calltarget:$disp),
                 "jsr $disp", []>;
  }
}

let isReturn = 1, isTerminator = 1, isBarrier = 1,
     DecoderMethod = "DecodeReturn" in {
  def RTS : FF<0x60, (outs), (ins), "rts", []>;
}

//===----------------------------------------------------------------------===//
// Transfer instructions

let Uses = [SR], neverHasSideEffects = 1 in {
  def TAX : FF<0xaa, (outs IX16:$dst), (ins ACC16:$src), "tax",
               [(set IX16:$dst, ACC16:$src)]>;

  def TAY : FF<0xa8, (outs IX16:$dst), (ins ACC16:$src), "tay",
               [(set IY16:$dst, ACC16:$src)]>;

  def TXA : FF<0x8a, (outs ACC16:$dst), (ins IX16:$src), "txa",
               [(set ACC16:$dst, IX16:$src)]>;

  def TYA : FF<0xa8, (outs ACC16:$dst), (ins IY16:$src), "tya",
               [(set ACC16:$dst, IY16:$src)]>;

  def TXY : FF<0x9b, (outs IY16:$dst), (ins IX16:$src), "txy",
               [(set IY16:$dst, IX16:$src)]>,
            Requires<[Has65C816]>;

  def TYX : FF<0xbb, (outs IX16:$dst), (ins IY16:$src), "tyx",
               [(set IX16:$dst, IY16:$src)]>,
            Requires<[Has65C816]>;
}

//===----------------------------------------------------------------------===//
// Flag instructions

let Defs = [SR], Uses = [SR] in {
  def CLC : FF<0x18, (outs), (ins), "clc", []>;
  def SEC : FF<0x38, (outs), (ins), "stc", []>;
  def CLD : FF<0xd8, (outs), (ins), "cld", []>;
  def SED : FF<0xf8, (outs), (ins), "std", []>;
  def CLI : FF<0x58, (outs), (ins), "cli", []>;
  def SEI : FF<0x78, (outs), (ins), "sti", []>;
  def CLV : FF<0xb8, (outs), (ins), "clv", []>;
}

//===----------------------------------------------------------------------===//
//  Arithmetic Instructions.

let Defs = [SR] in {
let Constraints = "$src = $dst" in {
  def INCa : FF<0x1a, (outs ACC16:$dst), (ins ACC16:$src), "inc A",
               [(set ACC16:$dst, (add ACC16:$src, 1))]>,
             Requires<[Has65C02]>;

  def INX : FF<0xe8, (outs IX16:$dst), (ins IX16:$src), "inx",
               [(set IX16:$dst, (add IX16:$src, 1))]>;

  def INY : FF<0xc8, (outs IY16:$dst), (ins IY16:$src), "iny",
               [(set IY16:$dst, (add IY16:$src, 1))]>;

  def DECa : FF<0x3a, (outs :A), (ins GR16:A), "dec A",
               [(set ACC16:$dst, (sub ACC16:$src, 1))]>,
             Requires<[Has65C02]>;

  def DEX : FF<0xca, (outs GR16:A), (ins GR16:A), "dex",
               [(set IX16:$dst, (sub IX16:$src, 1))]>;

  def DEY : FF<0x88, (outs GR16:A), (ins GR16:A), "dey",
               [(set IY16:$dst, (sub IY16:$src, 1))]>;
}
}
