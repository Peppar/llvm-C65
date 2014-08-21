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

// Function return node
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
// C65 operand class definitions
//===----------------------------------------------------------------------===//

// 8-bit memory operand class
def MEMi : Operand<iPTR> {
  let PrintMethod = "printMemOperand";
  let MIOperandInfo = (ops i16imm);
}

//===----------------------------------------------------------------------===//
// Format 01 instructions : ora, and, eor, adc, sta, lda, cmp, sbc
//===----------------------------------------------------------------------===//

class F01<bits<3> op, bits<3> addrmode, dag outs, dag ins,
          string asmstr, list<dag> pattern>
    : Fcc<op, addrmode, 0b01, outs, ins, asmstr, pattern>;

let Constraints = "$src = $dst" in {
  def ORAimm : F01<0b000, 0b010, (outs ACC8:$dst), (ins ACC8:$src, i8imm:$imm),
                   "ORA $imm",
                   [(set ACC8:$dst, (or ACC8:$src, imm:$imm))]>;
  def ANDimm : F01<0b001, 0b010, (outs ACC8:$dst), (ins ACC8:$src, i8imm:$imm),
                   "AND $imm",
                   [(set ACC8:$dst, (and ACC8:$src, imm:$imm))]>;
  def EORimm : F01<0b010, 0b010, (outs ACC8:$dst), (ins ACC8:$src, i8imm:$imm),
                   "EOR $imm",
                   [(set ACC8:$dst, (xor ACC8:$src, imm:$imm))]>;
  let mayLoad = 1 in {
    def EORi   : F01<0b010, 0b011, (outs ACC8:$dst), (ins ACC8:$src, MEMi:$addr),
                     "EOR $addr",
                     [(set ACC8:$dst, (xor ACC8:$src,
                                            (load tglobaladdr:$addr)))]>;
    def ANDi   : F01<0b001, 0b011, (outs ACC8:$dst), (ins ACC8:$src, MEMi:$addr),
                     "AND $addr",
                     [(set ACC8:$dst, (and ACC8:$src,
                                            (load tglobaladdr:$addr)))]>;
    def ORAi   : F01<0b000, 0b011, (outs ACC8:$dst), (ins ACC8:$src, MEMi:$addr),
                     "ORA $addr",
                     [(set ACC8:$dst, (or ACC8:$src,
                                           (load tglobaladdr:$addr)))]>;
  }
}

let mayStore = 1 in {
  def STAi   : F01<0b100, 0b011, (outs), (ins ACC8:$src, MEMi:$addr),
                   "STA $addr",
                   [(store ACC8:$src, tglobaladdr:$addr)]>;
}

let mayLoad = 1 in {
  def LDAi   : F01<0b101, 0b011, (outs ACC8:$dst), (ins MEMi:$addr),
                   "LDA $addr",
                   [(set ACC8:$dst, (load tglobaladdr:$addr))]>;
}

let isMoveImm = 1 in {
  def LDAimm : F01<0b101, 0b010, (outs ACC8:$dst), (ins i8imm:$imm),
                   "LDA $imm",
                   [(set ACC8:$dst, imm:$imm)]>;
}

//===----------------------------------------------------------------------===//
// Format-free instructions (not following an apparent binary pattern)
//===----------------------------------------------------------------------===//

class FF<bits<8> op, dag outs, dag ins,
         string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let Inst = op;
}

// Return from subroutine
def RTS : FF<0x60, (outs), (ins), "RTS", [(C65ret)]> {
  let isReturn = 1;
  let isTerminator = 1;
  let isBarrier = 1;
}

//===----------------------------------------------------------------------===//
// Pseudo-instructions
//===----------------------------------------------------------------------===//

class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : InstC65<outs, ins, asmstr, pattern> {
  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

// ADJCALLSTACKDOWN, ADJCALLSTACKUP are mandatory pseudo-instructions
def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i32imm:$amt),
                               ";!ADJCALLSTACKDOWN $amt",
                               [(callseq_start timm:$amt)]>;
def ADJCALLSTACKUP   : Pseudo<(outs), (ins i32imm:$amt1, i32imm:$amt2),
                               ";!ADJCALLSTACKUP $amt1",
                               [(callseq_end timm:$amt1, timm:$amt2)]>;
