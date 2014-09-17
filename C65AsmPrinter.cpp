//===-- C65AsmPrinter.cpp - C65 LLVM assembly writer ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to C65 assembly language.
//
//===----------------------------------------------------------------------===//

#include "C65.h"
#include "C65InstrInfo.h"
#include "C65TargetMachine.h"
#include "InstPrinter/C65InstPrinter.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfoImpls.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "c65-asm-printer"

namespace {
  class C65AsmPrinter : public AsmPrinter {
  public:
    explicit C65AsmPrinter(TargetMachine &TM, MCStreamer &Streamer)
      : AsmPrinter(TM, Streamer) {}

    const char *getPassName() const override {
      return "C65 Assembly Printer";
    }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);

    void printMemOperandAbs(const MachineInstr *MI, int opNum,
                            raw_ostream &OS, const char *Modifier = nullptr);
    void printMemOperandIndex(const MachineInstr *MI, int opNum,
                              raw_ostream &OS, const char *Modifier = nullptr);

    MCOperand LowerSymbolOperand(const MachineOperand &MO);

    MCOperand LowerOperand(const MachineOperand &MO);

    void LowerC65MachineInstrToMCInst(const MachineInstr *MI,
                                      MCInst &OutMI);

    static const char *getRegisterName(unsigned RegNo) {
      return C65InstPrinter::getRegisterName(RegNo);
    }

    uint8_t getZRAddress(MachineOperand Val) const;

    MCInstBuilder AddGA(MCInstBuilder MIB,
                        const MachineOperand &MO,
                        unsigned Offset);

    MCInstBuilder AddZR(MCInstBuilder MIB,
                        const MachineOperand &MO,
                        unsigned Offset);

    void EmitBR_CC(const MachineInstr *MI, unsigned NumBytes);

    void EmitShift(const MachineInstr *MI, unsigned NumBytes, bool Right);

    void EmitShiftOne(const MachineOperand &MO, unsigned NumBytes, bool Right);

    void EmitInstructionFinal(MCInst &Inst);

    virtual void EmitInstruction(const MachineInstr *) override;

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         unsigned AsmVariant, const char *ExtraCode,
                         raw_ostream &O) override;

    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               unsigned AsmVariant, const char *ExtraCode,
                               raw_ostream &O) override;
  };
} // end of anonymous namespace

// static void EmitCall(MCStreamer &OutStreamer,
//                      MCOperand &Callee,
//                      const MCSubtargetInfo &STI) {
//   MCInst CallInst;
//   CallInst.setOpcode(C65::CALL);
//   CallInst.addOperand(Callee);
//   OutStreamer.EmitInstruction(CallInst, STI);
// }

// static void EmitBinary(MCStreamer &OutStreamer, unsigned Opcode,
//                        MCOperand &RS1, MCOperand &Src2, MCOperand &RD,
//                        const MCSubtargetInfo &STI)
// {
//   MCInst Inst;
//   Inst.setOpcode(Opcode);
//   Inst.addOperand(RD);
//   Inst.addOperand(RS1);
//   Inst.addOperand(Src2);
//   OutStreamer.EmitInstruction(Inst, STI);
// }

// static void EmitOR(MCStreamer &OutStreamer,
//                    MCOperand &RS1, MCOperand &Imm, MCOperand &RD,
//                    const MCSubtargetInfo &STI) {
//   EmitBinary(OutStreamer, SP::ORri, RS1, Imm, RD, STI);
// }

// static void EmitADD(MCStreamer &OutStreamer,
//                     MCOperand &RS1, MCOperand &RS2, MCOperand &RD,
//                     const MCSubtargetInfo &STI) {
//   EmitBinary(OutStreamer, SP::ADDrr, RS1, RS2, RD, STI);
// }

// static void EmitSHL(MCStreamer &OutStreamer,
//                     MCOperand &RS1, MCOperand &Imm, MCOperand &RD,
//                     const MCSubtargetInfo &STI) {
//   EmitBinary(OutStreamer, SP::SLLri, RS1, Imm, RD, STI);
// }


// static void EmitHiLo(MCStreamer &OutStreamer,  MCSymbol *GOTSym,
//                      C65MCExpr::VariantKind HiKind,
//                      C65MCExpr::VariantKind LoKind,
//                      MCOperand &RD,
//                      MCContext &OutContext,
//                      const MCSubtargetInfo &STI) {

//   MCOperand hi = createC65MCOperand(HiKind, GOTSym, OutContext);
//   MCOperand lo = createC65MCOperand(LoKind, GOTSym, OutContext);
//   EmitSETHI(OutStreamer, hi, RD, STI);
//   EmitOR(OutStreamer, RD, lo, RD, STI);
// }

MCOperand C65AsmPrinter::LowerSymbolOperand(const MachineOperand &MO) {
  const MCSymbol *Symbol = nullptr;
  bool HasOffset = true;

  switch(MO.getType()) {
  default:
    llvm_unreachable("Unknown type in LowerSymbolOperand");

  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    HasOffset = 0;
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = getSymbol(MO.getGlobal());
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = GetBlockAddressSymbol(MO.getBlockAddress());
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = GetExternalSymbolSymbol(MO.getSymbolName());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = GetCPISymbol(MO.getIndex());
    break;

  case MachineOperand::MO_JumpTableIndex:
    Symbol = GetJTISymbol(MO.getIndex());
    HasOffset = false;
    break;
  }

  const MCExpr *Expr = MCSymbolRefExpr::Create(Symbol, OutContext);
  if (HasOffset) {
    if (int64_t Offset = MO.getOffset()) {
      const MCExpr *OffsetExpr = MCConstantExpr::Create(Offset, OutContext);
      Expr = MCBinaryExpr::CreateAdd(Expr, OffsetExpr, OutContext);
    }
  }
  return MCOperand::CreateExpr(Expr);
}

MCOperand C65AsmPrinter::LowerOperand(const MachineOperand &MO) {
  switch(MO.getType()) {
  default:
    llvm_unreachable("unknown operand type"); break;

  case MachineOperand::MO_Register:
    if (MO.isImplicit())
      break;
    return MCOperand::CreateReg(MO.getReg());

  case MachineOperand::MO_Immediate:
    return MCOperand::CreateImm(MO.getImm());

  case MachineOperand::MO_MachineBasicBlock:
  case MachineOperand::MO_GlobalAddress:
  case MachineOperand::MO_BlockAddress:
  case MachineOperand::MO_ExternalSymbol:
  case MachineOperand::MO_ConstantPoolIndex:
  case MachineOperand::MO_JumpTableIndex:
    return LowerSymbolOperand(MO);

  case MachineOperand::MO_RegisterMask:
    break;
  }
  return MCOperand();
}

void C65AsmPrinter::LowerC65MachineInstrToMCInst(const MachineInstr *MI,
                                                 MCInst &OutMI) {

  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}

/// Additions and subtractions require a clear carry
///
static bool getOpRequiresCLC(unsigned Op) {
  switch (Op) {
  default: return false;
  case C65::ADD8zz:
  case C65::ADD16zz:
  case C65::ADD32zz:
  case C65::ADD64zz:
  case C65::SUB8zz:
  case C65::SUB16zz:
  case C65::SUB32zz:
  case C65::SUB64zz:
    return true;
  }
}

/// Get the operand size of the function.
///
static unsigned getOpByteSize(const MachineInstr *MI) {
  return 1 << C65::getZROpSize(MI->getDesc().TSFlags);
}

/// Shift-type instructions are emitted with a special form
///
static bool isInstrShiftType(const MachineInstr *MI) {
  return MI->getDesc().TSFlags & C65::ZRShift;
}

/// Control flow instructions are emitted with a special form
///
static bool isInstrCtrlType(const MachineInstr *MI) {
  return MI->getDesc().TSFlags & C65::ZRCtrl;
}

/// ZR instructions are treated separately
///
static bool isZRInstr(const MachineInstr *MI) {
  return MI->getDesc().TSFlags & C65::ZRInstr;
}

/// Post-Z-instruction emit
///
void C65AsmPrinter::EmitInstructionFinal(MCInst &Inst) {
  OutStreamer.EmitInstruction(Inst, getSubtargetInfo());
}

/// Add global address with an offset as an operator
///
// MCInstBuilder C65AsmPrinter::AddGA(MCInstBuilder MIB,
//                                    const MachineOperand &MO,
//                                    unsigned Offset) {
//   ((MCInst&)MIB).addOperand
//     (LowerOperand
//      (MachineOperand::CreateGA(MO.getGlobal(), MO.getOffset() + Offset)));
//   return MIB;
// }

// /// Add ZR operator
// ///
// MCInstBuilder C65AsmPrinter::AddZR(MCInstBuilder MIB,
//                                    const MachineOperand &MO,
//                                    unsigned Offset) {
//   const C65RegisterInfo &TRI =
//     *static_cast<const C65RegisterInfo *>
//     (TM.getSubtargetImpl()->getRegisterInfo());
//   unsigned Addr = TRI.getZRAddress(MO.getReg());
//   MIB.addImm(Addr + Offset);
//   return MIB;
// }

// /// Emit a shift-by-one
// ///
// void C65AsmPrinter::EmitShiftOne(const MachineOperand &MO,
//                                  unsigned NumBytes,
//                                  bool Right) {
//   const unsigned ByteCapacity = 2;
//   unsigned FirstOpCode = Right ? C65::LSRzp : C65::ASLzp;
//   unsigned FirstOffset;
//   if (Right) {
//     FirstOffset = NumBytes == 1 ? 0 : NumBytes - ByteCapacity;
//   } else {
//     FirstOffset = 0;
//   }
//   EmitInstructionFinal
//     (AddZR(MCInstBuilder(FirstOpCode), MO, FirstOffset));
//   if (Right) {
//     for (int I = FirstOffset - ByteCapacity; I >= 0;
//          I -= ByteCapacity) {
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::RORzp), MO, I));
//     }
//   } else {
//     for (unsigned I = FirstOffset + ByteCapacity; I < NumBytes;
//          I += ByteCapacity) {
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::ROLzp), MO, I));
//     }
//   }
// }

// /// Emit a shift-by-N
// ///
// void C65AsmPrinter::EmitShift(const MachineInstr *MI,
//                               unsigned NumBytes,
//                               bool Right) {
//   const MachineOperand &Count = MI->getOperand(2);
//   if (NumBytes == 1) {
//     EmitInstructionFinal(MCInstBuilder(C65::SEP).addImm(0x20));
//   }
//   if (Count.isReg()) {
//     EmitInstructionFinal
//       (AddZR(MCInstBuilder(C65::LDXzp), Count, 0));
//     // ... FIXME!
//   } else {
//     assert(Count.isImm());
//     for (unsigned I = 0, E = Count.getImm(); I != E; ++I) {
//       EmitShiftOne(MI->getOperand(1), NumBytes, Right);
//     }
//   }
//   if (NumBytes == 1) {
//     EmitInstructionFinal(MCInstBuilder(C65::REP).addImm(0x20));
//   }
// }

// struct Comparison {
//   // The operands to the comparison.
//   const MachineOperand Op0, Op1;

//   // The opcode that should be used to compare Op0 and Op1.
//   bool Equality;

//   // Is signed comparison
//   bool Signed;

//   // The result on which we will trigger a branch:
//   //   Branch on bitresult == Z flag for equality
//   //   Branch on bitresult == N flag for signed
//   //   Branch on bitresult == C flag for unsigned
//   bool Bitvalue;
// };

// ///
// ///
// static struct Comparison getComparison(const MachineInstr *MI) {
//   ISD::CondCode CC = (ISD::CondCode)MI->getOperand(0).getImm();
//   const MachineOperand Op0 = MI->getOperand(1);
//   const MachineOperand Op1 = MI->getOperand(2);

//   switch (CC) { //           Op0  Op1  Equality Signed Bitvalue
//   case ISD::SETEQ:  return { Op0, Op1, true,    false, true };
//   case ISD::SETNE:  return { Op0, Op1, true,    false, false };
//   case ISD::SETLT:  return { Op1, Op0, false,   true,  false };
//   case ISD::SETLE:  return { Op0, Op1, false,   true,  true };
//   case ISD::SETGT:  return { Op0, Op1, false,   true,  false };
//   case ISD::SETGE:  return { Op1, Op0, false,   true,  true };
//   case ISD::SETULT: return { Op0, Op1, false,   false, true };
//   case ISD::SETULE: return { Op1, Op0, false,   false, false };
//   case ISD::SETUGT: return { Op1, Op0, false,   false, true };
//   case ISD::SETUGE: return { Op0, Op1, false,   false, false };
//   default:
//     llvm_unreachable("Cannot emit this type of comparison!");
//   }
// }

// /// Emit a BR_CC
// ///
// void C65AsmPrinter::EmitBR_CC(const MachineInstr *MI,
//                               unsigned NumBytes) {
//   const unsigned ByteCapacity = 2;
//   struct Comparison C = getComparison(MI);
//   const MachineOperand &Dest = MI->getOperand(3);

//   if (NumBytes == 1) {
//     EmitInstructionFinal(MCInstBuilder(C65::SEP).addImm(0x20));
//   }
//   if (C.Equality) {
//     for (unsigned I = 0; I < NumBytes; I += ByteCapacity) {
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::LDAzp), C.Op0, I));
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::CMPzp), C.Op1, I));
//       //      EmitInstructionFinal(MCInstBuilder(C65::BNE).addExpr(Dest));
//     }
//     // FIXME
//     if (NumBytes == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::REP).addImm(0x20));
//     }
//     if (C.Bitvalue) {
//       MCInstBuilder MIB = MCInstBuilder(C65::BNE);
//       ((MCInst&)MIB).addOperand(LowerOperand(Dest));
//       EmitInstructionFinal(MIB);
//     } else {
//       MCInstBuilder MIB = MCInstBuilder(C65::BEQ);
//       ((MCInst&)MIB).addOperand(LowerOperand(Dest));
//       EmitInstructionFinal(MIB);
//     }
//   } else if (C.Signed) {
//     EmitInstructionFinal
//       (AddZR(MCInstBuilder(C65::LDAzp), C.Op0, 0));
//     EmitInstructionFinal
//       (AddZR(MCInstBuilder(C65::CMPzp), C.Op1, 0));

//     for (unsigned I = ByteCapacity; I < NumBytes; I += ByteCapacity) {
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::LDAzp), C.Op0, I));
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::SBCzp), C.Op1, I));
//     }

//     // FIXME
//     EmitInstructionFinal(MCInstBuilder(C65::BVC).addImm(6));

//     if (NumBytes == 1 || ByteCapacity == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::EOR_8imm).addImm(0x80));
//     } else {
//       EmitInstructionFinal(MCInstBuilder(C65::EOR_16imm).addImm(0x8000));
//     }
//     if (NumBytes == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::REP).addImm(0x20));
//     }
//     if (C.Bitvalue) {
//       //      EmitInstructionFinal(MCInstBuilder(C65::BMI).addExpr(Dest));
//     } else {
//       //      EmitInstructionFinal(MCInstBuilder(C65::BPL).addExpr(Dest));
//     }
//   } else {
//     EmitInstructionFinal
//       (AddZR(MCInstBuilder(C65::LDAzp), C.Op0, 0));
//     EmitInstructionFinal
//       (AddZR(MCInstBuilder(C65::CMPzp), C.Op1, 0));

//     for (unsigned I = ByteCapacity; I < NumBytes; I += ByteCapacity) {
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::LDAzp), C.Op0, I));
//       EmitInstructionFinal
//         (AddZR(MCInstBuilder(C65::SBCzp), C.Op1, I));
//     }
//     if (NumBytes == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::REP).addImm(0x20));
//     }
//     if (C.Bitvalue) {
//       //      EmitInstructionFinal(MCInstBuilder(C65::BCS).addExpr(Dest));
//     } else {
//       //      EmitInstructionFinal(MCInstBuilder(C65::BCC).addExpr(Dest));
//     }
//   }
// }

/// Emit an instruction
///
void C65AsmPrinter::EmitInstruction(const MachineInstr *MI) {
  unsigned NumBytes = getOpByteSize(MI);
  if (MI->getOpcode() == TargetOpcode::DBG_VALUE) {
    // FIXME: Debug Value.
    return;
  } else if (!isZRInstr(MI)) {
    // Normal instruction, emitted as-is
    MCInst TmpInst;
    LowerC65MachineInstrToMCInst(MI, TmpInst);
    EmitToStreamer(OutStreamer, TmpInst);
  }
}

//  else {
//     // Emit the comment
//     MCInst TmpInst;
//     LowerC65MachineInstrToMCInst(MI, TmpInst);
//     EmitToStreamer(OutStreamer, TmpInst);

//     if (NumBytes == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::SEP)
//         .addImm(0x20));
//     }
//     if (getOpRequiresCLC(MI->getOpcode())) {
//       EmitInstructionFinal(MCInstBuilder(C65::CLC));
//     }
//     if (isInstrCtrlType(MI)) {
//       switch (MI->getOpcode()) {
//       case C65::BRCC8zz:
//       case C65::BRCC16zz:
//       case C65::BRCC32zz:
//       case C65::BRCC64zz:
//         break;
//         // EmitBR_CC(MI, NumBytes);
//       }
//     } else if (isInstrShiftType(MI)) {
//       switch (MI->getOpcode()) {
//       default: llvm_unreachable("Unknown Z shift instruction.");
//       case C65::SHL8zimm:
//       case C65::SHL16zimm:
//       case C65::SHL32zimm:
//       case C65::SHL64zimm:
//       case C65::SHL8zz:
//       case C65::SHL16zz:
//       case C65::SHL32zz:
//       case C65::SHL64zz: {
//         EmitShift(MI, NumBytes, false);
//       } break;
//       case C65::LSHR8zimm:
//       case C65::LSHR16zimm:
//       case C65::LSHR32zimm:
//       case C65::LSHR64zimm:
//       case C65::LSHR8zz:
//       case C65::LSHR16zz:
//       case C65::LSHR32zz:
//       case C65::LSHR64zz: {
//         EmitShift(MI, NumBytes, true);
//       } break;
//       }
//     } else {
//       for (unsigned I = 0; I < NumBytes; I += 2) {
//         switch (MI->getOpcode()) {
//         default: llvm_unreachable("Unknown Z instruction.");
//         case C65::STZ8z:
//         case C65::STZ16z:
//         case C65::STZ32z:
//         case C65::STZ64z: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STZzp), MI->getOperand(0), I));
//         } break;
//         case C65::ST8zi:
//         case C65::ST16zi:
//         case C65::ST32zi:
//         case C65::ST64zi: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(0), I));
//           EmitInstructionFinal
//             (AddGA(MCInstBuilder(C65::STAi), MI->getOperand(1), I));
//         } break;
//         case C65::LD8zi:
//         case C65::LD16zi:
//         case C65::LD32zi:
//         case C65::LD64zi: {
//           EmitInstructionFinal
//             (AddGA(MCInstBuilder(C65::LDAi), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::LD8zimm:
//         case C65::LD16zimm:
//         case C65::LD32zimm:
//         case C65::LD64zimm: {
//           MCInst LDAInst, STAInst;
//           unsigned Value = MI->getOperand(1).getImm() >> (16 * I) & 0xFFFF;
//           if (Value == 0) {
//             EmitInstructionFinal
//               (AddZR(MCInstBuilder(C65::STZzp), MI->getOperand(0), I));
//           } else {
//             EmitInstructionFinal
//               (MCInstBuilder(C65::LDAimm).addImm(Value));
//             EmitInstructionFinal
//               (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//           }
//         } break;
//         case C65::AND8zz:
//         case C65::AND16zz:
//         case C65::AND32zz:
//         case C65::AND64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::ANDzp), MI->getOperand(2), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::OR8zz:
//         case C65::OR16zz:
//         case C65::OR32zz:
//         case C65::OR64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::ORAzp), MI->getOperand(2), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::XOR8zz:
//         case C65::XOR16zz:
//         case C65::XOR32zz:
//         case C65::XOR64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::EORzp), MI->getOperand(2), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::ADD8zz:
//         case C65::ADD16zz:
//         case C65::ADD32zz:
//         case C65::ADD64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::ADCzp), MI->getOperand(2), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::SUB8zz:
//         case C65::SUB16zz:
//         case C65::SUB32zz:
//         case C65::SUB64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::SBCzp), MI->getOperand(2), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         case C65::MOV8zz:
//         case C65::MOV16zz:
//         case C65::MOV32zz:
//         case C65::MOV64zz: {
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::LDAzp), MI->getOperand(1), I));
//           EmitInstructionFinal
//             (AddZR(MCInstBuilder(C65::STAzp), MI->getOperand(0), I));
//         } break;
//         }
//       }
//     }
//     if (NumBytes == 1) {
//       EmitInstructionFinal(MCInstBuilder(C65::REP)
//         .addImm(0x20));
//     }
//   }
// }

void C65AsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                 raw_ostream &O) {
  const DataLayout *DL = TM.getSubtargetImpl()->getDataLayout();
  const MachineOperand &MO = MI->getOperand(opNum);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << StringRef(getRegisterName(MO.getReg()));
    break;
  case MachineOperand::MO_Immediate:
    O << (int)MO.getImm();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    O << *MO.getMBB()->getSymbol();
    return;
  case MachineOperand::MO_GlobalAddress:
    O << *getSymbol(MO.getGlobal());
    break;
  case MachineOperand::MO_BlockAddress:
    O << GetBlockAddressSymbol(MO.getBlockAddress())->getName();
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << MO.getSymbolName();
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    O << DL->getPrivateGlobalPrefix() << "CPI" << getFunctionNumber() << "_"
      << MO.getIndex();
    break;
  default:
    llvm_unreachable("<unknown operand type>");
  }
}

bool C65AsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                    unsigned AsmVariant,
                                    const char *ExtraCode,
                                    raw_ostream &O) {
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) {
      // Unknown modifier.
      return true;
    } else {
      return AsmPrinter::PrintAsmOperand(MI, OpNo, AsmVariant, ExtraCode, O);
    }
  } else {
    printOperand(MI, OpNo, O);
    return false;
  }
}

bool C65AsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                          unsigned OpNo, unsigned AsmVariant,
                                          const char *ExtraCode,
                                          raw_ostream &O) {
  if (ExtraCode && ExtraCode[0]) {
    // Unknown modifier
    return true;
  } else {
    printOperand(MI, OpNo, O);
    return false;
  }
}

// Force static initialization.
extern "C" void LLVMInitializeC65AsmPrinter() {
  RegisterAsmPrinter<C65AsmPrinter> X(The65C816Target);
}
