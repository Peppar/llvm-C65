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
#include "InstPrinter/C65InstPrinter.h"
#include "C65InstrInfo.h"
#include "C65TargetMachine.h"
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

#define DEBUG_TYPE "asm-printer"

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

    void InstAddGA(MCInst &Inst, const MachineOperand &MO,
                   unsigned Offset);

    void InstAddZR(MCInst &Inst, const MachineOperand &MO,
                   unsigned Offset);

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

  switch(MO.getType()) {
  default: llvm_unreachable("Unknown type in LowerSymbolOperand");
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
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
  }

  const MCSymbolRefExpr *MCSym = MCSymbolRefExpr::Create(Symbol, OutContext);
  return MCOperand::CreateExpr(MCSym);
}

MCOperand C65AsmPrinter::LowerOperand(const MachineOperand &MO) {
  switch(MO.getType()) {
  default: llvm_unreachable("unknown operand type"); break;
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

///
static bool getOpRequireCLC(unsigned Op) {
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

/// Get the operand size of the function. This should be replaced by a
/// more declarative approach.
///
static unsigned getOpByteSize(unsigned Op) {
  switch (Op) {
  default: return 0;
  case C65::STZ8z:
  case C65::ST8zi:
  case C65::LD8zi:
  case C65::LD8zimm:
  case C65::AND8zz:
  case C65::OR8zz:
  case C65::XOR8zz:
  case C65::MOV8zz:
  case C65::ADD8zz:
  case C65::SUB8zz:
  case C65::SHL8zimm:
  case C65::SHL8zz:
  case C65::LSHR8zimm:
  case C65::LSHR8zz:
    return 1;
  case C65::STZ16z:
  case C65::ST16zi:
  case C65::LD16zi:
  case C65::LD16zimm:
  case C65::AND16zz:
  case C65::OR16zz:
  case C65::XOR16zz:
  case C65::MOV16zz:
  case C65::ADD16zz:
  case C65::SUB16zz:
  case C65::SHL16zimm:
  case C65::SHL16zz:
  case C65::LSHR16zimm:
  case C65::LSHR16zz:
    return 2;
  case C65::STZ32z:
  case C65::ST32zi:
  case C65::LD32zi:
  case C65::LD32zimm:
  case C65::AND32zz:
  case C65::OR32zz:
  case C65::XOR32zz:
  case C65::MOV32zz:
  case C65::ADD32zz:
  case C65::SUB32zz:
  case C65::SHL32zimm:
  case C65::SHL32zz:
  case C65::LSHR32zimm:
  case C65::LSHR32zz:
    return 4;
  case C65::STZ64z:
  case C65::ST64zi:
  case C65::LD64zi:
  case C65::LD64zimm:
  case C65::AND64zz:
  case C65::OR64zz:
  case C65::XOR64zz:
  case C65::MOV64zz:
  case C65::ADD64zz:
  case C65::SUB64zz:
  case C65::SHL64zimm:
  case C65::SHL64zz:
  case C65::LSHR64zimm:
  case C65::LSHR64zz:
    return 8;
  }
}

void C65AsmPrinter::EmitInstructionFinal(MCInst &Inst) {
  OutStreamer.EmitInstruction(Inst, getSubtargetInfo());
}

void C65AsmPrinter::InstAddGA(MCInst &Inst,
                              const MachineOperand &MO,
                              unsigned Offset) {
  Inst.addOperand
    (LowerOperand
     (MachineOperand::CreateGA(MO.getGlobal(), MO.getOffset() + Offset)));
}

void C65AsmPrinter::InstAddZR(MCInst &Inst,
                              const MachineOperand &MO,
                              unsigned Offset) {
  const C65RegisterInfo &TRI =
    *static_cast<const C65RegisterInfo *>
    (TM.getSubtargetImpl()->getRegisterInfo());
  unsigned Addr = TRI.getZRAddress(MO.getReg());
  Inst.addOperand(MCOperand::CreateImm(Addr + Offset));
}

void C65AsmPrinter::EmitShiftOne(const MachineOperand &MO,
                                 unsigned NumBytes,
                                 bool Right) {
  unsigned FirstOp = Right ? C65::ASLzp : C65::LSRzp;
  unsigned ChainedOp = Right ? C65::RORzp : C65::ROLzp;
  MCInst FirstInst;
  FirstInst.setOpcode(FirstOp);
  InstAddZR(FirstInst, MO, 0);
  EmitInstructionFinal(FirstInst);
  for (unsigned I = 2; I < NumBytes; I += 2) {
    MCInst ChainedInst;
    ChainedInst.setOpcode(ChainedOp);
    InstAddZR(ChainedInst, MO, I);
    EmitInstructionFinal(ChainedInst);
  }
}

void C65AsmPrinter::EmitShift(const MachineInstr *MI,
                              unsigned NumBytes,
                              bool Right) {
  const MachineOperand &Count = MI->getOperand(2);
  if (NumBytes == 1) {
    EmitInstructionFinal(MCInstBuilder(C65::SEP)
      .addImm(0x20));
  }
  if (Count.isReg()) {
    MCInst Inst;
    Inst.setOpcode(C65::LDXzp);
    InstAddZR(Inst, Count, 0);
    EmitInstructionFinal(Inst);
  } else {
    assert(Count.isImm());
    for (unsigned I = 0, E = Count.getImm(); I != E; ++I) {
      EmitShiftOne(MI->getOperand(1), NumBytes, Right);
    }
  }
  if (NumBytes == 1) {
    EmitInstructionFinal(MCInstBuilder(C65::REP)
      .addImm(0x20));
  }
}

void C65AsmPrinter::EmitInstruction(const MachineInstr *MI) {
  unsigned NumBytes = getOpByteSize(MI->getOpcode());
  if (MI->getOpcode() == TargetOpcode::DBG_VALUE) {
    // FIXME: Debug Value.
    return;
  } else if (NumBytes == 0) {
    MCInst TmpInst;
    LowerC65MachineInstrToMCInst(MI, TmpInst);
    EmitToStreamer(OutStreamer, TmpInst);
  } else {
    // Emit the comment
    MCInst TmpInst;
    LowerC65MachineInstrToMCInst(MI, TmpInst);
    EmitToStreamer(OutStreamer, TmpInst);

    if (NumBytes == 1) {
      EmitInstructionFinal(MCInstBuilder(C65::SEP)
        .addImm(0x20));
    }
    if (getOpRequireCLC(MI->getOpcode())) {
      EmitInstructionFinal(MCInstBuilder(C65::CLC));
    }
    for (unsigned I = 0; I < NumBytes; I += 2) {
      switch (MI->getOpcode()) {
      default: llvm_unreachable("Unknown Z instruction.");
      case C65::STZ8z:
      case C65::STZ16z:
      case C65::STZ32z:
      case C65::STZ64z: {
        MCInst Inst;
        Inst.setOpcode(C65::STZzp);
        InstAddZR(Inst, MI->getOperand(0), I);
        EmitInstructionFinal(Inst);
      } break;
      case C65::ST8zi:
      case C65::ST16zi:
      case C65::ST32zi:
      case C65::ST64zi: {
        MCInst LDAInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(0), I);
        EmitInstructionFinal(LDAInst);

        STAInst.setOpcode(C65::STAi);
        InstAddGA(STAInst, MI->getOperand(1), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::LD8zi:
      case C65::LD16zi:
      case C65::LD32zi:
      case C65::LD64zi: {
        MCInst LDAInst, STAInst;
        LDAInst.setOpcode(C65::LDAi);
        InstAddGA(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::LD8zimm:
      case C65::LD16zimm:
      case C65::LD32zimm:
      case C65::LD64zimm: {
        MCInst LDAInst, STAInst;
        unsigned Value = MI->getOperand(1).getImm() >> (16 * I) & 0xFFFF;
        if (Value == 0) {
          STAInst.setOpcode(C65::STZzp);
          InstAddZR(STAInst, MI->getOperand(0), I);
          EmitInstructionFinal(STAInst);
        } else {
          LDAInst.setOpcode(C65::LDAimm);
          LDAInst.addOperand(MCOperand::CreateImm(Value));
          EmitInstructionFinal(LDAInst);

          STAInst.setOpcode(C65::STAzp);
          InstAddZR(STAInst, MI->getOperand(0), I);
          EmitInstructionFinal(STAInst);
        }
      } break;
      case C65::AND8zz:
      case C65::AND16zz:
      case C65::AND32zz:
      case C65::AND64zz: {
        MCInst LDAInst, ArithInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        ArithInst.setOpcode(C65::ANDzp);
        InstAddZR(ArithInst, MI->getOperand(2), I);
        EmitInstructionFinal(ArithInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::OR8zz:
      case C65::OR16zz:
      case C65::OR32zz:
      case C65::OR64zz: {
        MCInst LDAInst, ArithInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        ArithInst.setOpcode(C65::ORAzp);
        InstAddZR(ArithInst, MI->getOperand(2), I);
        EmitInstructionFinal(ArithInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::XOR8zz:
      case C65::XOR16zz:
      case C65::XOR32zz:
      case C65::XOR64zz: {
        MCInst LDAInst, ArithInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        ArithInst.setOpcode(C65::EORzp);
        InstAddZR(ArithInst, MI->getOperand(2), I);
        EmitInstructionFinal(ArithInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::ADD8zz:
      case C65::ADD16zz:
      case C65::ADD32zz:
      case C65::ADD64zz: {
        MCInst LDAInst, ArithInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        ArithInst.setOpcode(C65::ADCzp);
        InstAddZR(ArithInst, MI->getOperand(2), I);
        EmitInstructionFinal(ArithInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::SUB8zz:
      case C65::SUB16zz:
      case C65::SUB32zz:
      case C65::SUB64zz: {
        MCInst LDAInst, ArithInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        ArithInst.setOpcode(C65::SBCzp);
        InstAddZR(ArithInst, MI->getOperand(2), I);
        EmitInstructionFinal(ArithInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::MOV8zz:
      case C65::MOV16zz:
      case C65::MOV32zz:
      case C65::MOV64zz: {
        MCInst LDAInst, STAInst;
        LDAInst.setOpcode(C65::LDAzp);
        InstAddZR(LDAInst, MI->getOperand(1), I);
        EmitInstructionFinal(LDAInst);

        STAInst.setOpcode(C65::STAzp);
        InstAddZR(STAInst, MI->getOperand(0), I);
        EmitInstructionFinal(STAInst);
      } break;
      case C65::SHL8zimm:
      case C65::SHL16zimm:
      case C65::SHL32zimm:
      case C65::SHL64zimm:
      case C65::SHL8zz:
      case C65::SHL16zz:
      case C65::SHL32zz:
      case C65::SHL64zz: {
        EmitShift(MI, NumBytes, false);
      } break;
      case C65::LSHR8zimm:
      case C65::LSHR16zimm:
      case C65::LSHR32zimm:
      case C65::LSHR64zimm:
      case C65::LSHR8zz:
      case C65::LSHR16zz:
      case C65::LSHR32zz:
      case C65::LSHR64zz: {
        EmitShift(MI, NumBytes, true);
      } break;
      }
    }
    if (NumBytes == 1) {
      EmitInstructionFinal(MCInstBuilder(C65::REP)
        .addImm(0x20));
    }
  }
}

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
