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

    static const char *getRegisterName(unsigned RegNo) {
      return C65InstPrinter::getRegisterName(RegNo);
    }

    virtual void EmitInstruction(const MachineInstr *) override;

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         unsigned AsmVariant, const char *ExtraCode,
                         raw_ostream &O) override;

    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               unsigned AsmVariant, const char *ExtraCode,
                               raw_ostream &O) override;
  };
} // end of anonymous namespace

static MCOperand LowerSymbolOperand(const MachineInstr *MI,
                                    const MachineOperand &MO,
                                    AsmPrinter &AP) {

  const MCSymbol *Symbol = nullptr;

  switch(MO.getType()) {
  default: llvm_unreachable("Unknown type in LowerSymbolOperand");
  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    break;

  case MachineOperand::MO_GlobalAddress:
    Symbol = AP.getSymbol(MO.getGlobal());
    break;

  case MachineOperand::MO_BlockAddress:
    Symbol = AP.GetBlockAddressSymbol(MO.getBlockAddress());
    break;

  case MachineOperand::MO_ExternalSymbol:
    Symbol = AP.GetExternalSymbolSymbol(MO.getSymbolName());
    break;

  case MachineOperand::MO_ConstantPoolIndex:
    Symbol = AP.GetCPISymbol(MO.getIndex());
    break;
  }

  const MCSymbolRefExpr *MCSym =
    MCSymbolRefExpr::Create(Symbol, AP.OutContext);
  return MCOperand::CreateExpr(MCSym);
}

static MCOperand LowerOperand(const MachineInstr *MI,
                              const MachineOperand &MO,
                              AsmPrinter &AP) {
  switch(MO.getType()) {
  default: llvm_unreachable("Unknown operand type in LowerOperand");
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
    return LowerSymbolOperand(MI, MO, AP);

  case MachineOperand::MO_RegisterMask: break;
  }
  return MCOperand();
}

static void LowerC65MachineInstrToMCInst(const MachineInstr *MI,
                                         MCInst &OutMI,
                                         AsmPrinter &AP) {

  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);
    MCOperand MCOp = LowerOperand(MI, MO, AP);

    if (MCOp.isValid()) {
      OutMI.addOperand(MCOp);
    }
  }
}

void C65AsmPrinter::EmitInstruction(const MachineInstr *MI) {
  if (MI->getOpcode() == TargetOpcode::DBG_VALUE) {
    // TODO: Debug value support
    return;
  } else {
    MCInst TmpInst;
    LowerC65MachineInstrToMCInst(MI, TmpInst, *this);
    EmitToStreamer(OutStreamer, TmpInst);
  }
}

void C65AsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                 raw_ostream &O) {
  const DataLayout *DL = TM.getDataLayout();
  const MachineOperand &MO = MI->getOperand(opNum);

  switch (MO.getType()) {
  default: llvm_unreachable("Unknown operand type in printOperand");
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
  }
}

bool C65AsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                                    unsigned AsmVariant,
                                    const char *ExtraCode,
                                    raw_ostream &O) {
  if (ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0) {
      // Unknown modifier
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
  RegisterAsmPrinter<C65AsmPrinter> X(The6502Target);
}
