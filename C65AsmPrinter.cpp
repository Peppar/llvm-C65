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
#include "C65AsmPrinter.h"
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

MCOperand C65AsmPrinter::LowerSymbolOperand(const MachineOperand &MO) {
  const MCSymbol *Symbol = nullptr;
  bool HasOffset = true;
  unsigned ShiftAmt = MO.getTargetFlags();

  switch(MO.getType()) {
  default:
    llvm_unreachable("Unknown type in LowerSymbolOperand");

  case MachineOperand::MO_MachineBasicBlock:
    Symbol = MO.getMBB()->getSymbol();
    HasOffset = false;
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

  const MCExpr *Expr = MCSymbolRefExpr::create(Symbol, OutContext);
  if (HasOffset) {
    if (int64_t Offset = MO.getOffset()) {
      const MCExpr *OffsetExpr = MCConstantExpr::create(Offset, OutContext);
      Expr = MCBinaryExpr::createAdd(Expr, OffsetExpr, OutContext);
    }
  }
  if (ShiftAmt) {
    const MCExpr *ShiftAmtExpr =  MCConstantExpr::create(ShiftAmt, OutContext);
    Expr = MCBinaryExpr::createLShr(Expr, ShiftAmtExpr, OutContext);
  }
  return MCOperand::createExpr(Expr);
}

MCOperand C65AsmPrinter::LowerOperand(const MachineOperand &MO) {
  switch(MO.getType()) {
  default:
    llvm_unreachable("unknown operand type"); break;

  case MachineOperand::MO_Register:
    if (MO.isImplicit())
      break;
    return MCOperand::createReg(MO.getReg());

  case MachineOperand::MO_Immediate:
    return MCOperand::createImm(MO.getImm());

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

  for (unsigned I = 0, E = MI->getNumOperands(); I != E; ++I) {
    const MachineOperand &MO = MI->getOperand(I);
    MCOperand MCOp = LowerOperand(MO);

    if (MCOp.isValid())
      OutMI.addOperand(MCOp);
  }
}

/// Emit an instruction
///
void C65AsmPrinter::EmitInstruction(const MachineInstr *MI) {
  // unsigned NumBytes = getOpByteSize(MI);
  if (MI->getOpcode() == TargetOpcode::DBG_VALUE) {
    // FIXME: Debug Value.
    return;
  } else if (MI->getOpcode() == C65::LONGA_ON) {
    if (OutStreamer->hasRawTextSupport())
      OutStreamer->EmitRawText(".accu 16\n");
  } else if (MI->getOpcode() == C65::LONGA_OFF) {
    if (OutStreamer->hasRawTextSupport())
      OutStreamer->EmitRawText(".accu 8\n");
  } else if (MI->getOpcode() == C65::LONGI_ON) {
    if (OutStreamer->hasRawTextSupport())
      OutStreamer->EmitRawText(".index 16\n");
  } else if (MI->getOpcode() == C65::LONGI_OFF) {
    if (OutStreamer->hasRawTextSupport())
      OutStreamer->EmitRawText(".index 8\n");
  } else {
    // Normal instruction, emitted as-is
    MCInst TmpInst;
    LowerC65MachineInstrToMCInst(MI, TmpInst);
    EmitToStreamer(*OutStreamer, TmpInst);
  }
}

static const char *getRegisterName(unsigned RegNo) {
  return C65InstPrinter::getRegisterName(RegNo);
}

void C65AsmPrinter::printOperand(const MachineInstr *MI, int opNum,
                                 raw_ostream &O) {
  const DataLayout &DL = getDataLayout();
  const MachineOperand &MO = MI->getOperand(opNum);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << StringRef(getRegisterName(MO.getReg()));
    break;
  case MachineOperand::MO_Immediate:
    O << (int)MO.getImm();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MO.getMBB()->getSymbol()->print(O, MAI);
    return;
  case MachineOperand::MO_GlobalAddress:
    getSymbol(MO.getGlobal())->print(O, MAI);
    break;
  case MachineOperand::MO_BlockAddress:
    O << GetBlockAddressSymbol(MO.getBlockAddress())->getName();
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << MO.getSymbolName();
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    O << DL.getPrivateGlobalPrefix() << "CPI" << getFunctionNumber() << "_"
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
