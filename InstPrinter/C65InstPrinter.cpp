//===-- C65InstPrinter.cpp - Convert C65 MCInst to assembly syntax ---------==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints an C65 MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "C65InstPrinter.h"
#include "C65.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

#define DEBUG_TYPE "asm-printer"

#define GET_INSTRUCTION_NAME
#define PRINT_ALIAS_INSTR
#include "C65GenAsmWriter.inc"

void C65InstPrinter::printRegName(raw_ostream &OS, unsigned RegNo) const {
  OS << StringRef(getRegisterName(RegNo));
}

void C65InstPrinter::printInst(const MCInst *MI, raw_ostream &O,
                               StringRef Annot) {
  if (!printAliasInstr(MI, O)) {
    printInstruction(MI, O);
  }
  printAnnotation(O, Annot);
}

void C65InstPrinter::printOperand(const MCInst *MI, int opNum,
                                    raw_ostream &O) {
  const MCOperand &MO = MI->getOperand (opNum);

  if (MO.isReg()) {
    printRegName(O, MO.getReg());
  } else if (MO.isImm()) {
    O << '#' << (int)MO.getImm();
  } else {
    assert(MO.isExpr() && "Unknown operand kind in printOperand");
    MO.getExpr()->print(O);
  }
}

void C65InstPrinter::printMemOperand(const MCInst *MI, int opNum,
                                     raw_ostream &O, const char *Modifier) {
  printOperand(MI, opNum, O);

  // If this is an ADD operand, emit it like normal operands.
  // if (Modifier && !strcmp(Modifier, "arith")) {
  //   O << ", ";
  //   printOperand(MI, opNum+1, O);
  //   return;
  // } else {
  //   const MCOperand &MO = MI->getOperand(opNum+1);

  // if (MO.isReg() && MO.getReg() == SP::G0)
  //   return;   // don't print "+%g0"
  // if (MO.isImm() && MO.getImm() == 0)
  //   return;   // don't print "+0"

  // O << "+";

  // printOperand(MI, opNum+1, O);
}

void C65InstPrinter::printCCOperand(const MCInst *MI, int opNum,
                                    raw_ostream &O) {
  int CC = (int)MI->getOperand(opNum).getImm();
  switch (MI->getOpcode()) {
  default: break;
  // case SP::FBCOND:
  // case SP::FBCONDA:
  // case SP::BPFCC:
  // case SP::BPFCCA:
  // case SP::BPFCCNT:
  // case SP::BPFCCANT:
  // case SP::MOVFCCrr:  case SP::V9MOVFCCrr:
  // case SP::MOVFCCri:  case SP::V9MOVFCCri:
  // case SP::FMOVS_FCC: case SP::V9FMOVS_FCC:
  // case SP::FMOVD_FCC: case SP::V9FMOVD_FCC:
  // case SP::FMOVQ_FCC: case SP::V9FMOVQ_FCC:
  //   // Make sure CC is a fp conditional flag.
  //   CC = (CC < 16) ? (CC + 16) : CC;
  //   break;
  }
  //O << SPARCCondCodeToString((SPCC::CondCodes)CC);
}

bool C65InstPrinter::printGetPCX(const MCInst *MI, unsigned opNum,
                                  raw_ostream &O) {
  llvm_unreachable("FIXME: Implement C65InstPrinter::printGetPCX.");
  return true;
}
