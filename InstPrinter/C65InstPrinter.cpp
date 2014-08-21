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

#define DEBUG_TYPE "c65-inst-printer"

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
}
