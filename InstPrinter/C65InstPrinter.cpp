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

#include "C65.h"
#include "C65InstPrinter.h"
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

void C65InstPrinter::printInst(const MCInst *MI, raw_ostream &OS,
                               StringRef Annot) {
  if (!printAliasInstr(MI, OS)) {
    printInstruction(MI, OS);
  }
  printAnnotation(OS, Annot);
}

void C65InstPrinter::printOperand(const MCInst *MI, int OpNum,
                                  raw_ostream &OS, bool Mem) {
  const MCOperand &MO = MI->getOperand(OpNum);

  if (MO.isReg()) {
    printRegName(OS, MO.getReg());
  } else if (MO.isImm()) {
    if (!Mem) {
      // Immediate operator
      OS << '#';
    }
    OS << (int)MO.getImm();
  } else {
    assert(MO.isExpr() && "Unknown operand kind in printOperand");
    MO.getExpr()->print(OS);
  }
}

void C65InstPrinter::printMemOperandAbs(const MCInst *MI, int OpNum,
                                        raw_ostream &OS) {
  printOperand(MI, OpNum, OS, true);
}

void C65InstPrinter::printMemOperandIndex(const MCInst *MI, int OpNum,
                                          raw_ostream &OS) {
  printOperand(MI, OpNum + 1, OS, true);
  OS << ',';
  printOperand(MI, OpNum, OS, true);
}
