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
  assert(!MO.isReg());
  if (MO.isImm()) {
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

void
C65InstPrinter::printAddress(const MCInst *MI, int OpNum, unsigned Indirection,
                             char PreIndexReg, char PostIndexReg,
                             unsigned LengthConstraint, raw_ostream &OS) {
  const MCOperand &MO = MI->getOperand(OpNum);
  assert(!MO.isReg());
  if (Indirection == 1)
    OS << '(';
  else if (Indirection == 2)
    OS << '[';
  if (LengthConstraint == 8)
    OS << '<';
  else if (LengthConstraint == 16)
    OS << '!';
  else if (LengthConstraint == 24)
    OS << '>';
  if (MO.isImm())
    OS << (int)MO.getImm();
  else {
    assert(MO.isExpr() && "Unknown operand kind in printAddress");
    MO.getExpr()->print(OS);
  }
  if (PreIndexReg)
    OS << PreIndexReg;
  if (Indirection == 1)
    OS << ')';
  else if (Indirection == 2)
    OS << ']';
  if (PostIndexReg)
    OS << PostIndexReg;
}
void C65InstPrinter::printPCRel8Operand(const MCInst *MI, int OpNum,
                                        raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 0, 0, 0, OS);
}
void C65InstPrinter::printPCRel16Operand(const MCInst *MI, int OpNum,
                                         raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 0, 0, 0, OS);
}
void C65InstPrinter::printAbsOperand(const MCInst *MI, int OpNum,
                                     raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 0, 0, 16, OS);
}
void C65InstPrinter::printAbsXOperand(const MCInst *MI, int OpNum,
                                      raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'X', 0, 16, OS);
}
void C65InstPrinter::printAbsYOperand(const MCInst *MI, int OpNum,
                                      raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'Y', 0, 16, OS);
}
void C65InstPrinter::printAbsPreIXOperand(const MCInst *MI, int OpNum,
                                          raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 'X', 0, 16, OS);
}
void C65InstPrinter::printAbsIndOperand(const MCInst *MI, int OpNum,
                                        raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 0, 0, 16, OS);
}
void C65InstPrinter::printAbsIndLOperand(const MCInst *MI, int OpNum,
                                         raw_ostream &OS) {
  return printAddress(MI, OpNum, 2, 0, 0, 16, OS);
}
void C65InstPrinter::printAbsLOperand(const MCInst *MI, int OpNum,
                                      raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 0, 0, 24, OS);
}
void C65InstPrinter::printAbsXLOperand(const MCInst *MI, int OpNum,
                                       raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'X', 0, 24, OS);
}
void C65InstPrinter::printZPOperand(const MCInst *MI, int OpNum,
                                    raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 0, 0, 8, OS);
}
void C65InstPrinter::printZPXOperand(const MCInst *MI, int OpNum,
                                     raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'X', 0, 8, OS);
}
void C65InstPrinter::printZPYOperand(const MCInst *MI, int OpNum,
                                     raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'Y', 0, 8, OS);
}
void C65InstPrinter::printZPPostIXOperand(const MCInst *MI, int OpNum,
                                          raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 'X', 0, 8, OS);
}
void C65InstPrinter::printZPIndOperand(const MCInst *MI, int OpNum,
                                       raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 0, 0, 8, OS);
}
void C65InstPrinter::printDPIndLOperand(const MCInst *MI, int OpNum,
                                        raw_ostream &OS) {
  return printAddress(MI, OpNum, 2, 0, 0, 8, OS);
}
void C65InstPrinter::printZPPostIYOperand(const MCInst *MI, int OpNum,
                                          raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 0, 'Y', 8, OS);
}
void C65InstPrinter::printDPPostIYLOperand(const MCInst *MI, int OpNum,
                                           raw_ostream &OS) {
  return printAddress(MI, OpNum, 2, 0, 'Y', 8, OS);
}
void C65InstPrinter::printSRelOperand(const MCInst *MI, int OpNum,
                                      raw_ostream &OS) {
  return printAddress(MI, OpNum, 0, 'S', 0, 8, OS);
}
void C65InstPrinter::printSPostIYOperand(const MCInst *MI, int OpNum,
                                         raw_ostream &OS) {
  return printAddress(MI, OpNum, 1, 'S', 'Y', 8, OS);
}
// void C65InstPrinter::printIZOperand(const MCInst *MI, int OpNum,
//                                     raw_ostream &OS) {
//   llvm_unreachable("Zero-page register operands should have been expanded.");
// }
// void C65InstPrinter::printZZOperand(const MCInst *MI, int OpNum,
//                                     raw_ostream &OS) {
//   llvm_unreachable("Zero-page register operands should have been expanded.");
// }
