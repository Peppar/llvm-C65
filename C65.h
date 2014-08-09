//===-- C65.h - Top-level interface for C65 Target --------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// 6502 compatibles back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_C65_H
#define LLVM_TARGET_C65_H

#include "MCTargetDesc/C65MCTargetDesc.h"

namespace llvm {
  class FunctionPass;
  class C65TargetMachine;
  class MachineInstr;
  class AsmPrinter;
  class MCInst;

  FunctionPass *createC65ISelDag(C65TargetMachine &TM);

  void LowerC65MachineInstrToMCInst(const MachineInstr *MI, MCInst &OutMI,
                                    AsmPrinter &AP, bool isDarwin);
} // end namespace llvm;

#endif
