//===-- C65AsmPrinter.h - C65 implementation of AsmPrinter ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_C65ASMPRINTER_H
#define LLVM_LIB_TARGET_C65_C65ASMPRINTER_H

#include "C65Subtarget.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/StackMaps.h"
#include "llvm/Target/TargetMachine.h"

// Implemented in C65MCInstLower.cpp
namespace {
  class C65MCInstLower;
}

namespace llvm {
  class MCStreamer;
  class MCSymbol;

  class LLVM_LIBRARY_VISIBILITY C65AsmPrinter : public AsmPrinter {
    const C65Subtarget *Subtarget;

  public:
    explicit C65AsmPrinter(TargetMachine &TM,
                           std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)) {}

    const char *getPassName() const override {
      return "C65 Assembly / Object Emitter";
    }

    void printOperand(const MachineInstr *MI, int opNum, raw_ostream &OS);

    MCOperand LowerSymbolOperand(const MachineOperand &MO);

    MCOperand LowerOperand(const MachineOperand &MO);

    void LowerC65MachineInstrToMCInst(const MachineInstr *MI,
                                      MCInst &OutMI);

    virtual void EmitInstruction(const MachineInstr *) override;

    bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                         unsigned AsmVariant, const char *ExtraCode,
                         raw_ostream &O) override;

    bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                               unsigned AsmVariant, const char *ExtraCode,
                               raw_ostream &O) override;
  };
} // end namespace llvm

#endif
