//==- C65TargetMachine.h - Define TargetMachine for C65 -----------*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the 6502 compatibles specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//


#ifndef LLVM_TARGET_C65TARGETMACHINE_H
#define LLVM_TARGET_C65TARGETMACHINE_H

#include "C65Subtarget.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class C65TargetMachine : public LLVMTargetMachine {
  C65Subtarget Subtarget;

public:

  C65TargetMachine(const Target &T, StringRef TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Reloc::Model RM, CodeModel::Model CM,
                   CodeGenOpt::Level OL);

  const C65Subtarget *getSubtargetImpl() const override {
    return &Subtarget;
  }

  // Override LLVMTargetMachine
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
};

} // end namespace llvm

#endif
