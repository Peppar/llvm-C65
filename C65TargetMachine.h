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


#ifndef C65TARGETMACHINE_H
#define C65TARGETMACHINE_H

#include "C65InstrInfo.h"
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
  const C65InstrInfo *getInstrInfo() const override {
    return getSubtargetImpl()->getInstrInfo();
  }
  const C65RegisterInfo *getRegisterInfo() const override {
    return getSubtargetImpl()->getRegisterInfo();
  }
  const C65TargetLowering *getTargetLowering() const override {
    return getSubtargetImpl()->getTargetLowering();
  }
  const TargetFrameLowering *getFrameLowering() const override {
    return getSubtargetImpl()->getFrameLowering();
  }
  const DataLayout *getDataLayout() const override {
    return getSubtargetImpl()->getDataLayout();
  }
  const TargetSelectionDAGInfo *getSelectionDAGInfo() const override {
    return getSubtargetImpl()->getSelectionDAGInfo();
  }

  // Override LLVMTargetMachine
  //TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
};

} // end namespace llvm

#endif
