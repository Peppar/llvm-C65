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

#include "C65InstrInfo.h"
#include "C65Subtarget.h"
#include "C65ISelLowering.h"
#include "C65FrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class C65TargetMachine;

class C65SelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit C65SelectionDAGInfo(const DataLayout &DL)
    : TargetSelectionDAGInfo(&DL) {};

  ~C65SelectionDAGInfo() {};
};

class C65TargetMachine : public LLVMTargetMachine {
  C65Subtarget Subtarget;
  const DataLayout DL;
  C65InstrInfo InstrInfo;
  C65TargetLowering TLInfo;
  C65SelectionDAGInfo TSInfo;
  C65FrameLowering FrameLowering;

public:
  C65TargetMachine(const Target &T, StringRef TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Reloc::Model RM, CodeModel::Model CM,
                   CodeGenOpt::Level OL);

  const C65Subtarget *getSubtargetImpl() const override {
    return &Subtarget;
  }
  const C65InstrInfo *getInstrInfo() const override {
    return &InstrInfo;
  }
  const C65RegisterInfo *getRegisterInfo() const override {
    return &InstrInfo.getRegisterInfo();
  }
  const C65TargetLowering *getTargetLowering() const override {
    return &TLInfo;
  }
  const TargetFrameLowering *getFrameLowering() const override {
    return &FrameLowering;
  }
  const DataLayout *getDataLayout() const override {
    return &DL;
  }
  const TargetSelectionDAGInfo *getSelectionDAGInfo() const override {
    return &TSInfo;
  }

  // Override LLVMTargetMachine
  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;
};

} // end namespace llvm

#endif
