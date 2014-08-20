//===-- C65TargetMachine.cpp - Define TargetMachine for C65 ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "C65TargetMachine.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Transforms/Scalar.h"

using namespace llvm;

extern "C" void LLVMInitializeC65Target() {
  // Register the target.
  RegisterTargetMachine<C65TargetMachine> X(The65C816Target);
}

C65TargetMachine::C65TargetMachine(const Target &T, StringRef TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Reloc::Model RM, CodeModel::Model CM,
                                   CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
    Subtarget(TT, CPU, FS), DL("e-p:16:8-n16-S8"), InstrInfo(Subtarget),
    TLInfo(*this), TSInfo(DL), FrameLowering(*this, Subtarget) {
  initAsmInfo();
}

namespace {
/// C65 Code Generator Pass Configuration Options.
class C65PassConfig : public TargetPassConfig {
public:
  C65PassConfig(C65TargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  C65TargetMachine &getC65TargetMachine() const {
    return getTM<C65TargetMachine>();
  }

  bool addInstSelector() override;
};
} // end anonymous namespace

bool C65PassConfig::addInstSelector() {
  addPass(createC65ISelDag(getC65TargetMachine()));
  return false;
}

TargetPassConfig *C65TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new C65PassConfig(this, PM);
}
