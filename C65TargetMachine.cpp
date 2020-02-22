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
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetLoweringObjectFile.h"
#include "llvm/Transforms/Scalar.h"
#include "TargetInfo/C65TargetInfo.h"

using namespace llvm;

extern "C" void LLVMInitializeC65Target() {
  // Register the target.
  RegisterTargetMachine<C65TargetMachine> X(getThe65C816Target());
}

namespace {
const char *DescriptionString6502 =
  "e-m:e-p:16:8-i16:8-i32:8-i64:8-n8:16:32:64-S8";
const char *DescriptionString65816 =
  "e-m:e-p:16:8-p1:32:8-i16:8-i32:8-i64:8-n8:16:32:64-S8";

StringRef getDataLayout(const Triple &TT, StringRef CPU,
                        const TargetOptions &Options) {
  // TODO: Make the CPU distinctions conform better to the triple
  // system
  if (CPU.empty() || CPU == "generic" || CPU == "65816" || CPU == "65802")
    return DescriptionString65816;
  else
    return DescriptionString6502;
}
} // namespace

C65TargetMachine::C65TargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
  : LLVMTargetMachine(T, getDataLayout(TT, CPU, Options), TT, CPU, FS,
                      Options, RM.getValueOr(Reloc::Static),
                      CM.getValueOr(CodeModel::Small), OL),
    TLOF(make_unique<TargetLoweringObjectFileELF>()),
    Subtarget(TT, CPU, FS, *this) {
  initAsmInfo();
}

const C65Subtarget *
C65TargetMachine::getSubtargetImpl(const Function &F) const {
  return &Subtarget;
}

namespace {
/// C65 Code Generator Pass Configuration Options.
class C65PassConfig : public TargetPassConfig {
public:
  C65PassConfig(C65TargetMachine &TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  C65TargetMachine &getC65TargetMachine() const {
    return getTM<C65TargetMachine>();
  }

  bool addInstSelector() override;
  void addPostRegAlloc() override;
  void addPreEmitPass() override;
};
} // end anonymous namespace

bool C65PassConfig::addInstSelector() {
  addPass(createC65ISelDag(getC65TargetMachine()));
  return false;
}

void C65PassConfig::addPostRegAlloc() {
}

void C65PassConfig::addPreEmitPass() {
  addPass(createC65ZInstrExpanderPass());
  addPass(createC65RegSizeInsertPass());
}

TargetPassConfig *C65TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new C65PassConfig(*this, PM);
}
