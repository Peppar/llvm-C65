//===-- C65MCTargetDesc.cpp - C65 Target Descriptions --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides 6502 compatibles specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "C65MCTargetDesc.h"
#include "llvm/MC/MCAsmInfoELF.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "C65GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "C65GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "C65GenRegisterInfo.inc"

static MCAsmInfo *createC65MCAsmInfo(const MCRegisterInfo &MRI,
                                     StringRef TT) {
  // TODO: Do this properly with own class
  MCAsmInfo *MAI = new MCAsmInfoELF();
  unsigned Reg = MRI.getDwarfRegNum(C65::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, Reg, 0);
  MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCInstrInfo *createC65MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitC65MCInstrInfo(X);
  return X;
}

static MCSubtargetInfo *createC65MCSubtargetInfo(StringRef TT, StringRef CPU,
                                                 StringRef FS) {
  MCSubtargetInfo *X = new MCSubtargetInfo();
  Triple TheTriple(TT);
  //if (CPU.empty())
  //  CPU = (TheTriple.getArch() == Triple::sparcv9) ? "v9" : "v8";
  InitC65MCSubtargetInfo(X, TT, CPU, FS);
  return X;
}

static MCCodeGenInfo *createC65MCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                             CodeModel::Model CM,
                                             CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();

  // The default 32-bit code model is abs32/pic32 and the default 32-bit
  // code model for JIT is abs32.
  // TODO: Understand the codemodel
  switch (CM) {
  default: break;
  case CodeModel::Default:
    CM = CodeModel::Small; break;
  }
  X->InitMCCodeGenInfo(RM, CM, OL);
  return X;
}

static MCStreamer *createMCStreamer(const Target &T, StringRef TT,
                                    MCContext &Context, MCAsmBackend &MAB,
                                    raw_ostream &OS, MCCodeEmitter *Emitter,
                                    const MCSubtargetInfo &STI, bool RelaxAll,
                                    bool NoExecStack) {
  MCStreamer *S =
      createELFStreamer(Context, MAB, OS, Emitter, RelaxAll, NoExecStack);
  //new C65TargetELFStreamer(*S);
  return S;
}

static MCStreamer *
createMCAsmStreamer(MCContext &Ctx, formatted_raw_ostream &OS,
                    bool isVerboseAsm, bool useDwarfDirectory,
                    MCInstPrinter *InstPrint, MCCodeEmitter *CE,
                    MCAsmBackend *TAB, bool ShowInst) {

  MCStreamer *S = llvm::createAsmStreamer(
      Ctx, OS, isVerboseAsm, useDwarfDirectory, InstPrint, CE, TAB, ShowInst);
  new C65TargetAsmStreamer(*S, OS);
  return S;
}


extern "C" void LLVMInitializeC65TargetMC() {
  RegisterMCAsmInfoFn X(The65C816Target, createC65MCAsmInfo);

  TargetRegistry::RegisterMCCodeGenInfo(The65C816Target,
                                        createC65MCCodeGenInfo);

  TargetRegistry::RegisterMCInstrInfo(The65C815Target, createC65MCInstrInfo);

  TargetRegistry::RegisterMCRegInfo(The65C816Target, createC65MCRegisterInfo);

  TargetRegistry::RegisterMCSubtargetInfo(The65C816Target,
                                          create65C816MCSubtargetInfo);

  TargetRegistry::RegisterMCCodeEmitter(The65C816Target,
                                        create65C816MCCodeEmitter);

  TargetRegistry::RegisterMCAsmBackend(The65C816Target,
                                       create65C816AsmBackend);

  TargetRegistry::RegisterMCObjectStreamer(TheC65Target,
                                           createMCStreamer);

  TargetRegistry::RegisterAsmStreamer(TheC65Target,
                                      createMCAsmStreamer);

  TargetRegistry::RegisterMCInstPrinter(TheC65Target,
                                        createC65MCInstPrinter);
}
