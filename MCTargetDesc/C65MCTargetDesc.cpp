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

#include "C65MCAsmInfo.h"
#include "C65MCTargetDesc.h"
#include "InstPrinter/C65InstPrinter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCStreamer.h"
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
  MCAsmInfo *MAI = new C65ELFMCAsmInfo(TT);
  unsigned Reg = MRI.getDwarfRegNum(C65::SP, true);
  MCCFIInstruction Inst = MCCFIInstruction::createDefCfa(nullptr, Reg, 0);
  MAI->addInitialFrameState(Inst);
  return MAI;
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

static MCInstrInfo *createC65MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitC65MCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createC65MCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitC65MCRegisterInfo(X, C65::SP);
  return X;
}

static MCSubtargetInfo *createC65MCSubtargetInfo(StringRef TT, StringRef CPU,
                                                 StringRef FS) {
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitC65MCSubtargetInfo(X, TT, CPU, FS);
  return X;
}

// createC65MCCodeEmitter
// createC65MCAsmBackend
// static MCAsmBackend *createC65AsmBackend(const Target &T,
//                                          const MCRegisterInfo &MRI,
//                                          StringRef TT,
//                                          StringRef CPU) {
//   return new ELFSparcAsmBackend(T, Triple(TT).getOS());
// }

static MCStreamer *createC65MCStreamer(const Target &T, StringRef TT,
                                       MCContext &Context, MCAsmBackend &MAB,
                                       raw_ostream &OS, MCCodeEmitter *Emitter,
                                       const MCSubtargetInfo &STI, bool RelaxAll,
                                       bool NoExecStack) {
  MCStreamer *S =
      createELFStreamer(Context, MAB, OS, Emitter, RelaxAll, NoExecStack);
  //new MCTargetStreamer(*S);
  return S;
}

static MCStreamer *
createC65MCAsmStreamer(MCContext &Ctx, formatted_raw_ostream &OS,
                       bool isVerboseAsm, bool useDwarfDirectory,
                       MCInstPrinter *InstPrint, MCCodeEmitter *CE,
                       MCAsmBackend *TAB, bool ShowInst) {

  MCStreamer *S = llvm::createAsmStreamer(
      Ctx, OS, isVerboseAsm, useDwarfDirectory, InstPrint, CE, TAB, ShowInst);
  //new MCTargetStreamer(*S, OS);
  return S;
}

static MCInstPrinter *createC65MCInstPrinter(const Target &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI,
                                             const MCSubtargetInfo &STI) {
  return new C65InstPrinter(MAI, MII, MRI, STI);
}

extern "C" void LLVMInitializeC65TargetMC() {
  RegisterMCAsmInfoFn X(The65C816Target, createC65MCAsmInfo);

  TargetRegistry::RegisterMCCodeGenInfo(The65C816Target,
                                        createC65MCCodeGenInfo);

  TargetRegistry::RegisterMCInstrInfo(The65C816Target, createC65MCInstrInfo);

  TargetRegistry::RegisterMCRegInfo(The65C816Target, createC65MCRegisterInfo);

  TargetRegistry::RegisterMCSubtargetInfo(The65C816Target,
                                          createC65MCSubtargetInfo);

  //  TargetRegistry::RegisterMCCodeEmitter(The65C816Target,
  //                                        createC65MCCodeEmitter);

  //  TargetRegistry::RegisterMCAsmBackend(The65C816Target,
  //                                       createC65AsmBackend);

  TargetRegistry::RegisterMCObjectStreamer(The65C816Target,
                                           createC65MCStreamer);

  TargetRegistry::RegisterAsmStreamer(The65C816Target,
                                      createC65MCAsmStreamer);

  TargetRegistry::RegisterMCInstPrinter(The65C816Target,
                                        createC65MCInstPrinter);
}
