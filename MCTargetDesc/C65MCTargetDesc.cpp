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
#include "InstPrinter/C65InstPrinter.h"
#include "llvm/MC/MCAsmInfoELF.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCStreamer.h"
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

namespace llvm {
  class C65MCAsmInfo : public MCAsmInfo {
  public:
    explicit C65MCAsmInfo(StringRef TT) {
      // TODO: Make use of TT (triple string)
      PointerSize = 2;
      CalleeSaveStackSlotSize = 1;
      IsLittleEndian = false;

      Data8bitsDirective = "\t.db\t";
      Data16bitsDirective = "\t.dw\t";
      Data32bitsDirective = nullptr;
      Data64bitsDirective = nullptr;

      AsciiDirective = "\t.db\t";
      AscizDirective = nullptr;
      ZeroDirective = nullptr;
      CommentString = ";";

      InlineAsmStart = ";APP\n";
      InlineAsmEnd = ";NO_APP\n";

      HasSetDirective = false;
    }
  };
}

static MCAsmInfo *createC65MCAsmInfo(const MCRegisterInfo &MRI,
                                     StringRef TT) {
  // TODO: Add initial frame state information
  return new C65MCAsmInfo(TT);
}

static MCCodeGenInfo *createC65MCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                             CodeModel::Model CM,
                                             CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();

  // TODO: Understand how the code model system works
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
  InitC65MCRegisterInfo(X, C65::S);
  return X;
}

static MCSubtargetInfo *createC65MCSubtargetInfo(StringRef TT, StringRef CPU,
                                                 StringRef FS) {
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitC65MCSubtargetInfo(X, TT, CPU, FS);
  return X;
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

  TargetRegistry::RegisterMCInstPrinter(The65C816Target,
                                        createC65MCInstPrinter);
}
