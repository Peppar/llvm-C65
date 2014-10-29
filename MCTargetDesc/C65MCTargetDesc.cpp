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
      IsLittleEndian = true;

      // Data8bitsDirective = "\t.db\t";
      // Data16bitsDirective = "\t.dw\t";
      // Data32bitsDirective = "\t.dd\t";
      // Data64bitsDirective = "\t.dq\t";
      // AsciiDirective = "\t.db\t";
      // AscizDirective = nullptr;
      // ZeroDirective = nullptr;
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

  // Static code is suitable for use in a dynamic executable; there is no
  // separate DynamicNoPIC model.
  if (RM == Reloc::Default || RM == Reloc::DynamicNoPIC)
    RM = Reloc::Static;

  if (CM == CodeModel::Default)
    CM = CodeModel::Small;

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

static MCStreamer *createC65MCObjectStreamer(const Target &T, StringRef TT,
                                             MCContext &Ctx,
                                             MCAsmBackend &MAB,
                                             raw_ostream &OS,
                                             MCCodeEmitter *Emitter,
                                             const MCSubtargetInfo &STI,
                                             bool RelaxAll,
                                             bool NoExecStack) {
  return createELFStreamer(Ctx, MAB, OS, Emitter, RelaxAll, NoExecStack);
}

extern "C" void LLVMInitializeC65TargetMC() {
  //
  //RegisterMCAsmInfoFn X(The65C816Target, createC65MCAsmInfo);
  TargetRegistry::RegisterMCAsmInfo(The65C816Target,
                                    createC65MCAsmInfo);

  TargetRegistry::RegisterMCCodeGenInfo(The65C816Target,
                                        createC65MCCodeGenInfo);

  TargetRegistry::RegisterMCCodeEmitter(The65C816Target,
                                        createC65MCCodeEmitter);

  TargetRegistry::RegisterMCInstrInfo(The65C816Target,
                                      createC65MCInstrInfo);

  TargetRegistry::RegisterMCRegInfo(The65C816Target,
                                    createC65MCRegisterInfo);

  TargetRegistry::RegisterMCSubtargetInfo(The65C816Target,
                                          createC65MCSubtargetInfo);

  TargetRegistry::RegisterMCAsmBackend(The65C816Target,
                                       createC65MCAsmBackend);

  TargetRegistry::RegisterMCInstPrinter(The65C816Target,
                                        createC65MCInstPrinter);

  TargetRegistry::RegisterMCObjectStreamer(The65C816Target,
                                           createC65MCObjectStreamer);
}
