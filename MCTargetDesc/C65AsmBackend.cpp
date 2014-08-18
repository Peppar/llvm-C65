//===-- C65AsmBackend.cpp - C65 Assembler Backend ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCAsmBackend.h"
//#include "MCTargetDesc/C65FixupKinds.h"
#include "MCTargetDesc/C65MCTargetDesc.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

namespace {
  class C65AsmBackend : public MCAsmBackend {
    const Target &TheTarget;
  public:
    C65AsmBackend(const Target &T) : MCAsmBackend(), TheTarget(T) {}

    unsigned getNumFixupKinds() const override {
      return 0;
    }

    bool mayNeedRelaxation(const MCInst &Inst) const override {
      // FIXME.
      return false;
    }

    /// fixupNeedsRelaxation - Target specific predicate for whether a given
    /// fixup requires the associated instruction to be relaxed.
    bool fixupNeedsRelaxation(const MCFixup &Fixup,
                              uint64_t Value,
                              const MCRelaxableFragment *DF,
                              const MCAsmLayout &Layout) const override {
      // FIXME.
      llvm_unreachable("fixupNeedsRelaxation() unimplemented");
      return false;
    }
    void relaxInstruction(const MCInst &Inst, MCInst &Res) const override {
      // FIXME.
      llvm_unreachable("relaxInstruction() unimplemented");
    }

    bool writeNopData(uint64_t Count, MCObjectWriter *OW) const override {
      for (uint64_t i = 0; i != Count; ++i)
        OW->Write8(0xEA);
      return true;
    }
  };

  // class ELFC65AsmBackend : public C65AsmBackend {
  //   Triple::OSType OSType;
  // public:
  //   ELFC65AsmBackend(const Target &T, Triple::OSType OSType) :
  //     C65AsmBackend(T), OSType(OSType) { }

  //   void applyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
  //                   uint64_t Value, bool IsPCRel) const override {

  //     // Value = adjustFixupValue(Fixup.getKind(), Value);
  //     // if (!Value) return;           // Doesn't change encoding.

  //     // unsigned Offset = Fixup.getOffset();

  //     // // For each byte of the fragment that the fixup touches, mask in the bits
  //     // // from the fixup value. The Value has been "split up" into the
  //     // // appropriate bitfields above.
  //     // for (unsigned i = 0; i != 4; ++i)
  //     //   Data[Offset + i] |= uint8_t((Value >> ((4 - i - 1)*8)) & 0xff);

  //   }

  //   MCObjectWriter *createObjectWriter(raw_ostream &OS) const override {
  //     uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(OSType);
  //     return createC65ELFObjectWriter(OS, false, OSABI);
  //   }
  // };
} // end anonymous namespace


// MCAsmBackend *llvm::createC65AsmBackend(const Target &T,
//                                         const MCRegisterInfo &MRI,
//                                         StringRef TT,//
//                                         StringRef CPU) {
//   //  return new ELFC65AsmBackend(T, Triple(TT).getOS());
//   //}
