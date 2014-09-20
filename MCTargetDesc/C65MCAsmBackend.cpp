//===-- C65MCAsmBackend.cpp - C65 assembler backend ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65MCTargetDesc.h"
#include "MCTargetDesc/C65MCFixups.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"

using namespace llvm;

// Value is a fully-resolved relocation value: Symbol + Addend [- Pivot].
// Return the bits that should be installed in a relocation field for
// fixup kind Kind.
static uint64_t extractBitsForFixup(MCFixupKind Kind, uint64_t Value) {
  if (Kind < FirstTargetFixupKind)
    return Value;

  switch (unsigned(Kind)) {
  case C65::FK_390_PC16DBL:
  case C65::FK_390_PC32DBL:
  case C65::FK_390_PLT16DBL:
  case C65::FK_390_PLT32DBL:
    return (int64_t)Value / 2;
  }

  llvm_unreachable("Unknown fixup kind!");
}

namespace {
class C65MCAsmBackend : public MCAsmBackend {
  uint8_t OSABI;
public:
  C65MCAsmBackend(uint8_t osABI)
    : OSABI(osABI) {}

  // Override MCAsmBackend
  unsigned getNumFixupKinds() const override {
    return C65::NumTargetFixupKinds;
  }
  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override;
  void applyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                  uint64_t Value, bool IsPCRel) const override;
  bool mayNeedRelaxation(const MCInst &Inst) const override {
    return false;
  }
  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *Fragment,
                            const MCAsmLayout &Layout) const override {
    return false;
  }
  void relaxInstruction(const MCInst &Inst, MCInst &Res) const override {
    llvm_unreachable("C65 does do not have assembler relaxation");
  }
  bool writeNopData(uint64_t Count, MCObjectWriter *OW) const override;
  MCObjectWriter *createObjectWriter(raw_ostream &OS) const override {
    return createC65WLAKObjectWriter(OS, OSABI);
  }
};
} // end anonymous namespace

const MCFixupKindInfo &
C65MCAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo Infos[C65::NumTargetFixupKinds] = {
    { "FK_390_PC16DBL",  0, 16, MCFixupKindInfo::FKF_IsPCRel },
    { "FK_390_PC32DBL",  0, 32, MCFixupKindInfo::FKF_IsPCRel },
    { "FK_390_PLT16DBL", 0, 16, MCFixupKindInfo::FKF_IsPCRel },
    { "FK_390_PLT32DBL", 0, 32, MCFixupKindInfo::FKF_IsPCRel }
  };

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

void C65MCAsmBackend::applyFixup(const MCFixup &Fixup, char *Data,
                                     unsigned DataSize, uint64_t Value,
                                     bool IsPCRel) const {
  MCFixupKind Kind = Fixup.getKind();
  unsigned Offset = Fixup.getOffset();
  unsigned Size = (getFixupKindInfo(Kind).TargetSize + 7) / 8;

  assert(Offset + Size <= DataSize && "Invalid fixup offset!");

  // Big-endian insertion of Size bytes.
  Value = extractBitsForFixup(Kind, Value);
  unsigned ShiftValue = (Size * 8) - 8;
  for (unsigned I = 0; I != Size; ++I) {
    Data[Offset + I] |= uint8_t(Value >> ShiftValue);
    ShiftValue -= 8;
  }
}

bool C65MCAsmBackend::writeNopData(uint64_t Count,
                                   MCObjectWriter *OW) const {
  for (uint64_t I = 0; I != Count; ++I)
    OW->Write8(0xea);
  return true;
}

MCAsmBackend *llvm::createC65MCAsmBackend(const Target &T,
                                          const MCRegisterInfo &MRI,
                                          StringRef TT, StringRef CPU) {
  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(Triple(TT).getOS());
  return new C65MCAsmBackend(OSABI);
}
