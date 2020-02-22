//===-- C65MCAsmBackend.cpp - C65 assembler backend -----------------------===//
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
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"

using namespace llvm;

namespace {

// Value is a fully-resolved relocation value: Symbol + Addend [- Pivot].
// Return the bits that should be installed in a relocation field for
// fixup kind Kind.
uint64_t extractBitsForFixup(MCFixupKind Kind, uint64_t Value) {
  if (Kind < FirstTargetFixupKind)
    return Value;
  if (C65::isFixup8Bit(Kind))
    return Value >> C65::get8BitFixupShiftAmt(Kind) & 0xFF;
  if (C65::isFixup16Bit(Kind))
    return Value >> C65::get16BitFixupShiftAmt(Kind) & 0xFFFF;
  llvm_unreachable("Unknown fixup kind!");
}

class C65MCAsmBackend : public MCAsmBackend {
public:
  C65MCAsmBackend() : MCAsmBackend(support::little) {}

  // Override MCAsmBackend
  unsigned getNumFixupKinds() const override {
    return C65::NumTargetFixupKinds;
  }
  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override;
  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override;
  bool mayNeedRelaxation(const MCInst &Inst,
                         const MCSubtargetInfo &STI) const override {
    return false;
  }
  bool fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                            const MCRelaxableFragment *Fragment,
                            const MCAsmLayout &Layout) const override {
    return false;
  }
  void relaxInstruction(const MCInst &Inst, const MCSubtargetInfo &STI,
                        MCInst &Res) const override {
    llvm_unreachable("C65 does do not have assembler relaxation");
  }
  bool writeNopData(raw_ostream &OS, uint64_t Count) const override;
  std::unique_ptr<MCObjectTargetWriter> createObjectTargetWriter() const override {
    return createC65MCWLAVObjectTargetWriter();
  }
};
} // end anonymous namespace

const MCFixupKindInfo &
C65MCAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  const static MCFixupKindInfo Infos[C65::NumTargetFixupKinds] = {
    { "FK_C65_8",     0, 8,  0 },
    { "FK_C65_8s8",   0, 8,  0 },
    { "FK_C65_8s16",  0, 8,  0 },
    { "FK_C65_8s24",  0, 8,  0 },
    { "FK_C65_8s32",  0, 8,  0 },
    { "FK_C65_8s40",  0, 8,  0 },
    { "FK_C65_8s48",  0, 8,  0 },
    { "FK_C65_8s56",  0, 8,  0 },
    { "FK_C65_16",    0, 16, 0 },
    { "FK_C65_16s16", 0, 16, 0 },
    { "FK_C65_16s32", 0, 16, 0 },
    { "FK_C65_16s48", 0, 16, 0 },
    { "FK_C65_24",    0, 24, 0 }
  };

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

void C65MCAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                                 const MCValue &Target, MutableArrayRef<char> Data,
                                 uint64_t Value, bool IsResolved,
                                 const MCSubtargetInfo *STI) const {
  MCFixupKind Kind = Fixup.getKind();
  unsigned Offset = Fixup.getOffset();
  unsigned Size = (getFixupKindInfo(Kind).TargetSize + 7) / 8;

  assert(Offset + Size <= Data.size() && "Invalid fixup offset!");

  // Big-endian insertion of Size bytes.
  Value = extractBitsForFixup(Kind, Value);
  unsigned ShiftValue = (Size * 8) - 8;
  for (unsigned I = 0; I != Size; ++I) {
    Data[Offset + I] |= uint8_t(Value >> ShiftValue);
    ShiftValue -= 8;
  }
}

bool C65MCAsmBackend::writeNopData(raw_ostream &OS,
                                   uint64_t Count) const {
  for (uint64_t I = 0; I != Count; ++I)
    OS << '\xea';
  return true;
}

MCAsmBackend *llvm::createC65MCAsmBackend(const Target &T,
                                          const MCSubtargetInfo &STI,
                                          const MCRegisterInfo &MRI,
                                          const MCTargetOptions &Options) {
  //uint8_t OSABI =
  //    MCELFObjectTargetWriter::getOSABI(STI.getTargetTriple().getOS());
  return new C65MCAsmBackend();
}
