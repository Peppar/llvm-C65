//===-- C65MCObjectWriter.cpp - WLAV object writer ------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65MCTargetDesc.h"
#include "MCTargetDesc/C65MCFixups.h"
#include "llvm/BinaryFormat/WLAV.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCWLAVObjectWriter.h"

using namespace llvm;

class C65MCWLAVObjectTargetWriter : public MCWLAVObjectTargetWriter {
public:
  virtual ~C65MCWLAVObjectTargetWriter() = default;

  virtual Triple::ObjectFormatType getFormat() const { return Triple::WLAV; }
  static bool classof(const MCObjectTargetWriter *W) {
    return W->getFormat() == Triple::WLAV;
  }

  virtual unsigned getRelocType(const MCFixup &Fixup) const override {
    switch((unsigned)Fixup.getKind()) {
    default:
      llvm_unreachable("Unimplemented fixup -> relocation");
    case FK_PCRel_1:
      return WLAV::R_RELATIVE_8BIT;
    case FK_PCRel_2:
      return WLAV::R_RELATIVE_16BIT;
    case FK_Data_1:
    case C65::FK_C65_8:
    case C65::FK_C65_8s8:
    case C65::FK_C65_8s16:
    case C65::FK_C65_8s24:
    case C65::FK_C65_8s32:
    case C65::FK_C65_8s40:
    case C65::FK_C65_8s48:
    case C65::FK_C65_8s56:
      return WLAV::R_DIRECT_8BIT;
    case FK_Data_2:
    case C65::FK_C65_16:
    case C65::FK_C65_16s16:
    case C65::FK_C65_16s32:
    case C65::FK_C65_16s48:
      return WLAV::R_DIRECT_16BIT;
    case FK_Data_4:
    case C65::FK_C65_24:
      return WLAV::R_DIRECT_24BIT;
    }
  }

  unsigned getFixupShiftAmt(const MCFixup &Fixup) const override {
    int Kind = Fixup.getKind();
    if (C65::isFixup8Bit(Kind))
      return C65::get8BitFixupShiftAmt(Kind);
    else if (C65::isFixup16Bit(Kind))
      return C65::get16BitFixupShiftAmt(Kind);
    else
      return 0;
  }
};

std::unique_ptr<MCObjectTargetWriter>
llvm::createC65MCWLAVObjectTargetWriter() {
  return llvm::make_unique<C65MCWLAVObjectTargetWriter>();
}
