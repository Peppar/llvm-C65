//===-- C65MCCodeEmitter.cpp - Convert C65 code to machine code -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the C65MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/C65BaseInfo.h"
#include "MCTargetDesc/C65MCFixups.h"
#include "MCTargetDesc/C65MCTargetDesc.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInstrInfo.h"

using namespace llvm;

#define DEBUG_TYPE "c65-mccodeemitter"

namespace {
class C65MCCodeEmitter : public MCCodeEmitter {
  const MCInstrInfo &MCII;
  MCContext &Ctx;

public:
  C65MCCodeEmitter(const MCInstrInfo &mcii, MCContext &ctx)
    : MCII(mcii), Ctx(ctx) {
  }

  ~C65MCCodeEmitter() {}

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

private:
  // Get the binary encoding of a generic operand MO in MI.  Fixups is
  // the list of fixups against MI.
  uint64_t getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  // Get the binary encoding of an address.  Operand OpNum of MI needs
  // a 8-bit PC-relative fixup at Offset bytes from the start of the
  // next MI.
  uint64_t getPCRelOpValue(const MCInst &MI, const MCOperand &MO,
                           SmallVectorImpl<MCFixup> &Fixups,
                           const MCSubtargetInfo &STI) const;
};
} // end anonymous namespace

MCCodeEmitter *llvm::createC65MCCodeEmitter(const MCInstrInfo &MCII,
                                            const MCRegisterInfo &MRI,
                                            MCContext &Ctx) {
  return new C65MCCodeEmitter(MCII, Ctx);
}

void C65MCCodeEmitter::
encodeInstruction(const MCInst &MI, raw_ostream &OS,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  unsigned Opcode = MI.getOpcode();
  const MCInstrDesc &Desc = MCII.get(Opcode);
  unsigned TSFlags = Desc.TSFlags;

  unsigned C65Opcode = C65II::getOpcode(TSFlags);
  unsigned OpSize = C65II::getOpSize(TSFlags);
  bool OpPCRel = C65II::OpPCRel & TSFlags;

  //  unsigned Bits = getBinaryCodeForInstr(MI, Fixups, STI);
  //  assert (MCII.get(MI.getOpcode()).getSize() == 1);
  OS << uint8_t(C65Opcode);

  unsigned NumOps;

  // JSR/JSL has call arguments as MI arguments; skip these.
  switch (Opcode) {
  default:
    NumOps = MI.getNumOperands();
    break;
  case C65::JSRabs:
  case C65::JSLabsl:
  case C65::JSRabspreix8:
  case C65::JSRabspreix16:
    NumOps = 1;
    break;
  }
  for (unsigned I = 0; I < NumOps; ++I) {
    const MCOperand &MO = MI.getOperand(I);
    unsigned Value;
    if (OpPCRel)
      Value = getPCRelOpValue(MI, MO, Fixups, STI);
    else
      Value = getMachineOpValue(MI, MO, Fixups, STI);
    for (unsigned X = 0; X < OpSize; ++X)
      OS << uint8_t((Value >> (8 * X)) & 0xFF);
  }
}

uint64_t
C65MCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                    SmallVectorImpl<MCFixup> &Fixups,
                                    const MCSubtargetInfo &STI) const {
  if (MO.isImm())
    return MO.getImm();
  assert(MO.isExpr());

  unsigned Opcode = MI.getOpcode();
  const MCInstrDesc &Desc = MCII.get(Opcode);
  unsigned TSFlags = Desc.TSFlags;
  unsigned OpSize = C65II::getOpSize(TSFlags);

  const MCExpr *Expr = MO.getExpr();
  unsigned ShiftAmt = 0;

  // For 8-bit and 16-bit values, the outermost bit shift right is
  // converted to a corresponding fixup handled by the object writer.
  if (OpSize == 1 || OpSize == 2) {
    if (const MCBinaryExpr *BE = dyn_cast<MCBinaryExpr>(Expr)) {
      if (BE->getOpcode() == MCBinaryExpr::LShr) {
        if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(BE->getRHS())) {
          ShiftAmt = CE->getValue();
          Expr = BE->getLHS();
        }
      }
    }
  }

  MCFixupKind FixupKind;
  if (OpSize == 1) {
    assert(ShiftAmt < 64);
    FixupKind = (MCFixupKind)(C65::FK_C65_8 + (ShiftAmt >> 3));
  } else if (OpSize == 2) {
    assert(ShiftAmt < 64);
    FixupKind = (MCFixupKind)(C65::FK_C65_16 + (ShiftAmt >> 4));
  } else if (OpSize == 3) {
    assert(ShiftAmt == 0);
    FixupKind = (MCFixupKind)(C65::FK_C65_24);
  } else
    llvm_unreachable("Instruction expected to be without operands.");

  Fixups.push_back(MCFixup::create(1, MO.getExpr(), FixupKind));
  return 0;
}

uint64_t
C65MCCodeEmitter::getPCRelOpValue(const MCInst &MI, const MCOperand &MO,
                                  SmallVectorImpl<MCFixup> &Fixups,
                                  const MCSubtargetInfo &STI) const {
  if (MO.isImm())
    return MO.getImm();
  assert(MO.isExpr());

  const unsigned InstrSize = 1 + MCII.get(MI.getOpcode()).getSize();

  // The operand value is relative to the start of the next MI,
  // but the fixup is relative to the operand.
  const MCExpr *OffsetExpr = MCConstantExpr::create(InstrSize - 1, Ctx);
  const MCExpr *Expr = MCBinaryExpr::createSub(MO.getExpr(), OffsetExpr, Ctx);

  Fixups.push_back(MCFixup::create(1, Expr, FK_PCRel_1));
  return 0;
}
