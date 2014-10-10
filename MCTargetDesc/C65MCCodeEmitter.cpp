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

  void EncodeInstruction(const MCInst &MI, raw_ostream &OS,
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
                                            const MCSubtargetInfo &MCSTI,
                                            MCContext &Ctx) {
  return new C65MCCodeEmitter(MCII, Ctx);
}

void C65MCCodeEmitter::
EncodeInstruction(const MCInst &MI, raw_ostream &OS,
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

  unsigned NumOps = MI.getNumOperands();
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
  MCFixupKind FixupKind;
  if (OpSize == 1)
    FixupKind = FK_Data_1;
  else if (OpSize == 2)
    FixupKind = FK_Data_2;
  else if (OpSize == 3)
    FixupKind = FK_Data_4;
  else
    llvm_unreachable("Instruction expected to be without operands.");

  Fixups.push_back(MCFixup::Create(1, MO.getExpr(), FixupKind));
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
  const MCExpr *OffsetExpr = MCConstantExpr::Create(InstrSize - 1, Ctx);
  const MCExpr *Expr = MCBinaryExpr::CreateSub(MO.getExpr(), OffsetExpr, Ctx);

  Fixups.push_back(MCFixup::Create(1, Expr, FK_PCRel_1));
  return 0;
}
