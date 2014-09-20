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
  // Automatically generated by TableGen.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  // Called by the TableGen code to get the binary encoding of operand
  // MO in MI.  Fixups is the list of fixups against MI.
  uint64_t getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

  // Called by the TableGen code to get the binary encoding of an
  // address.  Operand OpNum of MI needs a 8-bit PC-relative fixup at
  // Offset bytes from the start of the next MI.
  uint64_t getPCRelEncoding(const MCInst &MI, unsigned OpNum,
                            SmallVectorImpl<MCFixup> &Fixups,
                            unsigned Kind, int64_t Offset) const;
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
  unsigned OpSize = C65II::getOpSize(TSFlags);

  unsigned Bits = getBinaryCodeForInstr(MI, Fixups, STI);
  assert (MCII.get(MI.getOpcode()).getSize() == 1);
  OS << uint8_t(Bits);

  unsigned NumOps = Desc.getNumOperands();
  for (unsigned I = 0; I < NumOps; ++I) {
    const MCOperand &MO = MI.getOperand(I);
    unsigned Value = getMachineOpValue(MI, MO, Fixups, STI);
    for (unsigned X = 0; X < OpSize; ++X) {
      OS << uint8_t((Value >> (8 * X)) & 0xFF);
    }
  }
}

uint64_t C65MCCodeEmitter::
getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                  SmallVectorImpl<MCFixup> &Fixups,
                  const MCSubtargetInfo &STI) const {
  assert(!MO.isReg());
  if (MO.isImm())
    return MO.getImm();

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

  Fixups.push_back(MCFixup::Create(0, MO.getExpr(), FixupKind));
  return 0;
}

uint64_t
C65MCCodeEmitter::getPCRelEncoding(const MCInst &MI, unsigned OpNum,
                                   SmallVectorImpl<MCFixup> &Fixups,
                                   unsigned Kind, int64_t Offset) const {
  const MCOperand &MO = MI.getOperand(OpNum);
  const MCExpr *Expr;
  const unsigned OpSize = MCII.get(MI.getOpcode()).getSize();
  if (MO.isImm())
    Expr = MCConstantExpr::Create(MO.getImm() + Offset - OpSize, Ctx);
  else {
    Expr = MO.getExpr();
    if (Offset) {
      // The operand value is relative to the start of the next MI,
      // but the fixup is relative to the operand field itself, which
      // is Offset bytes into MI.  Add Offset to the relocation value
      // to cancel out this difference.
      const MCExpr *OffsetExpr = MCConstantExpr::Create(Offset - OpSize, Ctx);
      Expr = MCBinaryExpr::CreateAdd(Expr, OffsetExpr, Ctx);
    }
  }
  Fixups.push_back(MCFixup::Create(Offset, Expr, (MCFixupKind)Kind));
  return 0;
}

#include "C65GenMCCodeEmitter.inc"
