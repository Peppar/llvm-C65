//===-- C65InstrInfo.cpp - C65 Instruction Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the C65 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "C65InstrInfo.h"
#include "C65Subtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "c65-codegen"

#define GET_INSTRINFO_CTOR_DTOR
#include "C65GenInstrInfo.inc"

// Pin the vtable to this file.
void C65InstrInfo::anchor() {}

C65InstrInfo::C65InstrInfo(C65Subtarget &ST)
  : C65GenInstrInfo(C65::ADJCALLSTACKDOWN, C65::ADJCALLSTACKUP),
    RI(ST), STI(ST) {}

void C65InstrInfo::buildPushReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                const DebugLoc &DL, unsigned Reg) const {
  // if (Reg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::PHA));
  // } else if (Reg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::PHX));
  // } else if (Reg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::PHY));
  // } else if (Reg == C65::D) {
  //   BuildMI(MBB, MI, DL, get(C65::PHD));
  // } else if (Reg == C65::P) {
  //   BuildMI(MBB, MI, DL, get(C65::PHP));
  // } else {
  //   LLVM_DEBUG(dbgs() << "Cannot push " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy push");
  // }
}

void C65InstrInfo::buildPullReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                const DebugLoc &DL, unsigned Reg) const {
  // if (Reg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::PLA));
  // } else if (Reg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::PLX));
  // } else if (Reg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::PLY));
  // } else if (Reg == C65::D) {
  //   BuildMI(MBB, MI, DL, get(C65::PLD));
  // } else if (Reg == C65::P) {
  //   BuildMI(MBB, MI, DL, get(C65::PLP));
  // } else {
  //   LLVM_DEBUG(dbgs() << "Cannot pull " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy pull");
  // }
}

void C65InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MBBI,
                               const DebugLoc &DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
  unsigned ZMOVInstr =
    C65::ZRC8RegClass.contains(DestReg, SrcReg) ? C65::ZMOV8 :
    C65::ZRC16RegClass.contains(DestReg, SrcReg) ? C65::ZMOV16 :
    C65::ZRC32RegClass.contains(DestReg, SrcReg) ? C65::ZMOV32 :
    C65::ZRC64RegClass.contains(DestReg, SrcReg) ? C65::ZMOV64 :
    0;

  if (ZMOVInstr) {
    BuildMI(MBB, MBBI, DL, get(ZMOVInstr), DestReg)
      .addReg(SrcReg, getKillRegState(KillSrc));
  } else
    llvm_unreachable("Impossible reg-to-reg copy");

  //  const unsigned ByteCapacity = STI.has65802() ? 2 : 1;
  // bool ZRegSrc = RI.isZReg(SrcReg);
        //  bool ZRegDest = RI.isZReg(DestReg);

  // if (ZRegSrc && ZRegDest) {
  //   unsigned LDAInstr;
  //   unsigned STAInstr;
  //   unsigned NumBytes = RI.getZRSize(DestReg);
  //   if (NumBytes == 1 || ByteCapacity == 1) {
  //     LDAInstr = C65::LDA8zp;
  //     STAInstr = C65::STA8zp;
  //   } else {
  //     LDAInstr = C65::LDA16zp;
  //     STAInstr = C65::STA16zp;
  //   }
  //   for (unsigned I = 0; I < NumBytes; I += ByteCapacity) {
  //     BuildMI(MBB, MBBI, DL, get(LDAInstr))
  //       .addImm(RI.getZRAddress(SrcReg) + I);
  //     BuildMI(MBB, MBBI, DL, get(STAInstr))
  //       .addImm(RI.getZRAddress(DestReg) + I);
  //   }
  // } else if (ZRegSrc) {
  //   unsigned SimpleOp =
  //     DestReg == C65::C ? C65::LDA16zp :
  //     DestReg == C65::X ? C65::LDX16zp :
  //     DestReg == C65::Y ? C65::LDY16zp :
  //     DestReg == C65::A ? C65::LDA8zp :
  //     DestReg == C65::XL ? C65::LDX8zp :
  //     DestReg == C65::YL ? C65::LDY8zp : 0;
  //   if (SimpleOp) {
  //     BuildMI(MBB, MBBI, DL, get(SimpleOp))
  //       .addImm(RI.getZRAddress(SrcReg));
  //   } else {
  //     unsigned TransOp16 =
  //       DestReg == C65::D ? C65::TCD : 0;
  //     if (TransOp16) {
  //       BuildMI(MBB, MBBI, DL, get(C65::LDA16zp))
  //         .addImm(RI.getZRAddress(SrcReg));
  //       BuildMI(MBB, MBBI, DL, get(TransOp16));
  //     } else {
  //       llvm_unreachable("Cannot copy Z register to this register.");
  //     }
  //   }
  // } else if (ZRegDest) {
  //   unsigned SimpleOp =
  //     SrcReg == C65::C ? C65::STA16zp :
  //     SrcReg == C65::X ? C65::STX16zp :
  //     SrcReg == C65::Y ? C65::STY16zp :
  //     SrcReg == C65::A ? C65::STA8zp :
  //     SrcReg == C65::XL ? C65::STX8zp :
  //     SrcReg == C65::YL ? C65::STY8zp : 0;
  //   if (SimpleOp) {
  //     BuildMI(MBB, MBBI, DL, get(SimpleOp))
  //       .addImm(RI.getZRAddress(SrcReg));
  //   } else {
  //     unsigned TransOp16 =
  //       SrcReg == C65::D ? C65::TDC :
  //       SrcReg == C65::S ? C65::TSC : 0;
  //     if (TransOp16) {
  //       BuildMI(MBB, MBBI, DL, get(TransOp16));
  //       BuildMI(MBB, MBBI, DL, get(C65::STA16zp))
  //         .addImm(RI.getZRAddress(DestReg));
  //     } else {
  //       llvm_unreachable("Cannot copy Z register from this register.");
  //     }
  //   }
  // } else {
  //   llvm_unreachable("Normal reg-to-reg copy not implemented.");
  // }

  LLVM_DEBUG(dbgs() << "CopyPhysReg from "
               << RI.getName(DestReg)
               << " to " << RI.getName(SrcReg) << '\n');
  // if (SrcReg == C65::A && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TAX));
  // } else if (SrcReg == C65::A && DestReg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::TAY));
  // } else if (SrcReg == C65::X && DestReg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::TXA));
  // } else if (SrcReg == C65::Y && DestReg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::TYA));
  // } else if (SrcReg == C65::X && DestReg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::TXY));
  // } else if (SrcReg == C65::Y && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TYX));
  // } else if (SrcReg == C65::X && DestReg == C65::S) {
  //   BuildMI(MBB, MI, DL, get(C65::TXS));
  // } else if (SrcReg == C65::S && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TSX));
  // } else if (SrcReg == C65::P || DestReg == C65::P ||
  //            SrcReg == C65::D || DestReg == C65::D) {
  //   buildPushReg(MBB, MI, DL, DestReg);
  //   buildPullReg(MBB, MI, DL, SrcReg);
  // } else {
  //   LLVM_DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg)
  //                << " to " << RI.getName(DestReg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy");
  // }
}

unsigned C65InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  if (MI.getOpcode() == C65::ZST8s ||
      MI.getOpcode() == C65::ZST16s ||
      MI.getOpcode() == C65::ZST32s ||
      MI.getOpcode() == C65::ZST64s) {
    if (MI.getOperand(1).isFI()) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void C65InstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB,
                    MachineBasicBlock::iterator MBBI,
                    unsigned SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  unsigned Opcode;
  if (RC == &C65::ZRC8RegClass)
    Opcode = C65::ZST8s;
  else if (RC == &C65::ZRC16RegClass)
    Opcode = C65::ZST16s;
  else if (RC == &C65::ZRC32RegClass)
    Opcode = C65::ZST32s;
  else if (RC == &C65::ZRC64RegClass)
    Opcode = C65::ZST64s;
  else
    llvm_unreachable("Unable to store reg from stack slot.");
  BuildMI(MBB, MBBI, DL, get(Opcode))
    .addReg(SrcReg, getKillRegState(isKill))
    .addFrameIndex(FI)
    .addImm(0);
}

unsigned C65InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  if (MI.getOpcode() == C65::ZLD8s ||
      MI.getOpcode() == C65::ZLD16s ||
      MI.getOpcode() == C65::ZLD32s ||
      MI.getOpcode() == C65::ZLD64s) {
    if (MI.getOperand(1).isFI()) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
  }
  return 0;
}

void C65InstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator MBBI,
                     unsigned DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  unsigned Opcode;
  if (RC == &C65::ZRC8RegClass)
    Opcode = C65::ZLD8s;
  else if (RC == &C65::ZRC16RegClass)
    Opcode = C65::ZLD16s;
  else if (RC == &C65::ZRC32RegClass)
    Opcode = C65::ZLD32s;
  else if (RC == &C65::ZRC64RegClass)
    Opcode = C65::ZLD64s;
  else
    llvm_unreachable("Unable to load reg from stack slot.");
  BuildMI(MBB, MBBI, DL, get(Opcode), DestReg)
    .addFrameIndex(FI)
    .addImm(0);
}

bool C65InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  return false;
}

bool C65InstrInfo::isPredicated(const MachineInstr &MI) const {
  return false;
}

bool C65InstrInfo::isUnpredicatedTerminator(const MachineInstr &MI) const {
  if (!MI.isTerminator())
    return false;

  // Conditional branch is a special case.
  if (MI.isBranch() && !MI.isBarrier())
    return true;

  return !isPredicated(MI);
}

// Branch analysis.
bool C65InstrInfo::analyzeBranch(MachineBasicBlock &MBB,MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();
  while (I != MBB.begin()) {
    --I;

    if (I->isDebugValue())
      continue;

    // Working from the bottom, we see a non-terminator, we are done.
    if (!isUnpredicatedTerminator(*I))
      break;

    // Terminator is not a branch.
    if (!I->isBranch())
      return true;

    // Handle Unconditional branches.
    if (I->getOpcode() == C65::BRA ||
        I->getOpcode() == C65::BRL ||
        I->getOpcode() == C65::JMPabs ||
        I->getOpcode() == C65::JMLabsl) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // Delete all instructions after an unconditional branch.
      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();

      // Any previously encountered conditional branches are
      // irrelevant now.
      Cond.clear();
      FBB = nullptr;

      // Delete the branch if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // If we're here, that means that the current instruction should
    // be a conditional branch.
    unsigned Opcode = I->getOpcode();
    if (Opcode != C65::ZBRCC8 &&
        Opcode != C65::ZBRCC16 &&
        Opcode != C65::ZBRCC32 &&
        Opcode != C65::ZBRCC64)
      return true; // Unknown Opcode.

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      FBB = TBB;
      Cond.push_back(MachineOperand::CreateImm(Opcode));
      Cond.push_back(I->getOperand(0));
      Cond.push_back(I->getOperand(1));
      Cond.push_back(I->getOperand(2));
      TBB = I->getOperand(3).getMBB();
      continue;
    }
    // FIXME: Handle subsequent conditional branches.
    // For now, we can't handle multiple conditional branches.
    return true;
  }
  return false;
}

unsigned C65InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  assert(!BytesRemoved && "code size not handled");

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;
  while (I != MBB.begin()) {
    --I;

    while (I->isDebugValue())
      continue;

    if (I->getOpcode() != C65::BRA && I->getOpcode() != C65::BRL &&
        I->getOpcode() != C65::JMPabs && I->getOpcode() != C65::JMLabsl &&
        I->getOpcode() != C65::ZBRCC8 && I->getOpcode() != C65::ZBRCC16 &&
        I->getOpcode() != C65::ZBRCC32 && I->getOpcode() != C65::ZBRCC64)
      break;

    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }
  return Count;
}

unsigned
C65InstrInfo::insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                           MachineBasicBlock *FBB,
                           ArrayRef<MachineOperand> Cond,
                           const DebugLoc &DL,
                           int *BytesAdded) const {
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 4 || Cond.size() == 0) &&
         "C65 branch conditions have four components!");

  // Unconditional branch.
  if(Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    BuildMI(&MBB, DL, get(C65::JMPabs)).addMBB(TBB);
    return 1;
  }

  unsigned Instr = Cond[0].getImm();

  // if (C65::ZRC8RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
  //   Instr = C65::ZBRCC8;
  // else if (C65::ZRC16RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
  //   Instr = C65::ZBRCC16;
  // else if (C65::ZRC32RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
  //   Instr = C65::ZBRCC32;
  // else if (C65::ZRC64RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
  //   Instr = C65::ZBRCC64;
  // else
  //   llvm_unreachable("Unexpected BRCC operands.");

  BuildMI(&MBB, DL, get(Instr))
    .add(Cond[1])
    .add(Cond[2])
    .add(Cond[3])
    .addMBB(TBB);

  if (!FBB)
    return 1;

  BuildMI(&MBB, DL, get(C65::JMPabs)).addMBB(FBB);
  return 2;
}
