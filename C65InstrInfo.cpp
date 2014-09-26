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
                                DebugLoc DL, unsigned Reg) const {
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
  //   DEBUG(dbgs() << "Cannot push " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy push");
  // }
}

void C65InstrInfo::buildPullReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                DebugLoc DL, unsigned Reg) const {
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
  //   DEBUG(dbgs() << "Cannot pull " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy pull");
  // }
}

void C65InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MBBI,
                               DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
  const unsigned ByteCapacity = 2;
  unsigned NumBytes;
  unsigned LDAInstr;
  unsigned STAInstr;

  if (C65::ZRC8RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 1;
  } else if (C65::ZRC16RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 2;
  } else if (C65::ZRC32RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 4;
  } else if (C65::ZRC64RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 8;
  } else {
    llvm_unreachable("Impossible reg-to-reg copy");
  }

  if (NumBytes == 1 || ByteCapacity == 1) {
    LDAInstr = C65::LDA_8zp;
    STAInstr = C65::STA_8zp;
  } else {
    LDAInstr = C65::LDA_16zp;
    STAInstr = C65::STA_16zp;
  }

  for (unsigned I = 0; I < NumBytes; I += ByteCapacity) {
    BuildMI(MBB, MBBI, DL, get(LDAInstr))
      .addImm(RI.getZRAddress(SrcReg) + I);
    BuildMI(MBB, MBBI, DL, get(STAInstr))
      .addImm(RI.getZRAddress(DestReg) + I);
  }

  DEBUG(dbgs() << "CopyPhysReg from "
               << RI.getName(DestReg)
               << " to " << RI.getName(SrcReg) << '\n');
  MBB.dump();
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
  //   DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg)
  //                << " to " << RI.getName(DestReg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy");
  // }
}

unsigned C65InstrInfo::isStoreToStackSlot(const MachineInstr *MI,
                                          int &FrameIndex) const {
  // if (MI->getOpcode() == C65::STAis) {
  //   if (MI->getOperand(0).isFI()) {
  //     FrameIndex = MI->getOperand(0).getIndex();
  //     return C65::A;
  //   }
  // }
  return 0;
}

void C65InstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB,
                    MachineBasicBlock::iterator MBBI,
                    unsigned SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  // DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  // if (SrcReg == C65::A) {
  //   BuildMI(MBB, MBBI, DL, get(C65::STAis)).addFrameIndex(FI);
  // } else {
  //   DEBUG(dbgs() << "Cannot store " << RI.getName(SrcReg)
  //                << " to stack frame\n");
  //   llvm_unreachable("Unable to store reg from stack slot.");
  // }
}

unsigned C65InstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                           int &FrameIndex) const {
  // if (MI->getOpcode() == C65::LDAis) {
  //   if (MI->getOperand(0).isFI()) {
  //     FrameIndex = MI->getOperand(0).getIndex();
  //     return C65::A;
  //   }
  // }
  return 0;
}

void C65InstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator MBBI,
                     unsigned DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  // DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  // if (DestReg == C65::A) {
  //   BuildMI(MBB, MBBI, DL, get(C65::LDAis)).addFrameIndex(FI);
  // } else {
  //   DEBUG(dbgs() << "Cannot load " << RI.getName(DestReg)
  //                << " from stack frame\n");
  //   llvm_unreachable("Unable to load reg from stack slot.");
  // }
}

bool C65InstrInfo::expandPostRAPseudo(MachineBasicBlock::iterator MBBI) const {
  return false;
}

bool C65InstrInfo::isPredicated(const MachineInstr *MI) const {
  return false;
}

bool C65InstrInfo::isUnpredicatedTerminator(const MachineInstr *MI) const {
  if (!MI->isTerminator())
    return false;

  // Conditional branch is a special case.
  if (MI->isBranch() && !MI->isBarrier())
    return true;

  return !isPredicated(MI);
}

// Branch analysis.
bool C65InstrInfo::AnalyzeBranch(MachineBasicBlock &MBB,MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {


  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();
  while (I != MBB.begin()) {
    --I;

    if (I->isDebugValue())
      continue;

    // When we see a non-terminator, we are done.
    if (!isUnpredicatedTerminator(I))
      break;

    // Terminator is not a branch.
    if (!I->isBranch())
      return true;

    // Handle Unconditional branches.
    if (I->getOpcode() == C65::JMP) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();

      Cond.clear();
      FBB = nullptr;

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

    unsigned Opcode = I->getOpcode();
    if (Opcode != C65::ZBRCC_8 &&
        Opcode != C65::ZBRCC_16 &&
        Opcode != C65::ZBRCC_32 &&
        Opcode != C65::ZBRCC_64)
      return true; // Unknown Opcode.

    if (Cond.empty()) {
      FBB = TBB;
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

unsigned C65InstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;
  while (I != MBB.begin()) {
    --I;

    while (I->isDebugValue())
      continue;

    if (I->getOpcode() != C65::JMP && I->getOpcode() != C65::ZBRCC_8 &&
        I->getOpcode() != C65::ZBRCC_16 && I->getOpcode() != C65::ZBRCC_32 &&
        I->getOpcode() != C65::ZBRCC_64)
      break;

    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }
  return Count;
}

unsigned
C65InstrInfo::InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                           MachineBasicBlock *FBB,
                           const SmallVectorImpl<MachineOperand> &Cond,
                           DebugLoc DL) const {
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 3 || Cond.size() == 0) &&
         "C65 branch conditions have three components!");

  if(Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    BuildMI(&MBB, DL, get(C65::JMP)).addMBB(TBB);
    return 1;
  }

  unsigned Instr;

  if (C65::ZRC8RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
    Instr = C65::ZBRCC_8;
  else if (C65::ZRC16RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
    Instr = C65::ZBRCC_16;
  else if (C65::ZRC32RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
    Instr = C65::ZBRCC_32;
  else if (C65::ZRC64RegClass.contains(Cond[1].getReg(), Cond[2].getReg()))
    Instr = C65::ZBRCC_64;
  else
    llvm_unreachable("Unexpected BRCC operands.");

  BuildMI(&MBB, DL, get(Instr))
    .addOperand(Cond[0])
    .addOperand(Cond[1])
    .addOperand(Cond[2])
    .addMBB(TBB);

  if (!FBB)
    return 1;

  BuildMI(&MBB, DL, get(C65::JMP)).addMBB(FBB);
  return 2;
}
