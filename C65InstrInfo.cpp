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
  if (Reg == C65::A) {
    BuildMI(MBB, MI, DL, get(C65::PHA));
  } else if (Reg == C65::X) {
    BuildMI(MBB, MI, DL, get(C65::PHX));
  } else if (Reg == C65::Y) {
    BuildMI(MBB, MI, DL, get(C65::PHY));
  } else if (Reg == C65::D) {
    BuildMI(MBB, MI, DL, get(C65::PHD));
  } else if (Reg == C65::P) {
    BuildMI(MBB, MI, DL, get(C65::PHP));
  } else {
    DEBUG(dbgs() << "Cannot push " << RI.getName(Reg) << '\n');
    llvm_unreachable("Impossible reg-to-reg copy push");
  }
}

void C65InstrInfo::buildPullReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                DebugLoc DL, unsigned Reg) const {
  if (Reg == C65::A) {
    BuildMI(MBB, MI, DL, get(C65::PLA));
  } else if (Reg == C65::X) {
    BuildMI(MBB, MI, DL, get(C65::PLX));
  } else if (Reg == C65::Y) {
    BuildMI(MBB, MI, DL, get(C65::PLY));
  } else if (Reg == C65::D) {
    BuildMI(MBB, MI, DL, get(C65::PLD));
  } else if (Reg == C65::P) {
    BuildMI(MBB, MI, DL, get(C65::PLP));
  } else {
    DEBUG(dbgs() << "Cannot pull " << RI.getName(Reg) << '\n');
    llvm_unreachable("Impossible reg-to-reg copy pull");
  }
}

void C65InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI, DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
  if (SrcReg == C65::A && DestReg == C65::X) {
    BuildMI(MBB, MI, DL, get(C65::TAX));
  } else if (SrcReg == C65::A && DestReg == C65::Y) {
    BuildMI(MBB, MI, DL, get(C65::TAY));
  } else if (SrcReg == C65::X && DestReg == C65::A) {
    BuildMI(MBB, MI, DL, get(C65::TXA));
  } else if (SrcReg == C65::Y && DestReg == C65::A) {
    BuildMI(MBB, MI, DL, get(C65::TYA));
  } else if (SrcReg == C65::X && DestReg == C65::Y) {
    BuildMI(MBB, MI, DL, get(C65::TXY));
  } else if (SrcReg == C65::Y && DestReg == C65::X) {
    BuildMI(MBB, MI, DL, get(C65::TYX));
  } else if (SrcReg == C65::X && DestReg == C65::S) {
    BuildMI(MBB, MI, DL, get(C65::TXS));
  } else if (SrcReg == C65::S && DestReg == C65::X) {
    BuildMI(MBB, MI, DL, get(C65::TSX));
  } else if (SrcReg == C65::P || DestReg == C65::P ||
             SrcReg == C65::D || DestReg == C65::D) {
    buildPushReg(MBB, MI, DL, DestReg);
    buildPullReg(MBB, MI, DL, SrcReg);
  } else {
    DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg)
                 << " to " << RI.getName(DestReg) << '\n');
    llvm_unreachable("Impossible reg-to-reg copy");
  }
}

unsigned C65InstrInfo::isStoreToStackSlot(const MachineInstr *MI,
                                          int &FrameIndex) const {
  if (MI->getOpcode() == C65::STAis) {
    if (MI->getOperand(0).isFI()) {
      FrameIndex = MI->getOperand(0).getIndex();
      return C65::A;
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
  if (SrcReg == C65::A) {
    BuildMI(MBB, MBBI, DL, get(C65::STAis)).addFrameIndex(FI);
  } else {
    DEBUG(dbgs() << "Cannot store " << RI.getName(SrcReg)
                 << " to stack frame\n");
    llvm_unreachable("Unable to store reg from stack slot.");
  }
}

unsigned C65InstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                           int &FrameIndex) const {
  if (MI->getOpcode() == C65::LDAis) {
    if (MI->getOperand(0).isFI()) {
      FrameIndex = MI->getOperand(0).getIndex();
      return C65::A;
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
  if (DestReg == C65::A) {
    BuildMI(MBB, MBBI, DL, get(C65::LDAis)).addFrameIndex(FI);
  } else {
    DEBUG(dbgs() << "Cannot load " << RI.getName(DestReg)
                 << " from stack frame\n");
    llvm_unreachable("Unable to load reg from stack slot.");
  }
}
