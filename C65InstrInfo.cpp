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
//#include "C65MachineFunctionInfo.h"
//#include "C65Subtarget.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
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

void C65InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator I, DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
  if (SrcReg == C65::A && DestReg == C65::X)
    BuildMI(MBB, I, DL, get(C65::TAX));
  else if (SrcReg == C65::A && DestReg == C65::Y)
    BuildMI(MBB, I, DL, get(C65::TAY));
  else if (SrcReg == C65::X && DestReg == C65::A)
    BuildMI(MBB, I, DL, get(C65::TXA));
  else if (SrcReg == C65::Y && DestReg == C65::A)
    BuildMI(MBB, I, DL, get(C65::TYA));
  else if (SrcReg == C65::X && DestReg == C65::Y)
    BuildMI(MBB, I, DL, get(C65::TXY));
  else if (SrcReg == C65::Y && DestReg == C65::X)
    BuildMI(MBB, I, DL, get(C65::TYX));
  else {
    DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg)
                 << " to " << RI.getName(DestReg) << '\n');
    llvm_unreachable("Impossible reg-to-reg copy");
  }
}

unsigned C65InstrInfo::isStoreToStackSlot(const MachineInstr *MI,
                                            int &FrameIndex) const {
  switch (MI->getOpcode()) {
  default: break;
  case C65::PLA:
    FrameIndex = 0;
    return C65::A;
  case C65::PLX:
    FrameIndex = 0;
    return C65::X;
  case C65::PLY:
    FrameIndex = 0;
    return C65::Y;
  }
  return 0;
}

void C65InstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB,
                    MachineBasicBlock::iterator I,
                    unsigned SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();

  if (SrcReg == C65::A)
    BuildMI(MBB, I, DL, get(C65::PHA));
  else if (SrcReg == C65::X)
    BuildMI(MBB, I, DL, get(C65::PHX));
  else if (SrcReg == C65::Y)
    BuildMI(MBB, I, DL, get(C65::PHY));
  else {
    DEBUG(dbgs() << "Cannot store " << RI.getName(SrcReg)
                 << " to stack slot\n");
    llvm_unreachable("Cannot store register to stack slot");
  }
}

unsigned C65InstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                           int &FrameIndex) const {
  switch (MI->getOpcode()) {
  default: break;
  case C65::PLA:
    FrameIndex = 0;
    return C65::A;
  case C65::PLX:
    FrameIndex = 0;
    return C65::X;
  case C65::PLY:
    FrameIndex = 0;
    return C65::Y;
  }
  return 0;
}

void C65InstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator I,
                     unsigned DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (I != MBB.end()) DL = I->getDebugLoc();

  if (DestReg == C65::A)
    BuildMI(MBB, I, DL, get(C65::PLA));
  else if (DestReg == C65::X)
    BuildMI(MBB, I, DL, get(C65::PLX));
  else if (DestReg == C65::Y)
    BuildMI(MBB, I, DL, get(C65::PLY));
  else {
    DEBUG(dbgs() << "Cannot load " << RI.getName(DestReg)
                 << " from stack slot\n");
    llvm_unreachable("Cannot load this register from stack slot");
  }
}
