//===-- C65RegisterInfo.cpp - C65 Register Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the C65 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "C65.h"
#include "C65RegisterInfo.h"
//#include "C65MachineFunctionInfo.h"
#include "C65Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetInstrInfo.h"

using namespace llvm;

#define GET_REGINFO_TARGET_DESC
#include "C65GenRegisterInfo.inc"

C65RegisterInfo::C65RegisterInfo(C65Subtarget &ST)
  : C65GenRegisterInfo(C65::PC), Subtarget(ST) {}

const MCPhysReg*
C65RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const MCPhysReg CSR_SaveList[] = { 0 };
  return CSR_SaveList;
}

BitVector C65RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  Reserved.set(C65::SP);
  Reserved.set(C65::DP);
  Reserved.set(C65::PC);
  Reserved.set(C65::SR);
  Reserved.set(C65::PB);
  Reserved.set(C65::DB);
  return Reserved;
}

const TargetRegisterClass*
C65RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  if (Kind == 0)
    return &C65::IX16RegClass;
  else
    return &C65::IY16RegClass;
}

void
C65RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                     int SPAdj, unsigned FIOperandNum,
                                     RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected");

  // // Get the instruction.
  // MachineInstr &MI = *II;
  // // Get the instruction's basic block.
  // MachineBasicBlock &MBB = *MI.getParent();
  // // Get the basic block's function.
  // MachineFunction &MF = *MBB.getParent();
  // // Get the instruction info.
  // const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();
  // // Get the frame info.
  // MachineFrameInfo *MFI = MF.getFrameInfo();
  // DebugLoc dl = MI.getDebugLoc();
  // int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  // MachineFunction &MF = *MI.getParent()->getParent();
  // int64_t Offset = MF.getFrameInfo()->getObjectOffset(FrameIndex);
  // uint64_t StackSize = MF.getFrameInfo()->getStackSize();
}

unsigned C65RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return C65::X;
}

