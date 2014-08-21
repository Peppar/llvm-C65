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
  // TODO: Add registers that are to be saved by the callee
  static const MCPhysReg CSR_SaveList[] = { 0 };
  return CSR_SaveList;
}

BitVector C65RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  // TODO: Mark which registers are reserved and cannot be used
  // by the LLVM register allocation algorithms
  BitVector Reserved(getNumRegs());
  return Reserved;
}

const TargetRegisterClass*
C65RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  // TODO: Add the remaining pointer classes
  return &C65::IX16RegClass;
}

void
C65RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                     int SPAdj, unsigned FIOperandNum,
                                     RegScavenger *RS) const {
  // TODO: Add frame index support
  assert(SPAdj == 0 && "Unexpected");
}

unsigned C65RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  // TODO: Add full frame register support
  return C65::X;
}
