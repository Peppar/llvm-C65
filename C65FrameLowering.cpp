//===-- C65FrameLowering.cpp - C65 Frame Information ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the C65 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "C65FrameLowering.h"
#include "C65RegisterInfo.h"
#include "C65InstrInfo.h"
#include "C65Subtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetOptions.h"

using namespace llvm;

C65FrameLowering::C65FrameLowering(const C65TargetMachine &TM,
				   const C65Subtarget &ST)
  : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 1, 0),
    TM(TM), ST(ST) {}

void C65FrameLowering::emitPrologue(MachineFunction &MF) const {
  assert(MF.getFrameInfo()->getStackSize() == 0 &&
         "Non-null stack frame not supported.");
}

void C65FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  assert(MF.getFrameInfo()->getStackSize() == 0 &&
         "Non-null stack frame not supported.");
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool C65FrameLowering::hasFP(const MachineFunction &MF) const {
  return false;
}
