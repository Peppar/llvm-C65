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
#include "C65InstrInfo.h"
#include "C65RegisterInfo.h"
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
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();
  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  //  MachineModuleInfo &MMI = MF.getMMI();
  //  const MCRegisterInfo *MRI = MMI.getContext().getRegisterInfo();
  //  const std::vector<CalleeSavedInfo> &CSI = MFFrame->getCalleeSavedInfo();
  //  bool HasFP = hasFP(MF);
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  int NumBytes = (int)MFI->getStackSize();

  while (NumBytes > 0) {
    // Push 2 bytes
    BuildMI(MBB, MBBI, DL, TII.get(C65::PHA));
    NumBytes -= 2;
  }
}

void C65FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  const TargetInstrInfo &TII = *MF.getTarget().getInstrInfo();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI != MBB.end() && "Returning block has no instructions");
  DebugLoc DL = MBBI->getDebugLoc();

  int NumBytes = (int)MFI->getStackSize();

  while (NumBytes > 0) {
    // Pull 2 bytes
    BuildMI(MBB, MBBI, DL, TII.get(C65::PLA));
    NumBytes -= 2;
  }
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool C65FrameLowering::hasFP(const MachineFunction &MF) const {
  // const MachineFrameInfo *MFI = MF.getFrameInfo();
  // return MF.getTarget().Options.DisableFramePointerElim(MF) ||
  //   MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();
  return false;
}
