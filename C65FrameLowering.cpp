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
//#include "C65MachineFunctionInfo.h"
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

C65FrameLowering::C65FrameLowering(const C65Subtarget &ST)
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 1, 0) {}

void C65FrameLowering::emitPrologue(MachineFunction &MF) const {
  MachineBasicBlock &MBB = MF.front();
  MachineFrameInfo *MFI = MF.getFrameInfo();
  const C65InstrInfo &TII =
    *static_cast<const C65InstrInfo *>(MF.getTarget().getInstrInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Get the number of bytes to allocate from the FrameInfo
  uint64_t NumBytes = MFI->getStackSize();

  MachineInstr *MI;

  assert(!hasFP(MF) && "Frame pointer not supported");

  // Push to the stack to move the stack pointer NumBytes down
  while (NumBytes) {
    if (NumBytes == 1) {
      // Set 8-bit accumulator, then push 1 byte,
      // then set 16-bit accumulator. This doesn't modify
      // the accumulator value.
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::REP))
	.addImm(0x20);
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::PHA));
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::SEP))
	.addImm(0x20);
      NumBytes -= 1;
    } else {
      // Push 2 bytes
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::PHA));
      NumBytes -= 2;
    }
  }
}

void C65FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  const C65InstrInfo &TII =
    *static_cast<const C65InstrInfo *>(MF.getTarget().getInstrInfo());
  DebugLoc DL = MBBI->getDebugLoc();

  // Get the number of bytes to allocate from the FrameInfo
  uint64_t NumBytes = MFI->getStackSize();

  MachineInstr *MI;

  assert(!hasFP(MF) && "Frame pointer not supported");

  // Pull (pop) from the stack to move the stack pointer NumBytes up
  while (NumBytes) {
    if (NumBytes == 1) {
      // Set 8-bit accumulator, then pull 1 byte,
      // then set 16-bit accumulator.
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::REP))
	.addImm(0x20);
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::PLA));
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::SEP))
	.addImm(0x20);
      NumBytes -= 1;
    } else {
      // Push 2 bytes
      MI = BuildMI(MBB, MBBI, DL, TII.get(C65::PLA));
      NumBytes -= 2;
    }
  }
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool C65FrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
    MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken();
}
