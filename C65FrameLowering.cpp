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

C65FrameLowering::C65FrameLowering(const C65Subtarget &ST)
  : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 1, 0),
    ST(ST) {}

void C65FrameLowering::emitPrologue(MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineBasicBlock &MBB = MF.front();
  MachineBasicBlock::iterator MBBI = MBB.begin();
  int Size = (int)MFI->getStackSize();
  if (Size)
    emitSAdjustment(MF, MBB, MBBI, -Size);
}

void C65FrameLowering::emitSAdjustment(MachineFunction &MF,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MBBI,
                                       int NumBytes) const {
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const C65InstrInfo &TII = *ST.getInstrInfo();
  const int PushPullThreshold = 10;

  if (NumBytes >= PushPullThreshold) {
    BuildMI(MBB, MBBI, DL, TII.get(C65::TSC));
    BuildMI(MBB, MBBI, DL, TII.get(C65::CLC));
    BuildMI(MBB, MBBI, DL, TII.get(C65::ADC16imm))
      .addImm(NumBytes);
    BuildMI(MBB, MBBI, DL, TII.get(C65::TCS));
  } else if (NumBytes <= -PushPullThreshold) {
    BuildMI(MBB, MBBI, DL, TII.get(C65::TSC));
    BuildMI(MBB, MBBI, DL, TII.get(C65::SEC));
    BuildMI(MBB, MBBI, DL, TII.get(C65::SBC16imm))
      .addImm(-NumBytes);
    BuildMI(MBB, MBBI, DL, TII.get(C65::TCS));
  } else {
    while (NumBytes != 0) {
      if (NumBytes >= 2) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::PLA16));
        NumBytes -= 2;
      } else if (NumBytes >= 1) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::PLA8));
        NumBytes -= 1;
      } else if (NumBytes <= -2) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::PHA16));
        NumBytes += 2;
      } else if (NumBytes <= -1) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::PHP));
        NumBytes += 1;
      }
    }
  }
}

// TODO: Implement this
// bool C65FrameLowering::spillCalleeSavedRegisters(
//     MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
//     const std::vector<CalleeSavedInfo> &CSI,
//     const TargetRegisterInfo *TRI) const {
//   DebugLoc DL = MBB.findDebugLoc(MI);

//   MachineFunction &MF = *MBB.getParent();
//   const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
//   const C65Subtarget &STI = MF.getTarget().getSubtarget<C65Subtarget>();

//   // Push GPRs. It increases frame size.
//   unsigned Opc = STI.is64Bit() ? X86::PUSH64r : X86::PUSH32r;
//   for (unsigned I = CSI.size(); I != 0; --I) {
//     unsigned Reg = CSI[I - 1].getReg();

//     if (!X86::GR64RegClass.contains(Reg) && !X86::GR32RegClass.contains(Reg))
//       continue;
//     // Add the callee-saved register as live-in. It's killed at the spill.
//     MBB.addLiveIn(Reg);

//     BuildMI(MBB, MI, DL, TII.get(Opc)).addReg(Reg, RegState::Kill)
//       .setMIFlag(MachineInstr::FrameSetup);
//   }

//   // Make XMM regs spilled. X86 does not have ability of push/pop XMM.
//   // It can be done by spilling XMMs to stack frame.
//   for (unsigned i = CSI.size(); i != 0; --i) {
//     unsigned Reg = CSI[i-1].getReg();
//     if (X86::GR64RegClass.contains(Reg) ||
//         X86::GR32RegClass.contains(Reg))
//       continue;
//     // Add the callee-saved register as live-in. It's killed at the spill.
//     MBB.addLiveIn(Reg);
//     const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);

//     TII.storeRegToStackSlot(MBB, MI, Reg, true, CSI[i - 1].getFrameIdx(), RC,
//                             TRI);
//     --MI;
//     MI->setFlag(MachineInstr::FrameSetup);
//     ++MI;
//   }

//   return true;
// }


void C65FrameLowering::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {
  MachineInstr &MI = *I;
  int Size = MI.getOperand(0).getImm();
  if (Size) {
    if (MI.getOpcode() == C65::ADJCALLSTACKDOWN)
      emitSAdjustment(MF, MBB, I, -Size);
    else
      emitSAdjustment(MF, MBB, I, Size);
  }
  MBB.erase(I);
}

void C65FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getOpcode() == C65::RTS &&
         "Can only put epilogue before 'rts' instruction!");
  int Size = (int)MFI->getStackSize();
  if (Size)
    emitSAdjustment(MF, MBB, MBBI, Size);
}

// hasFP - Return true if the specified function should have a
// dedicated frame pointer register.  This is true if the function has
// variable sized allocas or if frame pointer elimination is disabled.
// In addition, for C65 as well when we do not have 65802 capabilities (TSC,
// TCS) or when the frame is too large for stack relative indexing (255 bytes)
bool C65FrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo *MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
    MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken() ||
    MFI->getStackSize() > 0xFF ||
    !ST.has65802();
}
