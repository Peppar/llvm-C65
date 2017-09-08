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
#include "C65MachineFunctionInfo.h"
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

C65FrameLowering::C65FrameLowering()
  : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, 1, 0) {}

void C65FrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65RegisterInfo &RegInfo = *STI.getRegisterInfo();
  const C65InstrInfo &TII = *STI.getInstrInfo();
  const MachineFrameInfo &MFI = *MF.getFrameInfo();
  //C65MachineFunctionInfo *FuncInfo = MF.getInfo<C65MachineFunctionInfo>();

  bool HasFP = hasFP(MF);

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI->getDebugLoc();
  int Size = (int)MFI.getStackSize();

  unsigned FramePtr = RegInfo.getFrameRegister(MF);
  bool Is16Bit = STI.has65802();

  if (HasFP) {
    if (FramePtr == C65::X || FramePtr == C65::XL) {
      // Save the frame pointer.
      BuildMI(MBB, MBBI, DL, TII.get(Is16Bit ? C65::PHX16 : C65::PHX8))
        .setMIFlag(MachineInstr::FrameSetup);

      // Update the frame pointer with the new base value.
      BuildMI(MBB, MBBI, DL, TII.get(Is16Bit ? C65::TSX16 : C65::TSX8))
        .setMIFlag(MachineInstr::FrameSetup);

    } else
      llvm_unreachable("Only X may be a frame pointer");
  }
  if (Size) {
    emitSAdjustment(MF, MBB, MBBI, -Size);
    if (HasFP && !Is16Bit) {
      // emitSAdjustment overwrites X
      BuildMI(MBB, MBBI, DL, TII.get(C65::PLX8))
        .setMIFlag(MachineInstr::FrameSetup);
      BuildMI(MBB, MBBI, DL, TII.get(C65::PHX8))
        .setMIFlag(MachineInstr::FrameSetup);
    }
  }

  // Create the frame index object for the return address.
  // unsigned RetAddrSize = FuncInfo->getIsFar() ? 3 : 2;
  // int ReturnAddrIndex = MFI->CreateFixedObject(RetAddrSize,
  //                                              -(int64_t)RetAddrSize,
  //                                              false);
  // FuncInfo->setRAIndex(ReturnAddrIndex);
  //unsigned RetAddrSize = FuncInfo->getIsFar() ? 3 : 2;
  //int ReturnAddrIndex = MFI->CreateStackObject(RetAddrSize, 1,
  //                                             false);
  //FuncInfo->setRAIndex(ReturnAddrIndex);
}

void C65FrameLowering::emitSAdjustment(MachineFunction &MF,
                                       MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MBBI,
                                       int NumBytes) const {
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65InstrInfo &TII = *STI.getInstrInfo();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();

  if (STI.has65802()) {
    const int PushPullThreshold = 6;
    if (NumBytes >= PushPullThreshold
        || NumBytes <= -PushPullThreshold) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::TSC));
      if (NumBytes < 0) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::SEC));
        BuildMI(MBB, MBBI, DL, TII.get(C65::SBC16imm))
          .addImm(-NumBytes);
      } else {
        BuildMI(MBB, MBBI, DL, TII.get(C65::CLC));
        BuildMI(MBB, MBBI, DL, TII.get(C65::ADC16imm))
          .addImm(NumBytes);
      }
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
  } else {
    const int PushPullThreshold = 7;

    if (NumBytes >= PushPullThreshold
        || NumBytes <= -PushPullThreshold) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::TSX8));
      BuildMI(MBB, MBBI, DL, TII.get(C65::TXA8));
      if (NumBytes < 0) {
        BuildMI(MBB, MBBI, DL, TII.get(C65::SEC));
        BuildMI(MBB, MBBI, DL, TII.get(C65::SBC8imm))
          .addImm(-NumBytes);
      } else {
        BuildMI(MBB, MBBI, DL, TII.get(C65::CLC));
        BuildMI(MBB, MBBI, DL, TII.get(C65::ADC8imm))
          .addImm(NumBytes);
      }
      BuildMI(MBB, MBBI, DL, TII.get(C65::TAX8));
      BuildMI(MBB, MBBI, DL, TII.get(C65::TXS8));
    } else {
      while (NumBytes != 0) {
        if (NumBytes >= 1) {
          BuildMI(MBB, MBBI, DL, TII.get(C65::PLA8));
          --NumBytes;
        } else if (NumBytes <= -1) {
          BuildMI(MBB, MBBI, DL, TII.get(C65::PHA8));
          ++NumBytes;
        }
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
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65RegisterInfo &RegInfo = *STI.getRegisterInfo();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  const MachineFrameInfo &MFI = *MF.getFrameInfo();
  C65MachineFunctionInfo &FuncInfo = *MF.getInfo<C65MachineFunctionInfo>();

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  int Size = (int)MFI.getStackSize()
    + (int)FuncInfo.getBytesToPopOnReturn();
  bool Is16Bit = STI.has65802();
  unsigned FramePtr = RegInfo.getFrameRegister(MF);

  assert((MBBI->getOpcode() == C65::RTS ||
          MBBI->getOpcode() == C65::RTL) &&
         "Can only put epilogue before 'RTS' or 'RTL' instruction!");

  if (Size)
    emitSAdjustment(MF, MBB, MBBI, Size);

  if (hasFP(MF)) {
    // Restore the frame pointer.
    if (FramePtr == C65::X || FramePtr == C65::XL)
      BuildMI(MBB, MBBI, DL, TII.get(Is16Bit ? C65::PLX16 : C65::PLX8));
    else
      llvm_unreachable("Only X may be a frame pointer.");
  }
}

// hasFP - Return true if the specified function should have a
// dedicated frame pointer register.  This is true if the function has
// variable sized allocas or if frame pointer elimination is disabled.
// In addition, for C65 as well when we do not have 65802 capabilities (TSC,
// TCS) or when the frame is too large for stack relative indexing (255 bytes)
bool C65FrameLowering::hasFP(const MachineFunction &MF) const {
  //  const MachineFrameInfo *MFI = MF.getFrameInfo();
  return true;

  //  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
  //    MFI->hasVarSizedObjects() || MFI->isFrameAddressTaken() ||
  //    MFI->getStackSize() > 0xFF ||
  //    !ST.has65802();
}
