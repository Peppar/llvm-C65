//===-- C65InstrInfo.h - C65 instruction information ------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the 6502 compatibles implementation of the
// TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_C65INSTRINFO_H
#define LLVM_LIB_TARGET_C65_C65INSTRINFO_H

#include "C65.h"
#include "C65RegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "C65GenInstrInfo.inc"

namespace llvm {

class C65TargetMachine;
class C65Subtarget;
class C65InstrInfo : public C65GenInstrInfo {
  const C65RegisterInfo RI;
  C65Subtarget &STI;

  virtual void anchor();

public:
  explicit C65InstrInfo(C65Subtarget &STI);

  // Override TargetInstrInfo
  unsigned isLoadFromStackSlot(const MachineInstr &MI,
                               int &FrameIndex) const override;
  unsigned isStoreToStackSlot(const MachineInstr &MI,
                              int &FrameIndex) const override;
  void buildPushReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                    const DebugLoc &DL, unsigned Reg) const;
  void buildPullReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                    const DebugLoc &DL, unsigned Reg) const;
  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc) const override;
  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI,
                           Register SrcReg, bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override;
  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            Register DestReg, int FrameIndex,
                            const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override;
  const C65RegisterInfo &getRegisterInfo() const { return RI; }

  bool ExpandBR_CC(MachineInstr &MI,
                   unsigned NumBytes) const;

  bool expandPostRAPseudo(MachineInstr &MI) const override;

  // Predication support.
  bool isPredicated(const MachineInstr &MI) const override;

  bool isUnpredicatedTerminator(const MachineInstr &MI) const override;

  // Branch analysis.
  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify) const override;
  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override;
  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override;
};
} // end namespace llvm

#endif
