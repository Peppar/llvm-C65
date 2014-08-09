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

#ifndef LLVM_TARGET_C65INSTRINFO_H
#define LLVM_TARGET_C65INSTRINFO_H

#include "C65.h"
#include "C65RegisterInfo.h"
#include "llvm/Target/TargetInstrInfo.h"

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
  unsigned isLoadFromStackSlot(const MachineInstr *MI,
                               int &FrameIndex) const override;
  unsigned isStoreToStackSlot(const MachineInstr *MI,
                              int &FrameIndex) const override;
  void copyPhysReg(MachineBasicBlock &MBB,
                   MachineBasicBlock::iterator MBBI,
                   DebugLoc DL, unsigned DestReg, unsigned SrcReg,
                   bool KillSrc) const override;
  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MBBI,
                           unsigned SrcReg, bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override;
  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            unsigned DestReg, int FrameIdx,
                            const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override;
};
} // end namespace llvm

#endif
