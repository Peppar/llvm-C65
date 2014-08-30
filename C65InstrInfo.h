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

namespace C65 {

enum {
  // This needs to be kept in sync with C65InstrInfo.td TSFlags

  // Accumulator size requirement
  AccSize = (1 << 0),
  AccSizeShift = 0,
  AccSizeNC = 0,
  AccSize8 = 1,
  AccSize16 = 2,
};
static inline unsigned getAccSize(unsigned int Flags) {
  return (Flags & AccSize) >> AccSizeShift;
}


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
  void buildPushReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                    DebugLoc DL, unsigned Reg) const;
  void buildPullReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                    DebugLoc DL, unsigned Reg) const;
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
  const C65RegisterInfo &getRegisterInfo() const { return RI; }
};
} // end namespace llvm

#endif
