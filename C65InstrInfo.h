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

  // Is shift-type instruction
  ZRShift = (1 << 0),

  // Register size
  ZROpSize = (3 << 1),
  ZROpSizeShift = 1,
  ZROpSize8 = 0,
  ZROpSize16 = 1,
  ZROpSize32 = 2,
  ZROpSize64 = 2,

  // Is a ZR instruction
  ZRInstr = (1 << 3),

  // Is a control flow instruction
  ZRCtrl = (1 << 4)
};
static inline unsigned getZROpSize(unsigned int Flags) {
  return (Flags & ZROpSize) >> ZROpSizeShift;
}

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

  bool ExpandBR_CC(MachineInstr *MI,
                   unsigned NumBytes) const;

  bool expandZRInstr(MachineBasicBlock::iterator MBBI,
                     unsigned Instruction) const;

  bool expandPostRAPseudo(MachineBasicBlock::iterator MBBI) const override;
};
} // end namespace llvm

#endif
