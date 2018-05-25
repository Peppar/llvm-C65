//===-- C65FrameLowering.h - Define frame lowering for C65 ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//
//
//===----------------------------------------------------------------------===//

#ifndef C65_FRAMEINFO_H
#define C65_FRAMEINFO_H

#include "C65.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class C65Subtarget;

class C65FrameLowering : public TargetFrameLowering {
public:
  explicit C65FrameLowering();

  void emitSAdjustment(MachineFunction &MF,
                       MachineBasicBlock &MBB,
                       MachineBasicBlock::iterator MBBI,
                       int NumBytes) const;

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator I) const override;

  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;

  bool hasFP(const MachineFunction &MF) const override;
};

} // end llvm namespace

#endif
