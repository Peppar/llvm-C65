//===-- C65RegisterInfo.h - C65 Register Information Impl -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the C65 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef C65REGISTERINFO_H
#define C65REGISTERINFO_H

#include "llvm/Target/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "C65GenRegisterInfo.inc"

namespace llvm {

class C65Subtarget;
class TargetInstrInfo;
class Type;

struct C65RegisterInfo : public C65GenRegisterInfo {
  C65Subtarget &Subtarget;

  C65RegisterInfo(C65Subtarget &st);

  /// Code Generation virtual methods...
  const MCPhysReg *
  getCalleeSavedRegs(const MachineFunction *MF =nullptr) const override;
  const uint32_t* getCallPreservedMask(CallingConv::ID CC) const override;

  const uint32_t* getRTCallPreservedMask(CallingConv::ID CC) const;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF,
                                                unsigned Kind) const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF,
                                       RegScavenger *RS = nullptr) const;

  // Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const override;
};

} // end namespace llvm

#endif
