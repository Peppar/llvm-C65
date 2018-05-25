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

#ifndef LLVM_TARGET_C65REGISTERINFO_H
#define LLVM_TARGET_C65REGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "C65GenRegisterInfo.inc"

namespace llvm {

class C65Subtarget;
class TargetInstrInfo;
class Type;

struct C65RegisterInfo : public C65GenRegisterInfo {
  C65Subtarget &Subtarget;

private:

  /// StackPtr - Physical register used as stack ptr.
  ///
  //  unsigned StackPtr;

  /// FramePtr - Physical register used as frame ptr.
  ///
  //  unsigned FramePtr;

public:
  C65RegisterInfo(C65Subtarget &st);

  /// getCalleeSavedRegs - Return a null-terminated list of all of the
  /// callee-save registers on this target.
  const MCPhysReg *
  getCalleeSavedRegs(const MachineFunction *MF) const override;

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;

  const TargetRegisterClass *getPointerRegClass(const MachineFunction &MF,
                                                unsigned Kind) const override;

  bool isZReg(unsigned RegNo) const;

  unsigned getZRSize(unsigned RegNo) const;

  unsigned getZRAddress(unsigned RegNo) const;

  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  unsigned getFrameRegister(const MachineFunction &MF) const override;
  //  unsigned getStackRegister() const { return StackPtr; }
};

} // end namespace llvm

#endif
