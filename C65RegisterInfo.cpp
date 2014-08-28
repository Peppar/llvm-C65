//===-- C65RegisterInfo.cpp - C65 Register Information --------------------===//
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

#include "C65.h"
#include "C65RegisterInfo.h"
#include "C65Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "c65-register-info"

using namespace llvm;

#define GET_REGINFO_TARGET_DESC
#include "C65GenRegisterInfo.inc"

C65RegisterInfo::C65RegisterInfo(C65Subtarget &ST)
  : C65GenRegisterInfo(C65::PC), Subtarget(ST) {}

const MCPhysReg*
C65RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const MCPhysReg CSR_SaveList[] = { 0 };
  return CSR_SaveList;
}


BitVector C65RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  Reserved.set(C65::A);
  Reserved.set(C65::X);
  Reserved.set(C65::Y);
  Reserved.set(C65::S);
  Reserved.set(C65::D);
  Reserved.set(C65::PC);
  Reserved.set(C65::P);
  Reserved.set(C65::PBR);
  Reserved.set(C65::DBR);
  return Reserved;
}

const TargetRegisterClass*
C65RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  // if (Kind == 0)
  //   return &C65::IX16RegClass;
  // else if (Kind == 1)
  //   return &C65::IY16RegClass;
  // else if (Kind == 2)
  //   return &C65::IS16RegClass;
  // else
  // return &C65::ID16RegClass;
  return &C65::ZRC16RegClass;
}

unsigned
C65RegisterInfo::getZRAddress(unsigned RegNo) const {
  switch(RegNo) {
  default: {
    DEBUG(errs() << "Not a zero-page register: " << RegNo
          << C65::ZR0 << " "
          << C65::ZR0D << " "
          << C65::ZR0W << " "
          << C65::ZR0Q);
    llvm_unreachable("Not a zero-page register!");
  }
  case C65::ZR0Q:
  case C65::ZR0D:
  case C65::ZR0W:
  case C65::ZR0: return 0;
  case C65::ZR1: return 1;
  case C65::ZR2W:
  case C65::ZR2: return 2;
  case C65::ZR3: return 3;
  case C65::ZR4D:
  case C65::ZR4W:
  case C65::ZR4: return 4;
  case C65::ZR5: return 5;
  case C65::ZR6W:
  case C65::ZR6: return 6;
  case C65::ZR7: return 7;
  case C65::ZR8Q:
  case C65::ZR8D:
  case C65::ZR8W:
  case C65::ZR8: return 8;
  case C65::ZR9: return 9;
  case C65::ZR10W:
  case C65::ZR10: return 10;
  case C65::ZR11: return 11;
  case C65::ZR12D:
  case C65::ZR12W:
  case C65::ZR12: return 12;
  case C65::ZR13: return 13;
  case C65::ZR14W:
  case C65::ZR14: return 14;
  case C65::ZR15: return 15;
  }
}


void
C65RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                     int SPAdj, unsigned FIOperandNum,
                                     RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected");

  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  //  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned BasePtr = C65::S;

  unsigned Opc = MI.getOpcode();

  MI.getOperand(FIOperandNum).ChangeToRegister(C65::S, false);

  //  int FIOffset = TFI->getFrameIndexOffset(MF, FrameIndex);
  int64_t FIOffset = MF.getFrameInfo()->getObjectOffset(FrameIndex);

  if (MI.getOperand(FIOperandNum + 1).isImm()) {
    // Offset is a 16-bit integer.
    int Offset = FIOffset +
      (int)(MI.getOperand(FIOperandNum + 1).getImm());
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
  } else {
    // Offset is symbolic. This is extremely rare.
    uint64_t Offset = FIOffset +
      (uint64_t)MI.getOperand(FIOperandNum + 1).getOffset();
    MI.getOperand(FIOperandNum + 1).setOffset(Offset);
  }
}

unsigned C65RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return C65::S;
}
