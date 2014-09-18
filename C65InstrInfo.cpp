//===-- C65InstrInfo.cpp - C65 Instruction Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the C65 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "C65InstrInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/ISDOpcodes.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "c65-codegen"

#define GET_INSTRINFO_CTOR_DTOR
#include "C65GenInstrInfo.inc"

// Pin the vtable to this file.
void C65InstrInfo::anchor() {}

C65InstrInfo::C65InstrInfo(C65Subtarget &ST)
  : C65GenInstrInfo(C65::ADJCALLSTACKDOWN, C65::ADJCALLSTACKUP),
    RI(ST), STI(ST) {}

void C65InstrInfo::buildPushReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                DebugLoc DL, unsigned Reg) const {
  // if (Reg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::PHA));
  // } else if (Reg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::PHX));
  // } else if (Reg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::PHY));
  // } else if (Reg == C65::D) {
  //   BuildMI(MBB, MI, DL, get(C65::PHD));
  // } else if (Reg == C65::P) {
  //   BuildMI(MBB, MI, DL, get(C65::PHP));
  // } else {
  //   DEBUG(dbgs() << "Cannot push " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy push");
  // }
}

void C65InstrInfo::buildPullReg(MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI,
                                DebugLoc DL, unsigned Reg) const {
  // if (Reg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::PLA));
  // } else if (Reg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::PLX));
  // } else if (Reg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::PLY));
  // } else if (Reg == C65::D) {
  //   BuildMI(MBB, MI, DL, get(C65::PLD));
  // } else if (Reg == C65::P) {
  //   BuildMI(MBB, MI, DL, get(C65::PLP));
  // } else {
  //   DEBUG(dbgs() << "Cannot pull " << RI.getName(Reg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy pull");
  // }
}

void C65InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MBBI,
                               DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
  const unsigned ByteCapacity = 2;
  unsigned NumBytes;
  unsigned LDAInstr;
  unsigned STAInstr;

  if (C65::ZRC8RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 1;
  } else if (C65::ZRC16RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 2;
  } else if (C65::ZRC32RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 4;
  } else if (C65::ZRC64RegClass.contains(DestReg, SrcReg)) {
    NumBytes = 8;
  } else {
    llvm_unreachable("Impossible reg-to-reg copy");
  }

  if (NumBytes == 1 || ByteCapacity == 1) {
    LDAInstr = C65::LDA_8zp;
    STAInstr = C65::STA_8zp;
  } else {
    LDAInstr = C65::LDA_16zp;
    STAInstr = C65::STA_16zp;
  }

  if (NumBytes == 1) {
    BuildMI(MBB, MBBI, DL, get(C65::SEP)).addImm(0x20);
  }
  for (unsigned I = 0; I < NumBytes; I += ByteCapacity) {
    BuildMI(MBB, MBBI, DL, get(LDAInstr))
      .addImm(RI.getZRAddress(SrcReg) + I);
    BuildMI(MBB, MBBI, DL, get(STAInstr))
      .addImm(RI.getZRAddress(DestReg) + I);
  }
  if (NumBytes == 1) {
    BuildMI(MBB, MBBI, DL, get(C65::REP)).addImm(0x20);
  }

  DEBUG(dbgs() << "CopyPhysReg from "
               << RI.getName(DestReg)
               << " to " << RI.getName(SrcReg) << '\n');
  MBB.dump();
  // if (SrcReg == C65::A && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TAX));
  // } else if (SrcReg == C65::A && DestReg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::TAY));
  // } else if (SrcReg == C65::X && DestReg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::TXA));
  // } else if (SrcReg == C65::Y && DestReg == C65::A) {
  //   BuildMI(MBB, MI, DL, get(C65::TYA));
  // } else if (SrcReg == C65::X && DestReg == C65::Y) {
  //   BuildMI(MBB, MI, DL, get(C65::TXY));
  // } else if (SrcReg == C65::Y && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TYX));
  // } else if (SrcReg == C65::X && DestReg == C65::S) {
  //   BuildMI(MBB, MI, DL, get(C65::TXS));
  // } else if (SrcReg == C65::S && DestReg == C65::X) {
  //   BuildMI(MBB, MI, DL, get(C65::TSX));
  // } else if (SrcReg == C65::P || DestReg == C65::P ||
  //            SrcReg == C65::D || DestReg == C65::D) {
  //   buildPushReg(MBB, MI, DL, DestReg);
  //   buildPullReg(MBB, MI, DL, SrcReg);
  // } else {
  //   DEBUG(dbgs() << "Cannot copy " << RI.getName(SrcReg)
  //                << " to " << RI.getName(DestReg) << '\n');
  //   llvm_unreachable("Impossible reg-to-reg copy");
  // }
}

unsigned C65InstrInfo::isStoreToStackSlot(const MachineInstr *MI,
                                          int &FrameIndex) const {
  // if (MI->getOpcode() == C65::STAis) {
  //   if (MI->getOperand(0).isFI()) {
  //     FrameIndex = MI->getOperand(0).getIndex();
  //     return C65::A;
  //   }
  // }
  return 0;
}

void C65InstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB,
                    MachineBasicBlock::iterator MBBI,
                    unsigned SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  // DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  // if (SrcReg == C65::A) {
  //   BuildMI(MBB, MBBI, DL, get(C65::STAis)).addFrameIndex(FI);
  // } else {
  //   DEBUG(dbgs() << "Cannot store " << RI.getName(SrcReg)
  //                << " to stack frame\n");
  //   llvm_unreachable("Unable to store reg from stack slot.");
  // }
}

unsigned C65InstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                           int &FrameIndex) const {
  // if (MI->getOpcode() == C65::LDAis) {
  //   if (MI->getOperand(0).isFI()) {
  //     FrameIndex = MI->getOperand(0).getIndex();
  //     return C65::A;
  //   }
  // }
  return 0;
}

void C65InstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB,
                     MachineBasicBlock::iterator MBBI,
                     unsigned DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  // DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();
  // if (DestReg == C65::A) {
  //   BuildMI(MBB, MBBI, DL, get(C65::LDAis)).addFrameIndex(FI);
  // } else {
  //   DEBUG(dbgs() << "Cannot load " << RI.getName(DestReg)
  //                << " from stack frame\n");
  //   llvm_unreachable("Unable to load reg from stack slot.");
  // }
}


//static GlobalValue

// static C65AsmPrinter::getZRAddr(MCInstBuilder MIB,
//                                 const MachineOperand &MO,
//                                 unsigned Offset) {
//   const C65RegisterInfo &TRI =
//     *static_cast<const C65RegisterInfo *>
//     (TM.getSubtargetImpl()->getRegisterInfo());
//   unsigned Addr = TRI.getZRAddress(MO.getReg());
//   MIB.addImm(Addr + Offset);
//   return MIB;
// }

// static MachineBasicBlock *getSuccessor(MachineBasicBlock *MBB) {
//   MachineFunction::iterator I(MBB);
//   return std::next(I);
// }

bool C65InstrInfo::expandZRInstr(MachineBasicBlock::iterator MBBI,
                                 unsigned Instruction) const {
  MachineInstrBuilder MIB(*MBBI->getParent()->getParent(), MBBI);
  MachineBasicBlock &MBB = *MIB->getParent();
  DebugLoc DL = MIB->getDebugLoc();
  unsigned ZReg = MIB->getOperand(0).getReg();
  unsigned Offset = MIB->getOperand(1).getImm();
  MachineBasicBlock::iterator I = MIB;

  BuildMI(MBB, I, DL, get(Instruction))
    .addImm(getRegisterInfo().getZRAddress(ZReg) + Offset);

  MBBI->eraseFromParent();

  return true;
}

bool C65InstrInfo::expandPostRAPseudo(MachineBasicBlock::iterator MBBI) const {

  switch (MBBI->getOpcode()) {
  case C65::ORA_8zr8:
  case C65::ORA_8zr16:
  case C65::ORA_8zr32:
  case C65::ORA_8zr64: return expandZRInstr(MBBI, C65::ORA_8zp);
  case C65::ORA_16zr16:
  case C65::ORA_16zr32:
  case C65::ORA_16zr64: return expandZRInstr(MBBI, C65::ORA_16zp);
  case C65::AND_8zr8:
  case C65::AND_8zr16:
  case C65::AND_8zr32:
  case C65::AND_8zr64: return expandZRInstr(MBBI, C65::AND_8zp);
  case C65::AND_16zr16:
  case C65::AND_16zr32:
  case C65::AND_16zr64: return expandZRInstr(MBBI, C65::AND_16zp);
  case C65::EOR_8zr8:
  case C65::EOR_8zr16:
  case C65::EOR_8zr32:
  case C65::EOR_8zr64: return expandZRInstr(MBBI, C65::EOR_8zp);
  case C65::EOR_16zr16:
  case C65::EOR_16zr32:
  case C65::EOR_16zr64: return expandZRInstr(MBBI, C65::EOR_16zp);
  case C65::ADC_8zr8:
  case C65::ADC_8zr16:
  case C65::ADC_8zr32:
  case C65::ADC_8zr64: return expandZRInstr(MBBI, C65::ADC_8zp);
  case C65::ADC_16zr16:
  case C65::ADC_16zr32:
  case C65::ADC_16zr64: return expandZRInstr(MBBI, C65::ADC_16zp);
  case C65::SBC_8zr8:
  case C65::SBC_8zr16:
  case C65::SBC_8zr32:
  case C65::SBC_8zr64: return expandZRInstr(MBBI, C65::SBC_8zp);
  case C65::SBC_16zr16:
  case C65::SBC_16zr32:
  case C65::SBC_16zr64: return expandZRInstr(MBBI, C65::SBC_16zp);
  case C65::STA_8zr8:
  case C65::STA_8zr16:
  case C65::STA_8zr32:
  case C65::STA_8zr64: return expandZRInstr(MBBI, C65::STA_8zp);
  case C65::STA_16zr16:
  case C65::STA_16zr32:
  case C65::STA_16zr64: return expandZRInstr(MBBI, C65::STA_16zp);
  case C65::CMP_8zr8:
  case C65::CMP_8zr16:
  case C65::CMP_8zr32:
  case C65::CMP_8zr64: return expandZRInstr(MBBI, C65::CMP_8zp);
  case C65::CMP_16zr16:
  case C65::CMP_16zr32:
  case C65::CMP_16zr64: return expandZRInstr(MBBI, C65::CMP_16zp);
  case C65::LDA_8zr8:
  case C65::LDA_8zr16:
  case C65::LDA_8zr32:
  case C65::LDA_8zr64: return expandZRInstr(MBBI, C65::LDA_8zp);
  case C65::LDA_16zr16:
  case C65::LDA_16zr32:
  case C65::LDA_16zr64: return expandZRInstr(MBBI, C65::LDA_16zp);
  case C65::ASL_8zr8:
  case C65::ASL_8zr16:
  case C65::ASL_8zr32:
  case C65::ASL_8zr64: return expandZRInstr(MBBI, C65::ASL_8zp);
  case C65::ASL_16zr16:
  case C65::ASL_16zr32:
  case C65::ASL_16zr64: return expandZRInstr(MBBI, C65::ASL_16zp);
  case C65::ROL_8zr8:
  case C65::ROL_8zr16:
  case C65::ROL_8zr32:
  case C65::ROL_8zr64: return expandZRInstr(MBBI, C65::ROL_8zp);
  case C65::ROL_16zr16:
  case C65::ROL_16zr32:
  case C65::ROL_16zr64: return expandZRInstr(MBBI, C65::ROL_16zp);
  case C65::LSR_8zr8:
  case C65::LSR_8zr16:
  case C65::LSR_8zr32:
  case C65::LSR_8zr64: return expandZRInstr(MBBI, C65::LSR_8zp);
  case C65::LSR_16zr16:
  case C65::LSR_16zr32:
  case C65::LSR_16zr64: return expandZRInstr(MBBI, C65::LSR_16zp);
  case C65::ROR_8zr8:
  case C65::ROR_8zr16:
  case C65::ROR_8zr32:
  case C65::ROR_8zr64: return expandZRInstr(MBBI, C65::ROR_8zp);
  case C65::ROR_16zr16:
  case C65::ROR_16zr32:
  case C65::ROR_16zr64: return expandZRInstr(MBBI, C65::ROR_16zp);
  case C65::DEC_8zr8:
  case C65::DEC_8zr16:
  case C65::DEC_8zr32:
  case C65::DEC_8zr64: return expandZRInstr(MBBI, C65::DEC_8zp);
  case C65::DEC_16zr16:
  case C65::DEC_16zr32:
  case C65::DEC_16zr64: return expandZRInstr(MBBI, C65::DEC_16zp);
  case C65::INC_8zr8:
  case C65::INC_8zr16:
  case C65::INC_8zr32:
  case C65::INC_8zr64: return expandZRInstr(MBBI, C65::INC_8zp);
  case C65::INC_16zr16:
  case C65::INC_16zr32:
  case C65::INC_16zr64: return expandZRInstr(MBBI, C65::INC_16zp);
  case C65::STX_8zr8:
  case C65::STX_8zr16:
  case C65::STX_8zr32:
  case C65::STX_8zr64: return expandZRInstr(MBBI, C65::STX_8zp);
  case C65::STX_16zr16:
  case C65::STX_16zr32:
  case C65::STX_16zr64: return expandZRInstr(MBBI, C65::STX_16zp);
  case C65::LDX_8zr8:
  case C65::LDX_8zr16:
  case C65::LDX_8zr32:
  case C65::LDX_8zr64: return expandZRInstr(MBBI, C65::LDX_8zp);
  case C65::LDX_16zr16:
  case C65::LDX_16zr32:
  case C65::LDX_16zr64: return expandZRInstr(MBBI, C65::LDX_16zp);
  case C65::STY_8zr8:
  case C65::STY_8zr16:
  case C65::STY_8zr32:
  case C65::STY_8zr64: return expandZRInstr(MBBI, C65::STY_8zp);
  case C65::STY_16zr16:
  case C65::STY_16zr32:
  case C65::STY_16zr64: return expandZRInstr(MBBI, C65::STY_16zp);
  case C65::LDY_8zr8:
  case C65::LDY_8zr16:
  case C65::LDY_8zr32:
  case C65::LDY_8zr64: return expandZRInstr(MBBI, C65::LDY_8zp);
  case C65::LDY_16zr16:
  case C65::LDY_16zr32:
  case C65::LDY_16zr64: return expandZRInstr(MBBI, C65::LDY_16zp);
  case C65::CPY_8zr8:
  case C65::CPY_8zr16:
  case C65::CPY_8zr32:
  case C65::CPY_8zr64: return expandZRInstr(MBBI, C65::CPY_8zp);
  case C65::CPY_16zr16:
  case C65::CPY_16zr32:
  case C65::CPY_16zr64: return expandZRInstr(MBBI, C65::CPY_16zp);
  case C65::CPX_8zr8:
  case C65::CPX_8zr16:
  case C65::CPX_8zr32:
  case C65::CPX_8zr64: return expandZRInstr(MBBI, C65::CPX_8zp);
  case C65::CPX_16zr16:
  case C65::CPX_16zr32:
  case C65::CPX_16zr64: return expandZRInstr(MBBI, C65::CPX_16zp);
  case C65::STZ_8zr8:
  case C65::STZ_8zr16:
  case C65::STZ_8zr32:
  case C65::STZ_8zr64: return expandZRInstr(MBBI, C65::STZ_8zp);
  case C65::STZ_16zr16:
  case C65::STZ_16zr32:
  case C65::STZ_16zr64: return expandZRInstr(MBBI, C65::STZ_16zp);
  }
  return false;
}
