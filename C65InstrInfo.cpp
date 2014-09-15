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
                               MachineBasicBlock::iterator MI, DebugLoc DL,
                               unsigned DestReg, unsigned SrcReg,
                               bool KillSrc) const {
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

struct Comparison {
  // The operands to the comparison.
  const MachineOperand Op0, Op1;

  // The opcode that should be used to compare Op0 and Op1.
  bool Equality;

  // Is signed comparison
  bool Signed;

  // The result on which we will trigger a branch:
  //   Branch on bitresult == Z flag for equality
  //   Branch on bitresult == N flag for signed
  //   Branch on bitresult == C flag for unsigned
  bool Bitvalue;
};

///
///
static struct Comparison getComparison(const MachineInstr *MI) {
  ISD::CondCode CC = (ISD::CondCode)MI->getOperand(0).getImm();
  const MachineOperand Op0 = MI->getOperand(1);
  const MachineOperand Op1 = MI->getOperand(2);

  switch (CC) { //           Op0  Op1  Equality Signed Bitvalue
  case ISD::SETEQ:  return { Op0, Op1, true,    false, true };
  case ISD::SETNE:  return { Op0, Op1, true,    false, false };
  case ISD::SETLT:  return { Op1, Op0, false,   true,  false };
  case ISD::SETLE:  return { Op0, Op1, false,   true,  true };
  case ISD::SETGT:  return { Op0, Op1, false,   true,  false };
  case ISD::SETGE:  return { Op1, Op0, false,   true,  true };
  case ISD::SETULT: return { Op0, Op1, false,   false, true };
  case ISD::SETULE: return { Op1, Op0, false,   false, false };
  case ISD::SETUGT: return { Op1, Op0, false,   false, true };
  case ISD::SETUGE: return { Op0, Op1, false,   false, false };
  default:
    llvm_unreachable("Cannot emit this type of comparison!");
  }
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

static bool ExpandBR_CC(MachineInstrBuilder &MIB,
                        const C65InstrInfo &TII,
                        const C65RegisterInfo &RI,
                        unsigned NumBytes) {
  MachineBasicBlock &MBB = *MIB->getParent();
  DebugLoc DL = MIB->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MIB;

  const unsigned ByteCapacity = 2;
  struct Comparison C = getComparison(MIB);
  const MachineOperand &Dest = MIB->getOperand(3);

  if (NumBytes == 1) {
    BuildMI(MBB, MBBI, DL, TII.get(C65::SEP)).addImm(0x20);
  }
  if (C.Equality) {
    for (unsigned I = 0; I < NumBytes; I += ByteCapacity) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::LDAzp))
        .addImm(RI.getZRAddress(C.Op0.getReg()));
      BuildMI(MBB, MBBI, DL, TII.get(C65::CMPzp))
        .addImm(RI.getZRAddress(C.Op1.getReg()));
      //      EmitInstructionFinal(MCInstBuilder(C65::BNE).addExpr(Dest));
    }
    // FIXME
    if (NumBytes == 1) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::REP)).addImm(0x20);
    }
    if (C.Bitvalue) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::BNE)).addOperand(Dest);
    } else {
      BuildMI(MBB, MBBI, DL, TII.get(C65::BEQ)).addOperand(Dest);
    }
  } else if (C.Signed) {
    BuildMI(MBB, MBBI, DL, TII.get(C65::LDAzp))
      .addImm(RI.getZRAddress(C.Op0.getReg()));
    BuildMI(MBB, MBBI, DL, TII.get(C65::CMPzp))
      .addImm(RI.getZRAddress(C.Op1.getReg()));

    for (unsigned I = ByteCapacity; I < NumBytes; I += ByteCapacity) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::LDAzp))
        .addImm(RI.getZRAddress(C.Op0.getReg()) + I);
      BuildMI(MBB, MBBI, DL, TII.get(C65::SBCzp))
        .addImm(RI.getZRAddress(C.Op1.getReg()) + I);
    }

    // FIXME
    //    EmitInstructionFinal(MCInstBuilder(C65::BVC).addImm(6));

    if (NumBytes == 1 || ByteCapacity == 1) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::EOR_8imm)).addImm(0x80);
    } else {
      BuildMI(MBB, MBBI, DL, TII.get(C65::EOR_16imm)).addImm(0x8000);
    }
    if (NumBytes == 1) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::REP)).addImm(0x20);
    }
    if (C.Bitvalue) {
      //      EmitInstructionFinal(MCInstBuilder(C65::BMI).addExpr(Dest));
    } else {
      //      EmitInstructionFinal(MCInstBuilder(C65::BPL).addExpr(Dest));
    }
  } else {
    BuildMI(MBB, MBBI, DL, TII.get(C65::LDAzp))
      .addImm(RI.getZRAddress(C.Op0.getReg()));
    BuildMI(MBB, MBBI, DL, TII.get(C65::CMPzp))
      .addImm(RI.getZRAddress(C.Op1.getReg()));

    for (unsigned I = ByteCapacity; I < NumBytes; I += ByteCapacity) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::LDAzp))
        .addImm(RI.getZRAddress(C.Op0.getReg()) + I);
      BuildMI(MBB, MBBI, DL, TII.get(C65::SBCzp))
        .addImm(RI.getZRAddress(C.Op1.getReg()) + I);
    }
    if (NumBytes == 1) {
      BuildMI(MBB, MBBI, DL, TII.get(C65::REP)).addImm(0x20);
    }
    if (C.Bitvalue) {
      //      EmitInstructionFinal(MCInstBuilder(C65::BCS).addExpr(Dest));
    } else {
      //      EmitInstructionFinal(MCInstBuilder(C65::BCC).addExpr(Dest));
    }
  }
}

bool C65InstrInfo::expandPostRAPseudo(MachineBasicBlock::iterator MI) const {
  MachineInstrBuilder MIB(*MI->getParent()->getParent(), MI);

  switch (MI->getOpcode()) {
  case C65::BRCC8zz:
    return ExpandBR_CC(MIB, *this, RI, 1);
  case C65::BRCC16zz:
    return ExpandBR_CC(MIB, *this, RI, 2);
  case C65::BRCC32zz:
    return ExpandBR_CC(MIB, *this, RI, 4);
  case C65::BRCC64zz:
    return ExpandBR_CC(MIB, *this, RI, 8);
  }
}
