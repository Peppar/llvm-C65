//===-- C65RegSizeInsert.cpp - Zero-page instruction expander ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// 65c816 allows you to choose between 8 and 16-bit accumulator size,
// and 8 and 16-bit index size. The choice is made by manipulating the
// status register with a separate instruction, and is considered
// costly.  This pass attempts to reduce the number of such status
// register manipulations.
//
//===----------------------------------------------------------------------===//

#include "C65.h"
#include "C65InstrInfo.h"
#include "C65Subtarget.h"
#include "MCTargetDesc/C65BaseInfo.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetInstrInfo.h"
using namespace llvm;

#define DEBUG_TYPE "c65-reg-size-insert"

namespace {

  class RegSizeInsert : public MachineFunctionPass {
  public:
    static char ID;

    RegSizeInsert() : MachineFunctionPass(ID) {}
    const char *getPassName() const override {
      return "C65 Z instruction expander";
    }

  private:
    bool insertDefaultSizes(MachineFunction &MF);

    bool runOnMachineFunction(MachineFunction &MF) override;

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      MachineFunctionPass::getAnalysisUsage(AU);
    }
  };
  char RegSizeInsert::ID = 0;
}

FunctionPass *llvm::createC65RegSizeInsertPass() {
  return new RegSizeInsert();
}

bool RegSizeInsert::insertDefaultSizes(MachineFunction &MF) {
  const C65Subtarget &ST = MF.getTarget().getSubtarget<C65Subtarget>();
  const C65InstrInfo *TII = ST.getInstrInfo();
  MachineBasicBlock *MBB = MF.begin();
  DebugLoc DL; // Empty DebugLoc
  unsigned LONGAOpcode = ST.has65802() ? C65::LONGA_ON : C65::LONGA_OFF;
  unsigned LONGIOpcode = ST.has65802() ? C65::LONGI_ON : C65::LONGI_OFF;
  BuildMI(*MBB, MBB->begin(), DL, TII->get(LONGIOpcode));
  BuildMI(*MBB, MBB->begin(), DL, TII->get(LONGAOpcode));
  return true;
}

bool RegSizeInsert::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const C65Subtarget &ST = MF.getTarget().getSubtarget<C65Subtarget>();
  const C65InstrInfo *TII = ST.getInstrInfo();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned CurAccSize = C65II::Acc16Bit;;
    unsigned CurIxSize = C65II::Ix16Bit;
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ++MBBI) {
      MachineInstr *MI = MBBI;
      unsigned AccSize = 0;
      unsigned IxSize = 0;

      if (MI->getDesc().isBranch() || MI->getDesc().isBarrier()) {
        AccSize = C65II::Acc16Bit;
        IxSize = C65II::Ix16Bit;
      } else {
        AccSize = C65II::getAccSize(MI->getDesc().TSFlags);
        IxSize = C65II::getIxSize(MI->getDesc().TSFlags);
      }

      if ((AccSize && AccSize != CurAccSize) ||
          (IxSize && IxSize != CurIxSize)) {
        unsigned ResetBits =
          ((AccSize != CurAccSize && AccSize == C65II::Acc16Bit) ? 0x20 : 0) |
          ((IxSize != CurIxSize && IxSize  == C65II::Ix16Bit)  ? 0x10 : 0);
        unsigned SetBits =
          ((AccSize != CurAccSize && AccSize == C65II::Acc8Bit) ? 0x20 : 0) |
          ((IxSize != CurIxSize && IxSize  == C65II::Ix8Bit)  ? 0x10 : 0);

        DebugLoc DL = MI->getDebugLoc();
        if (ResetBits) {
          BuildMI(*MBB, MBBI, DL, TII->get(C65::REP))
            .addImm(ResetBits);
        }
        if (SetBits) {
          BuildMI(*MBB, MBBI, DL, TII->get(C65::SEP))
            .addImm(SetBits);
        }
        if (AccSize) CurAccSize = AccSize;
        if (IxSize) CurIxSize = IxSize;
        Changed = true;
      }
    }
    if (CurAccSize != C65II::Acc16Bit || CurIxSize != C65II::Ix16Bit) {
      DebugLoc DL;
      BuildMI(MBB, DL, TII->get(C65::REP))
        .addImm(0x30);
    }
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ) {
      MachineInstr *MI = MBBI++;
      DebugLoc DL = MI->getDebugLoc();
      if (MI->getOpcode() == C65::REP) {
        if (MI->getOperand(0).isImm()) {
          unsigned Reset = MI->getOperand(0).getImm();
          if (Reset & 0x20) {
            BuildMI(*MBB, MBBI, DL, TII->get(C65::LONGA_ON));
            Changed = true;
          }
          if (Reset & 0x10) {
            BuildMI(*MBB, MBBI, DL, TII->get(C65::LONGI_ON));
            Changed = true;
          }
        }
      } else if (MI->getOpcode() == C65::SEP) {
        if (MI->getOperand(0).isImm()) {
          unsigned Set = MI->getOperand(0).getImm();
          if (Set & 0x20) {
            BuildMI(*MBB, MBBI, DL, TII->get(C65::LONGA_OFF));
            Changed = true;
          }
          if (Set & 0x10) {
            BuildMI(*MBB, MBBI, DL, TII->get(C65::LONGI_OFF));
            Changed = true;
          }
        }
      }
    }
  }

  Changed = insertDefaultSizes(MF) || Changed;

  return Changed;
}
