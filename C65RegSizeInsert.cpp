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
// costly.  This pass attempts to the number of such status register
// manipulations.
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

bool RegSizeInsert::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const C65Subtarget &ST = MF.getTarget().getSubtarget<C65Subtarget>();
  const C65InstrInfo *TII = ST.getInstrInfo();

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ) {
      MachineInstr *MI = MBBI++;

      unsigned AccSize = C65II::getAccSize(MI->getDesc().TSFlags);
      unsigned IxSize = C65II::getIxSize(MI->getDesc().TSFlags);

      if (AccSize == C65II::Acc8Bit || IxSize == C65II::Ix8Bit) {
        unsigned StatusCode =
          (AccSize == C65II::Acc8Bit ? 0x20 : 0) +
          (IxSize  == C65II::Ix8Bit  ? 0x10 : 0);

        DebugLoc DL = MI->getDebugLoc();
        BuildMI(*MBB, MBBI, DL, TII->get(C65::SEP))
          .addImm(StatusCode);
        MBBI = BuildMI(*MBB, std::next(MBBI), DL, TII->get(C65::REP))
          .addImm(StatusCode);
        ++MBBI;
      }
    }
  }

  return Changed;
}
