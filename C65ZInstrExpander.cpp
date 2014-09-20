//===-- C65ZInstrExpander.cpp - Zero-page instruction expander ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Expand Z instructions.
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

#define DEBUG_TYPE "c65-z-instr-expander"

namespace {

  class ZInstrExpander : public MachineFunctionPass {
  public:
    static char ID;

    ZInstrExpander() : MachineFunctionPass(ID) {}
    const char *getPassName() const override {
      return "C65 Z instruction expander";
    }

  private:
    bool runOnMachineFunction(MachineFunction &MF) override;

    void getAnalysisUsage(AnalysisUsage &AU) const override {
      MachineFunctionPass::getAnalysisUsage(AU);
    }
  };
  char ZInstrExpander::ID = 0;
}

FunctionPass *llvm::createC65ZInstrExpanderPass() {
  return new ZInstrExpander();
}

/// runOnMachineFunction - Loop over all of the basic blocks, expanding
/// Z instructions
bool ZInstrExpander::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const C65Subtarget &ST = MF.getTarget().getSubtarget<C65Subtarget>();
  const C65TargetLowering *TL = ST.getTargetLowering();

  // Iterate through each instruction in the function, looking for pseudos.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ) {
      MachineInstr *MI = MBBI++;

      // If MI is a Z instruction, expand it.
      if (MI->getDesc().TSFlags & C65II::ZRInstr) {
        Changed = true;
        MachineBasicBlock *NewMBB =
          TL->EmitZInstr(MI, MBB);
        // The expansion may involve new basic blocks.
        if (NewMBB != MBB) {
          MBB = NewMBB;
          I = NewMBB;
          MBBI = NewMBB->begin();
          MBBE = NewMBB->end();
        }
      }
    }
  }

  return Changed;
}
