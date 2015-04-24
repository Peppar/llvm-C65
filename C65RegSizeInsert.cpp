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
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
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

static bool insertDefaultSizes(MachineFunction &MF) {
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65InstrInfo &TII = *STI.getInstrInfo();
  MachineBasicBlock *MBB = MF.begin();
  DebugLoc DL; // Empty DebugLoc
  unsigned LONGAOpcode = STI.has65802() ? C65::LONGA_ON : C65::LONGA_OFF;
  unsigned LONGIOpcode = STI.has65802() ? C65::LONGI_ON : C65::LONGI_OFF;
  BuildMI(*MBB, MBB->begin(), DL, TII.get(LONGIOpcode));
  BuildMI(*MBB, MBB->begin(), DL, TII.get(LONGAOpcode));
  return true;
}

// TODO: Convert outgoing branch dict-list {MBB_Source: [(MBB_Target,
// Status)]} to incoming branch dict-list {MBB_Target: [(MBB_Source,
// Status)]} Then eliminate incoming nodes with null status (0) by
// replacing these with the incoming nodes of the source MBB.
// However, the source MBB must first have any null-autoreferences
// removed.  This indicates a loop and will otherwise hang the
// algorithm.
//

// TODO: Test this.
struct Branch {
  MachineBasicBlock *MBB;
  unsigned Status;
};

static bool operator==(const Branch& LHS, const Branch& RHS) {
  return LHS.MBB == RHS.MBB && LHS.Status == RHS.Status;
}

typedef SmallVector<Branch, 4> BranchVector;
typedef DenseMap<MachineBasicBlock *, BranchVector> BranchMap;
typedef SmallSet<MachineBasicBlock *, 32> MBBSet;
typedef DenseMap<MachineBasicBlock *,
                 std::pair<unsigned, unsigned> > StatusMap;

static unsigned
resolveStatusBit(SmallVectorImpl<Branch> &IncomingBr) {
  auto I = IncomingBr.begin();
  if (I == IncomingBr.end())
    return 0;
  unsigned Status = I->Status;
  for (++I; I != IncomingBr.end(); ++I) {
    if (I->Status != Status)
      return 0;
  }
  return Status;
}

// static void
// nullMBBRemoveAutoreferences(MachineBasicBlock BranchVector &NullMBBBranches) {
//   BranchVector &NullMBBBranches = IncomingBr[NullMBB];
//   NullMBBBranches.erase(NullBranch);
// }

static void
removeNullBranches(BranchMap &IncomingBr, MBBSet &NullBr) {
  for (auto I = NullBr.begin(), E = NullBr.end(); I != E; ++I) {
    MachineBasicBlock *NullMBB = *I;
    Branch NullBranch = { NullMBB, 0 };
    BranchVector &NullMBBBranches = IncomingBr[NullMBB];
    std::remove(NullMBBBranches.begin(), NullMBBBranches.end(), NullBranch);
    // {
    //   auto N = std::find(NullMBBBranches.begin(), NullMBBBranches.end(),
    //                      NullBranch);
    //   if (N != NullMBBBranches.end())
    //     NullMBBBranches.erase(N);
    // }
    // NullMBBBranches.erase(NullBranch);
    for (auto J = IncomingBr.begin(), F = IncomingBr.end();
         J != F; ++J) {
      //      MachineBasicBlock *InMBB = J->first;
      SmallVectorImpl<Branch> &InBranches = J->second;
      auto N = std::find(InBranches.begin(), InBranches.end(), NullBranch);
      if (N != InBranches.end()) {
        InBranches.erase(N);
        InBranches.append(NullMBBBranches.begin(), NullMBBBranches.end());
      }
    }
  }
}

static void
insertBranchInversion(MachineBasicBlock *MBB,
                      SmallVectorImpl<Branch> &OutBr,
                      BranchMap &InBr,
                      MBBSet &NullBr) {
  for (auto I = OutBr.begin(), E = OutBr.end(); I != E; ++I) {
    if (!InBr.count(I->MBB))
      InBr[I->MBB] = BranchVector();
    InBr[I->MBB].push_back({ MBB, I->Status });
    if (!I->Status)
      NullBr.insert(MBB);
  }
}

static void
getOutBranches(MachineBasicBlock *MBB,
               SmallVectorImpl<Branch> &IxBr,
               SmallVectorImpl<Branch> &AccBr) {
  //  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  //  const C65InstrInfo &TII = *STI.getInstrInfo();

  unsigned AccSize = 0;
  unsigned IxSize = 0;

  for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
       MBBI != MBBE; ++MBBI) {
    MachineInstr *MI = MBBI;
    unsigned MIAccSize = 0;
    unsigned MIIxSize = 0;

    // Derive the processor status at the end of this instruction.
    if (MI->getDesc().isCall()) {
      // For now, JSR/JSL must branch with 16/16.
      // This would have to change to enable use of JSRabspreix8.
      assert(C65II::getAccSize(MI->getDesc().TSFlags) != C65II::Acc8Bit
             && C65II::getIxSize(MI->getDesc().TSFlags) != C65II::Ix8Bit);
      MIAccSize = C65II::Acc16Bit;
      MIIxSize = C65II::Ix16Bit;
    } else if (MI->getOpcode() == C65::REP) {
      // Explicit status bit reset
      unsigned Reset = MI->getOperand(0).getImm();
      if (Reset & 0x20) MIAccSize = C65II::Acc16Bit;
      if (Reset & 0x10) MIIxSize = C65II::Ix16Bit;
    } else if (MI->getOpcode() == C65::SEP) {
      // Explicit status bit set
      unsigned Set = MI->getOperand(0).getImm();
      if (Set & 0x20) MIAccSize = C65II::Acc8Bit;
      if (Set & 0x10) MIIxSize = C65II::Ix8Bit;
    } else {
      MIAccSize = C65II::getAccSize(MI->getDesc().TSFlags);
      MIIxSize = C65II::getIxSize(MI->getDesc().TSFlags);
    }

    // Update the status bits only if they were explicitly changed.
    if (MIAccSize) AccSize = MIAccSize;
    if (MIIxSize) IxSize = MIIxSize;

    // Treat branching instructions
    switch (MI->getOpcode()) {
    default:
      break;
    case C65::RTS:
    case C65::RTI:
      // No instructions are executed after these instructions.
      return;
    case C65::BRA:
    case C65::BRL:
    case C65::JMPabs:
    case C65::JMPabsind:
    case C65::JMPabspreix8:
    case C65::JMPabspreix16:
    case C65::JMLabsindl:
    case C65::JMLabsl:
      AccBr.push_back({ MI->getOperand(0).getMBB(), AccSize });
      IxBr.push_back({ MI->getOperand(0).getMBB(), IxSize });
      // No instructions are accounted for after an unconditional
      // jump.
      return;
    case C65::BPL:
    case C65::BMI:
    case C65::BVC:
    case C65::BVS:
    case C65::BCC:
    case C65::BCS:
    case C65::BNE:
    case C65::BEQ:
      AccBr.push_back({ MI->getOperand(0).getMBB(), AccSize });
      IxBr.push_back({ MI->getOperand(0).getMBB(), IxSize });
      break;
    }
  }

  // Fall-through to the next MBB.
  MachineBasicBlock *NMBB = std::next(MachineFunction::iterator(MBB));
  AccBr.push_back({ NMBB, AccSize });
  IxBr.push_back({ NMBB, IxSize });
}

// For each MBB, this function derives the accumulator and index word
// size status bits; if they can be assumed to have a certain value or
// if their values differ depending on the incoming code path.
static void
computeInStatus(MachineFunction &MF, StatusMap &InStatus) {
  //  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  //  const C65InstrInfo &TII = *STI.getInstrInfo();

  BranchMap InBrIx;
  BranchMap InBrAcc;
  MBBSet NullBrIx;
  MBBSet NullBrAcc;

  // Derive incoming branches for all MBB's.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    SmallVector<Branch, 4> OutBrIx;
    SmallVector<Branch, 4> OutBrAcc;
    getOutBranches(MBB, OutBrIx, OutBrAcc);
    insertBranchInversion(MBB, OutBrIx, InBrIx, NullBrIx);
    insertBranchInversion(MBB, OutBrAcc, InBrAcc, NullBrAcc);
  }

  // Functions are assumed to begin with 16/16 Ix/Acc
  InBrIx[MF.begin()].push_back({ nullptr, C65II::Ix16Bit });
  InBrAcc[MF.begin()].push_back({ nullptr, C65II::Acc16Bit });

  // Remove all null branches.
  removeNullBranches(InBrIx, NullBrIx);
  removeNullBranches(InBrAcc, NullBrAcc);

  // Calculte the status bits.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned IxStatus = resolveStatusBit(InBrIx[MBB]);
    unsigned AccStatus = resolveStatusBit(InBrAcc[MBB]);
    InStatus[MBB] = { IxStatus, AccStatus };
  }
}

bool RegSizeInsert::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65InstrInfo &TII = *STI.getInstrInfo();
  StatusMap InStatus;

  computeInStatus(MF, InStatus);

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = I;
    unsigned CurIxSize = InStatus[MBB].first;
    unsigned CurAccSize = InStatus[MBB].second;
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ++MBBI) {
      MachineInstr *MI = MBBI;
      unsigned AccSize = 0;
      unsigned IxSize = 0;

      if (MI->getOpcode() == C65::REP) {
        // Explicit status bit reset
        unsigned Reset = MI->getOperand(0).getImm();
        if (Reset & 0x20) CurAccSize = C65II::Acc16Bit;
        if (Reset & 0x10) CurIxSize = C65II::Ix16Bit;
        continue;
      } else if (MI->getOpcode() == C65::SEP) {
        // Explicit status bit set
        unsigned Set = MI->getOperand(0).getImm();
        if (Set & 0x20) CurAccSize = C65II::Acc8Bit;
        if (Set & 0x10) CurIxSize = C65II::Ix8Bit;
        continue;
      }

      // Derive the processor status at the end of this instruction.
      if (MI->getDesc().isCall() || MI->getDesc().isReturn()) {
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
          BuildMI(*MBB, MBBI, DL, TII.get(C65::REP))
            .addImm(ResetBits);
        }
        if (SetBits) {
          BuildMI(*MBB, MBBI, DL, TII.get(C65::SEP))
            .addImm(SetBits);
        }
        if (AccSize) CurAccSize = AccSize;
        if (IxSize) CurIxSize = IxSize;
        Changed = true;
      }
    }

    // Second pass: insert LONGA/LONGI _ ON/OFF declarations.
    for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
         MBBI != MBBE; ) {
      MachineInstr *MI = MBBI++;
      DebugLoc DL = MI->getDebugLoc();
      if (MI->getOpcode() == C65::REP) {
        if (MI->getOperand(0).isImm()) {
          unsigned Reset = MI->getOperand(0).getImm();
          if (Reset & 0x20) {
            BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGA_ON));
            Changed = true;
          }
          if (Reset & 0x10) {
            BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGI_ON));
            Changed = true;
          }
        }
      } else if (MI->getOpcode() == C65::SEP) {
        if (MI->getOperand(0).isImm()) {
          unsigned Set = MI->getOperand(0).getImm();
          if (Set & 0x20) {
            BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGA_OFF));
            Changed = true;
          }
          if (Set & 0x10) {
            BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGI_OFF));
            Changed = true;
          }
        }
      }
    }
  }

  // LONGI / LONGA insertion for the top of the MF.
  Changed = insertDefaultSizes(MF) || Changed;

  return Changed;
}

// bool RegSizeInsert::runOnMachineFunction(MachineFunction &MF) {
//   bool Changed = false;
//   const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
//   const C65InstrInfo &TII = *STI.getInstrInfo();

//   for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
//     MachineBasicBlock *MBB = I;
//     unsigned CurAccSize = C65II::Acc16Bit;
//     unsigned CurIxSize = C65II::Ix16Bit;
//     for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
//          MBBI != MBBE; ++MBBI) {
//       MachineInstr *MI = MBBI;
//       unsigned AccSize = 0;
//       unsigned IxSize = 0;

//       if (MI->getDesc().isBranch() || MI->getDesc().isBarrier() ||
//           MI->getDesc().isCall()) {
//         AccSize = C65II::Acc16Bit;
//         IxSize = C65II::Ix16Bit;
//       } else {
//         AccSize = C65II::getAccSize(MI->getDesc().TSFlags);
//         IxSize = C65II::getIxSize(MI->getDesc().TSFlags);
//       }

//       if ((AccSize && AccSize != CurAccSize) ||
//           (IxSize && IxSize != CurIxSize)) {
//         unsigned ResetBits =
//           ((AccSize != CurAccSize && AccSize == C65II::Acc16Bit) ? 0x20 : 0) |
//           ((IxSize != CurIxSize && IxSize  == C65II::Ix16Bit)  ? 0x10 : 0);
//         unsigned SetBits =
//           ((AccSize != CurAccSize && AccSize == C65II::Acc8Bit) ? 0x20 : 0) |
//           ((IxSize != CurIxSize && IxSize  == C65II::Ix8Bit)  ? 0x10 : 0);

//         DebugLoc DL = MI->getDebugLoc();
//         if (ResetBits) {
//           BuildMI(*MBB, MBBI, DL, TII.get(C65::REP))
//             .addImm(ResetBits);
//         }
//         if (SetBits) {
//           BuildMI(*MBB, MBBI, DL, TII.get(C65::SEP))
//             .addImm(SetBits);
//         }
//         if (AccSize) CurAccSize = AccSize;
//         if (IxSize) CurIxSize = IxSize;
//         Changed = true;
//       }
//     }
//     if (CurAccSize != C65II::Acc16Bit || CurIxSize != C65II::Ix16Bit) {
//       DebugLoc DL;
//       unsigned ResetBits =
//         ((CurAccSize != C65II::Acc16Bit) ? 0x20 : 0) |
//         ((CurIxSize != C65II::Ix16Bit)  ? 0x10 : 0);
//       BuildMI(MBB, DL, TII.get(C65::REP))
//         .addImm(ResetBits);
//     }
//     for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
//          MBBI != MBBE; ) {
//       MachineInstr *MI = MBBI++;
//       DebugLoc DL = MI->getDebugLoc();
//       if (MI->getOpcode() == C65::REP) {
//         if (MI->getOperand(0).isImm()) {
//           unsigned Reset = MI->getOperand(0).getImm();
//           if (Reset & 0x20) {
//             BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGA_ON));
//             Changed = true;
//           }
//           if (Reset & 0x10) {
//             BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGI_ON));
//             Changed = true;
//           }
//         }
//       } else if (MI->getOpcode() == C65::SEP) {
//         if (MI->getOperand(0).isImm()) {
//           unsigned Set = MI->getOperand(0).getImm();
//           if (Set & 0x20) {
//             BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGA_OFF));
//             Changed = true;
//           }
//           if (Set & 0x10) {
//             BuildMI(*MBB, MBBI, DL, TII.get(C65::LONGI_OFF));
//             Changed = true;
//           }
//         }
//       }
//     }
//   }

//   Changed = insertDefaultSizes(MF) || Changed;

//   return Changed;
// }
