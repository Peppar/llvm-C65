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
#include <list>
#include <map>
#include <set>
using namespace llvm;

#define DEBUG_TYPE "c65-reg-size-insert"

namespace {

  class RegSizeInsert : public MachineFunctionPass {
  public:
    static char ID;

    RegSizeInsert() : MachineFunctionPass(ID) {}
    StringRef getPassName() const override {
      return "C65 Register size insert";
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
  MachineBasicBlock &MBB = *MF.begin();
  DebugLoc DL; // Empty DebugLoc
  unsigned LONGAOpcode = STI.has65802() ? C65::LONGA_ON : C65::LONGA_OFF;
  unsigned LONGIOpcode = STI.has65802() ? C65::LONGI_ON : C65::LONGI_OFF;
  BuildMI(MBB, MBB.begin(), DL, TII.get(LONGIOpcode));
  BuildMI(MBB, MBB.begin(), DL, TII.get(LONGAOpcode));
  return true;
}

// This structure represents either an outgoing branch or an incoming
// branch, with the CPU having the specified status when branching. A
// status of 0 signifies a null branch.
struct Branch {
  MachineBasicBlock *MBB;
  unsigned Status;
};
static bool operator<(const Branch& LHS, const Branch& RHS) {
  std::less<MachineBasicBlock *> orderMBBPtr;
  if (LHS.MBB == RHS.MBB)
    return LHS.Status < RHS.Status;
  else
    return orderMBBPtr(LHS.MBB, RHS.MBB);
}

typedef std::set<Branch> BranchSet;
typedef std::map<MachineBasicBlock *, BranchSet> BranchMap;
typedef std::set<MachineBasicBlock *> MBBSet;
typedef std::map<MachineBasicBlock *,
                 std::pair<unsigned, unsigned> > StatusMap;

#if !defined(NDEBUG)
std::string getMBBName(MachineBasicBlock *MBB) {
  return MBB ? ("BB" + Twine(MBB->getNumber())).str() : "(call)";
}
static void dumpBranches(BranchSet &Branches) {
  bool First = true;
  for (auto I = Branches.begin(); I != Branches.end(); ++I) {
    if (First)
      First = false;
    else
      dbgs() << ',';
    dbgs() << getMBBName(I->MBB) << ':' << I->Status;
  }
}
#endif

static unsigned
resolveStatusBit(BranchSet &IncomingBr) {
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

static void
eliminateNullBranches(BranchMap &IncomingBr, MBBSet &NullBr) {
  for (auto I = NullBr.begin(), E = NullBr.end(); I != E; ++I) {
    MachineBasicBlock *NullMBB = *I;
    Branch NullBranch = { NullMBB, 0 };
    BranchSet &NullMBBBranches = IncomingBr[NullMBB];
    // Remove null self-references.
    NullMBBBranches.erase(NullBranch);
    for (auto J = IncomingBr.begin(), F = IncomingBr.end();
         J != F; ++J) {
      BranchSet &InBranches = J->second;
      if (InBranches.erase(NullBranch))
        InBranches.insert(NullMBBBranches.begin(), NullMBBBranches.end());
      assert(!InBranches.count(NullBranch));
    }
  }
#if !defined(NDEBUG)
  // Make sure that all null branches were eliminated.
  for (auto I = IncomingBr.begin(), E = IncomingBr.end(); I != E; ++I) {
    BranchSet &InBranches = I->second;
    for (auto J = InBranches.begin(), F = InBranches.end(); J != F; ++J)
      assert (J->Status);
  }
#endif
}

static void
getOutBranches(MachineBasicBlock *MBB,
               BranchSet &IxBr,
               BranchSet &AccBr,
               StatusMap &FallthroughStatus) {
  unsigned AccSize = 0;
  unsigned IxSize = 0;
  bool HasExited = false;

  for (MachineBasicBlock::iterator MBBI = MBB->begin(), MBBE = MBB->end();
       MBBI != MBBE; ++MBBI) {
    MachineInstr &MI = *MBBI;
    unsigned MIAccSize = 0;
    unsigned MIIxSize = 0;

    // Derive the processor status at the end of this instruction.
    if (MI.getDesc().isCall()) {
      // For now, JSR/JSL branches and returns with 16/16.
      // This would have to change to enable use of JSRabspreix8.
      assert(C65II::getAccSize(MI.getDesc().TSFlags) != C65II::Acc8Bit
             && C65II::getIxSize(MI.getDesc().TSFlags) != C65II::Ix8Bit);
      MIAccSize = C65II::Acc16Bit;
      MIIxSize = C65II::Ix16Bit;
    } else if (MI.getOpcode() == C65::REP) {
      // Explicit status bit reset
      unsigned Reset = MI.getOperand(0).getImm();
      if (Reset & 0x20) MIAccSize = C65II::Acc16Bit;
      if (Reset & 0x10) MIIxSize = C65II::Ix16Bit;
    } else if (MI.getOpcode() == C65::SEP) {
      // Explicit status bit set
      unsigned Set = MI.getOperand(0).getImm();
      if (Set & 0x20) MIAccSize = C65II::Acc8Bit;
      if (Set & 0x10) MIIxSize = C65II::Ix8Bit;
    } else {
      MIAccSize = C65II::getAccSize(MI.getDesc().TSFlags);
      MIIxSize = C65II::getIxSize(MI.getDesc().TSFlags);
    }

    // Update the status bits only if they were explicitly changed.
    if (MIAccSize) AccSize = MIAccSize;
    if (MIIxSize) IxSize = MIIxSize;

    // Treat branching instructions
    switch (MI.getOpcode()) {
    default:
      break;
    case C65::RTS:
    case C65::RTI:
      // No instructions are executed after returns.
      HasExited = true;
      break;
    case C65::BRA:
    case C65::BRL:
    case C65::JMPabs:
    case C65::JMPabsind:
    case C65::JMPabspreix8:
    case C65::JMPabspreix16:
    case C65::JMLabsindl:
    case C65::JMLabsl:
      if (!HasExited) {
        AccBr.insert({ MI.getOperand(0).getMBB(), AccSize });
        IxBr.insert({ MI.getOperand(0).getMBB(), IxSize });
        // No instructions are accounted for after an unconditional
        // jump.
        HasExited = true;
      }
      break;
    case C65::BPL:
    case C65::BMI:
    case C65::BVC:
    case C65::BVS:
    case C65::BCC:
    case C65::BCS:
    case C65::BNE:
    case C65::BEQ:
      if (!HasExited) {
        AccBr.insert({ MI.getOperand(0).getMBB(), AccSize });
        IxBr.insert({ MI.getOperand(0).getMBB(), IxSize });
      }
      break;
    }
  }

  MachineBasicBlock *NMBB = &(*std::next(MachineFunction::iterator(MBB)));
  if (!HasExited) {
    // Fall-through to the next MBB.
    AccBr.insert({ NMBB, AccSize });
    IxBr.insert({ NMBB, IxSize });
  }
  FallthroughStatus[NMBB] = std::make_pair(IxSize, AccSize);
}

// Convert outgoing branches to incoming branches, and mark MBB's with
// outgoing null branches as the same time.
static void
insertBranchInversion(MachineBasicBlock *MBB,
                      BranchSet &OutBr,
                      BranchMap &InBr,
                      MBBSet &NullBr) {
  for (auto I = OutBr.begin(), E = OutBr.end(); I != E; ++I) {
    if (!InBr.count(I->MBB))
      InBr[I->MBB] = BranchSet();
    InBr[I->MBB].insert({ MBB, I->Status });
    if (!I->Status)
      NullBr.insert(MBB);
  }
}

// When the fallthrough statuses are 0, we need to fill it out with
// the statuses of its predecessors.
static void
fillFallthroughStatus(MachineFunction &MF, StatusMap &FallthroughStatus) {
  unsigned CurStatusIx = C65II::Ix16Bit;
  unsigned CurStatusAcc = C65II::Acc16Bit;
  for (auto I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &(*I);
    if (FallthroughStatus.count(MBB)) {
      unsigned StatusIx = FallthroughStatus[MBB].first;
      unsigned StatusAcc = FallthroughStatus[MBB].second;
      if (StatusIx)
        CurStatusIx = StatusIx;
      if (StatusAcc)
        CurStatusAcc = StatusAcc;
    }
    FallthroughStatus[MBB] =
      std::make_pair(CurStatusIx, CurStatusAcc);
  }
}

// This function derives three things from any given MachineFunction:
//
// InBrIx/InBrAcc
//   For each MBB: all known incoming code paths with their respective
//   machine word sizes for index and accumulator.
//
// FallthroughStatus
//   For each MBB: machine word size at the end of the predecessor MBB.
//
// NullBrIx/NullBrAcc
//   The list of MBBs that has outgoing null branches
//   for index and accumulator.
static void
getInBranches(MachineFunction &MF,
              BranchMap &InBrIx, BranchMap &InBrAcc,
              MBBSet &NullBrIx, MBBSet &NullBrAcc,
              StatusMap &FallthroughStatus) {
  // Derive incoming branches for all MBB's.
  for (auto I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &(*I);
    BranchSet OutBrIx;
    BranchSet OutBrAcc;
    getOutBranches(MBB, OutBrIx, OutBrAcc, FallthroughStatus);

    LLVM_DEBUG(dbgs() << getMBBName(MBB) << "(Ix)->";
               dumpBranches(OutBrIx);
               dbgs() << '\n');
    LLVM_DEBUG(dbgs() << getMBBName(MBB) << "(Acc)->";
               dumpBranches(OutBrAcc);
               dbgs() << '\n');

    insertBranchInversion(MBB, OutBrIx, InBrIx, NullBrIx);
    insertBranchInversion(MBB, OutBrAcc, InBrAcc, NullBrAcc);
  }
  MachineBasicBlock *FirstMBB = &(*MF.begin());
  // The first MBB in the MachineFunction is assumed to enter with
  // 16/16 Ix/Acc.
  InBrIx[FirstMBB].insert({ nullptr, C65II::Ix16Bit });
  InBrAcc[FirstMBB].insert({ nullptr, C65II::Acc16Bit });
  fillFallthroughStatus(MF, FallthroughStatus);
}

// This function derives two things from any given MachineFunction:
//
// InStatus
//   For each MBB: the known index/accumulator register sizes at the
//   entry of the MBB. A status of 0 means that the specified register
//   size cannot be determined for some reason, and must be assumed to
//   be undefined.
//
// FallthroughStatus
//   For each MBB: machine word size at the end of the predecessor MBB.
static void
computeInStatus(MachineFunction &MF, StatusMap &InStatus,
                StatusMap &FallthroughStatus) {
  BranchMap InBrIx;
  BranchMap InBrAcc;
  MBBSet NullBrIx;
  MBBSet NullBrAcc;

  LLVM_DEBUG(
    dbgs() << "=== RegSizeInsert computeInStatus for " 
           << MF.getName() << " ===\n");

  getInBranches(MF, InBrIx, InBrAcc, NullBrIx, NullBrAcc,
                FallthroughStatus);

  LLVM_DEBUG(
    dbgs() << "Incoming branches:\n";
    for (auto I = MF.begin(); I != MF.end(); ++I) {
      MachineBasicBlock *MBB = &(*I);
      dbgs() << getMBBName(MBB) << "(Ix)<-";
      dumpBranches(InBrIx[MBB]);
      dbgs() << '\n';
      dbgs() << getMBBName(MBB) << "(Acc)<-";
      dumpBranches(InBrAcc[MBB]);
      dbgs() << '\n';
    });

  // Eliminate all null branches.
  eliminateNullBranches(InBrIx, NullBrIx);
  eliminateNullBranches(InBrAcc, NullBrAcc);

  LLVM_DEBUG(
    dbgs() << "Incoming branches after null elimination:\n";
    for (auto I = MF.begin(); I != MF.end(); ++I) {
      MachineBasicBlock *MBB = &(*I);
      dbgs() << getMBBName(MBB) << "(Ix)<-";
      dumpBranches(InBrIx[MBB]);
      dbgs() << '\n';
      dbgs() << getMBBName(MBB) << "(Acc)<-";
      dumpBranches(InBrAcc[MBB]);
      dbgs() << '\n';
    });

  // Calculte the status bits.
  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &(*I);
    unsigned IxStatus = resolveStatusBit(InBrIx[MBB]);
    unsigned AccStatus = resolveStatusBit(InBrAcc[MBB]);
    InStatus[MBB] = { IxStatus, AccStatus };
  }

  LLVM_DEBUG(
    dbgs() << "Status resolution:\n";
    for (auto I = MF.begin(); I != MF.end(); ++I) {
      MachineBasicBlock *MBB = &(*I);
      dbgs() << getMBBName(MBB) << " Ix:"
             << InStatus[MBB].first << '\n';
      dbgs() << getMBBName(MBB) << " Acc:"
             << InStatus[MBB].second << '\n';
    });
}

bool RegSizeInsert::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const C65Subtarget &STI = MF.getSubtarget<C65Subtarget>();
  const C65InstrInfo &TII = *STI.getInstrInfo();
  StatusMap InStatus;
  StatusMap FallthroughStatus;

  computeInStatus(MF, InStatus, FallthroughStatus);

  for (MachineFunction::iterator I = MF.begin(), E = MF.end(); I != E; ++I) {
    MachineBasicBlock *MBB = &(*I);
    unsigned InIxSize = InStatus[MBB].first;
    unsigned InAccSize = InStatus[MBB].second;
    unsigned CurIxSize = InIxSize;
    unsigned CurAccSize = InAccSize;

    // First pass: Insert REP/SEP when the machine word size changes.
    for (auto MBBI = MBB->begin(), MBBE = MBB->end(); MBBI != MBBE; ++MBBI) {
      MachineInstr &MI = *MBBI;
      unsigned AccSize = 0;
      unsigned IxSize = 0;

      // Account for already existing REP/SEP instructions.
      if (MI.getOpcode() == C65::REP) {
        // Explicit status bit reset
        unsigned Reset = MI.getOperand(0).getImm();
        if (Reset & 0x20) CurAccSize = C65II::Acc16Bit;
        if (Reset & 0x10) CurIxSize = C65II::Ix16Bit;
        continue;
      } else if (MI.getOpcode() == C65::SEP) {
        // Explicit status bit set
        unsigned Set = MI.getOperand(0).getImm();
        if (Set & 0x20) CurAccSize = C65II::Acc8Bit;
        if (Set & 0x10) CurIxSize = C65II::Ix8Bit;
        continue;
      }

      // Derive the machine word sizes at the end of this instruction.
      if (MI.getDesc().isCall() || MI.getDesc().isReturn()) {
        AccSize = C65II::Acc16Bit;
        IxSize = C65II::Ix16Bit;
      } else {
        AccSize = C65II::getAccSize(MI.getDesc().TSFlags);
        IxSize = C65II::getIxSize(MI.getDesc().TSFlags);
      }

      // Insert REP/SEP when the expected machine word sizes does not
      // match that of incoming code paths.
      if ((AccSize && AccSize != CurAccSize) ||
          (IxSize && IxSize != CurIxSize)) {
        unsigned ResetBits =
          ((AccSize != CurAccSize && AccSize == C65II::Acc16Bit) ? 0x20 : 0) |
          ((IxSize != CurIxSize && IxSize  == C65II::Ix16Bit)  ? 0x10 : 0);
        unsigned SetBits =
          ((AccSize != CurAccSize && AccSize == C65II::Acc8Bit) ? 0x20 : 0) |
          ((IxSize != CurIxSize && IxSize  == C65II::Ix8Bit)  ? 0x10 : 0);

        DebugLoc DL = MI.getDebugLoc();
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

    // Second pass: insert LONGA/LONGI declarations.

    // If the status as seen by an assembler differs to the status
    // from incoming branches then we need to insert LONGI/LONGA to
    // make sure that the code can be assembled correctly.
    if (InIxSize != FallthroughStatus[MBB].first) {
      if (InIxSize == C65II::Ix8Bit) {
        BuildMI(*MBB, MBB->begin(), DebugLoc(), TII.get(C65::LONGI_OFF));
        Changed = true;
      } else if (InIxSize == C65II::Ix16Bit) {
        BuildMI(*MBB, MBB->begin(), DebugLoc(), TII.get(C65::LONGI_ON));
        Changed = true;
      }
    }
    if (InAccSize != FallthroughStatus[MBB].second) {
      if (InAccSize == C65II::Acc8Bit) {
        BuildMI(*MBB, MBB->begin(), DebugLoc(), TII.get(C65::LONGA_OFF));
        Changed = true;
      } else if (InAccSize == C65II::Acc16Bit) {
        BuildMI(*MBB, MBB->begin(), DebugLoc(), TII.get(C65::LONGA_ON));
        Changed = true;
      }
    }
    // Insert LONGI/LONGA after REP/SEP instructions.
    for (auto MBBI = MBB->begin(), MBBE = MBB->end(); MBBI != MBBE; ++MBBI) {
      MachineInstr &MI = *MBBI;
      DebugLoc DL = MI.getDebugLoc();
      if (MI.getOpcode() == C65::REP) {
        if (MI.getOperand(0).isImm()) {
          unsigned Reset = MI.getOperand(0).getImm();
          if (Reset & 0x20) {
            BuildMI(*MBB, std::next(MBBI), DL, TII.get(C65::LONGA_ON));
            Changed = true;
          }
          if (Reset & 0x10) {
            BuildMI(*MBB, std::next(MBBI), DL, TII.get(C65::LONGI_ON));
            Changed = true;
          }
        }
      } else if (MI.getOpcode() == C65::SEP) {
        if (MI.getOperand(0).isImm()) {
          unsigned Set = MI.getOperand(0).getImm();
          if (Set & 0x20) {
            BuildMI(*MBB, std::next(MBBI), DL, TII.get(C65::LONGA_OFF));
            Changed = true;
          }
          if (Set & 0x10) {
            BuildMI(*MBB, std::next(MBBI), DL, TII.get(C65::LONGI_OFF));
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
