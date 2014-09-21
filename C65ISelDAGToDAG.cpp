//===-- C65ISelDAGToDAG.cpp - A dag to dag inst selector for C65 ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines an instruction selector for the C65 target.
//
//===----------------------------------------------------------------------===//

#include "C65TargetMachine.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "c65-isel-dag-to-dag"

//===----------------------------------------------------------------------===//
// Instruction Selector Implementation
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
/// C65DAGToDAGISel - C65 specific code to select C65 machine
/// instructions for SelectionDAG operations.
///
namespace {
class C65DAGToDAGISel : public SelectionDAGISel {
  /// Subtarget - Keep a pointer to the C65 Subtarget around so that we can
  /// make the right decision when generating code for different targets.
  ///
  const C65Subtarget &Subtarget;
  C65TargetMachine &TM;
public:
  explicit C65DAGToDAGISel(C65TargetMachine &tm)
    : SelectionDAGISel(tm),
      Subtarget(tm.getSubtarget<C65Subtarget>()),
      TM(tm) {
  }

  SDNode *Select(SDNode *N) override;

  // Complex pattern selectors
  bool SelectAddrZP(SDValue N, SDValue &Addr);
  bool SelectAddrZY(SDValue N, SDValue &Base, SDValue &Offset);
  bool SelectAddrZ(SDValue N, SDValue &Base, SDValue &Offset);
  bool SelectAddrXY(SDValue N, SDValue &Base, SDValue &Offset);
  bool SelectAddrS(SDValue N, SDValue &Base, SDValue &Offset);

  /// SelectInlineAsmMemoryOperand - Implement addressing mode selection for
  /// inline asm expressions.
  ///
  bool SelectInlineAsmMemoryOperand(const SDValue &Op,
                                    char ConstraintCode,
                                    std::vector<SDValue> &OutOps) override;

  const char *getPassName() const override {
    return "C65 DAG->DAG Pattern Instruction Selection";
  }

  // Include the pieces autogenerated from the target description.
#include "C65GenDAGISel.inc"
};
}  // end anonymous namespace

/// Select address for imm8+S
///
bool C65DAGToDAGISel::SelectAddrS(SDValue Addr, SDValue &Base,
                                  SDValue &Offset) {
  // Allow only frame indices with S indexing
  FrameIndexSDNode *FIN = nullptr;
  if ((FIN = dyn_cast<FrameIndexSDNode>(Addr))) {
    Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), MVT::i16);
    Offset = CurDAG->getTargetConstant(0, MVT::i8);
    return true;
  }
  if (Addr.getOpcode() == ISD::ADD) {
    ConstantSDNode *CN = nullptr;
    if ((FIN = dyn_cast<FrameIndexSDNode>(Addr.getOperand(0))) &&
        (CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1))) &&
        (isInt<8>(CN->getSExtValue()))) {
      // Constant positive word offset from frame index
      Base = CurDAG->getTargetFrameIndex(FIN->getIndex(), MVT::i16);
      Offset = CurDAG->getTargetConstant(CN->getSExtValue(), MVT::i8);
      return true;
    }
  }
  return false;
}

/// Select address for zero page, imm8
///
bool C65DAGToDAGISel::SelectAddrZP(SDValue Addr, SDValue &Offset) {
  ConstantSDNode *CN;
  if ((CN = dyn_cast<ConstantSDNode>(Addr)) &&
      isInt<8>(CN->getSExtValue())) {
    Offset = Addr.getOperand(0);
  }
  return false;
}

// /// Select address for ZR+Y
// ///
// bool C65DAGToDAGISel::SelectAddrZY(SDValue Addr, SDValue &Z, SDValue &Y) {
//   if (Addr.getOpcode() == ISD::ADD) {
//     Z = Addr.getOperand(0);
//     Y = Addr.getOperand(1);
//     return true;
//   }
//   return false;
// }

// /// Select address for ZR+Imm
// ///
// bool C65DAGToDAGISel::SelectAddrZ(SDValue Addr, SDValue &Base,
//                                   SDValue &Offset) {
//   if (Addr.getOpcode() == ISD::FrameIndex ||
//       Addr.getOpcode() == ISD::TargetExternalSymbol ||
//       Addr.getOpcode() == ISD::TargetGlobalAddress ||
//       Addr.getOpcode() == ISD::TargetGlobalTLSAddress) {
//     return false;
//   }
//   if (Addr.getOpcode() == ISD::ADD) {
//     ConstantSDNode *CN;
//     if ((CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1))) &&
//          isInt<16>(CN->getSExtValue())) {
//       FrameIndexSDNode *FIN;
//       if ((FIN =
// 	   dyn_cast<FrameIndexSDNode>(Addr.getOperand(0)))) {
//           // Constant offset from frame ref.
// 	return false;
//       } else {
// 	Base = Addr.getOperand(0);
//       }
//       Offset = CurDAG->getTargetConstant(CN->getZExtValue(), MVT::i16);
//       return true;
//     }
//   }
//   return false;
// }

/// Select address for reg16 + imm16
///
bool C65DAGToDAGISel::SelectAddrRI(SDValue Addr, SDValue &Base,
                                   SDValue &Offset) {
  if (Addr.getOpcode() == ISD::FrameIndex ||
      Addr.getOpcode() == ISD::TargetExternalSymbol ||
      Addr.getOpcode() == ISD::TargetGlobalAddress ||
      Addr.getOpcode() == ISD::TargetGlobalTLSAddress) {
    return false;
  }
  if (Addr.getOpcode() == ISD::ADD) {
    ConstantSDNode *CN;
    if ((CN = dyn_cast<ConstantSDNode>(Addr.getOperand(1))) &&
         isInt<16>(CN->getSExtValue())) {
      FrameIndexSDNode *FIN;
      if ((FIN =
	   dyn_cast<FrameIndexSDNode>(Addr.getOperand(0)))) {
          // Constant offset from frame ref.
	return false;
      } else {
	Base = Addr.getOperand(0);
      }
      Offset = CurDAG->getTargetConstant(CN->getZExtValue(), MVT::i16);
      return true;
    }
  }
  return false;
}

/// Select address for reg16 + reg16
///
bool C65DAGToDAGISel::SelectAddrRR(SDValue Addr, SDValue &R1,
                                   SDValue &R2) {
  if (Addr.getOpcode() == ISD::FrameIndex ||
      Addr.getOpcode() == ISD::TargetExternalSymbol ||
      Addr.getOpcode() == ISD::TargetGlobalAddress ||
      Addr.getOpcode() == ISD::TargetGlobalTLSAddress) {
    return false;
  }
  if (Addr.getOpcode() == ISD::ADD) {
    ConstantSDNode *CN;
    FrameIndexSDNode *FIN;
    if ((FIN = dyn_cast<FrameIndexSDNode>(Addr.getOperand(0)))) {
      // Constant offset from frame ref.
      return false;
    } else {
      R1 = Addr.getOperand(0);
      R2 = Addr.getOperand(1);
    }
  }
  return false;
}

SDNode *C65DAGToDAGISel::Select(SDNode *N) {
  MVT NVT = N->getSimpleValueType(0);
  SDLoc DL(N);

  // If we have a custom node, we already have selected!
  if (N->isMachineOpcode()) {
    DEBUG(dbgs() << "== ";  N->dump(CurDAG); dbgs() << '\n');
    N->setNodeId(-1);
    return nullptr;
  }

  unsigned OpCode = N->getOpcode();
  switch (OpCode) {
  default: break;
  }

  SDNode *ResNode = SelectCode(N);

  DEBUG(dbgs() << "=> ";
        if (ResNode == nullptr || ResNode == N)
          N->dump(CurDAG);
        else
          ResNode->dump(CurDAG);
        dbgs() << '\n');

  return ResNode;
}

/// SelectInlineAsmMemoryOperand - Implement addressing mode selection
/// for inline asm expressions.
///
bool
C65DAGToDAGISel::SelectInlineAsmMemoryOperand(const SDValue &Op,
                                              char ConstraintCode,
                                              std::vector<SDValue> &OutOps) {
  // TODO
  return true;
}

/// createC65ISelDag - This pass converts a legalized DAG into a
/// SPARC-specific DAG, ready for instruction scheduling.
///
FunctionPass *llvm::createC65ISelDag(C65TargetMachine &TM) {
  return new C65DAGToDAGISel(TM);
}
