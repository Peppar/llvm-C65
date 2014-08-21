//===-- C65ISelLowering.h - C65 DAG Lowering Interface ------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that C65 uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_C65_ISELLOWERING_H
#define LLVM_TARGET_C65_ISELLOWERING_H

#include "C65.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {
  class C65Subtarget;

  namespace C65ISD {
    enum NodeType {
      FIRST_NUMBER = ISD::BUILTIN_OP_END,
      PUSH,
      PULL,
      CALL,
      RET,
      CMP,
      BR,
      BRCOND
    };
  }

  class C65TargetLowering : public TargetLowering {
    const C65Subtarget *Subtarget;
  public:
    C65TargetLowering(TargetMachine &TM);

    /// LowerOperation - Provide custom lowering hooks for some operations.
    ///
    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    /// ReplaceNodeResults - Replace the results of node with an illegal result
    /// type with new values built out of custom code.
    ///
    void ReplaceNodeResults(SDNode *N,
                            SmallVectorImpl<SDValue>& Results,
                            SelectionDAG &DAG) const override;

    /// computeKnownBitsForTargetNode - Determine which of the bits specified
    /// in Mask are known to be either zero or one and return them in the
    /// KnownZero/KnownOne bitsets.
    void computeKnownBitsForTargetNode(const SDValue Op,
                                       APInt &KnownZero,
                                       APInt &KnownOne,
                                       const SelectionDAG &DAG,
                                       unsigned Depth = 0) const override;

    /// getTargetNodeName - This method returns the name of a target specific
    /// DAG node.
    const char *getTargetNodeName(unsigned Opcode) const override;

    bool isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const override;

    SDValue
      LowerGlobalAddress(GlobalAddressSDNode *Node, SelectionDAG &DAG) const;

    SDValue
      LowerFormalArguments(SDValue Chain,
                           CallingConv::ID CallConv,
                           bool isVarArg,
                           const SmallVectorImpl<ISD::InputArg> &Ins,
                           SDLoc dl, SelectionDAG &DAG,
                           SmallVectorImpl<SDValue> &InVals) const override;

    SDValue
      LowerCall(TargetLowering::CallLoweringInfo &CLI,
                SmallVectorImpl<SDValue> &InVals) const override;

    SDValue
      LowerReturn(SDValue Chain,
                  CallingConv::ID CallConv, bool isVarArg,
                  const SmallVectorImpl<ISD::OutputArg> &Outs,
                  const SmallVectorImpl<SDValue> &OutVals,
                  SDLoc dl, SelectionDAG &DAG) const override;

  };
} // end namespace llvm

#endif    // SPARC_ISELLOWERING_H
