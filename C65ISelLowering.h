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

#ifndef SPARC_ISELLOWERING_H
#define SPARC_ISELLOWERING_H

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
      BR,
      BRCOND
    };
  }

  class C65TargetLowering : public TargetLowering {
    const C65Subtarget *Subtarget;
  public:
    C65TargetLowering(TargetMachine &TM);

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    /// computeKnownBitsForTargetNode - Determine which of the bits specified
    /// in Mask are known to be either zero or one and return them in the
    /// KnownZero/KnownOne bitsets.
    void computeKnownBitsForTargetNode(const SDValue Op,
                                       APInt &KnownZero,
                                       APInt &KnownOne,
                                       const SelectionDAG &DAG,
                                       unsigned Depth = 0) const override;

    MachineBasicBlock *
      EmitInstrWithCustomInserter(MachineInstr *MI,
                                  MachineBasicBlock *MBB) const override;

    const char *getTargetNodeName(unsigned Opcode) const override;

    ConstraintType getConstraintType(const std::string &Constraint) const override;

    ConstraintWeight
    getSingleConstraintMatchWeight(AsmOperandInfo &info,
                                   const char *constraint) const override;

    void LowerAsmOperandForConstraint(SDValue Op,
                                      std::string &Constraint,
                                      std::vector<SDValue> &Ops,
                                      SelectionDAG &DAG) const override;

    std::pair<unsigned, const TargetRegisterClass*>
    getRegForInlineAsmConstraint(const std::string &Constraint,
                                 MVT VT) const override;

    bool isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const override;

    MVT getScalarShiftAmountTy(EVT LHSTy) const override { return MVT::i32; }

    /// getSetCCResultType - Return the ISD::SETCC ValueType
    EVT getSetCCResultType(LLVMContext &Context, EVT VT) const override;

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

    void ReplaceNodeResults(SDNode *N,
                            SmallVectorImpl<SDValue>& Results,
                            SelectionDAG &DAG) const override;
  };
} // end namespace llvm

#endif    // SPARC_ISELLOWERING_H
