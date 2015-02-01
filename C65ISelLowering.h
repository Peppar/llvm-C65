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
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Target/TargetLowering.h"

namespace llvm {
  class C65Subtarget;

  namespace C65ISD {
    enum NodeType {
      FIRST_NUMBER = ISD::BUILTIN_OP_END,
      CMP,
      BR_CC,
      SELECT_CC,
      CALL,
      RET,
      PUSH,
      PULL,
      Wrapper,
      FarWrapper
    };
  }

  namespace C65IC {
    enum InstrClass {
      ORA = 0, AND, EOR, ADC, SBC, STA, CMP, LDA, ASL,
      ROL, LSR, ROR, DEC, INC, STX, LDX, STY, LDY,
      CPY, CPX, STZ,

      INSTR_CLASS_END
    };
  }

  class C65TargetLowering : public TargetLowering {
    const C65Subtarget *Subtarget;
  public:
    struct CallReturnInfo {
      Type *Ty;
      bool SExt        : 1;
      bool ZExt        : 1;
      bool IsInReg     : 1;
      bool IsValueUsed : 1;

      CallReturnInfo()
        : Ty(nullptr), SExt(false), ZExt(false), IsInReg(false),
          IsValueUsed(true) {}

      CallReturnInfo &setInRegister(bool Value = true) {
        IsInReg = Value;
        return *this;
      }

      CallReturnInfo &setDiscard(bool Value = true) {
        IsValueUsed = !Value;
        return *this;
      }

      CallReturnInfo &setSExt(bool Value = true) {
        RetSExt = Value;
        return *this;
      }

      CallReturnInfo &setZExtResult(bool Value = true) {
        RetZExt = Value;
        return *this;
      }
    };

    C65TargetLowering(TargetMachine &TM);

    /// computeKnownBitsForTargetNode - Determine which of the bits specified
    /// in Mask are known to be either zero or one and return them in the
    /// KnownZero/KnownOne bitsets.
    void computeKnownBitsForTargetNode(const SDValue Op,
                                       APInt &KnownZero,
                                       APInt &KnownOne,
                                       const SelectionDAG &DAG,
                                       unsigned Depth = 0) const override;

    MachineBasicBlock *
      EmitSimpleZI(MachineInstr *MI, MachineBasicBlock *MBB,
                   unsigned NumBytes) const;
    MachineBasicBlock *
      EmitBinaryZI(MachineInstr *MI, MachineBasicBlock *MBB,
                   unsigned NumBytes, unsigned Instr8, unsigned Instr16,
                   bool clc = false, bool stc = false) const;
    MachineBasicBlock *EmitZBR_CC(MachineInstr *MI, MachineBasicBlock *MBB,
                                 unsigned NumBytes) const;
    MachineBasicBlock *EmitZSELECT_CC(MachineInstr *MI, MachineBasicBlock *MBB,
                                      unsigned NumBytes) const;
    MachineBasicBlock *EmitZST(MachineInstr *MI, MachineBasicBlock *MBB,
                               bool Stack,
                               unsigned NumBytes,
                               bool Far) const;
    MachineBasicBlock *EmitZLD(MachineInstr *MI, MachineBasicBlock *MBB,
                               bool Stack,
                               unsigned NumBytes,
                               bool Far,
                               unsigned ExtendBegin,
                               bool Signed = false) const;
    MachineBasicBlock *EmitZLDimm(MachineInstr *MI, MachineBasicBlock *MBB,
                                  unsigned NumBytes) const;
    MachineBasicBlock *EmitZMOV(MachineInstr *MI,
                                MachineBasicBlock *MBB,
                                unsigned NumBytes,
                                unsigned ExtendBegin,
                                bool Signed = false) const;
    MachineBasicBlock *EmitZLEA(MachineInstr *MI,
                                MachineBasicBlock *MBB) const;
    MachineBasicBlock *EmitZPUSH(MachineInstr *MI,
                                 MachineBasicBlock *MBB,
                                 unsigned NumBytes) const;
    MachineBasicBlock *EmitZPUSHimm(MachineInstr *MI,
                                    MachineBasicBlock *MBB,
                                    unsigned NumBytes) const;
    MachineBasicBlock *EmitZInstr(MachineInstr *MI,
                                  MachineBasicBlock *MBB) const;
    MachineBasicBlock *
      EmitInstrWithCustomInserter(MachineInstr *MI,
                                  MachineBasicBlock *MBB) const override;

    const char *getTargetNodeName(unsigned Opcode) const override;

    bool isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const override;

    MVT getScalarShiftAmountTy(EVT LHSTy) const override { return MVT::i8; }

    /// Return the ISD::SETCC ValueType
    EVT getSetCCResultType(LLVMContext &Context, EVT VT) const override;

    SDValue LowerOperation(SDValue Op, SelectionDAG &DAG) const override;

    SDValue LowerBR_CC(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerShift(SDValue Op, SelectionDAG &DAG) const;

    SDValue LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const;

    // ConstantPool, JumpTable, GlobalAddress, and ExternalSymbol are
    // lowered as their target countpart wrapped in the
    // C65ISD::Wrapper node. Suppose N is one of the above mentioned
    // nodes. It has to be wrapped because otherwise Select(N) returns
    // N. So the raw TargetGlobalAddress nodes, etc. can only be used
    // to form addressing mode.
    SDValue
    LowerConstantPool(SDValue Op, SelectionDAG &DAG) const;

    // SDValue
    // LowerGlobalAddress(const GlobalValue *GV, SDLoc DL,
    //                    int64_t Offset, SelectionDAG &DAG) const;

    SDValue
    LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const;

    SDValue
    LowerExternalSymbol(SDValue Op, SelectionDAG &DAG) const;

    SDValue
    LowerBlockAddress(SDValue Op, SelectionDAG &DAG) const;

    SDValue
    LowerJumpTable(SDValue Op, SelectionDAG &DAG) const;

    // Call lowering
    SDValue
    LowerFormalArguments(SDValue Chain,
                         CallingConv::ID CallConv,
                         bool IsVarArg,
                         const SmallVectorImpl<ISD::InputArg> &Ins,
                         SDLoc dl, SelectionDAG &DAG,
                         SmallVectorImpl<SDValue> &InVals) const override;
    SDValue
    LowerCallResult(SDValue Chain, SDValue Glue,
                    CallingConv::ID CallConv, bool IsVarArg,
                    const SmallVectorImpl<ISD::InputArg> &Ins,
                    SDLoc DL, SelectionDAG &DAG,
                    SmallVectorImpl<SDValue> &InVals) const;

    SDValue
    LowerCall(TargetLowering::CallLoweringInfo &CLI,
              SmallVectorImpl<SDValue> &InVals) const override;

    SDValue
    LowerReturn(SDValue Chain,
                CallingConv::ID CallConv, bool isVarArg,
                const SmallVectorImpl<ISD::OutputArg> &Outs,
                const SmallVectorImpl<SDValue> &OutVals,
                SDLoc dl, SelectionDAG &DAG) const override;


    std::pair<SDValue, SDValue>
    makeC65LibCall(SelectionDAG &DAG,
                   const char *LCName, EVT RetVT,
                   const SDValue *Ops, unsigned NumOps,
                   bool isSigned, SDLoc DL,
                   bool doesNotReturn = false,
                   bool isReturnValueUsed = true) const;

    void ReplaceNodeResults(SDNode *N,
                            SmallVectorImpl<SDValue>& Results,
                            SelectionDAG &DAG) const override;

    // Inline assembly

    // ConstraintType getConstraintType(const std::string &Constraint) const override;

    // ConstraintWeight
    // getSingleConstraintMatchWeight(AsmOperandInfo &info,
    //                                const char *constraint) const override;

    // void LowerAsmOperandForConstraint(SDValue Op,
    //                                   std::string &Constraint,
    //                                   std::vector<SDValue> &Ops,
    //                                   SelectionDAG &DAG) const override;

    // std::pair<unsigned, const TargetRegisterClass*>
    // getRegForInlineAsmConstraint(const std::string &Constraint,
    //                              MVT VT) const override;

  };
} // end namespace llvm

#endif    // SPARC_ISELLOWERING_H
