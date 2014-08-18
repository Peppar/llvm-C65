//===-- C65ISelLowering.cpp - C65 DAG Lowering Implementation -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the interfaces that C65 uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "C65ISelLowering.h"
//#include "MCTargetDesc/C65MCExpr.h"
//#include "C65MachineFunctionInfo.h"
#include "C65RegisterInfo.h"
#include "C65TargetMachine.h"
#include "C65TargetObjectFile.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
// TargetLowering Implementation
//===----------------------------------------------------------------------===//

C65TargetLowering::C65TargetLowering(TargetMachine &TM)
    : TargetLowering(TM, new C65ELFTargetObjectFile()) {
  Subtarget = &TM.getSubtarget<C65Subtarget>();

  // Set up the register classes.
  addRegisterClass(MVT::i16, &C65::ACC16RegClass);
  addRegisterClass(MVT::i16, &C65::IX16RegClass);
  addRegisterClass(MVT::i16, &C65::IY16RegClass);

  // C65 has no *EXTLOAD
  setLoadExtAction(ISD::EXTLOAD,  MVT::i1, Promote);
  setLoadExtAction(ISD::EXTLOAD,  MVT::i8, Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i1, Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i8, Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i1, Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i8, Promote);

  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1, Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8, Expand);

  // C65 doesn't have BRCOND either, it has BR_CC.
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);
  setOperationAction(ISD::BRIND, MVT::Other, Expand);
  setOperationAction(ISD::BR_JT, MVT::Other, Expand);
  setOperationAction(ISD::BR_CC, MVT::i32, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);

  // AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i8);

  // Custom legalize GlobalAddress nodes into LO/HI parts.
  //  setOperationAction(ISD::GlobalAddress, getPointerTy(), Custom);
  //  setOperationAction(ISD::GlobalTLSAddress, getPointerTy(), Custom);
  //  setOperationAction(ISD::ConstantPool, getPointerTy(), Custom);
  //  setOperationAction(ISD::BlockAddress, getPointerTy(), Custom);


  // for (unsigned VT = (unsigned)FIRST_INTEGER_VALUETYPE;
  //      VT <= (unsigned)LAST_INTEGER_VALUETYPE; ++VT) {
  //   setOperationAction(ISD::ADD, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::ADDC, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::ADDE, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SUB, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SUBC, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SUBE, (MVT::SimpleValueType)VT, Expand);

  //   // C65 has no MUL, SDIV, UDIV, SREM, UREM
  //   setOperationAction(ISD::MUL, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SDIV, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::UDIV, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SREM, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::UREM, (MVT::SimpleValueType)VT, Expand);

  //   // C65 has no REM or DIVREM operations.
  //   setOperationAction(ISD::UREM, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::UREM, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SREM, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::SDIVREM, (MVT::SimpleValueType)VT, Expand);
  //   setOperationAction(ISD::UDIVREM, (MVT::SimpleValueType)VT, Expand);
  // }

  // // Custom expand fp<->sint
  // setOperationAction(ISD::FP_TO_SINT, MVT::i32, Expand);
  // setOperationAction(ISD::SINT_TO_FP, MVT::i32, Expand);
  // setOperationAction(ISD::FP_TO_SINT, MVT::i64, Expand);
  // setOperationAction(ISD::SINT_TO_FP, MVT::i64, Expand);

  // // Custom Expand fp<->uint
  // setOperationAction(ISD::FP_TO_UINT, MVT::i32, Exa);
  // setOperationAction(ISD::UINT_TO_FP, MVT::i32, Custom);
  // setOperationAction(ISD::FP_TO_UINT, MVT::i64, Custom);
  // setOperationAction(ISD::UINT_TO_FP, MVT::i64, Custom);

  // setOperationAction(ISD::BITCAST, MVT::f32, Expand);
  // setOperationAction(ISD::BITCAST, MVT::i32, Expand);

  // // C65 has no select or setcc: expand to SELECT_CC.
  // setOperationAction(ISD::SELECT, MVT::i32, Expand);
  // setOperationAction(ISD::SELECT, MVT::f32, Expand);
  // setOperationAction(ISD::SELECT, MVT::f64, Expand);
  // setOperationAction(ISD::SELECT, MVT::f128, Expand);

  // setOperationAction(ISD::SETCC, MVT::i32, Expand);
  // setOperationAction(ISD::SETCC, MVT::f32, Expand);
  // setOperationAction(ISD::SETCC, MVT::f64, Expand);
  // setOperationAction(ISD::SETCC, MVT::f128, Expand);


  // ATOMICs.
  // FIXME: We insert fences for each atomics and generate sub-optimal code
  // for PSO/TSO. Also, implement other atomicrmw operations.

  // setInsertFencesForAtomic(true);

  // setOperationAction(ISD::ATOMIC_SWAP, MVT::i32, Expand);
  // setOperationAction(ISD::ATOMIC_CMP_SWAP, MVT::i32, Expand);
  // setOperationAction(ISD::ATOMIC_FENCE, MVT::Other, Legal);

  // // Custom Lower Atomic LOAD/STORE
  // setOperationAction(ISD::ATOMIC_LOAD, MVT::i32, Custom);
  // setOperationAction(ISD::ATOMIC_STORE, MVT::i32, Custom);

  // if (Subtarget->is64Bit()) {
  //   setOperationAction(ISD::ATOMIC_CMP_SWAP, MVT::i64, Legal);
  //   setOperationAction(ISD::ATOMIC_SWAP, MVT::i64, Legal);
  //   setOperationAction(ISD::ATOMIC_LOAD, MVT::i64, Custom);
  //   setOperationAction(ISD::ATOMIC_STORE, MVT::i64, Custom);
  // }

  // if (!Subtarget->isV9()) {
  //   // C65V8 does not have FNEGD and FABSD.
  //   setOperationAction(ISD::FNEG, MVT::f64, Custom);
  //   setOperationAction(ISD::FABS, MVT::f64, Custom);
  // }

  // setOperationAction(ISD::FSIN , MVT::f128, Expand);
  // setOperationAction(ISD::FCOS , MVT::f128, Expand);
  // setOperationAction(ISD::FSINCOS, MVT::f128, Expand);
  // setOperationAction(ISD::FREM , MVT::f128, Expand);
  // setOperationAction(ISD::FMA  , MVT::f128, Expand);
  // setOperationAction(ISD::FSIN , MVT::f64, Expand);
  // setOperationAction(ISD::FCOS , MVT::f64, Expand);
  // setOperationAction(ISD::FSINCOS, MVT::f64, Expand);
  // setOperationAction(ISD::FREM , MVT::f64, Expand);
  // setOperationAction(ISD::FMA  , MVT::f64, Expand);
  // setOperationAction(ISD::FSIN , MVT::f32, Expand);
  // setOperationAction(ISD::FCOS , MVT::f32, Expand);
  // setOperationAction(ISD::FSINCOS, MVT::f32, Expand);
  // setOperationAction(ISD::FREM , MVT::f32, Expand);
  // setOperationAction(ISD::FMA  , MVT::f32, Expand);
  // setOperationAction(ISD::CTTZ , MVT::i32, Expand);
  // setOperationAction(ISD::CTTZ_ZERO_UNDEF, MVT::i32, Expand);
  // setOperationAction(ISD::CTLZ , MVT::i32, Expand);
  // setOperationAction(ISD::CTLZ_ZERO_UNDEF, MVT::i32, Expand);
  // setOperationAction(ISD::ROTL , MVT::i32, Expand);
  // setOperationAction(ISD::ROTR , MVT::i32, Expand);
  // setOperationAction(ISD::BSWAP, MVT::i32, Expand);
  // setOperationAction(ISD::FCOPYSIGN, MVT::f128, Expand);
  // setOperationAction(ISD::FCOPYSIGN, MVT::f64, Expand);
  // setOperationAction(ISD::FCOPYSIGN, MVT::f32, Expand);
  // setOperationAction(ISD::FPOW , MVT::f128, Expand);
  // setOperationAction(ISD::FPOW , MVT::f64, Expand);
  // setOperationAction(ISD::FPOW , MVT::f32, Expand);

  //setOperationAction(ISD::SHL_PARTS, MVT::i32, Expand);
  //setOperationAction(ISD::SRA_PARTS, MVT::i32, Expand);
  //setOperationAction(ISD::SRL_PARTS, MVT::i32, Expand);

  // VASTART needs to be custom lowered to use the VarArgsFrameIndex.
  //  setOperationAction(ISD::VASTART           , MVT::Other, Custom);
  //  // VAARG needs to be lowered to not do unaligned accesses for doubles.
  //  setOperationAction(ISD::VAARG             , MVT::Other, Custom);
  //  setOperationAction(ISD::TRAP              , MVT::Other, Legal);

  // Use the default implementation.
  //  setOperationAction(ISD::VACOPY            , MVT::Other, Expand);
  //  setOperationAction(ISD::VAEND             , MVT::Other, Expand);
  //setOperationAction(ISD::STACKSAVE         , MVT::Other, Expand);
  //setOperationAction(ISD::STACKRESTORE      , MVT::Other, Expand);
  //  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i32  , Custom);

  setStackPointerRegisterToSaveRestore(C65::SP);

  //  setOperationAction(ISD::CTPOP, MVT::i32,
  //                     Subtarget->usePopc() ? Legal : Expand);

  // if (Subtarget->isV9() && Subtarget->hasHardQuad()) {
  //   setOperationAction(ISD::LOAD, MVT::f128, Legal);
  //   setOperationAction(ISD::STORE, MVT::f128, Legal);
  // } else {
  //   setOperationAction(ISD::LOAD, MVT::f128, Custom);
  //   setOperationAction(ISD::STORE, MVT::f128, Custom);
  // }

  //setMinFunctionAlignment(1);

  computeRegisterProperties();
}

const char *C65TargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default: return nullptr;
  case C65ISD::PUSH:   return "C65ISD::PUSH";
  case C65ISD::PULL:   return "C65ISD::PULL";
  case C65ISD::CALL:   return "C65ISD::CALL";
  case C65ISD::RET:    return "C65ISD::RET";
  case C65ISD::CMP:    return "C65ISD::CMP";
  case C65ISD::BR:     return "C65ISD::BR";
  case C65ISD::BRCOND: return "C65ISD::BRCOND";
  }
}

EVT C65TargetLowering::getSetCCResultType(LLVMContext &, EVT VT) const {
  if (!VT.isVector())
    return MVT::i8;
  return VT.changeVectorElementTypeToInteger();
}

SDValue C65TargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch(Op.getOpcode()) {
  default:
    llvm_unreachable("Unexpected node to lower");
  case ISD::BR_CC:
    llvm_unreachable("BR_CC not implemented.");
  case ISD::SELECT_CC:
    llvm_unreachable("SELECT_CC not implemented.");
  }
}

MachineBasicBlock *
C65TargetLowering::EmitInstrWithCustomInserter(MachineInstr *MI,
                                               MachineBasicBlock *BB) const {
  switch (MI->getOpcode()) {
  default:
    llvm_unreachable("Unknown SELECT_CC!");
  }
}

bool
C65TargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  return false;
}

// Determine which of the bits specified in Mask are known to be either zero
// or one and return them in the KnownZero/KnownOne bitsets.
void C65TargetLowering::
computeKnownBitsForTargetNode(const SDValue Op,
                              APInt &KnownZero,
                              APInt &KnownOne,
                              const SelectionDAG &DAG,
                              unsigned Depth) const {
  KnownZero = KnownOne = APInt(KnownZero.getBitWidth(), 0);
}

//===----------------------------------------------------------------------===//
// Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "C65GenCallingConv.inc"

SDValue
C65TargetLowering::LowerReturn(SDValue Chain,
                               CallingConv::ID CallConv, bool IsVarArg,
                               const SmallVectorImpl<ISD::OutputArg> &Outs,
                               const SmallVectorImpl<SDValue> &OutVals,
                               SDLoc DL, SelectionDAG &DAG) const {
  SmallVector<CCValAssign, 16> RVLocs;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 DAG.getTarget(), RVLocs, *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_65c816);

  assert(!IsVarArg && "Var args not supported.");
  assert(RVLocs.size() == 0 && "Return values not supported.");

  RetOps[0] = Chain;  // Update chain.

  return DAG.getNode(C65ISD::RET, DL, MVT::Other, RetOps);
}

SDValue C65TargetLowering::
LowerFormalArguments(SDValue Chain,
                     CallingConv::ID CallConv,
                     bool IsVarArg,
                     const SmallVectorImpl<ISD::InputArg> &Ins,
                     SDLoc DL,
                     SelectionDAG &DAG,
                     SmallVectorImpl<SDValue> &InVals) const {

  SmallVector<CCValAssign, 16> ArgLocs;

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), ArgLocs, *DAG.getContext());

  CCInfo.AnalyzeFormalArguments(Ins, CC_65c816);
  assert(ArgLocs.size() == 0 && "Formal arguments not supported.");

  return Chain;
}

SDValue
C65TargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                             SmallVectorImpl<SDValue> &InVals) const {

  SelectionDAG &DAG                     = CLI.DAG;
  SDLoc &DL                             = CLI.DL;

  SmallVector<SDValue, 8> Ops;
  SDValue Glue;
  SDValue Chain = CLI.Chain;
  SmallVector<CCValAssign, 16> ArgLocs;

  CCState CCInfo(CLI.CallConv, CLI.IsVarArg, DAG.getMachineFunction(),
                 DAG.getTarget(), ArgLocs, *DAG.getContext());

  CCInfo.AnalyzeCallOperands(CLI.Outs, CC_65c816);

  assert(!CLI.IsVarArg && "Var args not supported.");
  assert(ArgLocs.size() == 0 && "Call parameters not supported.");

  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);

  return DAG.getNode(C65ISD::CALL, DL, NodeTys, Ops);
}

