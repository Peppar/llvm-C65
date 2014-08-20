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
#include "llvm/Support/Debug.h"

using namespace llvm;

#define DEBUG_TYPE "isel-lowering"

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

  // TODO: Remove these for bare-bones?
  // C65 has no *EXTLOAD
  setLoadExtAction(ISD::EXTLOAD,  MVT::i1, Promote);
  setLoadExtAction(ISD::EXTLOAD,  MVT::i8, Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i1, Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i8, Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i1, Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i8, Promote);

  // TODO: Remove these for bare-bones?
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1, Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8, Expand);

  // TODO: Remove these for bare-bones?
  // C65 doesn't have BRCOND either, it has BR_CC.
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);
  setOperationAction(ISD::BRIND, MVT::Other, Expand);
  setOperationAction(ISD::BR_JT, MVT::Other, Expand);
  setOperationAction(ISD::BR_CC, MVT::i32, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);

  //setOperationAction(ISD::SHL, MVT::i16, Custom);

  // Custom legalize GlobalAddress nodes
  setOperationAction(ISD::GlobalAddress, getPointerTy(), Custom);
  setOperationAction(ISD::GlobalTLSAddress, getPointerTy(), Custom);
  setOperationAction(ISD::ConstantPool, getPointerTy(), Custom);
  setOperationAction(ISD::BlockAddress, getPointerTy(), Custom);

  // TODO: Remove this for bare-bones?
  setStackPointerRegisterToSaveRestore(C65::SP);

  computeRegisterProperties();
}

const char *C65TargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return TargetLowering::getTargetNodeName(Opcode);
  case C65ISD::PUSH:   return "C65ISD::PUSH";
  case C65ISD::PULL:   return "C65ISD::PULL";
  case C65ISD::CALL:   return "C65ISD::CALL";
  case C65ISD::RET:    return "C65ISD::RET";
  case C65ISD::CMP:    return "C65ISD::CMP";
  case C65ISD::BR:     return "C65ISD::BR";
  case C65ISD::BRCOND: return "C65ISD::BRCOND";
  }
}

// TODO: Remove this for bare-bones?
EVT C65TargetLowering::getSetCCResultType(LLVMContext &, EVT VT) const {
  if (!VT.isVector())
    return MVT::i8;
  return VT.changeVectorElementTypeToInteger();
}

// SDValue
// BlackfinTargetLowering::LowerGlobalAddress(SDValue Op, SelectionDAG  
// &DAG)
// {
//    DebugLoc DL = Op.getDebugLoc();
//    GlobalValue *GV = cast<GlobalAddressSDNode>(Op)->getGlobal();

//    Op = DAG.getTargetGlobalAddress(GV, MVT::i32);
//    return DAG.getNode(BfinISD::Wrapper, DL, MVT::i32, Op);
// }
SDValue C65TargetLowering::LowerGlobalAddress(GlobalAddressSDNode *Node,
                                              SelectionDAG &DAG) const {
  SDLoc DL(Node);
  const GlobalValue *GV = Node->getGlobal();
  int64_t Offset = Node->getOffset();
  //  Reloc::Model RM = DAG.getTarget().getRelocationModel();
  //  CodeModel::Model CM = DAG.getTarget().getCodeModel();

  DEBUG(errs() << "LowerGlobalAddress offset "
	<< Offset << '\n');
  DEBUG(Node->dump());
  DEBUG(GV->dump());

  SDValue OutNode = DAG.getTargetGlobalAddress(GV, DL, getPointerTy(), Offset);
  DEBUG(OutNode->dump());
  return OutNode;
}

SDValue C65TargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  switch(Op.getOpcode()) {
  default:
    llvm_unreachable("Unexpected node to lower");
  case ISD::GlobalAddress:
    return LowerGlobalAddress(cast<GlobalAddressSDNode>(Op), DAG);
  case ISD::SHL:
    // TODO
    return DAG.getNode(C65::ASLa, DL, MVT::i16, Op.getOperand(0));
  case ISD::BR_CC:
    // TODO
    llvm_unreachable("BR_CC not implemented.");
  case ISD::SELECT_CC:
    // TODO
    llvm_unreachable("SELECT_CC not implemented.");
  }
}

MachineBasicBlock *
C65TargetLowering::EmitInstrWithCustomInserter(MachineInstr *MI,
                                               MachineBasicBlock *BB) const {
  switch (MI->getOpcode()) {
  default:
    // TODO
    llvm_unreachable("Unknown SELECT_CC!");
  }
}

bool
C65TargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // TODO
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
  // TODO
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
  SDValue Glue;

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 DAG.getTarget(), RVLocs, *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_65c816);

  assert(!IsVarArg && "Var args not supported.");
  assert(RVLocs.size() <= 1 && "Maximum of 1 return value supported.");
  if(RVLocs.size() == 1) {
    CCValAssign &VA = RVLocs[0];
    assert(VA.isRegLoc());
    Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(),
                             OutVals[0], Glue);
    Glue = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the glue if we have it.
  if (Glue.getNode())
    RetOps.push_back(Glue);

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

  MachineFunction &MF = DAG.getMachineFunction();
  MachineRegisterInfo &RegInfo = MF.getRegInfo();
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 getTargetMachine(), ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeFormalArguments(Ins, CC_65c816);
  assert(ArgLocs.size() <= 1 && "Max 1 formal argument supported");

  if (ArgLocs.size() == 1) {
    CCValAssign &VA = ArgLocs[0];
    assert(VA.isRegLoc() && "The argument must fit in a register");
    unsigned VReg = RegInfo.createVirtualRegister(&C65::ACC16RegClass);
    RegInfo.addLiveIn(VA.getLocReg(), VReg);
    SDValue Arg = DAG.getCopyFromReg(Chain, DL, VReg, MVT::i16);
    InVals.push_back(Arg);
  }

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

void C65TargetLowering::ReplaceNodeResults(SDNode *N,
                                           SmallVectorImpl<SDValue>& Results,
                                           SelectionDAG &DAG) const {

  SDLoc DL(N);

  RTLIB::Libcall libCall = RTLIB::UNKNOWN_LIBCALL;

  DEBUG(errs() << "Legalize operation " << N->getOpcode());
  N->dump();

  switch (N->getOpcode()) {
  default:
    llvm_unreachable("Do not know how to custom type legalize this operation!");
  }
}
