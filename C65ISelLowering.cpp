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

  // //MachineFunction &MF = DAG.getMachineFunction();

  // // CCValAssign - represent the assignment of the return value to locations.
  // SmallVector<CCValAssign, 16> RVLocs;

  // // CCState - Info about the registers and stack slot.
  // CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
  //                DAG.getTarget(), RVLocs, *DAG.getContext());

  // // Analyze return values.
  // CCInfo.AnalyzeReturn(Outs, RetCC_65c816);

  // SDValue Flag;
  // SmallVector<SDValue, 4> RetOps(1, Chain);
  // // Make room for the return address offset.
  // RetOps.push_back(SDValue());

  // // Copy the result values into the output registers.
  // for (unsigned i = 0; i != RVLocs.size(); ++i) {
  //   CCValAssign &VA = RVLocs[i];
  //   assert(VA.isRegLoc() && "Can only return in registers!");

  //   Chain = DAG.getCopyToReg(Chain, DL, VA.getLocReg(),
  //                            OutVals[i], Flag);

  //   // Guarantee that all emitted copies are stuck together with flags.
  //   Flag = Chain.getValue(1);
  //   RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  // }

  // unsigned RetAddrOffset = 8; // Call Inst + Delay Slot
  // // If the function returns a struct, copy the SRetReturnReg to I0
  // // if (MF.getFunction()->hasStructRetAttr()) {
  // //   C65MachineFunctionInfo *SFI = MF.getInfo<C65MachineFunctionInfo>();
  // //   unsigned Reg = SFI->getSRetReturnReg();
  // //   if (!Reg)
  // //     llvm_unreachable("sret virtual register not created in the entry block");
  // //   SDValue Val = DAG.getCopyFromReg(Chain, DL, Reg, getPointerTy());
  // //   Chain = DAG.getCopyToReg(Chain, DL, SP::I0, Val, Flag);
  // //   Flag = Chain.getValue(1);
  // //   RetOps.push_back(DAG.getRegister(SP::I0, getPointerTy()));
  // //   RetAddrOffset = 12; // CallInst + Delay Slot + Unimp
  // // }

  // RetOps[0] = Chain;  // Update chain.
  // RetOps[1] = DAG.getConstant(RetAddrOffset, MVT::i32);

  // // Add the flag if we have it.
  // if (Flag.getNode())
  //   RetOps.push_back(Flag);

  //return DAG.getNode(C65ISD::RET, DL, MVT::Other, RetOps);
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

  // MachineFunction &MF = DAG.getMachineFunction();
  // MachineFrameInfo *MFI = MF.getFrameInfo();

  // // Assign locations to all of the incoming arguments.
  // SmallVector<CCValAssign, 16> ArgLocs;
  // MachineRegisterInfo &RegInfo = MF.getRegInfo();

  // for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
  //   CCValAssign &VA = ArgLocs[i];
  //   SDValue ArgIn;

  //   if (VA.isRegLoc()) {
  //     unsigned VReg = RegInfo.createVirtualRegister(&C65::ACC16RegClass);
  //     RegInfo.addLiveIn(VA.getLocReg(), VReg);
  //     ArgIn = DAG.getCopyFromReg(Chain, DL, VReg, VA.getLocVT());
  //     InVals.push_back(ArgIn);
  //   } else {
  //     assert(VA.isMemLoc());
  //     unsigned ObjSize = VA.getLocVT().getSizeInBits() / 8;

  //     // Create the frame index object for this incoming parameter
  //     int FI = MFI->CreateFixedObject(ObjSize,
  //                                     VA.getLocMemOffset(),
  //                                     true);

  //     // Create the SelectionDAG nodes corresponding to a load from this parameter
  //     SDValue FIN = DAG.getFrameIndex(FI, getPointerTy());
  //     ArgIn = DAG.getLoad(VA.getLocVT(), DL, Chain, FIN,
  //                         MachinePointerInfo::getFixedStack(FI),
  //                         false, false, false, 0);
  //   }
  // }
  //return Chain;
}

// Value is a value that has been passed to us in the location described by VA
// (and so has type VA.getLocVT()).  Convert Value to VA.getValVT(), chaining
// any loads onto Chain.
// static SDValue convertLocVTToValVT(SelectionDAG &DAG, SDLoc DL,
//                                    CCValAssign &VA, SDValue Chain,
//                                    SDValue Value) {
//   // If the argument has been promoted from a smaller type, insert an
//   // assertion to capture this.
//   if (VA.getLocInfo() == CCValAssign::SExt)
//     Value = DAG.getNode(ISD::AssertSext, DL, VA.getLocVT(), Value,
//                         DAG.getValueType(VA.getValVT()));
//   else if (VA.getLocInfo() == CCValAssign::ZExt)
//     Value = DAG.getNode(ISD::AssertZext, DL, VA.getLocVT(), Value,
//                         DAG.getValueType(VA.getValVT()));

//   if (VA.isExtInLoc())
//     Value = DAG.getNode(ISD::TRUNCATE, DL, VA.getValVT(), Value);
//   else if (VA.getLocInfo() == CCValAssign::Indirect)
//     Value = DAG.getLoad(VA.getValVT(), DL, Chain, Value,
//                         MachinePointerInfo(), false, false, false, 0);
//   else
//     assert(VA.getLocInfo() == CCValAssign::Full && "Unsupported getLocInfo");
//   return Value;
// }

// // Value is a value of type VA.getValVT() that we need to copy into
// // the location described by VA.  Return a copy of Value converted to
// // VA.getValVT().  The caller is responsible for handling indirect values.
// static SDValue convertValVTToLocVT(SelectionDAG &DAG, SDLoc DL,
//                                    CCValAssign &VA, SDValue Value) {
//   switch (VA.getLocInfo()) {
//   case CCValAssign::SExt:
//     return DAG.getNode(ISD::SIGN_EXTEND, DL, VA.getLocVT(), Value);
//   case CCValAssign::ZExt:
//     return DAG.getNode(ISD::ZERO_EXTEND, DL, VA.getLocVT(), Value);
//   case CCValAssign::AExt:
//     return DAG.getNode(ISD::ANY_EXTEND, DL, VA.getLocVT(), Value);
//   case CCValAssign::Full:
//     return Value;
//   default:
//     llvm_unreachable("Unhandled getLocInfo()");
//   }
// }

SDValue
C65TargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                             SmallVectorImpl<SDValue> &InVals) const {
  // SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  // SmallVectorImpl<SDValue> &OutVals     = CLI.OutVals;
  // SmallVectorImpl<ISD::InputArg> &Ins   = CLI.Ins;
  // SDValue Chain                         = CLI.Chain;
  // SDValue Callee                        = CLI.Callee;
  // bool &IsTailCall                      = CLI.IsTailCall;
  // CallingConv::ID CallConv              = CLI.CallConv;
  // bool IsVarArg                         = CLI.IsVarArg;

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
  Chain = DAG.getNode(C65ISD::CALL, DL, NodeTys, Ops);
  Glue = Chain.getValue(1);

  return Chain;

  // MachineFunction &MF = DAG.getMachineFunction();
  //MachineFrameInfo *MFI = MF.getFrameInfo();


  // // C65 target does not yet support tail call optimization.
  // IsTailCall = false;

  // // Analyze operands of the call, assigning locations to each operand.
  // SmallVector<CCValAssign, 16> ArgLocs;

  // // Get the size of the outgoing arguments stack space requirement.
  // unsigned NumBytes = CCInfo.getNextStackOffset();

  // Chain = DAG.getCALLSEQ_START(Chain, DAG.getIntPtrConstant(NumBytes, true), DL);

  // // Create local copies for byval args.
  // //  SmallVector<SDValue, 8> ByValArgs;
  // SmallVector<std::pair<unsigned, SDValue>, 9> RegsToPass;
  // SmallVector<SDValue, 8> MemOpChains;
  // SDValue StackPtr;

  // for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
  //   CCValAssign &VA = ArgLocs[i];
  //   SDValue ArgValue = OutVals[i];
  //   if (VA.getLocInfo() == CCValAssign::Indirect) {
  //     // Store the argument in a stack slot and pass its address.
  //     SDValue SpillSlot = DAG.CreateStackTemporary(VA.getValVT());
  //     int FI = cast<FrameIndexSDNode>(SpillSlot)->getIndex();
  //     MemOpChains.push_back(DAG.getStore(Chain, DL, ArgValue, SpillSlot,
  //                                        MachinePointerInfo::getFixedStack(FI),
  //                                        false, false, 0));
  //     ArgValue = SpillSlot;
  //   } else {
  //     ArgValue = convertValVTToLocVT(DAG, DL, VA, ArgValue);
  //   }
  //   if (VA.isRegLoc()) {
  //     RegsToPass.push_back(std::make_pair(VA.getLocReg(), ArgValue));
  //   } else {
  //     assert(VA.isMemLoc() && "Argument not register or memory");
  //     if (!StackPtr.getNode()) {
  //       StackPtr = DAG.getCopyFromReg(Chain, DL, C65::SP, getPointerTy());
  //     }
  //     unsigned Offset = VA.getLocMemOffset();
  //     SDValue Address = DAG.getNode(ISD::ADD, DL, getPointerTy(), StackPtr,
  //                                   DAG.getIntPtrConstant(Offset));
  //     // Emit the store
  //     MemOpChains.push_back(DAG.getStore(Chain, DL, ArgValue, Address,
  //                                        MachinePointerInfo(),
  //                                        false, false, 0));
  //   }
  // }

  // // Emit all stores, make sure the occur before any copies into physregs.
  // if (!MemOpChains.empty()) {
  //   Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);
  // }


  // // Build a sequence of copy-to-reg nodes chained together with token
  // // chain and flag operands which copy the outgoing args into registers.
  // // The Glue in necessary since all emitted instructions must be
  // // stuck together.
  // SDValue Glue;
  // for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
  //   Chain = DAG.getCopyToReg(Chain, DL, RegsToPass[i].first,
  //                            RegsToPass[i].second, Glue);
  //   Glue = Chain.getValue(1);
  // }

  // SmallVector<SDValue, 8> Ops;
  // Ops.push_back(Chain);
  // Ops.push_back(Callee);

  // // Add argument registers to the end of the list so that they are
  // // known live into the call.
  // for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
  //   Ops.push_back(DAG.getRegister(RegsToPass[i].first,
  //                                 RegsToPass[i].second.getValueType()));
  // }

  // // Glue the call to the argument copies, if any.
  // if (Glue.getNode())
  //   Ops.push_back(Glue);

  // // Emit the call.
  // SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  // Chain = DAG.getNode(C65ISD::CALL, DL, NodeTys, Ops);
  // Glue = Chain.getValue(1);

  // // Mark the end of the call, which is glued to the call itself.
  // Chain = DAG.getCALLSEQ_END(Chain,
  //                            DAG.getConstant(NumBytes, getPointerTy(), true),
  //                            DAG.getConstant(0, getPointerTy(), true),
  //                            Glue, DL);
  // Glue = Chain.getValue(1);

  // // Assign locations to each value returned by this call.
  // SmallVector<CCValAssign, 16> RetLocs;
  // CCState RetCCInfo(CallConv, IsVarArg, MF, DAG.getTarget(), RetLocs,
  //                   *DAG.getContext());

  // RetCCInfo.AnalyzeCallResult(Ins, RetCC_65c816);

  // // Copy all of the result registers out of their specified physreg.
  // for (unsigned i = 0, e = RetLocs.size(); i != e; ++i) {
  //   CCValAssign &VA = RetLocs[i];

  //   // Copy the value out, gluing the copy to the end of the call sequence.
  //   SDValue RetValue = DAG.getCopyFromReg(Chain, DL, VA.getLocReg(),
  //                                         VA.getLocVT(), Glue);
  //   Chain = RetValue.getValue(1);
  //   Glue = RetValue.getValue(2);

  //   // Convert the value of the return register into the value that's
  //   // being returned.
  //   InVals.push_back(convertLocVTToValVT(DAG, DL, VA, Chain, RetValue));
  // }

  // return Chain;
}

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

  // C65 doesn't have i1 sign extending load
  setLoadExtAction(ISD::EXTLOAD,  MVT::i1,  Promote);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i1,  Promote);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i1,  Promote);

  AddPromotedToType(ISD::SETCC, MVT::i1, MVT::i8);

  // Custom legalize GlobalAddress nodes into LO/HI parts.
  //  setOperationAction(ISD::GlobalAddress, getPointerTy(), Custom);
  //  setOperationAction(ISD::GlobalTLSAddress, getPointerTy(), Custom);
  //  setOperationAction(ISD::ConstantPool, getPointerTy(), Custom);
  //  setOperationAction(ISD::BlockAddress, getPointerTy(), Custom);

  // C65 doesn't have sext_inreg, replace them with shl/sra
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1, Expand);
  setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8, Expand);

  // C65 has only the equivalent of ADDE, SUBE
  setOperationAction(ISD::ADD, MVT::i16, Custom);
  setOperationAction(ISD::ADDC, MVT::i16, Custom);
  setOperationAction(ISD::SUB, MVT::i16, Custom);
  setOperationAction(ISD::SUBC, MVT::i16, Custom);

  // C65 has no MUL, SDIV, UDIV, SREM, UREM
  setOperationAction(ISD::MUL, MVT::i32, Expand);
  setOperationAction(ISD::SDIV, MVT::i32, Expand);
  setOperationAction(ISD::UDIV, MVT::i32, Expand);
  setOperationAction(ISD::SREM, MVT::i32, Expand);
  setOperationAction(ISD::UREM, MVT::i32, Expand);

  // C65 has no REM or DIVREM operations.
  setOperationAction(ISD::UREM, MVT::i32, Expand);
  setOperationAction(ISD::UREM, MVT::i32, Expand);
  setOperationAction(ISD::SREM, MVT::i32, Expand);
  setOperationAction(ISD::SDIVREM, MVT::i32, Expand);
  setOperationAction(ISD::UDIVREM, MVT::i32, Expand);

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
  setOperationAction(ISD::BITCAST, MVT::i32, Expand);

  // // C65 has no select or setcc: expand to SELECT_CC.
  // setOperationAction(ISD::SELECT, MVT::i32, Expand);
  // setOperationAction(ISD::SELECT, MVT::f32, Expand);
  // setOperationAction(ISD::SELECT, MVT::f64, Expand);
  // setOperationAction(ISD::SELECT, MVT::f128, Expand);

  // setOperationAction(ISD::SETCC, MVT::i32, Expand);
  // setOperationAction(ISD::SETCC, MVT::f32, Expand);
  // setOperationAction(ISD::SETCC, MVT::f64, Expand);
  // setOperationAction(ISD::SETCC, MVT::f128, Expand);

  // C65 doesn't have BRCOND either, it has BR_CC.
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);
  setOperationAction(ISD::BRIND, MVT::Other, Expand);
  setOperationAction(ISD::BR_JT, MVT::Other, Expand);
  setOperationAction(ISD::BR_CC, MVT::i32, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);

  // ATOMICs.
  // FIXME: We insert fences for each atomics and generate sub-optimal code
  // for PSO/TSO. Also, implement other atomicrmw operations.

  setInsertFencesForAtomic(true);

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

  setOperationAction(ISD::SHL_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRA_PARTS, MVT::i32, Expand);
  setOperationAction(ISD::SRL_PARTS, MVT::i32, Expand);

  // VASTART needs to be custom lowered to use the VarArgsFrameIndex.
  //  setOperationAction(ISD::VASTART           , MVT::Other, Custom);
  //  // VAARG needs to be lowered to not do unaligned accesses for doubles.
  //  setOperationAction(ISD::VAARG             , MVT::Other, Custom);
  //  setOperationAction(ISD::TRAP              , MVT::Other, Legal);

  // Use the default implementation.
  //  setOperationAction(ISD::VACOPY            , MVT::Other, Expand);
  //  setOperationAction(ISD::VAEND             , MVT::Other, Expand);
  setOperationAction(ISD::STACKSAVE         , MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE      , MVT::Other, Expand);
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

  setMinFunctionAlignment(2);

  computeRegisterProperties();
}

const char *C65TargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default: return nullptr;
  case C65ISD::PUSH:   return "C65ISD::PUSH";
  case C65ISD::PULL:   return "C65ISD::PULL";
  case C65ISD::CALL:   return "C65ISD::CALL";
  case C65ISD::RET:    return "C65ISD::RET";
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
  }
}

MachineBasicBlock *
C65TargetLowering::EmitInstrWithCustomInserter(MachineInstr *MI,
                                                 MachineBasicBlock *BB) const {
  switch (MI->getOpcode()) {
  default: llvm_unreachable("Unknown SELECT_CC!");
  }
}
