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

#include "C65InstrInfo.h"
#include "C65ISelLowering.h"
#include "C65MachineFunctionInfo.h"
#include "C65RegisterInfo.h"
#include "C65TargetMachine.h"
#include "MCTargetDesc/C65BaseInfo.h"
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
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "c65-isel-lowering"

//===----------------------------------------------------------------------===//
// TargetLowering implementation
//===----------------------------------------------------------------------===//

C65TargetLowering::C65TargetLowering(const TargetMachine &TM,
                                     const C65Subtarget &STI)
  : TargetLowering(TM), Subtarget(&STI) {

  // Zero-page registers
  addRegisterClass(MVT::i8, &C65::ZRC8RegClass);
  addRegisterClass(MVT::i16, &C65::ZRC16RegClass);
  addRegisterClass(MVT::i32, &C65::ZRC32RegClass);
  addRegisterClass(MVT::i64, &C65::ZRC64RegClass);

  computeRegisterProperties(Subtarget->getRegisterInfo());

  // Division and select is very expensive
  //setIntDivIsCheap(false);
  setSelectIsExpensive(true);

  // Jump is cheap
  setJumpIsExpensive(false);

  // Set up the register classes.
  // addRegisterClass(MVT::i16, &C65::ACC16RegClass);
  // addRegisterClass(MVT::i16, &C65::IX16RegClass);
  // addRegisterClass(MVT::i16, &C65::IY16RegClass);
  // addRegisterClass(MVT::i16, &C65::IS16RegClass);
  // addRegisterClass(MVT::i16, &C65::PC_REGRegClass);
  // addRegisterClass(MVT::i8, &C65::BANK_REGRegClass);
  // addRegisterClass(MVT::i8, &C65::CCRRegClass);

  // Compute derived properties from the register classes
  //computeRegisterProperties(Subtarget->getRegisterInfo());

  // Copied from SystemZ
  //setSchedulingPreference(Sched::RegPressure);

  // Handle operations that are handled in a similar way for all types.
  for (unsigned I = MVT::FIRST_INTEGER_VALUETYPE;
       I <= MVT::LAST_INTEGER_VALUETYPE;
       ++I) {
    MVT VT = MVT::SimpleValueType(I);
    if (isTypeLegal(VT)) {
      // Lower SET_CC into an IPM-based sequence.
      setOperationAction(ISD::SETCC, VT, Expand);

      // Expand SELECT(C, A, B) into SELECT_CC(X, 0, A, B, NE).
      setOperationAction(ISD::SELECT, VT, Expand);

      // SELECT_CC is custom inserted.
      setOperationAction(ISD::SELECT_CC, VT, Custom);

      // BR_CC is custom inserted.
      setOperationAction(ISD::BR_CC, VT, Custom);

      // Custom libcalls for these
      setOperationAction(ISD::SHL, VT, Custom);
      setOperationAction(ISD::SRA, VT, Custom);
      setOperationAction(ISD::SRL, VT, Custom);
      setOperationAction(ISD::ROTL, VT, Custom);
      setOperationAction(ISD::ROTR, VT, Custom);

      setOperationAction(ISD::SIGN_EXTEND_INREG, VT, Expand);
      setOperationAction(ISD::SIGN_EXTEND, VT, Expand);
      setOperationAction(ISD::ZERO_EXTEND, VT, Expand);
      setOperationAction(ISD::ANY_EXTEND, VT, Expand);
      setOperationAction(ISD::TRUNCATE, VT, Expand);

      // These are operations that the 6502 compatibles cannot perform
      // natively; extend to libcalls.
      setOperationAction(ISD::MUL, VT, Expand);
      setOperationAction(ISD::SDIV, VT, Expand);
      setOperationAction(ISD::UDIV, VT, Expand);
      setOperationAction(ISD::SREM, VT, Expand);
      setOperationAction(ISD::UREM, VT, Expand);
      setOperationAction(ISD::SMUL_LOHI, VT, Expand);
      setOperationAction(ISD::UMUL_LOHI, VT, Expand);
      setOperationAction(ISD::SDIVREM, VT, Expand);
      setOperationAction(ISD::UDIVREM, VT, Expand);
      setOperationAction(ISD::CARRY_FALSE, VT, Expand);
      setOperationAction(ISD::ADDC, VT, Expand);
      setOperationAction(ISD::SUBC, VT, Expand);
      setOperationAction(ISD::ADDE, VT, Expand);
      setOperationAction(ISD::SUBE, VT, Expand);
      setOperationAction(ISD::SADDO, VT, Expand);
      setOperationAction(ISD::UADDO, VT, Expand);
      setOperationAction(ISD::SSUBO, VT, Expand);
      setOperationAction(ISD::USUBO, VT, Expand);
      setOperationAction(ISD::SMULO, VT, Expand);
      setOperationAction(ISD::UMULO, VT, Expand);
      setOperationAction(ISD::MULHU, VT, Expand);
      setOperationAction(ISD::MULHS, VT, Expand);
      setOperationAction(ISD::BSWAP, VT, Expand);
      setOperationAction(ISD::CTTZ, VT, Expand);
      setOperationAction(ISD::CTLZ, VT, Expand);
      setOperationAction(ISD::CTPOP, VT, Expand);
      setOperationAction(ISD::CTTZ_ZERO_UNDEF, VT, Expand);
      setOperationAction(ISD::CTLZ_ZERO_UNDEF, VT, Expand);
      setOperationAction(ISD::SHL_PARTS, VT, Expand);
      setOperationAction(ISD::SRA_PARTS, VT, Expand);
    }
  }

  // Expand BRCOND into a BR_CC (see above).
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);
  setOperationAction(ISD::BRIND,  MVT::Other, Expand);
  setOperationAction(ISD::BR_JT,  MVT::Other, Expand);

  for (MVT VT : MVT::integer_valuetypes()) {
    setLoadExtAction(ISD::EXTLOAD,  VT, MVT::i1, Promote);
    setLoadExtAction(ISD::SEXTLOAD, VT, MVT::i1, Promote);
    setLoadExtAction(ISD::ZEXTLOAD, VT, MVT::i1, Promote);
  }

  setOperationAction(ISD::VASTART,        MVT::Other, Custom);
  setOperationAction(ISD::VAARG,          MVT::Other, Custom);

  // Use the default implementation.
  setOperationAction(ISD::VACOPY,         MVT::Other, Expand);
  setOperationAction(ISD::VAEND,          MVT::Other, Expand);
  setOperationAction(ISD::STACKSAVE,      MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE,   MVT::Other, Expand);

  // Custom legalize (near pointer).
  setOperationAction(ISD::ConstantPool,   MVT::i16, Custom);
  setOperationAction(ISD::GlobalAddress,  MVT::i16, Custom);
  setOperationAction(ISD::ExternalSymbol, MVT::i16, Custom);
  setOperationAction(ISD::BlockAddress,   MVT::i16, Custom);
  setOperationAction(ISD::JumpTable,      MVT::i16, Custom);

  // Custom legalize (far pointer).
  setOperationAction(ISD::ConstantPool,   MVT::i32, Custom);
  setOperationAction(ISD::GlobalAddress,  MVT::i32, Custom);
  setOperationAction(ISD::ExternalSymbol, MVT::i32, Custom);
  setOperationAction(ISD::BlockAddress,   MVT::i32, Custom);
  setOperationAction(ISD::JumpTable,      MVT::i32, Custom);

  // Custom libcall names.
  setLibcallName(RTLIB::MUL_I8,      "c65_mul8");
  setLibcallName(RTLIB::MUL_I16,     "c65_mul16");
  setLibcallName(RTLIB::MUL_I32,     "c65_mul32");
  setLibcallName(RTLIB::MUL_I64,     "c65_mul64");
  setLibcallName(RTLIB::SDIV_I8,     "c65_sdiv8");
  setLibcallName(RTLIB::SDIV_I16,    "c65_sdiv16");
  setLibcallName(RTLIB::SDIV_I32,    "c65_sdiv32");
  setLibcallName(RTLIB::SDIV_I64,    "c65_sdiv64");
  setLibcallName(RTLIB::UDIV_I8,     "c65_udiv8");
  setLibcallName(RTLIB::UDIV_I16,    "c65_udiv16");
  setLibcallName(RTLIB::UDIV_I32,    "c65_udiv32");
  setLibcallName(RTLIB::UDIV_I64,    "c65_udiv64");
  setLibcallName(RTLIB::SREM_I8,     "c65_srem8");
  setLibcallName(RTLIB::SREM_I16,    "c65_srem16");
  setLibcallName(RTLIB::SREM_I32,    "c65_srem32");
  setLibcallName(RTLIB::SREM_I64,    "c65_srem64");
  setLibcallName(RTLIB::UREM_I8,     "c65_urem8");
  setLibcallName(RTLIB::UREM_I16,    "c65_urem16");
  setLibcallName(RTLIB::UREM_I32,    "c65_urem32");
  setLibcallName(RTLIB::UREM_I64,    "c65_urem64");
  setLibcallName(RTLIB::SDIVREM_I8,  "c65_sdivrem8");
  setLibcallName(RTLIB::SDIVREM_I16, "c65_sdivrem16");
  setLibcallName(RTLIB::SDIVREM_I32, "c65_sdivrem32");
  setLibcallName(RTLIB::SDIVREM_I64, "c65_sdivrem64");
  setLibcallName(RTLIB::UDIVREM_I8,  "c65_udivrem8");
  setLibcallName(RTLIB::UDIVREM_I16, "c65_udivrem16");
  setLibcallName(RTLIB::UDIVREM_I32, "c65_udivrem32");
  setLibcallName(RTLIB::UDIVREM_I64, "c65_udivrem64");

  // Set all libcalls to preserve all registers.
  for (unsigned I = 0; I < RTLIB::UNKNOWN_LIBCALL; ++I) {
    setLibcallCallingConv((RTLIB::Libcall)I, CallingConv::PreserveAll);
  }

  setStackPointerRegisterToSaveRestore(C65::S);
}

/// This method returns the name of a target specific DAG node.
///
const char *C65TargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return TargetLowering::getTargetNodeName(Opcode);
  case C65ISD::CMP:        return "C65ISD::CMP";
  case C65ISD::BR_CC:      return "C65ISD::BR_CC";
  case C65ISD::SELECT_CC:  return "C65ISD::SELECT_CC";
  case C65ISD::CALL:       return "C65ISD::CALL";
  case C65ISD::RET:        return "C65ISD::RET";
  case C65ISD::PUSH:       return "C65ISD::PUSH";
  case C65ISD::PULL:       return "C65ISD::PULL";
  case C65ISD::FRAME_ADDR: return "C65ISD::FRAME_ADDR";
  case C65ISD::Wrapper:    return "C65ISD::Wrapper";
  case C65ISD::FarWrapper: return "C65ISD::FarWrapper";
  }
}

/// Return the ValueType of the result of SETCC operations.  Also used
/// to obtain the target's preferred type for the condition operand of
/// SELECT and BRCOND nodes.  In the case of BRCOND the argument
/// passed is MVT::Other since there are no other operands to get a
/// type hint from.
///
EVT C65TargetLowering::getSetCCResultType(const DataLayout &DL, LLVMContext &,
                                          EVT VT) const {
  if (!VT.isVector())
    return MVT::i8;
  return VT.changeVectorElementTypeToInteger();
}

/// Return the type that should be used to zero or sign extend a
/// zeroext/signext integer argument or return value.
EVT C65TargetLowering::
getTypeForExtArgOrReturn(LLVMContext &Context, EVT VT,
                         ISD::NodeType ExtendKind) const {
  return VT;
}

/// This callback is invoked for operations that are unsupported by
/// the target, which are registered to use 'custom' lowering, and
/// whose defined values are all legal.
///
SDValue C65TargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch(Op.getOpcode()) {
  default:
    llvm_unreachable("Unexpected node to lower");
  case ISD::SHL:
  case ISD::SRA:
  case ISD::SRL:
  case ISD::ROTL:
  case ISD::ROTR:           return LowerShift(Op, DAG);
  case ISD::BR_CC:          return LowerBR_CC(Op, DAG);
  case ISD::SELECT_CC:      return LowerSELECT_CC(Op, DAG);
  case ISD::FRAMEADDR:      return LowerFRAMEADDR(Op, DAG);
  case ISD::VASTART:        return LowerVASTART(Op, DAG);
  case ISD::VAARG:          return LowerVAARG(Op, DAG);
  case ISD::ConstantPool:   return LowerConstantPool(Op, DAG);
  case ISD::GlobalAddress:  return LowerGlobalAddress(Op, DAG);
  case ISD::ExternalSymbol: return LowerExternalSymbol(Op, DAG);
  case ISD::BlockAddress:   return LowerBlockAddress(Op, DAG);
  case ISD::JumpTable:      return LowerJumpTable(Op, DAG);
  }
}

SDValue C65TargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue Chain    = Op.getOperand(0);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue CmpLHS   = Op.getOperand(2);
  SDValue CmpRHS   = Op.getOperand(3);
  SDValue Dest     = Op.getOperand(4);

  return DAG.getNode(C65ISD::BR_CC, DL, Op.getValueType(),
                     Chain, DAG.getConstant(CC, SDLoc(Op), MVT::i32),
                     CmpLHS, CmpRHS, Dest);
}

SDValue C65TargetLowering::LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue CmpLHS   = Op.getOperand(0);
  SDValue CmpRHS   = Op.getOperand(1);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(4))->get();
  SDValue TrueVal  = Op.getOperand(2);
  SDValue FalseVal = Op.getOperand(3);

  return DAG.getNode(C65ISD::SELECT_CC, DL, Op.getValueType(),
                     CmpLHS, CmpRHS, TrueVal, FalseVal,
                     DAG.getConstant(CC, SDLoc(Op), MVT::i32));
}

SDValue C65TargetLowering::LowerShift(SDValue Op, SelectionDAG &DAG) const {
  EVT VT = Op.getValueType();
  SDLoc DL(Op);
  //  SDValue Chain = DAG.getEntryNode();
  //Type *RetTy = VT.getTypeForEVT(*DAG.getContext());

  // Emit a libcall.
  const char *LCName = 0;
  bool isSigned;
  if (Op.getOpcode() == ISD::SHL) {
    isSigned = false; /*sign irrelevant*/
    if      (VT == MVT::i8)  LCName = "c65_shl8";
    else if (VT == MVT::i16) LCName = "c65_shl16";
    else if (VT == MVT::i32) LCName = "c65_shl32";
    else if (VT == MVT::i64) LCName = "c65_shl64";
  } else if (Op.getOpcode() == ISD::SRL) {
    isSigned = false;
    if      (VT == MVT::i8)  LCName = "c65_lshr8";
    else if (VT == MVT::i16) LCName = "c65_lshr16";
    else if (VT == MVT::i32) LCName = "c65_lshr32";
    else if (VT == MVT::i64) LCName = "c65_lshr64";
  } else if (Op.getOpcode() == ISD::SRA) {
    isSigned = true;
    if      (VT == MVT::i8)  LCName = "c65_ashr8";
    else if (VT == MVT::i16) LCName = "c65_ashr16";
    else if (VT == MVT::i32) LCName = "c65_ashr32";
    else if (VT == MVT::i64) LCName = "c65_ashr64";
  } else if (Op.getOpcode() == ISD::ROTL) {
    isSigned = false;
    if      (VT == MVT::i8)  LCName = "c65_rotl8";
    else if (VT == MVT::i16) LCName = "c65_rotl16";
    else if (VT == MVT::i32) LCName = "c65_rotl32";
    else if (VT == MVT::i64) LCName = "c65_rotl64";
  } else {
    isSigned = false;
    if      (VT == MVT::i8)  LCName = "c65_rotr8";
    else if (VT == MVT::i16) LCName = "c65_rotr16";
    else if (VT == MVT::i32) LCName = "c65_rotr32";
    else if (VT == MVT::i64) LCName = "c65_rotr64";
  }

  if (LCName) {
    SDValue Ops[2] = { Op.getOperand(0), Op.getOperand(1) };
    // TargetLowering::CallLoweringInfo CLI(DAG);
    // CLI.setDebugLoc(DL).setChain(Chain)
    //   .setCallee(CallingConv::C, RetTy, Callee, std::move(Args), 0);
    // std::pair<SDValue, SDValue> CallInfo = LowerCall(CLI);
    // return CallInfo.first;
    return makeC65LibCall(DAG, LCName, VT, Ops, 2, isSigned, DL).first;
  } else {
    llvm_unreachable("Unable to expand shift/rotate.");
  }
}

SDValue
C65TargetLowering::LowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = *MF.getFrameInfo();
  C65MachineFunctionInfo *FuncInfo = MF.getInfo<C65MachineFunctionInfo>();

  // Need frame address to find the address of VarArgsFrameIndex.
  //TODO: Find out if this is necessary
  MFI.setFrameAddressIsTaken(true);

  // vastart just stores the address of the VarArgsFrameIndex slot
  // into the memory location argument.
  SDLoc DL(Op);
  SDValue Offset =
    DAG.getNode(C65ISD::FRAME_ADDR, DL, MVT::i16,
                DAG.getIntPtrConstant(FuncInfo->getVarArgsFrameOffset() + 1,
                                      DL));
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  return DAG.getStore(Op.getOperand(0), DL, Offset, Op.getOperand(1),
                      MachinePointerInfo(SV), false, false, 0);
}

SDValue
C65TargetLowering::LowerVAARG(SDValue Op, SelectionDAG &DAG) const {
  SDNode *Node = Op.getNode();
  EVT VT = Node->getValueType(0);
  SDValue InChain = Node->getOperand(0);
  SDValue VAListPtr = Node->getOperand(1);
  EVT PtrVT = VAListPtr.getValueType();
  const Value *SV = cast<SrcValueSDNode>(Node->getOperand(2))->getValue();
  SDLoc DL(Node);
  SDValue VAList = DAG.getLoad(PtrVT, DL, InChain, VAListPtr,
                               MachinePointerInfo(SV), false, false, false, 0);
  // Increment the pointer, VAList, to the next vaarg.
  SDValue NextPtr = DAG.getNode(ISD::ADD, DL, PtrVT, VAList,
                                DAG.getIntPtrConstant(VT.getSizeInBits()/8,
                                                      DL));
  // Store the incremented VAList to the legalized pointer.
  InChain = DAG.getStore(VAList.getValue(1), DL, NextPtr,
                         VAListPtr, MachinePointerInfo(SV), false, false, 0);
  // Load the actual argument out of the pointer VAList.
  return DAG.getLoad(VT, DL, InChain, VAList, MachinePointerInfo(),
                     false, false, false, 1);
}

SDValue
C65TargetLowering::LowerFRAMEADDR(SDValue Op, SelectionDAG &DAG) const {
  MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
  MFI->setFrameAddressIsTaken(true);

  EVT VT = Op.getValueType();
  SDLoc DL(Op);

  return DAG.getCopyFromReg(DAG.getEntryNode(), DL, C65::S, VT);
}

// ConstantPool, JumpTable, GlobalAddress, and ExternalSymbol are
// lowered as their target countpart wrapped in the C65ISD::Wrapper
// node. Suppose N is one of the above mentioned nodes. It has to be
// wrapped because otherwise Select(N) returns N. So the raw
// TargetGlobalAddress nodes, etc. can only be used to form addressing
// mode.
SDValue
C65TargetLowering::LowerConstantPool(SDValue Op, SelectionDAG &DAG) const {
  ConstantPoolSDNode *CP = cast<ConstantPoolSDNode>(Op);
  SDLoc DL(CP);
  SDValue Result = DAG.getTargetConstantPool(CP->getConstVal(),
                                             getPointerTy(DAG.getDataLayout()),
                                             CP->getAlignment(),
                                             CP->getOffset());
  Result = DAG.getNode(C65ISD::Wrapper, DL, getPointerTy(DAG.getDataLayout()),
                       Result);
  return Result;
}

// SDValue
// C65TargetLowering::LowerGlobalAddress(const GlobalValue *GV, SDLoc DL,
//                                       int64_t Offset, SelectionDAG &DAG) const {
//   GlobalAddressSDNode *G = cast<GlobalAddressSDNode
//   SDValue Result = DAG.getTargetGlobalAddress(GV, DL, getPointerTy(), Offset);
//   if (
//   Result = DAG.getNode(C65ISD::Wrapper, DL, getPointerTy(), Result);
//   return Result;
// }

SDValue
C65TargetLowering::LowerGlobalAddress(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  GlobalAddressSDNode *G = cast<GlobalAddressSDNode>(Op);
  const GlobalValue *GV = G->getGlobal();
  int64_t Offset = G->getOffset();

  SDValue Result = DAG.getTargetGlobalAddress(GV, DL,
                                              getPointerTy(DAG.getDataLayout()),
                                              Offset);
  if (G->getAddressSpace() == C65AS::NEAR_ADDRESS) {
    assert(Op.getValueType() == MVT::i16);
    Result = DAG.getNode(C65ISD::Wrapper, DL, MVT::i16, Result);
  } else if (G->getAddressSpace() == C65AS::FAR_ADDRESS) {
    assert(Op.getValueType() == MVT::i32);
    Result = DAG.getNode(C65ISD::FarWrapper, DL, MVT::i32, Result);
  } else
    llvm_unreachable("Unknown address space in GlobalAddress lowering.");
  return Result;
}

SDValue
C65TargetLowering::LowerExternalSymbol(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  const char *Sym = cast<ExternalSymbolSDNode>(Op)->getSymbol();
  SDValue Result = DAG.getTargetExternalSymbol(Sym,
                                             getPointerTy(DAG.getDataLayout()));
  if (Op.getValueType() == MVT::i16)
    Result = DAG.getNode(C65ISD::Wrapper, DL, MVT::i16, Result);
  else if (Op.getValueType() == MVT::i32)
    Result = DAG.getNode(C65ISD::FarWrapper, DL, MVT::i32, Result);
  else
    llvm_unreachable("Unexpeted return value for external symbol.");
  return Result;
}

SDValue
C65TargetLowering::LowerBlockAddress(SDValue Op, SelectionDAG &DAG) const {
  const BlockAddress *BA = cast<BlockAddressSDNode>(Op)->getBlockAddress();
  int64_t Offset = cast<BlockAddressSDNode>(Op)->getOffset();
  SDLoc DL(Op);
  SDValue Result = DAG.getTargetBlockAddress(BA,
                                             getPointerTy(DAG.getDataLayout()),
                                             Offset);
  if (Op.getValueType() == MVT::i16)
    Result = DAG.getNode(C65ISD::Wrapper, DL, MVT::i16, Result);
  else if (Op.getValueType() == MVT::i32)
    Result = DAG.getNode(C65ISD::FarWrapper, DL, MVT::i32, Result);
  else
    llvm_unreachable("Unexpeted return value for block address.");
  return Result;
}

SDValue
C65TargetLowering::LowerJumpTable(SDValue Op, SelectionDAG &DAG) const {
  JumpTableSDNode *JT = cast<JumpTableSDNode>(Op);
  SDLoc DL(JT);
  SDValue Result = DAG.getTargetJumpTable(JT->getIndex(),
                                          getPointerTy(DAG.getDataLayout()));
  if (Op.getValueType() == MVT::i16)
    Result = DAG.getNode(C65ISD::Wrapper, DL, MVT::i16, Result);
  else if (Op.getValueType() == MVT::i32)
    Result = DAG.getNode(C65ISD::FarWrapper, DL, MVT::i32, Result);
  else
    llvm_unreachable("Unexpeted return value for jump table.");
  return Result;
}

struct Comparison {
  // The operands to the comparison.
  const MachineOperand Op0, Op1;

  // The opcode that should be used to compare Op0 and Op1.
  bool Equality;

  // Is signed comparison
  bool Signed;

  // The result on which we will trigger a branch:
  //   Branch on bitresult == Z flag for equality
  //   Branch on bitresult == N flag for signed
  //   Branch on bitresult == C flag for unsigned
  bool Bitvalue;
};

/// Decompose the comparison to one more easily solved by our arch
///
static struct Comparison getComparison(ISD::CondCode CC,
                                       const MachineOperand Op0,
                                       const MachineOperand Op1) {

  switch (CC) { //           Op0  Op1  Equality Signed Bitvalue
  case ISD::SETEQ:  return { Op0, Op1, true,    false, true };
  case ISD::SETNE:  return { Op0, Op1, true,    false, false };
  case ISD::SETLT:  return { Op0, Op1, false,   true,  true };
  case ISD::SETLE:  return { Op1, Op0, false,   true,  false };
  case ISD::SETGT:  return { Op1, Op0, false,   true,  true };
  case ISD::SETGE:  return { Op0, Op1, false,   true,  false };
  case ISD::SETULT: return { Op0, Op1, false,   false, false };
  case ISD::SETULE: return { Op1, Op0, false,   false, true };
  case ISD::SETUGT: return { Op1, Op0, false,   false, false };
  case ISD::SETUGE: return { Op0, Op1, false,   false, true };
  default:
    llvm_unreachable("Cannot emit this type of comparison!");
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZBR_CC(MachineInstr *MI,
                              MachineBasicBlock *MBB,
                              unsigned NumBytes) const {

  bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  unsigned AccSize = Use8Bit ? 1 : 2;

  DebugLoc DL = MI->getDebugLoc();
  MachineFunction *MF = MBB->getParent();
  //  MachineRegisterInfo &MRI = MF->getRegInfo();

  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator MFI = MBB->getIterator();
  ++MFI;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  struct Comparison C = getComparison((ISD::CondCode)MI->getOperand(0).getImm(),
                                      MI->getOperand(1),
                                      MI->getOperand(2));
  MachineBasicBlock *Dest = MI->getOperand(3).getMBB();

  if (C.Equality) {
    // thisMBB:
    // cmpMBB0:
    //   lda %zra
    //   cmp %zrb
    //   bne sinkMBB/jumpMBB
    // cmpMBB1:
    //   lda %zra+2
    //   cmp %zrb+2
    //   bne sinkMBB/jumpMBB
    //   ...
    //   lda %zra+N
    //   cmp %zrb+N
    //   bne/beq sinkMBB
    // jumpMBB:
    //   jmp Dest
    // sinkMBB:

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *jumpMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *predMBB = thisMBB;
    MachineBasicBlock *cmpMBB;

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);

    for (unsigned I = 0; I < NumBytes; I += AccSize) {
      cmpMBB = MF->CreateMachineBasicBlock(BB);
      MF->insert(MFI, cmpMBB);
      predMBB->addSuccessor(cmpMBB);

      BuildMI(cmpMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(cmpMBB, DL, TII->get(CMPInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
      if (I == NumBytes - AccSize) {
        if (C.Bitvalue) {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(sinkMBB);
          cmpMBB->addSuccessor(sinkMBB);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BEQ)).addMBB(sinkMBB);
          cmpMBB->addSuccessor(sinkMBB);
        }
      } else {
        if (C.Bitvalue) {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(sinkMBB);
          cmpMBB->addSuccessor(sinkMBB);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(jumpMBB);
          cmpMBB->addSuccessor(jumpMBB);
        }
      }
      predMBB = cmpMBB;
    }
    MF->insert(MFI, jumpMBB);
    predMBB->addSuccessor(jumpMBB);
    BuildMI(jumpMBB, DL, TII->get(C65::JMPabs)).addMBB(Dest);
    jumpMBB->addSuccessor(Dest);

    MF->insert(MFI, sinkMBB);

    MI->eraseFromParent();
    return sinkMBB;
  } else if (C.Signed) {
    // thisMBB:
    //   lda %zra
    //   cmp %zrb
    //   lda %zra+2
    //   sbc %zrb+2
    //   ...
    //   bvc braMBB
    // ovfMBB:
    //   eor #$8000
    // braMBB:
    //   bmi/bpl sinkMBB
    // jumpMBB:
    //   jmp Dest
    // sinkMBB:

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *ovfMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *braMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *jumpMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, ovfMBB);
    MF->insert(MFI, braMBB);
    MF->insert(MFI, jumpMBB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(braMBB);
    thisMBB->addSuccessor(ovfMBB);
    ovfMBB->addSuccessor(braMBB);
    braMBB->addSuccessor(sinkMBB);
    braMBB->addSuccessor(jumpMBB);
    jumpMBB->addSuccessor(Dest);

    BuildMI(thisMBB, DL, TII->get(C65::SEC));

    for (unsigned I = 0; I < NumBytes; I += AccSize) {
      BuildMI(thisMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(thisMBB, DL, TII->get(SBCInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
    }

    BuildMI(thisMBB, DL, TII->get(C65::BVC))
      .addMBB(braMBB);

    if (Use8Bit) {
      BuildMI(ovfMBB, DL, TII->get(C65::EOR8imm)).addImm(0x80);
    } else {
      BuildMI(ovfMBB, DL, TII->get(C65::EOR16imm)).addImm(0x8000);
    }

    if (C.Bitvalue) {
      BuildMI(braMBB, DL, TII->get(C65::BPL)).addMBB(sinkMBB);
    } else {
      BuildMI(braMBB, DL, TII->get(C65::BMI)).addMBB(sinkMBB);
    }

    BuildMI(jumpMBB, DL, TII->get(C65::JMPabs)).addMBB(Dest);

    MI->eraseFromParent();
    return sinkMBB;
  } else {
    // thisMBB:
    //   lda %zra
    //   cmp %zrb
    //   lda %zra+2
    //   sbc %zrb+2
    //   ...
    //   bcc/bcs sinkMBB
    // jumpMBB:
    //   jmp Dest
    // sinkMBB:
    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *jumpMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, jumpMBB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(sinkMBB);
    thisMBB->addSuccessor(jumpMBB);
    jumpMBB->addSuccessor(Dest);

    BuildMI(thisMBB, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(C.Op0.getReg()));
    BuildMI(thisMBB, DL, TII->get(CMPInstr))
      .addImm(RI->getZRAddress(C.Op1.getReg()));

    for (unsigned I = AccSize; I < NumBytes; I += AccSize) {
      BuildMI(thisMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(thisMBB, DL, TII->get(SBCInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
    }
    if (C.Bitvalue) {
      BuildMI(thisMBB, DL, TII->get(C65::BCC)).addMBB(sinkMBB);
    } else {
      BuildMI(thisMBB, DL, TII->get(C65::BCS)).addMBB(sinkMBB);
    }
    BuildMI(jumpMBB, DL, TII->get(C65::JMPabs)).addMBB(Dest);

    MI->eraseFromParent();

    return sinkMBB;
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZSELECT_CC(MachineInstr *MI,
                                  MachineBasicBlock *MBB,
                                  unsigned NumBytes) const {

  bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  unsigned AccSize = Use8Bit ? 1 : 2;

  DebugLoc DL = MI->getDebugLoc();
  MachineFunction *MF = MBB->getParent();

  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator MFI = MBB->getIterator();
  ++MFI;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  struct Comparison C = getComparison((ISD::CondCode)MI->getOperand(5).getImm(),
                                      MI->getOperand(1),
                                      MI->getOperand(2));
  unsigned RetReg = MI->getOperand(0).getReg();
  unsigned TrueReg = MI->getOperand(3).getReg();
  unsigned FalseReg = MI->getOperand(4).getReg();

  unsigned ZMOVInstr;
  unsigned NumMovBytes;
  if (C65::ZRC8RegClass.contains(RetReg)) {
    ZMOVInstr = C65::ZMOV8;
    NumMovBytes = 1;
  } else if (C65::ZRC16RegClass.contains(RetReg)) {
    ZMOVInstr = C65::ZMOV16;
    NumMovBytes = 2;
  } else if (C65::ZRC32RegClass.contains(RetReg)) {
    ZMOVInstr = C65::ZMOV32;
    NumMovBytes = 4;
  } else if (C65::ZRC64RegClass.contains(RetReg)) {
    ZMOVInstr = C65::ZMOV64;
    NumMovBytes = 8;
  } else
    llvm_unreachable("Unrecognized register class.");

  // SELECT_CC is lowered in three parts: a comparison part depending
  // on the type of comparison (equality, signed or unsigned
  // comparison), a true MBB and a false MBB. The comparison part
  // branches to the false MBB or falls through or branches to the
  // true MBB depending on outcome.
  //
  // The true and false MBB loads the output register with the true
  // and false value of the machine instruction.

  MachineBasicBlock *thisMBB = MBB;
  MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
  MachineBasicBlock *trueMBB = MF->CreateMachineBasicBlock(BB);
  MachineBasicBlock *falseMBB = MF->CreateMachineBasicBlock(BB);
  MF->insert(MFI, trueMBB);
  MF->insert(MFI, falseMBB);
  MF->insert(MFI, sinkMBB);
  MFI = trueMBB->getIterator();

  // Transfer the remainder of the MBB and its successor edges to sinkMBB.
  sinkMBB->splice(sinkMBB->begin(), MBB,
                  std::next(MachineBasicBlock::iterator(MI)), MBB->end());
  sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);

  if (C.Equality) {
    // thisMBB:
    // cmpMBB0:
    //   lda %zra
    //   cmp %zrb
    //   bne falseMBB/trueMBB
    // cmpMBB1:
    //   lda %zra+2
    //   cmp %zrb+2
    //   bne falseMBB/trueMBB
    //   ...
    //   lda %zra+N
    //   cmp %zrb+N
    //   bne/beq falseMBB

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;

    MachineBasicBlock *cmpMBB;
    MachineBasicBlock *predMBB = thisMBB;

    // cmpMBB(N):
    for (unsigned I = 0; I < NumBytes; I += AccSize) {
      cmpMBB = MF->CreateMachineBasicBlock(BB);
      MF->insert(MFI, cmpMBB);
      predMBB->addSuccessor(cmpMBB);

      BuildMI(cmpMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(cmpMBB, DL, TII->get(CMPInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
      if (I == NumBytes - AccSize) {
        if (C.Bitvalue) {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(falseMBB);
          cmpMBB->addSuccessor(falseMBB);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BEQ)).addMBB(falseMBB);
          cmpMBB->addSuccessor(falseMBB);
        }
      } else {
        if (C.Bitvalue) {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(falseMBB);
          cmpMBB->addSuccessor(falseMBB);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(trueMBB);
          cmpMBB->addSuccessor(trueMBB);
        }
      }
      predMBB = cmpMBB;
    }
    predMBB->addSuccessor(trueMBB);
  } else if (C.Signed) {
    // thisMBB:
    //   sec
    //   lda %zra
    //   sbc %zrb
    //   lda %zra+2
    //   sbc %zrb+2
    //   ...
    //   bvc braMBB
    // ovfMBB:
    //   eor #$8000
    // braMBB:
    //   bmi/bpl falseMBB

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    MachineBasicBlock *ovfMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *braMBB = MF->CreateMachineBasicBlock(BB);

    // thisMBB:
    BuildMI(thisMBB, DL, TII->get(C65::SEC));

    for (unsigned I = 0; I < NumBytes; I += AccSize) {
      BuildMI(thisMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(thisMBB, DL, TII->get(SBCInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
    }
    BuildMI(thisMBB, DL, TII->get(C65::BVC))
      .addMBB(braMBB);
    thisMBB->addSuccessor(braMBB);
    thisMBB->addSuccessor(ovfMBB);

    // ovfMBB:
    MF->insert(MFI, ovfMBB);
    if (Use8Bit) {
      BuildMI(ovfMBB, DL, TII->get(C65::EOR8imm)).addImm(0x80);
    } else {
      BuildMI(ovfMBB, DL, TII->get(C65::EOR16imm)).addImm(0x8000);
    }
    ovfMBB->addSuccessor(braMBB);

    // braMBB:
    MF->insert(MFI, braMBB);
    if (C.Bitvalue) {
      BuildMI(braMBB, DL, TII->get(C65::BPL)).addMBB(falseMBB);
    } else {
      BuildMI(braMBB, DL, TII->get(C65::BMI)).addMBB(falseMBB);
    }
    braMBB->addSuccessor(falseMBB);
    braMBB->addSuccessor(trueMBB);
  } else {
    // thisMBB:
    //   lda %zra
    //   cmp %zrb
    //   lda %zra+2
    //   sbc %zrb+2
    //   ...
    //   bcc/bcs falseMBB
    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    BuildMI(thisMBB, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(C.Op0.getReg()));
    BuildMI(thisMBB, DL, TII->get(CMPInstr))
      .addImm(RI->getZRAddress(C.Op1.getReg()));

    for (unsigned I = AccSize; I < NumBytes; I += AccSize) {
      BuildMI(thisMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(thisMBB, DL, TII->get(SBCInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
    }
    if (C.Bitvalue) {
      BuildMI(thisMBB, DL, TII->get(C65::BCC)).addMBB(falseMBB);
    } else {
      BuildMI(thisMBB, DL, TII->get(C65::BCS)).addMBB(falseMBB);
    }
    thisMBB->addSuccessor(falseMBB);
    thisMBB->addSuccessor(trueMBB);
  }

  // trueMBB:
  MachineInstr *TrueMI =
    BuildMI(trueMBB, DL, TII->get(ZMOVInstr))
    .addReg(RetReg)
    .addReg(TrueReg);
  EmitZMOV(TrueMI, trueMBB, NumMovBytes, NumMovBytes, false);
  BuildMI(trueMBB, DL, TII->get(C65::JMPabs)).addMBB(sinkMBB);
  trueMBB->addSuccessor(sinkMBB);

  // falseMBB:
  MachineInstr *FalseMI =
    BuildMI(falseMBB, DL, TII->get(ZMOVInstr))
    .addReg(RetReg)
    .addReg(FalseReg);
  EmitZMOV(FalseMI, falseMBB, NumMovBytes, NumMovBytes, false);
  falseMBB->addSuccessor(sinkMBB);

  // sinkMBB:

  MI->eraseFromParent();

  return sinkMBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZST(MachineInstr *MI,
                           MachineBasicBlock *MBB,
                           bool Stack,
                           unsigned NumBytes,
                           bool Far) const {
  bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Src = &MI->getOperand(0);
  const MachineOperand *Op1 = &MI->getOperand(1);
  const MachineOperand *Op2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
  unsigned STAInstr;
  unsigned NumOperands = TII->get(MI->getOpcode()).getNumOperands();

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  bool YIndirect;

  if (Stack) {
    assert(!Op1->isReg());
    unsigned FrameReg = RI->getFrameRegister(*MBB->getParent());
    if (FrameReg == C65::S)
      STAInstr = Use8Bit ? C65::STA8srel : C65::STA16srel;
    else if (FrameReg == C65::X)
      STAInstr = Use8Bit ? C65::STA8absx16 : C65::STA16absx16;
    else if (FrameReg == C65::XL)
      STAInstr = C65::STA8absx8;
    else
      llvm_unreachable("Unknown frame register");
    YIndirect = false;
  } else if (NumOperands == 2) {
    assert(!Op1->isReg());
    if (Far)
      STAInstr = Use8Bit ? C65::STA8absl : C65::STA16absl;
    else
      STAInstr = Use8Bit ? C65::STA8abs : C65::STA16abs;
    YIndirect = false;
  } else /* NumOperands == 3 */ {
    assert(Op1->isReg());
    Op2 = &MI->getOperand(2);
    if (Far)
      STAInstr = Use8Bit ? C65::STA8dppostiyl16 : C65::STA16dppostiyl16;
    else
      STAInstr = Use8Bit ? C65::STA8zppostiy16 : C65::STA16zppostiy16;
    if (Op2->isReg()) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::LDY16zp))
        .addImm(RI->getZRAddress(Op2->getReg()));
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::LDY16imm))
        .addImm(Op2->getImm());
    }
    YIndirect = true;
  }

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(Src->getReg()) + I);
    if (YIndirect) {
      BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
        .addImm(RI->getZRAddress(Op1->getReg()));
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
        .addDisp(MI->getOperand(1), I);
    }
    if (YIndirect && I != NumBytes - AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::INY16));
      if (!Use8Bit)
        BuildMI(*MBB, MBBI, DL, TII->get(C65::INY16));
    }
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZLD(MachineInstr *MI,
                           MachineBasicBlock *MBB,
                           bool Stack,
                           unsigned NumBytes,
                           bool Far,
                           unsigned ExtendBegin,
                           bool Signed) const {
  bool Use8Bit = ExtendBegin == 1 || !Subtarget->has65802();
  unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Dest = &MI->getOperand(0);
  const MachineOperand *Op1 = &MI->getOperand(1);
  const MachineOperand *Op2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  unsigned STAInstr = Use8Bit ? C65::STA8zp : C65::STA16zp;
  unsigned STZInstr = Use8Bit ? C65::STZ8zp : C65::STZ16zp;
  unsigned LDAInstr;
  unsigned NumOperands = TII->get(MI->getOpcode()).getNumOperands();

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  bool YIndirect;

  if (Stack) {
    assert(!Op1->isReg());
    unsigned FrameReg = RI->getFrameRegister(*MBB->getParent());
    if (FrameReg == C65::S)
      LDAInstr = Use8Bit ? C65::LDA8srel : C65::LDA16srel;
    else if (FrameReg == C65::X)
      LDAInstr = Use8Bit ? C65::LDA8absx16 : C65::LDA16absx16;
    else if (FrameReg == C65::XL)
      LDAInstr = C65::LDA8absx8;
    else
      llvm_unreachable("Unknown frame register");
    YIndirect = false;
  } else if (NumOperands == 2) {
    assert(!Op1->isReg());
    if (Far)
      LDAInstr = Use8Bit ? C65::LDA8absl : C65::LDA16absl;
    else
      LDAInstr = Use8Bit ? C65::LDA8abs : C65::LDA16abs;
    YIndirect = false;
  } else /* NumOperands == 3 */ {
    assert(Op1->isReg());
    Op2 = &MI->getOperand(2);
    if (Far)
      LDAInstr = Use8Bit ? C65::LDA8dppostiyl16 : C65::LDA16dppostiyl16;
    else
      LDAInstr = Use8Bit ? C65::LDA8zppostiy16 : C65::LDA16zppostiy16;
    if (Op2->isReg()) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::LDY16zp))
        .addImm(RI->getZRAddress(Op2->getReg()));
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::LDY16imm))
        .addImm(Op2->getImm());
    }
    YIndirect = true;
  }

  for (unsigned I = 0; I < ExtendBegin; I += AccSize) {
    if (YIndirect) {
      BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(Op1->getReg()));
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
        .addDisp(MI->getOperand(1), I);
    }
    BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
      .addImm(RI->getZRAddress(Dest->getReg()) + I);
    if (YIndirect && I != NumBytes - AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::INY16));
      if (!Use8Bit)
        BuildMI(*MBB, MBBI, DL, TII->get(C65::INY16));
    }
  }

  if (ExtendBegin == NumBytes) {
    // No extend
    MI->eraseFromParent();
    return MBB;
  } else if (Signed) {
    // thisMBB:
    //   bmi sextMBB
    // zextMBB:
    //   stz %zr+N
    //   stz %zr+N+2
    //   ...
    //   bra/jmp sinkMBB
    // sextMBB:
    //   lda #$FFFF
    //   sta %zr+N
    //   sta %zr+N+2
    // sinkMBB:
    MachineFunction *MF = MBB->getParent();
    const BasicBlock *BB = MBB->getBasicBlock();
    MachineFunction::iterator MFI = MBB->getIterator();
    ++MFI;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *zextMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sextMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, zextMBB);
    MF->insert(MFI, sextMBB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);

    // If minus then jump
    BuildMI(thisMBB, DL, TII->get(C65::BMI))
      .addMBB(sextMBB);

    // Extend with zeroes and then jump to sinkMBB
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(zextMBB, DL, TII->get(STZInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
    BuildMI(zextMBB, DL, TII->get(Subtarget->has65C02() ?
                                  C65::BRA : C65::JMPabs))
      .addMBB(sinkMBB);

    // Extend with ones and then fallthrough to sinkMBB
    BuildMI(sextMBB, DL, TII->get(Use8Bit ? C65::LDA8imm : C65::LDA16imm))
      .addImm(Use8Bit ? 0xFF : 0xFFFF);
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(sextMBB, DL, TII->get(Use8Bit ? C65::STA8zp : C65::STA16zp))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }

    thisMBB->addSuccessor(sextMBB);
    thisMBB->addSuccessor(zextMBB);
    zextMBB->addSuccessor(sinkMBB);
    sextMBB->addSuccessor(sinkMBB);

    MI->eraseFromParent();
    return sinkMBB;
  } else {
    // Always extend with zeroes
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(STZInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
    MI->eraseFromParent();
    return MBB;
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZLDimm(MachineInstr *MI,
                              MachineBasicBlock *MBB,
                              unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Op1 = &MI->getOperand(1);

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA8imm : C65::LDA16imm;
  const unsigned STAInstr = Use8Bit ? C65::STA8zp  : C65::STA16zp;
  const unsigned STZInstr = Use8Bit ? C65::STZ8zp  : C65::STZ16zp;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    unsigned ShiftAmt = I << 3;
    if (Op1->isImm()) {
      unsigned Value;
      if (Use8Bit) {
        Value = Op1->getImm() >> ShiftAmt & 0xFF;
      } else {
        Value = Op1->getImm() >> ShiftAmt & 0xFFFF;
      }
      if (Value == 0) {
        BuildMI(*MBB, MBBI, DL, TII->get(STZInstr))
          .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
      } else {
        BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
          .addImm(Value);
        BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
          .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
      }
    } else {
      // C65 uses the MachineOperand target flags as a bit shift
      // amount. This is later converted to a Shr MCExpr (AsmPrinter),
      // then to the corresponding bit shift fixup kind (CodeEmitter),
      // and finally to the corresponding VLAK stack calculation
      // (MCObjectWriter).
      MachineInstr *LDAMI = BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
        .addOperand(*Op1);
      LDAMI->getOperand(LDAMI->getNumOperands() - 1).setTargetFlags(ShiftAmt);
      BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
        .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
    }
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitBinaryZI(MachineInstr *MI, MachineBasicBlock *MBB,
                                unsigned NumBytes,
                                unsigned Instr8, unsigned Instr16,
                                bool clc, bool stc) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
  const unsigned OPInstr  = Use8Bit ? Instr8 : Instr16;
  const unsigned STAInstr = Use8Bit ? C65::STA8zp : C65::STA16zp;
  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  if (clc) {
    BuildMI(*MBB, MBBI, DL, TII->get(C65::CLC));
  } else if (stc) {
    BuildMI(*MBB, MBBI, DL, TII->get(C65::SEC));
  }
  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(1).getReg()) + I);
    BuildMI(*MBB, MBBI, DL, TII->get(OPInstr))
      .addImm(RI->getZRAddress(MI->getOperand(2).getReg()) + I);
    BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZMOV(MachineInstr *MI,
                            MachineBasicBlock *MBB,
                            unsigned NumBytes,
                            unsigned ExtendBegin,
                            bool Signed) const {
  bool Use8Bit = ExtendBegin == 1 || !Subtarget->has65802();
  unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Dest = &MI->getOperand(0);
  const MachineOperand *Src = &MI->getOperand(1);

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
  unsigned STAInstr = Use8Bit ? C65::STA8zp : C65::STA16zp;
  unsigned STZInstr = Use8Bit ? C65::STZ8zp : C65::STZ16zp;
  //unsigned NumOperands = TII->get(MI->getOpcode()).getNumOperands();

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  if (Src->getReg() == Dest->getReg()) {
    if (ExtendBegin != NumBytes) {
      // This is an in-reg extend operation.  Load the most
      // significant byte to set the negative status bit when
      // performing a sign-extend.
      if (Signed) {
        BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
          .addImm(RI->getZRAddress(Src->getReg()) + ExtendBegin - AccSize);
      }
    }
  } else {
    // The last LDA instruction will set the negative status bit for
    // sign extend.
    for (unsigned I = 0; I < ExtendBegin; I += AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(Src->getReg()) + I);
      BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
  }

  if (ExtendBegin == NumBytes) {
    // No extend
    MI->eraseFromParent();
    return MBB;
  } else if (Signed) {
    // thisMBB:
    //   bmi sextMBB
    // zextMBB:
    //   stz %zr+N
    //   stz %zr+N+2
    //   ...
    //   bra/jmp sinkMBB
    // sextMBB:
    //   lda #$FFFF
    //   sta %zr+N
    //   sta %zr+N+2
    // sinkMBB:
    MachineFunction *MF = MBB->getParent();
    const BasicBlock *BB = MBB->getBasicBlock();
    MachineFunction::iterator MFI = MBB->getIterator();
    ++MFI;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *zextMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sextMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, zextMBB);
    MF->insert(MFI, sextMBB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);

    // If minus then jump
    BuildMI(thisMBB, DL, TII->get(C65::BMI))
      .addMBB(sextMBB);

    // Extend with zeroes and then jump to sinkMBB
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(zextMBB, DL, TII->get(STZInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
    BuildMI(zextMBB, DL, TII->get(Subtarget->has65C02() ?
                                  C65::BRA : C65::JMPabs))
      .addMBB(sinkMBB);

    // Extend with ones and then fallthrough to sinkMBB
    BuildMI(sextMBB, DL, TII->get(Use8Bit ? C65::LDA8imm : C65::LDA16imm))
      .addImm(Use8Bit ? 0xFF : 0xFFFF);
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(sextMBB, DL, TII->get(Use8Bit ? C65::STA8zp : C65::STA16zp))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }

    thisMBB->addSuccessor(sextMBB);
    thisMBB->addSuccessor(zextMBB);
    zextMBB->addSuccessor(sinkMBB);
    sextMBB->addSuccessor(sinkMBB);

    MI->eraseFromParent();
    return sinkMBB;
  } else {
    // Always extend with zeroes
    for (unsigned I = ExtendBegin; I < NumBytes; I += AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(STZInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
    MI->eraseFromParent();
    return MBB;
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZLEA(MachineInstr *MI,
                            MachineBasicBlock *MBB) const {
  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;
  unsigned FrameReg = RI->getFrameRegister(*MBB->getParent());

  BuildMI(*MBB, MBBI, DL, TII->get(C65::CLC));
  if (FrameReg == C65::S)
    BuildMI(*MBB, MBBI, DL, TII->get(C65::TSC));
  else if (FrameReg == C65::X)
    BuildMI(*MBB, MBBI, DL, TII->get(C65::TXA16));
  else if (FrameReg == C65::XL)
    BuildMI(*MBB, MBBI, DL, TII->get(C65::TXA8));
  else
    llvm_unreachable("Unknown frame register");

  BuildMI(*MBB, MBBI, DL, TII->get(C65::ADC16imm))
    .addImm(MI->getOperand(1).getImm());
  BuildMI(*MBB, MBBI, DL, TII->get(C65::STA16zp))
    .addImm(RI->getZRAddress(MI->getOperand(0).getReg()));
  MI->eraseFromParent();
  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZPUSH(MachineInstr *MI,
                             MachineBasicBlock *MBB,
                             unsigned NumBytes) const {
  bool Use8Bit = NumBytes == 1 || !Subtarget->has65802();
  unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Op = &MI->getOperand(0);

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  // Stack grows downwards, push in reverse order.
  for (int I = NumBytes - AccSize; I >= 0; I -= AccSize) {
    if (Use8Bit) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::LDA8zp))
        .addImm(RI->getZRAddress(Op->getReg()) + I);
      BuildMI(*MBB, MBBI, DL, TII->get(C65::PHA8));
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::PEI))
        .addImm(RI->getZRAddress(Op->getReg()) + I);
    }
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZPUSHimm(MachineInstr *MI,
                                MachineBasicBlock *MBB,
                                unsigned NumBytes) const {
  bool Use8Bit = NumBytes == 1 || !Subtarget->has65802();
  unsigned AccSize = Use8Bit ? 1 : 2;

  const MachineOperand *Op = &MI->getOperand(0);

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  //const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  // Stack grows downwards, push in reverse order.
  for (int I = NumBytes - AccSize; I >= 0; I -= AccSize) {
    unsigned ShiftAmt = I << 3;
    MachineInstrBuilder MIB;
    if (Use8Bit) {
      MIB = BuildMI(*MBB, MBBI, DL, TII->get(C65::LDA8imm));
      BuildMI(*MBB, MBBI, DL, TII->get(C65::PHA8));
    } else {
      MIB = BuildMI(*MBB, MBBI, DL, TII->get(C65::PEA));
    }
    if (Op->isImm()) {
      unsigned Value;
      if (Use8Bit) {
        Value = Op->getImm() >> ShiftAmt & 0xFF;
      } else {
        Value = Op->getImm() >> ShiftAmt & 0xFFFF;
      }
      MIB.addImm(Value);
    } else {
      // C65 uses the MachineOperand target flags as a bit shift
      // amount. This is later converted to a Shr MCExpr (AsmPrinter),
      // then to the corresponding bit shift fixup kind (CodeEmitter),
      // and finally to the corresponding VLAK stack calculation
      // (MCObjectWriter).
      MIB.addOperand(*Op);
      MachineInstr *MI2 = MIB;
      MI2->getOperand(MI2->getNumOperands() - 1).setTargetFlags(ShiftAmt);
    }
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZInstr(MachineInstr *MI, MachineBasicBlock *MBB) const {
  unsigned OpSize = 1 << C65II::getZROpSize(MI->getDesc().TSFlags);
  switch (MI->getOpcode()) {
  default: llvm_unreachable("unknown Z instruction to emit");
    // ZPUSH
  case C65::ZPUSH8:
  case C65::ZPUSH16:
  case C65::ZPUSH32:
  case C65::ZPUSH64:
    return EmitZPUSH(MI, MBB, OpSize);

  case C65::ZPUSH8imm:
  case C65::ZPUSH16imm:
  case C65::ZPUSH32imm:
  case C65::ZPUSH64imm:
    return EmitZPUSHimm(MI, MBB, OpSize);

    // BR_CC
  case C65::ZBRCC8:
  case C65::ZBRCC16:
  case C65::ZBRCC32:
  case C65::ZBRCC64:
    return EmitZBR_CC(MI, MBB, OpSize);

  case C65::ZSELECTCC8_8:
  case C65::ZSELECTCC8_16:
  case C65::ZSELECTCC8_32:
  case C65::ZSELECTCC8_64:
  case C65::ZSELECTCC16_8:
  case C65::ZSELECTCC16_16:
  case C65::ZSELECTCC16_32:
  case C65::ZSELECTCC16_64:
  case C65::ZSELECTCC32_8:
  case C65::ZSELECTCC32_16:
  case C65::ZSELECTCC32_32:
  case C65::ZSELECTCC32_64:
  case C65::ZSELECTCC64_8:
  case C65::ZSELECTCC64_16:
  case C65::ZSELECTCC64_32:
  case C65::ZSELECTCC64_64:
    return EmitZSELECT_CC(MI, MBB, OpSize);

    // ZST
  case C65::ZST8s:
  case C65::ZST16s:
  case C65::ZST32s:
  case C65::ZST64s:
    return EmitZST(MI, MBB, true, OpSize, false);
  case C65::ZST8zp:
  case C65::ZST16zp:
  case C65::ZST32zp:
  case C65::ZST64zp:
  case C65::ZST8abs:
  case C65::ZST16abs:
  case C65::ZST32abs:
  case C65::ZST64abs:
  case C65::ZST8ri:
  case C65::ZST16ri:
  case C65::ZST32ri:
  case C65::ZST64ri:
  case C65::ZST8rr:
  case C65::ZST16rr:
  case C65::ZST32rr:
  case C65::ZST64rr:
    return EmitZST(MI, MBB, false, OpSize, false);
  case C65::ZST8absl:
  case C65::ZST16absl:
  case C65::ZST32absl:
  case C65::ZST64absl:
  case C65::ZST8rif:
  case C65::ZST16rif:
  case C65::ZST32rif:
  case C65::ZST64rif:
  case C65::ZST8rrf:
  case C65::ZST16rrf:
  case C65::ZST32rrf:
  case C65::ZST64rrf:
    return EmitZST(MI, MBB, false, OpSize, true);

    // ZST truncating
  case C65::ZST16trunc8s:
  case C65::ZST32trunc8s:
  case C65::ZST64trunc8s:
    return EmitZST(MI, MBB, true, 1, false);
  case C65::ZST16trunc8zp:
  case C65::ZST16trunc8abs:
  case C65::ZST16trunc8ri:
  case C65::ZST16trunc8rr:
  case C65::ZST32trunc8zp:
  case C65::ZST32trunc8abs:
  case C65::ZST32trunc8ri:
  case C65::ZST32trunc8rr:
  case C65::ZST64trunc8zp:
  case C65::ZST64trunc8abs:
  case C65::ZST64trunc8ri:
  case C65::ZST64trunc8rr:
    return EmitZST(MI, MBB, false, 1, false);
  case C65::ZST16trunc8absl:
  case C65::ZST16trunc8rif:
  case C65::ZST16trunc8rrf:
  case C65::ZST32trunc8absl:
  case C65::ZST32trunc8rif:
  case C65::ZST32trunc8rrf:
  case C65::ZST64trunc8absl:
  case C65::ZST64trunc8rif:
  case C65::ZST64trunc8rrf:
    return EmitZST(MI, MBB, false, 1, false);
  case C65::ZST32trunc16s:
  case C65::ZST64trunc16s:
    return EmitZST(MI, MBB, true, 2, false);
  case C65::ZST32trunc16zp:
  case C65::ZST32trunc16abs:
  case C65::ZST32trunc16ri:
  case C65::ZST32trunc16rr:
  case C65::ZST64trunc16zp:
  case C65::ZST64trunc16abs:
  case C65::ZST64trunc16ri:
  case C65::ZST64trunc16rr:
    return EmitZST(MI, MBB, false, 2, false);
  case C65::ZST32trunc16absl:
  case C65::ZST32trunc16rif:
  case C65::ZST32trunc16rrf:
  case C65::ZST64trunc16absl:
  case C65::ZST64trunc16rif:
  case C65::ZST64trunc16rrf:
    return EmitZST(MI, MBB, false, 2, true);
  case C65::ZST64trunc32s:
    return EmitZST(MI, MBB, true, 4, false);
  case C65::ZST64trunc32zp:
  case C65::ZST64trunc32abs:
  case C65::ZST64trunc32ri:
  case C65::ZST64trunc32rr:
    return EmitZST(MI, MBB, false, 4, false);
  case C65::ZST64trunc32absl:
  case C65::ZST64trunc32rif:
  case C65::ZST64trunc32rrf:
    return EmitZST(MI, MBB, false, 4, true);

    // ZLD
  case C65::ZLD8s:
  case C65::ZLD16s:
  case C65::ZLD32s:
  case C65::ZLD64s:
    return EmitZLD(MI, MBB, true, OpSize, false, OpSize);
  case C65::ZLD8zp:
  case C65::ZLD16zp:
  case C65::ZLD32zp:
  case C65::ZLD64zp:
  case C65::ZLD8abs:
  case C65::ZLD16abs:
  case C65::ZLD32abs:
  case C65::ZLD64abs:
  case C65::ZLD8ri:
  case C65::ZLD16ri:
  case C65::ZLD32ri:
  case C65::ZLD64ri:
  case C65::ZLD8rr:
  case C65::ZLD16rr:
  case C65::ZLD32rr:
  case C65::ZLD64rr:
    return EmitZLD(MI, MBB, false, OpSize, false, OpSize);
  case C65::ZLD8absl:
  case C65::ZLD16absl:
  case C65::ZLD32absl:
  case C65::ZLD64absl:
  case C65::ZLD8rif:
  case C65::ZLD16rif:
  case C65::ZLD32rif:
  case C65::ZLD64rif:
  case C65::ZLD8rrf:
  case C65::ZLD16rrf:
  case C65::ZLD32rrf:
  case C65::ZLD64rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, OpSize);

    // ZLD any extend
  case C65::ZLD16ext8s:
  case C65::ZLD32ext8s:
  case C65::ZLD64ext8s:
    return EmitZLD(MI, MBB, true, 1, false, 1);
  case C65::ZLD16ext8zp:
  case C65::ZLD16ext8abs:
  case C65::ZLD16ext8ri:
  case C65::ZLD16ext8rr:
  case C65::ZLD32ext8zp:
  case C65::ZLD32ext8abs:
  case C65::ZLD32ext8ri:
  case C65::ZLD32ext8rr:
  case C65::ZLD64ext8zp:
  case C65::ZLD64ext8abs:
  case C65::ZLD64ext8ri:
  case C65::ZLD64ext8rr:
    return EmitZLD(MI, MBB, false, 1, false, 1);
  case C65::ZLD16ext8absl:
  case C65::ZLD16ext8rif:
  case C65::ZLD16ext8rrf:
  case C65::ZLD32ext8absl:
  case C65::ZLD32ext8rif:
  case C65::ZLD32ext8rrf:
  case C65::ZLD64ext8absl:
  case C65::ZLD64ext8rif:
  case C65::ZLD64ext8rrf:
    return EmitZLD(MI, MBB, false, 1, true, 1);
  case C65::ZLD32ext16s:
  case C65::ZLD64ext16s:
    return EmitZLD(MI, MBB, true, 2, false, 2);
  case C65::ZLD32ext16zp:
  case C65::ZLD32ext16abs:
  case C65::ZLD32ext16ri:
  case C65::ZLD32ext16rr:
  case C65::ZLD64ext16zp:
  case C65::ZLD64ext16abs:
  case C65::ZLD64ext16ri:
  case C65::ZLD64ext16rr:
    return EmitZLD(MI, MBB, false, 2, false, 2);
  case C65::ZLD32ext16absl:
  case C65::ZLD32ext16rif:
  case C65::ZLD32ext16rrf:
  case C65::ZLD64ext16absl:
  case C65::ZLD64ext16rif:
  case C65::ZLD64ext16rrf:
    return EmitZLD(MI, MBB, false, 2, true, 2);
  case C65::ZLD64ext32s:
    return EmitZLD(MI, MBB, true, 4, false, 4);
  case C65::ZLD64ext32zp:
  case C65::ZLD64ext32abs:
  case C65::ZLD64ext32ri:
  case C65::ZLD64ext32rr:
    return EmitZLD(MI, MBB, false, 4, false, 4);
  case C65::ZLD64ext32absl:
  case C65::ZLD64ext32rif:
  case C65::ZLD64ext32rrf:
    return EmitZLD(MI, MBB, false, 4, true, 4);

    // ZLD sign extend
  case C65::ZLD16sext8s:
  case C65::ZLD32sext8s:
  case C65::ZLD64sext8s:
    return EmitZLD(MI, MBB, true, OpSize, false, 1, true);
  case C65::ZLD16sext8zp:
  case C65::ZLD16sext8abs:
  case C65::ZLD16sext8ri:
  case C65::ZLD16sext8rr:
  case C65::ZLD32sext8zp:
  case C65::ZLD32sext8abs:
  case C65::ZLD32sext8ri:
  case C65::ZLD32sext8rr:
  case C65::ZLD64sext8zp:
  case C65::ZLD64sext8abs:
  case C65::ZLD64sext8ri:
  case C65::ZLD64sext8rr:
    return EmitZLD(MI, MBB, false, OpSize, false, 1, true);
  case C65::ZLD16sext8absl:
  case C65::ZLD16sext8rif:
  case C65::ZLD16sext8rrf:
  case C65::ZLD32sext8absl:
  case C65::ZLD32sext8rif:
  case C65::ZLD32sext8rrf:
  case C65::ZLD64sext8absl:
  case C65::ZLD64sext8rif:
  case C65::ZLD64sext8rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 1, true);
  case C65::ZLD32sext16s:
  case C65::ZLD64sext16s:
    return EmitZLD(MI, MBB, true, OpSize, false, 2, true);
  case C65::ZLD32sext16zp:
  case C65::ZLD32sext16abs:
  case C65::ZLD32sext16ri:
  case C65::ZLD32sext16rr:
  case C65::ZLD64sext16zp:
  case C65::ZLD64sext16abs:
  case C65::ZLD64sext16ri:
  case C65::ZLD64sext16rr:
    return EmitZLD(MI, MBB, false, OpSize, false, 2, true);
  case C65::ZLD32sext16absl:
  case C65::ZLD32sext16rif:
  case C65::ZLD32sext16rrf:
  case C65::ZLD64sext16absl:
  case C65::ZLD64sext16rif:
  case C65::ZLD64sext16rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 2, true);
  case C65::ZLD64sext32s:
    return EmitZLD(MI, MBB, true, OpSize, false, 4, true);
  case C65::ZLD64sext32zp:
  case C65::ZLD64sext32abs:
  case C65::ZLD64sext32ri:
  case C65::ZLD64sext32rr:
    return EmitZLD(MI, MBB, false, OpSize, false, 4, true);
  case C65::ZLD64sext32absl:
  case C65::ZLD64sext32rif:
  case C65::ZLD64sext32rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 4, true);

    // ZLD zero extend
  case C65::ZLD16zext8s:
  case C65::ZLD32zext8s:
  case C65::ZLD64zext8s:
    return EmitZLD(MI, MBB, true, OpSize, false, 1, false);
  case C65::ZLD16zext8zp:
  case C65::ZLD16zext8abs:
  case C65::ZLD16zext8ri:
  case C65::ZLD16zext8rr:
  case C65::ZLD32zext8zp:
  case C65::ZLD32zext8abs:
  case C65::ZLD32zext8ri:
  case C65::ZLD32zext8rr:
  case C65::ZLD64zext8zp:
  case C65::ZLD64zext8abs:
  case C65::ZLD64zext8ri:
  case C65::ZLD64zext8rr:
    return EmitZLD(MI, MBB, false, OpSize, false, 1, false);
  case C65::ZLD16zext8absl:
  case C65::ZLD16zext8rif:
  case C65::ZLD16zext8rrf:
  case C65::ZLD32zext8absl:
  case C65::ZLD32zext8rif:
  case C65::ZLD32zext8rrf:
  case C65::ZLD64zext8absl:
  case C65::ZLD64zext8rif:
  case C65::ZLD64zext8rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 1, false);
  case C65::ZLD32zext16s:
  case C65::ZLD64zext16s:
    return EmitZLD(MI, MBB, true, OpSize, false, 2, false);
  case C65::ZLD32zext16zp:
  case C65::ZLD32zext16abs:
  case C65::ZLD32zext16ri:
  case C65::ZLD32zext16rr:
  case C65::ZLD64zext16ri:
  case C65::ZLD64zext16rr:
  case C65::ZLD64zext16zp:
  case C65::ZLD64zext16abs:
    return EmitZLD(MI, MBB, false, OpSize, false, 2, false);
  case C65::ZLD32zext16absl:
  case C65::ZLD32zext16rif:
  case C65::ZLD32zext16rrf:
  case C65::ZLD64zext16absl:
  case C65::ZLD64zext16rif:
  case C65::ZLD64zext16rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 2, false);
  case C65::ZLD64zext32s:
    return EmitZLD(MI, MBB, true, OpSize, false, 4, false);
  case C65::ZLD64zext32zp:
  case C65::ZLD64zext32abs:
  case C65::ZLD64zext32ri:
  case C65::ZLD64zext32rr:
    return EmitZLD(MI, MBB, false, OpSize, false, 4, false);
  case C65::ZLD64zext32absl:
  case C65::ZLD64zext32rif:
  case C65::ZLD64zext32rrf:
    return EmitZLD(MI, MBB, false, OpSize, true, 4, false);

    // ZLD immediate
  case C65::ZLD8imm:
  case C65::ZLD16imm:
  case C65::ZLD32imm:
  case C65::ZLD64imm:
    return EmitZLDimm(MI, MBB, OpSize);

    // ZMOV
  case C65::ZMOV8:
  case C65::ZMOV16:
  case C65::ZMOV32:
  case C65::ZMOV64:
    return EmitZMOV(MI, MBB, OpSize, OpSize);

    // ZMOV any extend
  case C65::ZMOV16ext8:
  case C65::ZMOV32ext8:
  case C65::ZMOV64ext8:
    return EmitZMOV(MI, MBB, 1, 1);
  case C65::ZMOV32ext16:
  case C65::ZMOV64ext16:
    return EmitZMOV(MI, MBB, 2, 2);
  case C65::ZMOV64ext32:
    return EmitZMOV(MI, MBB, 4, 4);

    // ZMOV sign extend
  case C65::ZMOV16sext8:
  case C65::ZMOV32sext8:
  case C65::ZMOV64sext8:
    return EmitZMOV(MI, MBB, OpSize, 1, true);
  case C65::ZMOV32sext16:
  case C65::ZMOV64sext16:
    return EmitZMOV(MI, MBB, OpSize, 2, true);
  case C65::ZMOV64sext32:
    return EmitZMOV(MI, MBB, OpSize, 4, true);

    // ZMOV zero extend
  case C65::ZMOV16zext8:
  case C65::ZMOV32zext8:
  case C65::ZMOV64zext8:
    return EmitZMOV(MI, MBB, OpSize, 1, false);
  case C65::ZMOV32zext16:
  case C65::ZMOV64zext16:
    return EmitZMOV(MI, MBB, OpSize, 2, false);
  case C65::ZMOV64zext32:
    return EmitZMOV(MI, MBB, OpSize, 4, false);

    // ZAND
  case C65::ZAND8:
  case C65::ZAND16:
  case C65::ZAND32:
  case C65::ZAND64:
    return EmitBinaryZI(MI, MBB, OpSize, C65::AND8zp, C65::AND16zp);

    // ZOR
  case C65::ZOR8:
  case C65::ZOR16:
  case C65::ZOR32:
  case C65::ZOR64:
    return EmitBinaryZI(MI, MBB, OpSize, C65::ORA8zp, C65::ORA16zp);

    // ZXOR
  case C65::ZXOR8:
  case C65::ZXOR16:
  case C65::ZXOR32:
  case C65::ZXOR64:
    return EmitBinaryZI(MI, MBB, OpSize, C65::EOR8zp, C65::EOR16zp);

    // ZADD
  case C65::ZADD8:
  case C65::ZADD16:
  case C65::ZADD32:
  case C65::ZADD64:
    return EmitBinaryZI(MI, MBB, OpSize, C65::ADC8zp, C65::ADC16zp,
                        true, false);

    // ZSUB
  case C65::ZSUB8:
  case C65::ZSUB16:
  case C65::ZSUB32:
  case C65::ZSUB64:
    return EmitBinaryZI(MI, MBB, OpSize, C65::SBC8zp, C65::SBC16zp,
                        false, true);

    // ZLEA
  case C65::ZLEA16s:
    return EmitZLEA(MI, MBB);
  }
}

/// This method should be implemented by targets that mark
/// instructions with the 'usesCustomInserter' flag.  These
/// instructions are special in various ways, which require special
/// support to insert.  The specified MachineInstr is created but not
/// inserted into any basic blocks, and this method is called to
/// expand it into a sequence of instructions, potentially also
/// creating new basic blocks and control flow.
///
MachineBasicBlock *
C65TargetLowering::EmitInstrWithCustomInserter(MachineInstr *MI,
                                               MachineBasicBlock *MBB) const {
  return MBB;
}

/// Return true if folding a constant offset with the given
/// GlobalAddress is legal. It is frequently not legal in PIC
/// relocation models.
///
bool
C65TargetLowering::isOffsetFoldingLegal(const GlobalAddressSDNode *GA) const {
  // The C65 target isn't yet aware of offsets
  return false;
}

/// Determine which of the bits specified in Mask are known to be
/// either zero or one and return them in the KnownZero/KnownOne
/// bitsets.
///
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

/// This hook must be implemented to lower outgoing return values,
/// described by the Outs array, into the specified DAG. The
/// implementation should return the resulting token chain value.
///
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
                 RVLocs, *DAG.getContext());

  // Analyze return values.
  CCInfo.AnalyzeReturn(Outs, RetCC_C65);

  for (unsigned I = 0, E = RVLocs.size(); I != E; ++I) {
    CCValAssign &VA = RVLocs[I];
    SDValue RetValue = OutVals[I];
    assert(VA.isRegLoc());

    // Chain and glue the copies together.
    unsigned Reg = VA.getLocReg();
    Chain = DAG.getCopyToReg(Chain, DL, Reg, RetValue, Glue);
    Glue = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(Reg, VA.getLocVT()));
  }

  RetOps[0] = Chain;  // Update chain.

  // Add the glue if we have it.
  if (Glue.getNode())
    RetOps.push_back(Glue);

  return DAG.getNode(C65ISD::RET, DL, MVT::Other, RetOps);
}

/// This hook must be implemented to lower the incoming (formal)
/// arguments, described by the Ins array, into the specified DAG. The
/// implementation should fill in the InVals array with legal-type
/// argument values, and return the resulting token chain value.
///
SDValue C65TargetLowering::
LowerFormalArguments(SDValue Chain,
                     CallingConv::ID CallConv,
                     bool IsVarArg,
                     const SmallVectorImpl<ISD::InputArg> &Ins,
                     SDLoc DL,
                     SelectionDAG &DAG,
                     SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo *MFI = MF.getFrameInfo();
  //MachineRegisterInfo &MRI = MF.getRegInfo();
  C65MachineFunctionInfo *FuncInfo = MF.getInfo<C65MachineFunctionInfo>();

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  SmallVector<CCValAssign, 4> Pulls;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());

  bool Is16Bit = Subtarget->has65802();
  unsigned RetAddrSize = FuncInfo->getIsFar() ? 3 : 2;
  unsigned FramePtrSize = Is16Bit ? 2 : 1;
  CCInfo.AllocateStack(RetAddrSize, 1);
  CCInfo.AllocateStack(FramePtrSize, 1);
  CCInfo.AnalyzeFormalArguments(Ins, CC_C65);

  // The first stack object is the return address.
  //  int ReturnAddrIndex = MFI->CreateStackObject(RetAddrSize, 1,
  //                                               false);
  //  int ReturnAddrIndex = MFI->CreateFixedObject(RetAddrSize, , true);
  //  FuncInfo->setRAIndex(ReturnAddrIndex);

  for (unsigned I = 0, E = ArgLocs.size(); I != E; ++I) {
    SDValue ArgValue;
    CCValAssign &VA = ArgLocs[I];
    EVT LocVT = VA.getLocVT();
    if (VA.isRegLoc()) {
      // Reserve a register for the incoming parameter
      const TargetRegisterClass *RC;
      if (LocVT == MVT::i8) {
        RC = &C65::ZRC8RegClass;
      } else if (LocVT == MVT::i16) {
        RC = &C65::ZRC16RegClass;
      } else if (LocVT == MVT::i32) {
        RC = &C65::ZRC32RegClass;
      } else if (LocVT == MVT::i64) {
        RC = &C65::ZRC64RegClass;
      } else {
        llvm_unreachable("Unknown argument type!");
      }
      unsigned VReg = MF.addLiveIn(VA.getLocReg(), RC);
      ArgValue = DAG.getCopyFromReg(Chain, DL, VReg, LocVT);
    } else {
      assert(VA.isMemLoc() && "Argument not register nor memory");

      EVT ValVT = VA.getValVT();

      // Create the frame index object for this parameter.
      int FI = MFI->CreateFixedObject(ValVT.getSizeInBits() / 8,
                                      VA.getLocMemOffset(), true);

      // Create the SelectionDAG nodes corresponding to a load from
      // this parameter
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DAG.getDataLayout()));
      MachinePointerInfo MPI = MachinePointerInfo::getFixedStack(MF, FI);
      ArgValue = DAG.getLoad(ValVT, DL, Chain, FIN,
                             MPI, false, false, false, 0);
    }
    InVals.push_back(ArgValue);
  }

  if (IsVarArg)
    FuncInfo->setVarArgsFrameOffset(CCInfo.getNextStackOffset());

  return Chain;
}

/// This hook must be implemented to lower calls into the the
/// specified DAG. The outgoing arguments to the call are described by
/// the Outs array, and the values to be returned by the call are
/// described by the Ins array. The implementation should fill in the
/// InVals array with legal-type return values from the call, and
/// return the resulting token chain value.
///
SDValue
C65TargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                             SmallVectorImpl<SDValue> &InVals) const {

  SelectionDAG &DAG = CLI.DAG;
  SDLoc &DL = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  CallingConv::ID CallConv = CLI.CallConv;
  bool IsVarArg = CLI.IsVarArg;

  MachineFunction &MF = DAG.getMachineFunction();
  //MachineFrameInfo *MFI = MF.getFrameInfo();
  //MachineRegisterInfo &MRI = MF.getRegInfo();
  EVT PtrVT = getPointerTy(DAG.getDataLayout());

  // Analyze the operands of the call, assigning locations to each
  // operand.
  SmallVector<CCValAssign, 16> ArgLocs;

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_C65);

  // No support for tail calls
  CLI.IsTailCall = false;

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = CCInfo.getNextStackOffset();

  // Mark the start of the call.
  Chain = DAG.getCALLSEQ_START(Chain, DAG.getIntPtrConstant(0, DL, true), DL);

  // Copy argument values to their designated locations.
  SmallVector<std::pair<unsigned, SDValue>, 3> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;
  SmallVector<SDValue, 4> PushOps;

  for (unsigned I = 0, E = ArgLocs.size(); I != E; ++I) {
    CCValAssign &VA = ArgLocs[I];
    SDValue ArgValue = OutVals[I];

    if (VA.getLocInfo() == CCValAssign::Indirect) {
      // Store the argument in a stack slot and pass its address.
      SDValue SpillSlot = DAG.CreateStackTemporary(VA.getValVT());
      int FI = cast<FrameIndexSDNode>(SpillSlot)->getIndex();
      MachinePointerInfo MPI = MachinePointerInfo::getFixedStack(MF, FI);
      MemOpChains.push_back(DAG.getStore(Chain, DL, ArgValue, SpillSlot,
                                         MPI, false, false, 0));
      ArgValue = SpillSlot;
    }

    if (VA.isRegLoc()) {
      // Queue up the argument copies and emit them at the end.
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), ArgValue));
    } else {
      // Queue up the stack push operations to perform them in reverse
      // order below.
      assert(VA.isMemLoc() && "Argument not register nor memory");
      PushOps.push_back(ArgValue);
    }
  }

  // Perform the push operations right to left.
  if (!PushOps.empty()) {
    SDValue StackPushChain = Chain;
    for (int I = PushOps.size() - 1, E = -1; I != E; --I) {
      StackPushChain = DAG.getNode(C65ISD::PUSH, DL, MVT::Other,
                                   StackPushChain, PushOps[I]);
    }
    MemOpChains.push_back(StackPushChain);
  }

  // Join the stores, which are independent of one another.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);
  }

  SDValue Glue;

  // Accept direct calls by converting symbolic call addresses to the
  // associated Target* opcodes.
  if (auto *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL, PtrVT);
  } else if (auto *E = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(E->getSymbol(), PtrVT);
  }

  // Build a sequence of copy-to-reg nodes, chained and glued together.
  for (unsigned I = 0, E = RegsToPass.size(); I != E; ++I) {
    Chain = DAG.getCopyToReg(Chain, DL, RegsToPass[I].first,
                             RegsToPass[I].second, Glue);
    Glue = Chain.getValue(1);
  }

  // The first call operand is the chain and the second is the target
  // address.
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned I = 0, E = RegsToPass.size(); I != E; ++I) {
    Ops.push_back(DAG.getRegister(RegsToPass[I].first,
                                  RegsToPass[I].second.getValueType()));
  }

  // Add a register mask operand representing the call-preserved
  // registers.
  const TargetRegisterInfo *TRI = Subtarget->getRegisterInfo();
  const uint32_t *Mask = TRI->getCallPreservedMask(MF, CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  // Glue the call to the argument copies, if any.
  if (Glue.getNode()) {
    Ops.push_back(Glue);
  }

  // Emit the call.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  Chain = DAG.getNode(C65ISD::CALL, DL, NodeTys, Ops);
  Glue = Chain.getValue(1);

  // Mark the end of the call, which is glued to the call itself.
  Chain = DAG.getCALLSEQ_END(Chain,
                             DAG.getIntPtrConstant(NumBytes, DL, true),
                             DAG.getIntPtrConstant(0, DL, true),
                             Glue, DL);
  Glue = Chain.getValue(1);

  return LowerCallResult(Chain, Glue, CallConv, IsVarArg,
                         Ins, DL, DAG, InVals);
}

/// LowerCallResult - Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
///
SDValue
C65TargetLowering::LowerCallResult(SDValue Chain, SDValue Glue,
                                   CallingConv::ID CallConv, bool IsVarArg,
                                   const SmallVectorImpl<ISD::InputArg> &Ins,
                                   SDLoc DL, SelectionDAG &DAG,
                                   SmallVectorImpl<SDValue> &InVals) const {

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 RVLocs, *DAG.getContext());
  CCInfo.AnalyzeCallResult(Ins, RetCC_C65);

  // Copy all of the result registers out of their specified physreg.
  for (unsigned I = 0, E = RVLocs.size(); I != E; ++I) {
    CCValAssign &VA = RVLocs[I];
    EVT CopyVT = VA.getValVT();
    SDValue Val;

    Chain = DAG.getCopyFromReg(Chain, DL, VA.getLocReg(),
                               CopyVT, Glue).getValue(1);
    Val = Chain.getValue(0);
    Glue = Chain.getValue(2);
    InVals.push_back(Val);
  }

  return Chain;
}

// Generate a libcall taking the given operands as arguments and returning a
// result of type RetVT.
std::pair<SDValue, SDValue>
C65TargetLowering::makeC65LibCall(SelectionDAG &DAG,
                                  const char *LCName, EVT RetVT,
                                  const SDValue *Ops, unsigned NumOps,
                                  bool isSigned, SDLoc DL,
                                  bool doesNotReturn,
                                  bool isReturnValueUsed) const {
  TargetLowering::ArgListTy Args;
  Args.reserve(NumOps);

  TargetLowering::ArgListEntry Entry;
  for (unsigned I = 0; I != NumOps; ++I) {
    Entry.Node = Ops[I];
    Entry.Ty = Entry.Node.getValueType().getTypeForEVT(*DAG.getContext());
    Entry.isSExt = isSigned;
    Entry.isZExt = !isSigned;
    Args.push_back(Entry);
  }
  SDValue Callee = DAG.getExternalSymbol(LCName,
                                         getPointerTy(DAG.getDataLayout()));

  Type *RetTy = RetVT.getTypeForEVT(*DAG.getContext());
  TargetLowering::CallLoweringInfo CLI(DAG);
  CLI.setDebugLoc(DL).setChain(DAG.getEntryNode())
    .setCallee(CallingConv::PreserveAll, RetTy,
               Callee, std::move(Args), 0)
    .setNoReturn(doesNotReturn).setDiscardResult(!isReturnValueUsed)
    .setSExtResult(isSigned).setZExtResult(!isSigned);
  return LowerCallTo(CLI);
}

/// This callback is invoked when a node result type is illegal for
/// the target, and the operation was registered to use 'custom'
/// lowering for that result type.  The target places new result
/// values for the node in Results (their number and types must
/// exactly match those of the original return values of the node), or
/// leaves Results empty, which indicates that the node is not to be
/// custom lowered after all.
///
/// If the target has no operations that require custom lowering, it
/// need not implement this.  The default implementation aborts.
///
void C65TargetLowering::ReplaceNodeResults(SDNode *N,
                                           SmallVectorImpl<SDValue>& Results,
                                           SelectionDAG &DAG) const {

  //  DEBUG(errs() << "Legalize operation " << N->getOpcode());

  switch (N->getOpcode()) {
  default:
    llvm_unreachable("Do not know how to custom type legalize this operation!");

  }
}
