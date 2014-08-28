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
#include "C65RegisterInfo.h"
#include "C65TargetMachine.h"
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

C65TargetLowering::C65TargetLowering(TargetMachine &TM)
    : TargetLowering(TM, new TargetLoweringObjectFileELF()) {
  Subtarget = &TM.getSubtarget<C65Subtarget>();

  // Zero-page registers
  addRegisterClass(MVT::i8, &C65::ZRC8RegClass);
  addRegisterClass(MVT::i16, &C65::ZRC16RegClass);
  addRegisterClass(MVT::i32, &C65::ZRC32RegClass);
  addRegisterClass(MVT::i64, &C65::ZRC64RegClass);

  // Set up the register classes.
  // addRegisterClass(MVT::i16, &C65::ACC16RegClass);
  // addRegisterClass(MVT::i16, &C65::IX16RegClass);
  // addRegisterClass(MVT::i16, &C65::IY16RegClass);
  // addRegisterClass(MVT::i16, &C65::IS16RegClass);
  addRegisterClass(MVT::i16, &C65::PC_REGRegClass);
  addRegisterClass(MVT::i8, &C65::BANK_REGRegClass);
  addRegisterClass(MVT::i8, &C65::CCRRegClass);


  // TODO: Remove these for bare-bones?
  // C65 has no *EXTLOAD
  // setLoadExtAction(ISD::EXTLOAD,  MVT::i1, Promote);
  // setLoadExtAction(ISD::EXTLOAD,  MVT::i8, Promote);
  // setLoadExtAction(ISD::ZEXTLOAD, MVT::i1, Promote);
  // setLoadExtAction(ISD::ZEXTLOAD, MVT::i8, Promote);
  // setLoadExtAction(ISD::SEXTLOAD, MVT::i1, Promote);
  // setLoadExtAction(ISD::SEXTLOAD, MVT::i8, Promote);

  // TODO: Remove these for bare-bones?
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1, Expand);
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8, Expand);

  // TODO: Remove these for bare-bones?
  // C65 doesn't have BRCOND either, it has BR_CC.
  // setOperationAction(ISD::BRCOND, MVT::Other, Expand);
  // setOperationAction(ISD::BRIND, MVT::Other, Expand);
  // setOperationAction(ISD::BR_JT, MVT::Other, Expand);
  // setOperationAction(ISD::BR_CC, MVT::i32, Custom);
  // setOperationAction(ISD::SELECT_CC, MVT::i32, Custom);

  //setOperationAction(ISD::SHL, MVT::i16, Custom);

  // Custom legalize GlobalAddress nodes
  setOperationAction(ISD::GlobalAddress, getPointerTy(), Custom);
  // TODO: Remove these three?
  // setOperationAction(ISD::GlobalTLSAddress, getPointerTy(), Custom);
  // setOperationAction(ISD::ConstantPool, getPointerTy(), Custom);
  // setOperationAction(ISD::BlockAddress, getPointerTy(), Custom);

  // TODO: Remove this for bare-bones?
  setStackPointerRegisterToSaveRestore(C65::S);

  computeRegisterProperties();
}

/// This method returns the name of a target specific DAG node.
///
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

/// Return the ValueType of the result of SETCC operations.  Also used
/// to obtain the target's preferred type for the condition operand of
/// SELECT and BRCOND nodes.  In the case of BRCOND the argument
/// passed is MVT::Other since there are no other operands to get a
/// type hint from.
///
EVT C65TargetLowering::getSetCCResultType(LLVMContext &, EVT VT) const {
  if (!VT.isVector())
    return MVT::i8;
  return VT.changeVectorElementTypeToInteger();
}

SDValue C65TargetLowering::LowerGlobalAddress(GlobalAddressSDNode *Node,
                                              SelectionDAG &DAG) const {
  SDLoc DL(Node);
  return DAG.getTargetGlobalAddress(Node->getGlobal(), DL, getPointerTy());
}


/// This callback is invoked for operations that are unsupported by
/// the target, which are registered to use 'custom' lowering, and
/// whose defined values are all legal. If the target has no
/// operations that require custom lowering, it need not implement
/// this.  The default implementation of this aborts.
///
SDValue C65TargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  switch(Op.getOpcode()) {
  default:
    llvm_unreachable("Unexpected node to lower");
  case C65ISD::SPILL:
    Op.dump();
    llvm_unreachable("Trying to spill!");
  case ISD::GlobalAddress:
    return LowerGlobalAddress(cast<GlobalAddressSDNode>(Op), DAG);
  }
}

MCInstrDesc C65TargetLowering::getOp(unsigned Op) const {
  const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
  return TII.get(Op);
}

uint8_t C65TargetLowering::getZRAddress(MachineOperand Val) const {
  const C65RegisterInfo &TRI =
      *static_cast<const C65RegisterInfo *>(Subtarget->getRegisterInfo());
  //  CurDAG->getConstant(, Op.getValueType());
  return TRI.getZRAddress(Val.getReg());
}

/// Get the operand size of the function. This should be replaced by a
/// more declarative approach.
///
static unsigned getOpByteSize(unsigned Op) {
  switch (Op) {
  default: llvm_unreachable("Unknown opcode!");
  case C65::STZ8z:
  case C65::ST8zi:
  case C65::LD8zi:
  case C65::LD8zimm:
  case C65::AND8zz:
  case C65::OR8zz:
  case C65::XOR8zz:
  case C65::MOV8zz:
  case C65::ADD8zz:
  case C65::SUB8zz:
    return 1;
  case C65::STZ16z:
  case C65::ST16zi:
  case C65::LD16zi:
  case C65::LD16zimm:
  case C65::AND16zz:
  case C65::OR16zz:
  case C65::XOR16zz:
  case C65::MOV16zz:
  case C65::ADD16zz:
  case C65::SUB16zz:
    return 2;
  case C65::STZ32z:
  case C65::ST32zi:
  case C65::LD32zi:
  case C65::LD32zimm:
  case C65::AND32zz:
  case C65::OR32zz:
  case C65::XOR32zz:
  case C65::MOV32zz:
  case C65::ADD32zz:
  case C65::SUB32zz:
    return 4;
  case C65::STZ64z:
  case C65::ST64zi:
  case C65::LD64zi:
  case C65::LD64zimm:
  case C65::AND64zz:
  case C65::OR64zz:
  case C65::XOR64zz:
  case C65::MOV64zz:
  case C65::ADD64zz:
  case C65::SUB64zz:
    return 8;
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
  unsigned NumBytes = getOpByteSize(MI->getOpcode());
  DebugLoc DL = MI->getDebugLoc();

  if (NumBytes == 1) {
    BuildMI(*MBB, MI, DL, getOp(C65::SEP))
      .addImm(0x20);
  }
  MI->dump();
  for (unsigned I = 0; I < NumBytes; I += 2) {
    switch (MI->getOpcode()) {
    default: llvm_unreachable("Unknown custom opcode to emit!");
    case C65::STZ8z:
    case C65::STZ16z:
    case C65::STZ32z:
    case C65::STZ64z: {
      BuildMI(*MBB, MI, DL, getOp(C65::STZzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::ST8zi:
    case C65::ST16zi:
    case C65::ST32zi:
    case C65::ST64zi: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAi))
        .addGlobalAddress(MI->getOperand(1).getGlobal(), I);
    } break;
    case C65::LD8zi:
    case C65::LD16zi:
    case C65::LD32zi:
    case C65::LD64zi: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAi))
        .addGlobalAddress(MI->getOperand(1).getGlobal(), I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::LD8zimm:
    case C65::LD16zimm:
    case C65::LD32zimm:
    case C65::LD64zimm: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAimm))
        .addImm(MI->getOperand(1).getImm() >> (16 * I) & 0xFFFF);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::AND8zz:
    case C65::AND16zz:
    case C65::AND32zz:
    case C65::AND64zz: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAzp))
        .addImm(getZRAddress(MI->getOperand(1)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::ANDzp))
        .addImm(getZRAddress(MI->getOperand(2)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::OR8zz:
    case C65::OR16zz:
    case C65::OR32zz:
    case C65::OR64zz: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAzp))
        .addImm(getZRAddress(MI->getOperand(1)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::ORAzp))
        .addImm(getZRAddress(MI->getOperand(2)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::XOR8zz:
    case C65::XOR16zz:
    case C65::XOR32zz:
    case C65::XOR64zz: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAzp))
        .addImm(getZRAddress(MI->getOperand(1)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::EORzp))
        .addImm(getZRAddress(MI->getOperand(2)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    case C65::MOV8zz:
    case C65::MOV16zz:
    case C65::MOV32zz:
    case C65::MOV64zz: {
      BuildMI(*MBB, MI, DL, getOp(C65::LDAzp))
        .addImm(getZRAddress(MI->getOperand(1)) + I);
      BuildMI(*MBB, MI, DL, getOp(C65::STAzp))
        .addImm(getZRAddress(MI->getOperand(0)) + I);
    } break;
    }
  }
  if (NumBytes == 1) {
    BuildMI(*MBB, MI, DL, getOp(C65::REP))
      .addImm(0x20);
  }
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

/// This hook must be implemented to lower the incoming (formal) arguments,
/// described by the Ins array, into the specified DAG. The
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
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());

  CCInfo.AnalyzeFormalArguments(Ins, CC_65c816);

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    SDValue ArgValue;
    CCValAssign &VA = ArgLocs[i];
    EVT LocVT = VA.getLocVT();
    if (VA.isRegLoc()) {
      // Reserve a register for the incoming parameter
      unsigned VReg = MRI.createVirtualRegister(&C65::ZRC16RegClass);
      MRI.addLiveIn(VA.getLocReg(), VReg);
      ArgValue = DAG.getCopyFromReg(Chain, DL, VReg, MVT::i16);
    } else {
      assert(VA.isMemLoc());
      // Create the frame index object for this parameter (2 bytes)
      int FI = MFI->CreateFixedObject(2, VA.getLocMemOffset(), true);

      // Create the SelectionDAG nodes corresponding to a load from
      // this parameter
      EVT PtrVT = getPointerTy();
      SDValue FIN = DAG.getFrameIndex(FI, PtrVT);
      ArgValue = DAG.getLoad(LocVT, DL, Chain, FIN,
                             MachinePointerInfo::getFixedStack(FI),
                             false, false, false, 0);
    }
    InVals.push_back(ArgValue);
  }

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
  MachineRegisterInfo &MRI = MF.getRegInfo();
  EVT PtrVT = getPointerTy();

  assert(!CLI.IsVarArg && "Var args not supported.");

  // Analyze the operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;

  CCState CCInfo(CLI.CallConv, CLI.IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeCallOperands(CLI.Outs, CC_65c816);

  // No support for tail calls
  CLI.IsTailCall = false;

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = CCInfo.getNextStackOffset();

  // Mark the start of the call.
  Chain = DAG.getCALLSEQ_START(Chain, DAG.getConstant(NumBytes, PtrVT, true),
                               DL);

  // Copy argument values to their designated locations.
  SmallVector<std::pair<unsigned, SDValue>, 3> RegsToPass;
  SmallVector<SDValue, 8> MemOpChains;
  SDValue StackPtr;
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    SDValue ArgValue = OutVals[i];

    if (VA.getLocInfo() == CCValAssign::Indirect) {
      // Store the argument in a stack slot and pass its address.
      SDValue SpillSlot = DAG.CreateStackTemporary(VA.getValVT());
      int FI = cast<FrameIndexSDNode>(SpillSlot)->getIndex();
      MemOpChains.push_back(DAG.getStore(Chain, DL, ArgValue, SpillSlot,
                                         MachinePointerInfo::getFixedStack(FI),
                                         false, false, 0));
      ArgValue = SpillSlot;
    }

    if (VA.isRegLoc()) {
      // Queue up the argument copies and emit them at the end.
      RegsToPass.push_back(std::make_pair(VA.getLocReg(), ArgValue));
    } else {
      assert(VA.isMemLoc() && "Argument not register nor memory");

      // Work out the address of the stack slot.
      if (!StackPtr.getNode()) {
        StackPtr = DAG.getCopyFromReg(Chain, DL, C65::S, getPointerTy());
      }
      SDValue Address = DAG.getNode(ISD::ADD, DL, PtrVT, StackPtr,
                                    DAG.getIntPtrConstant(VA.getLocMemOffset()));
      // Emit the store.
      MemOpChains.push_back(DAG.getStore(Chain, DL, ArgValue, Address,
                                         MachinePointerInfo(),
                                         false, false, 0));
    }
  }

  // Join the stores, which are independent of one another.
  if (!MemOpChains.empty()) {
    Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);
  }

  SDValue Glue;

  // // Accept direct calls by converting symbolic call addresses to the
  // // associated Target* opcodes.  Force %r1 to be used for indirect
  // // tail calls.
  // if (auto *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
  //   Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL, PtrVT);
  //   Callee = DAG.getNode(SystemZISD::PCREL_WRAPPER, DL, PtrVT, Callee);
  // } else if (auto *E = dyn_cast<ExternalSymbolSDNode>(Callee)) {
  //   Callee = DAG.getTargetExternalSymbol(E->getSymbol(), PtrVT);
  //   Callee = DAG.getNode(SystemZISD::PCREL_WRAPPER, DL, PtrVT, Callee);
  // } else if (IsTailCall) {
  //   Chain = DAG.getCopyToReg(Chain, DL, SystemZ::R1D, Callee, Glue);
  //   Glue = Chain.getValue(1);
  //   Callee = DAG.getRegister(SystemZ::R1D, Callee.getValueType());
  // }

  // Build a sequence of copy-to-reg nodes, chained and glued together.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Chain = DAG.getCopyToReg(Chain, DL, RegsToPass[i].first,
                             RegsToPass[i].second, Glue);
    Glue = Chain.getValue(1);
  }

  // The first call operand is the chain and the second is the target address.
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are
  // known live into the call.
  for (unsigned i = 0, e = RegsToPass.size(); i != e; ++i) {
    Ops.push_back(DAG.getRegister(RegsToPass[i].first,
                                  RegsToPass[i].second.getValueType()));
  }

  // TODO:T HIS
  // Add a register mask operand representing the call-preserved registers.
  //  const TargetRegisterInfo *TRI =
  //    getTargetMachine().getSubtargetImpl()->getRegisterInfo();
  //  const uint32_t *Mask = TRI->getCallPreservedMask(CallConv);
  //  assert(Mask && "Missing call preserved mask for calling convention");
  //  Ops.push_back(DAG.getRegisterMask(Mask));

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
                             DAG.getConstant(NumBytes, PtrVT, true),
                             DAG.getConstant(0, PtrVT, true),
                             Glue, DL);
  Glue = Chain.getValue(1);

  return Chain;

  // TODO: Return values!

  // // Assign locations to each value returned by this call.
  // SmallVector<CCValAssign, 16> RetLocs;
  // CCState RetCCInfo(CallConv, IsVarArg, MF, RetLocs, *DAG.getContext());
  // RetCCInfo.AnalyzeCallResult(Ins, RetCC_65C816);

  // // Copy all of the result registers out of their specified physreg.
  // for (unsigned i = 0, e = RetLocs.size(); i != e; ++i) {
  //   CCValAssign &VA = RetLocs[I];

  //   // Copy the value out, gluing the copy to the end of the call sequence.
  //   SDValue RetValue = DAG.getCopyFromReg(Chain, DL, VA.getLocReg(),
  //                                         VA.getLocVT(), Glue);
  //   Chain = RetValue.getValue(1);
  //   Glue = RetValue.getValue(2);

  //   // Convert the value of the return register into the value that's
  //   // being returned.
  //   InVals.push_back(convertLocVTToValVT(DAG, DL, VA, Chain, RetValue));
  // }
  // return DAG.getNode(C65ISD::CALL, DL, NodeTys, Ops);
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

  llvm_unreachable("Do not know how to custom type legalize this operation!");

  // SDLoc DL(N);

  // RTLIB::Libcall libCall = RTLIB::UNKNOWN_LIBCALL;

  // DEBUG(errs() << "Legalize operation " << N->getOpcode());
  // N->dump();

  // switch (N->getOpcode()) {
  // default:
  // }
}
