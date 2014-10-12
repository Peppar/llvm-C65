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
  // addRegisterClass(MVT::i16, &C65::PC_REGRegClass);
  // addRegisterClass(MVT::i8, &C65::BANK_REGRegClass);
  // addRegisterClass(MVT::i8, &C65::CCRRegClass);

  // Compute derived properties from the register classes
  computeRegisterProperties();

  // Copied from SystemZ
  setSchedulingPreference(Sched::RegPressure);


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

      // Lower SELECT_CC and BR_CC into separate comparisons and branches.
      setOperationAction(ISD::SELECT_CC, VT, Expand);

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

  // setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i32, Expand);
  // setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i16, Expand);
  // setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i8, Expand);
  // setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i1, Expand);
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i32, Expand);
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16, Expand);
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i8, Expand);
  // setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1, Expand);

  // Expand BRCOND into a BR_CC (see above).
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);

  // Truncate store are custom lowered
  // setTruncStoreAction(MVT::i16, MVT::i8,  Custom);
  // setTruncStoreAction(MVT::i32, MVT::i8 , Custom);
  // setTruncStoreAction(MVT::i32, MVT::i16, Custom);
  // setTruncStoreAction(MVT::i64, MVT::i8 , Custom);
  // setTruncStoreAction(MVT::i64, MVT::i16, Custom);
  // setTruncStoreAction(MVT::i64, MVT::i32, Custom);

  // Extending loads are custom lowered
  setLoadExtAction(ISD::EXTLOAD,  MVT::i1,  Promote);
  // setLoadExtAction(ISD::EXTLOAD,  MVT::i8,  Custom);
  // setLoadExtAction(ISD::EXTLOAD,  MVT::i16, Custom);
  // setLoadExtAction(ISD::EXTLOAD,  MVT::i32, Custom);
  setLoadExtAction(ISD::SEXTLOAD, MVT::i1,  Promote);
  // setLoadExtAction(ISD::SEXTLOAD, MVT::i8,  Custom);
  // setLoadExtAction(ISD::SEXTLOAD, MVT::i16, Custom);
  // setLoadExtAction(ISD::SEXTLOAD, MVT::i32, Custom);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i1,  Promote);
  // setLoadExtAction(ISD::ZEXTLOAD, MVT::i8,  Custom);
  // setLoadExtAction(ISD::ZEXTLOAD, MVT::i16, Custom);
  // setLoadExtAction(ISD::ZEXTLOAD, MVT::i32, Custom);

  setOperationAction(ISD::STACKSAVE,          MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE,       MVT::Other, Expand);

  // Custom legalize GlobalAddress nodes
  setOperationAction(ISD::GlobalAddress, getPointerTy(), Custom);

  setStackPointerRegisterToSaveRestore(C65::S);
}

/// This method returns the name of a target specific DAG node.
///
const char *C65TargetLowering::getTargetNodeName(unsigned Opcode) const {
  switch (Opcode) {
  default:
    return TargetLowering::getTargetNodeName(Opcode);
  case C65ISD::CMP:       return "C65ISD::CMP";
  case C65ISD::BR_CC:     return "C65ISD::BR_CC";
  case C65ISD::SELECT_CC: return "C65ISD::SELECT_CC";
  case C65ISD::CALL:      return "C65ISD::CALL";
  case C65ISD::RET:       return "C65ISD::RET";
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

SDValue C65TargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
  SDLoc DL(Op);
  SDValue Chain    = Op.getOperand(0);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue CmpOp0   = Op.getOperand(2);
  SDValue CmpOp1   = Op.getOperand(3);
  SDValue Dest     = Op.getOperand(4);

  return DAG.getNode(C65ISD::BR_CC, DL, Op.getValueType(),
                     Chain, DAG.getConstant(CC, MVT::i32),
                     CmpOp0, CmpOp1, Dest);
}

SDValue C65TargetLowering::LowerShift(SDValue Op, SelectionDAG &DAG) const {
  EVT VT = Op.getValueType();
  SDLoc DL(Op);

  // Emit a libcall.
  const char *LCName = 0;
  bool isSigned;
  if (Op.getOpcode() == ISD::SHL) {
    isSigned = false; /*sign irrelevant*/
    if (VT == MVT::i8)
      LCName = "c65_shl8";
    else if (VT == MVT::i16)
      LCName = "c65_shl16";
    else if (VT == MVT::i32)
      LCName = "c65_shl32";
    else if (VT == MVT::i64)
      LCName = "c65_shl64";
  } else if (Op.getOpcode() == ISD::SRL) {
    isSigned = false;
    if (VT == MVT::i8)
      LCName = "c65_lshr8";
    else if (VT == MVT::i16)
      LCName = "c65_lshr16";
    else if (VT == MVT::i32)
      LCName = "c65_lshr32";
    else if (VT == MVT::i64)
      LCName = "c65_lshr64";
  } else if (Op.getOpcode() == ISD::SRA) {
    isSigned = true;
    if (VT == MVT::i8)
      LCName = "c65_ashr8";
    if (VT == MVT::i16)
      LCName = "c65_ashr16";
    else if (VT == MVT::i32)
      LCName = "c65_ashr32";
    else if (VT == MVT::i64)
      LCName = "c65_ashr64";
  } else if (Op.getOpcode() == ISD::ROTL) {
    isSigned = false;
    if (VT == MVT::i8)
      LCName = "c65_rotl8";
    if (VT == MVT::i16)
      LCName = "c65_rotl16";
    else if (VT == MVT::i32)
      LCName = "c65_rotl32";
    else if (VT == MVT::i64)
      LCName = "c65_rotl64";
  } else {
    isSigned = false;
    if (VT == MVT::i8)
      LCName = "c65_rotr8";
    if (VT == MVT::i16)
      LCName = "c65_rotr16";
    else if (VT == MVT::i32)
      LCName = "c65_rotr32";
    else if (VT == MVT::i64)
      LCName = "c65_rotr64";
  }

  if (LCName) {
    SDValue Ops[2] = { Op.getOperand(0), Op.getOperand(1) };
    return makeC65LibCall(DAG, LCName, VT, Ops, 2, isSigned, DL).first;
  } else {
    llvm_unreachable("Unable to expand shift/rotate.");
  }
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
  case ISD::ROTR:
    return LowerShift(Op, DAG);
  case ISD::BR_CC:
    return LowerBR_CC(Op, DAG);
  case ISD::GlobalAddress:
    return LowerGlobalAddress(cast<GlobalAddressSDNode>(Op), DAG);
  }
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
static struct Comparison getComparison(const MachineInstr *MI) {
  ISD::CondCode CC = (ISD::CondCode)MI->getOperand(0).getImm();
  const MachineOperand Op0 = MI->getOperand(1);
  const MachineOperand Op1 = MI->getOperand(2);

  switch (CC) { //           Op0  Op1  Equality Signed Bitvalue
  case ISD::SETEQ:  return { Op0, Op1, true,    false, true };
  case ISD::SETNE:  return { Op0, Op1, true,    false, false };
  case ISD::SETLT:  return { Op0, Op1, false,   true,  true };
  case ISD::SETLE:  return { Op1, Op0, false,   true,  false };
  case ISD::SETGT:  return { Op1, Op0, false,   true,  true };
  case ISD::SETGE:  return { Op0, Op1, false,   true,  false };
  case ISD::SETULT: return { Op0, Op1, false,   false, true };
  case ISD::SETULE: return { Op1, Op0, false,   false, false };
  case ISD::SETUGT: return { Op1, Op0, false,   false, true };
  case ISD::SETUGE: return { Op0, Op1, false,   false, false };
  default:
    llvm_unreachable("Cannot emit this type of comparison!");
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZBRCC(MachineInstr *MI,
                             MachineBasicBlock *MBB,
                             unsigned NumBytes) const {

  bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  unsigned AccSize = Use8Bit ? 1 : 2;

  DebugLoc DL = MI->getDebugLoc();
  MachineFunction *MF = MBB->getParent();
  //  MachineRegisterInfo &MRI = MF->getRegInfo();

  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator MFI = MBB;
  ++MFI;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  struct Comparison C = getComparison(MI);
  MachineBasicBlock *Dest = MI->getOperand(3).getMBB();

  if (C.Equality) {
    // thisMBB:
    // cmpMBB0:
    //   LDA %ZRA
    //   CMP %ZRB
    //   BNE sinkMBB/Dest
    // cmpMBB1:
    //   LDA %ZRA+2
    //   CMP %ZRB+2
    //   BNE sinkMBB/Dest
    //   ...
    //   LDA %ZRA+N
    //   CMP %ZRB+N
    //   BEQ/BNE Dest
    // sinkMBB:

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
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
          BuildMI(cmpMBB, DL, TII->get(C65::BEQ)).addMBB(Dest);
          cmpMBB->addSuccessor(Dest);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(Dest);
          cmpMBB->addSuccessor(Dest);
        }
      } else {
        if (C.Bitvalue) {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(sinkMBB);
          cmpMBB->addSuccessor(sinkMBB);
        } else {
          BuildMI(cmpMBB, DL, TII->get(C65::BNE)).addMBB(Dest);
          cmpMBB->addSuccessor(Dest);
        }
      }
      predMBB = cmpMBB;
    }
    MF->insert(MFI, sinkMBB);
    predMBB->addSuccessor(sinkMBB);

    MI->eraseFromParent();
    return sinkMBB;
  } else if (C.Signed) {
    // thisMBB:
    //   LDA %ZRA
    //   CMP %ZRB
    //   LDA %ZRA+2
    //   SBC %ZRB+2
    //   ...
    //   BVC braMBB
    // ovfMBB:
    //   EOR #$8000
    // braMBB:
    //   BPL/BMI %DEST
    // sinkMBB:

    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *ovfMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *braMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, ovfMBB);
    MF->insert(MFI, braMBB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(braMBB);
    thisMBB->addSuccessor(ovfMBB);
    ovfMBB->addSuccessor(braMBB);
    braMBB->addSuccessor(Dest);
    braMBB->addSuccessor(sinkMBB);

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

    BuildMI(thisMBB, DL, TII->get(C65::BVC))
      .addMBB(braMBB);
    if (Use8Bit) {
      BuildMI(*ovfMBB, ovfMBB->begin(), DL, TII->get(C65::EOR8imm))
        .addImm(0x80);
    } else {
      BuildMI(*ovfMBB, ovfMBB->begin(), DL, TII->get(C65::EOR16imm))
        .addImm(0x8000);
    }

    if (C.Bitvalue) {
      BuildMI(*braMBB, braMBB->begin(), DL, TII->get(C65::BMI)).addMBB(Dest);
    } else {
      BuildMI(*braMBB, braMBB->begin(), DL, TII->get(C65::BPL)).addMBB(Dest);
    }

    MI->eraseFromParent();
    return sinkMBB;
  } else {
    // thisMBB:
    //   LDA %ZRA
    //   CMP %ZRB
    //   LDA %ZRA+2
    //   SBC %ZRB+2
    //   ...
    //   BCS/BCC Dest
    // sinkMBB:
    const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP8zp : C65::CMP16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC8zp : C65::SBC16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(sinkMBB);
    thisMBB->addSuccessor(Dest);

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
      BuildMI(thisMBB, DL, TII->get(C65::BCS)).addMBB(Dest);
    } else {
      BuildMI(thisMBB, DL, TII->get(C65::BCC)).addMBB(Dest);
    }

    MI->eraseFromParent();

    return sinkMBB;
  }
}

MachineBasicBlock *
C65TargetLowering::EmitZST(MachineInstr *MI,
                           MachineBasicBlock *MBB,
                           bool Stack,
                           unsigned NumBytes) const {
  bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  unsigned AccSize = Use8Bit ? 1 : 2;

  unsigned NumOperands = MI->getNumOperands();
  const MachineOperand *Src = &MI->getOperand(0);
  const MachineOperand *Op1 = &MI->getOperand(1);
  const MachineOperand *Op2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
  unsigned STAInstr;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  bool YIndirect;

  if (NumOperands == 2) {
    assert(!Op1->isReg());

    if (Stack)
      STAInstr = Use8Bit ? C65::STA8srel  : C65::STA16srel;
    else
      STAInstr = Use8Bit ? C65::STA8abs  : C65::STA16abs;
    YIndirect = false;
  } else /* NumOperands == 3 */ {
    assert(Op1->isReg());
    Op2 = &MI->getOperand(2);

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
                           unsigned ExtendBegin,
                           bool Signed) const {
  bool Use8Bit = ExtendBegin == 1 || !Subtarget->has65802();
  unsigned AccSize = Use8Bit ? 1 : 2;

  unsigned NumOperands = MI->getNumOperands();
  const MachineOperand *Dest = &MI->getOperand(0);
  const MachineOperand *Op1 = &MI->getOperand(1);
  const MachineOperand *Op2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  unsigned STAInstr = Use8Bit ? C65::STA8zp : C65::STA16zp;
  unsigned STZInstr = Use8Bit ? C65::STZ8zp  : C65::STZ16zp;
  unsigned LDAInstr;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  bool YIndirect;

  if (NumOperands == 2) {
    assert(!Op1->isReg());
    if (Stack)
      LDAInstr = Use8Bit ? C65::LDA8srel : C65::LDA16srel;
    else
      LDAInstr = Use8Bit ? C65::LDA8abs  : C65::LDA16abs;
    YIndirect = false;
  } else /* NumOperands == 3 */ {
    assert(Op1->isReg());
    Op2 = &MI->getOperand(2);

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
    MachineFunction *MF = MBB->getParent();
    const BasicBlock *BB = MBB->getBasicBlock();
    MachineFunction::iterator MFI = MBB;
    ++MFI;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sextMBB = MF->CreateMachineBasicBlock(BB);
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
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
      BuildMI(thisMBB, DL, TII->get(STZInstr))
        .addImm(RI->getZRAddress(Dest->getReg()) + I);
    }
    BuildMI(thisMBB, DL, TII->get(Subtarget->has65C02() ?
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

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA8imm : C65::LDA16imm;
  const unsigned STAInstr = Use8Bit ? C65::STA8zp  : C65::STA16zp;
  const unsigned STZInstr = Use8Bit ? C65::STZ8zp  : C65::STZ16zp;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    unsigned Value;
    if (Use8Bit) {
      Value = MI->getOperand(1).getImm() >> (8 * I) & 0xFF;
    } else {
      Value = MI->getOperand(1).getImm() >> (8 * I) & 0xFFFF;
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
                             unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->has65816();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA8zp : C65::LDA16zp;
  const unsigned STAInstr = Use8Bit ? C65::STA8zp : C65::STA16zp;
  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(1).getReg()) + I);
    BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitZInstr(MachineInstr *MI, MachineBasicBlock *MBB) const {
  unsigned OpSize = 1 << C65II::getZROpSize(MI->getDesc().TSFlags);
  switch (MI->getOpcode()) {
  default: llvm_unreachable("unknown Z instruction to emit");
    // BR_CC
  case C65::ZBRCC8:
  case C65::ZBRCC16:
  case C65::ZBRCC32:
  case C65::ZBRCC64:
    return EmitZBRCC(MI, MBB, OpSize);

    // ZST
  case C65::ZST8s:
  case C65::ZST16s:
  case C65::ZST32s:
  case C65::ZST64s:
    return EmitZST(MI, MBB, true, OpSize);
  case C65::ZST8zp:
  case C65::ZST16zp:
  case C65::ZST32zp:
  case C65::ZST64zp:
  case C65::ZST8i16:
  case C65::ZST16i16:
  case C65::ZST32i16:
  case C65::ZST64i16:
  case C65::ZST8iz16:
  case C65::ZST16iz16:
  case C65::ZST32iz16:
  case C65::ZST64iz16:
  case C65::ZST8zz16:
  case C65::ZST16zz16:
  case C65::ZST32zz16:
  case C65::ZST64zz16:
    return EmitZST(MI, MBB, false, OpSize);

    // ZST truncating
  case C65::ZST16trunc8s:
  case C65::ZST32trunc8s:
  case C65::ZST64trunc8s:
    return EmitZST(MI, MBB, true, 1);
  case C65::ZST16trunc8zp:
  case C65::ZST16trunc8i16:
  case C65::ZST16trunc8iz16:
  case C65::ZST16trunc8zz16:
  case C65::ZST32trunc8zp:
  case C65::ZST32trunc8i16:
  case C65::ZST32trunc8iz16:
  case C65::ZST32trunc8zz16:
  case C65::ZST64trunc8zp:
  case C65::ZST64trunc8i16:
  case C65::ZST64trunc8iz16:
  case C65::ZST64trunc8zz16:
    return EmitZST(MI, MBB, false, 1);
  case C65::ZST32trunc16s:
  case C65::ZST64trunc16s:
    return EmitZST(MI, MBB, true, 2);
  case C65::ZST32trunc16zp:
  case C65::ZST32trunc16i16:
  case C65::ZST32trunc16iz16:
  case C65::ZST32trunc16zz16:
  case C65::ZST64trunc16zp:
  case C65::ZST64trunc16i16:
  case C65::ZST64trunc16iz16:
  case C65::ZST64trunc16zz16:
    return EmitZST(MI, MBB, false, 2);
  case C65::ZST64trunc32s:
    return EmitZST(MI, MBB, true, 4);
  case C65::ZST64trunc32zp:
  case C65::ZST64trunc32i16:
  case C65::ZST64trunc32iz16:
  case C65::ZST64trunc32zz16:
    return EmitZST(MI, MBB, false, 4);

    // ZLD
  case C65::ZLD8s:
  case C65::ZLD16s:
  case C65::ZLD32s:
  case C65::ZLD64s:
    return EmitZLD(MI, MBB, true, OpSize, OpSize);
  case C65::ZLD8zp:
  case C65::ZLD16zp:
  case C65::ZLD32zp:
  case C65::ZLD64zp:
  case C65::ZLD8i16:
  case C65::ZLD16i16:
  case C65::ZLD32i16:
  case C65::ZLD64i16:
  case C65::ZLD8iz16:
  case C65::ZLD16iz16:
  case C65::ZLD32iz16:
  case C65::ZLD64iz16:
  case C65::ZLD8zz16:
  case C65::ZLD16zz16:
  case C65::ZLD32zz16:
  case C65::ZLD64zz16:
    return EmitZLD(MI, MBB, false, OpSize, OpSize);

    // ZLD any extend
  case C65::ZLD16ext8s:
  case C65::ZLD32ext8s:
  case C65::ZLD64ext8s:
    return EmitZLD(MI, MBB, true, 1, 1);
  case C65::ZLD16ext8zp:
  case C65::ZLD16ext8i16:
  case C65::ZLD16ext8iz16:
  case C65::ZLD16ext8zz16:
  case C65::ZLD32ext8zp:
  case C65::ZLD32ext8i16:
  case C65::ZLD32ext8iz16:
  case C65::ZLD32ext8zz16:
  case C65::ZLD64ext8zp:
  case C65::ZLD64ext8i16:
  case C65::ZLD64ext8iz16:
  case C65::ZLD64ext8zz16:
    return EmitZLD(MI, MBB, false, 1, 1);
  case C65::ZLD32ext16s:
  case C65::ZLD64ext16s:
    return EmitZLD(MI, MBB, true, 2, 2);
  case C65::ZLD32ext16zp:
  case C65::ZLD32ext16i16:
  case C65::ZLD32ext16iz16:
  case C65::ZLD32ext16zz16:
  case C65::ZLD64ext16zp:
  case C65::ZLD64ext16i16:
  case C65::ZLD64ext16iz16:
  case C65::ZLD64ext16zz16:
    return EmitZLD(MI, MBB, false, 2, 2);
  case C65::ZLD64ext32s:
    return EmitZLD(MI, MBB, true, 4, 4);
  case C65::ZLD64ext32zp:
  case C65::ZLD64ext32i16:
  case C65::ZLD64ext32iz16:
  case C65::ZLD64ext32zz16:
    return EmitZLD(MI, MBB, false, 4, 4);

    // ZLD sign extend
  case C65::ZLD16sext8s:
  case C65::ZLD32sext8s:
  case C65::ZLD64sext8s:
    return EmitZLD(MI, MBB, true, OpSize, 1, true);
  case C65::ZLD16sext8zp:
  case C65::ZLD16sext8i16:
  case C65::ZLD16sext8iz16:
  case C65::ZLD16sext8zz16:
  case C65::ZLD32sext8zp:
  case C65::ZLD32sext8i16:
  case C65::ZLD32sext8iz16:
  case C65::ZLD32sext8zz16:
  case C65::ZLD64sext8zp:
  case C65::ZLD64sext8i16:
  case C65::ZLD64sext8iz16:
  case C65::ZLD64sext8zz16:
    return EmitZLD(MI, MBB, false, OpSize, 1, true);
  case C65::ZLD32sext16s:
  case C65::ZLD64sext16s:
    return EmitZLD(MI, MBB, true, OpSize, 2, true);
  case C65::ZLD32sext16zp:
  case C65::ZLD32sext16i16:
  case C65::ZLD32sext16iz16:
  case C65::ZLD32sext16zz16:
  case C65::ZLD64sext16zp:
  case C65::ZLD64sext16i16:
  case C65::ZLD64sext16iz16:
  case C65::ZLD64sext16zz16:
    return EmitZLD(MI, MBB, false, OpSize, 2, true);
  case C65::ZLD64sext32s:
    return EmitZLD(MI, MBB, true, OpSize, 4, true);
  case C65::ZLD64sext32zp:
  case C65::ZLD64sext32i16:
  case C65::ZLD64sext32iz16:
  case C65::ZLD64sext32zz16:
    return EmitZLD(MI, MBB, false, OpSize, 4, true);

    // ZLD zero extend
  case C65::ZLD16zext8s:
  case C65::ZLD32zext8s:
  case C65::ZLD64zext8s:
    return EmitZLD(MI, MBB, true, OpSize, 1, false);
  case C65::ZLD16zext8zp:
  case C65::ZLD16zext8i16:
  case C65::ZLD16zext8iz16:
  case C65::ZLD16zext8zz16:
  case C65::ZLD32zext8zp:
  case C65::ZLD32zext8i16:
  case C65::ZLD32zext8iz16:
  case C65::ZLD32zext8zz16:
  case C65::ZLD64zext8zp:
  case C65::ZLD64zext8i16:
  case C65::ZLD64zext8iz16:
  case C65::ZLD64zext8zz16:
    return EmitZLD(MI, MBB, false, OpSize, 1, false);
  case C65::ZLD32zext16s:
  case C65::ZLD64zext16s:
    return EmitZLD(MI, MBB, true, OpSize, 2, false);
  case C65::ZLD32zext16zp:
  case C65::ZLD32zext16i16:
  case C65::ZLD32zext16iz16:
  case C65::ZLD32zext16zz16:
  case C65::ZLD64zext16zp:
  case C65::ZLD64zext16i16:
  case C65::ZLD64zext16iz16:
  case C65::ZLD64zext16zz16:
    return EmitZLD(MI, MBB, false, OpSize, 2, false);
  case C65::ZLD64zext32s:
    return EmitZLD(MI, MBB, true, OpSize, 4, false);
  case C65::ZLD64zext32zp:
  case C65::ZLD64zext32i16:
  case C65::ZLD64zext32iz16:
  case C65::ZLD64zext32zz16:
    return EmitZLD(MI, MBB, false, OpSize, 4, false);

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
    return EmitZMOV(MI, MBB, OpSize);

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
    return EmitBinaryZI(MI, MBB, OpSize, C65::ADC8zp, C65::ADC16zp,
                        false, true);
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

  CCInfo.AnalyzeFormalArguments(Ins, CC_C65);

  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    SDValue ArgValue;
    CCValAssign &VA = ArgLocs[i];
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

  assert(!IsVarArg && "Var args not supported.");

  // Analyze the operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;

  CCState CCInfo(CallConv, IsVarArg, DAG.getMachineFunction(),
                 ArgLocs, *DAG.getContext());
  CCInfo.AnalyzeCallOperands(Outs, CC_C65);

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

  // Accept direct calls by converting symbolic call addresses to the
  // associated Target* opcodes.
  if (auto *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    Callee = DAG.getTargetGlobalAddress(G->getGlobal(), DL, PtrVT);
  } else if (auto *E = dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(E->getSymbol(), PtrVT);
  }

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

  // TODO: THIS
  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI =
    getTargetMachine().getSubtargetImpl()->getRegisterInfo();
  const uint32_t *Mask = TRI->getCallPreservedMask(CallConv);
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
                             DAG.getConstant(NumBytes, PtrVT, true),
                             DAG.getConstant(0, PtrVT, true),
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

// SDValue
// SparcTargetLowering::LowerF128_LibCallArg(SDValue Chain, ArgListTy &Args,
//                                           SDValue Arg, SDLoc DL,
//                                           SelectionDAG &DAG) const {
//   MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();
//   EVT ArgVT = Arg.getValueType();
//   Type *ArgTy = ArgVT.getTypeForEVT(*DAG.getContext());

//   ArgListEntry Entry;
//   Entry.Node = Arg;
//   Entry.Ty   = ArgTy;

//   if (ArgTy->isFP128Ty()) {
//     // Create a stack object and pass the pointer to the library function.
//     int FI = MFI->CreateStackObject(16, 8, false);
//     SDValue FIPtr = DAG.getFrameIndex(FI, getPointerTy());
//     Chain = DAG.getStore(Chain,
//                          DL,
//                          Entry.Node,
//                          FIPtr,
//                          MachinePointerInfo(),
//                          false,
//                          false,
//                          8);

//     Entry.Node = FIPtr;
//     Entry.Ty   = PointerType::getUnqual(ArgTy);
//   }
//   Args.push_back(Entry);
//   return Chain;
// }

// SDValue
// C65TargetLowering::LowerLibCall(SDValue Op, SelectionDAG &DAG,
//                                 const char *LibFuncName,
//                                 unsigned numArgs) const {
//   ArgListTy Args;

//   MachineFrameInfo *MFI = DAG.getMachineFunction().getFrameInfo();

//   SDValue Callee = DAG.getExternalSymbol(LibFuncName, getPointerTy());
//   Type *RetTy = Op.getValueType().getTypeForEVT(*DAG.getContext());
//   Type *RetTyABI = RetTy;
//   SDValue Chain = DAG.getEntryNode();
//   SDValue RetPtr;

//   // if (RetTy->isFP128Ty()) {
//   //   // Create a Stack Object to receive the return value of type f128.
//   //   ArgListEntry Entry;
//   //   int RetFI = MFI->CreateStackObject(16, 8, false);
//   //   RetPtr = DAG.getFrameIndex(RetFI, getPointerTy());
//   //   Entry.Node = RetPtr;
//   //   Entry.Ty   = PointerType::getUnqual(RetTy);
//   //   if (!Subtarget->is64Bit())
//   //     Entry.isSRet = true;
//   //   Entry.isReturned = false;
//   //   Args.push_back(Entry);
//   //   RetTyABI = Type::getVoidTy(*DAG.getContext());
//   // }

//   assert(Op->getNumOperands() >= numArgs && "Not enough operands!");
//   for (unsigned i = 0, e = numArgs; i != e; ++i) {
//     Chain = LowerF128_LibCallArg(Chain, Args, Op.getOperand(i), SDLoc(Op), DAG);
//   }
//   TargetLowering::CallLoweringInfo CLI(DAG);
//   CLI.setDebugLoc(SDLoc(Op)).setChain(Chain)
//     .setCallee(CallingConv::C, RetTyABI, Callee, std::move(Args), 0);

//   std::pair<SDValue, SDValue> CallInfo = LowerCallTo(CLI);

//   // chain is in second result.
//   if (RetTyABI == RetTy)
//     return CallInfo.first;

//   assert (RetTy->isFP128Ty() && "Unexpected return type!");

//   Chain = CallInfo.second;

//   // Load RetPtr to get the return value.
//   return DAG.getLoad(Op.getValueType(),
//                      SDLoc(Op),
//                      Chain,
//                      RetPtr,
//                      MachinePointerInfo(),
//                      false, false, false, 8);
// }


// SDValue C65::LowerShiftRotate(SDValue Op, SelectionDAG &DAG,
//                               const C65TargetLowering &TLI) {
//   EVT *VT = Op.getValueType();
//   SDLoc DL(Op);

//   RTLIB::Libcall LC = RTLIB::UNKNOWN_LIBCALL;
//   if (VT == MVT::i16)
//     LC = RTLIB::MUL_I16;
//   else if (VT == MVT::i32)
//     LC = RTLIB::MUL_I32;
//   else if (VT == MVT::i64)
//     LC = RTLIB::MUL_I64;
//   else if (VT == MVT::i128)
//     LC = RTLIB::MUL_I128;
//   assert(LC != RTLIB::UNKNOWN_LIBCALL && "Unsupported MUL!");

//   unsigned opcode = Op.getOpcode();
// //  assert((opcode == ISD::UMULO || opcode == ISD::SMULO) && "Invalid Opcode.");

// //  bool isSigned = (opcode == ISD::SMULO);
//   EVT VT = MVT::i64;
//   EVT WideVT = MVT::i128;
//   SDLoc DL(Op);
//   SDValue LHS = Op.getOperand(0);

//   if (LHS.getValueType() != VT)
//     return Op;

//   SDValue ShiftAmt = DAG.getConstant(63, VT);

//   SDValue RHS = Op.getOperand(1);
//   SDValue Args[] = { Op.getOperand(1), Op.getOperand(2) };
//   return makeLibCall(DAG, 
//   //HiLHS, LHS, HiRHS, RHS };
//   // SDValue MulResult = makeC65LibCall(Op, DAG,
//   //                                    "__CALL_HEJ",
//   //                                    Args, 4, isSigned).first;
// }



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
  for (unsigned i = 0; i != NumOps; ++i) {
    Entry.Node = Ops[i];
    Entry.Ty = Entry.Node.getValueType().getTypeForEVT(*DAG.getContext());
    Entry.isSExt = isSigned;
    Entry.isZExt = !isSigned;
    Args.push_back(Entry);
  }
  SDValue Callee = DAG.getExternalSymbol(LCName, getPointerTy());

  Type *RetTy = RetVT.getTypeForEVT(*DAG.getContext());
  TargetLowering::CallLoweringInfo CLI(DAG);
  CLI.setDebugLoc(DL).setChain(DAG.getEntryNode())
    .setCallee(CallingConv::C, RetTy, Callee, std::move(Args), 0)
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

  DEBUG(errs() << "Legalize operation " << N->getOpcode());
  N->dump();

  switch (N->getOpcode()) {
  default:
    llvm_unreachable("Do not know how to custom type legalize this operation!");

  }
}
