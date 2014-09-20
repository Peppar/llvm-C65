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
    }
  }

  // Expand BRCOND into a BR_CC (see above).
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);

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

SDValue C65TargetLowering::
lowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
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

/// This callback is invoked for operations that are unsupported by
/// the target, which are registered to use 'custom' lowering, and
/// whose defined values are all legal. If the target has no
/// operations that require custom lowering, it need not implement
/// this.  The default implementation of this aborts.
///
SDValue C65TargetLowering::
LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch(Op.getOpcode()) {
  default:
    llvm_unreachable("Unexpected node to lower");
  case ISD::BR_CC:
    return lowerBR_CC(Op, DAG);
  case ISD::GlobalAddress:
    return LowerGlobalAddress(cast<GlobalAddressSDNode>(Op), DAG);
  }
}

// MCInstrDesc C65TargetLowering::getOp(unsigned Op) const {
//   const TargetInstrInfo &TII = *Subtarget->getInstrInfo();
//   return TII.get(Op);
// }

// uint8_t C65TargetLowering::getZRAddress(MachineOperand Val) const {
//   const C65RegisterInfo &TRI =
//       *static_cast<const C65RegisterInfo *>(Subtarget->getRegisterInfo());
//   return TRI.getZRAddress(Val.getReg());
// }

/// Get the operand size of the function. This should be replaced by a
/// more declarative approach.
///
// static unsigned getOpByteSize(unsigned Op) {
//   switch (Op) {
//   default: llvm_unreachable("Unknown opcode!");
//     return 8;
//   }
// }

// unsigned static getInstr(unsigned InstrClass, unsigned AccSize,
//                          unsigned RegSize) {
//   static const unsigned InstructionMatrix[21][7] = {
//     { C65::ORA_8zr8,   C65::ORA_8zr16,  C65::ORA_8zr32, C65::ORA_8zr64,
//       C65::ORA_16zr16, C65::ORA_16zr32, C65::ORA_16zr64 },
//     { C65::AND_8zr8,   C65::AND_8zr16,  C65::AND_8zr32, C65::AND_8zr64,
//       C65::AND_16zr16, C65::AND_16zr32, C65::AND_16zr64 },
//     { C65::EOR_8zr8,   C65::EOR_8zr16,  C65::EOR_8zr32, C65::EOR_8zr64,
//       C65::EOR_16zr16, C65::EOR_16zr32, C65::EOR_16zr64 },
//     { C65::ADC_8zr8,   C65::ADC_8zr16,  C65::ADC_8zr32, C65::ADC_8zr64,
//       C65::ADC_16zr16, C65::ADC_16zr32, C65::ADC_16zr64 },
//     { C65::SBC_8zr8,   C65::SBC_8zr16,  C65::SBC_8zr32, C65::SBC_8zr64,
//       C65::SBC_16zr16, C65::SBC_16zr32, C65::SBC_16zr64 },
//     { C65::STA_8zr8,   C65::STA_8zr16,  C65::STA_8zr32, C65::STA_8zr64,
//       C65::STA_16zr16, C65::STA_16zr32, C65::STA_16zr64 },
//     { C65::CMP_8zr8,   C65::CMP_8zr16,  C65::CMP_8zr32, C65::CMP_8zr64,
//       C65::CMP_16zr16, C65::CMP_16zr32, C65::CMP_16zr64 },
//     { C65::LDA_8zr8,   C65::LDA_8zr16,  C65::LDA_8zr32, C65::LDA_8zr64,
//       C65::LDA_16zr16, C65::LDA_16zr32, C65::LDA_16zr64 },
//     { C65::ASL_8zr8,   C65::ASL_8zr16,  C65::ASL_8zr32, C65::ASL_8zr64,
//       C65::ASL_16zr16, C65::ASL_16zr32, C65::ASL_16zr64 },
//     { C65::ROL_8zr8,   C65::ROL_8zr16,  C65::ROL_8zr32, C65::ROL_8zr64,
//       C65::ROL_16zr16, C65::ROL_16zr32, C65::ROL_16zr64 },
//     { C65::LSR_8zr8,   C65::LSR_8zr16,  C65::LSR_8zr32, C65::LSR_8zr64,
//       C65::LSR_16zr16, C65::LSR_16zr32, C65::LSR_16zr64 },
//     { C65::ROR_8zr8,   C65::ROR_8zr16,  C65::ROR_8zr32, C65::ROR_8zr64,
//       C65::ROR_16zr16, C65::ROR_16zr32, C65::ROR_16zr64 },
//     { C65::DEC_8zr8,   C65::DEC_8zr16,  C65::DEC_8zr32, C65::DEC_8zr64,
//       C65::DEC_16zr16, C65::DEC_16zr32, C65::DEC_16zr64 },
//     { C65::INC_8zr8,   C65::INC_8zr16,  C65::INC_8zr32, C65::INC_8zr64,
//       C65::INC_16zr16, C65::INC_16zr32, C65::INC_16zr64 },
//     { C65::STX_8zr8,   C65::STX_8zr16,  C65::STX_8zr32, C65::STX_8zr64,
//       C65::STX_16zr16, C65::STX_16zr32, C65::STX_16zr64 },
//     { C65::LDX_8zr8,   C65::LDX_8zr16,  C65::LDX_8zr32, C65::LDX_8zr64,
//       C65::LDX_16zr16, C65::LDX_16zr32, C65::LDX_16zr64 },
//     { C65::STY_8zr8,   C65::STY_8zr16,  C65::STY_8zr32, C65::STY_8zr64,
//       C65::STY_16zr16, C65::STY_16zr32, C65::STY_16zr64 },
//     { C65::LDY_8zr8,   C65::LDY_8zr16,  C65::LDY_8zr32, C65::LDY_8zr64,
//       C65::LDY_16zr16, C65::LDY_16zr32, C65::LDY_16zr64 },
//     { C65::CPY_8zr8,   C65::CPY_8zr16,  C65::CPY_8zr32, C65::CPY_8zr64,
//       C65::CPY_16zr16, C65::CPY_16zr32, C65::CPY_16zr64 },
//     { C65::CPX_8zr8,   C65::CPX_8zr16,  C65::CPX_8zr32, C65::CPX_8zr64,
//       C65::CPX_16zr16, C65::CPX_16zr32, C65::CPX_16zr64 },
//     { C65::STZ_8zr8,   C65::STZ_8zr16,  C65::STZ_8zr32, C65::STZ_8zr64,
//       C65::STZ_16zr16, C65::STZ_16zr32, C65::STZ_16zr64 }
//   };
//   unsigned Idx;

//   assert(InstrClass < C65IC::INSTR_CLASS_END && "InstrClass invalid");

//   if (AccSize == 1 && RegSize == 1)      Idx = 0;
//   else if (AccSize == 1 && RegSize == 2) Idx = 1;
//   else if (AccSize == 1 && RegSize == 4) Idx = 2;
//   else if (AccSize == 1 && RegSize == 8) Idx = 3;
//   else if (AccSize == 2 && RegSize == 2) Idx = 4;
//   else if (AccSize == 2 && RegSize == 4) Idx = 5;
//   else if (AccSize == 2 && RegSize == 8) Idx = 6;
//   else {
//     llvm_unreachable("Impossible accumulator and register size combination.");
//   }
//   return InstructionMatrix[InstrClass][Idx];
// }

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
  case ISD::SETLT:  return { Op1, Op0, false,   true,  false };
  case ISD::SETLE:  return { Op0, Op1, false,   true,  true };
  case ISD::SETGT:  return { Op0, Op1, false,   true,  false };
  case ISD::SETGE:  return { Op1, Op0, false,   true,  true };
  case ISD::SETULT: return { Op0, Op1, false,   false, true };
  case ISD::SETULE: return { Op1, Op0, false,   false, false };
  case ISD::SETUGT: return { Op1, Op0, false,   false, true };
  case ISD::SETUGE: return { Op0, Op1, false,   false, false };
  default:
    llvm_unreachable("Cannot emit this type of comparison!");
  }
}

MachineBasicBlock *
C65TargetLowering::EmitBR_CC(MachineInstr *MI,
                             MachineBasicBlock *MBB,
                             unsigned NumBytes) const {

  bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  unsigned AccSize = Use8Bit ? 1 : 2;

  DebugLoc DL = MI->getDebugLoc();
  MachineFunction *MF = MBB->getParent();
  MachineRegisterInfo &MRI = MF->getRegInfo();

  const BasicBlock *BB = MBB->getBasicBlock();
  MachineFunction::iterator MFI = MBB;
  ++MFI;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();

  struct Comparison C = getComparison(MI);
  MachineBasicBlock *Dest = MI->getOperand(3).getMBB();

  if (C.Equality) {
    // thisMBB:
    //   LDA %ZRA
    //   CMP %ZRB
    //   BNE sinkMBB/Dest
    //   LDA %ZRA+2
    //   CMP %ZRB+2
    //   BNE sinkMBB/Dest
    //   ...
    //   LDA %ZRA+N
    //   CMP %ZRB+N
    //   BEQ/BNE Dest
    // sinkMBB:

    const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP_8zp : C65::CMP_16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(sinkMBB);

    for (unsigned I = 0; I < NumBytes; I += AccSize) {
      BuildMI(thisMBB, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(thisMBB, DL, TII->get(CMPInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
      if (I == NumBytes - AccSize) {
        if (C.Bitvalue) {
          BuildMI(thisMBB, DL, TII->get(C65::BEQ)).addMBB(Dest);
        } else {
          BuildMI(thisMBB, DL, TII->get(C65::BNE)).addMBB(Dest);
        }
      } else {
        if (C.Bitvalue) {
          BuildMI(thisMBB, DL, TII->get(C65::BNE)).addMBB(sinkMBB);
        } else {
          BuildMI(thisMBB, DL, TII->get(C65::BNE)).addMBB(Dest);
        }
      }
    }

    thisMBB->addSuccessor(sinkMBB);

    MI->eraseFromParent();

    return sinkMBB;
  } else if (C.Signed) {
    // thisMBB:
    //   LDA %ZRA
    //   CMP %ZRB
    //   LDA %ZRA+2
    //   SBC %ZRB+2
    //   ...
    //   BVC sinkMBB
    //   EOR #$8000
    // sinkMBB:
    //   BPL/BMI %DEST

    const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP_8zp : C65::CMP_16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC_8zp : C65::SBC_16zp;

    MachineBasicBlock *thisMBB = MBB;
    MachineBasicBlock *sinkMBB = MF->CreateMachineBasicBlock(BB);
    MF->insert(MFI, sinkMBB);

    // Transfer the remainder of the MBB and its successor edges to sinkMBB.
    sinkMBB->splice(sinkMBB->begin(), MBB,
                    std::next(MachineBasicBlock::iterator(MI)), MBB->end());
    sinkMBB->transferSuccessorsAndUpdatePHIs(MBB);
    thisMBB->addSuccessor(sinkMBB);

    if (C.Bitvalue) {
      BuildMI(*sinkMBB, sinkMBB->begin(), DL, TII->get(C65::BMI)).addMBB(Dest);
    } else {
      BuildMI(*sinkMBB, sinkMBB->begin(), DL, TII->get(C65::BPL)).addMBB(Dest);
    }

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
      .addMBB(sinkMBB);
    if (Use8Bit) {
      BuildMI(thisMBB, DL, TII->get(C65::EOR_8imm))
        .addImm(0x80);
    } else {
      BuildMI(thisMBB, DL, TII->get(C65::EOR_16imm))
        .addImm(0x8000);
    }

    thisMBB->addSuccessor(sinkMBB);

    MI->eraseFromParent();

    return sinkMBB;
  } else {
    //   LDA %ZRA
    //   CMP %ZRB
    //   LDA %ZRA+2
    //   SBC %ZRB+2
    //   ...
    //   BCS/BCC Dest
    const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
    const unsigned CMPInstr = Use8Bit ? C65::CMP_8zp : C65::CMP_16zp;
    const unsigned SBCInstr = Use8Bit ? C65::SBC_8zp : C65::SBC_16zp;

    MachineBasicBlock::iterator MBBI = MI;

    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(C.Op0.getReg()));
    BuildMI(*MBB, MBBI, DL, TII->get(CMPInstr))
      .addImm(RI->getZRAddress(C.Op1.getReg()));

    for (unsigned I = AccSize; I < NumBytes; I += AccSize) {
      BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
        .addImm(RI->getZRAddress(C.Op0.getReg()) + I);
      BuildMI(*MBB, MBBI, DL, TII->get(SBCInstr))
        .addImm(RI->getZRAddress(C.Op1.getReg()) + I);
    }
    if (C.Bitvalue) {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::BCS)).addMBB(Dest);
    } else {
      BuildMI(*MBB, MBBI, DL, TII->get(C65::BCC)).addMBB(Dest);
    }

    MI->eraseFromParent();

    return MBB;
  }
}

MachineBasicBlock *
C65TargetLowering::EmitSTZz(MachineInstr *MI, MachineBasicBlock *MBB,
                            unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned Instr = Use8Bit ? C65::STZ_8zp : C65::STZ_16zp;
  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  const unsigned Reg = MI->getOperand(0).getReg();

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(Instr))
      .addImm(RI->getZRAddress(Reg) + I);
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitSTzi(MachineInstr *MI, MachineBasicBlock *MBB,
                            unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
  const unsigned STAInstr = Use8Bit ? C65::STA_8i  : C65::STA_16i;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
    BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
      .addDisp(MI->getOperand(1), I);
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitLDzi(MachineInstr *MI,
                            MachineBasicBlock *MBB,
                            unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA_8i  : C65::LDA_16i;
  const unsigned STAInstr = Use8Bit ? C65::STA_8zp : C65::STA_16zp;

  DebugLoc DL = MI->getDebugLoc();
  MachineBasicBlock::iterator MBBI = MI;

  for (unsigned I = 0; I < NumBytes; I += AccSize) {
    BuildMI(*MBB, MBBI, DL, TII->get(LDAInstr))
      .addDisp(MI->getOperand(1), I);
    BuildMI(*MBB, MBBI, DL, TII->get(STAInstr))
      .addImm(RI->getZRAddress(MI->getOperand(0).getReg()) + I);
  }

  MI->eraseFromParent();

  return MBB;
}

MachineBasicBlock *
C65TargetLowering::EmitLDzimm(MachineInstr *MI,
                              MachineBasicBlock *MBB,
                              unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA_8imm : C65::LDA_16imm;
  const unsigned STAInstr = Use8Bit ? C65::STA_8zp  : C65::STA_16zp;
  const unsigned STZInstr = Use8Bit ? C65::STZ_8zp  : C65::STZ_16zp;

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
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
  const unsigned OPInstr  = Use8Bit ? Instr8 : Instr16;
  const unsigned STAInstr = Use8Bit ? C65::STA_8zp : C65::STA_16zp;
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
C65TargetLowering::EmitMOVzz(MachineInstr *MI,
                             MachineBasicBlock *MBB,
                             unsigned NumBytes) const {
  const bool Use8Bit = NumBytes == 1 || !Subtarget->is16bit();
  const unsigned AccSize = Use8Bit ? 1 : 2;

  const C65InstrInfo *TII = Subtarget->getInstrInfo();
  const C65RegisterInfo *RI = Subtarget->getRegisterInfo();
  const unsigned LDAInstr = Use8Bit ? C65::LDA_8zp : C65::LDA_16zp;
  const unsigned STAInstr = Use8Bit ? C65::STA_8zp : C65::STA_16zp;
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
  unsigned OpSize = 1 << C65::getZROpSize(MI->getDesc().TSFlags);
  switch (MI->getOpcode()) {
  case C65::BRCC8zz:
  case C65::BRCC16zz:
  case C65::BRCC32zz:
  case C65::BRCC64zz:
    return EmitBR_CC(MI, MBB, OpSize);
  case C65::STZ8z:
  case C65::STZ16z:
  case C65::STZ32z:
  case C65::STZ64z:
    return EmitSTZz(MI, MBB, OpSize);
  case C65::ST8zi:
  case C65::ST16zi:
  case C65::ST32zi:
  case C65::ST64zi:
    return EmitSTzi(MI, MBB, OpSize);
  case C65::LD8zi:
  case C65::LD16zi:
  case C65::LD32zi:
  case C65::LD64zi:
    return EmitLDzi(MI, MBB, OpSize);
  case C65::LD8zimm:
  case C65::LD16zimm:
  case C65::LD32zimm:
  case C65::LD64zimm:
    return EmitLDzimm(MI, MBB, OpSize);
  case C65::MOV8zz:
  case C65::MOV16zz:
  case C65::MOV32zz:
  case C65::MOV64zz:
    return EmitMOVzz(MI, MBB, OpSize);
  case C65::AND8zz:
  case C65::AND16zz:
  case C65::AND32zz:
  case C65::AND64zz:
    return EmitBinaryZI(MI, MBB, OpSize, C65::AND_8zp, C65::AND_16zp);
  case C65::OR8zz:
  case C65::OR16zz:
  case C65::OR32zz:
  case C65::OR64zz:
    return EmitBinaryZI(MI, MBB, OpSize, C65::ORA_8zp, C65::ORA_16zp);
  case C65::XOR8zz:
  case C65::XOR16zz:
  case C65::XOR32zz:
  case C65::XOR64zz:
    return EmitBinaryZI(MI, MBB, OpSize, C65::EOR_8zp, C65::EOR_16zp);
  case C65::ADD8zz:
  case C65::ADD16zz:
  case C65::ADD32zz:
  case C65::ADD64zz:
    return EmitBinaryZI(MI, MBB, OpSize, C65::ADC_8zp, C65::ADC_16zp,
                        true, false);
  case C65::SUB8zz:
  case C65::SUB16zz:
  case C65::SUB32zz:
  case C65::SUB64zz:
    return EmitBinaryZI(MI, MBB, OpSize, C65::ADC_8zp, C65::ADC_16zp,
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
  CCInfo.AnalyzeCallOperands(Outs, CC_65c816);

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

  // TODO: THIS
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
  CCInfo.AnalyzeCallResult(Ins, RetCC_65c816);

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
