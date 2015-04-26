//===-- C65BaseInfo.h - Top level definitions for C65 -------- --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum
// definitions for the C65 target useful for the compiler back-end and
// the MC libraries.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_MCTARGETDESC_C65BASEINFO_H
#define LLVM_LIB_TARGET_C65_MCTARGETDESC_C65BASEINFO_H

#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Support/DataTypes.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// C65II - This namespace holds all of the target specific flags that
/// instruction info tracks.
///
namespace C65II {

  enum {
    // This needs to be kept in sync with C65InstrInfo.td TSFlags

    // Accumulator register size.
    AccSize = (3 << 0),
    AccSizeShift = 0,
    AccUnknown = 1, // Used in C65RegSizeInsert to mark unknown size.
    Acc8Bit = 2,
    Acc16Bit = 3,

    // Index register size.
    IxSize = (3 << 2),
    IxSizeShift = 2,
    IxUnknown = 1, // Used in C65RegSizeInsert to mark unknown size.
    Ix8Bit = 2,
    Ix16Bit = 3,

    // Is a ZR instruction.
    ZRInstr = (1 << 4),

    // ZR operand size.
    ZROpSize = (3 << 5),
    ZROpSizeShift = 5,

    // Machine code operand byte size.
    OpSize = (3 << 7),
    OpSizeShift = 7,

    // Machine opcode.
    Opcode = (255 << 9),
    OpcodeShift = 9,

    // Machine operand is PC relative.
    OpPCRel = (1 << 17)
  };

  static inline unsigned getAccSize(unsigned int Flags) {
    return (Flags & AccSize) >> C65II::AccSizeShift;
  }
  static inline unsigned getIxSize(unsigned int Flags) {
    return (Flags & IxSize) >> C65II::IxSizeShift;
  }
  static inline unsigned getZROpSize(unsigned int Flags) {
    return (Flags & ZROpSize) >> C65II::ZROpSizeShift;
  }
  static inline unsigned getOpSize(unsigned int Flags) {
    return (Flags & OpSize) >> C65II::OpSizeShift;
  }
  static inline unsigned getOpcode(unsigned int Flags) {
    return (Flags & Opcode) >> C65II::OpcodeShift;
  }
}

} // end namespace llvm;

#endif
