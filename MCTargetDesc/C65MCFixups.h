//===-- C65MCFixups.h - C65-specific fixup entries --------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_MCTARGETDESC_C65MCFIXUPS_H
#define LLVM_LIB_TARGET_C65_MCTARGETDESC_C65MCFIXUPS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
  namespace C65 {
    enum Fixups {
      // Constants shifted by X to fit in an 8-bit register.
      FK_C65_8 = FirstTargetFixupKind,
      FK_C65_8s8,
      FK_C65_8s16,
      FK_C65_8s24,
      FK_C65_8s32,
      FK_C65_8s40,
      FK_C65_8s48,
      FK_C65_8s56,
      // Constants shifted by X to fit in a 16-bit register.
      FK_C65_16,
      FK_C65_16s16,
      FK_C65_16s32,
      FK_C65_16s48,
      // 24-bit fixup.
      FK_C65_24,

      // Marker
      LastTargetFixupKind,
      NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
    };

    static inline bool isFixup8Bit(int FK) {
      return FK >= FK_C65_8 && FK <= FK_C65_8s56;
    }
    static inline int get8BitFixupShiftAmt(int FK) {
      return (FK - FK_C65_8) << 3;
    }
    static inline bool isFixup16Bit(int FK) {
      return FK >= FK_C65_16 && FK <= FK_C65_16s48;
    }
    static inline int get16BitFixupShiftAmt(int FK) {
      return (FK - FK_C65_16) << 4;
    }
  } // end namespace C65
} // end namespace llvm

#endif
