//===- C65MCTargetDesc.h - 6502 compatibles Target Descriptions -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides 6502 compatibles specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_C65MCTARGETDESC_H
#define LLVM_TARGET_C65MCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class Target;
class StringRef;
class raw_ostream;

extern Target The65C816Target;

  MCCodeEmitter *createC65MCCodeEmitter(const MCInstrInfo &MCII,
                                        const MCRegisterInfo &MRI,
                                        const MCSubtargetInfo &STI,
                                        MCContext &Ctx);
  MCAsmBackend *createC65MCAsmBackend(const Target &T,
                                      const MCRegisterInfo &MRI,
                                      StringRef TT,
                                      StringRef CPU);
  MCObjectWriter *createC65WLAKObjectWriter(raw_ostream &OS,
                                            uint8_t OSABI);

} // End llvm namespace

// Defines symbolic names for C65 registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "C65GenRegisterInfo.inc"

// Defines symbolic names for the C65 instructions.
//
#define GET_INSTRINFO_ENUM
#include "C65GenInstrInfo.inc"
#define GET_SUBTARGETINFO_ENUM
#include "C65GenSubtargetInfo.inc"

#endif
