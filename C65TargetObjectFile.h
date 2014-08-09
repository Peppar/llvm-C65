//===-- C65TargetObjectFile.h - C65 Object Info -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_SPARC_TARGETOBJECTFILE_H
#define LLVM_TARGET_SPARC_TARGETOBJECTFILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

class MCContext;
class TargetMachine;

class C65ELFTargetObjectFile : public TargetLoweringObjectFileELF {
public:
  C65ELFTargetObjectFile() :
    TargetLoweringObjectFileELF() {}

  const MCExpr *
  getTTypeGlobalReference(const GlobalValue *GV, unsigned Encoding,
                          Mangler &Mang, const TargetMachine &TM,
                          MachineModuleInfo *MMI,
                          MCStreamer &Streamer) const override;
};

} // end namespace llvm

#endif
