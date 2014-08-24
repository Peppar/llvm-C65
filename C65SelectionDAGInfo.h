//==- C65TargetMachine.h - Define TargetMachine for C65 -----------*- C++ -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the 6502 compatibles specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_C65SELECTIONDAGINFO_H
#define LLVM_TARGET_C65SELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class C65TargetMachine;

class C65SelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit C65SelectionDAGInfo(const DataLayout &DL);
  ~C65SelectionDAGInfo();
};

}

#endif
