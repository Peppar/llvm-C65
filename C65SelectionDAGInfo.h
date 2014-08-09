//===-- C65SelectionDAGInfo.h - C65 SelectionDAG Info -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the C65 subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef C65SELECTIONDAGINFO_H
#define C65SELECTIONDAGINFO_H

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
