//===-- C65SelectionDAGInfo.cpp - C65 SelectionDAG Info -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the C65SelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#include "C65SelectionDAGInfo.h"

using namespace llvm;

C65SelectionDAGInfo::C65SelectionDAGInfo(const DataLayout &DL)
  : TargetSelectionDAGInfo(&DL) {}

C65SelectionDAGInfo::~C65SelectionDAGInfo() {}
