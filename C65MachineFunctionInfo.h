//===-- C65MachineFuctionInfo.h - C65 machine function info -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares C65-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_C65MACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_C65_C65MACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineValueType.h"
#include <vector>

namespace llvm {

/// C65MachineFunctionInfo - This class is derived from
/// MachineFunction and contains private C65 target-specific
/// information for each MachineFunction.
class C65MachineFunctionInfo : public MachineFunctionInfo {
  virtual void anchor();

  /// BytesToPopOnReturn - Number of bytes function pops on return (in
  /// addition to the space used by the return address).
  unsigned BytesToPopOnReturn;

public:
  C65MachineFunctionInfo() : BytesToPopOnReturn(0) {}

  explicit C65MachineFunctionInfo(MachineFunction &MF)
    : BytesToPopOnReturn(0) {}

  unsigned getBytesToPopOnReturn() const { return BytesToPopOnReturn; }
  void setBytesToPopOnReturn (unsigned bytes) { BytesToPopOnReturn = bytes; }
};

} // End llvm namespace

#endif
