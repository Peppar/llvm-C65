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

  /// VarArgsFrameOffset - Frame offset to start of varargs area.
  int VarArgsFrameOffset;

  /// BytesToPopOnReturn - Number of bytes function pops on return (in
  /// addition to the space used by the return address).
  unsigned BytesToPopOnReturn;

  /// ReturnAddrIndex - FrameIndex for return slot.
  int ReturnAddrIndex;

  /// Far - True if this function has a 24-bit return address.
  bool IsFar;

public:
  C65MachineFunctionInfo() : BytesToPopOnReturn(0),
                             ReturnAddrIndex(0),
                             IsFar(false) {}

  explicit C65MachineFunctionInfo(MachineFunction &MF)
    : BytesToPopOnReturn(0),
      ReturnAddrIndex(0),
      IsFar(false) {}

  unsigned getBytesToPopOnReturn() const { return BytesToPopOnReturn; }
  void setBytesToPopOnReturn (unsigned bytes) { BytesToPopOnReturn = bytes; }

  int getVarArgsFrameOffset() const { return VarArgsFrameOffset; }
  void setVarArgsFrameOffset(int Offset) { VarArgsFrameOffset = Offset; }

  int getRAIndex() const { return ReturnAddrIndex; }
  void setRAIndex(int Index) { ReturnAddrIndex = Index; }

  bool getIsFar() const { return IsFar; }
  void setIsFar(bool far) { IsFar = far; }

};

} // End llvm namespace

#endif
