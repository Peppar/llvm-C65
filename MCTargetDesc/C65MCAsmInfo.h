//===-- C65MCAsmInfo.h - C65 asm properties --------------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the C65MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef C65TARGETASMINFO_H
#define C65TARGETASMINFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {
class StringRef;

class C65ELFMCAsmInfo : public MCAsmInfoELF {
  void anchor() override;
public:
  explicit C65ELFMCAsmInfo(StringRef TT);
};

} // namespace llvm

#endif
