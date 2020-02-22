//===-- C65TargetInfo.h - C65 Target Information ----------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides the C65 Target.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_C65_TARGETINFO_C65TARGETINFO_H
#define LLVM_LIB_TARGET_C65_TARGETINFO_C65TARGETINFO_H

namespace llvm {
class Target;
Target &getThe65C816Target();
}

#endif

