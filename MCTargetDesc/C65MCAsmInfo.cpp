//===-- C65MCAsmInfo.cpp - C65 asm properties -----------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the C65MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "C65MCAsmInfo.h"
//#include "C65MCExpr.h"
#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCStreamer.h"

using namespace llvm;

void C65ELFMCAsmInfo::anchor() { }

C65ELFMCAsmInfo::C65ELFMCAsmInfo(StringRef TT) {
  Triple TheTriple(TT);

  PointerSize = 2;
  CalleeSaveStackSlotSize = 1;
  IsLittleEndian = false;

  Data8bitsDirective = "\t.db\t";
  Data16bitsDirective = "\t.dw\t";
  Data32bitsDirective = nullptr;
  Data64bitsDirective = nullptr;

  AsciiDirective = "\t.db\t";
  AscizDirective = nullptr;
  ZeroDirective = nullptr;
  CommentString = ";";

  InlineAsmStart = ";APP\n";
  InlineAsmEnd = ";NO_APP\n";

  HasSetDirective = false;
}
