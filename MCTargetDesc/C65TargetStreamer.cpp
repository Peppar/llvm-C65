//===-- C65TargetStreamer.cpp - C65 Target Streamer Methods ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides C65 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

//#include "C65TargetStreamer.h"
// #include "InstPrinter/C65InstPrinter.h"
// #include "llvm/Support/FormattedStream.h"

// using namespace llvm;

// // pin vtable to this file
// C65TargetStreamer::C65TargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

// void C65TargetStreamer::anchor() {}

// C65TargetAsmStreamer::C65TargetAsmStreamer(MCStreamer &S,
//                                            formatted_raw_ostream &OS)
//     : C65TargetStreamer(S), OS(OS) {}

// void C65TargetAsmStreamer::emitC65RegisterIgnore(unsigned reg) {
//   OS << "\t.register "
//      << StringRef(C65InstPrinter::getRegisterName(reg)).lower()
//      << ", #ignore\n";
// }

// void C65TargetAsmStreamer::emitC65RegisterScratch(unsigned reg) {
//   OS << "\t.register "
//      << << StringRef(C65InstPrinter::getRegisterName(reg)).lower()
//      << ", #scratch\n";
// }

// C65TargetELFStreamer::C65TargetELFStreamer(MCStreamer &S)
//     : C65TargetStreamer(S) {}

// MCELFStreamer &C65TargetELFStreamer::getStreamer() {
//   return static_cast<MCELFStreamer &>(Streamer);
// }
