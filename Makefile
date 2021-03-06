##===- lib/Target/C65/Makefile -----------------------------*- Makefile -*-===##
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
##===----------------------------------------------------------------------===##

LEVEL = ../../..
LIBRARYNAME = LLVMC65CodeGen
TARGET = C65

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = C65GenAsmMatcher.inc \
		C65GenAsmWriter.inc \
		C65GenCallingConv.inc \
		C65GenDAGISel.inc \
		C65GenInstrInfo.inc \
		C65GenRegisterInfo.inc \
		C65GenSubtargetInfo.inc \
		C65GenMCCodeEmitter.inc \
		C65GenCodeEmitter.inc

# C65GenDisassemblerTables.inc

DIRS = AsmParser InstPrinter TargetInfo MCTargetDesc

# Disassembler

include $(LEVEL)/Makefile.common

