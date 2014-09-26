//===-- C65.td - Target definition file for 6502 compatibles --*- tablegen -*-===//
//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// 6502 compatible subtarget features.
//

def ModeAcc8Bit  : SubtargetFeature<"acc8-mode", "InAcc8Mode", "true",
                                    "8-bit accumulator mode">;
def ModeAcc16Bit : SubtargetFeature<"acc16-mode", "InAcc16Mode", "true",
                                    "16-bit accumulator mode">;
def ModeIx8Bit   : SubtargetFeature<"ix8-mode", "InIx8Mode", "true",
                                    "8-bit index mode">;
def ModeIx16Bit  : SubtargetFeature<"ix16-mode", "InIx16Mode", "true",
                                    "16-bit index mode">;
def Feature65C02 : SubtargetFeature<"65C02", "Has65C02", "true",
                                    "Enable 65C02 instructions">;
def Feature65802 : SubtargetFeature<"65802", "Has65802", "true",
                                    "Enable 65802 instructions",
                                    [Feature65C02]>;
def Feature65816 : SubtargetFeature<"65816", "Has65816", "true",
                                    "Enable 65816 instructions",
                                    [Feature65802]>;
//def FeatureBitManip<"bit-manip", "HasBitManip", "BitManip",
//                    "Enable Rockwell and WDC bit manipulation functions",
//                    [Feature65C02]>;
//def FeatureSTP : SubtargetFeature<"stp", "HasSTP", "true",
//                                  "Enable STP instruction",
//                                  [Feature65C02]>;
//def FeatureWAI : SubtargetFeature<"wai", "HasWAI", "true",
//                                  "Enable WAI instruction",
//                                  [Feature65C02]>;

class Proc<string Name, list<SubtargetFeature> Features>
  : Processor<Name, NoItineraries, Features>;

def : Proc<"6502",  []>;
def : Proc<"65C02", [Feature65C02]>;
def : Proc<"65802", [Feature65802]>;
def : Proc<"65816", [Feature65816]>;

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "C65RegisterInfo.td"
include "C65CallingConv.td"
include "C65InstrInfo.td"

def C65InstrInfo : InstrInfo;

def C65AsmParser : AsmParser {
  bit ShouldEmitMatchRegisterName = 0;
}

//===----------------------------------------------------------------------===//
// Declare the target
//===----------------------------------------------------------------------===//

def C65 : Target {
  // Information about the instructions
  let InstructionSet = C65InstrInfo;
  let AssemblyParsers = [C65AsmParser];
}
