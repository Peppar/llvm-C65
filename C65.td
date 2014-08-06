//===-- C65.td - Target definition file for 6502 compatibles --*- tablegen -*-===//
//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// 6502 compatible subtarget features.
//

//def Mode16Bit : SubtargetFeature<"16bit-mode", "In16BitMode", "true",
//                                 "16-bit mode (65C816)">;
//def Mode8Bit : SubtargetFeature<"8bit-mode, "In8BitMode, "true",
//                                "8-bit mode (6502)">;

def Feature65C02 : SubtargetFeature<"65C02", "Has65C02", "true",
                                    "Enable 65C02 instructions">;
def Feature65C816 : SubtargetFeature<"65C816", "Has65C816", "true",
                                    "Enable 65C816 instructions",
                                    [Feature65C02]>

// TODO : Implement additional instructions found on some 65C02s
//def FeatureBitManip<"bit-manip", "HasBitManip", "BitManip",
//                   "Enable Rockwell and WDC bit manipulation functions",
//                   [Feature65C02]>;
//def FeatureSTP : SubtargetFeature<"stp", "HasSTP", "true",
//                                  "Enable STP instruction",
//                                  [Feature65C02]>;
//def FeatureWAI : SubtargetFeature<"wai", "HasWAI", "true",
//                                  "Enable WAI instruction",
//                                  [Feature65C02]>;

class Proc<string Name, list<SubtargetFeatures> Features>
  : ProcessorModel<Name, NoItineraries, Features>;

def : Proc<"6502",   []>;
def : Proc<"65C02",  [Feature65C02]>;
def : Proc<"65C816", [Feature65C816]>;

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