//===-- C65.td - Target definition file for C65 ------------*- tablegen -*-===//
//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Processor definitions
//===----------------------------------------------------------------------===//

class Proc<string Name, list<SubtargetFeature> Features>
  : Processor<Name, NoItineraries, Features>;

def : Proc<"6502", []>;

//===----------------------------------------------------------------------===//
// Register file, instruction descriptions
//===----------------------------------------------------------------------===//

include "C65RegisterInfo.td"
include "C65InstrInfo.td"

def C65InstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// Declare the target
//===----------------------------------------------------------------------===//

def C65 : Target {
  let InstructionSet = C65InstrInfo;
}
