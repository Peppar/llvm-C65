//===-- C65CallingConv.td - Calling Conventions 65C816 --*- tablegen -*-===//

def CC_65c816 : CallingConv<[
  CCIfType<[i16], CCAssignToReg<[A, ZR0, ZR1, ZR2, ZR3]>>,
  CCAssignToStack<2, 2>
]>;

def RetCC_65c816 : CallingConv<[
  // Return values are passed exactly like arguments
  CCDelegateTo<CC_65c816>
]>;
