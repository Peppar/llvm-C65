//===-- C65CallingConv.td - Calling Conventions 65C816 --*- tablegen -*-===//

def CC_65c816 : CallingConv<[
  CCIfType<[i8], CCPromoteToType<i16>>,
  CCIfType<[i16], CCAssignToReg<[A, X, Y]>>,
  CCAssignToStack<2, 2>
]>;

def RetCC_65c816 : CallingConv<[
  // Return values are passed exactly like arguments
  CCDelegateTo<CC_65c816>
]>;
