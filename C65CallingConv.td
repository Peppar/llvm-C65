//===-- C65CallingConv.td - Calling Conventions 65C816 --*- tablegen -*-===//

//def RetCC_C65 : CallingConv<[
//  CCIfType<[i8] , CCAssignToReg<[AL, XL, YL]>>,
//  CCIfSubtarget<"is65816()", CCDelegateTo<RetCC_65C816>>,
//]>;

def CC_C65 : CallingConv<[
  // Handles byval parameters.
  //CCIfByVal<CCPassByVal<1, 1>>,

  CCIfType<[i8], CCAssignToReg<[AL, XL, YL]>>,
  CCIfType<[i16], CCAssignToReg<[A, X, Y]>>,

  CCAssignToStack<1, 1>
]>;

def RetCC_65C816 : CallingConv<[
  // Return values are passed exactly like arguments
  CCDelegateTo<CC_C65>
]>;

