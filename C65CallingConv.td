//===-- C65CallingConv.td - Calling Conventions 65C816 --*- tablegen -*-===//

def CC_65c816 : CallingConv<[
  CCIfType<[i8],  CCAssignToReg<[ZR0, ZR1, ZR2, ZR3, ZR4, ZR5, ZR6, ZR7>>,
  CCIfType<[i16], CCAssignToReg<[ZR0W, ZR2W, ZR4W, ZR6W]>>,
  CCIfType<[i32], CCAssignToReg<[ZR0D, ZR4D]>>,
  CCIfType<[i64], CCAssignToReg<[ZR0Q]>>
]>;

def RetCC_65c816 : CallingConv<[
  // Return values are passed exactly like arguments
  CCDelegateTo<CC_65c816>
]>;
