//===-- C65CallingConv.td - Calling Conventions for C65 --*- tablegen -*-===//

def CC_C65 : CallingConv<[
  CCIfType<[i8],  CCAssignToReg<[ZR0, ZR1, ZR2, ZR3,
                                 ZR4, ZR5, ZR6, ZR7,
                                 ZR8, ZR9, ZR10, ZR11,
                                 ZR12, ZR13, ZR14, ZR15]>>,
  CCIfType<[i16], CCAssignToReg<[ZR0W, ZR2W, ZR4W, ZR6W,
                                 ZR8W, ZR10W, ZR12W, ZR14W]>>,
  CCIfType<[i32], CCAssignToReg<[ZR0D, ZR4D, ZR8D, ZR12D]>>,
  CCIfType<[i64], CCAssignToReg<[ZR0Q, ZR8Q]>>,

  // Assign to stack if we run out of zero page registers.
  // Set align to 1 (Do not align)
  CCIfType<[i8],  CCAssignToStack<1, 1>>,
  CCIfType<[i16], CCAssignToStack<2, 1>>,
  CCIfType<[i32], CCAssignToStack<4, 1>>,
  CCIfType<[i64], CCAssignToStack<8, 1>>
]>;


def RetCC_C65 : CallingConv<[
  // Return values are passed exactly like arguments.
  CCDelegateTo<CC_C65>
]>;

// By default, save half of the ZP registers.
def CSR_C65 : CalleeSavedRegs<(add ZR16W, ZR18W, ZR20W, ZR22W, ZR24W,
                                   ZR26W, ZR28W, ZR30W)>;

// Libcalls are required to save all ZP registers that are touched.
def CSR_C65_AllRegs : CalleeSavedRegs<(add CSR_C65, ZR0W, ZR2W, ZR4W, ZR8W,
                                           ZR10W, ZR12W, ZR14W)>;

/// WDC W65816 C compiler calling convention.
///
// def CC_W65816CC : CallingConv<[
//   // All arguments are passed via the stack.
//   CCIfType<[i8],  CCAssignToStack<1, 1>>,
//   CCIfType<[i16], CCAssignToStack<2, 1>>,
//   CCIfType<[i32], CCAssignToStack<4, 1>>,
//   CCIfType<[i64], CCAssignToStack<8, 1>>
// ]>;

// def RetCC_W65816CC : CallingConv<[
//   // Return values are passed via A, or X:A for 32-bit integers.
//   CCIfType<[i8], CCAssignToReg<[AL]>>,
//   CCIfType<[i16], CCAssignToReg<[A, X]>>
// ]>;
