//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

// 8-bit accumulator register
def A : Register<"A"> {
  let Namespace = "C65";
}

// 16-bit pointer register
def X : Register<"X"> {
  let Namespace = "C65";
}

// 16-bit program counter register
def PC : Register<"PC"> {
  let Namespace = "C65";
}

// Register class for the accumulator
def ACC8 : RegisterClass<"C65", [i8], 8, (add A)>;

// Register class for the pointer
def IX16 : RegisterClass<"C65", [i16], 16, (add X)>;
