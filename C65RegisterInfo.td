//===- C65RegisterInfo.td - Describe the C65 Register File --*- tablegen -*-==//

def A : Register<"A"> {
  let Namespace = "C65";
}

def X : Register<"X"> {
  let Namespace = "C65";
}

def ACC8 : RegisterClass<"C65", [i8], 8, (add A)>;
def IX16 : RegisterClass<"C65", [i16], 16, (add X)>;
