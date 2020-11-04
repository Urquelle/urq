use urq::vm::{ Mem, Mapper, Instr, Reg, Vm, Device };

fn main() {
    let mem = Mem::new(256*256);
    let mapper = Mapper::new();
    let mut vm = Vm::new(mem);

}

