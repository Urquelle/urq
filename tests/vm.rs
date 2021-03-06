use urq::vm::Mem;
use urq::vm::Instr;
use urq::vm::Reg;
use urq::vm::Vm;
use urq::vm::Device;

#[test]
fn mem_write() {
    let mut mem = Mem::new(1024);
    mem.write(0, 3);
    assert_eq!(3, mem.read(0));
    assert_ne!(5, mem.read(0));
}

#[test]
fn mem_write16() {
    let mut mem = Mem::new(1024);
    mem.write16(0, 1024);
    assert_eq!(1024, mem.read16(0));
    assert_ne!(0, mem.read16(0));
}

#[test]
fn mem_hex() {
    let mut mem = Mem::new(1024);
    mem.write(0, 0x34);
    mem.write(1, 0x12);
    assert_eq!(0x1234, mem.read16(0));
}

#[test]
fn reg_write() {
    let mem = Mem::new(1024);
    let mut vm = Vm::new(mem);

    vm.write_reg(Reg::R1, 1024);
    assert_eq!(1024, vm.read_reg(Reg::R1 as u8));

    vm.write_reg(Reg::R8, 2048);
    assert_eq!(2048, vm.read_reg(Reg::R8 as u8));
}

#[test]
fn instr_add_reg_reg() {
    let mem = Mem::new(256*256);
    let mut vm = Vm::new(mem);

    let mut pc = 0;

    // =============================== PROGRAMM ===================================
    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x1111) as usize;
    pc += vm.device.write(pc, Reg::R1) as usize;

    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x2222) as usize;
    pc += vm.device.write(pc, Reg::R2) as usize;

    pc += vm.device.write(pc, Instr::ADD_REG_REG) as usize;
    pc += vm.device.write(pc, Reg::R1) as usize;
    pc += vm.device.write(pc, Reg::R2) as usize;

    vm.device.write(pc, Instr::HLT) as usize;
    // ============================================================================

    vm.run();

    assert_eq!(0x1111, vm.read_reg(Reg::R1));
    assert_eq!(0x2222, vm.read_reg(Reg::R2));
    assert_eq!(0x3333, vm.read_reg(Reg::ACC));
}
