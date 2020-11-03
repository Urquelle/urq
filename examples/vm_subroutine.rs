use urq::vm::Mem;
use urq::vm::Instr;
use urq::vm::Reg;
use urq::vm::Vm;
use urq::vm::Device;

fn main() {
    let mem = Mem::new(256*256);
    let mut vm = Vm::new(mem);

    let subroutine_address = 0x3000;
    let mut pc = 0;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x3333) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x2222) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x1111) as usize;

    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x1234) as usize;
    pc += vm.device.write(pc, Reg::R1) as usize;

    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x5678) as usize;
    pc += vm.device.write(pc, Reg::R4) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x0000) as usize;

    pc += vm.device.write(pc, Instr::CAL_IMM) as usize;
    pc += vm.device.write16(pc, subroutine_address) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x4444) as usize;

    vm.device.write(pc, Instr::HLT) as usize;

    pc = subroutine_address as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x0102) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x0304) as usize;

    pc += vm.device.write(pc, Instr::PSH_IMM) as usize;
    pc += vm.device.write16(pc, 0x0506) as usize;

    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x0708) as usize;
    pc += vm.device.write(pc, Reg::R1) as usize;

    pc += vm.device.write(pc, Instr::MOV_IMM_REG) as usize;
    pc += vm.device.write16(pc, 0x090A) as usize;
    pc += vm.device.write(pc, Reg::R8) as usize;

    vm.device.write(pc, Instr::RET);

    vm.run();

    vm.print_regs();
    vm.device.print(vm.pc, "PC", 32);
    vm.device.print(vm.device.size() - 42, "STACK", 41);
}
