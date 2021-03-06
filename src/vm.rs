#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(dead_code)]

use std::collections::LinkedList;
use std::mem::size_of;

pub trait Value {
    fn uint8(&self) -> u8;
    fn uint16(&self) -> u16;
}

impl Value for u8 {
    fn uint8(&self) -> u8 {
        *self
    }

    fn uint16(&self) -> u16 {
        *self as u16
    }
}

impl Value for u16 {
    fn uint8(&self) -> u8 {
        *self as u8
    }

    fn uint16(&self) -> u16 {
        *self
    }
}

impl Value for i32 {
    fn uint8(&self) -> u8 {
        (*self & 0xff) as u8
    }

    fn uint16(&self) -> u16 {
        (*self & 0xffff) as u16
    }
}

impl Value for Reg {
    fn uint8(&self) -> u8 {
        *self as u8
    }

    fn uint16(&self) -> u16 {
        *self as u16
    }
}

impl Value for Instr {
    fn uint8(&self) -> u8 {
        *self as u8
    }

    fn uint16(&self) -> u16 {
        *self as u16
    }
}

pub enum Result {
    Ok,
    Error(String),
    Hlt,
}

pub trait Device {
    fn write<T: Value>(&mut self, addr: usize, val: T)   -> u8;
    fn write16<T: Value>(&mut self, addr: usize, val: T) -> u8;
    fn read(&self, addr: usize) -> u8;
    fn read16(&self, addr: usize) -> u16;
    fn size(&self) -> usize;
    fn print(&self, addr: usize, label: &str, n: usize);
}

pub struct Mem {
    pub mem  : Vec<u8>,
    pub size : usize,
}

impl Mem {
    pub fn new(size: usize) -> Self {
        Mem {
            mem  : vec![0; size],
            size : size,
        }
    }
}

impl Device for Mem {
    fn write<T: Value>(&mut self, addr: usize, val: T) -> u8 {
        self.mem[addr] = val.uint8();

        return 1;
    }

    fn write16<T: Value>(&mut self, addr: usize, val: T) -> u8 {
        self.mem[addr]   = val.uint16() as u8;
        self.mem[addr+1] = (val.uint16() >> 8) as u8;

        return 2;
    }

    fn read(&self, addr: usize) -> u8 {
        return self.mem[addr];
    }

    fn read16(&self, addr: usize) -> u16 {
        let result: u16 = (self.mem[addr+1] as u16) << 8 | (self.mem[addr] as u16);

        return result;
    }

    fn size(&self) -> usize {
        self.size
    }

    fn print(&self, addr: usize, label: &str, n: usize) {
        println!("\n{:=^69}", format!(" {} ", label));
        println!("{: ^69}", format!("MEM ADDR 0x{:04x}", addr));
        for i in 0..n {
            if (i % 14) == 0 && i > 0 {
                print!("\n");
            }

            print!("0x{:02x} ", self.mem[addr+i]);
        }
    }
}

pub struct Screen {
}

impl Screen {
    fn clear_screen() {
        print!("\x1b[2J");
    }

    fn set_bold() {
        print!("\x1b[1m");
    }

    fn set_regular() {
        print!("\x1b[0m");
    }

    fn move_to(x: usize, y: usize) {
        print!("\x1b[{};{}H", y, x);
    }
}

impl Device for Screen {
    fn write<T: Value>(&mut self, _addr: usize, _val: T) -> u8 {
        return 1;
    }

    fn write16<T: Value>(&mut self, addr: usize, val: T) -> u8 {
        let val = val.uint16();

        let cmd = (val & 0xff00) >> 8;
        let chr = val & 0x00ff;

        if cmd == 0xff {
            Screen::clear_screen();
        } else if cmd == 0x01 {
            Screen::set_bold();
        } else if cmd == 0x02 {
            Screen::set_regular();
        }

        let x : usize = (addr % 16) + 1;
        let y : usize = (addr / 16) + 1;

        Screen::move_to(x*2, y);
        print!("{}", chr);

        0
    }

    fn read(&self, _addr: usize) -> u8 {
        0
    }

    fn read16(&self, _addr: usize) -> u16 {
        0
    }

    fn size(&self) -> usize {
        0
    }

    fn print(&self, _addr: usize, _label: &str, _n: usize) {
        //
    }
}

struct Region {
    device  : *mut (dyn Device + 'static),
    start   : usize,
    end     : usize,
    remap   : bool,
}

pub struct Mapper {
    regions: LinkedList<Region>,
}

impl Mapper {
    pub fn new() -> Self {
        Mapper {
            regions: LinkedList::<Region>::new(),
        }
    }

    fn find(&self, addr: usize) -> std::result::Result<&Region, String> {
        loop {
            match self.regions.iter().next() {
                Some(r) => {
                    if addr >= r.start && addr <= r.end {
                        return Ok(r);
                    }
                },

                None => {
                    return Err(String::from("Kein passendes Ausgabegerät gefunden"));
                }
            }
        }
    }

    fn add(&mut self, device: *mut dyn Device, start: usize, end: usize, remap: bool) {
        self.regions.push_front(Region {
            device, start, end, remap
        });
    }
}

impl Device for Mapper {
    fn write<V: Value>(&mut self, addr: usize, val: V) -> u8 {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).write(actual_addr, val) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }

    fn write16<V: Value>(&mut self, addr: usize, val: V) -> u8 {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).write16(actual_addr, val) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }

    fn read(&self, addr: usize) -> u8 {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).read(actual_addr) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }

    fn read16(&self, addr: usize) -> u16 {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).read16(actual_addr) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }

    fn size(&self) -> usize {
        0
    }

    fn print(&self, addr: usize, label: &str, n: usize) {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).print(actual_addr, label, n) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }
}

#[derive(Copy, Clone)]
pub enum Instr {
    Unknown,

    MOV_IMM_REG     = 0x10,
    MOV_REG_REG     = 0x11,
    MOV_REG_MEM     = 0x12,
    MOV_MEM_REG     = 0x13,
    MOV_IMM_MEM     = 0x14,
    MOV_REG_PTR_REG = 0x15,
    MOV_IMM_OFF_REG = 0x16,

    ADD_REG_REG     = 0x20,
    ADD_IMM_REG     = 0x21,
    SUB_REG_REG     = 0x22,
    SUB_IMM_REG     = 0x23,
    SUB_REG_IMM     = 0x24,
    INC_REG         = 0x25,
    DEC_REG         = 0x26,
    MUL_IMM_REG     = 0x27,
    MUL_REG_REG     = 0x28,

    LSF_REG_IMM     = 0x30,
    LSF_REG_REG     = 0x31,
    RSF_REG_IMM     = 0x32,
    RSF_REG_REG     = 0x33,
    AND_REG_IMM     = 0x34,
    AND_REG_REG     = 0x35,
    OR_REG_IMM      = 0x36,
    OR_REG_REG      = 0x37,
    XOR_REG_IMM     = 0x38,
    XOR_REG_REG     = 0x39,
    NOT             = 0x3A,

    JNE_REG         = 0x40,
    JNE_IMM         = 0x41,
    JEQ_REG         = 0x42,
    JEQ_IMM         = 0x43,
    JLT_REG         = 0x44,
    JLT_IMM         = 0x45,
    JGT_REG         = 0x46,
    JGT_IMM         = 0x47,
    JLE_REG         = 0x48,
    JLE_IMM         = 0x49,
    JGE_REG         = 0x4A,
    JGE_IMM         = 0x4B,

    PSH_IMM         = 0x50,
    PSH_REG         = 0x51,
    POP             = 0x52,
    CAL_IMM         = 0x53,
    CAL_REG         = 0x54,
    RET             = 0x55,
    HLT             = 0x56,
}

impl From<u8> for Instr {
    fn from(input: u8) -> Self {
        match input {
            0x10 => Instr::MOV_IMM_REG,
            0x11 => Instr::MOV_REG_REG,
            0x12 => Instr::MOV_REG_MEM,
            0x13 => Instr::MOV_MEM_REG,
            0x14 => Instr::MOV_IMM_MEM,
            0x15 => Instr::MOV_REG_PTR_REG,
            0x16 => Instr::MOV_IMM_OFF_REG,

            0x20 => Instr::ADD_REG_REG,
            0x21 => Instr::ADD_IMM_REG,
            0x22 => Instr::SUB_REG_REG,
            0x23 => Instr::SUB_IMM_REG,
            0x24 => Instr::SUB_REG_IMM,
            0x25 => Instr::INC_REG,
            0x26 => Instr::DEC_REG,
            0x27 => Instr::MUL_IMM_REG,
            0x28 => Instr::MUL_REG_REG,

            0x30 => Instr::LSF_REG_IMM,
            0x31 => Instr::LSF_REG_REG,
            0x32 => Instr::RSF_REG_IMM,
            0x33 => Instr::RSF_REG_REG,
            0x34 => Instr::AND_REG_IMM,
            0x35 => Instr::AND_REG_REG,
            0x36 => Instr::OR_REG_IMM ,
            0x37 => Instr::OR_REG_REG ,
            0x38 => Instr::XOR_REG_IMM,
            0x39 => Instr::XOR_REG_REG,
            0x3A => Instr::NOT,

            0x40 => Instr::JNE_REG,
            0x41 => Instr::JNE_IMM,
            0x42 => Instr::JEQ_REG,
            0x43 => Instr::JEQ_IMM,
            0x44 => Instr::JLT_REG,
            0x45 => Instr::JLT_IMM,
            0x46 => Instr::JGT_REG,
            0x47 => Instr::JGT_IMM,
            0x48 => Instr::JLE_REG,
            0x49 => Instr::JLE_IMM,
            0x4A => Instr::JGE_REG,
            0x4B => Instr::JGE_IMM,

            0x50 => Instr::PSH_IMM,
            0x51 => Instr::PSH_REG,
            0x52 => Instr::POP,
            0x53 => Instr::CAL_IMM,
            0x54 => Instr::CAL_REG,
            0x55 => Instr::RET,
            0x56 => Instr::HLT,

            _    => Instr::Unknown,
        }
    }
}

#[derive(Copy, Clone)]
pub enum Reg {
    R1  = 0x0,
    R2  = 0x1,
    R3  = 0x2,
    R4  = 0x3,
    R5  = 0x4,
    R6  = 0x5,
    R7  = 0x6,
    R8  = 0x7,
    ACC = 0x8,
    Unknown,
}

pub struct Vm<T>
where
    T: Device
{
    pub device : T,
    pub regs   : [u16; 9],
    pub pc     : usize,
    pub sp     : usize,
    pub fp     : usize,
    pub stack_frame_size: usize,
}

impl<T> Vm<T>
where
    T: Device
{
    pub fn new(device: T) -> Self {
        Vm {
            sp     : device.size() - 1 - size_of::<usize>(),
            fp     : device.size() - 1 - size_of::<usize>(),
            pc     : 0,
            regs   : [0; 9],
            device : device,
            stack_frame_size: 0,
        }
    }

    pub fn read_reg<V: Value>(&self, reg: V) -> u16 {
        self.regs[reg.uint8() as usize]
    }

    pub fn write_reg<V: Value, W: Value>(&mut self, reg: V, val: W) {
        self.regs[reg.uint8() as usize] = val.uint16();
    }

    pub fn print_regs(&self) {
        println!("\n{:=^69}", " REGS ");
        println!("ACC: {:#06x} ", self.regs[Reg::ACC as usize]);
        for i in 0..8 {
            print!("R{}: {:#06x} ", i+1, self.regs[i]);
            if i % 5 == 0 && i > 0 {
                print!("\n");
            }
        }
    }

    pub fn step(&mut self) -> Result {
        let instr = Instr::from(self.fetch());

        self.exec(instr)
    }

    pub fn run(&mut self) {
        loop {
            match self.step() {
                Result::Ok         => (),
                Result::Hlt       => break,
                Result::Error(msg) => panic!(msg),
            }
        }
    }

    pub fn fetch(&mut self) -> u8 {
        let result = self.device.read(self.pc);
        self.pc += 1;

        result
    }

    pub fn fetch16(&mut self) -> u16 {
        let result = self.device.read16(self.pc);
        self.pc += 2;

        result
    }

    pub fn push(&mut self, val: u16) {
        self.device.write16(self.sp, val);
        self.sp -= 2;
        self.stack_frame_size += 2;
    }

    pub fn pop(&mut self) -> u16 {
        self.sp += 2;
        self.stack_frame_size -= 2;
        self.device.read16(self.sp)
    }

    pub fn push_state(&mut self) {
        self.push(self.read_reg(Reg::R1));
        self.push(self.read_reg(Reg::R2));
        self.push(self.read_reg(Reg::R3));
        self.push(self.read_reg(Reg::R4));
        self.push(self.read_reg(Reg::R5));
        self.push(self.read_reg(Reg::R6));
        self.push(self.read_reg(Reg::R7));
        self.push(self.read_reg(Reg::R8));
        self.push(self.pc as u16);
        self.push(self.stack_frame_size as u16 + 2);

        self.fp = self.sp;
        self.stack_frame_size = 0;
    }

    pub fn pop_state(&mut self) {
        self.sp = self.fp;
        self.stack_frame_size = self.pop() as usize;
        let stack_frame_size = self.stack_frame_size;

        self.pc = self.pop() as usize;

        let mut val = self.pop();
        self.write_reg(Reg::R8, val);

        val = self.pop();
        self.write_reg(Reg::R7, val);

        val = self.pop();
        self.write_reg(Reg::R6, val);

        val = self.pop();
        self.write_reg(Reg::R5, val);

        val = self.pop();
        self.write_reg(Reg::R4, val);

        val = self.pop();
        self.write_reg(Reg::R3, val);

        val = self.pop();
        self.write_reg(Reg::R2, val);

        val = self.pop();
        self.write_reg(Reg::R1, val);

        let num_args = self.pop();
        for _ in 0..num_args {
            self.pop();
        }

        self.fp = self.fp + stack_frame_size;
    }

    pub fn exec(&mut self, instr: Instr) -> Result {
        match instr {
            Instr::MOV_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                self.write_reg(reg, imm);

                Result::Ok
            },

            Instr::MOV_REG_REG => {
                let src = self.fetch();
                let dst = self.fetch();
                self.write_reg(src, self.read_reg(dst));

                Result::Ok
            },

            Instr::MOV_REG_MEM => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let addr = self.fetch16();
                self.device.write16(addr.into(), val);

                Result::Ok
            },

            Instr::MOV_MEM_REG => {
                let addr = self.fetch16();
                let val = self.device.read16(addr.into());
                let reg = self.fetch();
                self.write_reg(reg, val);

                Result::Ok
            },

            Instr::MOV_IMM_MEM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                self.device.write16(addr.into(), imm);

                Result::Ok
            },

            Instr::MOV_REG_PTR_REG => {
                let reg = self.fetch();
                let dst = self.fetch();
                let addr = self.read_reg(reg);
                let val = self.device.read16(addr.into());
                self.write_reg(dst, val);

                Result::Ok
            },

            Instr::MOV_IMM_OFF_REG => {
                let base_addr = self.fetch16();
                let offset_reg = self.fetch();
                let dst = self.fetch();
                let offset = self.read_reg(offset_reg);
                let val = self.device.read16((base_addr + offset).into());
                self.write_reg(dst, val);

                Result::Ok
            },

            Instr::ADD_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 + val2);

                Result::Ok
            },

            Instr::ADD_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC, reg_val + imm);

                Result::Ok
            },

            Instr::SUB_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 - val2);

                Result::Ok
            },

            Instr::SUB_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC, reg_val - imm);

                Result::Ok
            },

            Instr::SUB_REG_IMM => {
                let reg = self.fetch();
                let imm = self.fetch16();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC, reg_val - imm);

                Result::Ok
            },

            Instr::INC_REG => {
                let reg = self.fetch();
                self.write_reg(reg, self.read_reg(reg) + 1);

                Result::Ok
            },

            Instr::DEC_REG => {
                let reg = self.fetch();
                self.write_reg(reg, self.read_reg(reg) - 1);

                Result::Ok
            },

            Instr::MUL_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                self.write_reg(Reg::ACC, imm * self.read_reg(reg));

                Result::Ok
            },

            Instr::MUL_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 * val2);

                Result::Ok
            },

            Instr::LSF_REG_IMM => {
                let reg = self.fetch();
                let imm = self.fetch16();
                let val = self.read_reg(reg);
                self.write_reg(reg, val << imm);

                Result::Ok
            },

            Instr::LSF_REG_REG => {
                let reg = self.fetch();
                let shift = self.fetch();
                let shift = self.read_reg(shift);
                let val = self.read_reg(reg);
                self.write_reg(reg, val << shift);

                Result::Ok
            },

            Instr::RSF_REG_IMM => {
                let reg = self.fetch();
                let imm = self.fetch16();
                let val = self.read_reg(reg);
                self.write_reg(reg, val >> imm);

                Result::Ok
            },

            Instr::RSF_REG_REG => {
                let reg = self.fetch();
                let shift = self.fetch();
                let shift = self.read_reg(shift);
                let val = self.read_reg(reg);
                self.write_reg(reg, val >> shift);

                Result::Ok
            },

            Instr::AND_REG_IMM => {
                let reg = self.fetch();
                let val1 = self.read_reg(reg);
                let val2 = self.fetch16();
                self.write_reg(Reg::ACC, val1 & val2);

                Result::Ok
            },

            Instr::AND_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 & val2);

                Result::Ok
            },

            Instr::OR_REG_IMM => {
                let reg = self.fetch();
                let val1 = self.read_reg(reg);
                let val2 = self.fetch16();
                self.write_reg(Reg::ACC, val1 | val2);

                Result::Ok
            },

            Instr::OR_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 | val2);

                Result::Ok
            },

            Instr::XOR_REG_IMM => {
                let reg = self.fetch();
                let val1 = self.read_reg(reg);
                let val2 = self.fetch16();
                self.write_reg(Reg::ACC, val1 ^ val2);

                Result::Ok
            },

            Instr::XOR_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC, val1 ^ val2);

                Result::Ok
            },

            Instr::NOT => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                self.write_reg(Reg::ACC, (!val) & 0xffff);

                Result::Ok
            },

            Instr::JNE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                if val != acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JNE_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                if imm != acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JEQ_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                if val == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JEQ_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                if imm == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLT_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                if val == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLT_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc < imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGT_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc > val {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGT_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc > imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc <= val {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLE_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc <= imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC);
                let addr = self.fetch16();

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc >= val {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGE_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc >= imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::PSH_IMM => {
                let val = self.fetch16();
                self.push(val);

                Result::Ok
            },

            Instr::PSH_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                self.push(val);

                Result::Ok
            },

            Instr::POP => {
                let reg = self.fetch();
                let val = self.pop();
                self.write_reg(reg, val);

                Result::Ok
            },

            Instr::CAL_IMM => {
                let addr = self.fetch16();
                self.push_state();
                self.pc = addr as usize;

                Result::Ok
            },

            Instr::CAL_REG => {
                let reg = self.fetch();
                let addr = self.read_reg(reg);
                self.push_state();
                self.pc = addr as usize;

                Result::Ok
            },

            Instr::RET => {
                self.pop_state();

                Result::Ok
            },

            Instr::HLT => {
                Result::Hlt
            },

            _ => {
                Result::Error(String::from("Unbekannte Anweisung"))
            },
        }
    }
}

