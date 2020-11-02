#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(dead_code)]

use std::collections::LinkedList;
use std::mem::size_of;

trait Value {
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

enum Result {
    Ok,
    Error(String),
    Halt,
}

trait Device {
    fn write<T: Value>(&mut self, addr: usize, val: T) -> u8;
    fn write16(&mut self, addr: usize, val: u16) -> u8;
    fn read(&self, addr: usize) -> u8;
    fn read16(&self, addr: usize) -> u16;
    fn size(&self) -> usize;
    fn print(&self, addr: usize, n: usize);
}

struct Mem {
    pub mem  : Vec<u8>,
    pub size : usize,
}

impl Mem {
    fn new(size: usize) -> Self {
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

    fn write16(&mut self, addr: usize, val: u16) -> u8 {
        self.mem[addr]   = val as u8;
        self.mem[addr+1] = (val >> 8) as u8;

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

    fn print(&self, addr: usize, n: usize) {
        println!("########################## MEM ####################################");
        print!("MEM {:#04x}: ", addr);
        for i in 0..n+1 {
            print!("{:#02x} ", self.mem[addr+i]);
        }
        println!("###################################################################");
    }
}

struct Region<T>
where
    T: Device
{
    device  : *mut T,
    start   : usize,
    end     : usize,
    remap   : bool,
}

struct Mapper<T>
where
    T: Device
{
    regions: LinkedList<Region<T>>,
}

impl<T> Mapper<T>
where
    T: Device
{
    fn find(&self, addr: usize) -> std::result::Result<&Region<T>, String> {
        loop {
            match self.regions.iter().next() {
                Some(r) => {
                    if addr >= r.start && addr <= r.end {
                        return Ok(r);
                    }
                },

                None => {
                    return Err(String::from("bla"));
                }
            }
        }
    }

    fn add(&mut self, device: *mut T, start: usize, end: usize, remap: bool) {
        self.regions.push_front(Region {
            device, start, end, remap
        });
    }
}

impl<T> Device for Mapper<T>
where
    T: Device
{
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

    fn write16(&mut self, addr: usize, val: u16) -> u8 {
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

    fn print(&self, addr: usize, n: usize) {
        let result = self.find(addr);

        match result {
            Ok(region) => {
                let mut actual_addr = addr;

                if region.remap {
                    actual_addr = addr - region.start;
                }

                return unsafe { (*region.device).print(actual_addr, n) };
            },

            Err(msg) => {
                panic!(msg)
            }
        }
    }
}

#[derive(Copy, Clone)]
enum Instr {
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
enum Reg {
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

impl From<u8> for Reg {
    fn from(input: u8) -> Self {
        match input {
            0x0 => Reg::R1,
            0x1 => Reg::R2,
            0x2 => Reg::R3,
            0x3 => Reg::R4,
            0x4 => Reg::R5,
            0x5 => Reg::R6,
            0x6 => Reg::R7,
            0x7 => Reg::R8,
            0x8 => Reg::ACC,
            _   => Reg::Unknown,
        }
    }
}

struct Vm<T>
where
    T: Device
{
    pub device : T,
    regs   : [u16; 9],
    pc     : usize,
    sp     : usize,
    fp     : usize,
    stack_frame_size: usize,
}

impl<T> Vm<T>
where
    T: Device
{
    pub fn new(device: T) -> Self {
        Vm {
            sp     : device.size() - 1 - size_of::<usize>(),
            fp     : device.size() - 1 - size_of::<usize>(),
            stack_frame_size: 0,
            pc     : 0,
            regs   : [0; 9],
            device : device,
        }
    }

    pub fn read_reg(&self, reg: u8) -> u16 {
        self.regs[reg as usize]
    }

    pub fn write_reg(&mut self, reg: u8, val: u16) {
        self.regs[reg as usize] = val;
    }

    pub fn step(&mut self) -> Result {
        let instr = Instr::from(self.fetch());

        self.exec(instr)
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
        self.push(self.read_reg(Reg::R1 as u8));
        self.push(self.read_reg(Reg::R2 as u8));
        self.push(self.read_reg(Reg::R3 as u8));
        self.push(self.read_reg(Reg::R4 as u8));
        self.push(self.read_reg(Reg::R5 as u8));
        self.push(self.read_reg(Reg::R6 as u8));
        self.push(self.read_reg(Reg::R7 as u8));
        self.push(self.read_reg(Reg::R8 as u8));
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
        self.write_reg(Reg::R8 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R7 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R6 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R5 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R4 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R3 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R2 as u8, val);
        val = self.pop();
        self.write_reg(Reg::R1 as u8, val);

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
                self.write_reg(Reg::ACC as u8, val1 + val2);

                Result::Ok
            },

            Instr::ADD_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC as u8, reg_val + imm);

                Result::Ok
            },

            Instr::SUB_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC as u8, val1 - val2);

                Result::Ok
            },

            Instr::SUB_IMM_REG => {
                let imm = self.fetch16();
                let reg = self.fetch();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC as u8, reg_val - imm);

                Result::Ok
            },

            Instr::SUB_REG_IMM => {
                let reg = self.fetch();
                let imm = self.fetch16();
                let reg_val = self.read_reg(reg);
                self.write_reg(Reg::ACC as u8, reg_val - imm);

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
                self.write_reg(Reg::ACC as u8, imm * self.read_reg(reg));

                Result::Ok
            },

            Instr::MUL_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC as u8, val1 * val2);

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
                self.write_reg(Reg::ACC as u8, val1 & val2);

                Result::Ok
            },

            Instr::AND_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC as u8, val1 & val2);

                Result::Ok
            },

            Instr::OR_REG_IMM => {
                let reg = self.fetch();
                let val1 = self.read_reg(reg);
                let val2 = self.fetch16();
                self.write_reg(Reg::ACC as u8, val1 | val2);

                Result::Ok
            },

            Instr::OR_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC as u8, val1 | val2);

                Result::Ok
            },

            Instr::XOR_REG_IMM => {
                let reg = self.fetch();
                let val1 = self.read_reg(reg);
                let val2 = self.fetch16();
                self.write_reg(Reg::ACC as u8, val1 ^ val2);

                Result::Ok
            },

            Instr::XOR_REG_REG => {
                let reg1 = self.fetch();
                let reg2 = self.fetch();
                let val1 = self.read_reg(reg1);
                let val2 = self.read_reg(reg2);
                self.write_reg(Reg::ACC as u8, val1 ^ val2);

                Result::Ok
            },

            Instr::NOT => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                self.write_reg(Reg::ACC as u8, (!val) & 0xffff);

                Result::Ok
            },

            Instr::JNE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
                let addr = self.fetch16();

                if val != acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JNE_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC as u8);

                if imm != acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JEQ_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
                let addr = self.fetch16();

                if val == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JEQ_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC as u8);

                if imm == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLT_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
                let addr = self.fetch16();

                if val == acc {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLT_IMM => {
                let imm = self.fetch16();
                let addr = self.fetch16();
                let acc = self.read_reg(Reg::ACC as u8);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc < imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGT_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
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
                let acc = self.read_reg(Reg::ACC as u8);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc > imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JLE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
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
                let acc = self.read_reg(Reg::ACC as u8);

                // @ACHTUNG: prüfen ob der vergleich richtig ist
                if acc <= imm {
                    self.pc = addr as usize;
                }

                Result::Ok
            },

            Instr::JGE_REG => {
                let reg = self.fetch();
                let val = self.read_reg(reg);
                let acc = self.read_reg(Reg::ACC as u8);
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
                let acc = self.read_reg(Reg::ACC as u8);

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
                Result::Halt
            },

            _ => {
                Result::Error(String::from("Unbekannte Anweisung"))
            },
        }
    }

    pub fn run(&mut self) {
        loop {
            match self.step() {
                Result::Ok         => (),
                Result::Halt       => break,
                Result::Error(msg) => panic!(msg),
            }
        }
    }
}

#[test]
fn mem_write() {
    let mut mem = Mem::new(1024);
    mem.write(0, 3 as u8);
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
    mem.write(0, 0x34 as u8);
    mem.write(1, 0x12 as u8);
    assert_eq!(0x1234, mem.read16(0));
}

#[test]
fn reg_test() {
    let mem = Mem::new(1024);
    let mut vm = Vm::new(mem);

    vm.write_reg(Reg::R1 as u8, 1024);
    assert_eq!(1024, vm.read_reg(Reg::R1 as u8));

    vm.write_reg(Reg::R8 as u8, 2048);
    assert_eq!(2048, vm.read_reg(Reg::R8 as u8));
}

#[test]
fn vm_test() {
    let mem = Mem::new(256*256);
    let mut vm = Vm::new(mem);

    let mut pc = 0;
    let subroutine_address = 0x3000;

    // =============================== PROGRAMM ===================================

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
    vm.device.write16(pc, 0x4444);

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
    // ============================================================================
}
