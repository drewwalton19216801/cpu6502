mod addresses;
pub mod bus;
pub mod device;
mod instructions;
mod registers;

use bus::Bus;
use instructions::INSTRUCTION_LIST;

use crate::{
    addresses::addresses::IRQ_VECTOR, addresses::addresses::NMI_VECTOR,
    addresses::addresses::RESET_VECTOR, instructions::AddressingMode,
    registers::registers::Registers,
};

pub struct Cpu {
    pub variant: Variant,     // CPU variant
    pub state: State,         // CPU state
    pub registers: Registers, // Registers

    pub bus: Box<dyn Bus>, // Bus

    pub cycles: u8,    // Number of cycles remaining for current instruction
    pub temp: u16,     // Temporary storage for various operations
    pub addr_abs: u16, // Absolute address
    pub addr_rel: u16, // Relative address
    pub addr_mode: AddressingMode, // Addressing mode
    pub opcode: u8,    // Current opcode
    pub fetched: u8,   // Fetched data

    pub enable_illegal_opcodes: bool, // Enable illegal opcodes

    pub current_instruction_string: String, // Current instruction string
    pub debug: bool,       // Print debug information?
}

#[derive(Clone, Copy, PartialEq)]
pub enum Variant {
    NMOS, // Original 6502 (with ROR bug)
    CMOS, // Modified 65C02 (no ROR bug)
    NES,  // Modified 2A03 (no decimal mode)
}

impl Variant {
    pub fn from_string(variant: String) -> Self {
        match variant.as_str() {
            "NMOS" => return Self::NMOS,
            "CMOS" => return Self::CMOS,
            "NES" => return Self::NES,
            _ => panic!("Invalid CPU variant"),
        }
    }

    pub fn to_string(&self) -> String {
        match self {
            Self::NMOS => return String::from("NMOS"),
            Self::CMOS => return String::from("CMOS"),
            Self::NES => return String::from("NES"),
        }
    }
}

pub enum State {
    Stopped,       // CPU is stopped
    Fetching,      // CPU is fetching an instruction
    Executing,     // CPU is executing an instruction
    Interrupt,     // CPU is handling an interrupt
    IllegalOpcode, // CPU encountered an illegal opcode
}

impl Cpu {
    pub fn default(debug: bool) -> Self {
        Self::new(Box::new(bus::DefaultBus::default()), debug)
    }

    pub fn new(bus: Box<dyn Bus>, debug: bool) -> Self {
        Self {
            registers: Registers::new(),
            variant: Variant::CMOS,
            state: State::Stopped,

            cycles: 0,
            temp: 0,
            addr_abs: 0,
            addr_rel: 0,
            addr_mode: AddressingMode::Implied,
            opcode: 0,
            fetched: 0,

            bus,

            enable_illegal_opcodes: false,

            current_instruction_string: String::from(""),
            debug: debug,
        }
    }

    pub fn dump_cycles(&self) {
        println!("Cycles remaining: {}", self.cycles);
    }

    pub fn change_variant(&mut self, variant: Variant) {
        self.variant = variant;
    }

    pub fn set_illegal_opcodes(&mut self, enable: bool) {
        self.enable_illegal_opcodes = enable;
    }

    pub fn reset(&mut self) {
        // Reset registers to initial state
        self.registers.a = 0x00;
        self.registers.x = 0x00;
        self.registers.y = 0x00;
        self.registers.pc = self.read_word(RESET_VECTOR);
        self.registers.sp = 0xFD;
        // Set all flags to 0x00, except for the unused flag and the interrupt disable flag
        self.registers.flags = 0x24;
        self.cycles = 8;
    }

    pub fn read(&mut self, address: u16) -> u8 {
        let data = self.bus.read(address);
        return data;
    }

    pub fn read_word(&mut self, address: u16) -> u16 {
        let lo = self.read(address) as u16;
        let hi = self.read(address + 1) as u16;
        return (hi << 8) | lo;
    }

    pub fn write(&mut self, address: u16, data: u8) {
        self.bus.write(address, data);
    }

    pub fn write_word(&mut self, address: u16, data: u16) {
        let lo = (data & 0x00FF) as u8;
        let hi = ((data & 0xFF00) >> 8) as u8;
        self.write(address, lo);
        self.write(address + 1, hi);
    }

    pub fn fetch(&mut self) -> u8 {
        // Set state to Fetching
        self.state = State::Fetching;
        // If the current mode is implied, return 0
        if self.addr_mode != AddressingMode::Implied {
            self.fetched = self.read(self.addr_abs)
        }
        return self.fetched;
    }

    pub fn push(&mut self, data: u8) {
        // Push data to the stack
        self.write(0x0100 + self.registers.sp as u16, data);
        self.registers.decrement_sp();
    }

    pub fn push_word(&mut self, data: u16) {
        // Push data to the stack
        self.push(((data & 0xFF00) >> 8) as u8);
        self.push((data & 0x00FF) as u8);
    }

    pub fn pop(&mut self) -> u8 {
        // Pop data from the stack
        self.registers.increment_sp();
        return self.read(0x0100 + self.registers.sp as u16);
    }

    pub fn pop_word(&mut self) -> u16 {
        // Pop data from the stack
        let lo = self.pop() as u16;
        let hi = self.pop() as u16;
        return (hi << 8) | lo;
    }

    pub fn execute_instruction(&mut self, opcode: u8) -> u8 {
        let instruction = &INSTRUCTION_LIST[opcode as usize];
        (instruction.function)(self)
    }

    pub fn get_operand_string(&mut self, mode: AddressingMode, address: u16) -> String {
        match mode {
            AddressingMode::Implied => return String::from(""),
            AddressingMode::Immediate => {
                // Get the value at the address
                let value = self.read(address);
                // Return the value as decimal
                return format!("#${:02X}", value);
            },
            AddressingMode::ZeroPage => return format!("${:02X}", address),
            AddressingMode::ZeroPageX => return format!("${:02X},X", address),
            AddressingMode::ZeroPageY => return format!("${:02X},Y", address),
            AddressingMode::Relative => return format!("${:02X}", address),
            AddressingMode::Absolute => return format!("${:04X}", address),
            AddressingMode::AbsoluteX => return format!("${:04X},X", address),
            AddressingMode::AbsoluteY => return format!("${:04X},Y", address),
            AddressingMode::Indirect => return format!("(${:04X})", address),
            AddressingMode::IndexedIndirect => return format!("(${:02X},X)", address),
            AddressingMode::IndirectIndexed => return format!("(${:02X}),Y", address),
        }
    }

    pub fn disassemble_instruction_at(&mut self, from_pc: u16) -> String {
        let opcode = self.read(from_pc);
        let instruction = &INSTRUCTION_LIST[opcode as usize];
        let addr_mode = instructions::get_addr_mode(opcode);
        let addr_str = self.get_operand_string(addr_mode, from_pc + 1);
        return format!("{} {}", instruction.name, addr_str);
    }

    pub fn clock(&mut self) {
        // If we have no cycles remaining, fetch the next opcode
        if self.cycles == 0 {
            // Set state to fetching
            self.state = State::Fetching;
            self.current_instruction_string = self.disassemble_instruction_at(self.registers.pc);
            if self.debug {
                println!("{}", self.current_instruction_string);
            }
            self.opcode = self.read(self.registers.pc);
            self.registers.pc += 1;

            // Get the number of cycles for this opcode
            self.cycles = self.get_cycles(self.opcode);

            // Get the addressing mode for this opcode
            let addr_mode = instructions::get_addr_mode(self.opcode);

            // We are now in the executing state
            self.state = State::Executing;

            // Execute the addressing mode function, getting the number of extra cycles required
            let cycles_addr = self.execute_addr_mode(addr_mode);

            // Execute the instruction, getting the number of cycles required
            let cycles_insn = self.execute_instruction(self.opcode);

            // Add the number of cycles required for the addressing mode and the instruction
            self.cycles += cycles_addr + cycles_insn;
        }

        // Decrement the number of cycles remaining
        self.cycles -= 1;
    }

    pub fn execute_addr_mode(&mut self, mode: AddressingMode) -> u8 {
        // Set the addressing mode
        self.addr_mode = mode;

        // Execute the addressing mode
        match mode {
            AddressingMode::Implied => return self.addr_implied(),
            AddressingMode::Immediate => return self.addr_immediate(),
            AddressingMode::ZeroPage => return self.addr_zero_page(),
            AddressingMode::ZeroPageX => return self.addr_zero_page_x(),
            AddressingMode::ZeroPageY => return self.addr_zero_page_y(),
            AddressingMode::Relative => return self.addr_relative(),
            AddressingMode::Absolute => return self.addr_absolute(),
            AddressingMode::AbsoluteX => return self.addr_absolute_x(),
            AddressingMode::AbsoluteY => return self.addr_absolute_y(),
            AddressingMode::Indirect => return self.addr_indirect(),
            AddressingMode::IndexedIndirect => return self.addr_indexed_indirect(),
            AddressingMode::IndirectIndexed => return self.addr_indirect_indexed(),
        }
    }

    pub fn get_cycles(&self, opcode: u8) -> u8 {
        return instructions::get_cycles(opcode);
    }

    pub fn irq(&mut self) {
        // If interrupts are enabled, push the program counter and flags to the stack
        if self
            .registers
            .get_flag(registers::registers::Flag::InterruptDisable)
            == false
        {
            self.push_word(self.registers.pc);

            // Set the break flag to 0
            self.registers
                .set_flag(registers::registers::Flag::Break, false);

            // Push the status flags to the stack
            self.registers
                .set_flag(registers::registers::Flag::Unused, true);
            self.registers
                .set_flag(registers::registers::Flag::Break, true);
            self.registers
                .set_flag(registers::registers::Flag::InterruptDisable, true);
            self.push(self.registers.flags);
            self.registers
                .set_flag(registers::registers::Flag::InterruptDisable, false);

            // Set the program counter to the interrupt vector
            self.registers.pc = self.read_word(IRQ_VECTOR);

            // Set the state to Interrupt
            self.state = State::Interrupt;

            // Set the number of cycles remaining to 7
            self.cycles = 7;
        }
    }

    pub fn nmi(&mut self) {
        // Push the program counter and flags to the stack
        self.push_word(self.registers.pc);

        // Set the break flag to 0
        self.registers
            .set_flag(registers::registers::Flag::Break, false);

        // Push the status flags to the stack
        self.registers
            .set_flag(registers::registers::Flag::Unused, true);
        self.registers
            .set_flag(registers::registers::Flag::Break, true);
        self.registers
            .set_flag(registers::registers::Flag::InterruptDisable, true);
        self.push(self.registers.flags);
        self.registers
            .set_flag(registers::registers::Flag::InterruptDisable, false);

        // Set the program counter to the NMI vector
        self.registers.pc = self.read_word(NMI_VECTOR);

        // Set the state to Interrupt
        self.state = State::Interrupt;

        // Set the number of cycles remaining to 7
        self.cycles = 7;
    }

    pub fn print_instruction_list(&self) {
        instructions::print_instruction_list();
    }

    /**
     * Addressing modes (https://wiki.nesdev.com/w/index.php/CPU_addressing_modes)
     */
    pub fn addr_implied(&mut self) -> u8 {
        self.fetched = self.registers.a;
        return 0;
    }
    pub fn addr_immediate(&mut self) -> u8 {
        self.addr_abs = self.registers.pc;
        self.registers.pc += 1;
        return 0;
    }
    pub fn addr_zero_page(&mut self) -> u8 {
        self.addr_abs = (self.read(self.registers.pc) as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    pub fn addr_zero_page_x(&mut self) -> u8 {
        self.addr_abs = ((self.read(self.registers.pc) as u16) + self.registers.x as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    pub fn addr_zero_page_y(&mut self) -> u8 {
        self.addr_abs = ((self.read(self.registers.pc) as u16) + self.registers.y as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    pub fn addr_relative(&mut self) -> u8 {
        self.addr_rel = self.read(self.registers.pc) as u16;
        self.registers.pc += 1;
        if self.addr_rel & 0x80 != 0 {
            self.addr_rel |= 0xFF00;
        }
        return 0;
    }
    pub fn addr_absolute(&mut self) -> u8 {
        let lo = self.read(self.registers.pc) as u16;
        let hi = self.read(self.registers.pc + 1) as u16;
        self.addr_abs = (hi << 8) | lo;
        self.registers.pc += 2;
        return 0;
    }
    pub fn addr_absolute_x(&mut self) -> u8 {
        let lo = self.read(self.registers.pc) as u16;
        let hi = self.read(self.registers.pc + 1) as u16;
        self.addr_abs = ((hi << 8) | lo) + self.registers.x as u16;
        self.registers.pc += 2;

        // Check if the page changed, and if so, add an extra cycle
        if (self.addr_abs & 0xFF00) != (hi << 8) {
            return 1;
        }
        return 0;
    }
    pub fn addr_absolute_y(&mut self) -> u8 {
        let lo = self.read(self.registers.pc) as u16;
        let hi = self.read(self.registers.pc + 1) as u16;
        self.addr_abs = ((hi << 8) | lo) + self.registers.y as u16;
        self.registers.pc += 2;

        // Check if the page changed, and if so, add an extra cycle
        if (self.addr_abs & 0xFF00) != (hi << 8) {
            return 1;
        }
        return 0;
    }
    pub fn addr_indirect(&mut self) -> u8 {
        let ptr_lo = self.read(self.registers.pc) as u16;
        let ptr_hi = self.read(self.registers.pc + 1) as u16;
        let ptr = (ptr_hi << 8) | ptr_lo;

        // Check for page boundary crossing
        if ptr_lo == 0x00FF {
            // Simulate page boundary hardware bug
            self.addr_abs = (self.read(ptr & 0xFF00) as u16) << 8 | self.read(ptr + 0) as u16;
        } else {
            self.addr_abs = (self.read(ptr + 1) as u16) << 8 | self.read(ptr + 0) as u16;
        }
        self.registers.pc += 2;
        return 0;
    }
    pub fn addr_indexed_indirect(&mut self) -> u8 {
        let t = self.read(self.registers.pc) as u16;
        let lo = self.read((t + self.registers.x as u16) & 0x00FF) as u16;
        let hi = self.read((t + self.registers.x as u16 + 1) & 0x00FF) as u16;
        self.addr_abs = (hi << 8) | lo;
        self.registers.pc += 1;
        return 0;
    }
    pub fn addr_indirect_indexed(&mut self) -> u8 {
        let t = self.read(self.registers.pc) as u16;
        let lo = self.read(t & 0x00FF) as u16;
        let hi = self.read((t + 1) & 0x00FF) as u16;
        self.addr_abs = ((hi << 8) | lo) + self.registers.y as u16;
        self.registers.pc += 1;

        // Check if the page changed, and if so, add an extra cycle
        if (self.addr_abs & 0xFF00) != (hi << 8) {
            return 1;
        }
        return 0;
    }

    /**
     * CPU instructions
     */
    pub fn adc(&mut self) -> u8 {
        let mut extra_cycle: u8 = 0;
        // Fetch the next byte from memory
        self.fetch();

        // Perform the addition
        self.temp = self.registers.a as u16 + self.fetched as u16 + self.registers.get_flag(registers::registers::Flag::Carry) as u16;

        // Set the zero flag if the result is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, (self.temp & 0x00FF) == 0);

        // if the CPU variant is NOT NES, check for decimal flag
        if self.variant != Variant::NES {
            // If the decimal flag is set, perform BCD addition
            if self.registers.get_flag(registers::registers::Flag::DecimalMode) {
                // If the result is greater than 99, set the carry flag
                if self.temp > 99 {
                    self.registers
                        .set_flag(registers::registers::Flag::Carry, true);
                }

                // Set the accumulator to the result modulo 100
                self.registers.a = (self.temp % 100) as u8;

                // Set the negative flag if the result is negative
                self.registers
                    .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

                // Set the overflow flag if the result is greater than 99
                self.registers
                    .set_flag(registers::registers::Flag::Overflow, self.temp > 99);

                // We did decimal addition, so add an extra cycle
                extra_cycle = 1;
            }
        } else {
            // Set the negative flag if the result is negative
            self.registers
                .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

            // Set the overflow flag if the result is greater than 127 or less than -128
            self.registers
                .set_flag(registers::registers::Flag::Overflow, ((self.registers.a ^ self.fetched) & 0x80 == 0) && ((self.fetched ^ self.temp as u8) & 0x80 != 0));

            // Set the carry flag if the result is greater than 255
            self.registers
                .set_flag(registers::registers::Flag::Carry, self.temp > 255);
        }

        // Store the result in the accumulator
        self.registers.a = (self.temp & 0x00FF) as u8;

        // Return the number of cycles required
        return extra_cycle;
    }
    pub fn and(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the AND operation
        self.registers.a &= self.fetched;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.a == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.a & 0x80) > 0,
        );

        // Return the number of cycles required
        return 1;
    }
    pub fn asl(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Shift the fetched byte left by 1 bit
        self.temp = self.fetched as u16;
        self.temp <<= 1;

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0xFF00) > 0);

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, (self.temp & 0x00FF) == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x80) > 0,
        );

        // If we are in implied mode, store the temp variable in the accumulator
        if self.addr_mode == AddressingMode::Implied {
            self.registers.a = (self.temp & 0x00FF) as u8;
        } else {
            // Otherwise, store the temp variable in memory
            self.write(self.addr_abs, (self.temp & 0x00FF) as u8);
        }

        // Return the number of cycles required
        return 0;
    }
    pub fn bcc(&mut self) -> u8 {
        return 0;
    }
    pub fn bcs(&mut self) -> u8 {
        return 0;
    }
    pub fn beq(&mut self) -> u8 {
        // If the zero flag is 1, branch
        if self.registers.get_flag(registers::registers::Flag::Zero) {
            // We branched, so add a cycle
            self.cycles += 1;

            // Calculate the absolute address
            self.addr_abs = self.registers.pc + self.addr_rel;

            // If the page changed, add another cycle
            if (self.addr_abs & 0xFF00) != (self.registers.pc & 0xFF00) {
                self.cycles += 1;
            }

            // Set the program counter to the absolute address
            self.registers.pc = self.addr_abs;
        }

        // Return the number of cycles required
        return 0;
    }
    pub fn bit(&mut self) -> u8 {
        return 0;
    }
    pub fn bmi(&mut self) -> u8 {
        return 0;
    }
    pub fn bne(&mut self) -> u8 {
        // If the zero flag is 0, branch
        if self.registers.get_flag(registers::registers::Flag::Zero) == false {
            // We branched, so add a cycle
            self.cycles += 1;

            // Calculate the absolute address
            self.addr_abs = self.registers.pc + self.addr_rel;

            // If the page changed, add another cycle
            if (self.addr_abs & 0xFF00) != (self.registers.pc & 0xFF00) {
                self.cycles += 1;
            }

            // Set the program counter to the absolute address
            self.registers.pc = self.addr_abs;
        }

        // Return the number of cycles required
        return 0;
    }
    pub fn bpl(&mut self) -> u8 {
        return 0;
    }
    pub fn brk(&mut self) -> u8 {
        // Increment the program counter
        self.registers.pc += 1;

        // Set the interrupt disable flag to 1
        self.registers
            .set_flag(registers::registers::Flag::InterruptDisable, true);

        // Push the PC to the stack
        self.push_word(self.registers.pc);

        // Set the break flag
        self.registers
            .set_flag(registers::registers::Flag::Break, true);

        // Push the flags to the stack
        self.push(self.registers.flags);

        // Clear the break flag
        self.registers
            .set_flag(registers::registers::Flag::Break, false);

        // Set the PC to the data at the interrupt vector
        self.registers.pc = self.read_word(IRQ_VECTOR);

        // Return the number of cycles required
        return 0;
    }
    pub fn bvc(&mut self) -> u8 {
        return 0;
    }
    pub fn bvs(&mut self) -> u8 {
        // If the overflow flag is 1, branch
        if self.registers.get_flag(registers::registers::Flag::Overflow) {
            // We branched, so add a cycle
            self.cycles += 1;

            // Calculate the absolute address
            self.addr_abs = self.registers.pc + self.addr_rel;

            // If the page changed, add another cycle
            if (self.addr_abs & 0xFF00) != (self.registers.pc & 0xFF00) {
                self.cycles += 1;
            }

            // Set the program counter to the absolute address
            self.registers.pc = self.addr_abs;
        }

        // Return the number of cycles required
        return 0;
    }
    pub fn clc(&mut self) -> u8 {
        // Set the carry flag to 0
        self.registers
            .set_flag(registers::registers::Flag::Carry, false);

        // Return the number of cycles required
        return 0;
    }
    pub fn cld(&mut self) -> u8 {
        return 0;
    }
    pub fn cli(&mut self) -> u8 {
        return 0;
    }
    pub fn clv(&mut self) -> u8 {
        return 0;
    }
    pub fn cmp(&mut self) -> u8 {
        return 0;
    }
    pub fn cpx(&mut self) -> u8 {
        return 0;
    }
    pub fn cpy(&mut self) -> u8 {
        return 0;
    }
    pub fn dec(&mut self) -> u8 {
        return 0;
    }
    pub fn dex(&mut self) -> u8 {
        return 0;
    }
    pub fn dey(&mut self) -> u8 {
        return 0;
    }
    pub fn eor(&mut self) -> u8 {
        return 0;
    }
    pub fn inc(&mut self) -> u8 {
        return 0;
    }
    pub fn inx(&mut self) -> u8 {
        return 0;
    }
    pub fn iny(&mut self) -> u8 {
        return 0;
    }
    pub fn jmp(&mut self) -> u8 {
        // Set the program counter to the absolute address
        self.registers.pc = self.addr_abs;

        // Return the number of cycles required
        return 0;
    }
    pub fn jsr(&mut self) -> u8 {
        // Push the program counter to the stack
        self.push_word(self.registers.pc - 1);

        // Set the program counter to the absolute address
        self.registers.pc = self.addr_abs;

        // Return the number of cycles required
        return 0;
    }
    pub fn lda(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Load the fetched byte into the accumulator
        self.registers.a = self.fetched;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.a == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.a & 0x80) > 0,
        );

        // Return the number of cycles required
        return 1;
    }
    pub fn ldx(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Load the fetched byte into the X register
        self.registers.x = self.fetched;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.x == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.x & 0x80) > 0,
        );

        // Return the number of cycles required
        return 1;
    }
    pub fn ldy(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Load the fetched byte into the Y register
        self.registers.y = self.fetched;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.y == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.y & 0x80) > 0,
        );

        // Return the number of cycles required
        return 1;
    }
    pub fn lsr(&mut self) -> u8 {
        return 0;
    }
    pub fn nop(&mut self) -> u8 {
        return 0;
    }
    pub fn ora(&mut self) -> u8 {
        return 0;
    }
    pub fn pha(&mut self) -> u8 {
        // Push the accumulator to the stack
        self.push(self.registers.a);

        // Return the number of cycles required
        return 0;
    }
    pub fn php(&mut self) -> u8 {
        // Push the flags to the stack
        self.push(self.registers.flags);

        // Set break flag to 0
        self.registers
            .set_flag(registers::registers::Flag::Break, false);

        // Set unused flag to 1
        self.registers
            .set_flag(registers::registers::Flag::Unused, true);

        // Return the number of cycles required
        return 0;
    }
    pub fn pla(&mut self) -> u8 {
        // Pop the next byte from the stack into the accumulator
        self.registers.a = self.pop();

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.a == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.a & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }
    pub fn plp(&mut self) -> u8 {
        // Pop the status flags from the stack
        self.registers.flags = self.pop();

        // Set the unused flag to 1
        self.registers
            .set_flag(registers::registers::Flag::Unused, true);

        // Return the number of cycles required
        return 0;
    }
    pub fn rol(&mut self) -> u8 {
        return 0;
    }
    pub fn ror_a(&mut self) -> u8 {
        // If the variant is NMOS, use the NMOS ROR instruction,
        // otherwise use the CMOS ROR instruction
        if self.variant == Variant::NMOS {
            return self.ror_a_nmos();
        } else {
            return self.ror_a_cmos();
        }
    }
    pub fn ror(&mut self) -> u8 {
        // If the variant is NMOS, use the NMOS ROR instruction,
        // otherwise use the CMOS ROR instruction
        if self.variant == Variant::NMOS {
            return self.ror_nmos();
        } else {
            return self.ror_cmos();
        }
    }
    fn ror_a_nmos(&mut self) -> u8 {
        // Load the accumulator into the temporary variable
        self.temp = self.registers.a as u16;

        // Set the 9th bit of the temp variable to 0
        self.temp &= 0x7F;

        // Shift the temp variable left by 1 bit
        self.temp <<= 1;

        // Mask the temp variable to 8 bits
        self.temp &= 0xFF;

        // Set the negative flag if the 8th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

        // Set the zero flag if the temp variable is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.temp == 0x00);

        // If the addressing mode is immediate, store the temp variable in the accumulator
        if self.addr_mode == AddressingMode::Immediate {
            self.registers.a = self.temp as u8;
        } else {
            // Store the temp variable in memory
            self.write(self.addr_abs, self.temp as u8);
        }

        // Return the number of extra cycles required
        return 0;
    }
    fn ror_a_cmos(&mut self) -> u8 {
        // Load the accumulator into the temporary variable
        self.temp = self.registers.a as u16;

        // If the carry flag is set, set the 9th bit of the temp variable
        if self.registers.get_flag(registers::registers::Flag::Carry) {
            self.temp |= 0x100;
        }

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0x01) > 0);

        // Shift the temp variable right by 1 bit
        self.temp >>= 1;

        // Mask the temp variable to 8 bits
        self.temp &= 0xFF;

        // Set the negative flag if the 8th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

        // Set the zero flag if the temp variable is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.temp == 0x00);

        // Store the temp variable in the accumulator
        self.registers.a = self.temp as u8;

        // Return the number of extra cycles required
        return 0;
    }
    fn ror_nmos(&mut self) -> u8 {
        // Load the next byte from memory into the temporary variable
        self.temp = self.fetch() as u16;

        // Set the 9th bit of the temp variable to 0
        self.temp &= 0x7F;

        // Shift the temp variable left by 1 bit
        self.temp <<= 1;

        // Mask the temp variable to 8 bits
        self.temp &= 0xFF;

        // Set the carry flag if the 8th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0x80) > 0);

        // Set the negative flag if the temp variable is 0
        self.registers
            .set_flag(registers::registers::Flag::Negative, self.temp == 0x00);

        // Store the temp variable in memory
        self.write(self.addr_abs, self.temp as u8);

        // Return the number of extra cycles required
        return 0;
    }
    fn ror_cmos(&mut self) -> u8 {
        // Load the next byte from memory into the temporary variable
        self.temp = self.fetch() as u16;

        // If the carry flag is set, set the 9th bit of the temp variable
        if self.registers.get_flag(registers::registers::Flag::Carry) {
            self.temp |= 0x100;
        }

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0x01) > 0);

        // Shift the temp variable right by 1 bit
        self.temp >>= 1;

        // Mask the temp variable to 8 bits
        self.temp &= 0xFF;

        // Set the negative flag if the temp variable is 0
        self.registers
            .set_flag(registers::registers::Flag::Negative, self.temp == 0x00);

        // Set the zero flag if the temp variable is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.temp == 0x00);

        // Store the temp variable in memory
        self.write(self.addr_abs, self.temp as u8);

        // Return the number of extra cycles required
        return 0;
    }
    pub fn rti(&mut self) -> u8 {
        // Pop the status flags from the stack
        self.registers.flags = self.pop();

        self.registers
            .set_flag(registers::registers::Flag::Break, false);
        self.registers
            .set_flag(registers::registers::Flag::Unused, false);

        // Pop the program counter from the stack
        self.registers.pc = self.pop_word();

        // Return the number of cycles required
        return 0;
    }
    pub fn rts(&mut self) -> u8 {
        // Pop the program counter from the stack and increment it
        self.registers.pc = self.pop_word() + 1;

        // Return the number of cycles required
        return 0;
    }
    pub fn sbc(&mut self) -> u8 {
        return 0;
    }
    pub fn sec(&mut self) -> u8 {
        return 0;
    }
    pub fn sed(&mut self) -> u8 {
        return 0;
    }
    pub fn sei(&mut self) -> u8 {
        return 0;
    }
    pub fn sta(&mut self) -> u8 {
        // Store the accumulator in memory
        self.write(self.addr_abs, self.registers.a);

        // Return the number of cycles required
        return 0;
    }
    pub fn stx(&mut self) -> u8 {
        // Store the X register in memory
        self.write(self.addr_abs, self.registers.x);

        // Return the number of cycles required
        return 0;
    }
    pub fn sty(&mut self) -> u8 {
        // Store the Y register in memory
        self.write(self.addr_abs, self.registers.y);

        // Return the number of cycles required
        return 0;
    }
    pub fn tax(&mut self) -> u8 {
        // Load the accumulator into the X register
        self.registers.x = self.registers.a;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.x == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.x & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }
    pub fn tay(&mut self) -> u8 {
        // Load the accumulator into the Y register
        self.registers.y = self.registers.a;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.y == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.y & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }
    pub fn tsx(&mut self) -> u8 {
        // Load the stack pointer into the X register
        self.registers.x = self.registers.sp;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.x == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.x & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }
    pub fn txa(&mut self) -> u8 {
        // Load the X register into the accumulator
        self.registers.a = self.registers.x;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.a == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.a & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }
    pub fn txs(&mut self) -> u8 {
        // Load the X register into the stack pointer
        self.registers.sp = self.registers.x;

        // Return the number of cycles required
        return 0;
    }
    pub fn tya(&mut self) -> u8 {
        // Load the Y register into the accumulator
        self.registers.a = self.registers.y;

        // Set the Zero and Negative flags
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.registers.a == 0x00);
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.registers.a & 0x80) > 0,
        );

        // Return the number of cycles required
        return 0;
    }

    /**
     * Illegal instructions
     */
    pub fn ahx(&mut self) -> u8 {
        return 0;
    }
    pub fn alr(&mut self) -> u8 {
        return 0;
    }
    pub fn anc(&mut self) -> u8 {
        return 0;
    }
    pub fn arr(&mut self) -> u8 {
        return 0;
    }
    pub fn axs(&mut self) -> u8 {
        return 0;
    }
    pub fn dcp(&mut self) -> u8 {
        return 0;
    }
    pub fn isc(&mut self) -> u8 {
        return 0;
    }
    pub fn kil(&mut self) -> u8 {
        return 0;
    }
    pub fn las(&mut self) -> u8 {
        return 0;
    }
    pub fn lax(&mut self) -> u8 {
        return 0;
    }
    pub fn rla(&mut self) -> u8 {
        return 0;
    }
    pub fn rra(&mut self) -> u8 {
        return 0;
    }
    pub fn sax(&mut self) -> u8 {
        return 0;
    }
    pub fn shx(&mut self) -> u8 {
        return 0;
    }
    pub fn shy(&mut self) -> u8 {
        return 0;
    }
    pub fn slo(&mut self) -> u8 {
        return 0;
    }
    pub fn sre(&mut self) -> u8 {
        return 0;
    }
    pub fn tas(&mut self) -> u8 {
        return 0;
    }
    pub fn xaa(&mut self) -> u8 {
        return 0;
    }
}
