#![warn(missing_docs)]
//! A 6502 emulator library written in Rust.
//!
//! Aims to provide a simple, easy-to-use interface for emulating the 6502 CPU.
//! The CPU connects to a bus, and your emulator can define any number of devices
//! on the bus. The CPU can then read and write to these devices.

mod addresses;
pub mod bus;
pub mod device;
mod instructions;
mod registers;

use bus::Bus;
use instructions::INSTRUCTION_LIST;
use registers::registers::Flag;

use crate::{
    addresses::addresses::IRQ_VECTOR, addresses::addresses::NMI_VECTOR,
    addresses::addresses::RESET_VECTOR, instructions::AddressingMode,
    registers::registers::Registers,
};

/// The CPU struct
pub struct Cpu {
    variant: Variant, // CPU variant
    state: State,     // CPU state

    /// CPU registers
    pub registers: Registers,

    /// CPU bus
    pub bus: Box<dyn Bus>, // Bus

    cycles: u8,                // Number of cycles remaining for current instruction
    temp: u16,                 // Temporary storage for various operations
    addr_abs: u16,             // Absolute address
    addr_rel: u16,             // Relative address
    addr_mode: AddressingMode, // Addressing mode
    opcode: u8,                // Current opcode
    fetched: u8,               // Fetched data

    enable_illegal_opcodes: bool, // Enable illegal opcodes

    /// Current disassembled instruction string
    pub current_instruction_string: String,

    debug: bool, // Debug mode
}

/// CPU variants
#[derive(Clone, Copy, PartialEq)]
pub enum Variant {
    /// Modified 65C02 (no ROR bug)
    CMOS,
    /// Modified 2A03 (no decimal mode)
    NES,
    /// Original 6502 (with ROR bug)
    NMOS,
}

/// Variant implementation
impl Variant {
    /// Returns a new Variant from a string
    pub fn from_string(variant: String) -> Self {
        match variant.as_str() {
            "NMOS" => return Self::NMOS,
            "CMOS" => return Self::CMOS,
            "NES" => return Self::NES,
            _ => panic!("Invalid CPU variant"),
        }
    }

    /// Returns a string representation of the variant
    pub fn to_string(&self) -> String {
        match self {
            Self::NMOS => return String::from("NMOS"),
            Self::CMOS => return String::from("CMOS"),
            Self::NES => return String::from("NES"),
        }
    }
}

/// CPU states
#[derive(PartialEq)]
pub enum State {
    /// CPU is stopped
    Stopped,
    /// CPU is fetching an instruction
    Fetching,
    /// CPU is executing an instruction
    Executing,
    /// CPU is handling an interrupt
    Interrupt,
    /// CPU encountered an illegal opcode
    IllegalOpcode,
}

/// CPU implementation
impl Cpu {
    /// Returns a new CPU with the specified bus
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

    /// Changes the CPU variant
    pub fn change_variant(&mut self, variant: Variant) {
        self.variant = variant;
    }

    /// Enables or disables illegal opcodes
    pub fn set_illegal_opcodes(&mut self, enable: bool) {
        self.enable_illegal_opcodes = enable;
    }

    /// Resets the CPU
    pub fn reset(&mut self) {
        // Reset registers to initial state
        self.registers.a = 0x00;
        self.registers.x = 0x00;
        self.registers.y = 0x00;
        self.registers.pc = self.read_word(RESET_VECTOR);
        self.registers.sp = 0xFD;
        // Set all flags to 0x00, except for the unused flag and the interrupt disable flag
        self.registers.flags = 0x00 | Flag::Unused as u8 | Flag::InterruptDisable as u8;
        self.cycles = 8;
    }

    /// Reads a byte from the specified address
    pub fn read(&mut self, address: u16) -> u8 {
        let data = self.bus.read(address);
        return data;
    }

    /// Reads a word from the specified address
    pub fn read_word(&mut self, address: u16) -> u16 {
        let lo = self.read(address) as u16;
        let hi = self.read(address + 1) as u16;
        return (hi << 8) | lo;
    }

    /// Writes a byte to the specified address
    pub fn write(&mut self, address: u16, data: u8) {
        self.bus.write(address, data);
    }

    /// Writes a word to the specified address
    pub fn write_word(&mut self, address: u16, data: u16) {
        let lo = (data & 0x00FF) as u8;
        let hi = ((data & 0xFF00) >> 8) as u8;
        self.write(address, lo);
        self.write(address + 1, hi);
    }

    /// Fetches the next byte from memory
    pub fn fetch(&mut self) -> u8 {
        // Set state to Fetching
        self.state = State::Fetching;
        // If the current mode is implied, return 0
        if self.addr_mode != AddressingMode::Implied {
            self.fetched = self.read(self.addr_abs)
        }
        return self.fetched;
    }

    /// Pushes a byte to the stack
    pub fn push(&mut self, data: u8) {
        // Push data to the stack
        self.write(0x0100 + self.registers.sp as u16, data);
        self.registers.decrement_sp();
    }

    /// Pushes a word to the stack
    pub fn push_word(&mut self, data: u16) {
        // Push data to the stack
        self.push(((data & 0xFF00) >> 8) as u8);
        self.push((data & 0x00FF) as u8);
    }

    /// Pops a byte from the stack
    pub fn pop(&mut self) -> u8 {
        // Pop data from the stack
        self.registers.increment_sp();
        return self.read(0x0100 + self.registers.sp as u16);
    }

    /// Pops a word from the stack
    pub fn pop_word(&mut self) -> u16 {
        // Pop data from the stack
        let lo = self.pop() as u16;
        let hi = self.pop() as u16;
        return (hi << 8) | lo;
    }

    /// Executes an instruction
    pub fn execute_instruction(&mut self, opcode: u8) -> u8 {
        let instruction = &INSTRUCTION_LIST[opcode as usize];
        (instruction.function)(self)
    }

    /// Returns a string representation of the operand
    pub fn get_operand_string(&mut self, mode: AddressingMode, address: u16) -> String {
        match mode {
            AddressingMode::Implied => return String::from(""),
            AddressingMode::Immediate => return format!("#${:02X}", self.read(address)),
            AddressingMode::ZeroPage => return format!("${:02X}", self.read(address)),
            AddressingMode::ZeroPageX => return format!("${:02X},X", self.read(address)),
            AddressingMode::ZeroPageY => return format!("${:02X},Y", self.read(address)),
            AddressingMode::Relative => return format!("${:02X}", self.read(address)),
            AddressingMode::Absolute => return format!("${:04X}", self.read_word(address)),
            AddressingMode::AbsoluteX => return format!("${:04X},X", self.read_word(address)),
            AddressingMode::AbsoluteY => return format!("${:04X},Y", self.read_word(address)),
            AddressingMode::Indirect => return format!("(${:04X})", self.read_word(address)),
            AddressingMode::IndexedIndirect => return format!("(${:02X},X)", self.read(address)),
            AddressingMode::IndirectIndexed => return format!("(${:02X}),Y", self.read(address)),
        }
    }

    /// Disassembles the instruction at the specified address
    pub fn disassemble_instruction_at(&mut self, from_pc: u16) -> String {
        let opcode = self.read(from_pc);
        let instruction = &INSTRUCTION_LIST[opcode as usize];
        let addr_mode = instructions::get_addr_mode(opcode);
        let addr_str = self.get_operand_string(addr_mode, from_pc + 1);
        return format!("{} {}", instruction.name, addr_str);
    }

    /// Executes a single clock cycle
    pub fn clock(&mut self) {
        // If We are in a stopped state, return
        if self.state == State::Stopped {
            return;
        }
        // If we have no cycles remaining, fetch the next opcode
        if self.cycles == 0 {
            // Set state to fetching
            self.state = State::Fetching;

            // Set the current instruction string
            self.current_instruction_string = self.disassemble_instruction_at(self.registers.pc);

            // Fetch the next opcode
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

            if self.debug {
                let reg_status = self.registers.get_status_string();
                // Print the instruction and the registers
                println!(
                    "{:04X}  {:<32} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} {} {:02X}",
                    self.registers.pc,
                    self.current_instruction_string,
                    self.registers.a,
                    self.registers.x,
                    self.registers.y,
                    self.registers.flags,
                    self.registers.sp,
                    reg_status,
                    self.opcode
                );
            }
        }

        // Decrement the number of cycles remaining
        self.cycles -= 1;
    }

    /// Executes the addressing mode function for the current instruction
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

    /// Returns the number of cycles required for the specified opcode
    pub fn get_cycles(&self, opcode: u8) -> u8 {
        return instructions::get_cycles(opcode);
    }

    /// Interrupt request
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

    /// Non-maskable interrupt
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

    /// Prints the instruction list
    pub fn print_instruction_list(&self) {
        instructions::print_instruction_list();
    }

    fn addr_implied(&mut self) -> u8 {
        self.fetched = self.registers.a;
        return 0;
    }
    fn addr_immediate(&mut self) -> u8 {
        self.addr_abs = self.registers.pc;
        self.registers.pc += 1;
        return 0;
    }
    fn addr_zero_page(&mut self) -> u8 {
        self.addr_abs = (self.read(self.registers.pc) as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    fn addr_zero_page_x(&mut self) -> u8 {
        self.addr_abs = ((self.read(self.registers.pc) as u16) + self.registers.x as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    fn addr_zero_page_y(&mut self) -> u8 {
        self.addr_abs = ((self.read(self.registers.pc) as u16) + self.registers.y as u16) & 0x00FF;
        self.registers.pc += 1;
        return 0;
    }
    fn addr_relative(&mut self) -> u8 {
        self.addr_rel = self.read(self.registers.pc) as u16;
        self.registers.pc += 1;
        if self.addr_rel & 0x80 != 0 {
            self.addr_rel |= 0xFF00;
        }
        return 0;
    }
    fn addr_absolute(&mut self) -> u8 {
        let lo = self.read(self.registers.pc) as u16;
        let hi = self.read(self.registers.pc + 1) as u16;
        self.addr_abs = (hi << 8) | lo;
        self.registers.pc += 2;
        return 0;
    }
    fn addr_absolute_x(&mut self) -> u8 {
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
    fn addr_absolute_y(&mut self) -> u8 {
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
    fn addr_indirect(&mut self) -> u8 {
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
    fn addr_indexed_indirect(&mut self) -> u8 {
        let t = self.read(self.registers.pc) as u16;
        let lo = self.read((t + self.registers.x as u16) & 0x00FF) as u16;
        let hi = self.read((t + self.registers.x as u16 + 1) & 0x00FF) as u16;
        self.addr_abs = (hi << 8) | lo;
        self.registers.pc += 1;
        return 0;
    }
    fn addr_indirect_indexed(&mut self) -> u8 {
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

    /// Adds the value of the accumulator and the carry flag to a memory value, and sets the
    /// accumulator to the result. Decimal mode is supported on the original NMOS 6502 and the 65C02.
    fn adc(&mut self) -> u8 {
        let mut extra_cycle: u8 = 0;

        // Fetch the next byte from memory
        self.fetch();

        // Perform the addition
        self.temp = self.registers.a as u16
            + self.fetched as u16
            + self.registers.get_flag(registers::registers::Flag::Carry) as u16;

        // Set the zero flag if the result is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, (self.temp & 0x00FF) == 0);

        // if the CPU variant is NOT NES, check for decimal flag
        if self.variant != Variant::NES {
            // If the decimal flag is set, perform BCD addition
            if self
                .registers
                .get_flag(registers::registers::Flag::DecimalMode)
            {
                // If the result is greater than 99, add 96 to it
                if (self.registers.a & 0xF)
                    + (self.fetched & 0xF)
                    + (self.registers.get_flag(registers::registers::Flag::Carry) as u8)
                    > 9
                {
                    self.temp += 6;
                }

                // Set the negative flag if the result is negative
                self.registers
                    .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

                // Set the overflow flag if the result is greater than 127 or less than -128
                self.registers.set_flag(
                    registers::registers::Flag::Overflow,
                    ((self.registers.a ^ self.fetched) & 0x80 == 0)
                        && ((self.fetched ^ self.temp as u8) & 0x80 != 0),
                );

                // If the result is greater than 99, add 96 to it
                if self.temp > 99 {
                    self.temp += 96;
                }

                // Set the carry flag if the result is greater than 0x99
                self.registers
                    .set_flag(registers::registers::Flag::Carry, self.temp > 0x99FF);

                // We used an extra cycle
                extra_cycle = 1;
            }
        } else {
            // Set the negative flag if the result is negative
            self.registers
                .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

            // Set the overflow flag if the result is greater than 127 or less than -128
            self.registers.set_flag(
                registers::registers::Flag::Overflow,
                ((self.registers.a ^ self.fetched) & 0x80 == 0)
                    && ((self.fetched ^ self.temp as u8) & 0x80 != 0),
            );

            // Set the carry flag if the result is greater than 255
            self.registers
                .set_flag(registers::registers::Flag::Carry, self.temp > 255);
        }

        // Store the result in the accumulator
        self.registers.a = (self.temp & 0x00FF) as u8;

        // Return the number of cycles required
        return extra_cycle;
    }

    /// Performs a bitwise AND operation between the accumulator and a value in memory,
    /// and stores the result in the accumulator.
    fn and(&mut self) -> u8 {
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

    /// Shifts the accumulator left by one bit, setting the carry flag if the most significant bit is set.
    /// Returns the new value of the accumulator.
    fn asl(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Shift the fetched byte left by 1 bit
        self.temp = self.fetched as u16;
        self.temp <<= 1;

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0xFF00) > 0);

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x00,
        );
        self.registers
            .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

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

    /// Branches to the address specified by the relative offset if the carry flag is clear.
    fn bcc(&mut self) -> u8 {
        // If the carry flag is 0, branch
        if self.registers.get_flag(registers::registers::Flag::Carry) == false {
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

        return 0;
    }

    /// Branches to the address specified by the relative offset if the carry flag is set.
    fn bcs(&mut self) -> u8 {
        // If the carry flag is 1, branch
        if self.registers.get_flag(registers::registers::Flag::Carry) {
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

        return 0;
    }

    /// Branches to the address specified by the relative offset if the zero flag is set.
    ///
    /// # Returns
    ///
    /// The number of cycles the operation took.
    fn beq(&mut self) -> u8 {
        // If the zero flag is 1, branch
        if self.registers.get_flag(registers::registers::Flag::Zero) {
            // We branched, so add a cycle
            self.cycles += 1;

            // Calculate the absolute address using wrapping_add
            self.addr_abs = self.registers.pc.wrapping_add(self.addr_rel);

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

    /// Test bits in memory with the accumulator.
    ///
    /// # Returns
    ///
    /// The number of cycles required.
    ///
    /// # Flags
    ///
    /// The negative flag is set to the seventh bit of the fetched value.
    /// The overflow flag is set to the sixth bit of the fetched value.
    /// The zero flag is set to the result of the AND operation between the fetched value and the accumulator.
    fn bit(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the AND operation
        self.temp = self.registers.a as u16 & self.fetched as u16;

        // Set the negative flag to the seventh bit of the fetched value
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.fetched & 0x80) > 0,
        );

        // Set the overflow flag to the sixth bit of the fetched value
        self.registers.set_flag(
            registers::registers::Flag::Overflow,
            (self.fetched & 0x40) > 0,
        );

        // Set the Zero flag to the result of the AND operation
        self.registers
            .set_flag(registers::registers::Flag::Zero, self.temp == 0x00);

        // Return the number of cycles required
        return 0;
    }

    /// Branches to the address specified by the relative offset if the negative flag is set.
    fn bmi(&mut self) -> u8 {
        // If the negative flag is 1, branch
        if self
            .registers
            .get_flag(registers::registers::Flag::Negative)
        {
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

    /// Branches to the address specified by the relative offset if the zero flag is not set.
    ///
    /// # Returns
    ///
    /// The number of cycles the operation took.
    fn bne(&mut self) -> u8 {
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

    /// Branches to the address specified by the relative offset if the negative flag is not set.
    fn bpl(&mut self) -> u8 {
        // If the negative flag is 0, branch
        if self
            .registers
            .get_flag(registers::registers::Flag::Negative)
            == false
        {
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

    /// Break instruction, which generates an interrupt request and pushes the program counter
    /// and status register to the stack. Returns the number of cycles taken.
    fn brk(&mut self) -> u8 {
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

    /// Branch if overflow flag is clear
    fn bvc(&mut self) -> u8 {
        // If the overflow flag is 0, branch
        if self
            .registers
            .get_flag(registers::registers::Flag::Overflow)
            == false
        {
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

    /// Branch if overflow flag is set
    fn bvs(&mut self) -> u8 {
        // If the overflow flag is 1, branch
        if self
            .registers
            .get_flag(registers::registers::Flag::Overflow)
        {
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

    /// Clear the carry flag
    fn clc(&mut self) -> u8 {
        // Set the carry flag to 0
        self.registers
            .set_flag(registers::registers::Flag::Carry, false);

        // Return the number of cycles required
        return 0;
    }

    /// Clear the decimal mode flag
    fn cld(&mut self) -> u8 {
        // Set the decimal mode flag to 0
        self.registers
            .set_flag(registers::registers::Flag::DecimalMode, false);

        // Return the number of cycles required
        return 0;
    }

    /// Clear the interrupt disable flag
    fn cli(&mut self) -> u8 {
        // Set the interrupt disable flag to 0
        self.registers
            .set_flag(registers::registers::Flag::InterruptDisable, false);

        // Return the number of cycles required
        return 0;
    }

    /// Clear the overflow flag
    fn clv(&mut self) -> u8 {
        // Set the overflow flag to 0
        self.registers
            .set_flag(registers::registers::Flag::Overflow, false);

        // Return the number of cycles required
        return 0;
    }

    /// Compares the accumulator with a value in memory.
    fn cmp(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the comparison with wrapping_sub
        self.temp = self.registers.a as u16;
        self.temp = self.temp.wrapping_sub(self.fetched as u16);

        // Set the carry flag if the accumulator is greater than or equal to the fetched value
        self.registers.set_flag(
            registers::registers::Flag::Carry,
            self.registers.a >= self.fetched,
        );

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x0080) > 0,
        );

        // Return the number of cycles required
        return 1;
    }

    /// Compares the X register with a value in memory.
    fn cpx(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the comparison with wrapping_sub
        self.temp = self.registers.x as u16;
        self.temp = self.temp.wrapping_sub(self.fetched as u16);

        // Set the carry flag if the X register is greater than or equal to the fetched value
        self.registers.set_flag(
            registers::registers::Flag::Carry,
            self.registers.x >= self.fetched,
        );

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x0080) > 0,
        );

        // Return the number of cycles required
        return 0;
    }

    /// Compares the Y register with a value in memory.
    fn cpy(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the comparison with wrapping_sub
        self.temp = self.registers.y as u16;
        self.temp = self.temp.wrapping_sub(self.fetched as u16);

        // Set the carry flag if the Y register is greater than or equal to the fetched value
        self.registers.set_flag(
            registers::registers::Flag::Carry,
            self.registers.y >= self.fetched,
        );

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x0080) > 0,
        );

        // Return the number of cycles required
        return 0;
    }

    /// Decrements a value in memory by one.
    fn dec(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Decrement the fetched value
        self.temp = self.fetched as u16;
        self.temp = self.temp.wrapping_sub(1);

        // Store the decremented value in memory
        self.write(self.addr_abs, (self.temp & 0x00FF) as u8);

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x0080) > 0,
        );

        // Return the number of cycles required
        return 0;
    }

    /// Decrements the X register by one.
    fn dex(&mut self) -> u8 {
        // Decrement the X register
        self.registers.x = self.registers.x.wrapping_sub(1);

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

    /// Decrements the Y register by one.
    fn dey(&mut self) -> u8 {
        // Decrement the Y register
        self.registers.y = self.registers.y.wrapping_sub(1);

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

    fn eor(&mut self) -> u8 {
        return 0;
    }

    /// Increments a value in memory by one.
    fn inc(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Increment the fetched value
        self.temp = self.fetched as u16;
        self.temp = self.temp.wrapping_add(1);

        // Store the incremented value in memory
        self.write(self.addr_abs, (self.temp & 0x00FF) as u8);

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers.set_flag(
            registers::registers::Flag::Negative,
            (self.temp & 0x0080) > 0,
        );

        // Return the number of cycles required
        return 0;
    }

    /// Increments the X register by one.
    fn inx(&mut self) -> u8 {
        // Increment the X register
        self.registers.x = self.registers.x.wrapping_add(1);

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

    /// Increments the Y register by one.
    fn iny(&mut self) -> u8 {
        // Increment the Y register
        self.registers.y = self.registers.y.wrapping_add(1);

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

    /// Jumps to the address specified by the program counter.
    ///
    /// # Returns
    ///
    /// The number of cycles it took to execute the instruction.
    fn jmp(&mut self) -> u8 {
        // Set the program counter to the absolute address
        self.registers.pc = self.addr_abs;

        // Return the number of cycles required
        return 0;
    }

    /// The JSR instruction pushes the address (minus one) of the return
    /// point on to the stack and then sets the program counter to the target memory address.
    fn jsr(&mut self) -> u8 {
        // Push the program counter to the stack
        self.push_word(self.registers.pc - 1);

        // Set the program counter to the absolute address
        self.registers.pc = self.addr_abs;

        // Return the number of cycles required
        return 0;
    }

    /// Loads the accumulator with a value from memory.
    fn lda(&mut self) -> u8 {
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

    /// Load the X register with a byte of memory.
    fn ldx(&mut self) -> u8 {
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

    /// Loads the Y register with an 8-bit value from memory.
    fn ldy(&mut self) -> u8 {
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

    /// Shifts the accumulator left by one bit, setting the carry flag if the most significant bit is set.
    fn lsr(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Shift the fetched byte right by 1 bit
        self.temp = self.fetched as u16;
        self.temp >>= 1;

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0x01) > 0);

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers
            .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

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

    /// No-operation
    fn nop(&mut self) -> u8 {
        return 0;
    }

    /// Performs a bitwise OR operation between the accumulator and a value in memory,
    fn ora(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Perform the OR operation
        self.registers.a |= self.fetched;

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

    /// Push the accumulator onto the stack
    fn pha(&mut self) -> u8 {
        // Push the accumulator to the stack
        self.push(self.registers.a);

        // Return the number of cycles required
        return 0;
    }

    /// Push Processor Status
    ///
    /// Pushes a copy of the status flags on to the stack.
    ///
    /// Flags: N V - B D I Z C
    ///
    /// The flags are pushed onto the stack in this order:
    ///
    ///     1. Break flag is set to 1
    ///     2. Unused flag is set to 1
    ///     3. Interrupt flag
    ///     4. Zero flag
    ///     5. Negative flag
    ///
    /// The processor status register is not affected by this operation.
    fn php(&mut self) -> u8 {
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

    /// Pulls a byte from the stack and loads it into the accumulator.
    fn pla(&mut self) -> u8 {
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

    /// Pull Processor Status from stack
    fn plp(&mut self) -> u8 {
        // Pop the status flags from the stack
        self.registers.flags = self.pop();

        // Set the unused flag to 1
        self.registers
            .set_flag(registers::registers::Flag::Unused, true);

        // Return the number of cycles required
        return 0;
    }

    /// Rotate the accumulator left by one bit, with the carry flag replacing the bit that is shifted out.
    fn rol(&mut self) -> u8 {
        // Fetch the next byte from memory
        self.fetch();

        // Shift the fetched byte left by 1 bit
        self.temp = self.fetched as u16;
        self.temp <<= 1;

        // Set the 0th bit of the temp variable to the value of the carry flag
        if self.registers.get_flag(registers::registers::Flag::Carry) {
            self.temp |= 0x01;
        }

        // Set the carry flag if the 9th bit of the temp variable is 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, (self.temp & 0xFF00) > 0);

        // Set the Zero and Negative flags
        self.registers.set_flag(
            registers::registers::Flag::Zero,
            (self.temp & 0x00FF) == 0x0000,
        );
        self.registers
            .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

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

    /// Rotate the accumulator right by one bit, with the carry flag replacing the bit that is shifted out.
    /// Returns the new value of the accumulator.
    ///
    /// This function calls the NMOS ROR instruction if the CPU variant is NMOS,
    /// otherwise it calls the CMOS ROR instruction.
    fn ror_a(&mut self) -> u8 {
        // If the variant is NMOS, use the NMOS ROR instruction,
        // otherwise use the CMOS ROR instruction
        if self.variant == Variant::NMOS {
            return self.ror_a_nmos();
        } else {
            return self.ror_a_cmos();
        }
    }

    /// Rotate one bit right in memory, then shift right the entire byte.
    ///
    /// Flags affected:
    /// - N: Set if the resulting value has bit 7 set.
    /// - Z: Set if the resulting value is 0.
    /// - C: The carry flag is set to the value of the bit that was shifted out.
    ///
    /// This function calls the NMOS ROR instruction if the CPU variant is NMOS,
    /// otherwise it calls the CMOS ROR instruction.
    fn ror(&mut self) -> u8 {
        // If the variant is NMOS, use the NMOS ROR instruction,
        // otherwise use the CMOS ROR instruction
        if self.variant == Variant::NMOS {
            return self.ror_nmos();
        } else {
            return self.ror_cmos();
        }
    }

    /// Rotate the accumulator right by one bit, with the bit that was shifted out
    /// being shifted into the carry flag.
    ///
    /// This is the NMOS version of the ROR instruction, so it's actually a LSR instruction.
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

    /// Rotate the accumulator right by one bit, with the bit that was shifted
    /// out being shifted into the carry flag.
    ///
    /// This is the CMOS version of the ROR instruction.
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

    /// Rotate right (memory) with carry, using the NMOS 6502 processor's logic.
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

    /// Rotate the bits of the accumulator right by one bit, with the bit that was shifted
    /// out being shifted into the carry flag.
    ///
    /// This is the CMOS version of the ROR instruction.
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

    /// Return from Interrupt
    ///
    /// Pulls an 8-bit value from the stack and into the program counter (low byte),
    /// then another 8-bit value and into the program counter (high byte).
    ///
    /// # Returns
    ///
    /// The number of cycles this operation took.
    fn rti(&mut self) -> u8 {
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

    /// Return from Subroutine
    ///
    /// Pulls the program counter (PC) from the stack and sets it to the address of the
    /// instruction that follows the subroutine call.
    fn rts(&mut self) -> u8 {
        // Pop the program counter from the stack and increment it
        self.registers.pc = self.pop_word() + 1;

        // Return the number of cycles required
        return 0;
    }

    fn sbc(&mut self) -> u8 {
        let mut extra_cycle: u8 = 0;

        // Fetch the next byte from memory
        self.fetch();

        // Perform the subtraction using wrapping_sub
        self.temp = (self.registers.a as u16)
            .wrapping_sub(self.fetched as u16)
            .wrapping_sub(1 - self.registers.get_flag(registers::registers::Flag::Carry) as u16);

        // Set the zero flag if the result is 0
        self.registers
            .set_flag(registers::registers::Flag::Zero, (self.temp & 0x00FF) == 0);

        // If the CPU variant is NOT NES
        if self.variant != Variant::NES {
            // If the decimal flag is set, perform BCD subtraction
            if self
                .registers
                .get_flag(registers::registers::Flag::DecimalMode)
            {
                // Adjust the result for BCD
                if (self.temp & 0x000F) > 0x0009 {
                    self.temp -= 0x0006;
                }

                // Set the negative flag if the result is negative
                self.registers.set_flag(
                    registers::registers::Flag::Negative,
                    (self.temp & 0x0080) > 0,
                );

                // Set the overflow flag if the result is greater than 127 or less than -128
                self.registers.set_flag(
                    registers::registers::Flag::Overflow,
                    ((self.registers.a ^ self.temp as u8)
                        & (self.fetched ^ self.temp as u8)
                        & 0x80)
                        > 0,
                );

                // Adjust the result for BCD
                if self.temp > 0x99FF {
                    self.temp += 0x0060;
                }

                // Set the carry flag if the result is greater than 127 or less than -128
                self.registers
                    .set_flag(registers::registers::Flag::Carry, self.temp > 0x99FF);

                // We used an extra cycle
                extra_cycle = 1;
            }
        } else {
            // Set the negative flag if the result is negative
            self.registers
                .set_flag(registers::registers::Flag::Negative, (self.temp & 0x80) > 0);

            // Set the overflow flag if the result is greater than 127 or less than -128
            self.registers.set_flag(
                registers::registers::Flag::Overflow,
                ((self.registers.a ^ self.fetched) & 0x80 != 0)
                    && ((self.fetched ^ self.temp as u8) & 0x80 != 0),
            );

            // Set the carry flag if the result is greater than 255
            self.registers
                .set_flag(registers::registers::Flag::Carry, self.temp > 255);
        }

        // Store the result in the accumulator
        self.registers.a = (self.temp & 0x00FF) as u8;

        // Return the number of cycles required
        return extra_cycle;
    }

    /// Set the carry flag
    fn sec(&mut self) -> u8 {
        // Set the carry flag to 1
        self.registers
            .set_flag(registers::registers::Flag::Carry, true);

        // Return the number of cycles required
        return 0;
    }

    /// Set the decimal mode flag
    fn sed(&mut self) -> u8 {
        // Set the decimal mode flag to 1
        self.registers
            .set_flag(registers::registers::Flag::DecimalMode, true);

        // Return the number of cycles required
        return 0;
    }

    /// Set the interrupt disable flag
    fn sei(&mut self) -> u8 {
        // Set the interrupt disable flag to 1
        self.registers
            .set_flag(registers::registers::Flag::InterruptDisable, true);

        // Return the number of cycles required
        return 0;
    }

    /// Stores the accumulator register value into memory at the specified address.
    fn sta(&mut self) -> u8 {
        // Store the accumulator in memory
        self.write(self.addr_abs, self.registers.a);

        // Return the number of cycles required
        return 0;
    }

    /// Stores the value of the X register into memory.
    fn stx(&mut self) -> u8 {
        // Store the X register in memory
        self.write(self.addr_abs, self.registers.x);

        // Return the number of cycles required
        return 0;
    }

    /// Stores the value of the X register into memory.
    fn sty(&mut self) -> u8 {
        // Store the Y register in memory
        self.write(self.addr_abs, self.registers.y);

        // Return the number of cycles required
        return 0;
    }

    /// Transfers the accumulator to the X register.
    fn tax(&mut self) -> u8 {
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

    /// Transfers the accumulator to the Y register.
    fn tay(&mut self) -> u8 {
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

    /// Transfers the stack pointer to the X register.
    fn tsx(&mut self) -> u8 {
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

    /// Transfers the X register to the accumulator.
    fn txa(&mut self) -> u8 {
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

    /// Transfers the X register to the stack pointer.
    fn txs(&mut self) -> u8 {
        // Load the X register into the stack pointer
        self.registers.sp = self.registers.x;

        // Return the number of cycles required
        return 0;
    }

    /// Transfers the Y register to the accumulator.
    fn tya(&mut self) -> u8 {
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
    fn ahx(&mut self) -> u8 {
        return 0;
    }
    fn alr(&mut self) -> u8 {
        return 0;
    }
    fn anc(&mut self) -> u8 {
        return 0;
    }
    fn arr(&mut self) -> u8 {
        return 0;
    }
    fn axs(&mut self) -> u8 {
        return 0;
    }
    fn dcp(&mut self) -> u8 {
        return 0;
    }
    fn isc(&mut self) -> u8 {
        return 0;
    }
    fn kil(&mut self) -> u8 {
        // KIL instruction
        // TODO: Implement this
        // For now, just halt the CPU
        self.state = State::Stopped;
        return 0;
    }
    fn las(&mut self) -> u8 {
        return 0;
    }
    fn lax(&mut self) -> u8 {
        return 0;
    }
    fn rla(&mut self) -> u8 {
        return 0;
    }
    fn rra(&mut self) -> u8 {
        return 0;
    }
    fn sax(&mut self) -> u8 {
        return 0;
    }
    fn shx(&mut self) -> u8 {
        return 0;
    }
    fn shy(&mut self) -> u8 {
        return 0;
    }
    fn slo(&mut self) -> u8 {
        return 0;
    }
    fn sre(&mut self) -> u8 {
        return 0;
    }
    fn tas(&mut self) -> u8 {
        return 0;
    }
    fn xaa(&mut self) -> u8 {
        return 0;
    }
}
