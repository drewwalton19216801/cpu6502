//! # Registers
//!
//! This module contains the registers for the 6502 CPU.
//!
//! It contains the following registers:
//! - Accumulator
//! - X register
//! - Y register
//! - Program counter
//! - Stack pointer
//! - Status flags

pub mod registers {

    pub struct Registers {
        pub a: u8,     // Accumulator
        pub x: u8,     // X register
        pub y: u8,     // Y register
        pub pc: u16,   // Program counter
        pub sp: u8,    // Stack pointer
        pub flags: u8, // Status flags
    }

    bitflags! {
        pub struct Flags: u8 {
            const None = 0b00000000;
            const Carry = 0b00000001;
            const Zero = 0b00000010;
            const InterruptDisable = 0b00000100;
            const DecimalMode = 0b00001000;
            const Break = 0b00010000;
            const Unused = 0b00100000;
            const Overflow = 0b01000000;
            const Negative = 0b10000000;
        }
    }

    impl Registers {
        pub fn new() -> Self {
            Self {
                a: 0x00,
                x: 0x00,
                y: 0x00,
                pc: 0x0000,
                sp: 0x00,
                flags: 0x00,
            }
        }

        pub fn set_flag(&mut self, flag: Flags, value: bool) {
            if value {
                self.flags |= flag.bits();
            } else {
                self.flags &= !flag.bits();
            }
        }

        pub fn get_flag(&mut self, flag: Flags) -> bool {
            self.flags & flag.bits() != 0
        }

        pub fn increment_sp(&mut self) {
            self.sp = self.sp.wrapping_add(1);
        }

        pub fn decrement_sp(&mut self) {
            self.sp = self.sp.wrapping_sub(1);
        }

        pub fn get_status_string(&mut self) -> String {
            let mut status = String::new();
            status.push_str("STATUS: ");
            status.push_str(if self.get_flag(Flags::Negative) {
                "N"
            } else {
                "n"
            });
            status.push_str(if self.get_flag(Flags::Overflow) {
                "V"
            } else {
                "v"
            });
            status.push_str("-");
            status.push_str(if self.get_flag(Flags::Break) {
                "B"
            } else {
                "b"
            });
            status.push_str(if self.get_flag(Flags::DecimalMode) {
                "D"
            } else {
                "d"
            });
            status.push_str(if self.get_flag(Flags::InterruptDisable) {
                "I"
            } else {
                "i"
            });
            status.push_str(if self.get_flag(Flags::Zero) { "Z" } else { "z" });
            status.push_str(if self.get_flag(Flags::Carry) {
                "C"
            } else {
                "c"
            });
            status
        }
    }

    #[cfg(test)]
    mod tests {
        use crate::registers;

        use super::*;

        #[test]
        fn test_flags() {
            let mut registers = Registers::new();
            registers.set_flag(registers::registers::Flags::Carry, true);
            registers.set_flag(registers::registers::Flags::Zero, true);
            registers.set_flag(registers::registers::Flags::Negative, true);

            assert_eq!(registers.get_flag(registers::registers::Flags::Carry), true);
            assert_eq!(registers.get_flag(registers::registers::Flags::Zero), true);
            assert_eq!(
                registers.get_flag(registers::registers::Flags::Negative),
                true
            )
        }

        #[test]
        fn test_increment_sp() {
            let mut registers = Registers::new();
            registers.sp = 0x01;
            registers.increment_sp();
            assert_eq!(registers.sp, 0x02);
        }

        #[test]
        fn test_decrement_sp() {
            let mut registers = Registers::new();
            registers.sp = 0x02;
            registers.decrement_sp();
            assert_eq!(registers.sp, 0x01);
        }

        #[test]
        fn test_get_status_string() {
            let mut registers = Registers::new();
            // Set flags to random values
            registers.set_flag(registers::registers::Flags::Negative, true);
            registers.set_flag(registers::registers::Flags::Overflow, true);
            registers.set_flag(registers::registers::Flags::Break, false);
            registers.set_flag(registers::registers::Flags::DecimalMode, true);
            registers.set_flag(registers::registers::Flags::InterruptDisable, false);
            registers.set_flag(registers::registers::Flags::Zero, true);
            registers.set_flag(registers::registers::Flags::Carry, false);
            assert_eq!(registers.get_status_string(), "STATUS: NV-bDiZc");
        }

        #[test]
        fn test_set_registers() {
            let mut registers = Registers::new();
            registers.a = 0x01;
            registers.x = 0x02;
            registers.y = 0x03;
            registers.pc = 0x0405;
            registers.sp = 0x06;
            registers.flags = 0x07;
            assert_eq!(registers.a, 0x01);
            assert_eq!(registers.x, 0x02);
            assert_eq!(registers.y, 0x03);
            assert_eq!(registers.pc, 0x0405);
            assert_eq!(registers.sp, 0x06);
            assert_eq!(registers.flags, 0x07);
        }
    }
}
