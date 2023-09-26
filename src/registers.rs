pub mod registers {

    pub struct Registers {
        pub a: u8,     // Accumulator
        pub x: u8,     // X register
        pub y: u8,     // Y register
        pub pc: u16,   // Program counter
        pub sp: u8,    // Stack pointer
        pub flags: u8, // Status flags
    }

    pub enum Flag {
        Carry = 0b00000001,            // 1 << 0
        Zero = 0b00000010,             // 1 << 1
        InterruptDisable = 0b00000100, // 1 << 2
        DecimalMode = 0b00001000,      // 1 << 3
        Break = 0b00010000,            // 1 << 4
        Unused = 0b00100000,           // 1 << 5
        Overflow = 0b01000000,         // 1 << 6
        Negative = 0b10000000,         // 1 << 7
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

        pub fn set_flag(&mut self, flag: Flag, value: bool) {
            if value {
                self.flags |= flag as u8;
            } else {
                self.flags &= !(flag as u8);
            }
        }

        pub fn get_flag(&self, flag: Flag) -> bool {
            (self.flags & (flag as u8)) > 0
        }

        pub fn increment_sp(&mut self) {
            self.sp = self.sp.wrapping_add(1);
        }

        pub fn decrement_sp(&mut self) {
            self.sp = self.sp.wrapping_sub(1);
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_set_flag() {
            let mut registers = Registers::new();
            registers.set_flag(Flag::Negative, true);
            assert_eq!(registers.get_flag(Flag::Negative), true);
            registers.set_flag(Flag::Negative, false);
            assert_eq!(registers.get_flag(Flag::Negative), false);
        }

        #[test]
        fn test_get_flag() {
            let mut registers = Registers::new();
            registers.set_flag(Flag::Negative, true);
            assert_eq!(registers.get_flag(Flag::Negative), true);
            registers.set_flag(Flag::Negative, false);
            assert_eq!(registers.get_flag(Flag::Negative), false);
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
