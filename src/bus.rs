//! # Bus
//!
//! The bus is the component that connects all the devices together. It is responsible for
//! reading and writing to the devices.

/// Trait representing a bus that can be read from and written to. Generally you will want to
/// only use a single bus for your emulator.
pub trait Bus {
    /// Reads a byte from the bus at the specified address.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to read from.
    ///
    /// # Returns
    ///
    /// The byte read from the bus.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cpu6502::bus::Bus;
    /// let mut bus = Bus::new();
    /// let value = bus.read(0x0000);
    /// ```
    ///
    /// # Notes
    ///
    /// This function will call the `read()` function on the device that contains the address.
    ///
    /// # See Also
    ///
    /// * `write()`
    /// * `load_rom_at()`
    fn read(&mut self, address: u16) -> u8;

    /// Writes a byte to the bus at the specified address.
    ///
    /// # Arguments
    ///
    /// * `address` - The address to write to.
    /// * `value` - The value to write.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cpu6502::bus::Bus;
    /// let mut bus = Bus::new();
    /// bus.write(0x0000, 0x00);
    /// ```
    ///
    /// # Notes
    ///
    /// This function will call the `write()` function on the device that contains the address.
    fn write(&mut self, address: u16, value: u8);

    /// Loads a ROM into memory at the specified address.
    ///
    /// # Arguments
    ///
    /// * `rom` - The ROM to load.
    /// * `address` - The address to load the ROM at.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cpu6502::bus::Bus;
    /// let mut bus = Bus::new();
    /// let rom = [0x00, 0x01, 0x02, 0x03];
    /// bus.load_rom_at(&rom, 0x0000);
    /// ```
    ///
    /// # Notes
    ///
    /// This function will call the `load_rom_at()` function on the device that contains the address.
    fn load_rom_at(&mut self, rom: &[u8], address: u16);
}

/// A default implementation of a bus.
pub struct DefaultBus;

/// Implements the Bus trait for the DefaultBus struct.
impl Bus for DefaultBus {
    fn read(&mut self, address: u16) -> u8 {
        println!("Bus Reading from address: {:#06X}", address);
        0
    }

    fn write(&mut self, address: u16, value: u8) {
        println!(
            "Bus Writing to address: {:#06X} value: {:#04X}",
            address, value
        );
    }

    fn load_rom_at(&mut self, _rom: &[u8], address: u16) {
        println!("Loading ROM at address: {:#06X}", address);
    }
}

/// Implements the default behavior for the `DefaultBus` struct.
impl Default for DefaultBus {
    fn default() -> Self {
        Self {}
    }
}
