//! # Bus
//! 
//! The bus is the component that connects all the devices together. It is responsible for
//! reading and writing to the devices. It also loads the ROM into memory.
use crate::device::Device;

/// Trait representing a bus that can be read from and written to. Generally you will want to
/// only use a single bus for your emulator.
pub trait Bus {
    /// Adds a device to the bus.
    ///
    /// # Arguments
    ///
    /// * `device` - A boxed trait object that implements the `Device` trait.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cpu6502::bus::Bus;
    /// # use cpu6502::device::Device;
    /// # struct TestDevice;
    /// # impl Device for TestDevice {}
    /// let mut bus = Bus::new();
    /// let device = Box::new(TestDevice {});
    /// bus.add_device(device);
    /// ```
    fn add_device(&mut self, device: Box<dyn Device>);
    fn read(&mut self, address: u16) -> u8;
    fn write(&mut self, address: u16, value: u8);
    fn load_rom_at(&mut self, rom: &[u8], address: u16);
}

/// A default implementation of a bus.
pub struct DefaultBus;

/// Implements the Bus trait for the DefaultBus struct.
impl Bus for DefaultBus {
    fn add_device(&mut self, _device: Box<dyn Device>) {
        println!("Adding device");
    }

    fn read(&mut self, address: u16) -> u8 {
        println!("Bus Reading from address: {:#06X}", address);
        0
    }

    fn write(&mut self, address: u16, value: u8) {
        println!("Bus Writing to address: {:#06X} value: {:#04X}", address, value);
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