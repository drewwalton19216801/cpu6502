use crate::device::Device;

pub trait Bus {
    fn add_device(&mut self, device: Box<dyn Device>);
    fn read(&mut self, address: u16) -> u8;
    fn write(&mut self, address: u16, value: u8);
    fn load_rom_at(&mut self, rom: &[u8], address: u16);
}

pub struct DefaultBus;

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

impl Default for DefaultBus {
    fn default() -> Self {
        Self {}
    }
}