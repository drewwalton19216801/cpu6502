use std::{cell::RefCell, sync::Arc};

#[derive(Clone)]
pub struct Bus {
    // 64KB memory space
    pub memory: Box<[u8; 64 * 1024]>,

    pub hooks: Vec<Option<Hook>>,
}

#[derive(Clone)]
pub struct Hook {
    pub read: Option<Arc<RefCell<dyn FnMut(u16) -> u8>>>,
    pub write: Option<Arc<RefCell<dyn FnMut(u16, u8)>>>,
}

impl Bus {
    pub fn new() -> Self {
        Self {
            memory: Box::new([0; 64 * 1024]),
            hooks: vec![None; 64 * 1024],
        }
    }

    #[allow(dead_code)]
    pub fn add_hook(&mut self, address: u16, hook: Hook) {
        self.hooks[address as usize] = Some(hook);
    }

    #[allow(dead_code)]
    pub fn add_hook_range(&mut self, range: std::ops::Range<u16>, hook: Hook) {
        for address in range {
            self.hooks[address as usize] = Some(hook.clone());
        }
    }

    #[allow(dead_code)]
    pub fn bus_read(&mut self, address: u16) -> u8 {
        if let Some(hook) = &mut self.hooks[address as usize] {
            if let Some(read) = &mut hook.read {
                return read.borrow_mut()(address);
            }
        }

        self.memory[address as usize]
    }

    #[allow(dead_code)]
    pub fn bus_write(&mut self, address: u16, value: u8) {
        if let Some(hook) = &mut self.hooks[address as usize] {
            if let Some(write) = &mut hook.write {
                return write.borrow_mut()(address, value);
            }
        }

        self.memory[address as usize] = value;
    }

    #[allow(dead_code)]
    pub fn load_rom_at(&mut self, rom: &[u8], address: u16) {
        for (i, &byte) in rom.iter().enumerate() {
            self.memory[address as usize + i] = byte;
        }
    }
}