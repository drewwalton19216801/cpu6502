pub trait Device {
    // Start address of the device
    fn start_address(&self) -> u16;

    // End address of the device
    fn end_address(&self) -> u16;

    /**
     * Set the address and data lines
     * 
     * @param address The address to set
     * @param data The data to set
     * @remarks This must be called prior to reading or writing from/to the device.
     */
    fn set_address_data(&mut self, address: u16, data: u8);

    /**
     * Read the address and data lines
     * 
     * @return The address and data lines
     * @remarks This must be called after reading or writing from/to the device.
     */
    fn read_address_data(&self) -> (u16, u8);

    /**
     * Set the write enable line
     * 
     * @param write_enable The write enable line
     * @remarks This must be called prior to writing to the device, unless you're using
     *        the force_write parameter of the write() function (which is not recommended)
     */
    fn set_write_enable(&mut self, write_enable: bool);

    /**
     * Read from the device
     *
     * @remarks This function will only read from the device if the address is in the range of the device,
     *         and the caller must have set the address and data lines before calling this function. The caller
     *         must also read from the address and data lines after calling this function.
     */
    fn read(&mut self);

    /**
     * Write to the device
     * 
     * @param force_write If true, then the device will be written to regardless of the state of the write enable line
     * @remarks This function will only write to the device if the address is in the range of the device, and the
     *       caller must have set the address and data lines (set_address_data()) before calling this function. In addition, the
     *       caller must read from address and data lines (read_address_data()) after calling this function.
     */
    fn write(&mut self, force_write: bool);
}

#[allow(dead_code)]
pub struct Devices {
    devices: Vec<Box<dyn Device>>,
}