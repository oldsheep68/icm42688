//! An [embedded-hal] driver for the ICM-42670 6-axis IMU from InvenSense.
//!
//! The ICM-42688 combines a 3-axis accelerometer with a 3-axis gyroscope into a
//! single package. It has a configurable host interface which supports I²C,
//! SPI, and I3C communications. Presently this driver only supports using the
//! I²C interface.
//!
//! For additional information about this device please refer to the
//! datasheet at TDK.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! 

#![no_std]

use core::f32::consts::PI;
use core::fmt::Debug;

//use esp_println::println;

pub use accelerometer;
use accelerometer::{
    error::Error as AccelerometerError,
    vector::{F32x3, I16x3},
    Accelerometer, RawAccelerometer,
};
use embedded_hal::blocking::{
    delay::DelayUs,
    i2c::{Write, WriteRead},
};

use crate::{
    config::{Bitfield},
    error::SensorError,
    register::{Bank0, Bank1, Bank2, Bank4, Register, RegisterBank},
};
pub use crate::{
    config::{AccelBw, 
             AccelOdr, 
             AccelRange, 
             Address, 
             GyroBw, 
             GyroOdr, 
             GyroRange, 
             PowerMode, 
             I2cSlewRate},
    error::Error,
};

pub mod config;
mod error;
mod register;

/// Re-export any traits which may be required by end users
pub mod prelude {
    pub use accelerometer::{
        Accelerometer as _accelerometer_Accelerometer,
        RawAccelerometer as _accelerometer_RawAccelerometer,
    };
}

const GRAVITY: f32 = 9.81;

/// ICM-Icm42688 driver
#[derive(Debug, Clone, Copy)]
pub struct Icm42688<I2C> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// I²C slave address to use
    address: Address,
}

impl<I2C, E> Icm42688<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    E: Debug,
{
    /// Unique device identifiers for the ICM-Icm42688
    pub const DEVICE_IDS: [u8; 1] = [
        0x47, // ICM-42688
    ];

    /// Instantiate a new instance of the driver and initialize the device
    pub fn new(i2c: I2C, address: Address) -> Result<Self, Error<E>> {
        let mut me = Self { i2c, address };

        // Verify that the device has the correct ID before continuing. If the ID does
        // not match either of the expected values then it is likely the wrong chip is
        // connected.
        if !Self::DEVICE_IDS.contains(&me.device_id()?) {
            return Err(Error::SensorError(SensorError::BadChip));
        }

        // Make sure that any configuration has been restored to the default values when
        // initializing the driver.
        me.set_accel_range(AccelRange::default())?;
        me.set_gyro_range(GyroRange::default())?;

        // The IMU uses `PowerMode::Sleep` by default, which disables both the accel and
        // gyro, so we enable them both during driver initialization.
        me.set_power_mode(PowerMode::SixAxisLowNoise)?;

        Ok(me)
    }


    // /// Instantiate a new instance of the driver and initialize the device
    // /// for the i2C interface. The slew rate of the icm42688 can be stet at startup
    // pub fn new_i2c_slew(i2c: I2C, address: Address, slew_rate: I2cSlewRate) -> Result<Self, Error<E>> {
    //     let mut me = Self { i2c, address };

    //     // set slwe rate
    //     me.set_i2c_slew_rate(slew_rate)?;

    //     // Verify that the device has the correct ID before continuing. If the ID does
    //     // not match either of the expected values then it is likely the wrong chip is
    //     // connected.
    //     if !Self::DEVICE_IDS.contains(&me.device_id()?) {
    //         return Err(Error::SensorError(SensorError::BadChip));
    //     }

    //     // Make sure that any configuration has been restored to the default values when
    //     // initializing the driver.
    //     me.set_accel_range(AccelRange::default())?;
    //     me.set_gyro_range(GyroRange::default())?;

    //     // The IMU uses `PowerMode::Sleep` by default, which disables both the accel and
    //     // gyro, so we enable them both during driver initialization.
    //     me.set_power_mode(PowerMode::SixAxisLowNoise)?;

    //     Ok(me)
    // }

    /// Instantiate a new instance of the driver and initialize the device
    /// for the i2C interface. The slew rate of the icm42688 can be stet at startup
    pub fn new_i2c_slew(i2c: I2C, 
                        address: Address, 
                        slew_rate: I2cSlewRate,
                        delay: &mut dyn DelayUs<u8>,
                    ) -> Result<Self, Error<E>> {
        let mut me = Self { i2c, address };
        //println!("new_i2c_slew");

        me.soft_reset()?;

        for _i in 0..1200 {
            delay.delay_us(250);
        }

        me.set_bank(RegisterBank::Bank0)?;

        // set slwe rate
        me.set_i2c_slew_rate(slew_rate)?;
        //println!("new_i2c_slew slewrate set");

        for _i in 0..1200 {
            delay.delay_us(250);
        }

        // Verify that the device has the correct ID before continuing. If the ID does
        // not match either of the expected values then it is likely the wrong chip is
        // connected.
        if !Self::DEVICE_IDS.contains(&me.device_id()?) {
            return Err(Error::SensorError(SensorError::BadChip));
        }
        //println!("new_i2c_slew device id read ");

        for _i in 0..1200 {
            delay.delay_us(250);
        }

        // Make sure that any configuration has been restored to the default values when
        // initializing the driver.
        me.set_accel_range(AccelRange::default())?;
        me.set_gyro_range(GyroRange::default())?;

        //println!("new_i2c_slew accel & gyro set");

        for _i in 0..1200 {
            delay.delay_us(250);
        }

        // The IMU uses `PowerMode::Sleep` by default, which disables both the accel and
        // gyro, so we enable them both during driver initialization.
        me.set_power_mode(PowerMode::SixAxisLowNoise)?;
        for _i in 0..1200 {
            delay.delay_us(250);
        }

        Ok(me)
    }


    /// Instantiate a new instance of the driver and initialize the device
    /// 
    /// limitations: currently no temperature in fifo mode TODO: implement
    pub fn new_fifo(
        i2c: I2C,
        address: Address,
        packet_type: FifoPacketType,
        delay: &mut dyn DelayUs<u8>,
    ) -> Result<Self, Error<E>> {
        let mut me = Self { i2c, address };
        // Verify that the device has the correct ID before continuing. If the ID does
        // not match either of the expected values then it is likely the wrong chip is
        // connected.
        if !Self::DEVICE_IDS.contains(&me.device_id()?) {
            return Err(Error::SensorError(SensorError::BadChip));
        }
        //println!("new_fifo 1");
        // Make sure that any configuration has been restored to the default values when
        // initializing the driver.
        me.set_accel_range(AccelRange::default())?;
        me.set_gyro_range(GyroRange::default())?;

        // enable RC oszillator, so that configuration is possible
        me.set_power_mode(PowerMode::Idle)?;
        //println!("new_fifo 2");
        // setup FIFO configurations FIFO_MODE=1 => 0x40
        me.update_reg(&Bank0::FIFO_CONFIG, 0x80, 0x80)?;

        // TMST_EN = 1           0x1
        // TMST_FSYNC_EN = 1     0x0
        // TMST_DELATA_EN = 1    0x4
        // TMST_RES = 0
        // TMST_TO_TEGS_EN = 1  0x10
        me.update_reg(&Bank0::TMST_CONFIG, 0x15, 0x15)?;
        
        match packet_type {
            FifoPacketType::Packet1 => {
                me.write_reg(&Bank0::FIFO_CONFIG1, 0x05)?;
                // no FSYNC
            }
            FifoPacketType::Packet2 => {
                me.write_reg(&Bank0::FIFO_CONFIG1, 0x06)?;
                // no FSYNC
            }
            FifoPacketType::Packet3 => {
                me.write_reg(&Bank0::FIFO_CONFIG1, 0x07)?;
                // FSYNC
            }
            FifoPacketType::Packet4 => {
                me.write_reg(&Bank0::FIFO_CONFIG1, 0x17)?;
                // no FSYNC
            }
        }
        
        
        // reduce number of generated packtets to 100Hz
        // me.set_accel_odr(AccelOdr::Hz12_5)?;
        // me.set_gyro_odr(GyroOdr::Hz12_5)?;
        me.set_accel_odr(AccelOdr::Hz1000)?;
        me.set_gyro_odr(GyroOdr::Hz1000)?;
        
        // The IMU uses `PowerMode::Sleep` by default, which disables both the accel and
        // gyro, so we enable them both during driver initialization.
        me.set_power_mode(PowerMode::SixAxisLowNoiseTemp)?;
        // me.set_power_mode(PowerMode::SixAxisLowNoise)?;
        for _i in 0..200 {
            delay.delay_us(250);
        }

        me.update_reg(&Bank0::SIGNAL_PATH_RESET, 0b0000_0010, 0b0000_0010)?;

        Ok(me)
    }

    /// Return the raw interface to the underlying `I2C` instance
    pub fn free(self) -> I2C {
        self.i2c
    }

    /// Read the ID of the connected device
    pub fn device_id(&mut self) -> Result<u8, Error<E>> {
        self.read_reg(&Bank0::WHO_AM_I)
    }

    /// soft reset the device
    pub fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::DEVICE_CONFIG, 0x01, 0b0000_0001)
    }

    /// soft reset the device
    pub fn set_i2c_slew_rate(&mut self, slew_rate: I2cSlewRate) -> Result<(), Error<E>> {
        self.write_reg(&Bank0::DRIVE_CONFIG, (slew_rate as u8) << 3)?;
        Ok(())
        
    }

    /// Return the normalized gyro data for each of the three axes
    pub fn gyro_norm(&mut self) -> Result<F32x3, Error<E>> {
        let range = self.gyro_range()?;
        let scale = range.scale_factor();

        // Scale the raw Gyroscope data using the appropriate factor based on the
        // configured range.
        let raw = self.gyro_raw()?;
        let x = raw.x as f32 / scale;
        let y = raw.y as f32 / scale;
        let z = raw.z as f32 / scale;

        Ok(F32x3::new(x, y, z))
    }

    /// Read the raw gyro data for each of the three axes
    pub fn gyro_raw(&mut self) -> Result<I16x3, Error<E>> {
        let x = self.read_reg_i16(&Bank0::GYRO_DATA_X1, &Bank0::GYRO_DATA_X0)?;
        let y = self.read_reg_i16(&Bank0::GYRO_DATA_Y1, &Bank0::GYRO_DATA_Y0)?;
        let z = self.read_reg_i16(&Bank0::GYRO_DATA_Z1, &Bank0::GYRO_DATA_Z0)?;

        Ok(I16x3::new(x, y, z))
    }

    /// Read the built-in temperature sensor and return the value in degrees
    /// centigrade
    pub fn temperature(&mut self) -> Result<f32, Error<E>> {
        let raw = self.temperature_raw()? as f32;
        let deg = (raw / 132.48) + 25.0;

        Ok(deg)
    }

    /// Read the raw data from the built-in temperature sensor
    pub fn temperature_raw(&mut self) -> Result<i16, Error<E>> {
        self.read_reg_i16(&Bank0::TEMP_DATA1, &Bank0::TEMP_DATA0)
    }

    /// Return the currently configured power mode
    pub fn power_mode(&mut self) -> Result<PowerMode, Error<E>> {
        //  `GYRO_MODE` occupies bits 3:2 in the register
        // `ACCEL_MODE` occupies bits 1:0 in the register
        let bits = self.read_reg(&Bank0::PWR_MGMT0)? & 0x3F;
        let mode = PowerMode::try_from(bits)?;

        Ok(mode)
    }

    /// Set the power mode of the IMU
    pub fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::PWR_MGMT0, mode.bits(), PowerMode::BITMASK)
    }

    /// Return the currently configured accelerometer range
    pub fn accel_range(&mut self) -> Result<AccelRange, Error<E>> {
        // `ACCEL_UI_FS_SEL` occupies bits 6:5 in the register
        let fs_sel = self.read_reg(&Bank0::ACCEL_CONFIG0)? >> 5;
        let range = AccelRange::try_from(fs_sel)?;

        Ok(range)
    }

    /// Set the range of the accelerometer
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::ACCEL_CONFIG0, range.bits(), AccelRange::BITMASK)
    }

    /// Return the currently configured gyroscope range
    pub fn gyro_range(&mut self) -> Result<GyroRange, Error<E>> {
        // `GYRO_UI_FS_SEL` occupies bits 6:5 in the register
        let fs_sel = self.read_reg(&Bank0::GYRO_CONFIG0)? >> 5;
        let range = GyroRange::try_from(fs_sel)?;

        Ok(range)
    }

    /// Set the range of the gyro
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::GYRO_CONFIG0, range.bits(), GyroRange::BITMASK)
    }

    /// Return the currently configured output data rate for the gyroscope
    pub fn gyro_odr(&mut self) -> Result<GyroOdr, Error<E>> {
        // `GYRO_ODR` occupies bits 3:0 in the register
        let odr = self.read_reg(&Bank0::GYRO_CONFIG0)? & 0xF;
        let odr = GyroOdr::try_from(odr)?;

        Ok(odr)
    }

    /// Set the output data rate of the gyroscope
    pub fn set_gyro_odr(&mut self, odr: GyroOdr) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::GYRO_CONFIG0, odr.bits(), GyroOdr::BITMASK)
    }

    pub fn gyro_bandwith(&mut self) -> Result<GyroBw, Error<E>> {
        // `GYRO_UI_FILT_BW` occupies bits 2:0 in the register
        let bw_sel = self.read_reg(&Bank0::GYRO_ACCEL_CONFIG0)? & 0x0F;
        let bw = GyroBw::try_from(bw_sel)?;

        Ok(bw)
    }

    /// Set the gyro_bandwith filter of the gyro
    pub fn set_gyro_bw(&mut self, range: GyroBw) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::GYRO_ACCEL_CONFIG0, range.bits(), GyroBw::BITMASK)
    }

    /// Return the currently configured output data rate for the accelerometer
    pub fn accel_odr(&mut self) -> Result<AccelOdr, Error<E>> {
        // `ACCEL_ODR` occupies bits 3:0 in the register
        let odr = self.read_reg(&Bank0::ACCEL_CONFIG0)? & 0xF;
        let odr = AccelOdr::try_from(odr)?;

        Ok(odr)
    }

    /// Set the output data rate of the accelerometer
    pub fn set_accel_odr(&mut self, odr: AccelOdr) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::ACCEL_CONFIG0, odr.bits(), AccelOdr::BITMASK)
    }

    pub fn accel_bandwith(&mut self) -> Result<AccelBw, Error<E>> {
        // `ACCEL_UI_FILT_BW` occupies bits 2:0 in the register
        let bw_sel = self.read_reg(&Bank0::GYRO_ACCEL_CONFIG0)? >> 4 & 0x0F;
        let bw = AccelBw::try_from(bw_sel)?;

        Ok(bw)
    }

    /// Set the accel_bandwith filter of the accel-meter
    pub fn set_accel_bw(&mut self, range: AccelBw) -> Result<(), Error<E>> {
        self.update_reg(&Bank0::GYRO_ACCEL_CONFIG0, range.bits(), AccelBw::BITMASK)
    }

    // /// Enable pedometer of APEX functions
    // pub fn ped_ena(&mut self, enable: bool) -> Result<(), Error<E>> {
    //     let mut bits: u8 = 0;
    //     if enable == true {
    //         bits = 0b0000_1000
    //     }

    //     self.update_reg(&Bank0::APEX_CONFIG0, 0b0000_0100, 0b0000_0100)?;
    //     self.update_reg(&Bank4::APEX_CONFIG1, bits, 0b0000_1000)
    // }

    // /// read the ped counter value of APEX function
    // pub fn read_ped_cnt(&mut self) -> Result<u16, Error<E>> {
    //     let ped_cnt = self.read_reg_u16(&Bank0::APEX_DATA1, &Bank0::APEX_DATA0)?;
    //     Ok(ped_cnt)
    // }

    /// read time stampe from register
    pub fn read_tmst(&mut self) -> Result<u16, Error<E>> {
        let ped_cnt = self.read_reg_u16(&Bank0::TMST_FSYNCH, &Bank0::TMST_FSYNCL)?;
        Ok(ped_cnt)
    }

    /// read current fifo buffer level, available to read
    pub fn read_fifo_cnt(&mut self) -> Result<u16, Error<E>> {
        let fifo_cnt = self.read_reg_u16(&Bank0::FIFO_COUNTH, &Bank0::FIFO_COUNTL)?;
        Ok(fifo_cnt)
    }

    // pub fn set_fifo_mode(&mut self, delay: &mut dyn DelayUs<u8>) -> Result<(), Error<E>> {
    //     self.write_reg(&Bank0::FIFO_CONFIG1, 0)?;
    //     let _res = self.write_mreg(delay, RegisterBank::MReg1, &Mreg1::FIFO_CONFIG5, 3)?;
    //     Ok(())
    // }

    // pub fn read_Bank4(
    //     &mut self,
    //     delay: &mut dyn DelayUs<u8>,
    //     //reg: &dyn Register,
    // ) -> Result<u8, Error<E>> {
    //     let read_val = self.read_mreg(delay, RegisterBank::Bank4, &Bank04::FIFO_CONFIG5)?;
    //     Ok(read_val)
    // }

    // -----------------------------------------------------------------------
    // development use temporare functions

    pub fn read_fifo(&mut self, addr: u8, buffer: &mut [u8]) -> Result<u8, Error<E>> {
        //let mut buffer = [0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8];
        let adr: [u8; 1] = [addr];
        self.i2c
            .write_read(self.address as u8, &adr, buffer)
            .map_err(|e| Error::BusError(e))?;

        Ok(buffer[0])
    }

    // pub fn readreg(&mut self, addr: u8) -> Result<u8, Error<E>> {
    //     let mut buffer = [0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8, 0u8];
    //     let adr: [u8; 1] = [addr];
    //     self.i2c
    //         .write_read(self.address as u8, &adr, &mut buffer)
    //         .map_err(|e| Error::BusError(e))?;

    //     Ok(buffer[2])
    // }

    // -----------------------------------------------------------------------
    // PRIVATE

    // FIXME: 'Sleep mode' and 'accelerometer low power mode with WUOSC' do not
    //        support MREG1, MREG2 or MREG3 access.
    #[allow(unused)]
    fn read_bank(
        &mut self,
        delay: &mut dyn DelayUs<u8>,
        bank: RegisterBank,
        reg: &dyn Register,
    ) -> Result<u8, Error<E>> {
        // See "ACCESSING MREG1, MREG2 AND MREG3 REGISTERS" (page 40)

        // Wait until the internal clock is running prior to writing.
        // while self.read_reg(&Bank0::MCLK_RDY)? != 0x9 {}

        // Set Bank to read from
        self.write_reg(&Bank0::REG_BANK_SEL, bank.blk_sel())?;
        // self.write_reg(&Bank0::MADDR_R, reg.addr())?;
        delay.delay_us(10);

        // Read a value from the register.
        let result = self.read_reg(reg)?;
        delay.delay_us(10);

        // Reset block selection registers.
        self.update_reg(&Bank0::REG_BANK_SEL, 0x00, 0x07)?;

        Ok(result)
    }

    
    #[allow(unused)]
    fn write_bank(
        &mut self,
        delay: &mut dyn DelayUs<u8>,
        bank: RegisterBank,
        reg: &dyn Register,
        value: u8,
    ) -> Result<(), Error<E>> {
        // Set Bank to write to
        self.update_reg(&Bank0::REG_BANK_SEL, bank.blk_sel(), 0x07)?;

        // Write the value to the register.
        self.write_reg(reg, value)?;
        delay.delay_us(10);

        // Reset block selection registers.
        self.update_reg(&Bank0::REG_BANK_SEL, 0x00, 0x07)?;

        Ok(())
    }
    
    fn set_bank(
                &mut self,
                bank: RegisterBank,
    ) -> Result<(), Error<E>> {
        // Set Bank to write to
        self.update_reg(&Bank0::REG_BANK_SEL, bank.blk_sel(), 0x07)?;
        Ok(())
    }

    
    /// Read a register at the provided address.
    fn read_reg(&mut self, reg: &dyn Register) -> Result<u8, Error<E>> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.address as u8, &[reg.addr()], &mut buffer)
            .map_err(|e| Error::BusError(e))?;

        Ok(buffer[0])
    }

    
    /// Read two registers and combine them into a single value.
    fn read_reg_i16(
        &mut self,
        reg_hi: &dyn Register,
        reg_lo: &dyn Register,
    ) -> Result<i16, Error<E>> {
        let data_lo = self.read_reg(reg_lo)?;
        let data_hi = self.read_reg(reg_hi)?;

        let data = i16::from_be_bytes([data_hi, data_lo]);

        Ok(data)
    }

    /// Read two registers and combine them into a single value.
    fn read_reg_u16(
        &mut self,
        reg_hi: &dyn Register,
        reg_lo: &dyn Register,
    ) -> Result<u16, Error<E>> {
        let data_lo = self.read_reg(reg_lo)?;
        let data_hi = self.read_reg(reg_hi)?;

        let data = u16::from_be_bytes([data_hi, data_lo]);

        Ok(data)
    }

    /// Set a register at the provided address to a given value.
    fn write_reg(&mut self, reg: &dyn Register, value: u8) -> Result<(), Error<E>> {
        if reg.read_only() {
            Err(Error::SensorError(SensorError::WriteToReadOnly))
        } else {
            self.i2c
                .write(self.address as u8, &[reg.addr(), value])
                .map_err(|e| Error::BusError(e))
        }
    }

    /// Update the register at the provided address.
    ///
    /// Rather than overwriting any active bits in the register, we first read
    /// in its current value and then update it accordingly using the given
    /// value and mask before writing back the desired value.
    fn update_reg(&mut self, reg: &dyn Register, value: u8, mask: u8) -> Result<(), Error<E>> {
        if reg.read_only() {
            Err(Error::SensorError(SensorError::WriteToReadOnly))
        } else {
            let current = self.read_reg(reg)?;
            let value = (current & !mask) | (value & mask);

            self.write_reg(reg, value)
        }
    }
}

/// Fifo packe type to use in fifo mode
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FifoPacketType {
    Packet1,
    Packet2,
    Packet3,
    Packet4,
}

#[derive(Debug, Copy, Clone)]
pub struct FifoDataP1 {
    pub ax: i16,
    pub ay: i16,
    pub az: i16,
    pub t: i8,
}

impl FifoDataP1 {
    pub fn to_fifodata_raw(buffer: &mut [u8]) -> Self {
        let ax = i16::from_be_bytes([buffer[1], buffer[2]]);
        let ay = i16::from_be_bytes([buffer[3], buffer[4]]);
        let az = i16::from_be_bytes([buffer[5], buffer[6]]);
        let t = i8::from_be_bytes([buffer[7]]);

        Self { ax, ay, az, t }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FifoDataP2 {
    pub gx: i16,
    pub gy: i16,
    pub gz: i16,
    pub t: i8,
}

impl FifoDataP2 {
    pub fn to_fifodata_raw(buffer: &mut [u8]) -> Self {
        let gx = i16::from_be_bytes([buffer[1], buffer[2]]);
        let gy = i16::from_be_bytes([buffer[3], buffer[4]]);
        let gz = i16::from_be_bytes([buffer[5], buffer[6]]);
        let t = i8::from_be_bytes([buffer[7]]);

        Self { gx, gy, gz, t }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FifoDataP3 {
    pub ax: i16,
    pub ay: i16,
    pub az: i16,
    pub gx: i16,
    pub gy: i16,
    pub gz: i16,
    pub t: i8,
    pub ts: u16,
}

impl FifoDataP3 {
    pub fn to_fifodata_raw(buffer: &mut [u8]) -> Self {
        let ax = i16::from_be_bytes([buffer[1], buffer[2]]);
        let ay = i16::from_be_bytes([buffer[3], buffer[4]]);
        let az = i16::from_be_bytes([buffer[5], buffer[6]]);
        let gx = i16::from_be_bytes([buffer[7], buffer[8]]);
        let gy = i16::from_be_bytes([buffer[9], buffer[10]]);
        let gz = i16::from_be_bytes([buffer[11], buffer[12]]);
        let t = i8::from_be_bytes([buffer[13]]);
        let ts = u16::from_be_bytes([buffer[14], buffer[15]]);

        Self {
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            t,
            ts,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FifoDataP4 {
    pub ax: i32,
    pub ay: i32,
    pub az: i32,
    pub gx: i32,
    pub gy: i32,
    pub gz: i32,
    pub t: i16,
    pub ts: u16,
}

impl FifoDataP4 {
    pub fn to_fifodata_raw(buffer: &mut [u8]) -> Self {
        let ax0: u8 = ((buffer[17] & 0xf0) >> 4) | (buffer[2] & 0x0f) << 4;
        let ax1: u8 = ((buffer[2] & 0xf0) >> 4) | (buffer[1] & 0x0f) << 4;
        let mut ax2: u8 = 0;
        let mut ax3: u8 = 0;
        if (buffer[1] & 0x80) > 0 {
            ax2 = ((buffer[1] & 0xf0) >> 4) | 0xf0;
            ax3 = 0xff;
        } else {
            ax2 = (buffer[1] & 0xf0) >> 4;
        }
        let ax = i32::from_be_bytes([ax3, ax2, ax1, ax0]);
        let ay0: u8 = ((buffer[18] & 0xf0) >> 4) | (buffer[4] & 0x0f) << 4;
        let ay1: u8 = ((buffer[4] & 0xf0) >> 4) | (buffer[3] & 0x0f) << 4;
        let mut ay2: u8 = 0;
        let mut ay3: u8 = 0;
        if (buffer[3] & 0x80) > 0 {
            ay2 = ((buffer[3] & 0xf0) >> 4) | 0xf0;
            ay3 = 0xff;
        } else {
            ay2 = (buffer[3] & 0xf0) >> 4;
        }
        let ay = i32::from_be_bytes([ay3, ay2, ay1, ay0]);
        let az0: u8 = ((buffer[19] & 0xf0) >> 4) | (buffer[6] & 0x0f) << 4;
        let az1: u8 = ((buffer[6] & 0xf0) >> 4) | (buffer[5] & 0x0f) << 4;
        let mut az2: u8 = 0;
        let mut az3: u8 = 0;
        if (buffer[5] & 0x80) > 0 {
            az2 = ((buffer[5] & 0xf0) >> 4) | 0xf0;
            az3 = 0xff;
        } else {
            az2 = (buffer[5] & 0xf0) >> 4;
        }
        let az = i32::from_be_bytes([az3, az2, az1, az0]);

        let gx0: u8 = (buffer[17] & 0x0f) | (buffer[8] & 0x0f) << 4;
        let gx1: u8 = ((buffer[8] & 0xf0) >> 4) | (buffer[7] & 0x0f) << 4;
        let mut gx2: u8 = 0;
        let mut gx3: u8 = 0;
        if (buffer[7] & 0x80) > 0 {
            gx2 = ((buffer[7] & 0xf0) >> 4) | 0xf0;
            gx3 = 0xff;
        } else {
            gx2 = (buffer[7] & 0xf0) >> 4;
        }
        let gx = i32::from_be_bytes([gx3, gx2, gx1, gx0]);

        let gy0: u8 = (buffer[18] & 0x0f) | (buffer[10] & 0x0f) << 4;
        let gy1: u8 = ((buffer[10] & 0xf0) >> 4) | (buffer[9] & 0x0f) << 4;
        let mut gy2: u8 = 0;
        let mut gy3: u8 = 0;
        if (buffer[9] & 0x80) > 0 {
            gy2 = ((buffer[9] & 0xf0) >> 4) | 0xf0;
            gy3 = 0xff;
        } else {
            gy2 = (buffer[9] & 0xf0) >> 4;
        }
        let gy = i32::from_be_bytes([gy3, gy2, gy1, gy0]);

        let gz0: u8 = (buffer[19] & 0x0f) | (buffer[12] & 0x0f) << 4;
        let gz1: u8 = ((buffer[12] & 0xf0) >> 4) | (buffer[11] & 0x0f) << 4;
        let mut gz2: u8 = 0;
        let mut gz3: u8 = 0;
        if (buffer[11] & 0x80) > 0 {
            gz2 = ((buffer[11] & 0xf0) >> 4) | 0xf0;
            gz3 = 0xff;
        } else {
            gz2 = (buffer[11] & 0xf0) >> 4;
        }
        let gz = i32::from_be_bytes([gz3, gz2, gz1, gz0]);

        let t = i16::from_be_bytes([buffer[13], buffer[14]]);
        let ts = u16::from_be_bytes([buffer[15], buffer[16]]);

        Self {
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            t,
            ts,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FifoDataSiP1 {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub t: f32,
}

impl FifoDataSiP1 {
    pub fn to_fifodata_si(buffer: &mut [u8], ascal: f32) -> Self {
        let ax = i16::from_be_bytes([buffer[1], buffer[2]]);
        let ay = i16::from_be_bytes([buffer[3], buffer[4]]);
        let az = i16::from_be_bytes([buffer[5], buffer[6]]);
        let t = i8::from_be_bytes([buffer[7]]);

        Self {
            ax: (ax as f32) / ascal * GRAVITY,
            ay: (ay as f32) / ascal * GRAVITY,
            az: (az as f32) / ascal * GRAVITY,
            t: (((t as f32) / 2.07) + 25.0),
        }
    }
}

#[derive(Debug, Clone)]
pub struct FifoDataSiP2 {
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub t: f32,
}

impl FifoDataSiP2 {
    pub fn to_fifodata_si(buffer: &mut [u8], gscal: f32) -> Self {
        let gx = i16::from_be_bytes([buffer[7], buffer[8]]);
        let gy = i16::from_be_bytes([buffer[9], buffer[10]]);
        let gz = i16::from_be_bytes([buffer[11], buffer[12]]);
        let t = i8::from_be_bytes([buffer[7]]);

        Self {
            gx: ((gx as f32) / gscal) * PI / 180.0,
            gy: ((gy as f32) / gscal) * PI / 180.0,
            gz: ((gz as f32) / gscal) * PI / 180.0,
            t: (((t as f32) / 2.07) + 25.0),
        }
    }
}

#[derive(Debug, Clone)]
pub struct FifoDataSiP3 {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub t: f32,
    pub ts: f32,
}

impl FifoDataSiP3 {
    pub fn to_fifodata_si(buffer: &mut [u8], ascal: f32, gscal: f32) -> Self {
        let ax = i16::from_be_bytes([buffer[1], buffer[2]]);
        let ay = i16::from_be_bytes([buffer[3], buffer[4]]);
        let az = i16::from_be_bytes([buffer[5], buffer[6]]);
        let gx = i16::from_be_bytes([buffer[7], buffer[8]]);
        let gy = i16::from_be_bytes([buffer[9], buffer[10]]);
        let gz = i16::from_be_bytes([buffer[11], buffer[12]]);
        let t = i8::from_be_bytes([buffer[13]]);
        let ts = u16::from_be_bytes([buffer[14], buffer[15]]);

        Self {
            ax: (ax as f32) / ascal * GRAVITY,
            ay: (ay as f32) / ascal * GRAVITY,
            az: (az as f32) / ascal * GRAVITY,
            gx: ((gx as f32) / gscal) * PI / 180.0,
            gy: ((gy as f32) / gscal) * PI / 180.0,
            gz: ((gz as f32) / gscal) * PI / 180.0,
            t: (((t as f32) / 2.07) + 25.0),
            ts: (ts as f32) / 1_000_000.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct FifoDataSiP4 {
    pub ax: f32,
    pub ay: f32,
    pub az: f32,
    pub gx: f32,
    pub gy: f32,
    pub gz: f32,
    pub t: f32,
    pub ts: f32,
}

impl FifoDataSiP4 {
    pub fn to_fifodata_si(raw_data: &FifoDataP4) -> Self {
        const ASCAL: f32 = 8192.0 * 4.0;
        const GSCAL: f32 = 131.0 * 2.0;

        let ax = raw_data.ax as f32;
        let ay = raw_data.ay as f32;
        let az = raw_data.az as f32;
        let gx = raw_data.gx as f32;
        let gy = raw_data.gy as f32;
        let gz = raw_data.gz as f32;
        let t = raw_data.t as f32;
        let ts = raw_data.ts as f32;

        Self {
            ax: (ax as f32) / ASCAL * GRAVITY,
            ay: (ay as f32) / ASCAL * GRAVITY,
            az: (az as f32) / ASCAL * GRAVITY,
            gx: ((gx as f32) / GSCAL) * PI / 180.0,
            gy: ((gy as f32) / GSCAL) * PI / 180.0,
            gz: ((gz as f32) / GSCAL) * PI / 180.0,
            t: (((t as f32) / 132.48) + 25.0),
            ts: (ts as f32) / 1_000_000.0,
        }
    }
}

impl<I2C, E> Accelerometer for Icm42688<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    fn accel_norm(&mut self) -> Result<F32x3, AccelerometerError<Self::Error>> {
        let range = self.accel_range()?;
        let scale = range.scale_factor();

        // Scale the raw Accelerometer data using the appropriate factor based on the
        // configured range.
        let raw = self.accel_raw()?;
        let x = raw.x as f32 / scale;
        let y = raw.y as f32 / scale;
        let z = raw.z as f32 / scale;

        Ok(F32x3::new(x, y, z))
    }

    fn sample_rate(&mut self) -> Result<f32, AccelerometerError<Self::Error>> {
        let odr = self.accel_odr()?;
        let rate = odr.as_f32();

        Ok(rate)
    }
}

impl<I2C, E> RawAccelerometer<I16x3> for Icm42688<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
    E: Debug,
{
    type Error = Error<E>;

    fn accel_raw(&mut self) -> Result<I16x3, AccelerometerError<Self::Error>> {
        let x = self.read_reg_i16(&Bank0::ACCEL_DATA_X1, &Bank0::ACCEL_DATA_X0)?;
        let y = self.read_reg_i16(&Bank0::ACCEL_DATA_Y1, &Bank0::ACCEL_DATA_Y0)?;
        let z = self.read_reg_i16(&Bank0::ACCEL_DATA_Z1, &Bank0::ACCEL_DATA_Z0)?;

        Ok(I16x3::new(x, y, z))
    }
}
