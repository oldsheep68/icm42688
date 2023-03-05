#![allow(non_camel_case_types)]
#![allow(unused)]

// All reads and writes actually go through User Bank 0, and the remaining banks
// are accessed via this bank; as such, User Bank 0 has been omitted, given that
// we are not actually able to "select" it.
#[derive(Debug, Clone, Copy)]
pub(crate) enum RegisterBank {
    Bank0,
    Bank1,
    Bank2,
    Bank3,
    Bank4,
}

impl RegisterBank {
    /// The block selection value for a given register bank
    pub fn blk_sel(self) -> u8 {
        match self {
            RegisterBank::Bank0 => 0x00,
            RegisterBank::Bank1 => 0x01,
            RegisterBank::Bank2 => 0x02,
            RegisterBank::Bank3 => 0x03,
            RegisterBank::Bank4 => 0x04,
        }
    }
}

pub(crate) trait Register {
    /// Get the address of the register
    fn addr(&self) -> u8;

    /// Is the register read-only?
    fn read_only(&self) -> bool;
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Bank0 {
    DEVICE_CONFIG = 0x11,
    DRIVE_CONFIG = 0x13,  
    INT_CONFIG = 0x14,
    FIFO_CONFIG = 0x16,
    TEMP_DATA1 = 0x1D,
    TEMP_DATA0 = 0x1E,
    ACCEL_DATA_X1 = 0x1F,
    ACCEL_DATA_X0 = 0x20,
    ACCEL_DATA_Y1 = 0x21,
    ACCEL_DATA_Y0 = 0x22,
    ACCEL_DATA_Z1 = 0x23,
    ACCEL_DATA_Z0 = 0x24,
    GYRO_DATA_X1 = 0x25,
    GYRO_DATA_X0 = 0x26,
    GYRO_DATA_Y1 = 0x27,
    GYRO_DATA_Y0 = 0x28,
    GYRO_DATA_Z1 = 0x29,
    GYRO_DATA_Z0 = 0x2A,
    TMST_FSYNCH = 0x2B,
    TMST_FSYNCL = 0x2C,
    INT_STATUS = 0x2D,
    FIFO_COUNTH = 0x2E,
    FIFO_COUNTL = 0x2F,
    FIFO_DATA = 0x30,
    APEX_DATA0 = 0x31,
    APEX_DATA1 = 0x32,
    APEX_DATA2 = 0x33,
    APEX_DATA3 = 0x34,
    APEX_DATA4 = 0x35,
    APEX_DATA5 = 0x36,
    INT_STATUS2 = 0x37,
    INT_STATUS3 = 0x38,
    SIGNAL_PATH_RESET = 0x48,
    INTF_CONFIG0 = 0x4C,
    INTF_CONFIG1 = 0x4D,
    PWR_MGMT0 = 0x4E,
    GYRO_CONFIG0 = 0x4F,
    ACCEL_CONFIG0 = 0x50,
    GYRO_CONFIG1 = 0x51,
    GYRO_ACCEL_CONFIG0 = 0x52,
    ACCEL_CONFIG1 = 0x53,
    TMST_CONFIG = 0x54,
    APEX_CONFIG0 = 0x56,
    SMD_CONFIG = 0x57,
    FIFO_CONFIG1 = 0x5F,
    FIFO_CONFIG2 = 0x60,
    FIFO_CONFIG3 = 0x61,
    FSYNC_CONFIG = 0x62,
    INT_CONFIG0 = 0x63,
    INT_CONFIG1 = 0x64,
    INT_SOURCE0 = 0x65,
    INT_SOURCE1 = 0x66,
    INT_SOURCE3 = 0x68,
    INT_SOURCE4 = 0x69,
    FIFO_LOST_PKT0 = 0x6C,
    FIFO_LOST_PKT1 = 0x6D,
    SELF_TEST_CONFIG = 0x70,
    WHO_AM_I = 0x75,
    REG_BANK_SEL = 0x76,
}

impl Register for Bank0 {
    fn addr(&self) -> u8 {
        *self as u8
    }

    fn read_only(&self) -> bool {
        use Bank0::*;

        matches!(
            self,
                TEMP_DATA1
                | TEMP_DATA0
                | ACCEL_DATA_X1
                | ACCEL_DATA_X0
                | ACCEL_DATA_Y1
                | ACCEL_DATA_Y0
                | ACCEL_DATA_Z1
                | ACCEL_DATA_Z0
                | GYRO_DATA_X1
                | GYRO_DATA_X0
                | GYRO_DATA_Y1
                | GYRO_DATA_Y0
                | GYRO_DATA_Z1
                | GYRO_DATA_Z0
                | TMST_FSYNCH
                | TMST_FSYNCL
                | INT_STATUS
                | APEX_DATA0
                | APEX_DATA1
                | APEX_DATA2
                | APEX_DATA3
                | APEX_DATA4
                | APEX_DATA5
                | INT_STATUS2
                | INT_STATUS3
                | FIFO_LOST_PKT0
                | FIFO_LOST_PKT1
                | FIFO_COUNTH
                | FIFO_COUNTL
                | FIFO_DATA
                | WHO_AM_I
        )
    }
}

#[allow(clippy::upper_case_acronyms)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum Bank1 {
    SENSOR_CONFIG0 = 0x03,
    GYRO_CONFIG_STATIC2 = 0x0B,
    GYRO_CONFIG_STATIC3 = 0x0C,
    GYRO_CONFIG_STATIC4 = 0x0D,
    GYRO_CONFIG_STATIC5 = 0x0E,
    GYRO_CONFIG_STATIC6 = 0x0F,
    GYRO_CONFIG_STATIC7 = 0x10,
    GYRO_CONFIG_STATIC8 = 0x11,
    GYRO_CONFIG_STATIC9 = 0x12,
    GYRO_CONFIG_STATIC10 = 0x13,
    XG_ST_DATA = 0x5F,
    YG_ST_DATA = 0x60,
    ZG_ST_DATA = 0x61,
    TMSTVAL0 = 0x62,
    TMSTVAL1 = 0x63,
    TMSTVAL2 = 0x64,
    INTF_CONFIG4 = 0x7A,
    INTF_CONFIG5 = 0x7B,
    INTF_CONFIG6 = 0x7C,
}

impl Register for Bank1 {
    fn addr(&self) -> u8 {
        *self as u8
    }

    fn read_only(&self) -> bool {
        matches!(self, Bank1::TMSTVAL0 | Bank1::TMSTVAL1 | Bank1::TMSTVAL2)
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Bank2 {
    ACCEL_CONFIG_STATIC2 = 0x03,
    ACCEL_CONFIG_STATIC3 = 0x04,
    ACCEL_CONFIG_STATIC4 = 0x05,
    XA_ST_DATA = 0x3B,
    YA_ST_DATA = 0x3C,
    ZA_ST_DATA = 0x3D,
}

impl Register for Bank2 {
    fn addr(&self) -> u8 {
        *self as u8
    }

    fn read_only(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum Bank4 {
    APEX_CONFIG1 = 0x40,
    APEX_CONFIG2 = 0x41,
    APEX_CONFIG3 = 0x42,
    APEX_CONFIG4 = 0x43,
    APEX_CONFIG5 = 0x44,
    APEX_CONFIG6 = 0x45,
    APEX_CONFIG7 = 0x46,
    APEX_CONFIG8 = 0x47,
    APEX_CONFIG9 = 0x48,
    ACCEL_WOM_X_THR = 0x4A,
    ACCEL_WOM_Y_THR = 0x4B,
    ACCEL_WOM_Z_THR = 0x4C,
    INT_SOURCE6 = 0x4D,
    INT_SOURCE7 = 0x4E,
    INT_SOURCE8 = 0x4F,
    INT_SOURCE9 = 0x50,
    INT_SOURCE10 = 0x51,
    OFFSET_USER0 = 0x77,
    OFFSET_USER1 = 0x78,
    OFFSET_USER2 = 0x79,
    OFFSET_USER3 = 0x7A,
    OFFSET_USER4 = 0x7B,
    OFFSET_USER5 = 0x7C,
    OFFSET_USER6 = 0x7D,
    OFFSET_USER7 = 0x7E,
    OFFSET_USER8 = 0x7F,
}

impl Register for Bank4 {
    fn addr(&self) -> u8 {
        *self as u8
    }

    fn read_only(&self) -> bool {
        false
    }
}
