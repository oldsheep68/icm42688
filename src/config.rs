use crate::error::SensorError;

pub(crate) trait Bitfield {
    const BITMASK: u8;

    /// Bit value of a discriminant, shifted to the correct position if
    /// necessary
    fn bits(self) -> u8;
}

/// Defined sloewrate constant for I2C bus 
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum I2cSlewRate {
    /// 20..60ns
    I2cSlwe60ns = 0b000,
    /// 12..36ns
    I2cSlwe18ns = 0b001,
    /// 6..18ns
    I2cSlwe12ns = 0b011,
    /// 2..6ns
    I2cSlwe6ns = 0b100,
    /// <2ns  (default)
    I2cSlwe2ns = 0b101,
}

/// I²C slave addresses, determined by the logic level of pin `AP_AD0`
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Address {
    /// `AP_AD0` pin == 0
    Primary = 0x68,
    /// `AP_AD0` pin == 1
    Secondary = 0x69,
}

/// Configurable ranges of the Accelerometer ACCEL_FS_SEL
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AccelRange {
    /// ±2G
    G2 = 3,
    /// ±4G
    G4 = 2,
    /// ±8G
    G8 = 1,
    /// ±16G
    G16 = 0,
}

impl AccelRange {
    /// Sensitivity scale factor
    pub fn scale_factor(&self) -> f32 {
        use AccelRange::*;

        // Values taken from Table 2 of the data sheet
        match self {
            G2 => 16_384.0,
            G4 => 8_192.0,
            G8 => 4_096.0,
            G16 => 2_048.0,
        }
    }
}

impl Bitfield for AccelRange {
    const BITMASK: u8 = 0b0110_0000;

    fn bits(self) -> u8 {
        // `ACCEL_UI_FS_SEL` occupies bits 6:5 in the register
        (self as u8) << 5
    }
}

impl Default for AccelRange {
    fn default() -> Self {
        Self::G16
    }
}

impl TryFrom<u8> for AccelRange {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelRange::*;

        match value {
            0 => Ok(G16),
            1 => Ok(G8),
            2 => Ok(G4),
            3 => Ok(G2),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Configurable ranges of the Gyroscope
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyroRange {
    /// ±15.625 deg/sec
    Deg15_625 = 7,
    /// ±31.25 deg/sec
    Deg31_25 = 6,
    /// ±62.5 deg/sec
    Deg62_5 = 5,
    /// ±125 deg/sec
    Deg125 = 4,
    /// ±250 deg/sec
    Deg250 = 3,
    /// ±500 deg/sec
    Deg500 = 2,
    /// ±1000 deg/sec
    Deg1000 = 1,
    /// ±2000 deg/sec
    Deg2000 = 0,
}

impl GyroRange {
    /// Sensitivity scale factor "GYRO_FS_SEL"
    pub fn scale_factor(&self) -> f32 {
        use GyroRange::*;

        // Values taken from Table 1 of the data sheet
        match self {
            Deg15_625 => 2097.2,
            Deg31_25 => 1048.6,
            Deg62_5 => 524.3,
            Deg125 => 262.0,
            Deg250 => 131.0,
            Deg500 => 65.5,
            Deg1000 => 32.8,
            Deg2000 => 16.4,
        }
    }
}

impl Bitfield for GyroRange {
    const BITMASK: u8 = 0b1110_0000;

    fn bits(self) -> u8 {
        // `GYRO_UI_FS_SEL` occupies bits 7:5 in the register
        (self as u8) << 5
    }
}

impl Default for GyroRange {
    fn default() -> Self {
        Self::Deg2000
    }
}

impl TryFrom<u8> for GyroRange {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use GyroRange::*;

        match value {
            0 => Ok(Deg2000),
            1 => Ok(Deg1000),
            2 => Ok(Deg500),
            3 => Ok(Deg250),
            4 => Ok(Deg125),
            5 => Ok(Deg62_5),
            6 => Ok(Deg31_25),
            7 => Ok(Deg15_625),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Configurable power modes of the "IMU PWR_MGMT0"
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PowerMode {
    /// Gyroscope: OFF, Accelerometer: OFF, Temperature: OFF
    Sleep = 0b100000,
    /// Gyroscope: DRIVE ON Standby mode, Accelerometer: OFF, Temperature: OFF
    Standby = 0b100100,
    /// Gyroscope: OFF, Accelerometer: LowPoer Mode, Temperature: OFF
    AccelLowPower = 0b110010,
    /// Gyroscope: OFF, Accelerometer: ON, Temperature: OFF
    AccelLowNoise = 0b100011,
    /// Gyroscope: ON, Accelerometer: OFF, Temperature: OFF
    GyroLowNoise = 0b101100,
    /// Gyroscope: ON, Accelerometer: ON, Temperatur: OFF
    SixAxisLowNoise = 0b101111,
    /// Idle mode RC oscilator is powered on Accel and Gyro OFF and TEMPERATURSENSOR off
    Idle = 0b110000,
    /// Gyroscope: OFF, Accelerometer: OFF, Temperature: ON
    
    /// Gyroscope: OFF, Accelerometer: ON LowPoer, Temperature: ON
    AccelLowPowerTemp = 0b010010,
    /// Gyroscope: OFF, Accelerometer: ON LN, Temperature: ON
    AccelLowNoiseTemp = 0b010011,
    /// Gyroscope: ON, Accelerometer: OFF, Temperature: ON
    GyroLowNoiseTemp = 0b001100,
    /// Gyroscope: ON, Accelerometer: ON, Temperature: ON
    SixAxisLowNoiseTemp = 0b001111,
}

impl Bitfield for PowerMode {
    const BITMASK: u8 = 0b0011_1111;

    fn bits(self) -> u8 {
        // Temperature sensor on off occupies bit 5  (1 = OFF)
        // Idle occupies bit 4
        // `GYRO_MODE` occupies bits 3:2 in the register
        // `ACCEL_MODE` occupies bits 1:0 in the register
        self as u8
    }
}

impl Default for PowerMode {
    fn default() -> Self {
        PowerMode::Sleep
    }
}

impl TryFrom<u8> for PowerMode {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use PowerMode::*;

        match value {
            0b100000 => Ok(Sleep),
            0b100100 => Ok(Standby),
            0b100010 => Ok(AccelLowPower),
            0b100011 => Ok(AccelLowNoise),
            0b101100 => Ok(GyroLowNoise),
            0b101111 => Ok(SixAxisLowNoise),
            0b110000 => Ok(Idle),
            0b010010 => Ok(AccelLowPowerTemp),
            0b010011 => Ok(AccelLowNoiseTemp),
            0b001100 => Ok(GyroLowNoiseTemp),
            0b001111 => Ok(Self::SixAxisLowNoiseTemp),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Accelerometer ODR selection values
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AccelOdr {
    /// 32 kHz
    Hz32000 = 0b0001,
    /// 16 kHz
    Hz16000 = 0b0010,
    /// 8 kHz
    Hz8000 = 0b0011,
    /// 4 kHz
    Hz4000 = 0b0100,
    /// 2 kHz
    Hz2000 = 0b0101,
    /// 1000 Hz
    Hz1000 = 0b0110,
    /// 200 Hz
    Hz200 = 0b0111,
    /// 100 Hz
    Hz100 = 0b1000,
    /// 50 Hz
    Hz50 = 0b1001,
    /// 25 Hz
    Hz25 = 0b1010,
    /// 12.5 Hz
    Hz12_5 = 0b1011,
    /// 6.25 Hz (LP mode)
    Hz6_25 = 0b1100,
    /// 3.125 Hz (LP mode)
    Hz3_125 = 0b1101,
    /// 1.5625 Hz (LP mode
    Hz1_5625 = 0b1110,
    /// 500 Hz
    Hz500 = 0b1111,
}

impl AccelOdr {
    pub fn as_f32(self) -> f32 {
        use AccelOdr::*;

        match self {
            Hz32000 => 32000.0,
            Hz16000 => 16000.0,
            Hz8000 => 8000.0,
            Hz4000 => 4000.0,
            Hz2000 => 2000.0,
            Hz1000 => 1000.0,
            Hz200 => 200.0,
            Hz100 => 100.0,
            Hz50 => 50.0,
            Hz25 => 25.0,
            Hz12_5 => 12.5,
            Hz6_25 => 6.25,
            Hz3_125 => 3.125,
            Hz1_5625 => 1.5625,
            Hz500 => 500.0,
        }
    }
}

impl Bitfield for AccelOdr {
    const BITMASK: u8 = 0b0000_1111;

    fn bits(self) -> u8 {
        // `ACCEL_ODR` occupies bits 3:0 in the register
        self as u8
    }
}

impl Default for AccelOdr {
    fn default() -> Self {
        Self::Hz1000
    }
}

impl TryFrom<u8> for AccelOdr {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelOdr::*;

        match value {
            0b0001 => Ok(Hz32000),
            0b0010 => Ok(Hz16000),
            0b0011 => Ok(Hz8000),
            0b0100 => Ok(Hz4000),
            0b0101 => Ok(Hz2000),
            0b0110 => Ok(Hz1000),
            0b0111 => Ok(Hz200),
            0b1000 => Ok(Hz100),
            0b1001 => Ok(Hz50),
            0b1010 => Ok(Hz25),
            0b1011 => Ok(Hz12_5),
            0b1100 => Ok(Hz6_25),
            0b1101 => Ok(Hz3_125),
            0b1110 => Ok(Hz1_5625),
            0b1111 => Ok(Hz500),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Gyroscope ODR selection values "GYRO_ODR"
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyroOdr {
    /// 32 kHz
    Hz32000 = 0b0001,
    /// 16 kHz
    Hz16000 = 0b0010,
    /// 8 kHz
    Hz8000 = 0b0011,
    /// 4 kHz
    Hz4000 = 0b0100,
    /// 2 kHz
    Hz2000 = 0b0101,
    /// 1000 Hz
    Hz1000 = 0b0110,
    /// 200 Hz
    Hz200 = 0b0111,
    /// 100 Hz
    Hz100 = 0b1000,
    /// 50 Hz
    Hz50 = 0b1001,
    /// 25 Hz
    Hz25 = 0b1010,
    /// 12.5 Hz
    Hz12_5 = 0b1011,
    /// 500 Hz
    Hz500 = 0b1111,
}

impl GyroOdr {
    pub fn as_f32(self) -> f32 {
        use GyroOdr::*;

        match self {
            Hz32000 => 32000.0,
            Hz16000 => 16000.0,
            Hz8000 => 8000.0,
            Hz4000 => 4000.0,
            Hz2000 => 2000.0,
            Hz1000 => 1000.0,
            Hz200 => 200.0,
            Hz100 => 100.0,
            Hz50 => 50.0,
            Hz25 => 25.0,
            Hz12_5 => 12.5,
            Hz500 => 500.0,
        }
    }
}

impl Default for GyroOdr {
    fn default() -> Self {
        GyroOdr::Hz1000
    }
}


impl Bitfield for GyroOdr {
    const BITMASK: u8 = 0b0000_1111;

    fn bits(self) -> u8 {
        // `GYRO_ODR` occupies bits 3:0 in the register
        self as u8
    }
}


impl TryFrom<u8> for GyroOdr {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use GyroOdr::*;

        match value {
            0b0001 => Ok(Hz32000),
            0b0010 => Ok(Hz16000),
            0b0011 => Ok(Hz8000),
            0b0100 => Ok(Hz4000),
            0b0101 => Ok(Hz2000),
            0b0110 => Ok(Hz1000),
            0b0111 => Ok(Hz200),
            0b1000 => Ok(Hz100),
            0b1001 => Ok(Hz50),
            0b1010 => Ok(Hz25),
            0b1011 => Ok(Hz12_5),
            0b1111 => Ok(Hz500),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Gyroscope LP Filter Bandwith selection values "GYRO_UI_FILT_BW"
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GyroBw {
    /// Half of ODR bandwidth set 
    OdrHalf = 0b0000,
    /// one quarter of ODR bandwidth set 
    OdrQuarter = 0b0001,
    /// one fifth of ODR bandwidth set 
    OdrFifth = 0b0010,
    /// one eighth of ODR bandwidth set 
    OdrEighth = 0b0011,
    /// one tenth of ODR bandwidth set 
    OdrTenth = 0b0100,
    /// on sixteenth of ODR bandwidth set 
    OdrSixteenth = 0b0101,
    /// one twenty of ODR bandwidth set 
    OdrTwenty = 0b0110,
    /// one fourteen of ODR bandwidth set 
    OdrFourteen = 0b0111,
    /// Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2
    /// runs at max(400Hz, ODR)
    OdrLowLatency400 = 0b1110,
    ///  Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2
    /// runs at max(200Hz, 8*ODR)
    OdrLowLatency200 = 0b1111,
}

impl GyroBw {
    pub fn as_f32(self) -> f32 {
        use GyroBw::*;

        match self {
            OdrHalf => 0.5, // filter is bypassed
            OdrQuarter => 0.25,
            OdrFifth => 0.2,
            OdrEighth => 0.125,
            OdrTenth => 0.1,
            OdrSixteenth => 1.0/16.0,
            OdrTwenty => 0.05,
            OdrFourteen => 1.0/40.0,
            OdrLowLatency400 => 400.0,
            OdrLowLatency200 => 200.0,
        }
    }
}

impl Default for GyroBw {
    fn default() -> Self {
        Self::OdrQuarter
    }
}

impl Bitfield for GyroBw {
    const BITMASK: u8 = 0b0000_1111;

    fn bits(self) -> u8 {
        // `GYRO_UI_FILT_BW` occupies bits 2:0 in the register
        self as u8
    }
}

impl TryFrom<u8> for GyroBw {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use GyroBw::*;

        match value {
            0b0000 => Ok(OdrHalf), // filter is bypassed
            0b0001 => Ok(OdrQuarter),
            0b0010 => Ok(OdrFifth),
            0b0011 => Ok(OdrEighth),
            0b0100 => Ok(OdrTenth),
            0b0101 => Ok(OdrSixteenth),
            0b0110 => Ok(OdrTwenty),
            0b0111 => Ok(OdrFourteen),
            0b1110 => Ok(OdrLowLatency400),
            0b1111 => Ok(OdrLowLatency200),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}

/// Accelareration Filter Bandwith selection values
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum AccelBw {
    /// Half of ODR bandwidth set 
    OdrHalf = 0b0000,
    /// one quarter of ODR bandwidth set 
    OdrQuarter = 0b0001,
    /// one fifth of ODR bandwidth set 
    OdrFifth = 0b0010,
    /// one eighth of ODR bandwidth set 
    OdrEighth = 0b0011,
    /// one tenth of ODR bandwidth set 
    OdrTenth = 0b0100,
    /// on sixteenth of ODR bandwidth set 
    OdrSixteenth = 0b0101,
    /// one twenty of ODR bandwidth set 
    OdrTwenty = 0b0110,
    /// one fourteen of ODR bandwidth set 
    OdrFourteen = 0b0111,
    /// Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2
    /// runs at max(400Hz, ODR)
    OdrLowLatency400 = 0b1110,
    ///  Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2
    /// runs at max(200Hz, 8*ODR)
    OdrLowLatency200 = 0b1111,
}

impl AccelBw {
    pub fn as_f32(self) -> f32 {
        use AccelBw::*;

        match self {
            OdrHalf => 0.5, // filter is bypassed
            OdrQuarter => 0.25,
            OdrFifth => 0.2,
            OdrEighth => 0.125,
            OdrTenth => 0.1,
            OdrSixteenth => 1.0/16.0,
            OdrTwenty => 0.05,
            OdrFourteen => 1.0/40.0,
            OdrLowLatency400 => 400.0,
            OdrLowLatency200 => 200.0,
        }
    }
}

impl Default for AccelBw {
    fn default() -> Self {
        Self::OdrQuarter
    }
}

impl Bitfield for AccelBw {
    const BITMASK: u8 = 0b0000_1111;

    fn bits(self) -> u8 {
        // `ACCEL_UI_FILT_BW` occupies bits 2:0 in the register
        self as u8
    }
}

impl TryFrom<u8> for AccelBw {
    type Error = SensorError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        use AccelBw::*;

        match value {
            0b0000 => Ok(OdrHalf), // filter is bypassed
            0b0001 => Ok(OdrQuarter),
            0b0010 => Ok(OdrFifth),
            0b0011 => Ok(OdrEighth),
            0b0100 => Ok(OdrTenth),
            0b0101 => Ok(OdrSixteenth),
            0b0110 => Ok(OdrTwenty),
            0b0111 => Ok(OdrFourteen),
            0b1110 => Ok(OdrLowLatency400),
            0b1111 => Ok(OdrLowLatency200),
            _ => Err(SensorError::InvalidDiscriminant),
        }
    }
}
