#![no_std]
#![no_main]

// use icm42688::*;
use icm42688::{prelude::*, Address, Icm42688, I2cSlewRate};
use esp_println::println;

use esp32_hal::{
    clock::ClockControl, 
    peripherals::Peripherals, 
    prelude::*, 
    timer::TimerGroup,
     Rtc, 
     i2c,
     Delay, IO,// Rng, Rtc,
};

use shared_bus::BusManagerSimple;

use esp_backtrace as _;

#[xtensa_lx_rt::entry]
fn main() -> ! { 
    println!("Hello Main");
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    //let system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sda = io.pins.gpio21;
    let scl = io.pins.gpio22;

    let i2c = i2c::I2C::new(
        peripherals.I2C0,
        sda,
        scl,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let bus = BusManagerSimple::new(i2c);

    //let mut icm = Icm42688::new(bus.acquire_i2c(), Address::Primary).unwrap();
    let mut icm = Icm42688::new_i2c_slew(bus.acquire_i2c(), Address::Primary, I2cSlewRate::I2cSlwe2ns).unwrap();
    let gyro_data =icm.gyro_norm();
    println!("Gyro data: {:?}", gyro_data);
    delay.delay_ms(500u32);

    loop {
        let accel_norm = icm.accel_norm().unwrap();
        delay.delay_ms(500u32);
        let gyro_norm = icm.gyro_norm().unwrap();
        println!("Acel_norm: {:?}   Gyro norm: {:?}", accel_norm, gyro_norm);
        // let gyro_data =icm.gyro_norm();
        // println!("Gyro data: {:?}", gyro_data);
        delay.delay_ms(500u32);
    }
}
