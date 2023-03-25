#![no_std]
#![no_main]

// use icm42688::*;
use icm42688::{prelude::*, Address, Icm42688, I2cSlewRate};
use esp_println::{println, print};

use esp32c3_hal::{
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

//#[xtensa_lx_rt::entry]

#[riscv_rt::entry]
fn main() -> ! { 
    println!("Hello Main");
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
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

    let sda = io.pins.gpio3;
    let scl = io.pins.gpio2;

    let i2c = i2c::I2C::new(
        peripherals.I2C0,
        sda,
        scl,
        100u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );
    delay.delay_ms(255u8);

    //let bus = BusManagerSimple::new(i2c);
    // let mut icm = Icm42688::new_i2c_slew(bus.acquire_i2c(), Address::Primary, I2cSlewRate::I2cSlwe2ns, &mut delay).unwrap();

    //let mut icm = Icm42688::new(bus.acquire_i2c(), Address::Primary).unwrap();
    if let Ok(mut icm) = Icm42688::new_i2c_slew(i2c, Address::Primary, I2cSlewRate::I2cSlwe2ns, &mut delay) {


        // let mut icm = Icm42688::new_i2c_slew(i2c, Address::Primary, I2cSlewRate::I2cSlwe2ns, &mut delay).unwrap();
        // let gyro_data =icm.gyro_norm();
        // println!("Gyro data: {:?}", gyro_data);
        // delay.delay_ms(500u32);

        loop {
            if let Ok(accel_data) = icm.accel_norm(){
                println!("accel_data: {:?}  ", accel_data);
            } else {
                println!("accel_data: error" );
                //icm.soft_reset();
            }
            if let Ok(gyro_data) = icm.gyro_norm(){
                println!("gyro_data: {:?}  ", gyro_data);
            } else  {
                println!("gyro_data: error" );
                //icm.soft_reset();
            }
            delay.delay_ms(5u32);
            
            // let accel_norm = icm.accel_norm().unwrap();
            // delay.delay_ms(500u32);
            // let gyro_norm = icm.gyro_norm().unwrap();
            // println!("Acel_norm: {:?}   Gyro norm: {:?}", accel_norm, gyro_norm);
            // let gyro_data =icm.gyro_norm();
            // println!("Gyro data: {:?}", gyro_data);
            // delay.delay_ms(500u32);
        }
    } else {
        println!("error while connecting to icm-42688 over i2c");
    };
    loop{
        delay.delay_ms(500u32);
        print!(".");
    }
}
