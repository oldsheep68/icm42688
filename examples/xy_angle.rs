#![no_std]
#![no_main]

/// The xy-plane is used to rotate in the gravity feeld.
/// An angel can be calculated in this direction using the x and y acceleration of the gravity feeld.
/// an other linear accelartion will impact the result



// use core::fmt::Write;
// use core::{cell::RefCell, fmt::Write};

// use critical_section::Mutex;

use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    // interrupt,
    // peripherals,
    peripherals::Peripherals, // Interrupt,, USB_DEVICE
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    // Cpu,
    Delay,
    Rtc,
};

use esp_backtrace as _;
use esp_println::println;

use icm42688::{Address, FifoDataP1, FifoPacketType, Icm42688}; // prelude::*, FifoDataSiP1, 
use icm42688::config::{AccelRange, AccelOdr};
// use shtcx;
// use shtcx::PowerMode;

use libm::*;

use core::f32::consts::PI;

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();
    println!("Hellow from Main");

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    let mut _sys_t = SystemTimer::new(peripherals.SYSTIMER);

    const K:f32 = 180.0/PI;
    const AVERAGE_NUMBER: usize = 32;

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio3, // sda
        io.pins.gpio2, // scl
        1000u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);
    println!("I2C created");

    let mut icm = Icm42688::new_fifo(
        bus.acquire_i2c(),
        Address::Primary,
        FifoPacketType::Packet1,
        &mut delay,
    )
    .unwrap();
    println!("icm_fifo created");

    let _ret = icm.set_accel_range(AccelRange::G2);
    let arange = icm.accel_range().unwrap();
    println!("icm_accel_range {:?}", arange);
    let _ascal = arange.scale_factor();
    let _ret = icm.set_accel_odr(AccelOdr::Hz32000);

   
    let mut count: usize = 0;
    let mut xy_angle_min: f32 = 91.0;
    let mut xy_angle_max: f32 = -91.0;
    let mut x_averagef = 0f32;
    let mut y_averagef = 0f32;
    let mut angle360: f32 = 0f32;

    // let mut last_now = SystemTimer::now();

    let mut accel_data_arry: [FifoDataP1; AVERAGE_NUMBER] = [FifoDataP1{ax: 0i16, ay: 0i16, az: 0i16, t: 0i8}; AVERAGE_NUMBER];

    loop {
        let mut fifodata: [u8; 16] = [0; 16];

        // read fifo data from register 0x30
        let _ret = icm.read_fifo(0x30 as u8, &mut fifodata);
        // only continue, if fifo read contains valied data
        // if no data, do nothing
        if (fifodata[0] & 0x80) == 0x80 {
            //println!("fifodata[0] = 0x80 or 0xff...; fifodata = {:?}", fifodata);
        } else {
            // else evaluate the data and present them every second
            let fdata = FifoDataP1::to_fifodata_raw(&mut fifodata);
            //let fdatasi = FifoDataSiP1::to_fifodata_si(&mut fifodata, ascal);

            // println!("{:?}", fifodata);
            // println!("fifo count: {:?}", icm.read_fifo_cnt());

            count = count + 1;
            if count == AVERAGE_NUMBER {
                count = 0;
                //println!("arry accel data: {:?}",accel_data_arry);

                let mut x_average = 0i32;
                let mut y_average = 0i32;
                //let mut z_average = 0i32;
                for data in accel_data_arry {
                    x_average += data.ax as i32;
                    y_average += data.ay as i32;
                    // z_average += data.az as i32;
                }
                let x_averagef = x_average as f32 / AVERAGE_NUMBER as f32;
                let y_averagef = y_average as f32 / AVERAGE_NUMBER as f32;
                // let z_average = z_average as f32 / AVERAGE_NUMBER as f32;

                let xy_avarage_a = K*atanf(x_averagef/y_averagef); // scale: 16_384.0

                if xy_avarage_a > xy_angle_max {
                    xy_angle_max = xy_avarage_a;
                }

                if xy_avarage_a < xy_angle_min {
                    xy_angle_min = xy_avarage_a;
                }

                
                if x_averagef > 0.0 && y_averagef > 0.0 {
                    angle360 = xy_avarage_a;
                } else if x_averagef < 0.0 && y_averagef < 0.0 {
                    angle360 = xy_avarage_a - 180.0;
                } else if x_averagef < 0.0 && y_averagef > 0.0 {
                    angle360 = xy_avarage_a;
                } else if x_averagef > 0.0 && y_averagef < 0.0 {
                    angle360 = xy_avarage_a + 180.0;
                }


              
                // println!("x_average: {:?}, y_average: {:?}, z_average: {:?}", x_average, y_average, z_average);
                // println!("x_average_g: {:?}, y_average_g: {:?}, z_average_g: {:?}", x_average/16_384.0, y_average/16_384.0, z_average/16_384.0);
                
                // println!("x_angle: {:?}, y_angle_g: {:?}, z_anglee_g: {:?}", K*asinf(x_average/16_384.0), K*asinf(y_average/16_384.0), K*asinf(z_average/16_384.0));
                println!("x_angle: {:.3} {:.3} {:.3} {:.3} xa: {:.3} ya: {:.3}", angle360, 
                                                             xy_angle_min, 
                                                             xy_angle_max, 
                                                             xy_angle_max-xy_angle_min,
                                                             x_averagef,
                                                             y_averagef );
            } else {
                
                if count < AVERAGE_NUMBER {
                    // println!("{:?}", fdata);
                    accel_data_arry[count as usize] = fdata;
                }

            }

            // delay.delay_ms(10u32);
        }
    }
}
