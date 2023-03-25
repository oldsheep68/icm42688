#![no_std]
#![no_main]

// use core::fmt::Write;
// use core::{cell::RefCell, fmt::Write};

use core::f32::consts::PI;
// use critical_section::Mutex;

use esp32c3_hal::{
    clock::ClockControl,
    gpio::IO,
    i2c::I2C,
    // interrupt,
    // peripherals,
    peripherals::Peripherals, //, Interrupt, USB_DEVICE},
    prelude::*,
    systimer::SystemTimer,
    timer::TimerGroup,
    // Cpu,
    Delay,
    Rtc,
};

use esp_backtrace as _;
use esp_println::println;

use icm42688::config::{AccelBw, GyroBw};
use icm42688::{Address, FifoDataP4, FifoDataSiP4, FifoPacketType, Icm42688}; //prelude::*,

use dcmimu::DCMIMU;

#[riscv_rt::entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut wdt0 = timer_group0.wdt;
    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut wdt1 = timer_group1.wdt;

    let _sys_t = SystemTimer::new(peripherals.SYSTIMER);

    rtc.swd.disable();
    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    let mut dcmimu = DCMIMU::new();

    let mut delay = Delay::new(&clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio3,
        io.pins.gpio2,
        1000u32.kHz(),
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let bus = shared_bus::BusManagerSimple::new(i2c);

    let mut icm = Icm42688::new_fifo(
        bus.acquire_i2c(),
        Address::Primary,
        FifoPacketType::Packet4,
        &mut delay,
    )
    .unwrap();
    let _ret = icm.set_gyro_bw(GyroBw::OdrEighth); // Gyro Bandwith should probably be even lower than half the sampling rate
    let _ret = icm.set_accel_bw(AccelBw::OdrEighth);
    let arange = icm.accel_range().unwrap();
    let _ascal = arange.scale_factor();
    let grange = icm.gyro_range().unwrap();
    let _gscal = grange.scale_factor();

    let mut count: u32 = 0;

    let mut last_now = SystemTimer::now();

    loop {
        let t1 = SystemTimer::now();
        let now = SystemTimer::now();
        let t3 = now;
        let delta_time = ((now - last_now) as f32) / SystemTimer::TICKS_PER_SECOND as f32;
        last_now = now;

        let fifo_cnt = icm.read_fifo_cnt();
        let ts = icm.read_tmst();

        let t4 = SystemTimer::now();
        let mut fifodata: [u8; 20] = [0; 20];
        let _ret = icm.read_fifo(0x3F as u8, &mut fifodata);
        // only continue, if fifo read contains valied data
        // if no data, do nothing
        if (fifodata[0] & 0x80) == 0x80 {
        } else {
            // else evaluate the data and present them every second
            let fdata = FifoDataP4::to_fifodata_raw(&mut fifodata);
            let fdatasi = FifoDataSiP4::to_fifodata_si(&fdata); //, ascal, gscal);

            let (dcm, _gyro_biases) = dcmimu.update(
                ((fdatasi.gx), (fdatasi.gy), (fdatasi.gz)),
                (fdatasi.ax, fdatasi.ay, fdatasi.az),
                fdatasi.ts,
            );
            let t5 = SystemTimer::now();

            count = count + 1;
            if count == 200 {
                count = 0;

                println!(
                        "fifo_cnt:{:?} T:{:?} ts:{:?}  \n{:?} \n roll{:?} yaw{:?} pitch{:?} \n{:?} \n{:?}",
                        fifo_cnt.unwrap(),
                        icm.temperature().unwrap(),
                        ts.unwrap(),
                        dcmimu.all(),
                        dcm.roll*180.0/PI, dcm.yaw*180.0/PI, dcm.pitch*180.0/PI,
                        fdata, // fifodata,
                        fdatasi,
                    );
                println!(
                    "{:?} t3-t1:{:?} t4-t3:{:?} t5-t4:{:?}",
                    delta_time, // SystemTimer::now()
                    (t3 - t1) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                    (t4 - t3) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                    (t5 - t4) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                );
            }

            // delay.delay_ms(1u32);
        }
    }
}
