#![no_std]
#![no_main]

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

use icm42688::{Address, FifoDataP1, FifoDataSiP1, FifoPacketType, Icm42688}; // prelude::*,
use icm42688::config::AccelRange;

// use core::f32::consts::PI;

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

    let _sys_t = SystemTimer::new(peripherals.SYSTIMER);

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
    let ascal = arange.scale_factor();
   
    let mut count: usize = 0;

    let mut last_now = SystemTimer::now();

    loop {
        let t1 = SystemTimer::now();
        // let accel_norm = icm.accel_norm().unwrap();
        let t2 = SystemTimer::now();
        let now = SystemTimer::now();
        let t3 = now;
        let delta_time = ((now - last_now) as f32) / SystemTimer::TICKS_PER_SECOND as f32;
        last_now = now;

        let fifo_cnt = icm.read_fifo_cnt();
        let ts = icm.read_tmst();

        let t4 = SystemTimer::now();
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
            let fdatasi = FifoDataSiP1::to_fifodata_si(&mut fifodata, ascal);

            // println!("{:?}", fifodata);
            // println!("fifo count: {:?}", icm.read_fifo_cnt());

            let t5 = SystemTimer::now();

            count = count + 1;
            if count == 100 {
                count = 0;
                //println!("arry accel data: {:?}",accel_data_arry);

                println!(
                    "fifo_cnt:{:?}  T:{:?} ts {:?}  {:?} {:?}",
                    fifo_cnt.unwrap(),
                    icm.temperature().unwrap(),
                    ts.unwrap(),
                    fdata,
                    fdatasi,
                );
               
                println!(
                    "{:?} t2-t1:{:?} t3-t2:{:?} t4-t3:{:?} t5-t4:{:?}",
                    delta_time, // SystemTimer::now()
                    (t2 - t1) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                    (t3 - t2) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                    (t4 - t3) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                    (t5 - t4) as f32 / SystemTimer::TICKS_PER_SECOND as f32,
                );
            }
            // delay.delay_ms(10u32);
        }
    }
}
