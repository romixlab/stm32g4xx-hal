#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as _;
use cortex_m_rt as rt;
use panic_halt as _;
use stm32g4xx_hal as hal;

use nb::block;
use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use hal::pwr::PWR;
use hal::lptim::{LpTimer, ClockSrc};
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi());
    let mut pwr = PWR::new(dp.PWR, &mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    let mut lptim = LpTimer::init_periodic(dp.LPTIMER1, &mut pwr, &mut rcc, ClockSrc::Lsi);

    loop {
        led.toggle().unwrap();
        lptim.start(2.hz());
        block!(lptim.wait()).unwrap();        
    }
}
