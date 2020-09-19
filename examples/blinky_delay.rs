#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as _;
use cortex_m_rt as rt;
use panic_halt as _;
use stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::rcc::Config;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let mut rcc = dp.RCC.freeze(Config::hsi());

    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    let mut delay = cp.SYST.delay(&rcc.clocks);

    loop {
        led.toggle().unwrap();
        delay.delay(500.ms());
    }
}
