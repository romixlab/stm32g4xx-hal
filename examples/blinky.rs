#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

<<<<<<< HEAD
//use cortex_m;
use cortex_m_rt as rt;
#[allow(unused_imports)]
use panic_halt;
=======
use cortex_m as _;
use cortex_m_rt as rt;
use panic_halt as _;
>>>>>>> bc0e085a1af15cdc653182f495d66ca35659f37d
use stm32g4xx_hal as hal;

use crate::hal::prelude::*;
use crate::hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.freeze(hal::rcc::Config::pll());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa5.into_push_pull_output();

    loop {
        for _ in 0..1_00_000 {
            led.set_low().unwrap();
        }
        for _ in 0..1_00_000 {
            led.set_high().unwrap();
        }
    }
}
