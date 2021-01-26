#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate nb;
//extern crate panic_halt;
extern crate panic_semihosting;
extern crate stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use nb::block;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();

    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led = gpiob.pb8.into_push_pull_output();

    let mut timer = dp.TIM15.timer(&mut rcc);
    timer.start(500.ms());

    loop {
        led.toggle().unwrap();
        block!(timer.wait()).unwrap();
    }
}
