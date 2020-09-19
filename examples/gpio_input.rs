#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as _;
use cortex_m_rt as rt;
use panic_halt as _;
use stm32g4xx_hal as hal;

use hal::prelude::*;
use hal::stm32;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let btn = gpioa.pa12.into_pull_up_input();
    let mut led = gpiob.pb8.into_push_pull_output();

    loop {
        let delay = if btn.is_high().unwrap() { 1_000_000 } else { 250_000 };

        for _ in 0..delay {
            led.set_low().unwrap();
        }
        for _ in 0..delay {
            led.set_high().unwrap();
        }
    }
}
