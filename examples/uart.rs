#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32g4xx_hal as hal;
extern crate panic_halt;

use core::fmt::Write;

use hal::prelude::*;
use hal::serial::FullConfig;
use hal::stm32;
use nb::block;
use rt::entry;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let cfg = FullConfig::default();
    let mut usart = dp
        .USART2
        .usart(gpioa.pa2, gpioa.pa3, cfg, &mut rcc)
        .unwrap();

    writeln!(usart, "Hello\r").unwrap();

    let mut cnt = 0;
    loop {
        let byte = block!(usart.read()).unwrap();
        writeln!(usart, "{}: {}\r", cnt, byte).unwrap();
        cnt += 1;
    }
}
