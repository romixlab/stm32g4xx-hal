#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate stm32g4xx_hal as hal;
use panic_rtt_target as _;


use core::fmt::Write;

use hal::prelude::*;
use hal::serial::FullConfig;
use hal::stm32;
use nb::block;
use rtt_target::{rprintln, rtt_init_print};
use rt::entry;

#[entry]
fn main() -> ! {
    rtt_init_print!();    
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);
    let cfg = FullConfig::default();
    rprintln!("cfg: {:?}", cfg);
    let mut usart = dp
        .USART2
        .usart(gpioa.pa2, gpioa.pa3, cfg, &mut rcc)
        .unwrap();

    rprintln!("Starting uart");
    writeln!(usart, "Hello\r").unwrap();

    let mut cnt = 0;
    loop {
        let byte = block!(usart.read()).unwrap();
        writeln!(usart, "{}: {}\r", cnt, byte).unwrap();
        cnt += 1;
    }
}
