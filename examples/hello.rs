#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as _;
use cortex_m_rt as rt;
use panic_halt as _;
use stm32g4xx_hal as _;

use rtt_target::{rprintln, rtt_init_print};
use rt::entry;

#[entry]
fn main() -> ! {
    rtt_init_print!();    
    loop {
        for i in 0.. {
            rprintln!("Hello, world {}", i);
            for _ in 0..1_000_000 {
                cortex_m::asm::nop();
            }
        }
    }
}
