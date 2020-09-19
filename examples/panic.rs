#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m as _;
use cortex_m_rt as rt;
use panic_rtt_target as _;
use stm32g4xx_hal as _;

use rtt_target::{rtt_init_print};
use rt::entry;

#[entry]
fn main() -> ! {
    rtt_init_print!();    
    panic!("panic test");
}
