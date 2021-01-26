use crate::gpio::gpioa::*;
use crate::gpio::gpiob::*;
use crate::gpio::gpioc::*;
use crate::gpio::gpiod::*;
// use crate::gpio::gpiof::*;
use crate::gpio::{AltFunction, DefaultMode};
use crate::stm32::*;
use crate::timer::*;

pub trait TimerPin<TIM> {
    type Channel;

    fn setup(&self);
}

macro_rules! timer_pins {
    ($TIMX:ident, [ $(($ch:ty, $pin:ty, $af_mode:expr),)+ ]) => {
        $(
            impl TimerPin<$TIMX> for $pin {
                type Channel = $ch;

                fn setup(&self) {
                    self.set_alt_mode($af_mode);
                }
            }
        )+
    };
}

timer_pins!(TIM1, [
    (Channel1, PA8<DefaultMode>, AltFunction::AF2),
    (Channel1, PC8<DefaultMode>, AltFunction::AF2),
    (Channel2, PA9<DefaultMode>, AltFunction::AF2),
    (Channel2, PB3<DefaultMode>, AltFunction::AF1),
    (Channel2, PC9<DefaultMode>, AltFunction::AF2),
    (Channel3, PA10<DefaultMode>, AltFunction::AF2),
    (Channel3, PB6<DefaultMode>, AltFunction::AF1),
    (Channel3, PC10<DefaultMode>, AltFunction::AF2),
    (Channel4, PA11<DefaultMode>, AltFunction::AF2),
    (Channel4, PC11<DefaultMode>, AltFunction::AF2),
]);

#[cfg(feature = "stm32g0x1")]
timer_pins!(TIM2, [
    (Channel1, PA0<DefaultMode>, AltFunction::AF2),
    (Channel1, PA5<DefaultMode>, AltFunction::AF2),
    (Channel1, PA15<DefaultMode>, AltFunction::AF2),
    (Channel1, PC4<DefaultMode>, AltFunction::AF2),
    (Channel2, PA1<DefaultMode>, AltFunction::AF2),
    (Channel2, PB3<DefaultMode>, AltFunction::AF2),
    (Channel2, PC5<DefaultMode>, AltFunction::AF2),
    (Channel3, PA2<DefaultMode>, AltFunction::AF2),
    (Channel3, PB10<DefaultMode>, AltFunction::AF2),
    (Channel3, PC6<DefaultMode>, AltFunction::AF2),
    (Channel4, PA3<DefaultMode>, AltFunction::AF2),
    (Channel4, PB11<DefaultMode>, AltFunction::AF2),
    (Channel4, PC7<DefaultMode>, AltFunction::AF2),
]);

timer_pins!(TIM3, [
    (Channel1, PA6<DefaultMode>, AltFunction::AF1),
    (Channel1, PB4<DefaultMode>, AltFunction::AF1),
    (Channel1, PC6<DefaultMode>, AltFunction::AF1),
    (Channel2, PA7<DefaultMode>, AltFunction::AF1),
    (Channel2, PB5<DefaultMode>, AltFunction::AF1),
    (Channel2, PC7<DefaultMode>, AltFunction::AF1),
    (Channel3, PB0<DefaultMode>, AltFunction::AF1),
    (Channel3, PC8<DefaultMode>, AltFunction::AF1),
    (Channel4, PB1<DefaultMode>, AltFunction::AF1),
    (Channel4, PC9<DefaultMode>, AltFunction::AF1),
]);

// timer_pins!(TIM14, [
//     (Channel1, PA4<DefaultMode>, AltFunction::AF4),
//     (Channel1, PA7<DefaultMode>, AltFunction::AF4),
//     (Channel1, PB1<DefaultMode>, AltFunction::AF0),
//     (Channel1, PC12<DefaultMode>, AltFunction::AF2),
//     (Channel1, PF0<DefaultMode>, AltFunction::AF2),
// ]);

#[cfg(any(feature = "stm32g070", feature = "stm32g071", feature = "stm32g081"))]
timer_pins!(TIM15, [
    (Channel1, PA2<DefaultMode>, AltFunction::AF5),
    (Channel1, PB14<DefaultMode>, AltFunction::AF5),
    (Channel1, PC1<DefaultMode>, AltFunction::AF2),
]);

timer_pins!(TIM16, [
    (Channel1, PA6<DefaultMode>, AltFunction::AF5),
    (Channel1, PB8<DefaultMode>, AltFunction::AF2),
    (Channel1, PD0<DefaultMode>, AltFunction::AF2),
]);

timer_pins!(TIM17, [
    (Channel1, PA7<DefaultMode>, AltFunction::AF6),
    (Channel1, PB9<DefaultMode>, AltFunction::AF2),
    (Channel1, PD1<DefaultMode>, AltFunction::AF2),
]);
