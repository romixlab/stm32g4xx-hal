use crate::gpio::{
    gpioa::{ PA8, PA11, PA12, PA15 },
    gpiob::{ PB3, PB4, PB5, PB6, PB8, PB9 },
    gpiod::{ PD0, PD1, },
    //gpioe::{ PE0 }
};
use crate::gpio::{AltFunction, DefaultMode};
use crate::pac::{FDCAN1, FDCAN2, FDCAN3};
use crate::rcc::Rcc;
use cortex_m::peripheral::DWT;

#[derive(Debug)]
pub enum Error {
    ResetFail,
    WriteUnlockFail,

}

pub trait Pins<CAN> {
    fn setup(&self);
}

pub trait PinTx<CAN> {
    fn setup(&self);
}

pub trait PinRx<CAN> {
    fn setup(&self);
}

impl<CAN, CAN_TX, CAN_RX> Pins<CAN> for (CAN_TX, CAN_RX)
    where
        CAN_TX: PinTx<CAN>,
        CAN_RX: PinRx<CAN>,
{
    fn setup(&self) {
        self.0.setup();
        self.1.setup();
    }
}

pub struct Can<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

pub struct ClassicalCan<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

pub struct FdCan<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

pub enum CanType {
    Classical,
    FdIso,
    FdBosch
}

#[derive(PartialEq)]
pub enum Mode {
    Normal,
    BusMonitoring,
    //Test
}

#[derive(PartialEq)]
pub enum Retransmission {
    Enabled,
    Disabled
}

#[derive(PartialEq)]
pub enum TransmitPause {
    Enabled,
    Disabled
}

pub struct BitTiming {
    ntseg1: u8,
    ntseg2: u8,
    nsjw: u8,
    baudrate: u32
}

impl BitTiming {
    pub fn default_1mbps() -> Self {
        BitTiming {
            ntseg1: 57,
            ntseg2: 20,
            nsjw: 20,
            baudrate: 1_000_000
        }
    }
}

// pub trait ClassicalCanInitState {
//     fn set_bitrate();
// }
//
// pub trait ClassicalCanNormalState {
//
// }
//
// pub trait ClassicalCanBusOffState {
//
// }
//
// pub trait FdCanInitState {
//     fn set_arbitration_bitrate();
//     fn set_data_bitrate();
//
// }
// pub trait RangeFilter {
//     fn filter_count() -> usize;
//     fn filter_range(from to)
// }
// pub trait CanExt<CAN>: Sized {
//     fn can<PINS>(self, pins: PINS, rcc: &mut Rcc) -> Can<CAN, PINS>
//         where
//             PINS: Pins<CAN>;
// }

macro_rules! checked_wait_or {
    ($check:expr, $err:expr) => {
        let mut instant = DWT::get_cycle_count();
        while $check {
            if DWT::get_cycle_count() - instant > 100 {
                return Err($err);
            }
        }
    };
}

macro_rules! can {
    (
        $CANX:ident, $canX:ident,
        can_tx: [ $(($CAN_TX:ty, $CAN_TX_AF:expr),)+ ],
        can_rx: [ $(($CAN_RX:ty, $CAN_RX_AF:expr),)+ ],
    ) => {
        $(
            impl PinTx<$CANX> for $CAN_TX {
                fn setup(&self) {
                    self.set_alt_mode($CAN_TX_AF);
                }
            }
        )*
        $(
            impl PinRx<$CANX> for $CAN_RX {
                fn setup(&self) {
                    self.set_alt_mode($CAN_RX_AF);
                }
            }
        )*
        impl<PINS> Can<$CANX, PINS> {
            pub fn new_classical(
                can: $CANX,
                pins: PINS,
                mode: Mode,
                retransmission: Retransmission,
                transmit_pause: TransmitPause,
                bit_timing: BitTiming,
                rcc: &mut Rcc,
                dwt: &mut DWT
            ) -> Result<ClassicalCan<$CANX, PINS>, Error>
            where
                PINS: Pins<$CANX>,
            {
                // Configure pins
                pins.setup();
                // Enable clock & reset
                rcc.rb.apb1enr1.modify(|_, w| w.fdcanen().set_bit());
                rcc.rb.apb1rstr1.modify(|_, w| w.fdcanrst().set_bit());
                rcc.rb.apb1rstr1.modify(|_, w| w.fdcanrst().clear_bit());
                // Select PLLQ as
                rcc.rb.ccpir1.modify(|_, w| w.fdcansel().bits()); // 0b00 - HSE, 0b01 - PLLQ, 0b10 - PCLK, 0b11 - RSVD
                // Enable DWT if not already
                dwt.enable_cycle_counter();
                // Check that reset succeeded
                checked_wait_or!(can.cccr.read().bits() != 0x0000_0001, Error::ResetFail);
                // Unlock write access
                can.cccr.modify(|_, w| w.cce().set_bit());
                checked_wait_or!(can.cccr.read().cce().bit_is_clear(), Error::WriteUnlockFail);

                can.cccr.modify(|_, w| w
                    .mon().bit(mode == Mode::BusMonitoring)
                    .dar().bit(retransmission == Retransmission::Disabled)
                    .txp().bit(transmit_pause == TransmitPause::Enabled)
                );

                Ok(ClassicalCan { can, pins })
            }
        }

    };
}

can!(
    FDCAN1,
    fdcan1,
    can_tx: [
        (PA12<DefaultMode>, AltFunction::AF9),
        (PB9<DefaultMode>,  AltFunction::AF9),
        //(PD1<DefaultMode>,  AltFunction::AF9)
    ],
    can_rx: [
        (PA11<DefaultMode>, AltFunction::AF9),
        (PB8<DefaultMode>,  AltFunction::AF9),
        //(PD0<DefaultMode>,  AltFunction::AF9),
        //(PE0<DefaultMode>,  AltFunction::AF9)
    ],
);

// can!(
//     CAN2,
//     can_tx: [
//         (PB6<DefaultMode>,  AltFunction::AF9),
//         (PB13<DefaultMode>, AltFunction::AF9)
//     ],
//     can_rx: [
//         (PB5<DefaultMode>,  AltFunction::AF9),
//         (PB12<DefaultMode>, AltFunction::AF9)
//     ]
// );
//
// can!(
//     CAN3,
//     can_tx: [
//         (PA15<DefaultMode>, AltFunction::AF11),
//         (PB4<DefaultMode>,  AltFunction::AF11)
//     ],
//     can_rx: [
//         (PA8<DefaultMode>,  AltFunction::AF11),
//         (PB3<DefaultMode>,  AltFunction::AF11)
//     ]
// );