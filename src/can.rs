
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

use vcell::VolatileCell;

#[repr(C)]
pub struct RxFifoElement{
    pub r0: R0,
    pub r1: R1,
    pub r2_18: [R2_18; 16],
    //_reserved1: [u32; 16usize],
}

pub struct R0{
    register: vcell::VolatileCell<u32>
}
pub mod rx_fifo_element_r0;

pub struct R1{
    register: vcell::VolatileCell<u32>
}
pub mod rx_fifo_element_r1;

pub struct R2_18{
    register: vcell::VolatileCell<u32>
}
pub mod rx_fifo_element_r2_18;

#[repr(C)]
pub struct TxBufferElement{
    pub t0: T0,
    pub t1: T1,
    pub t2_18: [T2_18; 16],
    //_reserved1: [u32; 16usize],
}

pub struct T0{
    register: vcell::VolatileCell<u32>
}
pub mod tx_buffer_element_t0;

pub struct T1{
    register: vcell::VolatileCell<u32>
}
pub mod tx_buffer_element_t1;

pub struct T2_18{
    register: vcell::VolatileCell<u32>
}
pub mod tx_buffer_element_r2_18;


#[repr(C)]
pub struct TxEventFifoElement{
    pub e0: E0,
    pub e1: E1,
    //_reserved1: [u32; 16usize],
}

pub struct E0{
    register: vcell::VolatileCell<u32>
}
pub mod tx_event_fifo_element_e0;

pub struct E1{
    register: vcell::VolatileCell<u32>
}
pub mod tx_event_fifo_element_e1;


#[repr(C)]
pub struct StandartMessageIdFilter{
    pub s0: S0,
}

pub struct S0{
    register: vcell::VolatileCell<u32>
}
pub mod standart_message_id_filter_s0;

#[repr(C)]
pub struct ExtendedMessageIdFilter{
    pub f0: F0,
    pub f1: F1,
}

pub struct F0{
    register: vcell::VolatileCell<u32>
}
pub mod extended_message_id_filter_f0;

pub struct F1{
    register: vcell::VolatileCell<u32>
}
pub mod extended_message_id_filter_f1;





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
                //rcc.rb.ccpir1.modify(|_, w| w.fdcansel().bits()); // 0b00 - HSE, 0b01 - PLLQ, 0b10 - PCLK, 0b11 - RSVD
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