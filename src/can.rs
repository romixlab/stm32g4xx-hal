use core::ops::Deref;

use crate::gpio::{
    gpioa::{PA11, PA12, PA15, PA8},
    gpiob::{PB3, PB4, PB5, PB6, PB8, PB9},
    gpiod::{PD0, PD1},
    //gpioe::{ PE0 }
};
use crate::gpio::{AltFunction, DefaultMode};
use crate::pac::fdcan::RegisterBlock;
use crate::pac::FDCAN1;
use crate::rcc::Rcc;
use cortex_m::peripheral::DWT;

use crate::time::Hertz;
use vcell::VolatileCell;
use vhrdcan::id::FrameId;

/// Rx FIFO 0 and 1 R0 register
pub mod rx_fifo_element_r0;
/// Rx FIFO 0 and 1 R1 register
pub mod rx_fifo_element_r1;
/// Tx Buffer 0, 1, 2 T0 register
pub mod tx_buffer_element_t0;
pub mod tx_buffer_element_t1;

pub mod extended_message_id_filter_f0;
pub mod extended_message_id_filter_f1;
pub mod standart_message_id_filter;
pub mod tx_event_fifo_element_e0;
pub mod tx_event_fifo_element_e1;

struct MessageRam {
    _marker: PhantomData<*const ()>,
}

#[repr(C)]
struct MessageRamBlock {
    /// Standard filters, 28 x 4 bytes each, offset = 0 bytes (0x4000a400)
    sid_filter: [StandartMessageIdFilter; 28],
    /// Extended filters, 8 x 8 bytes each, offset = 112 bytes (0x4000a470)
    eid_filter: [ExtendedMessageIdFilter; 8],
    /// RX FIFO 0 x 3 x 72 bytes each, offset = 176 bytes (0x4000a4b0)
    rx_fifo_0: [RxFifoElement; 3],
    /// RX FIFO 1 x 3 x 72 bytes each, offset = 392 bytes (0x4000a588)
    rx_fifo_1: [RxFifoElement; 3],
    /// TX event FIFO x 3 x 8 bytes each, offset = 608 bytes (0x4000a660)
    tx_event_fifo: [TxEventFifoElement; 3],
    /// TX buffer x 3 x 72 bytes each, offset = 632 bytes (0x4000a678), end = 848
    tx_buffer: [TxBufferElement; 3],
}

impl MessageRam {
    pub fn get() -> Self {
        MessageRam {
            _marker: PhantomData,
        }
    }
    #[doc = r"Returns a pointer to the RAM block"]
    #[inline(always)]
    pub const fn ptr() -> *const MessageRamBlock {
        0x4000_a400 as *const _
    }
}

impl Deref for MessageRam {
    type Target = MessageRamBlock;
    fn deref(&self) -> &Self::Target {
        unsafe { &*MessageRam::ptr() }
    }
}

const SIZE_OF_MESSAGE_RAM: usize = 848;
fn _check_message_ram_size() {
    // If that doesn't compile, message ram size is incorrect
    unsafe {
        core::mem::transmute::<MessageRamBlock, [u8; SIZE_OF_MESSAGE_RAM]>(panic!());
    }
}

#[repr(C)]
#[doc = "Standard message ID Filter element, up to 28 of such filters can be configured"]
pub struct StandartMessageIdFilter {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct ExtendedMessageIdFilterF0 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct ExtendedMessageIdFilterF1 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct ExtendedMessageIdFilter {
    pub f0: ExtendedMessageIdFilterF0,
    pub f1: ExtendedMessageIdFilterF1,
}

#[repr(C)]
pub struct RxFifoElementR0 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct RxFifoElementR1 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct RxFifoElement {
    pub r0: RxFifoElementR0,
    pub r1: RxFifoElementR1,
    pub data: [vcell::VolatileCell<u32>; 16],
}
impl RxFifoElement {
    unsafe fn data(&self) -> &[u8] {
        let p = (self as *const _ as *const u8).offset(8);
        let dlc = self.r1.read().dlc().bits() as usize;
        core::slice::from_raw_parts(p, dlc)
    }
}

#[repr(C)]
pub struct TxEventFifoElementE0 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct TxEventFifoElementE1 {
    register: vcell::VolatileCell<u32>,
}

#[repr(C)]
pub struct TxEventFifoElement {
    pub e0: TxEventFifoElementE0,
    pub e1: TxEventFifoElementE1,
}

#[repr(C)]
pub struct TxBufferElementT0 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct TxBufferElementT1 {
    register: vcell::VolatileCell<u32>,
}
#[repr(C)]
pub struct TxBufferElement {
    pub t0: TxBufferElementT0,
    pub t1: TxBufferElementT1,
    pub data: [vcell::VolatileCell<u32>; 16],
}

/// Standard filter configuration
/// All enabled filter elements are used for acceptance filtering of standard frames.
/// Acceptance filtering stops at the first matching enabled filter element or when the end
/// of the filter list is reached. If SFEC = 100, 101 or 110 a match sets interrupt flag
/// IR.HPM and, if enabled, an interrupt is generated. In this case register HPMS is
/// updated with the status of the priority match.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum FilterConfiguration {
    #[doc = "Disable filter element"]
    Disabled = 0b000,
    #[doc = "Store in Rx FIFO 0 if filter matches"]
    Store0 = 0b001,
    #[doc = "Store in Rx FIFO 1 if filter matches"]
    Store1 = 0b010,
    #[doc = "Reject ID if filter matches"]
    Reject = 0b011,
    #[doc = "Set priority if filter matches"]
    SetPriority = 0b100,
    #[doc = "Set priority and store in FIFO 0 if filter matches"]
    SetPriorityAndStore0 = 0b101,
    #[doc = "Set priority and store in FIFO 1 if filter matches"]
    SetPriorityAndStore1 = 0b110,
    #[doc = "Not used"]
    NotUsed = 0b111,
}
impl From<FilterConfiguration> for u8 {
    fn from(variant: FilterConfiguration) -> Self {
        variant as _
    }
}

#[derive(Debug)]
pub enum Error {
    ResetFail,
    WriteUnlockFail,
    IncorrectRccConfig,
    InitFail,
    NoSlotsAvailable,
    WrongFrameLength,
    TfqpiIndexInvalid, // 3 elements available but datasheet says that TFQPI can be 3
}

#[derive(Copy, Clone)]
pub enum ClockSource {
    Hse(Hertz),
    Pllq,
    Pclk,
}
impl Into<u8> for ClockSource {
    fn into(self) -> u8 {
        use ClockSource::*;
        match self {
            Hse(_) => 0b00,
            Pllq => 0b01,
            Pclk => 0b10,
        }
    }
}

pub enum ClockDiv {
    Div1 = 0b0000,
    Div2 = 0b0001,
    Div4 = 0b0010,
    Div6 = 0b0011,
    Div8 = 0b0100,
    Div10 = 0b0101,
    Div12 = 0b0110,
    Div14 = 0b0111,
    Div16 = 0b1000,
    Div18 = 0b1001,
    Div20 = 0b1010,
    Div22 = 0b1011,
    Div24 = 0b1100,
    Div26 = 0b1101,
    Div28 = 0b1110,
    Div30 = 0b1111,
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

/// One of the CAN controller instances
pub struct CanInstance<CAN, PINS> {
    can: CAN,
    pins: PINS,
}

/// Consumed pins, maybe be used to reinitialize them later for other use (maybe for interrupt enable on RX pin?).
pub struct CanPinsToken<PINS> {
    pins: PINS,
}

#[derive(PartialEq)]
pub enum Mode {
    /// Node can transmit and receive frames.
    Normal,
    /// Node can only receive frames, always transmitting recessive bits.
    BusMonitoring,
    /// Treat own messages as received, messages can be seen on TX pin, RX ignored.
    ExternalLoopBack,
    /// Treat own messages as received and do not disturb the bus (TX always recessive, RX ignored).
    InternalLoopBack,
}

impl Mode {
    pub fn is_test_mode(&self) -> bool {
        use Mode::*;
        match self {
            Normal => false,
            BusMonitoring => false,
            ExternalLoopBack => true,
            InternalLoopBack => true,
        }
    }

    pub fn is_mon_mode(&self) -> bool {
        use Mode::*;
        match self {
            Normal => false,
            BusMonitoring => true,
            ExternalLoopBack => false,
            InternalLoopBack => true,
        }
    }
}

#[derive(PartialEq)]
pub enum Retransmission {
    Enabled,
    Disabled,
}

#[derive(PartialEq)]
pub enum TransmitPause {
    Enabled,
    Disabled,
}

#[derive(PartialEq)]
pub enum TxBufferMode {
    Queue,
    Fifo,
}

pub struct BitTiming {
    ntseg1: u8,
    ntseg2: u8,
    nsjw: u8,
    baudrate: u32,
}

impl BitTiming {
    pub fn default_1mbps() -> Self {
        BitTiming {
            ntseg1: 57,
            ntseg2: 20,
            nsjw: 20,
            baudrate: 1_000_000,
        }
    }
}

pub struct ProtocolStatus(pub crate::pac::fdcan::psr::R);

impl ProtocolStatus {
    pub fn is_bus_off(&self) -> bool {
        self.0.bo().bit_is_set()
    }
}

use crate::can::standart_message_id_filter::FilterType;
use core::fmt;
use core::marker::PhantomData;
use vhrdcan::Frame;

impl fmt::Debug for ProtocolStatus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "PSR(Flags:");
        if self.0.bo().bit_is_set() {
            write!(f, "BO|");
        }
        if self.0.ew().bit_is_set() {
            write!(f, "EW|");
        }
        if self.0.ep().bit_is_set() {
            write!(f, "EP|");
        }
        let act = (self.0.bits() & 0b11000) as u8 >> 3;

        write!(f, " Status:");
        match act {
            0b00 => {
                write!(f, "Sync");
            }
            0b01 => {
                write!(f, "Idle");
            }
            0b10 => {
                write!(f, "Rx");
            }
            0b11 => {
                write!(f, "Tx");
            }
            _ => {
                unreachable!()
            }
        }
        write!(f, " LEC:");
        match self.0.lec().bits() {
            0b000 => {
                write!(f, "NoError|");
            }
            0b001 => {
                write!(f, "Stuff|");
            }
            0b010 => {
                write!(f, "Form|");
            }
            0b011 => {
                write!(f, "Ack|");
            }
            0b100 => {
                write!(f, "Bit1|");
            }
            0b101 => {
                write!(f, "Bit0|");
            }
            0b110 => {
                write!(f, "CRC|");
            }
            0b111 => {
                write!(f, "NoChange|");
            }
            _ => {
                unreachable!()
            }
        }
        write!(f, ")")
    }
}

pub struct InterruptReason(pub crate::pac::fdcan::ir::R);

impl fmt::Debug for InterruptReason {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "IR(");
        if self.0.ara().bit_is_set() {
            write!(f, "ARA|");
        }
        if self.0.ped().bit_is_set() {
            write!(f, "PED|");
        }
        if self.0.pea().bit_is_set() {
            write!(f, "PEA|");
        }
        if self.0.wdi().bit_is_set() {
            write!(f, "WDI|");
        }
        if self.0.bo().bit_is_set() {
            write!(f, "BO|");
        }
        if self.0.ew().bit_is_set() {
            write!(f, "EW|");
        }
        if self.0.ep().bit_is_set() {
            write!(f, "EP|");
        }
        if self.0.elo().bit_is_set() {
            write!(f, "ELO|");
        }
        if self.0.too().bit_is_set() {
            write!(f, "TOO|");
        }
        if self.0.mraf().bit_is_set() {
            write!(f, "MRAF|");
        }
        if self.0.tsw().bit_is_set() {
            write!(f, "TSW|");
        }
        if self.0.tefl().bit_is_set() {
            write!(f, "TEFL|");
        }
        if self.0.teff().bit_is_set() {
            write!(f, "TEFF|");
        }
        if self.0.tefn().bit_is_set() {
            write!(f, "TEFN|");
        }
        if self.0.tfe().bit_is_set() {
            write!(f, "TFE|");
        }
        if self.0.tcf().bit_is_set() {
            write!(f, "TCF|");
        }
        if self.0.tc().bit_is_set() {
            write!(f, "TC|");
        }
        if self.0.hpm().bit_is_set() {
            write!(f, "HPM|");
        }
        write!(f, " RF1:");
        if self.0.rf1l().bit_is_set() {
            write!(f, "L|");
        }
        if self.0.rf1f().bit_is_set() {
            write!(f, "F|");
        }
        if self.0.rf1n().bit_is_set() {
            write!(f, "N|");
        }
        write!(f, " RF0:");
        if self.0.rf0l().bit_is_set() {
            write!(f, "L|");
        }
        if self.0.rf0f().bit_is_set() {
            write!(f, "F|");
        }
        if self.0.rf0n().bit_is_set() {
            write!(f, "N|");
        }
        write!(f, ")")
    }
}

#[derive(Debug)]
pub enum RxPinState {
    Dominant,
    Recessive,
}

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

#[derive(Copy, Clone)]
pub enum CanInstanceNumber {
    Fdcan1 = 0,
    Fdcan2 = 1,
    Fdcan3 = 2,
}
impl CanInstanceNumber {
    pub fn mask(&self) -> u8 {
        1 << (*self as u8)
    }
    pub fn instance_number(&self) -> usize {
        *self as usize
    }
}
/// CAN instance with erased types
pub struct ClassicalCanInstance {
    regs: *const RegisterBlock,
    instance: CanInstanceNumber,
    txbuffer_mode: TxBufferMode,
    // regs2: *const RegisterBlock,
    // regs3: *const RegisterBlock
}
unsafe impl Send for ClassicalCanInstance {}
impl ClassicalCanInstance {
    pub fn send(&mut self, frame: &Frame) -> Result<(), Error> {
        if frame.len() > 8 {
            return Err(Error::WrongFrameLength);
        }
        if self.free_slots() == 0 {
            return Err(Error::NoSlotsAvailable);
        }
        let txfqs = self.regs().txfqs.read();
        if txfqs.tfqpi().bits() > 2 {
            return Err(Error::TfqpiIndexInvalid);
        }
        let mr = &MessageRam::get();

        let tx_buffer_put_index = match self.txbuffer_mode {
            TxBufferMode::Queue => self.instance.instance_number(),
            TxBufferMode::Fifo => txfqs.tfqpi().bits() as usize,
        };

        let mut fours = frame.len() / 4;
        if frame.len() % 4 != 0 {
            fours += 1;
        }
        for i in 0..fours {
            let to_copy = if (i != fours - 1) || (frame.len() % 4 == 0) {
                4
            } else {
                frame.len() % 4
            };
            let mut fourbytes = 0u32;
            unsafe {
                core::ptr::copy_nonoverlapping(
                    frame.data().as_ptr().offset(i as isize * 4),
                    &mut fourbytes as *mut _ as *mut u8,
                    to_copy,
                );
            }
            mr.tx_buffer[tx_buffer_put_index].data[i].set(fourbytes);
        }

        mr.tx_buffer[tx_buffer_put_index]
            .t0
            .write(|w| w.id().frame_id(frame.id()));
        unsafe {
            mr.tx_buffer[tx_buffer_put_index]
                .t1
                .write(|w| w.dlc().bits(frame.len() as u8))
        };
        let pend_slot_mask = match self.txbuffer_mode {
            TxBufferMode::Queue => self.instance.mask() as u32,
            TxBufferMode::Fifo => 1 << txfqs.tfqpi().bits(),
        };
        unsafe { self.regs_mut().txbar.write(|w| w.ar().bits(pend_slot_mask)) };

        Ok(())
    }

    pub fn get_all<F: FnMut(FrameId, &[u8])>(&mut self, mut f: F) {
        for _ in 0..3 {
            let fifo_status = self.regs().rxf0s.read();
            if fifo_status.f0fl().bits() > 0 {
                let get_index = fifo_status.f0gi().bits();
                let mr = &MessageRam::get();
                let id = mr.rx_fifo_0[get_index as usize].r0.read().id().frame_id();
                f(id, unsafe { mr.rx_fifo_0[get_index as usize].data() });
                unsafe { self.regs().rxf0a.write(|w| w.f0ai().bits(get_index)) };
            } else {
                break;
            }
        }
    }
}
// Unfortunately there doesn't seem to be a good way to see what other instances are using
// Probably need to keep pointers to other FDCANs, different MCUs have different amount of them,
// code will quickly become ugly... Several instances using the same RAM seems to be convinient for redundancy,
// not if you need to build a gateway for example.
// pub enum SlotState {
//     /// No pending bit set, safe to do anything.
//     Empty,
//     /// Pending bit set, but there are other slots pending with higher priority (lower id), should be safe to swap slots if done fast enough.
//     Queued,
//     /// Highest priority slot for this CAN instance, meaning it is probably touched by the hw right now.
//     AccessInProgress,
//     /// Not used right now
//     UsedByOtherInstance
// }

pub trait ClassicalCan {
    fn regs(&self) -> &RegisterBlock;
    unsafe fn regs_mut(&mut self) -> &mut RegisterBlock;
    // unsafe fn aliased_instances(&self) -> [*const RegisterBlock; 2];

    fn total_error_count(&self) -> u8 {
        unsafe { self.regs().ecr.read().cel().bits() }
    }

    fn receive_error_counter(&self) -> u8 {
        unsafe { self.regs().ecr.read().trec().bits() }
    }

    fn transmit_error_counter(&self) -> u8 {
        unsafe { self.regs().ecr.read().tec().bits() }
    }

    fn protocol_status(&self) -> ProtocolStatus {
        ProtocolStatus(unsafe { self.regs().psr.read() })
    }

    fn interrupt_reason(&self) -> InterruptReason {
        InterruptReason(unsafe { self.regs().ir.read() })
    }

    fn rx_pin_state(&self) -> RxPinState {
        if unsafe { self.regs().test.read().rx().bit_is_set() } {
            RxPinState::Recessive
        } else {
            RxPinState::Dominant
        }
    }

    fn free_slots(&self) -> usize;
    // fn next_queue_slot(&self) -> Option<u8> {
    //     let trp = (unsafe { (*self.regs()).txbrp.read().trp().bits() } & 0b111) as u8;
    //     match trp {
    //         0b000 => Some(0),
    //         0b001 => Some(1),
    //         0b010 => Some(0),
    //         0b011 => Some(2),
    //         0b100 => Some(0),
    //         0b101 => Some(1),
    //         0b110 => Some(0),
    //         0b111 => None,
    //         _ => unreachable!()
    //     }
    // }
    // fn slot_count() -> usize {
    //     3
    // }
    // fn lowest_queued_id(&self) -> Option<FrameId> {
    //     let mut lowest_id = None;
    //     let trp_this_instance = (unsafe { (*self.regs()).txbrp.read().trp().bits() } & 0b111) as u8;
    //     let mr = &MessageRam::get();
    //     for i in 0..=2 {
    //         let mask = 0b1u8 << i;
    //         if trp_this_instance & mask != 0 {
    //             let id = mr.tx_buffer[i].t0.read().id().frame_id();
    //             if lowest_id.is_none() {
    //                 lowest_id = Some(id);
    //             } else if id < lowest_id.unwrap() {
    //                 lowest_id = Some(id);
    //             }
    //         }
    //     }
    //     lowest_id
    // }
    // fn slot_state(&self, idx: usize) -> Option<SlotState> {
    //     if idx < Self::slot_count() {
    //         use SlotState::*;
    //         let mask = 0b1u8 << idx;
    //         let trp_this_instance = (unsafe { (*self.regs()).txbrp.read().trp().bits() } & 0b111) as u8;
    //         if trp_this_instance & mask != 0 { // Used at least by this instance
    //             let mr = &MessageRam::get();
    //             let id = mr.tx_buffer[idx].t0.read().id().frame_id();
    //             let lowest_id = self.lowest_queued_id();
    //             if lowest_id.is_none() {
    //                 Some(Queued)
    //             } else if id == lowest_id.unwrap() {
    //                 Some(AccessInProgress)
    //             } else {
    //                 Some(Queued)
    //             }
    //         } else {
    //             unsafe {
    //                 for instance in self.aliased_instances().iter() {
    //                     if *instance != core::ptr::null() {
    //                         let trp_other = ((**instance).txbrp.read().trp().bits() & 0b111) as u8;
    //                         if mask & trp_other != 0 {
    //                             return Some(UsedByOtherInstance);
    //                         }
    //                     }
    //                 }
    //                 return Some(Empty);
    //             }
    //         }
    //     } else {
    //         None
    //     }
    // }

    fn put_frame(&mut self) -> Result<(), Error> {
        Ok(())
    }

    unsafe fn ll<F: FnMut(&mut RegisterBlock)>(&mut self, mut f: F) {
        f(self.regs_mut());
    }
}

impl ClassicalCan for ClassicalCanInstance {
    fn regs(&self) -> &RegisterBlock {
        unsafe { &*self.regs }
    }

    unsafe fn regs_mut(&mut self) -> &mut RegisterBlock {
        &mut *(self.regs as *const _ as *mut RegisterBlock)
    }

    fn free_slots(&self) -> usize {
        match self.txbuffer_mode {
            TxBufferMode::Queue => {
                // 1 slot per FDCAN instance for now, probably can be improved
                let trp = (unsafe { self.regs().txbrp.read().trp().bits() } & 0b111) as u8;
                if self.instance as u8 & trp != 0 {
                    0
                } else {
                    1
                }
            }
            TxBufferMode::Fifo => self.regs().txfqs.read().tffl().bits() as usize,
        }
    }

    // unsafe fn aliased_instances(&self) -> [*const RegisterBlock; 2] {
    //     [self.regs2, self.regs3]
    // }
}
/// Entry point for the CAN bus initialization
pub struct CanController {
    // Common to all FDCAN instances
    clock_source: ClockSource,
}
impl CanController {
    pub fn new(clock_source: ClockSource, rcc: &mut Rcc, dwt: &mut DWT) -> Result<Self, Error> {
        // Enable clock & reset
        rcc.rb.apb1enr1.modify(|_, w| w.fdcanen().set_bit());
        rcc.rb.apb1rstr1.modify(|_, w| w.fdcanrst().set_bit());
        rcc.rb.apb1rstr1.modify(|_, w| w.fdcanrst().clear_bit());
        // Select clock source
        rcc.rb
            .ccipr
            .modify(|_, w| unsafe { w.fdcansel().bits(clock_source.into()) }); // 0b00 - HSE, 0b01 - PLLQ, 0b10 - PCLK, 0b11 - RSVD
        let fdcan_clk = match clock_source {
            ClockSource::Hse(hse_freq) => hse_freq,
            ClockSource::Pllq => match rcc.clocks.pll_clk.unwrap().q {
                Some(pllq) => pllq,
                None => {
                    return Err((Error::IncorrectRccConfig));
                }
            },
            ClockSource::Pclk => rcc.clocks.apb1_clk,
        };
        // Enable DWT if not already
        dwt.enable_cycle_counter();

        Ok(CanController { clock_source })
    }
}

macro_rules! can {
    (
        $CANX:ident, $canX:ident, $CanInstanceNumber:expr,
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
        impl<PINS> CanInstance<$CANX, PINS> {
            pub fn new_classical(
                can_controller: &mut CanController,
                can: $CANX,
                pins: PINS,
                mode: Mode,
                retransmission: Retransmission,
                transmit_pause: TransmitPause,
                txbuffer_mode: TxBufferMode,
                clock_div: ClockDiv,
                bit_timing: BitTiming,
            ) -> Result<(ClassicalCanInstance, CanPinsToken<PINS>), (Error, $CANX, PINS)>
            where
                PINS: Pins<$CANX>,
            {
                // Configure pins
                pins.setup();
                // Check that reset succeeded
                checked_wait_or!(can.cccr.read().bits() != 0x0000_0001, (Error::ResetFail, can, pins));
                // Unlock write access
                can.cccr.modify(|_, w| w.cce().set_bit());
                checked_wait_or!(can.cccr.read().cce().bit_is_clear(), (Error::WriteUnlockFail, can, pins));
                // Apply config
                can.cccr.modify(|_, w| w
                    .mon().bit(mode.is_mon_mode())
                    .test().bit(mode.is_test_mode())
                    .dar().bit(retransmission == Retransmission::Disabled)
                    .txp().bit(transmit_pause == TransmitPause::Enabled)
                );
                can.test.modify(|_, w| w
                    .lbck().bit(mode.is_test_mode())
                );
                // Enable queue mode
                //can.txbc.modify(|_, w| w.tfqm().set_bit()); // Wrong PAL
                if txbuffer_mode == TxBufferMode::Queue {
                    can.txbc.write(|w| unsafe { w.bits(0b1 << 24) });
                }
                // Calculate prescaler & apply bit timings
                // sync_seg = 1tq
                // bs1 = prop_seg + phseg1 = 1..16tq
                // bs2 = phseg2 = 1..8tq
                // sjw = 1..4tq
                // bit time = tsyncseg + tbs1 + tbs2
                // tq = (nbtp.nbrp + 1) * tfdcan_tq_clk, nbrp = 0..511
                // fdcan_tq_clk = fdcan_clk / ckdiv.pdiv, pdiv = 1, 2, 4, 6 .. 30
                // tsync_seg = 1tq
                // tbs1 = tq * (nbtp.ntseg1 + 1), ntseg1 = 0..255
                // tbs2 = tq * (nbtp.ntseg2 + 1), ntseg2 = 0..127
                //
                // example from register layout section of the datasheet:
                // ntseg1 = 10, ntseg2 = 3, nbrp = 0, nsjw = 6 => bit rate = 3Mbit/s
                // fdcan_clk = 48MHz, tfdcan_tq_clkd = 0.02083(3)us
                // tq = 0.02083(3)us
                // tsync_seg = 1tq = 0.02083(3)us
                // tbs1 = 11 * tq = 0.22916(6)us
                // tbs2 = 4 * tq = 0.083(3)us
                // bit time = (1 + 11 + 4)tq = 0.3(3)us

                // Configure clock divider
                can.ckdiv.write(|w| unsafe { w.pdiv().bits(clock_div as u8) }); // 1, 2, 4, 6 .. 30
                // Configure timings
                can.nbtp.write(|w| unsafe { w
                    .nsjw().bits(bit_timing.nsjw)
                    .nbrp().bits(0)
                    .ntseg1().bits(bit_timing.ntseg1)
                    .tseg2().bits(bit_timing.ntseg2)
                });
                // Transition to normal state
                can.cccr.modify(|_, w| w.init().clear_bit());
                checked_wait_or!(can.cccr.read().init().bit_is_set(), (Error::InitFail, can, pins));

                // let (regs2, regs3) = match $CanInstanceNumber {
                //     CanInstanceNumber::Fdcan1 => (FDCAN2::ptr(), FDCAN3::ptr()),
                //     CanInstanceNumber::Fdcan2 => (FDCAN1::ptr(), FDCAN3::ptr()),
                //     CanInstanceNumber::Fdcan3 => (FDCAN1::ptr(), FDCAN2::ptr()),
                // };
                Ok((ClassicalCanInstance {
                    regs: $CANX::ptr(),
                    instance: $CanInstanceNumber,
                    txbuffer_mode
                }, CanPinsToken { pins }))
            }
        }

    };
}

can!(
    FDCAN1,
    fdcan1,
    CanInstanceNumber::Fdcan1,
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
