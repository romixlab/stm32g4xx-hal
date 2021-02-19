use crate::pac::{FLASH, PWR, RCC};
use crate::time::{Hertz, U32Ext};

mod clockout;
mod config;

pub use clockout::*;
pub use config::*;

/// HSI speeds
pub const HSI_FREQ: u32 = 16_000_000;
pub const HSI48_FREQ: u32 = 48_000_000;

/// Clock frequencies
#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    /// System frequency
    pub sys_clk: Hertz,
    /// Core frequency
    pub core_clk: Hertz,
    /// AHB frequency
    pub ahb_clk: Hertz,
    /// APB1 frequency
    pub apb1_clk: Hertz,
    /// APB2 frequency
    pub apb2_clk: Hertz,
    /// APB timers frequency
    pub apb_tim_clk: Hertz,
    /// PLL frequency
    pub pll_clk: Option<PLLClocks>,
}

impl Clocks {
    pub fn sysclk(&self) -> Hertz {
        self.sys_clk
    }
}

/// PLL Clock frequencies
#[derive(Clone, Copy, Debug)]
pub struct PLLClocks {
    /// R frequency
    pub r: Hertz,
    /// Q frequency
    pub q: Option<Hertz>,
    /// P frequency
    pub p: Option<Hertz>,
}

impl Default for Clocks {
    fn default() -> Clocks {
        Clocks {
            sys_clk: 160.mhz(),
            ahb_clk: 160.mhz(),
            core_clk: 160.mhz(),
            apb1_clk: 160.mhz(),
            apb2_clk: 160.mhz(),
            apb_tim_clk: 160.mhz(),
            pll_clk: Some(PLLClocks {
                r: 160.mhz(),
                q: Some(80.mhz()),
                p: Some(160.mhz()),
            }),
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// Clock configuration
    pub clocks: Clocks,
    pub(crate) rb: RCC,
}

impl Rcc {
    /// Apply clock configuration
    pub fn freeze(self, rcc_cfg: Config) -> Self {
        let (sys_clk, sw_bits, pll_clk) = match rcc_cfg.sys_mux {
            SysClockSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq, 0b10, None)
            }
            SysClockSrc::HSE_BYPASS(freq) => {
                self.enable_hse(true);
                (freq, 0b10, None)
            }
            SysClockSrc::PLL(pll_cfg) => {
                let pll_clk = self.config_pll(pll_cfg);
                (pll_clk.r, 0b11, Some(pll_clk))
            },
            SysClockSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ.hz(), 0b01, None)
            }
        };

        let sys_freq = sys_clk.0;
        let (ahb_freq, ahb_psc_bits) = match rcc_cfg.ahb_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b1000),
            Prescaler::Div4 => (sys_freq / 4, 0b1001),
            Prescaler::Div8 => (sys_freq / 8, 0b1010),
            Prescaler::Div16 => (sys_freq / 16, 0b1011),
            Prescaler::Div64 => (sys_freq / 64, 0b1100),
            Prescaler::Div128 => (sys_freq / 128, 0b1101),
            Prescaler::Div256 => (sys_freq / 256, 0b1110),
            Prescaler::Div512 => (sys_freq / 512, 0b1111),
            _ => (sys_clk.0, 0b0000),
        };

        let (apb1_freq, apb1_psc_bits) = match rcc_cfg.apb1_psc {
            Prescaler::Div2 => (sys_freq / 2, 0b100),
            Prescaler::Div4 => (sys_freq / 4, 0b101),
            Prescaler::Div8 => (sys_freq / 8, 0b110),
            Prescaler::Div16 => (sys_freq / 16, 0b111),
            _ => (sys_clk.0, 0b000),
        };

        let (apb2_freq, apb_tim_freq, apb2_psc_bits) = match rcc_cfg.apb2_psc {
            Prescaler::Div2 => (sys_freq / 2, sys_freq, 0b100),
            Prescaler::Div4 => (sys_freq / 4, sys_freq / 2, 0b101),
            Prescaler::Div8 => (sys_freq / 8, sys_freq / 4, 0b110),
            Prescaler::Div16 => (sys_freq / 16, sys_freq / 8, 0b111),
            _ => (sys_clk.0, sys_clk.0, 0b000),
        };

        // TODO - move this to a Flash module.
        unsafe {
            // Adjust flash wait states
            let flash = &(*FLASH::ptr());
            flash.acr.modify(|_, w| {
                w.latency().bits(if sys_clk.0 <= 20_000_000 {
                    0b0000
                } else if sys_clk.0 <= 40_000_000 {
                    0b0001
                } else if sys_clk.0 <= 60_000_000 {
                    0b0010
                } else if sys_clk.0 <= 80_000_000 {
                    0b0011
                } else if sys_clk.0 <= 100_000_000 {
                    0b0100
                } else if sys_clk.0 <= 120_000_000 {
                    0b0101
                } else if sys_clk.0 <= 140_000_000 {
                    0b0110
                } else if sys_clk.0 <= 160_000_000 {
                    0b0111
                } else if sys_clk.0 <= 170_000_000 {
                    0b1000
                } else {
                    0b1000
                })
            })
        }

        self.rb.cfgr.modify(|_, w| unsafe {
            w.hpre()
                .bits(ahb_psc_bits)
                .ppre1()
                .bits(apb1_psc_bits)
                .ppre2()
                .bits(apb2_psc_bits)
                .sw()
                .bits(sw_bits)
        });

        while self.rb.cfgr.read().sws().bits() != sw_bits {}

        Rcc {
            rb: self.rb,
            clocks: Clocks {
                pll_clk,
                sys_clk,
                core_clk: ahb_freq.hz(),
                ahb_clk: ahb_freq.hz(),
                apb1_clk: apb1_freq.hz(),
                apb2_clk: apb2_freq.hz(),
                apb_tim_clk: apb_tim_freq.hz(),
            },
        }
    }

    fn config_pll(&self, pll_cfg: PllConfig) -> PLLClocks {
        assert!(pll_cfg.m > 0 && pll_cfg.m <= 16);
        assert!(pll_cfg.n > 7 && pll_cfg.n <= 127);

        // Disable PLL
        self.rb.cr.write(|w| w.pllon().clear_bit());
        while self.rb.cr.read().pllrdy().bit_is_set() {}

        let (freq, pll_sw_bits) = match pll_cfg.mux {
            PLLSrc::HSI => {
                self.enable_hsi();
                (HSI_FREQ, 0b10)
            }
            PLLSrc::HSE(freq) => {
                self.enable_hse(false);
                (freq.0, 0b11)
            }
            PLLSrc::HSE_BYPASS(freq) => {
                self.enable_hse(true);
                (freq.0, 0b11)
            }
        };

        let pll_freq = freq / (pll_cfg.m as u32) * (pll_cfg.n as u32);

        let (r, pllr_bits) = match pll_cfg.r {
            Some(PLLQRDiv::Div2) => (pll_freq / 2, 0b00u8),
            Some(PLLQRDiv::Div4) => (pll_freq / 4, 0b01u8),
            Some(PLLQRDiv::Div6) => (pll_freq / 6, 0b10u8),
            Some(PLLQRDiv::Div8) => (pll_freq / 8, 0b11u8),
            _ => (pll_freq / 2, 0b00u8),
        };

        let q = match pll_cfg.q {
            Some(div) => {
                let (q, pllq_bits) = match div {
                    PLLQRDiv::Div2 => (pll_freq / 2, 0b00u8),
                    PLLQRDiv::Div4 => (pll_freq / 4, 0b01u8),
                    PLLQRDiv::Div6 => (pll_freq / 6, 0b10u8),
                    PLLQRDiv::Div8 => (pll_freq / 8, 0b11u8),
                };
                self.rb
                    .pllcfgr
                    .modify(|_, w| unsafe { w.pllq().bits(pllq_bits).pllqen().set_bit() });
                Some(q.hz())
            }
            _ => None,
        };

        let p = match pll_cfg.p {
            Some(div) => {
                let div: u8 = div as u8;

                self.rb
                    .pllcfgr
                    .modify(move |_, w| unsafe { w.pllpdiv().bits(div).pllpen().set_bit() });
                let req = pll_freq / div as u32;
                Some(req.hz())
            }
            _ => None,
        };

        self.rb.pllcfgr.modify(move |_, w| unsafe {
            w.pllsrc()
                .bits(pll_sw_bits)
                .pllm()
                .bits(pll_cfg.m - 1)
                .plln()
                .bits(pll_cfg.n)
                .pllr()
                .bits(pllr_bits)
                .pllren()
                .set_bit()
        });

        // Enable PLL
        self.rb.cr.modify(|_, w| w.pllon().set_bit());
        while self.rb.cr.read().pllrdy().bit_is_clear() {}

        PLLClocks { r: r.hz(), q, p }
    }

    pub(crate) fn enable_hsi(&self) {
        self.rb.cr.write(|w| w.hsion().set_bit());
        while self.rb.cr.read().hsirdy().bit_is_clear() {}
    }

    pub(crate) fn enable_hsi48(&self) {
        self.rb.crrcr.write(|w| w.hsi48on().set_bit());
        while self.rb.crrcr.read().hsi48rdy().bit_is_clear() {}
    }

    pub(crate) fn enable_hse(&self, bypass: bool) {
        self.rb
            .cr
            .write(|w| w.hseon().set_bit().hsebyp().bit(bypass));
        while self.rb.cr.read().hserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lse(&self, bypass: bool) {
        self.rb
            .bdcr
            .write(|w| w.lseon().set_bit().lsebyp().bit(bypass));
        while self.rb.bdcr.read().lserdy().bit_is_clear() {}
    }

    pub(crate) fn enable_lsi(&self) {
        self.rb.csr.write(|w| w.lsion().set_bit());
        while self.rb.csr.read().lsirdy().bit_is_clear() {}
    }

    pub(crate) fn unlock_rtc(&self) {
        self.rb.apb1enr1.modify(|_, w| w.pwren().set_bit());
        let pwr = unsafe { &(*PWR::ptr()) };
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
    }
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
    /// Constrains the `RCC` peripheral and apply clock configuration
    fn freeze(self, rcc_cfg: Config) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            rb: self,
            clocks: Clocks::default(),
        }
    }

    fn freeze(self, rcc_cfg: Config) -> Rcc {
        self.constrain().freeze(rcc_cfg)
    }
}
