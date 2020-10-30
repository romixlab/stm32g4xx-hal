#[doc = r"Value read from the register"]
pub struct R {
    bits: u32,
}
#[doc = r"Value to write to the register"]
pub struct W {
    bits: u32,
}
impl super::S0 {
    #[doc = r"Modifies the contents of the register"]
    #[inline(always)]
    pub fn modify<F>(&self, f: F)
        where
                for<'w> F: FnOnce(&R, &'w mut W) -> &'w mut W,
    {
        let bits = self.register.get();
        self.register.set(f(&R { bits }, &mut W { bits }).bits);
    }
    #[doc = r"Reads the contents of the register"]
    #[inline(always)]
    pub fn read(&self) -> R {
        R {
            bits: self.register.get(),
        }
    }
    #[doc = r"Writes to the register"]
    #[inline(always)]
    pub fn write<F>(&self, f: F)
        where
            F: FnOnce(&mut W) -> &mut W,
    {
        self.register.set(
            f(&mut W {
                bits: Self::reset_value(),
            })
                .bits,
        );
    }
    #[doc = r"Reset value of the register"]
    #[inline(always)]
    pub const fn reset_value() -> u32 {
        0xffff_0000
    }
    #[doc = r"Writes the reset value to the register"]
    #[inline(always)]
    pub fn reset(&self) {
        self.register.set(Self::reset_value())
    }
}

pub struct SFTR{
    bits: u8,
}
impl SFTR{
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _SFTW<'a> {
    w: &'a mut W,
}
impl<'a> _SFTW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x03 << 30);
        self.w.bits |= ((value as u32) & 0x03) << 30;
        self.w
    }
}

pub struct SFECR{
    bits: u8,
}
impl SFECR{
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _SFECW<'a> {
    w: &'a mut W,
}
impl<'a> _SFECW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x07 << 27);
        self.w.bits |= ((value as u32) & 0x07) << 27;
        self.w
    }
}

pub struct SFID1R{
    bits: u16,
}
impl SFID1R{
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

pub struct _SFID1W<'a> {
    w: &'a mut W,
}
impl<'a> _SFID1W<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x03_FF << 16);
        self.w.bits |= ((value as u32) & 0x03_FF) << 16;
        self.w
    }
}

pub struct SFID2R{
    bits: u16,
}
impl SFID2R{
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

pub struct _SFID2W<'a> {
    w: &'a mut W,
}
impl<'a> _SFID2W<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x03_FF << 0);
        self.w.bits |= ((value as u32) & 0x03_FF) << 0;
        self.w
    }
}
impl R {
    #[inline(always)]
    pub fn bits(&self) -> u32 {
        self.bits
    }
    #[inline(always)]
    pub fn sft(&self) -> SFTR {
        let bits = ((self.bits >> 30) & 0x03) as u8;
        SFTR { bits }
    }
    #[inline(always)]
    pub fn sfec(&self) -> SFECR {
        let bits = ((self.bits >> 27) & 0x07) as u8;
        SFECR { bits }
    }
    #[inline(always)]
    pub fn sfid1(&self) -> SFID1R {
        let bits = ((self.bits >> 16) & 0x03_FF) as u16;
        SFID1R { bits }
    }

    #[inline(always)]
    pub fn sfid2(&self) -> SFID2R {
        let bits = ((self.bits >> 0) & 0x03_FF) as u16;
        SFID2R { bits }
    }
}
impl W {
    #[doc = r"Writes raw bits to the register"]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.bits = bits;
        self
    }
    #[inline(always)]
    pub fn sft(&mut self) -> _SFTW {
        _SFTW { w: self }
    }
    #[inline(always)]
    pub fn sfec(&mut self) -> _SFECW {
        _SFECW { w: self }
    }
    #[inline(always)]
    pub fn sfid1(&mut self) -> _SFID1W {
        _SFID1W { w: self }
    }
    pub fn sfid2(&mut self) -> _SFID2W {
        _SFID2W { w: self }
    }
}