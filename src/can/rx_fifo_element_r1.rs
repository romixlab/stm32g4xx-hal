#[doc = r"Value read from the register"]
pub struct R {
    bits: u32,
}
#[doc = r"Value to write to the register"]
pub struct W {
    bits: u32,
}
impl super::R1 {
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

pub struct ANMFR{
    bits: bool,
}
impl ANMFR{
    #[inline(always)]
    pub fn bit(&self) -> bool {
        self.bits
    }
    #[inline(always)]
    pub fn bit_is_clear(&self) -> bool {
        !self.bit()
    }
    #[inline(always)]
    pub fn bit_is_set(&self) -> bool {
        self.bit()
    }
}

pub struct _ANMFW<'a> {
    w: &'a mut W,
}
impl<'a> _ANMFW<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits &= !(0x01 << 31);
        self.w.bits |= ((value as u32) & 0x01) << 31;
        self.w
    }
}

pub struct FIDXR{
    bits: u8,
}
impl FIDXR{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _FIDXW<'a> {
    w: &'a mut W,
}
impl<'a> _FIDXW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x7F << 24);
        self.w.bits |= ((value as u32) & 0x7F) << 24;
        self.w
    }
}
pub struct FDFR{
    bits: bool,
}
impl FDFR{
    #[inline(always)]
    pub fn bit(&self) -> bool {
        self.bits
    }
    #[inline(always)]
    pub fn bit_is_clear(&self) -> bool {
        !self.bit()
    }
    #[inline(always)]
    pub fn bit_is_set(&self) -> bool {
        self.bit()
    }
}

pub struct _FDFW<'a> {
    w: &'a mut W,
}
impl<'a> _FDFW<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits &= !(0x01 << 21);
        self.w.bits |= ((value as u32) & 0x01) << 21;
        self.w
    }
}

pub struct BRSR{
    bits: bool,
}
impl BRSR{
    #[inline(always)]
    pub fn bit(&self) -> bool {
        self.bits
    }
    #[inline(always)]
    pub fn bit_is_clear(&self) -> bool {
        !self.bit()
    }
    #[inline(always)]
    pub fn bit_is_set(&self) -> bool {
        self.bit()
    }
}

pub struct _BRSW<'a> {
    w: &'a mut W,
}
impl<'a> _BRSW<'a> {
    #[doc = r"Sets the field bit"]
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W {
        self.bit(true)
    }
    #[doc = r"Clears the field bit"]
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W {
        self.bit(false)
    }
    #[doc = r"Writes raw bits to the field"]
    #[inline(always)]
    pub fn bit(self, value: bool) -> &'a mut W {
        self.w.bits &= !(0x01 << 20);
        self.w.bits |= ((value as u32) & 0x01) << 20;
        self.w
    }
}


pub struct DLCR{
    bits: u8,
}
impl DLCR{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _DLCW<'a> {
    w: &'a mut W,
}
impl<'a> _DLCW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x0F << 16);
        self.w.bits |= ((value as u32) & 0x0F) << 16;
        self.w
    }
}

pub struct RXTSR{
    bits: u16,
}
impl RXTSR{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

pub struct _RXTSW<'a> {
    w: &'a mut W,
}
impl<'a> _RXTSW<'a> {
    pub unsafe fn bits(self, value: u16) -> &'a mut W {
        self.w.bits &= !(0xFFFF<< 0);
        self.w.bits |= ((value as u32) & 0xFFFF) << 0;
        self.w
    }
}

impl R {
    #[inline(always)]
    pub fn bits(&self) -> u32 {
        self.bits
    }
    #[inline(always)]
    pub fn anmf(&self) -> ANMFR {
        let bits = ((self.bits >> 31) & 0x01) != 0;
        ANMFR { bits }
    }
    #[inline(always)]
    pub fn fidx(&self) -> FIDXR {
        let bits = ((self.bits >> 24) & 0x7F) as u8;
        FIDXR { bits }
    }
    #[inline(always)]
    pub fn fdf(&self) -> FDFR {
        let bits = ((self.bits >> 21) & 0x01) != 0;
        FDFR { bits }
    }

    #[inline(always)]
    pub fn brs(&self) -> BRSR {
        let bits = ((self.bits >> 20) & 0x01) != 0;
        BRSR { bits }
    }

    #[inline(always)]
    pub fn dlc(&self) -> DLCR {
        let bits = ((self.bits >> 16) & 0x0F) as u8;
        DLCR { bits }
    }

    #[inline(always)]
    pub fn rxts(&self) -> RXTSR {
        let bits = ((self.bits >> 0) & 0xFFFF) as u16;
        RXTSR { bits }
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
    pub fn anmf(&mut self) -> _ANMFW {
        _ANMFW { w: self }
    }
    #[inline(always)]
    pub fn fidx(&mut self) -> _FIDXW {
        _FIDXW { w: self }
    }
    #[inline(always)]
    pub fn fdf(&mut self) -> _FDFW {
        _FDFW { w: self }
    }
    #[inline(always)]
    pub fn brs(&mut self) -> _BRSW {
        _BRSW { w: self }
    }
    #[inline(always)]
    pub fn dlc(&mut self) -> _DLCW {
        _DLCW { w: self }
    }
    #[inline(always)]
    pub fn rxts(&mut self) -> _RXTSW {
        _RXTSW { w: self }
    }
}