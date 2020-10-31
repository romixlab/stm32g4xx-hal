#[doc = r"Value read from the register"]
pub struct R {
    bits: u32,
}
#[doc = r"Value to write to the register"]
pub struct W {
    bits: u32,
}
impl super::F1 {
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

pub struct EFTIR{
    bits: u8,
}
impl EFTIR{
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _EFTIW<'a> {
    w: &'a mut W,
}
impl<'a> _EFTIW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x03 << 29);
        self.w.bits |= ((value as u32) & 0x03) << 29;
        self.w
    }
}

pub struct EFID2R{
    bits: u32,
}
impl EFID2R{
    pub fn bits(&self) -> u32 {
        self.bits
    }
}

pub struct _EFID2W<'a> {
    w: &'a mut W,
}
impl<'a> _EFID2W<'a> {
    pub unsafe fn bits(self, value: u32) -> &'a mut W {
        self.w.bits &= !(0x1FFFFFFF << 0);
        self.w.bits |= ((value as u32) & 0x1FFFFFFF) << 0;
        self.w
    }
}


impl R {
    #[inline(always)]
    pub fn bits(&self) -> u32 {
        self.bits
    }
    #[inline(always)]
    pub fn efti(&self) -> EFTIR {
        let bits = ((self.bits >> 29) & 0x07) as u8;
        EFTIR { bits }
    }
    #[inline(always)]
    pub fn efid2(&self) -> EFID2R {
        let bits = ((self.bits >> 0) & 0x1FFFFFFF) as u32;
        EFID2R { bits }
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
    pub fn efti(&mut self) -> _EFTIW {
        _EFTIW { w: self }
    }
    #[inline(always)]
    pub fn efid2(&mut self) -> _EFID2W {
        _EFID2W { w: self }
    }

}