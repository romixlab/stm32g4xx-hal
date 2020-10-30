#[doc = r"Value read from the register"]
pub struct R {
    bits: u32,
}
#[doc = r"Value to write to the register"]
pub struct W {
    bits: u32,
}
impl super::F0 {
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

pub struct EFECR{
    bits: u8,
}
impl EFECR{
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _EFECW<'a> {
    w: &'a mut W,
}
impl<'a> _EFECW<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0x07 << 29);
        self.w.bits |= ((value as u32) & 0x07) << 29;
        self.w
    }
}

pub struct EFID1R{
    bits: u32,
}
impl EFID1R{
    pub fn bits(&self) -> u32 {
        self.bits
    }
}

pub struct _EFID1W<'a> {
    w: &'a mut W,
}
impl<'a> _EFID1W<'a> {
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
    pub fn efec(&self) -> EFECR {
        let bits = ((self.bits >> 29) & 0x07) as u8;
        EFECR { bits }
    }
    #[inline(always)]
    pub fn efid1(&self) -> EFID1R {
        let bits = ((self.bits >> 0) & 0x1FFFFFFF) as u32;
        EFID1R { bits }
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
    pub fn efec(&mut self) -> _EFECW {
        _EFECW { w: self }
    }
    #[inline(always)]
    pub fn efid1(&mut self) -> _EFID1W {
        _EFID1W { w: self }
    }
    
}