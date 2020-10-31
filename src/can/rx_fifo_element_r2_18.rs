#[doc = r"Value read from the register"]
pub struct R {
    bits: u32,
}
#[doc = r"Value to write to the register"]
pub struct W {
    bits: u32,
}
impl super::R2_18 {
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

pub struct DB0R{
    bits: u8,
}
impl DB0R{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _DB0W<'a> {
    w: &'a mut W,
}
impl<'a> _DB0W<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0xFF << 0);
        self.w.bits |= ((value as u32) & 0xFF) << 0;
        self.w
    }
}

pub struct DB1R{
    bits: u8,
}

impl DB1R{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u8 {
        self.bits
    }
}

pub struct _DB1W<'a> {
    w: &'a mut W,
}
impl<'a> _DB1W<'a> {
    pub unsafe fn bits(self, value: u8) -> &'a mut W {
        self.w.bits &= !(0xFF << 8);
        self.w.bits |= ((value as u32) & 0xFF) << 8;
        self.w
    }
}

pub struct DB2R{
    bits: u16,
}
impl DB2R{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

pub struct _DB2W<'a> {
    w: &'a mut W,
}
impl<'a> _DB2W<'a> {
    pub unsafe fn bits(self, value: u16) -> &'a mut W {
        self.w.bits &= !(0xFF_FF << 16);
        self.w.bits |= ((value as u32) & 0xFF_FF) << 16;
        self.w
    }
}

pub struct DB3R{
    bits: u16,
}

impl DB3R{
    #[doc = r"Value of the field as raw bits"]
    #[inline(always)]
    pub fn bits(&self) -> u16 {
        self.bits
    }
}

pub struct _DB3W<'a> {
    w: &'a mut W,
}
impl<'a> _DB3W<'a> {
    pub unsafe fn bits(self, value: u16) -> &'a mut W {
        self.w.bits &= !(0xFF_FF << 24);
        self.w.bits |= ((value as u32) & 0xFF_FF) << 24;
        self.w
    }
}

impl R {
    #[inline(always)]
    pub fn bits(&self) -> u32 {
        self.bits
    }
    #[inline(always)]
    pub fn db0(&self) -> DB0R {
        let bits = ((self.bits >> 0) & 0xFF) as u8;
        DB0R { bits }
    }
    #[inline(always)]
    pub fn db1(&self) -> DB1R {
        let bits = ((self.bits >> 8) & 0xFF) as u8;
        DB1R { bits }
    }
    #[inline(always)]
    pub fn db2(&self) -> DB2R {
        let bits = ((self.bits >> 16) & 0xFF_FF) as u16;
        DB2R { bits }
    }
    #[inline(always)]
    pub fn db3(&self) -> DB3R {
        let bits = ((self.bits >> 24) & 0xFF_FF) as u16;
        DB3R { bits }
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
    pub fn db0(&mut self) -> _DB0W {
        _DB0W { w: self }
    }
    #[inline(always)]
    pub fn db1(&mut self) -> _DB1W {
        _DB1W { w: self }
    }
    #[inline(always)]
    pub fn db2(&mut self) -> _DB2W {
        _DB2W { w: self }
    }
    #[inline(always)]
    pub fn db3(&mut self) -> _DB3W {
        _DB3W { w: self }
    }
}