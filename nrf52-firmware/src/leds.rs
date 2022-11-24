#[allow(clippy::cast_possible_truncation)]
fn effect(led: &mut u8) {
    let mut data: u16 = u16::from(*led);
    data *= 14;
    data >>= 4;
    *led = data as u8;
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct GRB {
    pub g: u8,
    pub r: u8,
    pub b: u8,
}

impl Default for GRB {
    fn default() -> Self {
        Self { g: 0, r: 0, b: 0 }
    }
}

impl GRB {
    pub fn effect(&mut self) {
        effect(&mut self.g);
        effect(&mut self.r);
        effect(&mut self.b);
    }

    pub fn from_bytes(data: &[u8; 3]) -> Self {
        Self {
            g: data[0],
            r: data[1],
            b: data[2],
        }
    }
}

pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::core::slice::from_raw_parts((p as *const T) as *const u8, ::core::mem::size_of::<T>())
}
