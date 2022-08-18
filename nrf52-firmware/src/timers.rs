#![deny(clippy::pedantic)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::inline_always)]

use super::hal::pac::{timer0, TIMER0, TIMER1, TIMER2};

pub struct StopWatch<T: Instance32>(T);

impl<T: Instance32> StopWatch<T> {
    pub fn new(regs: T, top: u32) -> Self {
        // NOTE(unsafe) 1 is a valid pattern to write to this register
        regs.tasks_stop.write(|w| unsafe { w.bits(1) });

        regs.bitmode.write(|w| w.bitmode()._32bit());

        // 16 Mhz / 2**4 = 1 Mhz = µs resolution
        // NOTE(unsafe) 4 is a valid pattern to write to this register
        regs.prescaler.write(|w| unsafe { w.prescaler().bits(4) });
        // NOTE(unsafe) 1 is a valid pattern to write to this register
        regs.tasks_clear.write(|w| unsafe { w.bits(1) });

        // Clear timer if CC0 is hit.
        regs.shorts.write(|w| w.compare0_clear().set_bit());

        regs.cc[0].write(|w| unsafe { w.bits(top) });

        let mut tmr = Self(regs);

        // Enable interrupt for CC0
        tmr.enable_interrupt(0);

        tmr
    }

    #[inline(always)]
    pub fn start(&mut self) {
        // NOTE(unsafe) 1 is a valid pattern to write to this register
        self.0.tasks_start.write(|w| unsafe { w.bits(1) });
    }

    #[inline(always)]
    pub fn now(&self) -> u32 {
        // NOTE(unsafe) 1 is a valid pattern to write to this register
        self.0.tasks_capture[0].write(|w| unsafe { w.bits(1) });
        self.0.cc[0].read().bits()
    }
    #[inline(always)]
    pub fn read_event(&mut self, cc: u32) -> bool {
        if cc < 5 {
            return self.0.events_compare[cc as usize].read().bits() & 1 != 0;
        }
        false
    }

    #[inline(always)]
    pub fn reset_event(&mut self, cc: u32) {
        if cc < 5 {
            // Write zero to clear the event!
            self.0.events_compare[cc as usize].write(|w| unsafe { w.bits(0) });
        }
    }

    #[inline(always)]
    pub fn enable_interrupt(&mut self, cc: usize) {
        match cc {
            0 => self.0.intenset.write(|w| w.compare0().set_bit()),
            1 => self.0.intenset.write(|w| w.compare1().set_bit()),
            2 => self.0.intenset.write(|w| w.compare2().set_bit()),
            3 => self.0.intenset.write(|w| w.compare3().set_bit()),
            // 4 => self.0.intenset.write(|w| w.compare4().set_bit()),
            // 5 => self.0.intenset.write(|w| w.compare5().set_bit()),
            _ => (),
        }
    }
    #[inline(always)]
    pub fn set_event(&mut self, cc: usize, val: u32) {
        if cc < 5 {
            // Write zero to clear the event!
            self.0.tasks_capture[cc].write(|w| unsafe { w.bits(1) });

            // Reprogram CC value
            let cc_val = self.0.cc[cc].read().bits();
            let cc_val = cc_val.wrapping_add(val) % 1_000_000;
            self.0.cc[cc as usize].write(|w| unsafe { w.bits(cc_val) });

            // enable interrupt
            self.enable_interrupt(cc);
        }
    }

    // #[inline(always)]
    // pub fn enable_event(&mut self, cc: usize) {
    //     if cc < 5 {
    //         // Write zero to clear the event!
    //         self.0.events_compare[cc].write(|w| unsafe { w.bits(0) });

    //         // read counter
    //         self.0.events_compare[cc].write(|w| unsafe { w.bits(0) });
    //     }
    // }

    #[inline(always)]
    pub fn stop(&mut self) {
        // NOTE(unsafe) 1 is a valid pattern to write to this register
        self.0.tasks_stop.write(|w| unsafe { w.bits(1) });

        // NOTE(unsafe) 1 is a valid pattern to write to this register
        self.0.tasks_clear.write(|w| unsafe { w.bits(1) });
    }
}

pub trait Instance32: core::ops::Deref<Target = timer0::RegisterBlock> {}
impl Instance32 for TIMER0 {}
impl Instance32 for TIMER1 {}
impl Instance32 for TIMER2 {}