#![no_main]
#![no_std]

mod timers;
use timers::StopWatch;

use defmt_rtt as _; // global logger
use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

use embedded_hal::digital::v2::OutputPin;

use nrf52840_hal as hal;

mod ieee802154;

use ieee802154::{Channel, Packet, Radio, RadioReturn};

use hal::{
    clocks::{ExternalOscillator, Internal, LfOscStopped},
    gpio::{Level, Output, Pin, PushPull},
    pac::PWM0,
    //pac::PWM0, TIMER1,
    pwm::{self, LoadMode, Prescaler, Pwm, PwmEvent, PwmSeq, Seq},
    usbd::{UsbPeripheral, Usbd},
    // Timer,
};

use heapless::{
    pool,
    pool::singleton::{Box, Pool},
    spsc::{Consumer, Producer, Queue},
    Deque,
};

// use ringbuffer::{self, RingBufferExt, RingBufferRead, RingBufferWrite};
use usb_device::{
    self,
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};

use usbd_serial::{self, DefaultBufferStore, SerialPort, USB_CLASS_CDC};

use uart_protocol::{self, Programs, Commands, Responce, N_BYTES, N_LEDS, TOTAL_BYTES};

pool!(FRAMEBUFFER: Deque<u8, TOTAL_BYTES>);

// Number of leds programmed in a single run
const PWM_N_LEDS: usize = 4;

const PWM_POL: u16 = 0x8000;

const PWM_T0H: u16 = 6 | PWM_POL;
const PWM_T1H: u16 = 13 | PWM_POL;
const PWM_MAX: u16 = 20;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Led {
    g: u8,
    r: u8,
    b: u8,
}

impl Led {
    fn pwm_next(&mut self, buf: &mut [u16]) {
        let mut n = 0;

        for _ in 0..PWM_N_LEDS {
            for mut d in [self.g, self.r, self.b] {
                for _ in 0..8 {
                    buf[n] = if d & 0x80 != 0 { PWM_T1H } else { PWM_T0H };
                    d <<= 1;
                    n += 1;
                }
            }
        }

        buf[PWM_DMA_MEM_SIZE-1] = PWM_POL;
    }
}

// impl LedDataBuffer {
//     pub fn push_led(&mut self, a: Led) {
//         self.push_back(a.g);
//         self.push_back(a.r);
//         self.push_back(a.b);
//     }
// }

pub struct Frame {
    pub inner: Box<FRAMEBUFFER>,
}

impl<'a> Frame {
    #[must_use]
    pub fn new() -> Option<Self> {
        if let Some(buf) = FRAMEBUFFER::alloc() {
            let buf = buf.init(Deque::new());
            Some(Frame { inner: buf })
        } else {
            None
        }
    }

    pub fn copy_from_slice(&mut self, data: &[u8]) -> bool {
        if data.len() <= (self.inner.capacity() - self.inner.len()) {
            for d in data {
                // SAFETY, we checked capacity.
                unsafe {
                    self.inner.push_back_unchecked(*d);
                };
            }
            true
        } else {
            false
        }
    }

    pub fn push_led(&mut self, a: Led) {
        if self.inner.len() + 3 >= TOTAL_BYTES {
            unsafe {
                // SAFETY: We check the size before we procede.
                self.inner.push_back_unchecked(a.g);
                self.inner.push_back_unchecked(a.r);
                self.inner.push_back_unchecked(a.b);
            }
        }
    }

    // pub fn push_byte(&mut self, data: u8) ->  {
    //     self.inner.push_back(data)
    // }

    // pub fn pop(&mut self) -> Option<&[u8]> {
    //     if self.buf.len() != self.tail {
    //         let ptr = self.buf[self.tail..self.tail + N_BYTES].as_ref();
    //         self.tail += N_BYTES;
    //         return Some(ptr);
    //     }
    //     None
    // }

    pub fn pwm_next(&mut self, buf: &mut [u16]) -> bool {
        let mut n = 0;
        let mut d = 0;
     
        for (num, data) in buf[0..(PWM_N_LEDS * N_BYTES * 8)].iter_mut().enumerate() {
            if num % 8 == 0 {
                let val = self.inner.pop_front();
                if val.is_none() {
                    break;
                }
                d = val.unwrap();
                n += 8;
            }
                
            *data = if d & 0x80 != 0 { PWM_T1H } else { PWM_T0H };
            d <<= 1;
        }

        // Allways fill remaining values with empty data
        for d in buf[n..].iter_mut() {
            *d = PWM_POL;
        }

        n != 0
    }
}

// This trivial implementation of `drop` adds a print to console.
// impl Drop for Frame {
//     fn drop(&mut self) {
//     }
// }

pub enum RadioAction {
    Disabled,
    RxMode,
    Tx(Packet),
}

// #[derive(PartialEq, Eq)]
pub enum PwmData {
    SINGLE { repeat: u16, data: Led },
    RAW(Frame),
    EMPTY,
    SetProgram(Programs),
}

impl Default for PwmData {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl PartialEq for PwmData {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (
                Self::SINGLE {
                    repeat: l_repeat,
                    data: l_data,
                },
                Self::SINGLE {
                    repeat: r_repeat,
                    data: r_data,
                },
            ) => l_repeat == r_repeat && l_data == r_data,
            (Self::RAW(_l0), Self::RAW(_r0)) => true,
            (Self::SetProgram(l0), Self::SetProgram(r0)) => l0 == r0,
            _ => core::mem::discriminant(self) == core::mem::discriminant(other),
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum State {
    R_UP,
    R_DOWN,
    G_UP,
    G_DOWN,
    B_UP,
    B_DOWN,
}

const LEDMAX: u8 = 0x80;

const G: usize = 0;
const R: usize = 1;
const B: usize = 2;
// const W: usize = 3;

type UsbBus = Usbd<UsbPeripheral<'static>>;

// PWM (DMA) Sequence memory
const PWM_DMA_MEM_SIZE: usize = PWM_N_LEDS * N_BYTES * 8 + 1;
type SeqBuffer = &'static mut [u16; PWM_DMA_MEM_SIZE];

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        // packet_buf: ringbuffer::ConstGenericRingBuffer<Packet, 4>,
        #[lock_free]
        pwm: Option<PwmSeq<PWM0, SeqBuffer, SeqBuffer>>,
        // tmr1: Timer<TIMER1>,
        #[lock_free]
        radio: Option<Radio<'static>>,
        program: uart_protocol::Programs,
    }

    #[local]
    struct Local {
        usb_device: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus, DefaultBufferStore, DefaultBufferStore>,
        led_buf_producer: Producer<'static, PwmData, 4>,
        led_buf_consumer: Consumer<'static, PwmData, 4>,
        ser_buf_producer: Producer<'static, PwmData, 4>,
        ser_buf_consumer: Consumer<'static, PwmData, 4>,
        // rxrf_buf_producer: Producer<'static, Packet, 4>,
        rxrf_buf_consumer: Consumer<'static, RadioReturn, 4>,
        tmr0: StopWatch,
        led_gr: Pin<Output<PushPull>>,
        // `_led_bl: Pin<Output<PushPull>>,
        debug_pin: Pin<Output<PushPull>>,
        update_pin: Pin<Output<PushPull>>,
    }

    #[init(local = [memory: [u8; 4096] = [0; 4096], rxrf_buf: Queue<Packet, 4> = Queue::new(), led_buf: Queue<PwmData, 4> = Queue::new(), ser_buf: Queue<PwmData, 4> = Queue::new(), radio_buf: Queue<ieee802154::RadioReturn, 4> = Queue::new(), usbbus: Option<UsbBusAllocator<UsbBus>> = None, clock: Option<hal::clocks::Clocks<ExternalOscillator, Internal, LfOscStopped>> =
    None, pwm0_buffer: [u16; PWM_DMA_MEM_SIZE] = [0; PWM_DMA_MEM_SIZE], packet_buf: Packet = Packet::new()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let pos = FRAMEBUFFER::grow(ctx.local.memory);

        assert_eq!(pos, 4);

        *ctx.local.clock = Some(hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc());

        let clock = &*ctx.local.clock.as_ref().unwrap();

        defmt::println!("test");

        defmt::info!("init");
        defmt::info!(
            "PwmData size: {} pos = {}",
            core::mem::size_of::<PwmData>(),
            pos
        );

        // while ctx.device.CLOCK.events_hfclkstarted.read().bits() != 1 {}
        // while !ctx
        //     .device
        //     .POWER
        //     .usbregstatus
        //     .read()
        //     .vbusdetect()
        //     .is_vbus_present()
        // {}

        // unsafe {
        //     CLOCK = Some(clock);
        // }

        // let clock = unsafe { CLOCK.as_ref().unwrap() };

        let port0 = hal::gpio::p0::Parts::new(ctx.device.P0);

        let mut red_led = port0.p0_23.into_push_pull_output(Level::Low).degrade();
        let mut blue_led = port0.p0_24.into_push_pull_output(Level::Low).degrade();
        let mut green_led = port0.p0_22.into_push_pull_output(Level::High).degrade();

        let pin_debug = port0.p0_06.into_push_pull_output(Level::Low).degrade();
        let pin_update = port0.p0_07.into_push_pull_output(Level::Low).degrade();

        // let tmr1 = Timer::new(ctx.device.TIMER1);

        *ctx.local.usbbus = Some(Usbd::new(UsbPeripheral::new(ctx.device.USBD, clock)));

        let usb_bus = ctx.local.usbbus.as_ref().unwrap();

        let usb_serial = SerialPort::new(usb_bus);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();


        // INIT RADIO
        let (radio_tx, radio_rx) = ctx.local.radio_buf.split();
        let mut radio = Radio::init(ctx.device.RADIO, &clock, radio_tx, ctx.local.packet_buf);
        radio.set_channel(Channel::_15);

        radio_handler::spawn( RadioAction::RxMode );

        green_led.set_low().unwrap();
        red_led.set_high().unwrap();
        blue_led.set_high().unwrap();

        let pwm = Pwm::new(ctx.device.PWM0);

        pwm.set_output_pin(
            pwm::Channel::C0,
            port0.p0_05.into_push_pull_output(Level::Low).degrade(),
        )
            .set_max_duty(PWM_MAX)
            .set_prescaler(Prescaler::Div1);


        let (mut led_buf_producer, led_buf_consumer) = ctx.local.led_buf.split();

        // Set all leds to blue;
        led_buf_producer
            .enqueue(PwmData::SINGLE {
                data: Led {
                    g: 0x00,
                    r: 0x00,
                    b: 0x04,
                },
                repeat: N_LEDS as u16,
            })
            .ok();

        pwm.set_load_mode(LoadMode::Common);
        pwm.one_shot();
        pwm.enable_interrupt(PwmEvent::SeqEnd(Seq::Seq0));

        let pwm = pwm
            .load::<SeqBuffer, SeqBuffer>(Some(ctx.local.pwm0_buffer), None, false)
            .ok();

        update_pwm::spawn(true).unwrap();

        // FPS Timer 120Hz, needed for USB pool()!
        let mut tmr0 = StopWatch::new(ctx.device.TIMER0, 8333);
        tmr0.start();

        let (ser_buf_producer, ser_buf_consumer) = ctx.local.ser_buf.split();
        // let (rxrf_buf_producer, rxrf_buf_consumer) = ctx.local.rxrf_buf.split();

        (
            Shared {
                // packet_buf: PACKET_BUF,
                pwm,
                // tmr1,
                radio: Some(radio),
                program: uart_protocol::Programs::Two,
            },
            Local {
                led_buf_consumer,
                led_buf_producer,
                ser_buf_consumer,
                ser_buf_producer,
                // rxrf_buf_producer,
                rxrf_buf_consumer: radio_rx,
                usb_device,
                usb_serial,
                led_gr: green_led,
                tmr0,
                debug_pin: pin_debug,
                update_pin: pin_update,
            },
            init::Monotonics(),
        )
    }

    #[task(priority = 3, shared = [pwm], local = [led_buf_consumer, state: PwmData = PwmData::EMPTY, debug_pin])]
    fn update_pwm(ctx: update_pwm::Context, user_start: bool) {
        let STATE = ctx.local.state;

        // Only update state if EMPTY on a interrupt
        if user_start && *STATE != PwmData::EMPTY {
            return;
        }

        let dp = ctx.local.debug_pin;
        // dp.set_high().unwrap();

        // Check is something in the queue.
        if *STATE == PwmData::EMPTY {
            if let Some(pd) = ctx.local.led_buf_consumer.dequeue() {
                *STATE = pd;
            }
        }

        let local_pwm = ctx.shared.pwm;
        if let Some(pwm) = local_pwm.take() {
            *local_pwm = match STATE {
                PwmData::RAW(p) => {
                    let (buf0, buf1, pwm) = pwm.split();
                    
                    let (buf0, start_pwm) = if let Some(buf0) = buf0 {
                        let next_data = p.pwm_next(buf0);
                        dp.set_high().unwrap();
                        if !next_data {
                            *STATE = PwmData::EMPTY;
                            dp.set_low().unwrap();
                        }                        
                        (Some(buf0), true)
                    } else {
                        defmt::println!("NoBUF0");
                        (None, false)
                    };
                    
                    pwm.load(buf0, buf1, start_pwm).ok()
                }                
                PwmData::EMPTY => {
                    dp.set_low().unwrap();
                    pwm.stop();
                    Some(pwm)
                }
                PwmData::SINGLE { repeat, data } => {
                    let (buf0, buf1, pwm) = pwm.split();
                    let buf0 = buf0.unwrap();
                    if *repeat == 0 {
                        for data in buf0.iter_mut() {
                            *data = PWM_POL;
                        }
                        dp.set_low().unwrap();
                        *STATE = PwmData::EMPTY;
                    } else {
                        data.pwm_next(buf0);
                        *repeat -= 1;
                    }
                    pwm.load(Some(buf0), buf1, true).ok()
                }
                PwmData::SetProgram(_) => Some(pwm),
            };
        }

        //dp.set_low().unwrap();
    }

    #[task(priority = 3, local = [led_buf_producer,update_pin])]
    fn pwm_enqueue(ctx: pwm_enqueue::Context, input: Frame) {
        let led = ctx.local.update_pin;
        led.set_high().unwrap();
        if ctx
            .local
            .led_buf_producer
            .enqueue(PwmData::RAW(input))
            .is_ok()
        {
		update_pwm::spawn(true).ok();
        } else {
		// defmt::println!("pwm_enqueue");
	}        
        led.set_low().unwrap();
    }

    #[task(local = [buf: [u8; TOTAL_BYTES] = [0u8; TOTAL_BYTES], state: State = State::R_UP, tick: u8 = 0])]
    fn program_loop_one(ctx: program_loop_one::Context) {
        let BUF = ctx.local.buf;
        let STATE = ctx.local.state;
        let TICK = ctx.local.tick;

        if let Some(mut buf) = Frame::new() {
            // move BUFfer to the right
            for i in (0..BUF.len() - N_BYTES).rev() {
                BUF[i + N_BYTES] = BUF[i]
            }

            // Render new colour
            if *TICK != 0 {
                *TICK -= 1;
            } else {
                *TICK = 20;
                match STATE {
                    State::R_UP => {
                        // R UP, B FULL
                        BUF[R] += 1;
                        BUF[B] = LEDMAX - BUF[R];
                        if BUF[1] >= LEDMAX << 1 {
                            // This check is needed because at startup Blue value = 0.
                            // The causes BUF[2] to underflow in B_DOWN.
                            // Just skip it, go stread to G_UP
                            // *STATE = if BUF[2] != 0 {
                            //     State::B_DOWN
                            // } else {
                            //     State::G_UP
                            // };
                            *STATE = State::B_DOWN;
                        }
                    }
                    State::B_DOWN => {
                        // B DOWN, R FULL
                        BUF[B] -= 1;
                        BUF[R] = LEDMAX - BUF[B];
                        if BUF[B] == 0 {
                            *STATE = State::G_UP;
                        }
                    }
                    State::G_UP => {
                        // G UP, R FULL
                        BUF[G] += 1;
                        BUF[R] = LEDMAX - BUF[G];
                        if BUF[G] >= LEDMAX << 1 {
                            *STATE = State::R_DOWN;
                        }
                    }
                    State::R_DOWN => {
                        // R DOWN, G FULL
                        BUF[R] -= 1;
                        BUF[G] = LEDMAX - BUF[R];
                        if BUF[R] == 0 {
                            *STATE = State::B_UP;
                        }
                    }
                    State::B_UP => {
                        // B UP, G FULL
                        BUF[B] += 1;
                        BUF[G] = LEDMAX - BUF[B];
                        if BUF[B] >= LEDMAX << 1 {
                            *STATE = State::G_DOWN;
                        }
                    }
                    State::G_DOWN => {
                        // G DOWN, B FULL
                        BUF[G] -= 1;
                        BUF[B] = LEDMAX - BUF[G];
                        if BUF[G] == 0 {
                            *STATE = State::R_UP;
                        }
                    }
                }
                // rprintln!("{:?}: {} {} {}", *STATE, BUF[G], BUF[R], BUF[B]);
            }

            if buf.copy_from_slice(&BUF[..]) {
                // Push
                pwm_enqueue::spawn(buf).ok();
            } else {
                defmt::println!("copy_from_slice");
            }
        } else {
            // defmt::println!("No buf!");
        }
    }

    #[task(local = [buf: [u8; TOTAL_BYTES] = [0u8; TOTAL_BYTES], state: State = State::R_UP, tick: u8 = 0])]
    fn program_loop_two(ctx: program_loop_two::Context) {
        let BUF = ctx.local.buf;
        let STATE = ctx.local.state;
        let TICK = ctx.local.tick;

        const STEP_SIZE: u8 = 10;

        if let Some(mut buf) = Frame::new() {
            // move BUFfer to the right
            for i in (0..BUF.len() - N_BYTES).rev() {
                BUF[i + N_BYTES] = BUF[i]
            }

            // Render new colour
            if *TICK != 0 {
                *TICK -= 1;
            } else {
                *TICK = 10;
                match STATE {
                    State::R_UP => {
                        // R UP, B FULL
                        BUF[R] = if BUF[R] >= (LEDMAX - STEP_SIZE) {
                            LEDMAX
                        } else {
                            BUF[R] + STEP_SIZE
                        };
                        BUF[B] = LEDMAX - BUF[R];
                        if BUF[1] >= LEDMAX << 1 {
                            // This check is needed because at startup Blue value = 0.
                            // The causes BUF[2] to underflow in B_DOWN.
                            // Just skip it, go stread to G_UP
                            // *STATE = if BUF[2] != 0 {
                            //     State::B_DOWN
                            // } else {
                            //     State::G_UP
                            // };
                            *STATE = State::B_DOWN;
                        }
                    }
                    State::B_DOWN => {
                        // B DOWN, R FULL
                        BUF[B] = if BUF[B] <= STEP_SIZE {
                            0
                        } else {
                            BUF[B] - STEP_SIZE
                        };
                        BUF[R] = LEDMAX - BUF[B];
                        if BUF[B] == 0 {
                            *STATE = State::G_UP;
                        }
                    }
                    State::G_UP => {
                        // G UP, R FULL
                        BUF[G] = if BUF[G] >= (LEDMAX - STEP_SIZE) {
                            LEDMAX
                        } else {
                            BUF[G] + STEP_SIZE
                        };
                        BUF[R] = LEDMAX - BUF[G];
                        if BUF[G] >= LEDMAX << 1 {
                            *STATE = State::R_DOWN;
                        }
                    }
                    State::R_DOWN => {
                        // R DOWN, G FULL
                        BUF[R] = if BUF[R] <= STEP_SIZE {
                            0
                        } else {
                            BUF[R] - STEP_SIZE
                        };
                        BUF[G] = LEDMAX - BUF[R];
                        if BUF[R] == 0 {
                            *STATE = State::B_UP;
                        }
                    }
                    State::B_UP => {
                        // B UP, G FULL
                        BUF[B] = if BUF[B] >= (LEDMAX - STEP_SIZE) {
                            LEDMAX
                        } else {
                            BUF[B] + STEP_SIZE
                        };
                        BUF[G] = LEDMAX - BUF[B];
                        if BUF[B] >= LEDMAX << 1 {
                            *STATE = State::G_DOWN;
                        }
                    }
                    State::G_DOWN => {
                        // G DOWN, B FULL
                        BUF[G] = if BUF[G] <= STEP_SIZE {
                            0
                        } else {
                            BUF[G] - STEP_SIZE
                        };
                        BUF[B] = LEDMAX - BUF[G];
                        if BUF[G] == 0 {
                            *STATE = State::R_UP;
                        }
                    }
                }
                // rprintln!("{:?}: {} {} {}", *STATE, BUF[G], BUF[R], BUF[B]);
            }

            if buf.copy_from_slice(&BUF[..]) {
                // Push
                pwm_enqueue::spawn(buf).ok();
            } else {
                // defmt::println!("copy_from_slice");
            }
        } else {
            // defmt::println!("No buf!");
        }
    }

    // USB EP has a fixed buffer size.
    const USB_EP_BUF_SIZE: usize = 64;

    //shared = [packet_buf]
    #[task(shared = [program], local = [usb_serial, usb_device, ser_buf_producer, rxrf_buf_consumer, usb_buffer: [u8; 1024] = [0; 1024], usb_write: [u8; 256] = [0; 256], start: usize = 0])]
    fn usb_poll(mut ctx: usb_poll::Context) {
        let serial = ctx.local.usb_serial;


        if ctx.local.usb_device.poll(&mut [serial]) {
                let USB_BUFFER = ctx.local.usb_buffer;
                let start = *ctx.local.start;
                let end = start + USB_EP_BUF_SIZE;
                let buf_prt = &mut USB_BUFFER[start..end];

            match serial.read(buf_prt) {
                Ok(0) => panic!("Can't happen!"),
                Ok(count) => {
                    // defmt::println!("c{}s{}", count, start);
                    // zero bytes is expected on the end.
                    let found_zero = buf_prt[count - 1] == 0;

                    // Invalid data, expect count < USB_EP_BUF_SIZE and no found_zero.
                    *ctx.local.start = if count != USB_EP_BUF_SIZE && !found_zero { 
                        defmt::println!("data error");
                        0
                    } else if found_zero {
                        let cmd = Commands::from_bytes(&mut USB_BUFFER[..start+count]);

                        let res: Responce = match cmd {
                            Some(Commands::LedData(data)) => {
                                // defmt::println!("Set Led");
                                if let Some(mut pf) = Frame::new() {
                                    let _ = pf.copy_from_slice(data);
                                
                                    match ctx.local.ser_buf_producer.enqueue(PwmData::RAW(pf)) {
                                        Ok(_) => Responce::LedAcceptedBufferSpace(1),
                                        Err(_) => Responce::Error,
                                    }
                                } else {
                                    Responce::BufferFull
                                }
                            }
                            Some(Commands::SetProgram(prg)) => {
                                defmt::println!("Set PRG {}", prg);
                                match ctx.local.ser_buf_producer.enqueue(PwmData::SetProgram(prg)) {
                                    Ok(_) => Responce::Ok,
                                    Err(_) => Responce::Error,
                                }                                
                            }
                            Some(Commands::GetProgram) => {
                                defmt::println!("Get PRG");
                                ctx.shared.program.lock( |prg| {
                                    Responce::Program(*prg)
                                })
                            }
                            _ => Responce::Error,
                        };

                        let ser_res = res.to_slice(&mut USB_BUFFER[..]).unwrap();
                        let _ = serial.write(ser_res);
                        0
                    } else if start + USB_EP_BUF_SIZE >=USB_BUFFER.len() {
                        0
                    } else {
                        start + USB_EP_BUF_SIZE
                    }
                }
                Err(_e) => (), //rprintln!("{:?}", e),
            }
        }

        // Write data
        if let Some(pkt) = ctx.local.rxrf_buf_consumer.dequeue() {                
            let mut tb = [0; 144];
            let slice = pkt.to_slice(&mut tb);
            let res = Responce::RadioRecv(&slice);
            let ser_res = res.to_slice(&mut ctx.local.usb_write[..]).unwrap();
            // defmt::println!("TX COBS {}", ser_res);
            let _ = serial.write(ser_res);
        }
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        defmt::println!("INIT LOOP");
        loop {
            usb_poll::spawn().ok();
            // Don't use this, causes USB not to work.
            //cortex_m::asm::wfi();
        }
    }

    #[task(priority = 2, shared = [radio])]
    fn radio_handler(ctx: radio_handler::Context, action: RadioAction) {
        defmt::println!("RADIO");

        if let Some(mut radio) = ctx.shared.radio.take() {
            radio.action(action);

            *ctx.shared.radio = Some(radio);            
        }
    }

    #[task(priority = 2, binds=RADIO, shared = [radio])]
    fn RADIO_handler(ctx: RADIO_handler::Context) {
        if let Some(mut radio) = ctx.shared.radio.take() {
            radio.handle_interrupt();
            
            *ctx.shared.radio = Some(radio);
        }
    }

    #[task(priority = 3, binds = PWM0, shared = [pwm])]
    fn PWM0_handler(ctx: PWM0_handler::Context) { 
        if let Some(pwm) = ctx.shared.pwm {
            if pwm.is_event_triggered(PwmEvent::SeqEnd(Seq::Seq0)) {
                pwm.reset_event(PwmEvent::SeqEnd(Seq::Seq0));
                update_pwm::spawn(false).ok();
            }
        }
    }

    #[task(binds = TIMER0, shared = [program], local = [ser_buf_consumer, tmr0, led_gr, tick: bool = false, timeout: u16 = 500])]
    fn on_TIMER0(mut ctx: on_TIMER0::Context) {
        let TICK = ctx.local.tick;

        ctx.local.tmr0.reset_event(0);

        let led = ctx.local.led_gr;
        let ser_buf = ctx.local.ser_buf_consumer;
        let timeout = ctx.local.timeout;

        ctx.shared.program.lock(|prg| {      
            if let Some(cmd) = ser_buf.dequeue() {          
                match cmd {
                    PwmData::RAW(data) => {
                        if *prg == uart_protocol::Programs::Serial {
                            *timeout = 500;
                            pwm_enqueue::spawn(data).ok();
                        } else {
                            drop(data);
                        };
                    }
                    PwmData::SetProgram(program) => {
                        defmt::println!("Switch program");
                        *prg = program;
                    }
                    _e => (),
                }
            }

            if *TICK {
                *TICK = false;
                led.set_low().unwrap();
            } else {
                *TICK = true;
                led.set_high().unwrap();
                match *prg {
                    uart_protocol::Programs::Serial => (
                        // if *timeout == 0 {
                        //     // *prg = uart_protocol::Programs::Two;
                        //     rprintln!("Timeout! Switch program to {:?}", *prg);
                        // } else {
                        //     *timeout -= 1; 
                        // }
                    ),
                    uart_protocol::Programs::One => {
                        let _ = program_loop_one::spawn().ok();
                    }
                    uart_protocol::Programs::Two => {
                        let _ = program_loop_two::spawn().ok();
                    }
                }
            }
        });
    }

    // #[task(binds = USBD, spawn=[usb_poll], shared = [usb_device])]
    // fn int_usb_handle(ctx: int_usb_handle::Context) {
    //     ctx.shared.usbd;
    //     usb_poll::spawn().ok();
    // }
}

#[allow(unused)]
fn hex(value: u8, buf: &mut [u8]) {
    let mut value = value;
    for data in buf[0..=1].iter_mut() {
        *data = match (value & 0xf0) >> 4 {
            0 => '0',
            1 => '1',
            2 => '2',
            3 => '3',
            4 => '4',
            5 => '5',
            6 => '6',
            7 => '7',
            8 => '8',
            9 => '9',
            10 => 'A',
            11 => 'B',
            12 => 'C',
            13 => 'D',
            14 => 'E',
            15 => 'F',
            _ => '?',
        } as u8;
        value <<= 4;
    }
}

// #[task(binds = USBD, shared = [usb_device, usb_serial, pwm, led_buf])]
// fn usb_handler(ctx: usb_handler::Context) {
//     static mut BUF: [u8; N_BYTES * N_LEDS] = [0u8; N_BYTES * N_LEDS];
//     let mut cnt: usize = 0;

//     let serial = ctx.shared.usb_serial;

//     let mut buf = [0u8; 64];

//     match serial.read(&mut buf) {
//         Ok(count) if count > 0 => {
//             let mut buf = &mut buf[..count];
//             let mut pos = 0;

//             let lb = ctx.shared.led_buf.as_mut();
//             if let Some(lb) = lb {
//                 loop {
//                     BUF[cnt] = buf[pos];
//                     cnt += 1;
//                     if cnt == BUF.len() {
//                         lb.push(PwmData::RAW(Pwm0State {
//                             cnt: 0,
//                             tail: 0,
//                             buf: BUF.clone(),
//                         }));

//                         *ctx.shared.pwm = update_pwm(ctx.shared.pwm.take().unwrap(), lb);
//                         cnt = 0;
//                     }
//                     pos += 1;
//                     if pos == count {
//                         break;
//                     }
//                 }
//             }

//             // Echo back in upper case
//             for c in buf.iter_mut() {
//                 if 0x61 <= *c && *c <= 0x7a {
//                     *c &= !0x20;
//                 }
//             }

//             while !buf.is_empty() {
//                 match serial.write(buf) {
//                     Ok(len) => buf = &mut buf[len..],
//                     _ => {}
//                 }
//             }
//         }
//         _ => {}
//     }
//     ctx.shared.usb_device.poll(&mut [serial]);
// }

// 03 08  B8  FF FF  FF FF  07
// 03 08  FA  FF FF  FF FF  07
// 01 C8  4D  FF FF  FF FF  99 88 19 02 AD FE FF F9 E3 B4 0B 00 0B 00 10 5E C0 11 FB 00 E9 5B 25 A7 02 93
