#![deny(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![no_main]
#![no_std]

mod timers;
use core::convert::TryFrom;
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
    gpio::{Input, Level, Output, Pin, PullUp, PushPull},
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

use uart_protocol::{self, Commands, Programs, Responce, N_BYTES, N_LEDS, TOTAL_BYTES};

pool!(FRAMEBUFFER: Deque<u8, TOTAL_BYTES>);

// Number of leds programmed in a single run
const PWM_N_LEDS: usize = 4;

const PWM_POL: u16 = 0x8000;

const PWM_T0H: u16 = 6 | PWM_POL;
const PWM_T1H: u16 = 13 | PWM_POL;
const PWM_MAX: u16 = 20;


fn itoa(s: &mut heapless::String<10>, input: u32) {
    if input >= 10 {
        itoa(s, input / 10);
    }
    let c = (input % 10) + b'0' as u32;
    let cr = char::from_u32(c).unwrap();
    s.push(cr).unwrap();
}

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
                    buf[n] = if d & 0x80 == 0 { PWM_T0H } else { PWM_T1H };
                    d <<= 1;
                    n += 1;
                }
            }
        }

        buf[PWM_DMA_MEM_SIZE - 1] = PWM_POL;
    }
}

pub struct Frame {
    pub inner: Box<FRAMEBUFFER>,
}

impl Frame {
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

    pub fn pwm_next(&mut self, buf: &mut [u16]) -> bool {
        let mut n = 0;
        let mut d = 0;

        for (num, data) in buf[0..(PWM_N_LEDS * N_BYTES * 8)].iter_mut().enumerate() {
            if num % 8 == 0 {
                match self.inner.pop_front() {
                    None => break,
                    Some(val) => d = val,
                };
                n += 8;
            }

            *data = if d & 0x80 == 0 { PWM_T0H } else { PWM_T1H };
            d <<= 1;
        }

        // Allways fill remaining values with empty data
        for d in buf[n..].iter_mut() {
            *d = PWM_POL;
        }

        n != 0
    }
}

pub enum RadioAction {
    Disabled,
    RxMode,
    Tx(Packet),
    Channel(u8),
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


const PATTERN: &[u8; 102] = &[
    28, 36, 35, 27, 19, 20, 21, 29, 37, 45, 44, 43, 42, 34, 26, 18, 10, 11, 12, 13, 14, 22, 30, 38,
    46, 54, 53, 52, 51, 50, 49, 41, 33, 25, 17, 9, 1, 2, 3, 4, 5, 6, 7, 15, 23, 31, 39, 47, 55, 63,
    62, 61, 60, 59, 58, 57, 56, 48, 40, 32, 24, 16, 8, 0, 1, 2, 3, 4, 5, 6, 14, 22, 30, 38, 46, 54,
    53, 52, 51, 50, 49, 41, 33, 25, 17, 9, 10, 11, 12, 13, 21, 29, 37, 45, 44, 43, 42, 34, 26, 18,
    19, 20,
];

mod leds;

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1, SWI2_EGU2, SWI3_EGU3])]
mod app {
    use defmt::println;
    use hal::pac::{TIMER0, TIMER1, TIMER2, SPIM0};
    use hal::spim::Spim;
    use nrf52840_hal::prelude::{StatefulOutputPin, InputPin};

    use super::{
        hal, ieee802154, pwm, uart_protocol, Channel, Commands, Consumer, DefaultBufferStore,
        ExternalOscillator, Frame, Input, Internal, Led, Level, LfOscStopped, LoadMode, Output,
        OutputPin, Packet, Pin, Pool, Prescaler, Producer, PullUp, PushPull, Pwm, PwmData,
        PwmEvent, PwmSeq, Queue, Radio, RadioAction, RadioReturn, Responce, Seq, SeqBuffer,
        SerialPort, State, StopWatch, TryFrom, UsbBus, UsbBusAllocator, UsbDevice,
        UsbDeviceBuilder, UsbPeripheral, UsbVidPid, Usbd, B, FRAMEBUFFER, G, LEDMAX, N_BYTES,
        N_LEDS, PWM0, PWM_DMA_MEM_SIZE, PWM_MAX, PWM_POL, R, TOTAL_BYTES, USB_CLASS_CDC, itoa,
        PATTERN, leds,
    };

    use leds::{GRB, any_as_u8_slice};

    use embedded_graphics::{
        mono_font::{ascii, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };
    use ssd1306::{prelude::*, Ssd1306, mode::BufferedGraphicsMode};

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
        tmr0: StopWatch<TIMER0>,
        led_gr: Pin<Output<PushPull>>,
        led_red: Pin<Output<PushPull>>,
        // `_led_bl: Pin<Output<PushPull>>,
        debug_pin: Pin<Output<PushPull>>,
        update_pin: Pin<Output<PushPull>>,
        tmr1: StopWatch<TIMER1>,
        tmr2: StopWatch<TIMER2>,
        display: Ssd1306<SPIInterface<Spim<SPIM0>, Pin<Output<PushPull>>, Pin<Output<PushPull>>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
        button: Pin<Input<PullUp>>,
    }

    #[shared]
    struct Shared {
        // packet_buf: ringbuffer::ConstGenericRingBuffer<Packet, 4>,
        #[lock_free]
        pwm: Option<PwmSeq<PWM0, SeqBuffer, SeqBuffer>>,
        // tmr1: Timer<TIMER1>,
        #[lock_free]
        radio: Option<Radio<'static>>,
        program: uart_protocol::Programs,
        idle_counter: usize,
        tick: usize,
    }

    #[init(local = [memory: [u8; 4096] = [0; 4096], rxrf_buf: Queue<Packet, 4> = Queue::new(), led_buf: Queue<PwmData, 4> = Queue::new(), ser_buf: Queue<PwmData, 4> = Queue::new(), radio_buf: Queue<ieee802154::RadioReturn, 4> = Queue::new(), usbbus: Option<UsbBusAllocator<UsbBus>> = None, clock: Option<hal::clocks::Clocks<ExternalOscillator, Internal, LfOscStopped>> =
    None, pwm0_buffer: [u16; PWM_DMA_MEM_SIZE] = [0; PWM_DMA_MEM_SIZE], packet_buf: Packet = Packet::new()])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let pos = FRAMEBUFFER::grow(ctx.local.memory);

        assert_eq!(pos, 4);

        *ctx.local.clock = Some(hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc());

        let clock = ctx.local.clock.as_ref().unwrap();

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

        let bla = ctx.device.UICR;

        bla.pselreset[0].write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        bla.pselreset[1].write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        let mut red_led = port0.p0_23.into_push_pull_output(Level::Low).degrade();
        let mut blue_led = port0.p0_24.into_push_pull_output(Level::Low).degrade();
        let mut green_led = port0.p0_22.into_push_pull_output(Level::High).degrade();

        let pin_debug = port0.p0_19.into_push_pull_output(Level::Low).degrade();
        let pin_update = port0.p0_20.into_push_pull_output(Level::Low).degrade();

        *ctx.local.usbbus = Some(UsbBusAllocator::new(Usbd::new(UsbPeripheral::new(
            ctx.device.USBD,
            clock,
        ))));

        let usb_bus = ctx.local.usbbus.as_ref().unwrap();

        let usb_serial = SerialPort::new(usb_bus);

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .max_packet_size_0(64) // (makes control transfers 8x faster)
            .build();

        defmt::println!("INIT1: {=u32}", ctx.local.packet_buf.as_ptr() as u32);

        // INIT RADIO
        let (radio_tx, radio_rx) = ctx.local.radio_buf.split();
        let mut radio = Radio::init(ctx.device.RADIO, clock, radio_tx, ctx.local.packet_buf);
        radio.set_channel(Channel::_15);
        radio.set_txpower(ieee802154::TxPower::Pos8dBm);
        let mut p = Packet::new();
        p.copy_from_slice(&[3, 8, 0xAA, 255, 255, 255, 255, 7]);
        radio.send(&p);

        radio_handler::spawn(RadioAction::RxMode).ok();

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
                repeat: u16::try_from(N_LEDS).expect("N_LEDS fit in u16!"),
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

        let mut tmr1 = StopWatch::new(ctx.device.TIMER1, 1_000_000);
        tmr1.start();

        let mut tmr2 = StopWatch::new(ctx.device.TIMER2, 100_000); // 8 Hz
        tmr2.start();

        let (ser_buf_producer, ser_buf_consumer) = ctx.local.ser_buf.split();
        // let (rxrf_buf_producer, rxrf_buf_consumer) = ctx.local.rxrf_buf.split();

        // Button
        let button = port0.p0_03.into_pullup_input().degrade();

        // OLED/SPI Setup
        let lcd_spi_clk = port0.p0_10.into_push_pull_output(Level::Low).degrade();
        let lcd_spi_mosi = port0.p0_09.into_push_pull_output(Level::Low).degrade();
        let lcd_spi_cs = port0.p0_07.into_push_pull_output(Level::High).degrade();
        let _lcd_rst = port0.p0_06.into_push_pull_output(Level::High).degrade();
        let lcd_spi_dcx = port0.p0_08.into_push_pull_output(Level::High).degrade();

        let pins = hal::spim::Pins {
            sck: Some(lcd_spi_clk),
            miso: None,
            mosi: Some(lcd_spi_mosi),
        };

        let spi = hal::Spim::new(
            ctx.device.SPIM0,
            pins,
            hal::spim::Frequency::K500,
            hal::spim::MODE_0,
            0,
        );

        let interface = SPIInterface::new(spi, lcd_spi_dcx, lcd_spi_cs);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        // display
        display.init().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&ascii::FONT_9X15)
            .text_color(BinaryColor::On)
            .background_color(BinaryColor::Off)
            .build();

        let text_style_invert = MonoTextStyleBuilder::new()
            .font(&ascii::FONT_9X15)
            .text_color(BinaryColor::Off)
            .background_color(BinaryColor::On)
            .build();

        Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        Text::with_baseline(
            "Hello Rust!",
            Point::new(32, 16),
            text_style_invert,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        Text::with_baseline("Hello world!", Point::new(0, 32), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        Text::with_baseline("Hello world!", Point::new(0, 48), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.set_pixel(127, 63, true);
        display.flush().unwrap();

        (
            Shared {
                // packet_buf: PACKET_BUF,
                pwm,
                // tmr1,
                radio: Some(radio),
                program: uart_protocol::Programs::Two,
                idle_counter: 0,
                tick: 0,
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
                led_red: red_led,
                tmr0,
                debug_pin: pin_debug,
                update_pin: pin_update,
                tmr1,
                tmr2,
                display,
                button,
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
                BUF[i + N_BYTES] = BUF[i];
            }

            // Render new colour
            if *TICK == 0 {
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
            } else {
                *TICK -= 1;
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

    #[task(local = [buf: [GRB; 64] = [GRB { r: 0, g: 0, b: 0}; 64], state: u8 = 0, col: u8 = 0])]
    fn program_loop_two(ctx: program_loop_two::Context) {
        let BUF = ctx.local.buf;
        let STATE = ctx.local.state;
        let COL = ctx.local.col;

        if let Some(mut buf) = Frame::new() {
            *STATE += 1;
            if usize::from(*STATE) == PATTERN.len() { *COL += 1; *STATE = 0 };
            if *COL == 9 { *COL = 0 };
            let loc = PATTERN[usize::from(*STATE)];

            for d in BUF.iter_mut() {
                d.effect();
            }

            let pos = usize::from(loc);

            const D: u8 = 0x10;

            let pixel = match *COL {
                0 => &[255, 0, 0],
                1 => &[0, 255, 0],
                2 => &[0, 0, 255],
                3 => &[D, D, 0],
                4 => &[D, 0, D],
                5 => &[0, D, D],
                6 => &[0xAF, 255, 0xF],
                7 => &[0xFF, 0xFF, 0xFF],
                8 => &[0x1F, 0xFF, 0x1F],
                _ => unreachable!("Error!"),
            };

            BUF[pos] = GRB::from_bytes(pixel);

            let refbuf = unsafe { any_as_u8_slice(BUF) };

            if buf.copy_from_slice(&refbuf[..]) {
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
                        defmt::println!("data error c{} f{}", count, found_zero);
                        0
                    } else if found_zero {
                        let cmd = Commands::from_bytes(&mut USB_BUFFER[..start + count]);

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
                                ctx.shared.program.lock(|prg| Responce::Program(*prg))
                            }
                            Some(Commands::RadioChannel(c)) => {
                                if radio_handler::spawn(RadioAction::Channel(c)).is_ok() {
                                    Responce::Ok
                                } else {
                                    Responce::Reject
                                }
                            }
                            Some(Commands::RadioSend(d)) => {
                                let mut p = Packet::new();
                                p.copy_from_slice(d);
                                if radio_handler::spawn(RadioAction::Tx(p)).is_ok() {
                                    Responce::RadioAcceptedBufferSpace(1)
                                } else {
                                    Responce::Error
                                }
                            }
                            _ => Responce::Error,
                        };

                        let ser_res = res.to_slice(&mut USB_BUFFER[..]).unwrap();
                        let _ = serial.write(ser_res);
                        0
                    } else if start + USB_EP_BUF_SIZE >= USB_BUFFER.len() {
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
            let res = Responce::RadioRecv(slice);
            let ser_res = res.to_slice(&mut ctx.local.usb_write[..]).unwrap();
            // defmt::println!("TX COBS {}", ser_res);
            let _ = serial.write(ser_res);
        }
    }

    #[idle(shared = [idle_counter])]
    fn idle(mut ctx: idle::Context) -> ! {
        defmt::println!("INIT LOOP");
        loop {
            ctx.shared.idle_counter.lock(|v| {
                *v += 1;
            });
            usb_poll::spawn().ok();
            // Don't use this, causes USB not to work.
            //cortex_m::asm::wfi();
        }
    }

    #[task(priority = 2, shared = [radio])]
    fn radio_handler(ctx: radio_handler::Context, action: RadioAction) {
        defmt::println!("RADIO");

        if let Some(mut radio) = ctx.shared.radio.take() {
            radio.action(&action);

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

    #[task(binds = TIMER2, shared = [tick], local = [tmr2, button, btn1_state: usize = 0, led_red])]
    fn on_TIMER2(mut ctx: on_TIMER2::Context) {
        const KEY_SHORT: usize = 10;

        ctx.local.tmr2.reset_event(0);

        let tick = ctx.shared.tick.lock(|t| {
            let mut tick = *t;
            tick += 1;
            *t = tick;
            tick
        });

        let button = ctx.local.button.is_low().unwrap();

        let button_time = *ctx.local.btn1_state;

        let delta = tick - button_time;

        if button_time == 0 {
            if button {
                *ctx.local.btn1_state = tick;
                defmt::println!("key: press");
                ctx.local.led_red.set_low().unwrap();
            }            
        } else {
            if button {
                if (delta % KEY_SHORT) == 0 {
                    defmt::println!("longkey: still pressed {}", delta);
                }
                if delta > KEY_SHORT {
                    if ctx.local.led_red.is_set_high().unwrap() {
                        ctx.local.led_red.set_low().unwrap();
                    } else {
                        ctx.local.led_red.set_high().unwrap();
                    }
                }
            } else {
                if delta < KEY_SHORT {
                    defmt::println!("shortkey {}", delta);
                } else {
                    defmt::println!("longkey: released {}", delta);
                }
                *ctx.local.btn1_state = 0;
                ctx.local.led_red.set_high().unwrap();
            }
        }
    }

    #[task(binds = TIMER1, shared = [idle_counter, program], local = [tmr1, led_gr, display])]
    fn on_TIMER1(mut ctx: on_TIMER1::Context) {
        ctx.local.tmr1.reset_event(0);

        let mut bla: heapless::String<10> = heapless::String::new();
   
        ctx.shared.idle_counter.lock(|c| {
            let q = *c;
            println!("IDLE: {=usize}", q);
            *c = 0;
            itoa(&mut bla, q as u32);
        });

        let led = ctx.local.led_gr;
        //let led_red = ctx.local.led_red;
        //let button = ctx.local.button.is_low().unwrap();


        if led.is_set_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }

        let display = ctx.local.display;

        let text_style = MonoTextStyleBuilder::new()
            .font(&ascii::FONT_9X15)
            .text_color(BinaryColor::Off)
            .background_color(BinaryColor::On)
            .build();

        Text::with_baseline(bla.as_str(), Point::new(0, 48), text_style, Baseline::Top)
            .draw(display)
            .unwrap();

        // if button {
        //     led_red.set_high().unwrap();

        //     bla.clear();
        //     bla.push_str("PRG: ");

        //     ctx.shared.program.lock(|prg| {
        //         *prg = match *prg {
        //             uart_protocol::Programs::One => {
        //                 bla.push('2');
        //                 uart_protocol::Programs::Two
        //             },
        //             uart_protocol::Programs::Two |
        //             uart_protocol::Programs::Serial => {
        //                 bla.push('1');
        //                 uart_protocol::Programs::One
        //             },
        //         };

        //     });

        //     Text::with_baseline(&bla, Point::zero(), text_style, Baseline::Top)
        //         .draw(display)
        //         .unwrap();
        // } else {
        //     led_red.set_low().unwrap();            
        // };

        display.flush().unwrap();


    }

    #[task(binds = TIMER0, shared = [program], local = [ser_buf_consumer, tmr0, tick: bool = false, timeout: u16 = 500])]
    fn on_TIMER0(mut ctx: on_TIMER0::Context) {
        let TICK = ctx.local.tick;

        ctx.local.tmr0.reset_event(0);

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
            } else {
                *TICK = true;

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
