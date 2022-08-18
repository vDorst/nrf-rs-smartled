use rand::{self, Rng};
use std::{thread, time::Duration};

use crossterm::event::{
    read, Event,
    KeyCode::{self, Char},
    KeyEvent, KeyModifiers, KeyEventKind, KeyEventState,
};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};

use pc_software::UartLeds;
use rgb::{ComponentBytes, RGBA8};

const N_LEDS: usize = 50;
const COLMAX: usize = 15;
const BRIGHTMAX: usize = 4;

fn effect_u8(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 7;
    data >>= 3;
    *led = data as u8;
}

#[allow(dead_code)]
fn effect(led: &mut RGBA8) {
    effect_u8(&mut led.r);
    effect_u8(&mut led.g);
    effect_u8(&mut led.b);
    effect_u8(&mut led.a);
}

fn dim_u8(led: &mut u8, bright: usize) {
    let mut data: u16 = *led as u16;
    data *= bright as u16;
    data >>= BRIGHTMAX;
    *led = data as u8;
}

fn dim(led: &mut RGBA8, bright: usize) {
    dim_u8(&mut led.r, bright);
    dim_u8(&mut led.g, bright);
    dim_u8(&mut led.b, bright);
    dim_u8(&mut led.a, bright);
}

fn main() -> crossterm::Result<()> {
    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();

    // Clear all leds
    let buf: [rgb::RGBA8; 150] = [rgb::RGBA8::default(); 150];

    match port.write_bytes(buf.as_bytes()) {
        Ok(_) => (),
        Err(e) => {
            println!("Write: {:?}", e);
            return Ok(());
        }
    }

    let mut buf: [rgb::RGBA8; N_LEDS] = [rgb::RGBA8::default(); N_LEDS];

    enable_raw_mode()?;

    let mut col: usize = COLMAX;
    let mut bright: usize = 1 << BRIGHTMAX;

    loop {
        match read()? {
            Event::Key(event) => match event {
                KeyEvent {
                    code,
                    modifiers: KeyModifiers::NONE,
                    kind: KeyEventKind::Press,
                    state: KeyEventState::NONE,
                } => match code {
                    Char(' ') => {
                        col = rand::thread_rng().gen_range(0..=COLMAX);
                        bright = rand::thread_rng().gen_range(1..=(1 << BRIGHTMAX));
                    }
                    Char('q') => break,
                    KeyCode::Right => {
                        if col == COLMAX {
                            col = 0
                        } else {
                            col += 1;
                        }
                    }
                    KeyCode::Left => {
                        if col == 0 {
                            col = COLMAX
                        } else {
                            col -= 1;
                        }
                    }
                    KeyCode::Up => {
                        if bright != (1 << BRIGHTMAX) {
                            bright += 1;
                        }
                    }
                    KeyCode::Down => {
                        if bright != 0 {
                            bright -= 1;
                        }
                    }
                    Char('m') => bright = 1 << BRIGHTMAX,
                    Char('h') => bright = (1 << BRIGHTMAX) >> 1,
                    code => println!("CODE: {:?}", code),
                },
                event => println!("{:?}", event),
            },
            Event::Mouse(event) => println!("{:?}", event),
            Event::Resize(width, height) => println!("New size {}x{}", width, height),
            Event::FocusGained | Event::FocusLost | Event::Paste(_) => (),
        }

        buf[0] = match col {
            0 => rgb::RGBA8 {
                r: 255,
                g: 0,
                b: 0,
                a: 0,
            },
            1 => rgb::RGBA8 {
                r: 0,
                g: 255,
                b: 0,
                a: 0,
            },
            2 => rgb::RGBA8 {
                r: 0,
                g: 0,
                b: 255,
                a: 0,
            },
            3 => rgb::RGBA8 {
                r: 0,
                g: 0,
                b: 0,
                a: 0xFF,
            },
            4 => rgb::RGBA8 {
                r: 0xFF,
                g: 0xFF,
                b: 0xFF,
                a: 0x00,
            },
            5 => rgb::RGBA8 {
                r: 255,
                g: 255,
                b: 0,
                a: 0,
            },
            6 => rgb::RGBA8 {
                r: 255,
                g: 0,
                b: 255,
                a: 0,
            },
            7 => rgb::RGBA8 {
                r: 0,
                g: 255,
                b: 255,
                a: 0,
            },
            8 => rgb::RGBA8 {
                r: 0x80,
                g: 0xFF,
                b: 0x80,
                a: 0,
            },
            9 => rgb::RGBA8 {
                r: 0xFF,
                g: 0x80,
                b: 0x80,
                a: 0,
            },
            10 => rgb::RGBA8 {
                r: 0x80,
                g: 0x80,
                b: 0xFF,
                a: 0,
            },
            11 => rgb::RGBA8 {
                r: 0,
                g: 0xFF,
                b: 0,
                a: 0xFF,
            },
            12 => rgb::RGBA8 {
                r: 0xFF,
                g: 0,
                b: 0,
                a: 0xFF,
            },
            13 => rgb::RGBA8 {
                r: 0,
                g: 0,
                b: 0xFF,
                a: 0xFF,
            },
            14 => rgb::RGBA8 {
                r: 0xFF,
                g: 0xFF,
                b: 0xFF,
                a: 0xFF,
            },
            15 => rgb::RGBA8 {
                r: 0xFF,
                g: 0,
                b: 0x20,
                a: 0x20,
            },
            _ => unreachable!("Error!"),
        };

        dim(&mut buf[0], bright);

        println!("\rCol: {col:02} B {bright:02} {:02x?}", buf[0]);

        // Swap R and G, we are using WS2812 GRBW leds.
        {
            let RGBA8 { r, g, b: _, a: _ } = &mut buf[0];
            (*r, *g) = (*g, *r);
        }

        let tb = buf[0];

        for d in buf[1..N_LEDS].iter_mut() {
            *d = tb;
        }

        match port.write_bytes(buf.as_bytes()) {
            Ok(_) => (),
            Err(e) => {
                println!("Write: {:?}", e);
                return Ok(());
            }
        }

        thread::sleep(Duration::from_millis(20));
    }

    disable_raw_mode()?;

    let _ = port.restore_program();

    Ok(())
}
