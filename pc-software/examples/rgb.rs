use rand::{self, Rng};
use std::{thread, time::Duration};
use uart_protocol::Commands;

use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use crossterm::{
    event::{
        read, Event,
        KeyCode::{self, Char},
        KeyEvent, KeyModifiers,
    },
    Command,
};

use pc_software::UartLeds;
use rgb::{self, ComponentBytes, RGB8};

const N_LEDS: usize = 64;
const COLMAX: usize = 15;
const BRIGHTMAX: usize = 4;

fn effect_u8(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 7;
    data >>= 3;
    *led = data as u8;
}

#[allow(dead_code)]
fn effect(led: &mut RGB8) {
    effect_u8(&mut led.r);
    effect_u8(&mut led.g);
    effect_u8(&mut led.b);
}

fn dim_u8(led: &mut u8, bright: usize) {
    let mut data: u16 = *led as u16;
    data *= bright as u16;
    data >>= BRIGHTMAX;
    *led = data as u8;
}

fn dim(led: &mut RGB8, bright: usize) {
    dim_u8(&mut led.r, bright);
    dim_u8(&mut led.g, bright);
    dim_u8(&mut led.b, bright);
}

fn main() -> crossterm::Result<()> {
    let mut port = UartLeds::new("/dev/ttyACM2").unwrap();

    // Clear all leds
    let buf: [RGB8; 150] = [RGB8::default(); 150];

    match port.write_bytes(buf.as_bytes()) {
        Ok(_) => (),
        Err(e) => {
            println!("Write: {:?}", e);
            return Ok(());
        }
    }

    let mut buf: [RGB8; N_LEDS] = [RGB8::default(); N_LEDS];

    enable_raw_mode()?;

    let mut col: usize = COLMAX;
    let mut bright: usize = 1 << BRIGHTMAX;

    let mut cmd: Option<Commands> = None;

    loop {
        match read()? {
            Event::Key(event) => match event {
                KeyEvent {
                    code,
                    modifiers: KeyModifiers::NONE,
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
                    Char('p') => cmd = Some(Commands::GetProgram),
                    code => println!("CODE: {:?}", code),
                },
                event => println!("{:?}", event),
            },
            Event::Mouse(event) => println!("{:?}", event),
            Event::Resize(width, height) => println!("New size {}x{}", width, height),
        }

        let ser_ret = if let Some(c) = cmd {
            let ret = port.write_command(c);
            cmd = None;
            ret
        } else {
            buf[0] = match col {
                0 => RGB8 { r: 255, g: 0, b: 0 },
                1 => RGB8 { r: 0, g: 255, b: 0 },
                2 => RGB8 { r: 0, g: 0, b: 255 },
                3 => RGB8 { r: 0, g: 0, b: 0 },
                4 => RGB8 {
                    r: 0xFF,
                    g: 0xFF,
                    b: 0xFF,
                },
                5 => RGB8 {
                    r: 255,
                    g: 255,
                    b: 0,
                },
                6 => RGB8 {
                    r: 255,
                    g: 0,
                    b: 255,
                },
                7 => RGB8 {
                    r: 0,
                    g: 255,
                    b: 255,
                },
                8 => RGB8 {
                    r: 0x80,
                    g: 0xFF,
                    b: 0x80,
                },
                9 => RGB8 {
                    r: 0xFF,
                    g: 0x80,
                    b: 0x80,
                },
                10 => RGB8 {
                    r: 0x80,
                    g: 0x80,
                    b: 0xFF,
                },
                11 => RGB8 {
                    r: 0,
                    g: 0xFF,
                    b: 0,
                },
                12 => RGB8 {
                    r: 0xFF,
                    g: 0,
                    b: 0,
                },
                13 => RGB8 {
                    r: 0,
                    g: 0,
                    b: 0xFF,
                },
                14 => RGB8 {
                    r: 0xFF,
                    g: 0xFF,
                    b: 0xFF,
                },
                15 => RGB8 {
                    r: 0xFF,
                    g: 0,
                    b: 0x20,
                },
                _ => unreachable!("Error!"),
            };

            dim(&mut buf[0], bright);

            println!("\rCol: {col:02} B {bright:02} {:02x?}", buf[0]);

            // Swap R and G, we are using WS2812 GRBW leds.
            {
                let RGB8 { r, g, b: _ } = &mut buf[0];
                (*r, *g) = (*g, *r);
            }

            let tb = buf[0];

            for d in buf[1..N_LEDS].iter_mut() {
                *d = tb;
            }

            port.write_bytes(buf.as_bytes())
        };

        match ser_ret {
            Ok(r) => println!("{:?}", r),
            Err(e) => {
                println!("Write: {:?}", e);
                return Ok(());
            }
        }
    }

    disable_raw_mode()?;

    let _ = port.restore_program();

    Ok(())
}
