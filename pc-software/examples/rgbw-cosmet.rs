use rand::{self, Rng};
use std::{thread, time::Duration};

use pc_software::UartLeds;
use rgb::{self, RGBA8, ComponentBytes};

const N_LEDS: usize = 50;

fn effect_u8(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 7;
    data >>= 3;
    *led = data as u8;
}

fn effect(led: &mut RGBA8) {
    effect_u8(&mut led.r);
    effect_u8(&mut led.g);
    effect_u8(&mut led.b);
    effect_u8(&mut led.a);
}

fn dim(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 7;
    data >>= 3;
    *led = data as u8;
}

const COLMAX: usize = 14;

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

    loop {
        let col = rand::thread_rng().gen_range(0..=COLMAX);
        
        println!("\rCol: {col}");

        buf[0] = match col {
            0 => rgb::RGBA8 { r: 255, g: 0, b: 0, a: 0},
            1 => rgb::RGBA8 { r: 0, g: 255, b: 0, a: 0 },
            2 => rgb::RGBA8 { r: 0, g: 0, b: 255, a: 0 },
            3 => rgb::RGBA8 { r: 0, g: 0, b: 0, a: 0xFF },
            4 => rgb::RGBA8 { r: 255, g: 255, b: 0, a: 0 },
            5 => rgb::RGBA8 { r: 255, g: 0, b: 255, a: 0 },
            6 => rgb::RGBA8 { r: 0, g: 255, b: 255, a: 0 },
            7 => rgb::RGBA8 { r: 0xAF, g: 255, b: 0xF, a: 0 },
            8 => rgb::RGBA8 { r: 0, g: 0, b: 0, a: 0xFF },
            9 => rgb::RGBA8 { r: 0x1F, g: 0xFF, b: 0x1F, a: 0 },
            10 => rgb::RGBA8 { r: 0, g: 0xFF, b: 0, a: 0xFF },
            11 => rgb::RGBA8 { r: 0xFF, g: 0, b: 0, a: 0xFF },
            12 => rgb::RGBA8 { r: 0, g: 0, b: 0xFF, a: 0xFF },
            13 => rgb::RGBA8 { r: 0xFF, g: 0xFF, b: 0xFF, a: 0xFF },
            14 => rgb::RGBA8 { r: 0x40, g: 0x40, b: 0x40, a: 0x40 },
            _ => unreachable!("Error!"),
        };


        for _ in 0..50 {
            effect(&mut buf[0]);

            for i in (1..buf.len()).rev() {
                buf[i] = buf[i-1];
            }

            match port.write_bytes(buf.as_bytes()) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return Ok(());
                }
            }

            thread::sleep(Duration::from_millis(40));
        }    
    }

    let _ = port.restore_program();

    Ok(())
}
