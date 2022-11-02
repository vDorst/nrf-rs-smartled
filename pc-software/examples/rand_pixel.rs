#![deny(clippy::pedantic)]

use crossbeam_channel::bounded;
use crossterm::event::{poll, read, Event, KeyCode::Char, KeyEvent};
use crossterm::event::{KeyEventKind, KeyEventState, KeyModifiers};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use pc_software::UartLeds;
use rand::seq::SliceRandom;
use rand::{self, Rng};
use std::time::SystemTime;
use std::{thread, time::Duration};

const N_X: usize = 8;
const N_Y: usize = 8;

const N_LEDS: usize = N_X * N_Y;

// const SUB: u8 = 4;

// #[allow(clippy::cast_possible_truncation)]
// fn effect(led: &mut u8) {
//     *led = if *led > SUB {
//         *led - SUB
//     } else {
//         0
//     }
// }

#[allow(clippy::cast_possible_truncation)]
fn effect(led: &mut u8) {
    let mut data: u16 = u16::from(*led);
    data *= 31;
    data >>= 5;
    *led = data as u8;
}

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct GRB {
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

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    ::std::slice::from_raw_parts((p as *const T) as *const u8, ::std::mem::size_of::<T>())
}

fn main() -> crossterm::Result<()> {
    let mut buf: [GRB; N_LEDS] = [GRB::default(); N_LEDS];

    let mut port = match UartLeds::new(None) {
        Ok(port) => port,
        Err(e) => {
            println!("Error {e}");
            return Err(crossterm::ErrorKind::new(std::io::ErrorKind::Other, e));
        }
    };
    let time_delay = Duration::from_millis(1000 / 64);

    let (s, r) = bounded::<bool>(10);

    let mut pixel = &[0x80, 0, 0];

    let mut field = Vec::with_capacity(N_LEDS);
    for n in 0..N_LEDS {
        field.push(n);
    }

    let id = thread::spawn(move || {
        'lus: loop {
            field.shuffle(&mut rand::thread_rng());

            for &pos in &field {
                for d in buf.iter_mut() {
                    d.effect();
                }

                match r.try_recv() {
                    Ok(_) => {
                        let col: u8 = rand::thread_rng().gen_range(0..=6);

                        const S: u8 = 0x50;
                        const D: u8 = S >> 1;
                        const T: u8 = S >> 2;

                        pixel = match col {
                            0 => &[S, 0, 0],
                            1 => &[0, S, 0],
                            2 => &[0, 0, S],
                            3 => &[D, D, 0],
                            4 => &[D, 0, D],
                            5 => &[0, D, D],
                            6 => &[T, T, T],
                            7 => &[0xAF, 255, 0xF],
                            8 => &[0xFF, 0xFF, 0xFF],
                            9 => &[0x1F, 0xFF, 0x1F],
                            _ => unreachable!("Error!"),
                        };
                    }
                    Err(crossbeam_channel::TryRecvError::Disconnected) => break 'lus,
                    Err(crossbeam_channel::TryRecvError::Empty) => (),
                }

                buf[pos] = GRB::from_bytes(pixel);

                let refbuf = unsafe { any_as_u8_slice(&buf) };

                match port.write_bytes(refbuf) {
                    Ok(_) => (),
                    Err(e) => {
                        println!("Serial error {:?}", e);
                        break 'lus;
                    }
                }
                thread::sleep(time_delay);
            }
        }
        let _e = port.restore_program();
    });

    enable_raw_mode()?;

    let mut update = Duration::from_secs(1);

    let mut e_last = SystemTime::now();

    let mut e_vec = Vec::<Duration>::with_capacity(10);

    loop {
        if s.send(true).is_err() {
            break;
        };

        if poll(update)? {
            match read()? {
                Event::Key(event) => {
                    match event {
                        KeyEvent {
                            code,
                            modifiers: KeyModifiers::NONE,
                            kind: KeyEventKind::Press,
                            state: KeyEventState::NONE,
                        } => {
                            // println!("CODE: {:?}", code);
                            match code {
                                Char(' ') => {
                                    let e_cur = SystemTime::now();
                                    let delta = e_cur.duration_since(e_last).unwrap();
                                    e_last = e_cur;
                                    if delta >= Duration::from_secs(3) {
                                        e_vec.clear();
                                        update = Duration::from_secs(4);
                                        println!("\rReset");
                                    } else {
                                        e_vec.push(delta);
                                        if e_vec.len() >= 2 {
                                            let mut total: usize = 0;
                                            for i in e_vec[1..].iter() {
                                                total += i.as_millis() as usize;
                                            }
                                            total /= e_vec.len();
                                            println!("\rBeat {} ms", total);
                                            update = Duration::from_millis(total as u64);
                                        }
                                    }
                                }
                                Char('q') => break,
                                _ => (),
                            }
                        }
                        event => println!("{:?}", event),
                    }
                }
                Event::Mouse(event) => println!("{:?}", event),
                Event::Resize(width, height) => println!("New size {}x{}", width, height),
                Event::FocusGained | Event::FocusLost | Event::Paste(_) => (),
            }
        }
    }

    disable_raw_mode()?;

    drop(s);

    let _e = id.join();

    Ok(())
}
