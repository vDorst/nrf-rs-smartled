use crossterm::event::KeyModifiers;
use pc_software::UartLeds;        
use rand::{self, Rng};
use std::time::SystemTime;
use std::{thread, time::Duration};
use crossbeam_channel::{bounded};
use crossterm::event::{poll, read, Event, KeyEvent, KeyCode::{Char}};
use crossterm::terminal::{enable_raw_mode, disable_raw_mode};


const N_LEDS: usize = 300;

fn effect(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 6;
    data >>= 3;
    *led = data as u8;
}

fn main() -> crossterm::Result<()> {
    let mut buf = [0u8; N_LEDS * 3];

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();
    let time_delay = Duration::from_millis(25);
    
    // let time_delay_update = Duration::from_millis(2000);

    //rand::thread_rng().fill_bytes(&mut buf);

    let (s, r) = bounded::<bool>(10);
    
    let id = thread::spawn(move || {
        'lus: loop {
            match r.try_recv() {
                Ok(_) => {
                    let col: u8 = rand::thread_rng().gen_range(0..=8);
                    // buf[1] = 0xAF;
                    // buf[0] = 0xFF;
                    // buf[2] = 0x0F;
        
                    let pixel = match col {
                        0 => &[255, 0, 0],
                        1 => &[0, 255, 0],
                        2 => &[0, 0, 255],
                        3 => &[255, 255, 0],
                        4 => &[255, 0, 255],
                        5 => &[0, 255, 255],
                        6 => &[0xAF, 255, 0xF],
                        7 => &[0xFF, 0xFF, 0xFF],
                        8 => &[0x1F, 0xFF, 0x1F],
                        _ => unreachable!("Error!"),
                    };

                    buf[0..=2].copy_from_slice(pixel);                    
                    buf[900-3..900].copy_from_slice(pixel);
                },
                Err(crossbeam_channel::TryRecvError::Disconnected) => break 'lus,
                Err(crossbeam_channel::TryRecvError::Empty) => (),
            }
    
            for i in (0..450-3).rev() {
                buf[i + 3] = buf[i]
            }
            for i in 450..900-3 {
                buf[i] = buf[i+3]
            }
    
            effect(&mut buf[0]);
            effect(&mut buf[1]);
            effect(&mut buf[2]);
    
            effect(&mut buf[900-3]);
            effect(&mut buf[900-2]);
            effect(&mut buf[900-1]);

            match port.write_bytes(buf.as_ref()) {
                Ok(_) => (),
                Err(e) => {
                    println!("Serial error {:?}", e);
                    break 'lus;
                }
            }
            thread::sleep(time_delay);
            
        }
        let _ = port.restore_program();      
    });

    enable_raw_mode()?;

    let mut update = Duration::from_secs(4);

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
                        KeyEvent { code, modifiers: KeyModifiers::NONE } => {
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
            }
        }
    }

    disable_raw_mode()?;

    drop(s);

    let _ = id.join();

    Ok(())
}
