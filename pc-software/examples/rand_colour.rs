use rand::{self, Rng};
use signal_hook::consts::SIGINT;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::{thread, time::Duration};

use pc_software::UartLeds;

const N_LEDS: usize = 300;
const BUF_LEN: usize = N_LEDS * 3;

fn effect(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 0x40;
    data >>= 8;
    *led = data as u8;
}

fn dim(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 15;
    data >>= 4;
    *led = data as u8;
}

fn main() {
    let term = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(SIGINT, Arc::clone(&term)).unwrap();
    let mut buf = [01u8; BUF_LEN];

    let mut port = UartLeds::new(None).unwrap();

    let time_delay = Duration::from_millis(1000);

    'lus: loop {
        let col: u8 = rand::thread_rng().gen_range(0..=8);

        buf[0..=2].copy_from_slice(match col {
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
        });

        effect(&mut buf[0]);
        effect(&mut buf[1]);
        effect(&mut buf[2]);

        for i in (6..BUF_LEN).step_by(6) {
            buf[i] = buf[0];
            buf[i + 1] = buf[1];
            buf[i + 2] = buf[2];

            match port.write_bytes(&buf) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return;
                }
            }
            if term.load(Ordering::Relaxed) {
                break 'lus;
            }
            thread::sleep(Duration::from_millis(30));
        }

        for i in (3..BUF_LEN).step_by(6).rev() {
            buf[i] = buf[0];
            buf[i + 1] = buf[1];
            buf[i + 2] = buf[2];

            match port.write_bytes(&buf) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return;
                }
            }
            if term.load(Ordering::Relaxed) {
                break 'lus;
            }
            thread::sleep(Duration::from_millis(30));
        }

        thread::sleep(time_delay);

        for _ in 0..32 {
            for b in buf.iter_mut() {
                dim(b);
            }

            match port.write_bytes(&buf) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return;
                }
            }
            if term.load(Ordering::Relaxed) {
                break 'lus;
            }
            thread::sleep(Duration::from_millis(30));
        }
    }

    let _ = port.restore_program();
}
