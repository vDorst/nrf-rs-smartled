use pc_software::UartLeds;
use rand::{self, prelude::SliceRandom, Rng};
use signal_hook::consts::SIGINT;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::{thread, time::Duration};

const N_LEDS: usize = 64;
const BUF_LEN: usize = N_LEDS * 3;

fn effect(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 0x40;
    data >>= 8;
    *led = data as u8;
}

fn main() {
    let term = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(SIGINT, Arc::clone(&term)).unwrap();

    let mut buf = [01u8; BUF_LEN];

    let mut col_val = [0_u8; 3];

    let mut leds = Vec::<u16>::with_capacity(N_LEDS);
    for led in 0..N_LEDS as u16 {
        leds.push(led);
    }

    let mut rng = rand::thread_rng();

    let mut port = UartLeds::new("/dev/ttyACM1").unwrap();

    let time_delay = Duration::from_millis(25);

    'lus: loop {
        let col: u8 = rand::thread_rng().gen_range(0..=14);

        col_val.copy_from_slice(match col {
            0 => &[255, 0, 0],
            1 => &[0, 255, 0],
            2 => &[0, 0, 255],
            3 => &[255, 255, 0],
            4 => &[255, 0, 255],
            5 => &[0, 255, 255],
            6 => &[0xAF, 255, 0xF],
            7 => &[0xFF, 0xFF, 0xFF],
            8 => &[0x1F, 0xFF, 0x1F],
            9 => &[255, 0x80, 0],
            10 => &[0, 0x80, 255],
            11 => &[0x80, 255, 0],
            12 => &[0x80, 0, 255],
            13 => &[0, 255, 0x80],
            14 => &[255, 0, 0x80],
            _ => unreachable!("Error!"),
        });

        effect(&mut col_val[0]);
        effect(&mut col_val[1]);
        effect(&mut col_val[2]);

        leds.shuffle(&mut rng);

        let mut count = 0;

        for i in leds.iter() {
            let pos = *i as usize * 3;
            buf[pos] = col_val[0];
            buf[pos + 1] = col_val[1];
            buf[pos + 2] = col_val[2];

            if count == 0 {
                count = 0;
                match port.write_bytes(&buf) {
                    Ok(_) => (),
                    Err(e) => {
                        println!("Write: {:?}", e);
                        return;
                    }
                }
            } else {
                count += 1;
            }

            if term.load(Ordering::Relaxed) {
                break 'lus;
            }
            thread::sleep(time_delay);
        }

        thread::sleep(Duration::from_millis(1000));
    }

    let _ = port.restore_program();
}
