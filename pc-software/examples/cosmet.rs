use pc_software::UartLeds;
use rand::{self, Rng};
use signal_hook::consts::SIGINT;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::{thread, time::Duration};

const N_LEDS: usize = 300;

fn effect(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 7;
    data >>= 3;
    *led = data as u8;
}

fn main() {
    let term = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(SIGINT, Arc::clone(&term)).unwrap();

    let mut buf = [0u8; N_LEDS * 3];

    let mut port = UartLeds::new(None).unwrap();
    let time_delay = Duration::from_millis(20);

    //rand::thread_rng().fill_bytes(&mut buf);

    'lus: loop {
        let col: u8 = rand::thread_rng().gen_range(0..=8);
        // buf[1] = 0xAF;
        // buf[0] = 0xFF;
        // buf[2] = 0x0F;

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

        for _ in 0..buf.len() / 6 {
            for i in (0..buf.len() - 3).rev() {
                buf[i + 3] = buf[i]
            }

            effect(&mut buf[0]);
            effect(&mut buf[1]);
            effect(&mut buf[2]);

            match port.write_bytes(buf.as_ref()) {
                Ok(_) => (),
                Err(e) => {
                    println!("Serial error {:?}", e);
                    break 'lus;
                }
            }
            if term.load(Ordering::Relaxed) {
                break 'lus;
            }
            thread::sleep(time_delay);
        }
    }

    let _ = port.restore_program();
}
