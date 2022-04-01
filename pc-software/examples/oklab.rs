use oklab::{oklab_to_srgb, srgb_to_oklab, Oklab, RGB};
use signal_hook::consts::SIGINT;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::{thread, time::Duration};

use pc_software::UartLeds;

const N_LEDS: usize = 300;

fn main() {
    let term = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(SIGINT, Arc::clone(&term)).unwrap();

    let mut buf = [01u8; N_LEDS * 3];

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();

    let time_delay = Duration::from_millis(22);

    let mut packet_cnt = 0;

    //rand::thread_rng().fill_bytes(&mut buf);

    let mut o = Oklab {
        l: 0.0,
        a: 0.0,
        b: 0.0,
    };

    println!("{:?}", o);

    let mut rad = 0.0;

    while !term.load(Ordering::Relaxed) {
        for i in (0..buf.len() - 3).rev() {
            buf[i + 3] = buf[i]
        }

        rad += std::f32::consts::TAU / (10 * N_LEDS) as f32;

        if rad >= std::f32::consts::TAU {
            rad = 0.0;
        }

        (o.a, o.b) = rad.sin_cos();

        let rgb = oklab_to_srgb(o);

        println!("{:.4?} - {:02x?}", o, rgb);

        buf[0] = rgb.g;
        buf[1] = rgb.r;
        buf[2] = rgb.b;

        packet_cnt += 1;

        match port.write_bytes(&buf) {
            Ok(_) => {
                // println!("{}: accepted", packet_cnt)
            }
            Err(e) => {
                println!("{}: e {:?}", packet_cnt, e);
            }
        }
        thread::sleep(time_delay);
    }

    let _ = port.restore_program();
}
