use colorsys::{ColorTransform, Hsl, Rgb};
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

    let rgb = Rgb::new(12.0, 0.0, 0.0, None);
    let mut hsl = Hsl::from(&rgb);
    let mut hue = 0.0;

    //hsl.set_saturation(100.0);

    while !term.load(Ordering::Relaxed) {
        hue += 0.01;
        hue %= 360.0;

        hsl.adjust_hue(hue);

        let b: [u8; 3] = Rgb::from(&hsl).into();

        for i in (0..buf.len() - 3).rev() {
            buf[i + 3] = buf[i]
        }

        buf[0] = b[1];
        buf[1] = b[0];
        buf[2] = b[2];

        packet_cnt += 1;

        match port.write_bytes(&buf) {
            Ok(_) => {
                println!("{}: accepted", packet_cnt)
            }
            Err(e) => {
                println!("{}: e {:?}", packet_cnt, e);
            }
        }
        thread::sleep(time_delay);
    }

    let _ = port.restore_program();
}
