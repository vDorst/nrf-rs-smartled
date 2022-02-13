// use crossterm;
use colorsys::{ColorTransform, Hsl, Rgb};
//use rand::{self, RngCore};
use std::{thread, time::Duration};

use pc_software::UartLeds;

const N_LEDS: usize = 300;

fn main() {
    let mut buf = [01u8; N_LEDS * 3];

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();

    let time_delay = Duration::from_millis(30);

    //rand::thread_rng().fill_bytes(&mut buf);

    let rgb = Rgb::new(127.0, 0.0, 0.0, None);
    let mut hsl = Hsl::from(&rgb);
    let mut hue = 0.0;

    //hsl.set_saturation(100.0);

    loop {
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

        match port.write_bytes(&buf) {
            Ok(_) => (),
            Err(e) => println!("e {:?}", e),
        }
        thread::sleep(time_delay);
    }
}

// fn shift_up<P, Container>(buf: &mut ImageBuffer<Rgb<u8>, Container>)
// where
//     P: Pixel + 'static,
//     P::Subpixel: 'static,

//     Container: Deref<Target = [P::Subpixel]> + DerefMut,
// {
//     let (size_x, size_y) = buf.dimensions();

// }
