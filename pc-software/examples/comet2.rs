// use crossterm;
use rand::{self, Rng};
use std::{thread, time::Duration};
use pc_software::UartLeds;

const N_LEDS: usize = 300;

fn effect(led: &mut u8) {
    let mut data: u16 = *led as u16;
    data *= 10;
    data >>= 4;
    *led = data as u8;
}

const BUF_LEN: usize = N_LEDS * 3;

fn main() {
    let mut buf = [0x01; BUF_LEN];

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();

    let time_delay = Duration::from_millis(30);

    //rand::thread_rng().fill_bytes(&mut buf);

    loop {
        let col: u8 = rand::thread_rng().gen_range(0..=8);
        // buf[1] = 0xAF;
        // buf[0] = 0xFF;
        // buf[2] = 0x0F;

        buf[BUF_LEN-3..BUF_LEN].copy_from_slice(match col {
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

        for _ in 0..BUF_LEN / 10 {

            for i in 3..BUF_LEN {
                buf[i-3] = buf[i]
            }

            effect(&mut buf[BUF_LEN-3]);
            effect(&mut buf[BUF_LEN-2]);
            effect(&mut buf[BUF_LEN-1]);

            match port.write_bytes(&buf) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return;
                }
            }
            thread::sleep(time_delay);
        }
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
