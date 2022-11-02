use pc_software::UartLeds;
use std::{thread, time::Duration};

#[allow(non_camel_case_types)]
enum State {
    R_UP,
    R_DOWN,
    G_UP,
    G_DOWN,
    B_UP,
    B_DOWN,
}

const N_LEDS: usize = 300;
const LEDMAX: u8 = 0x49;
const UPDATE: u8 = 2;

fn main() {
    let mut buf = [0x01_u8; N_LEDS * 3];

    let mut port = UartLeds::new(None).unwrap();

    let time_delay = Duration::from_millis(25);

    let mut rgb: [u8; 3] = [0, 0, LEDMAX];

    let mut state = State::R_UP;
    let mut tick = UPDATE;

    loop {
        if tick != UPDATE {
            tick += 1;
        } else {
            tick = 0;
            match state {
                State::R_UP => {
                    rgb[1] += 1;
                    if rgb[1] >= LEDMAX {
                        state = State::B_DOWN;
                    }
                }
                State::R_DOWN => {
                    rgb[1] -= 1;
                    if rgb[1] == 0 {
                        state = State::B_UP;
                    }
                }
                State::G_UP => {
                    rgb[0] += 1;
                    if rgb[0] >= LEDMAX {
                        state = State::R_DOWN;
                    }
                }
                State::G_DOWN => {
                    rgb[0] -= 1;
                    if rgb[0] == 0 {
                        state = State::R_UP;
                    }
                }
                State::B_UP => {
                    rgb[2] += 1;
                    if rgb[2] >= LEDMAX {
                        state = State::G_DOWN;
                    }
                }
                State::B_DOWN => {
                    rgb[2] -= 1;
                    if rgb[2] == 0 {
                        state = State::G_UP;
                    }
                }
            }
        }

        println!("{:?}", rgb);

        for i in (0..buf.len() - 3).rev() {
            buf[i + 3] = buf[i]
        }

        buf[0] = rgb[0];
        buf[1] = rgb[1];
        buf[2] = rgb[2];

        match port.write_bytes(&buf) {
            Ok(_) => (),
            Err(e) => {
                println!("Write: {:?}", e);
                return;
            }
        }

        // for _ in 0..3 {
        //     match port.read(serial_buf.as_mut()) {
        //         Ok(num) => match Responce::from_byte(serial_buf[0]) {
        //             None => println!("Got {}, {:?}", num, &serial_buf[0..num]),
        //             Some(res) => match res {
        //                 Responce::BufferReady => break,
        //                 Responce::Ok => break,
        //                 Responce::BufferFull => {
        //                     println!("BufferFull");

        //                     thread::sleep(time_delay);
        //                 }
        //                 e => {
        //                     println!("{:?}", e);
        //                     break;
        //                 }
        //             },
        //         },
        //         Err(e) => {
        //             println!("Read: {:?}", e);
        //             //return;
        //         }
        //     }
        //     thread::sleep(time_delay);
        // }
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
