// use crossterm;
use image::{self, GenericImageView, Pixel};
use serialport;
use std::{thread, time::Duration};

fn main() {
    let mut port = serialport::new("/dev/ttyACM0", 115_200)
        .timeout(Duration::from_millis(1000))
        .open()
        .expect("Failed to open port");

    let mut serial_buf = [0x1Fu8; 64 * 3];

    //port.write(serial_buf.as_slice()).expect("Write failed!");

    let imgo = image::open("../nrf52-blinky/banner2.png").unwrap();
    let img = imgo.as_rgb8().unwrap();
    let (size_x, size_y) = img.dimensions();
    println!("dimensions {}x{}", size_x, size_y);

    let time_delay = Duration::from_millis(100);

    for start_x in 0..size_x - 7 {
        let mut p = 0;
        for y in 0..8 {
            for x in start_x..start_x + 8 {
                let pixel = imgo.get_pixel(x, y);
                let pixel = pixel.channels();
                serial_buf[p] = pixel[1];
                serial_buf[p + 1] = pixel[0];
                serial_buf[p + 2] = pixel[2];
                p += 3;
            }
        }
        for buf in serial_buf.iter_mut() {
            let mut p = *buf as u32;
            p = p * 0x3F;
            p >>= 8;
            *buf = p as u8;
        }

        // for n in (0..serial_buf.len()).step_by(3) {
        //     let r = serial_buf[n];
        //     let g = serial_buf[n + 1];
        //     serial_buf[n] = g;
        //     serial_buf[n + 1] = r;
        // }

        match port.write(serial_buf.as_ref()) {
            Ok(_) => (),
            Err(e) => println!("e {:?}", e),
        }
        thread::sleep(time_delay);
    }
}
