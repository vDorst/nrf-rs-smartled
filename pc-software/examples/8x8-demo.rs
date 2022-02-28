use font8x8::legacy::BASIC_LEGACY;
use image::{self, Pixel, Rgb};
use pc_software::UartLeds;
use rand::{self, RngCore};
use std::{thread, time::Duration};

struct AppArgs {
    bg: u32,
    fg: u32,
    file: Option<std::path::PathBuf>,
    text: Option<String>,
    fire: bool,
    speed: u32,
}

const N_LEDS: usize = 300;

fn parse_args() -> Result<AppArgs, pico_args::Error> {
    let mut pargs = pico_args::Arguments::from_env();
    let args = AppArgs {
        // Parses a required value that implements `FromStr`.
        // Returns an error if not present.
        bg: pargs
            .opt_value_from_fn("--bg", parse_width)?
            .unwrap_or(0x010101),
        // Parses an optional value that implements `FromStr`.
        fg: pargs
            .opt_value_from_fn("--fg", parse_width)?
            .unwrap_or(0x001000),
        // Parses an optional value from `&str` using a specified function.
        text: pargs.opt_value_from_str("--text")?,
        // Parses an optional value from `&OsStr` using a specified function.
        file: pargs.opt_value_from_os_str("--photo", parse_path)?,
        speed: pargs
            .opt_value_from_fn("--speed", parse_width)?
            .unwrap_or(80),
        fire: pargs.contains(["-f", "--fire"]),
    };
    Ok(args)
}

fn parse_width(s: &str) -> Result<u32, &'static str> {
    if s.starts_with("0x") {
        let raw = s.trim_start_matches("0x");
        u32::from_str_radix(raw, 16)
    } else {
        s.parse()
    }
    .map_err(|_| "not a number")
}

fn parse_path(s: &std::ffi::OsStr) -> Result<std::path::PathBuf, &'static str> {
    Ok(s.into())
}

fn main() {
    let pargs = parse_args().expect("Unable to parse options!");

    let mut image = if let Some(txt) = pargs.text {
        let size_x = (txt.len() as u32) * 8;
        let mut img = image::RgbImage::new(size_x, 8);

        //println!("Colour BG {:x} FG {:x}", pargs.bg, pargs.fg);

        for (pos, byte) in txt.as_bytes().iter().enumerate() {
            let char = *byte as usize;
            if let Some(glyph) = BASIC_LEGACY.get(char) {
                for (y, bits) in glyph.iter().enumerate() {
                    for x in 0..8 {
                        let pos_x = (pos * 8) + x;
                        let color = match *bits & 1 << x != 0 {
                            false => pargs.bg,
                            true => pargs.fg,
                        }
                        .to_le_bytes();
                        let r = color[2];
                        let g = color[1];
                        let b = color[0];
                        let pixel = Rgb([r, g, b]);
                        img.put_pixel(pos_x as u32, y as u32, pixel);
                    }
                }
            }
        }

        img
    } else if let Some(filename) = pargs.file {
        let imgo = image::open(filename).expect("Unable to open your file!");
        imgo.into_rgb8()
    } else if pargs.fire {
        image::RgbImage::new(8, 8)
    } else {
        panic!("Please provide --photo, --text or --fire");
    };

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();

    let mut buf = [0x1Fu8; N_LEDS * 3];

    let time_delay = Duration::from_millis(pargs.speed as u64);

    let (size_x, size_y) = image.dimensions();

    if !pargs.fire {
        for start_x in 0..size_x - 7 {
            let mut p = 0;
            for y in 0..8 {
                for x in start_x..start_x + 8 {
                    let pixel = image.get_pixel(x, y);
                    let pixel = pixel.channels();
                    buf[p] = pixel[1];
                    buf[p + 1] = pixel[0];
                    buf[p + 2] = pixel[2];
                    p += 3;
                }
            }

            match port.write_bytes(&buf) {
                Ok(_) => (),
                Err(e) => {
                    println!("Write: {:?}", e);
                    return;
                }
            }
            thread::sleep(time_delay);
        }
    } else {
        let mut data = vec![0u8; (size_x * 2) as usize];
        let mut data = data.as_mut_slice();

        loop {
            for x in 0..size_x {
                for y in 0..size_y - 1 {
                    // let p = if x == 0 {
                    //     let p = image.get_pixel(x, y + 1).to_rgb();
                    //     let p2 = image.get_pixel(x + 1, y + 1).to_rgb();
                    //     let b = p.channels();
                    //     let b2 = p2.channels();
                    //     Rgb([((b[0] >> 1) + (b2[0] >> 1)) >> 2, b[1] >> 2, b[2]])
                    // } else if x == (size_x - 1) {
                    //     let p = image.get_pixel(x, y + 1).to_rgb();
                    //     let p2 = image.get_pixel(x - 1, y + 1).to_rgb();
                    //     let b = p.channels();
                    //     let b2 = p2.channels();
                    //     Rgb([((b[0] >> 1) + (b2[0] >> 1)) >> 2, b[1] >> 2, b[2]])
                    // } else {
                    //     let p = image.get_pixel(x, y + 1).to_rgb();
                    //     let p2 = image.get_pixel(x + 1, y + 1).to_rgb();
                    //     let p3 = image.get_pixel(x - 1, y + 1).to_rgb();
                    //     let b = p.channels();
                    //     let b2 = p2.channels();
                    //     let b3 = p3.channels();
                    //     let g = (b[0] as u16 + b2[0] as u16 + b2[0] as u16 + b3[0] as u16) / 9;
                    //     Rgb([g as u8, b[1] >> 2, b[2]])
                    // };
                    let p = {
                        let p = image.get_pixel(x, y + 1).to_rgb();
                        let b = p.channels();
                        Rgb([b[0] >> 1, b[1] >> 2, b[2]])
                    };
                    image.put_pixel(x, y, p);
                }
            }

            rand::thread_rng().fill_bytes(&mut data);
            for x in 0..image.width() {
                // let r = (data[x as usize] >> 3 << 3) & 0x1f;
                let r = data[x as usize];
                let g = data[(size_x + x) as usize];
                let g = if g > r { r >> 1 } else { g };
                let pixel = Rgb([r, g >> 2, 0]);
                image.put_pixel(x, size_y - 1, pixel);
            }

            let mut p = 0;
            for y in 0..size_y {
                for x in 0..size_x {
                    let pixel = image.get_pixel(x, y);
                    let pixel = pixel.channels();
                    buf[p] = pixel[1];
                    buf[p + 1] = pixel[0];
                    buf[p + 2] = pixel[2];
                    p += 3;
                }
            }

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
