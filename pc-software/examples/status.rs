use crossbeam_channel::bounded;

use byte::BytesExt;
use crossterm::event::KeyModifiers;
use crossterm::event::{poll, read, Event, KeyCode::Char, KeyEvent};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use ieee802154::mac::{
    Address, FooterMode, Frame, FrameSerDesContext, FrameType, PanId, ShortAddress,
};
use image::EncodableLayout;
use pc_software::UartLeds;
use psila_data::application_service::ApplicationServiceHeader;
use psila_data::pack::Pack;
use std::fs::File;
use std::io::{ErrorKind, Write};
use std::time::SystemTime;
use std::{thread, time::Duration};
use uart_protocol::Responce;

use psila_data::network::NetworkHeader;

fn hex_to_bytes(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 == 0 {
        (0..s.len())
            .step_by(2)
            .map(|i| {
                s.get(i..i + 2)
                    .and_then(|sub| u8::from_str_radix(sub, 16).ok())
            })
            .collect()
    } else {
        None
    }
}

// Generates data for wireshark Hex Import "IEEE 802.15.4 Wireless PAN with FCS not present"!
fn bytes_to_wireshark_hex(data: &[u8]) -> String {
    let mut hs = String::with_capacity(1024);

    for (addr, d) in data.iter().enumerate() {
        if addr & 0xF == 0 {
            hs.push_str(format!("{:04x}:\t ", addr).as_str());
        }
        hs.push_str(format!("{:02x} ", d).as_str());
        if addr & 0xF == 0xF {
            hs.push('\n');
        }
    }
    hs
}

fn print_frame(frame: &ApplicationServiceHeader) {
    print!(
        "APS {:?} {:?}",
        frame.control.frame_type, frame.control.delivery_mode,
    );
    if frame.control.security {
        print!(" Secure");
    }
    if frame.control.acknowledge_request {
        print!(" AckReq");
    }
    if frame.control.extended_header {
        print!(" ExtHdr");
    }
    if let Some(addr) = frame.destination {
        print!(" Dst {:02x}", addr);
    }
    if let Some(group) = frame.group {
        print!(" Group {:04x}", group);
    }
    if let Some(cluster) = frame.cluster {
        print!(" Cluster {:04x}", cluster);
    }
    if let Some(profile) = frame.profile {
        print!(" Profile {:04x}", profile);
    }
    if let Some(addr) = frame.source {
        print!(" Src {:02x}", addr);
    }
    println!(" Counter {:02x}", frame.counter);
}

fn main() -> crossterm::Result<()> {
    // let bytes = [1, 200, 81, 255, 255, 255, 255, 153, 136, 25, 2, 173, 254, 255, 249, 227, 180, 11, 0, 11, 0, 16, 94, 192, 17, 239, 0, 204, 92, 174, 50, 2, 147, 40];

    // let s = "01c8b4ffffffff99881902adfefff9e3b40b000b00105ec011da00acd926330293253a";

    // // let bytes = [2, 173, 254, 255, 249, 227, 180, 11, 0, 11, 0, 16, 94, 192, 17, 239, 0, 204, 92, 174, 50, 2, 147, 40];
    // let bytes = hex_to_bytes(s).unwrap();
    // let bytes = bytes.as_slice();

    // let frame = bytes.read_with::<Frame>(&mut 0, FooterMode::None);

    // println!("{:x?}", frame);

    // println!("{}", bytes_to_wireshark_hex(bytes));

    let path = "/tmp/ws";

    let mut f = File::create(path).unwrap();

    // write!(f, "{}\n\n", bytes_to_wireshark_hex(bytes)).is_ok();

    // return Ok(());

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();
    let time_delay = Duration::from_millis(25);

    let _ = port.restore_program();

    // let time_delay_update = Duration::from_millis(2000);

    //rand::thread_rng().fill_bytes(&mut buf);

    let (s, r) = bounded::<bool>(10);

    let id = thread::spawn(move || {
        'lus: loop {
            match r.try_recv() {
                Ok(_) => (),
                Err(crossbeam_channel::TryRecvError::Disconnected) => break 'lus,
                Err(crossbeam_channel::TryRecvError::Empty) => (),
            }

            match port.read_responce() {
                Ok(r) => match r {
                    Responce::RadioRecv(data) => {
                        let l = data[1] as usize;
                        let d = data[2..2 + l].as_bytes();
                        let hs = if data.len() > l {
                            bytes_to_wireshark_hex(d)
                        } else {
                            String::from("\n")
                        };
                        println!("\r\n{}", hs);
                        write!(f, "{}\n\n", hs).is_ok();
                        let frame = d.read_with::<Frame>(&mut 0, FooterMode::Explicit);
                        println!("\r\nDECODE: {:x?}", frame);

                        let payload = frame.unwrap().payload;

                        if let Ok((_nwk, used)) = NetworkHeader::unpack(&payload[..]) {
                            let payload = &payload[used..];
                            println!("\r\n Payload {:02x?}", payload);

                            if let Ok((aps, _used)) = ApplicationServiceHeader::unpack(&payload[..])
                            {
                                print_frame(&aps);
                            }
                        }
                    }
                    r => println!("r: {:?}", r),
                },

                Err(ref e) if e.kind() == ErrorKind::TimedOut => (),
                Err(ref e) if e.kind() == ErrorKind::BrokenPipe => break 'lus,
                Err(e) => {
                    println!("SE: {:?}", e);
                    break 'lus;
                }
            }
            thread::sleep(time_delay);
        }
        let _ = port.restore_program();
    });

    enable_raw_mode()?;

    let mut update = Duration::from_secs(4);

    let mut e_last = SystemTime::now();

    let mut e_vec = Vec::<Duration>::with_capacity(10);

    loop {
        if s.send(true).is_err() {
            break;
        };

        if poll(update)? {
            match read()? {
                Event::Key(event) => {
                    match event {
                        KeyEvent {
                            code,
                            modifiers: KeyModifiers::NONE,
                        } => {
                            // println!("CODE: {:?}", code);
                            match code {
                                Char(' ') => {
                                    let e_cur = SystemTime::now();
                                    let delta = e_cur.duration_since(e_last).unwrap();
                                    e_last = e_cur;
                                    if delta >= Duration::from_secs(3) {
                                        e_vec.clear();
                                        update = Duration::from_secs(4);
                                        println!("\rReset");
                                    } else {
                                        e_vec.push(delta);
                                        if e_vec.len() >= 2 {
                                            let mut total: usize = 0;
                                            for i in e_vec[1..].iter() {
                                                total += i.as_millis() as usize;
                                            }
                                            total /= e_vec.len();
                                            println!("\rBeat {} ms", total);
                                            update = Duration::from_millis(total as u64);
                                        }
                                    }
                                }
                                Char('q') => break,
                                _ => (),
                            }
                        }
                        event => println!("{:?}", event),
                    }
                }
                Event::Mouse(event) => println!("{:?}", event),
                Event::Resize(width, height) => println!("New size {}x{}", width, height),
            }
        }
    }

    disable_raw_mode()?;

    drop(s);

    let _ = id.join();

    Ok(())
}
