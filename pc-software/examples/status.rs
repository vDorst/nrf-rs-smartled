use crossbeam_channel::bounded;

//use byte::BytesExt;
use crossterm::event::KeyModifiers;
use crossterm::event::{poll, read, Event, KeyCode::Char, KeyEvent};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
// use ieee802154::mac::{
//     Address, FooterMode, Frame, FrameSerDesContext, FrameType, PanId, ShortAddress,
// };
use image::EncodableLayout;
use pc_software::UartLeds;
use psila_data::{ExtendedAddress, Key};
//use psila_data::application_service::header::FrameControl;
use psila_data::application_service::{ApplicationServiceHeader};
use psila_data::cluster_library::{AttributeDataType};
//use psila_data::pack::Pack;
//use psila_data::security::CryptoProvider;
use psila_service::{PsilaService, ClusterLibraryHandler};
use std::fs::File;
use std::io::{ErrorKind, Write};
use std::time::{SystemTime, Duration};
use std::{thread};
use uart_protocol::Responce;

// use psila_data::network::{NetworkHeader, };

// use psila_crypto::CryptoBackend;
use psila_crypto_rust_crypto::RustCryptoBackend;
use bbqueue::BBBuffer;

use std::thread::sleep;


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

    hs.push('\n');
    hs.push('\r');
    
    for (addr, d) in data.iter().enumerate() {
        if addr & 0xF == 0 {
            hs.push_str(format!("{:04x}:\t ", addr).as_str());
        }
        hs.push_str(format!("{:02x} ", d).as_str());
        if addr & 0xF == 0xF {
            hs.push('\r');
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

struct Device {
    profile: u16,
    cluster: u16,
    endpoint: u8,
    command: u8,
    arguments: [u8; 20],
    data_type: AttributeDataType,
}

impl Device {
    pub fn new() -> Self {
        Self {
            profile: 0x00,
            cluster: 0x000,
            endpoint: 0x00,
            command: 0x00,
            arguments: [0; 20],
            data_type: AttributeDataType::Boolean,
        }
    }
}

impl ClusterLibraryHandler for Device  {
    fn active_endpoints(&self) -> &[u8] {
        todo!()
    }

    fn get_simple_desciptor(&self, endpoint: u8) -> Option<psila_data::device_profile::SimpleDescriptor> {
        todo!()
    }

    fn read_attribute(
        &self,
        profile: u16,
        cluster: u16,
        endpoint: u8,
        attribute: u16,
        value: &mut [u8],
    ) -> Result<(psila_data::cluster_library::AttributeDataType, usize), psila_data::cluster_library::ClusterLibraryStatus> {
        todo!()
    }

    fn write_attribute(
        &mut self,
        profile: u16,
        cluster: u16,
        endpoint: u8,
        attribute: u16,
        data_type: psila_data::cluster_library::AttributeDataType,
        value: &[u8],
    ) -> Result<(), psila_data::cluster_library::ClusterLibraryStatus> {
        todo!()
    }

    fn run(
        &mut self,
        profile: u16,
        cluster: u16,
        endpoint: u8,
        command: u8,
        arguments: &[u8],
    ) -> Result<(), psila_data::cluster_library::ClusterLibraryStatus> {
        todo!()
    }
    // 
}

enum ThreadAction {
    Beat,
    RadioBeacon,
}

fn main() -> crossterm::Result<()> {
    let crypto= RustCryptoBackend::default();
    let mut packet = Vec::<u8>::with_capacity(127);

    static bbq: BBBuffer<1024> = BBBuffer::<1024>::new();
    
    let (tx_queue, mut rx_queue) = bbq.try_split().unwrap();
    
    let address = ExtendedAddress::new(0x1234_1234_1234_1234);
    let default_link_key = Key::from([
        0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5, 0x96, 0x87, 0x78, 0x69, 0x5a, 0x4b, 0x3c, 0x2d,
        0x1e, 0x0f,
    ]);

    let cluser_library_handler = Device::new();

    let mut ps = PsilaService::new(crypto, tx_queue, address, default_link_key, cluser_library_handler);

    // let path = "/tmp/ws";

    // let mut f = File::open(path).unwrap();

    let mut port = UartLeds::new("/dev/ttyACM0").unwrap();
    let time_delay = Duration::from_millis(50);

    let _ = port.restore_program();

    // let time_delay_update = Duration::from_millis(2000);

    //rand::thread_rng().fill_bytes(&mut buf);

    // 12cf60ba4245422320c1cbf77101f715653a22b20a5d4375f60801c1b1f5a8235e0000

    let (s, r) = bounded::<ThreadAction>(10);

    let mut timestamp: u32 = 0x1233;

    let id = thread::spawn(move || {
        'lus: loop {
            match r.try_recv() {
                Ok(ThreadAction::Beat) => (),
                Ok(ThreadAction::RadioBeacon) => packet.extend([3, 8, 0xbe, 255, 255, 255, 255, 7]),
                //Ok(ThreadAction::RadioBeacon) => packet.extend([3, 8, 0xbe, 255, 255, 255, 255, 7, 0x1e, 0x4b]),
                Err(crossbeam_channel::TryRecvError::Disconnected) => break 'lus,
                Err(crossbeam_channel::TryRecvError::Empty) => (),
            }

            timestamp += 0x0100;

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

                        println!("\r\tRECV: {:?}: {}", ps.receive(timestamp, d), hs);

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
            

            if let  Ok( data) =  rx_queue.read() {
                println!("\r\tTX: {:?} {:?}", port.write_command(uart_protocol::Commands::RadioSend(&data[1..])), data);
                      
                // println!("{:?}", bytes_to_wireshark_hex(&data.as_bytes()));
                let l = data.len();
                data.release(l );
            } else { 
                let _ = ps.update(timestamp);
                if packet.len() != 0 {
                    let data = packet.as_bytes();
                    println!("\r\tTX: {:?} {:?}", port.write_command(uart_protocol::Commands::RadioSend(&data)), data);
                    packet.clear();
                }
            }
        }
        let _ = port.restore_program();
    });

    enable_raw_mode()?;

    let mut update = Duration::from_secs(4);

    let mut e_last = SystemTime::now();

    let mut e_vec = Vec::<Duration>::with_capacity(10);

    loop {
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
                                    if s.send(ThreadAction::Beat).is_err() {
                                        break;
                                    }
                                }
                                Char('b') => if s.send(ThreadAction::RadioBeacon).is_err() {
                                    break;
                                },
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
