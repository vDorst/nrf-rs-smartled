#![cfg_attr(not(all(test, target_arch = "x86_64")), no_std)]

pub const N_LEDS: usize = 300;
pub const N_BYTES: usize = 3;
pub const TOTAL_BYTES: usize = N_LEDS * N_BYTES;

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Clone, Copy)]
pub enum Responce {
    Ok = 0,
    BufferFull = 0xEF,
    BufferReady = 0xEE,
    TimeOut = 0xEC,
    None = 0xFF,
    Error = 0xE0,
    NotInSerialProgram = 0xE1,
}

impl Responce {
    pub fn from_byte(data: u8) -> Option<Self> {
        match data {
            0 => Some(Self::Ok),
            0xEF => Some(Self::BufferFull),
            0xEE => Some(Self::BufferReady),
            0xEC => Some(Self::TimeOut),
            0xFF => Some(Self::None),
            0xE0 => Some(Self::Error),
            0xE1 => Some(Self::NotInSerialProgram),
            _ => None,
        }
    }
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug, Copy, Clone)]
pub enum Programs {
    Serial = 0,
    One,
    Two,
}

impl Programs {
    pub fn from_bytes(data: u8) -> Self {
        match data {
            1 => Programs::One,
            2 => Programs::Two,
            _ => Programs::Serial,
        }
    }
}

#[derive(PartialEq, Eq, Debug)]
pub enum Commands<'a> {
    SetProgram(Programs),
    FRAME(&'a [u8]),
    GetProgram,
}

impl<'a> Commands<'a> {
    pub fn to_slice<'b>(&self, buf: &'b mut [u8]) -> Option<&'b mut [u8]> {
        let counter;
        match self {
            Commands::SetProgram(prg) => {
                counter = 2;
                if buf.len() < counter {
                    return None;
                }
                buf[0] = 0x00;
                buf[1] = *prg as u8;
            }
            Commands::FRAME(data) => {
                //let data = *data;
                let size = data.len();
                counter = 3 + size;
                if buf.len() < counter {
                    return None;
                }
                buf[0] = 0x01;
                buf[1] = (size & 0xFF) as u8;
                buf[2] = (size >> 8) as u8;
                buf[3..3 + size].copy_from_slice(&data[..]);
            }
            Commands::GetProgram => {
                counter = 1;
                if buf.len() < counter {
                    return None;
                }
                buf[0] = 0x02;
            }
        }

        Some(&mut buf[..counter])
    }

    pub fn from_bytes(data: &'a mut [u8]) -> Option<Self> {
        let size = data.len();
        match data[0] {
            0 => {
                if size < 2 {
                    return None;
                }
                let prg = match data[1] {
                    0 => Programs::Serial,
                    1 => Programs::One,
                    2 => Programs::Two,
                    _ => return None,
                };
                Some(Self::SetProgram(prg))
            }
            1 => {
                if size < 4 {
                    return None;
                }
                let data_size = data[1] as usize + ((data[2] as usize) << 8);
                if size < (3 + data_size) {
                    return None;
                }
                return Some(Self::FRAME(&data[3..3 + data_size]));
            }
            2 => {
                if size < 1 {
                    return None;
                }
                Some(Self::GetProgram)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Commands;
    use crate::{Programs, TOTAL_BYTES};

    // #[test]
    // fn it_works() {
    //     let cmd = Programs::One;
    //     let bla = bincode::serialize::<Programs>(&cmd);
    //     println!("ser: Program = {:?}", bla);

    //     let data = [1u8, 0, 0, 0];
    //     let bla = bincode::deserialize::<Programs>(&data[..]);
    //     println!("de: Program = {:?}", bla);
    // }

    #[test]
    fn postcard_commands_setprogram() {
        let mut buf: [u8; 1024] = [0; 1024];
        let frame = Commands::SetProgram(Programs::One);
        let bla = frame.to_slice(&mut buf).unwrap();
        assert_eq!(bla, [0, 1]);

        let frame = Commands::SetProgram(Programs::Serial);
        let bla = frame.to_slice(&mut buf).unwrap();
        assert_eq!(bla, [0, 0]);

        let frame = Commands::SetProgram(Programs::Two);
        let bla = frame.to_slice(&mut buf).unwrap();
        assert_eq!(bla, [0, 2]);
    }

    #[test]
    fn postcard_commands_frame() {
        let mut buf: [u8; 1024] = [0; 1024];

        let frame = Commands::FRAME(&[0xAA; TOTAL_BYTES]);
        let bla = frame.to_slice(&mut buf).unwrap();
        // println!("ser: ({}) Frame = {:?}", bla.len(), bla);

        let data = [
            1u8, 132, 3, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170, 170,
            170,
        ];

        assert!(bla.len() == data.len());

        let mut value = 0u8;

        for i in 0..bla.len() {
            value |= bla[i] ^ data[i];
        }
        assert!(value == 0);

        let de_frame = Commands::from_bytes(bla).unwrap();

        assert_eq!(frame, de_frame);
    }
}
