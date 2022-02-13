
use serialport::{self, SerialPort};
pub use uart_protocol::Programs;
use uart_protocol::Commands;

use std::{time::Duration, io::{Error, ErrorKind}};

pub struct UartLeds {
    ser_dev: Box<dyn SerialPort>,
    ser_buf: [u8; 1024],
}

impl UartLeds {
    pub fn new(port: &str) -> Result<Self, String> {
        let ser_dev = match serialport::new(port, 115_200)
            .timeout(Duration::from_millis(100))
            .open() {
                Err(e) => return Err(format!("{:?}", e)),
                Ok(dev) => dev,
            };
        let mut s =  Self {
            ser_dev,
            ser_buf: [0; 1024],
        };

        if let Err(e) = s.set_program(Programs::Serial) {
            return Err(format!("{:?}", e));
        }

        Ok(s)
    }

    pub fn set_program(&mut self, prg: Programs) -> Result<(), Error> {
        let command = Commands::SetProgram(prg);

        self.write_command(command)
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let command = Commands::FRAME(bytes);

        self.write_command(command)
    }


    pub fn write_command(&mut self, command: Commands) -> Result<(), Error> {
        let data = match command.to_slice(self.ser_buf.as_mut()) {
            None => return Err(Error::new(ErrorKind::Other, "can't convert command to slice!")),
            Some(data) => data,
        };
        match self.ser_dev.write(data) {
            Ok(_num) => Ok(()),
            Err(e) => Err(e),
        }
    }
}
