#![deny(clippy::pedantic)]
#![allow(clippy::missing_errors_doc)]

use dotenv::dotenv;
use serialport::{self, SerialPort};
pub use uart_protocol::Programs;
use uart_protocol::{Commands, Responce};

use std::{
    io::{Error, ErrorKind},
    time::Duration,
};

pub struct UartLeds {
    ser_dev: Box<dyn SerialPort>,
    ser_buf: [u8; 1024],
}

impl UartLeds {
    /// Creata a new `UartLeds`.
    ///
    /// # Errors
    ///
    /// String with the fault
    pub fn new(port: Option<&str>) -> Result<Self, String> {
        let port = match port {
            Some(port) => port.to_owned(),
            None => match dotenv() {
                Ok(_) => {
                    if let Ok(port) = dotenv::var("PORT") {
                        port
                    } else {
                        return Err("env PORT not set!".to_owned());
                    }
                }
                Err(e) => return Err(format!("Error {e}")),
            },
        };
        let ser_dev = match serialport::new(port, 115_200)
            .timeout(Duration::from_millis(100))
            .open()
        {
            Err(e) => return Err(format!("{:?}", e)),
            Ok(dev) => dev,
        };
        let mut s = Self {
            ser_dev,
            ser_buf: [0; 1024],
        };

        if let Err(e) = s.set_program(Programs::Serial) {
            return Err(format!("{:?}", e));
        }

        Ok(s)
    }

    /// `set_program` mode in the uC software.
    ///
    /// # Errors
    ///
    /// String with the fault
    pub fn set_program(&mut self, prg: Programs) -> Result<Responce, Error> {
        let command = Commands::SetProgram(prg);

        self.write_command(&command)
    }

    /// `write_bytes` to the uC software.
    ///
    /// # Errors
    ///
    /// String with the fault
    pub fn write_bytes(&mut self, bytes: &[u8]) -> Result<Responce, Error> {
        let command = Commands::LedData(bytes);

        self.write_command(&command)
    }

    /// `write_command` to the uC software.
    ///
    /// # Errors
    ///
    /// String with the fault
    pub fn write_command(&mut self, command: &Commands) -> Result<Responce, Error> {
        let data = match command.to_slice(self.ser_buf.as_mut()) {
            None => {
                return Err(Error::new(
                    ErrorKind::Other,
                    "can't convert command to slice!",
                ))
            }
            Some(data) => data,
        };
        // println!("\tSend data {:?}", data);

        match self.ser_dev.write(data) {
            Ok(_num) => self.read_responce(),
            Err(e) => Err(e),
        }
    }

    pub fn restore_program(&mut self) -> Result<Responce, Error> {
        let command = Commands::SetProgram(Programs::Two);

        self.write_command(&command)
    }

    pub fn read_responce(&mut self) -> Result<Responce, Error> {
        match self.ser_dev.read(self.ser_buf.as_mut()) {
            Ok(0) => Err(Error::new(ErrorKind::BrokenPipe, "Serial port gone!")),
            Ok(count) => match Responce::from_bytes(&mut self.ser_buf[..count]) {
                Some(r) => Ok(r),
                None => Err(Error::new(ErrorKind::Other, "Responce::from_bytes")),
            },
            Err(e) => Err(e),
        }
    }
}
