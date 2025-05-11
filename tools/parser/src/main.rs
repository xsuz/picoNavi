use std::fs;
use std::io::{BufReader,Read,BufWriter,Write};
use std::env;

mod parser;

pub fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        println!("Usage: {} <log_file>", args[0]);
        return;
    }
    let log_file = &args[1];
    println!("Reading log file: {}", log_file);
    let f = fs::File::open(log_file).unwrap();
    let mut reader = BufReader::new(f);
    let mut buf = vec![0u8; 4096 as usize];
    let mut parser = parser::Parser::new();
    loop {
        if let Ok(read) = reader.read(&mut buf) {
            if read == 0 {
                break;
            }
            parser.parse(&buf[0..read]);
        } else {
            break;
        }
    }
    {
        let f = fs::File::create("imu.csv").unwrap();
        let mut writer = BufWriter::new(f);
        writeln!(writer, "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z").unwrap();
        for (data,timestamp) in &parser.db.imu.data {
            writeln!(writer, "{},{},{},{},{},{},{}", timestamp, data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z).unwrap();
        }
        writer.flush().unwrap();
    }
    {
        let f = fs::File::create("gps.csv").unwrap();
        let mut writer = BufWriter::new(f);
        writeln!(writer, "timestamp,latitude,longitude,altitude,east_velocity,north_velocity").unwrap();
        for (data,timestamp) in &parser.db.gps.data {
            writeln!(writer, "{},{},{},{},{},{}", timestamp, data.latitude, data.longitude, data.altitude, data.east_velocity, data.north_velocity).unwrap();
        }
        writer.flush().unwrap();
    }
}