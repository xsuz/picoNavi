use std::fs;
use std::io::{BufReader,Read};
use std::env;

use polars::prelude::*;

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
        let imu_data = parser.get_imu_data();

        let imu_timestamp = imu_data.iter().map(|(_, t)| *t).collect::<Vec<i64>>();
        let imu_ax = imu_data.iter().map(|(d, _)| d.accel_x).collect::<Vec<f32>>();
        let imu_ay = imu_data.iter().map(|(d, _)| d.accel_y).collect::<Vec<f32>>();
        let imu_az = imu_data.iter().map(|(d, _)| d.accel_z).collect::<Vec<f32>>();
        let imu_gx = imu_data.iter().map(|(d, _)| d.gyro_x). collect::<Vec<f32>>();
        let imu_gy = imu_data.iter().map(|(d, _)| d.gyro_y). collect::<Vec<f32>>();
        let imu_gz = imu_data.iter().map(|(d, _)| d.gyro_z). collect::<Vec<f32>>();

        let df_imu = df!(
            "timestamp" => imu_timestamp,
            "accel_x" => imu_ax,
            "accel_y" => imu_ay,
            "accel_z" => imu_az,
            "gyro_x" => imu_gx,
            "gyro_y" => imu_gy,
            "gyro_z" => imu_gz
        ).unwrap();

        println!("IMU Data: {:?}", df_imu);

        CsvWriter::new(std::fs::File::create("imu_data.csv").unwrap())
            .finish(&mut df_imu.clone().lazy().collect().unwrap())
            .unwrap();
    }
    {
        let gps_data = parser.get_gps_data();

        let gps_timestamp = gps_data.iter().map(|(_, t)| *t).collect::<Vec<i64>>();
        let gps_lat = gps_data.iter().map(|(d, _)| d.latitude).collect::<Vec<f64>>();
        let gps_lon = gps_data.iter().map(|(d, _)| d.longitude).collect::<Vec<f64>>();
        let gps_alt = gps_data.iter().map(|(d, _)| d.altitude).collect::<Vec<f32>>();
        let gps_velocity_east = gps_data.iter().map(|(d, _)| d.east_velocity).collect::<Vec<f32>>();
        let gps_velocity_north = gps_data.iter().map(|(d, _)| d.north_velocity).collect::<Vec<f32>>();
        let gps_hdop = gps_data.iter().map(|(d, _)| d.hdop).collect::<Vec<f32>>();

        let df_gps = df!(
            "timestamp" => gps_timestamp,
            "latitude" => gps_lat,
            "longitude" => gps_lon,
            "altitude" => gps_alt,
            "velocity_east" => gps_velocity_east,
            "velocity_north" => gps_velocity_north,
            "hdop" => gps_hdop
        ).unwrap();

        println!("GPS Data: {:?}", df_gps);

        CsvWriter::new(std::fs::File::create("gps_data.csv").unwrap())
            .finish(&mut df_gps.clone().lazy().collect().unwrap())
            .unwrap();
    }
}