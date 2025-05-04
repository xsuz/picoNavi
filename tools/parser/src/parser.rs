use helium_core::{parse_data, sensor::*,cobs};

pub struct Parser{
    log: Vec<u8>,
    imu_data: Vec<(IMUData,i64)>,
    gps_data: Vec<(GPSData,i64)>,
}

impl Parser{
    pub fn new() -> Self{
        Parser{
            log: vec![],
            imu_data: vec![],
            gps_data: vec![],
        }
    }

    pub fn parse(&mut self, buf:&[u8]){
        self.log.extend(buf.iter());
        let (mut decoded,mut rest) = cobs::decode(&self.log);
        while decoded.len() > 0 {
            match decoded[8] & 0xF0 {
                0x40 => {
                    parse_data(&mut self.imu_data, &decoded,None);
                },
                0x60 => {
                    parse_data(&mut self.gps_data, &decoded,None);
                },
                _ => {
                    println!("Unknown data: {:?}", decoded);
                    break;
                }
            }
            self.log = rest.to_vec();
            (decoded, rest) = cobs::decode(&self.log);
        }
    }
    #[allow(dead_code)]
    pub fn get_imu_data(&self) -> &Vec<(IMUData,i64)>{
        &self.imu_data
    }
    #[allow(dead_code)]
    pub fn get_gps_data(&self) -> &Vec<(GPSData,i64)>{
        &self.gps_data
    }
}