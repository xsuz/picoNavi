use helium_core::{DataBase, cobs};

pub struct Parser{
    log: Vec<u8>,
    pub db: DataBase,
}

impl Parser{
    pub fn new() -> Self{
        Parser{
            log: vec![],
            db: DataBase::new(),
        }
    }

    pub fn parse(&mut self, buf:&[u8]){
        self.log.extend(buf.iter());
        let (mut decoded,mut rest) = cobs::decode(&self.log);
        while decoded.len() > 0 {
            self.db.update(&decoded, None);
            self.log = rest.to_vec();
            (decoded, rest) = cobs::decode(&self.log);
        }
    }
}