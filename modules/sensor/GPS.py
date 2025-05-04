from modules.sensor import Sensor
import struct

class GPS(Sensor):
    def __init__(self) -> None:
        super().__init__()
        self.id=0x20
        self.raw_data={
            f"timestamp":[],
            f"latitude":[],
            f"longitude":[],
            f"altitude":[],
            f"speed":[],
            f"heading":[],
        }
        self.parser=struct.Struct(">BxxxIhhHhHhHh")
    def parse(self, array: list[int]):
        for n in range(len(array)//16):
            _id,timestamp,latitude,longitude,altitude,speed,heading=self.parser.unpack(bytes(array[16*n:16*(n+1)]))
            self.raw_data[f"timestamp"].append(timestamp)
            self.raw_data[f"latitude"].append(latitude)
            self.raw_data[f"longitude"].append(longitude)
            self.raw_data[f"altitude"].append(altitude)
            self.raw_data[f"speed"].append(speed)
            self.raw_data[f"heading"].append(heading)
        return len(array)//16
    @property
    def database(self)->dict[str,list]:
        return self.raw_data
    @property
    def ID(self)->int:
        return self.id
    @property
    def delta_t(self)->int:
        return 1000
