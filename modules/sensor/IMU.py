from modules.sensor import Sensor
import struct

class IMU(Sensor):
    def __init__(self) -> None:
        super().__init__()
        self.id=0x21
        self.raw_data={
            f"timestamp":[],
            f"cadence":[],
            f"power":[],
        }
        self.parser=struct.Struct(">BxxxIhh")
    def parse(self, array: list[int]):
        for n in range(len(array)//12):
            _id,timestamp,power,rpm=self.parser.unpack(bytes(array[16*n:16*(n+1)]))
            self.raw_data[f"timestamp"].append(timestamp)
            self.raw_data[f"cadence"].append(rpm)
            self.raw_data[f"power"].append(power)
        return len(array)//12
    @property
    def database(self)->dict[str,list]:
        return self.raw_data
    @property
    def ID(self)->int:
        return self.id
    @property
    def delta_t(self)->int:
        return 100