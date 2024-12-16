from cobs import cobs_decode
from tqdm import tqdm
from matplotlib import pyplot as plt
from struct import Struct
import numpy as np
from datetime import datetime

if __name__=="__main__":
    with open("./log.bin","rb") as f:
        data = f.read()
    gps=[]
    pos=[]
    imu=[]
    imu_t=[]
    with tqdm(total=len(data)+1) as pbar:
        parser_gps=Struct(">QxxxxIddfffxxxx")
        parser_imu=Struct(">QxxxxIfffffffffffff")
        while len(data)>0:
            before = len(data)
            dec, data = cobs_decode(data)
            pbar.update(before-len(data))
            if before == len(data):
                print("end")
                break
            if len(dec)==0: # Empty packet
                print("empty packet")
                continue
            match dec[8]:
                case 0x40:
                    timestamp,t,ax,ay,az,wx,wy,wz,mx,my,mz,q0,q1,q2,q3=parser_imu.unpack(bytes(dec))
                    imu.append([timestamp,ax,ay,az,wx,wy,wz,mx,my,mz])
                    imu_t.append(datetime.fromtimestamp(timestamp/1000))
                case 0x60:
                    timestamp,t,lat,lng,alt,ve,vn=parser_gps.unpack(bytes(dec))
                    gps.append((timestamp,lat,lng,alt,ve,vn,0))
                    pos.append([timestamp,lat,lng,alt,ve,vn,0])
    imu=np.array(imu).T
    np.savetxt("imu.csv",imu.T,delimiter=",")
    np.savetxt("gps.csv",np.array(gps),delimiter=",")
    plt.scatter(imu_t,imu[1],label='$a_x$',s=1)
    plt.scatter(imu_t,imu[2],label='$a_y$',s=1)
    plt.scatter(imu_t,imu[3],label='$a_z$',s=1)
    plt.legend()
    plt.show()
    plt.scatter(imu[0],imu[4],label='$w_x$',s=1)
    plt.scatter(imu[0],imu[5],label='$w_y$',s=1)
    plt.scatter(imu[0],imu[6],label='$w_z$',s=1)
    plt.legend()
    plt.show()
    plt.hist(imu[4],1024)
    plt.show()
