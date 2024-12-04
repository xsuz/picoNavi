from cobs import cobs_decode
from tqdm import tqdm
from matplotlib import pyplot as plt
from struct import unpack
import numpy as np

if __name__=="__main__":
    with open("./log.bin","rb") as f:
        data = f.read()
    gps=[]
    pos=[]
    imu=[]
    with tqdm(total=len(data)+1) as pbar:
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
            match dec[4]:
                case 0x40:
                    timestamp,t,ax,ay,az,wx,wy,wz=unpack(">IxxxxIffffff",bytes(dec))
                    imu.append([timestamp,ax,ay,az,wx,wy,wz])
                case 0x60:
                    timestamp,t,lat,lng,alt,ve,vn=unpack(">IxxxxIddfffxxxx",bytes(dec))
                    gps.append((timestamp,t,(lat,lng),(ve,vn)))
                    pos.append([timestamp,lat,lng,alt,ve,vn,0])
    imu=np.array(imu).T
    plt.scatter(imu[0],imu[1],label='$a_x$',s=1)
    plt.scatter(imu[0],imu[2],label='$a_y$',s=1)
    plt.scatter(imu[0],imu[3],label='$a_z$',s=1)
    plt.legend()
    plt.show()
    plt.scatter(imu[0],imu[4],label='$w_x$',s=1)
    plt.scatter(imu[0],imu[5],label='$w_y$',s=1)
    plt.scatter(imu[0],imu[6],label='$w_z$',s=1)
    plt.legend()
    plt.show()
    plt.hist(imu[4],1024)
    plt.show()
