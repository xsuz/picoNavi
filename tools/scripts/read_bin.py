from cobs import cobs_decode
from tqdm import tqdm
from matplotlib import pyplot as plt
from struct import Struct
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation
import pandas as pd

if __name__=="__main__":
    with open("./log.bin","rb") as f:
        data = f.read()
    gps=[]
    imu=[]
    with tqdm(total=len(data)) as pbar:
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
                    try:
                        timestamp,t,ax,ay,az,wx,wy,wz,mx,my,mz,q0,q1,q2,q3=parser_imu.unpack(bytes(dec))
                        imu.append([timestamp,ax,ay,az,wx,wy,wz,mx,my,mz])
                        # euler_q.append([ 
                        #   timestamp,
                        #   np.arctan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1*q1+q2*q2)),    # roll
                        #   np.arcsin(-2.0*(q1*q3-q0*q2)),                          # pitch
                        #   np.arctan2(2.0*(q0*q3+q1*q2),1.0-2.0*(q2*q2+q3*q3))     # yaw
                        # ])
                    except:
                        print("Error")
                        print(dec)
                case 0x60:
                    timestamp,t,lat,lng,alt,ve,vn=parser_gps.unpack(bytes(dec))
                    gps.append((timestamp,lat,lng,alt,ve,vn,0))
    
    df_imu=pd.DataFrame(imu,columns=["timestamp","ax","ay","az","wx","wy","wz","mx","my","mz"])
    df_gps=pd.DataFrame(gps,columns=["timestamp","lat","lng","alt","ve","vn","vd"])

    df_imu["timestamp"]=pd.to_datetime(df_imu["timestamp"],unit="ms")
    df_gps["timestamp"]=pd.to_datetime(df_gps["timestamp"],unit="ms")

    df_imu.set_index("timestamp",inplace=True)
    df_gps.set_index("timestamp",inplace=True)

    df_imu.to_csv("imu.csv")
    df_gps.to_csv("gps.csv")

    plt.plot(df_imu.index,df_imu["ax"],label="$a_x$")
    plt.plot(df_imu.index,df_imu["ay"],label="$a_y$")
    plt.plot(df_imu.index,df_imu["az"],label="$a_z$")
    plt.legend()
    plt.title("Acceleration")
    plt.xlabel("unix time")
    plt.ylabel("acceleration [$m/s^2$]")
    plt.show()

    plt.plot(df_imu.index,df_imu["wx"],label=r"$\omega_x$")
    plt.plot(df_imu.index,df_imu["wy"],label=r"$\omega_y$")
    plt.plot(df_imu.index,df_imu["wz"],label=r"$\omega_z$")
    plt.legend()
    plt.title("Angular velocity")
    plt.xlabel("unix time")
    plt.ylabel("angular velocity [$rad/s$]")
    plt.show()

    plt.hist(df_imu["wx"],bins=256,alpha=0.5,label=r"$\omega_x$")
    plt.hist(df_imu["wy"],bins=256,alpha=0.5,label=r"$\omega_y$")
    plt.hist(df_imu["wz"],bins=256,alpha=0.5,label=r"$\omega_z$")
    plt.legend()
    plt.title("Angular velocity")
    plt.xlabel("angular velocity [$rad/s$]")
    plt.ylabel("count")
    plt.show()