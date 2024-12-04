#
# https://www.flight.t.u-tokyo.ac.jp/?p=800 を参照
#   - ローカルな角度誤差
#

from cobs import cobs_decode
from matplotlib import pyplot as plt
from tqdm import tqdm
from struct import unpack
import numpy as np

def gps2xyz(lat,lon,h, a=6378137, b=6356752.31425, e2=0.00669438002290):
    e2=1-(b/a)**2
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    NQ = a**2/np.sqrt(a**2*np.cos(lat)**2 + b**2*np.sin(lat)**2)
    x = (NQ+h)*np.cos(lat)*np.cos(lon)
    y = (NQ+h)*np.cos(lat)*np.sin(lon)
    z = (NQ*b**2/a**2+h)*np.sin(lat)
    return x,y,z

if __name__=="__main__":
    with open("./log-new.bin","rb") as f:
        data = f.read()
    gps=[]
    pos=[]
    imu=[]
    dt_m=20e-3 # (s)
    R=np.eye(3)
    x=np.zeros(19) #  [p,v,q,ab,wb,g] : State variable
    ex=np.zeros(18) # [ep,ev,etheta,eab,ewb,eg] : Error-State variable
    P=np.zeros((18,18)) # Covariance matrix
    sigma_v=0.1**2*dt_m**2
    sigma_theta=1e-8**2*dt_m**2
    sigma_a=1e-3**2*dt_m
    sigma_w=5e-3**2*dt_m
    Q=np.diag([sigma_v,sigma_v,sigma_v,sigma_theta,sigma_theta,sigma_theta,sigma_a,sigma_a,sigma_a,sigma_w,sigma_w,sigma_w]) # Process noise covariance
    Fx=np.eye(18) # State transition matrix

    R=np.zeros((3,3)) # R[q] : Rotation matrix

    Fi=np.zeros((18,12))
    # (270)
    Fi[3:9,3:9]=np.eye(6)
    Fx[6:9,9:12]=np.eye(3)*dt_m
    Fx[0:3,3:6]=np.eye(3)*dt_m
    Fx[3:6,15:18]=np.eye(3)*dt_m

    # z=[re,rn,ve,vn] : observation variable
    Hx=np.zeros((4,19))
    Hx[0:2,0:2]=np.eye(2)
    Hx[2:4,3:5]=np.eye(2)
    Xex=np.eye(19,18)
    Xex[0:6,0:6]=np.eye(6)
    Xex[10:19,9:18]=np.eye(9)


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

                    p=x[0:3].copy()
                    v=x[3:6].copy()
                    q=x[6:10].copy()
                    ab=x[10:13].copy()
                    wb=x[13:16].copy()
                    g=x[16:19].copy()

                    am=np.array([ax,ay,az])
                    wm=np.array([wx,wy,wz])

                    # System dynamics
                    p+=v*dt_m+0.5*(R@(am-ab)-g)*dt_m**2
                    v+=R@(am-ab)-g*dt_m
                    q+=0.5*np.array([
                        [-q[1],-q[2],-q[3]],
                        [ q[0],-q[3],q[2]],
                        [ q[3],q[0],-q[1]],
                        [-q[2],q[1], q[0]]
                        ])@wb*dt_m
                    # wb, ab, g : not updated

                    # calculate (270)
                    Fx[3:6,6:9]=-R@np.array([[             0,-(am[2]-ab[2]), (am[1]-ab[1])],
                                             [ (am[2]-ab[2]),             0,-(am[0]-ab[0])],
                                             [-(am[1]-ab[1]), (am[0]-ab[0]),             0]])*dt_m
                    Fx[3:6,9:12]=-R*dt_m
                    Fx[6:9,6:9]=R.T@np.diag(wm-wb)*dt_m

                    #ex=Fx@ex # (268)
                    P=Fx@P@(Fx.T)+Fi@Q@(Fi.T) #(269) : Covariance update

                case 0x60:
                    timestamp,t,lat,lng,alt,ve,vn=unpack(">IxxxxIddfffxxxx",bytes(dec))

                    p=x[0:3].copy()
                    v=x[3:6].copy()
                    q=x[6:10].copy()
                    ab=x[10:13].copy()
                    wb=x[13:16].copy()
                    g=x[16:19].copy()

                    # (279) ~ (281) : the Jacobian of the observation model
                    Xex[6:10,6:9]=0.5*np.array([
                        [-q[1],-q[2],-q[3]],
                        [ q[0],-q[3],q[2]],
                        [ q[3],q[0],-q[1]],
                        [-q[2],q[1], q[0]]
                        ])

                    H= Hx@Xex
                    K = P@(H.T)@np.linalg.inv(H@P@(H.T)+np.diag([2.5,2.5,0.1,0.1]))
                    ex = K@(np.array([0,0,0,0])-H@ex)
                    
                    # (282) : nominal state update
                    x[0:6]+=ex[0:6]
                    x[10:19]=ex[9:18]
                    ex[:]=0
                    #dt.append(timestamp-t)
                    #gps.append((timestamp,t,(lat,lng),(ve,vn)))
                    #pos.append([lat,lng])
