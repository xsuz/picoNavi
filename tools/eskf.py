#
# https://www.flight.t.u-tokyo.ac.jp/?p=800 を参照
#   - ローカルな角度誤差
#

from cobs import cobs_decode
from matplotlib import pyplot as plt
from tqdm import tqdm
from struct import Struct
import numpy as np
from scipy.spatial.transform import Rotation

def gps2xyz(lat,lon,h, a=6378137, b=6356752.31425, e2=0.00669438002290):
    e2=1-(b/a)**2
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    NQ = a**2/np.sqrt(a**2*np.cos(lat)**2 + b**2*np.sin(lat)**2)
    x = (NQ+h)*np.cos(lat)*np.cos(lon)
    y = (NQ+h)*np.cos(lat)*np.sin(lon)
    z = (NQ*b**2/a**2+h)*np.sin(lat)
    return x,y,z

def skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def mul_quat(q1,q2)->np.ndarray:
    return np.array([
        q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3],
        q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2],
        q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1],
        q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]
    ])

if __name__=="__main__":
    with open("./log.bin","rb") as f:
        data = f.read()
    pos=[]
    pos_gps=[]
    imu=[]

    yaw=[]
    pitch=[]
    roll=[]
    euler_q=[]
    x_history=[]

    dt_m=20e-3 # (s)
    R=np.eye(3)
    x=np.array([0.0,0.0,0.0, 0.0,0.0,0.0, 1.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,9.8]) #  [p,v,q,ab,wb,g] : State variable
    ex=np.zeros(18) # [ep,ev,etheta,eab,ewb,eg] : Error-State variable
    P=np.eye(18)*1e-9 # Covariance matrix
    sigma_v=1**2*dt_m**2
    sigma_theta=0.1**2*dt_m**2
    sigma_a=1e-1**2*dt_m**2
    sigma_w=1e-1**2*dt_m**2
    Q=np.diag([0,0,0,sigma_v,sigma_v,sigma_v,sigma_theta,sigma_theta,sigma_theta,sigma_a,sigma_a,sigma_a,sigma_w,sigma_w,sigma_w,0,0,0]) # Process noise covariance

    #(270)
    Fx=np.zeros((18,18)) # State transition matrix
    Fx[0:6,0:6]=np.eye(6)
    Fx[0:3,3:6]=np.eye(3)*dt_m
    Fx[3:6,15:18]=np.eye(3)*dt_m
    Fx[6:9,12:15]=-np.eye(3)*dt_m
    Fx[9:18,9:18]=np.eye(9)
    #(271)
    Fi=np.zeros((18,12))
    Fi[3:15,0:12]=np.eye(12)

    # z=[re,rn,ru] : observation variable
    Hx=np.zeros((3,19))
    Hx[0:3,0:3]=np.eye(3)
    # Hx[3:5,3:5]=np.eye(2)
    V=np.diag([2.5,2.5,2.5])#,1.0,1.0,1.0])

    Xex=np.zeros((19,18))
    Xex[0:6,0:6]=np.eye(6)
    Xex[10:19,9:18]=np.eye(9)

    t_start=1
    R_start=np.eye(3)
    r_start=np.zeros(3)


    with tqdm(total=len(data)) as pbar:
        parser_imu=Struct(">QxxxxIfffffffffffff")
        parser_gps=Struct(">QxxxxIddfffxxxx")
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
                    if t_start <0:

                        # timestamp,t,ax,ay,az,wx,wy,wz=unpack(">IxxxxIffffff",bytes(dec[:36]))
                        try:
                            timestamp,t,ax,ay,az,wx,wy,wz,mx,my,mz,q0,q1,q2,q3=parser_imu.unpack(bytes(dec))
                        except:
                            print("error")
                            continue
                        imu.append([timestamp,ax,ay,az,wx,wy,wz,mx,my,mz,q0,q1,q2,q3])

                        p=x[0:3].copy()
                        v=x[3:6].copy()
                        q=x[6:10].copy()
                        ab=x[10:13].copy()
                        wb=x[13:16].copy()
                        g=x[16:19].copy()

                        R=Rotation.from_quat(q).as_matrix()

                        am=np.array([ax,ay,az])
                        wm=np.array([wx,wy,wz])

                        # System dynamics
                        p+=v*dt_m+0.5*(R@(am-ab)+g)*dt_m**2
                        v+=(R@(am-ab)+g)*dt_m
                        q=mul_quat(q,np.array([1,*(wm-wb)*dt_m*0.5]))
                        q/=np.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3])

                        x[0:3]=p.copy()
                        x[3:6]=v.copy()
                        x[6:10]=q.copy()
                        x[10:13]=ab.copy()
                        x[13:16]=wb.copy()
                        x[16:19]=g.copy()

                        x_history.append(x.copy())

                        roll.append(
                            np.arctan2(2.0*(q[0]*q[1]+q[2]*q[3]),1.0-2.0*(q[1]*q[1]+q[2]*q[2]))
                        )
                        pitch.append(
                            np.arcsin(-2.0*(q[1]*q[3]-q[0]*q[2]))
                        )
                        yaw.append(
                            np.arctan2(2.0*(q[0]*q[3]+q[1]*q[2]),1.0-2.0*(q[2]*q[2]+q[3]*q[3]))
                        )
                        euler_q.append(
                            [np.arctan2(2.0*(q0*q1+q2*q3),1.0-2.0*(q1*q1+q2*q2)),
                            np.arcsin(-2.0*(q1*q3-q0*q2)),
                            np.arctan2(2.0*(q0*q3+q1*q2),1.0-2.0*(q2*q2+q3*q3))]
                        )
                        pos.append(p)

                        # calculate (270)
                        Fx[3:6,6:9]=-R@skew(am-ab)*dt_m
                        Fx[3:6,9:12]=-R*dt_m
                        Fx[6:9,6:9]=R.T@np.diag(wm-wb)*dt_m

                        # ex=Fx@ex # (268)
                        P=Fx@P@(Fx.T)+Q #(269) : Covariance update
                        # print(P[-3:])

                case 0x60:
                    timestamp,t,lat,lng,alt,ve,vn=parser_gps.unpack(bytes(dec))

                    rx,ry,rz=gps2xyz(lat,lng,alt)

                    if t_start >0:
                        t_start-=1
                        continue
                    elif t_start==0:
                        t_start-=1
                        r_start=np.array([rx,ry,rz])
                        R_start=Rotation.from_euler('zyz',[np.pi/2,np.pi/2-np.deg2rad(lat),np.deg2rad(lng)]).as_matrix()


                    p=x[0:3].copy()
                    v=x[3:6].copy()
                    q=x[6:10].copy()
                    ab=x[10:13].copy()
                    wb=x[13:16].copy()
                    g=x[16:19].copy()

                    p_obs=R_start.T@(np.array([rx,ry,rz])-r_start)
                    z=np.array([0.0,0.0,0.0])

                    pos_gps.append(p_obs)

                    # (279) ~ (281) : the Jacobian of the observation model
                    Xex[6:10,6:9]=0.5*np.array([
                        [-q[1],-q[2],-q[3]],
                        [ q[0],-q[3],q[2]],
                        [ q[3],q[0],-q[1]],
                        [-q[2],q[1], q[0]]
                        ])

                    H= Hx@Xex
                    K = P@(H.T)@np.linalg.inv(H@P@(H.T)+V)
                    P=(np.eye(18)-K@H)@P@(np.eye(18)-K@H).T+K@V@K.T
                    ex = K@(z-x[:3])
                    
                    # (282) : nominal state update
                    x[0:6]+=ex[0:6]
                    x[6:10]=mul_quat(x[6:10],np.array([1.0,*ex[6:9]]))
                    q_norm=np.sqrt(x[6]*x[6]+x[7]*x[7]+x[8]*x[8]+x[9]*x[9])
                    x[6]/=q_norm
                    x[7]/=q_norm
                    x[8]/=q_norm
                    x[9]/=q_norm
                    x[10:19]+=ex[9:18].copy()

                    # ex[:]=0
    imu=np.array(imu)
    euler_q=np.array(euler_q)
    pos=np.array(pos)
    pos_gps=np.array(pos_gps)
    fig=plt.figure()
    ax=fig.add_subplot(111,projection='3d')
    ax.scatter(pos[:,0],pos[:,1],pos[:,2],label="Estimated")
    ax.plot(pos_gps[:,0],pos_gps[:,1],pos_gps[:,2],label="GPS")
    ax.legend()
    ax.axis('equal')
    plt.show()
    print(np.mean(np.rad2deg(pitch)),np.mean(np.rad2deg(roll)))
    # plt.plot(imu[:,0],np.rad2deg(roll),"-.",label="roll")
    # plt.plot(imu[:,0],np.rad2deg(pitch) ,":",label="pitch")
    # plt.plot(imu[:,0],np.rad2deg(yaw)  ,"-o",label="yaw")
    plt.plot(imu[:,0],np.rad2deg(euler_q[:,0]),"-.",label="roll(madgwick)")
    plt.plot(imu[:,0],np.rad2deg(euler_q[:,1]),":",label="pitch(madgwick)")
    plt.plot(imu[:,0],np.rad2deg(euler_q[:,2]),"--",label="yaw(madgwick)")
    plt.legend()
    plt.show()

    x_history=np.array(x_history)
    plt.scatter(imu[:,0],x_history[:,-3],label="gx")
    plt.scatter(imu[:,0],x_history[:,-2],label="gy")
    plt.scatter(imu[:,0],x_history[:,-1],label="gz")
    plt.legend()
    plt.show()