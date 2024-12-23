from cobs import cobs_decode
from tqdm import tqdm
from matplotlib import pyplot as plt
from struct import Struct
import numpy as np
from scipy.spatial.transform import Rotation as R

def gps2xyz(lat,lon,h, a=6378137, b=6356752.31425, e2=0.00669438002290):
    e2=1-(b/a)**2
    lat = np.deg2rad(lat)
    lon = np.deg2rad(lon)
    NQ = a**2/np.sqrt(a**2*np.cos(lat)**2 + b**2*np.sin(lat)**2)
    x = (NQ+h)*np.cos(lat)*np.cos(lon)
    y = (NQ+h)*np.cos(lat)*np.sin(lon)
    z = (NQ*b**2/a**2+h)*np.sin(lat)
    return x,y,z

gps=[]
pos=[]
imu=[]
if __name__=="__main__":
    with open("./log.bin","rb") as f:
        data = f.read()
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
                    pass
                case 0x60:
                    try:
                        timestamp,t,lat,lng,alt,ve,vn=parser_gps.unpack(bytes(dec))
                        gps.append((timestamp,t,(lat,lng),(ve,vn)))
                        pos.append([timestamp,lat,lng,alt,ve,vn,0])
                    except:
                        pass
pos=np.array(pos)
x,y,z=gps2xyz(pos[:,1],pos[:,2],pos[:,3])
v=np.sqrt(pos[:,-3]**2+pos[:,-2]**2+pos[:,-1]**2)
#x,y,z=gps2xyz(pos[:,1],pos[:,2],40)
R=R.from_euler('zyz',[np.pi/2,np.pi/2-np.deg2rad(pos[10,1]),np.deg2rad(pos[10,2])]).as_matrix()
p=R.T@np.array([x,y,z])
p[0]-=p[0,0]
p[1]-=p[1,0]
p[2]-=p[2,0]
fig=plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.plot(p[0],p[1],p[2])
# ax.plot(p[0],p[1],0)
ax.set_xlabel('E')
ax.set_ylabel('N')
ax.set_zlabel('U')
dt=0.1
q=ax.quiver(p[0],p[1],p[2],pos[:,-3]*dt,pos[:,-2]*dt,pos[:,-1]*dt,cmap="jet")
q.set_array(v)
plt.colorbar(q)
ax.axis("equal")
plt.show()

plt.plot(pos[:,0],pos[:,-3])
plt.plot(pos[:,0],pos[:,-2])
plt.scatter(pos[:,0],pos[:,-3],label="$v_E$")
plt.scatter(pos[:,0],pos[:,-2],label="$v_N$")
plt.legend()
plt.show()
