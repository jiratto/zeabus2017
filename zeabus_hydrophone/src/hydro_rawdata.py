	#!/usr/bin/env python

import struct
import numpy
import math
import time
import serial
import numpy as np
import scipy.stats as stats
#from zeabus_hydrophone.srv import hydro_info
#from zeabus_hydrophone.msg import hydro_msg

#ser = serial.Serial('/dev/ttyUSB0', 115200)
ser = serial.Serial('/dev/usb2serial/ftdi_AJ038YFU', 115200)
fre = 0
set_stat = False
send_azi = [0]*3
send_elv = [0]*3
mean_azi = 0 
mean_elv = 0 
sum_azi = 0
sum_elv = 0
send_elv = 0
tick_azi = 0
tick_elv = 0
a = 0
def float_hex4(f):
	return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<f', f))
def uint_hex4(I):
	return ''.join(('%2.2x'%ord(c)) for c in struct.pack('<I', I))
def set(mode,value):
	res = []
	res.append('\xff')
	res.append('\xff')
	#res.append(mode)
	if mode==0x00 or mode==0x01 or mode==0x04:
		print "uint32"
		if mode==0x00:
			res.append('\x00')
		elif mode==0x01:
			res.append('\x01')
		elif mode==0x04:
			res.append('\x04')
		tmp = uint_hex4(value)
		res.append(tmp[0:2].decode("hex"))
		res.append(tmp[2:4].decode("hex"))
		res.append(tmp[4:6].decode("hex"))
		res.append(tmp[6:8].decode("hex"))
	elif mode==0x02 or mode==0x03:
		print "float"
		if mode==0x02:
			res.append('\x02')
		elif mode==0x03:
			res.append('\x03')
		tmp = float_hex4(value)
		res.append(tmp[0:2].decode("hex"))
		res.append(tmp[2:4].decode("hex"))
		res.append(tmp[4:6].decode("hex"))
		res.append(tmp[6:8].decode("hex"))
	res.append('\x00')
	#print list(array.array('B', res).tostring())
	print res
	return ''.join(res)


class particleFilter:
    def __init__(self,N,x_min,x_max,f,h,simga_r):
        nx = x_min.shape[0]     #number of element 
        d = np.random.rand(nx,N) #random 3 to N
        rang_x = (x_max-x_min).reshape((nx,1)) 
        rang_x = np.repeat(rang_x,N,1)
        x_min = x_min.reshape((nx,1))
        x_min = np.repeat(x_min,N,1)
        state_x = d*rang_x+x_min
        self.N = N
        self.state_x = state_x
        self.weight = (1./float(N))*np.ones((N,))
        self.predict_function = f
        self.observation_function = h
        self.sigma_r = simga_r
        self.nx = nx
    def predict(self,v):
        state_x_km1 = self.state_x
        state_x_k = self.predict_function(state_x_km1,v)
        self.state_x = state_x_k

    def update_weight(self,z_obv):
        nx = self.nx
        ny = z_obv.shape[0]
        N = self.N
        state_x = self.state_x
        mu = self.observation_function(state_x)
        z= z_obv.reshape((ny,1))
        z = np.repeat(z,N,1)
        z = z -mu
        z2 = np.linalg.solve(self.sigma_r,z)
        e = 0.5*(z*z2).sum(0) + 0.5*np.log(np.linalg.det(self.sigma_r))
        prob = np.exp(-e)
        w = self.weight
        w = w*prob
        w = w/w.sum()
        self.weight = w

    def resampling(self):
        N = self.N
        w = self.weight
        nx = self.nx
        Neff = 1./(w**2).sum()
        if Neff < (2./3.)*N : # resampling
            w_cum = w.cumsum()
            d = np.random.rand(N,)
            x_new = np.zeros((nx,N))
            x_old = self.state_x
            self.weight = (1./float(N))*np.ones((N,))
            for k in range(N):
                dk = d[k]
                b = np.nonzero(w_cum>dk)
                b = b[0]
                b = b.min()
                x_new[:,k] = x_old[:,b]
            self.state_x = x_new

    def get_mmse(self):
        x = self.state_x
        w = self.weight
        xmmse = (w*x).sum(1)
        return xmmse









def fx(x):
    x[-1] = 0
    return x

def fx2(x,y):
    pi = np.pi
    z = x+y
    z[0] = z[0]#%(2*np.pi)
    z[0] = z[0]*(z[0]<= pi) +(z[0]-2*pi)*(z[0]>pi)
    z[0] = z[0]*(z[0]> -pi) +(z[0]+2*pi)*(z[0]<=-pi)
    z[1] = z[1]*(z[1]<= pi/2.0) +(pi-z[1])*(z[1]>pi/2.0)
    z[1] = z[1]*(z[1]>=0) +(-z[1])*(z[1]<0)
    z[2] = z[2]#%(2*np.pi)
    z[2] = z[2]*(z[2]<= pi) +(z[2]-2*pi)*(z[2]>pi)
    z[2] = z[2]*(z[2]> -pi) +(z[2]+2*pi)*(z[2]<=-pi)
    return z


def hx(x):
    global fre
    az = x[0]
    el = x[1]
    c  = x[2]
    if x.ndim > 1:
        nx,N = x.shape
    else:
        N = 1
    if fre == 0:
        fre = 32000 ##### edit ####
    fs = fre/1000.0
    lamb = 1500./fs
    phase = np.array([np.cos(az)*np.sin(el),np.sin(az)*np.sin(el),c])
    ant = np.array([[10,10],[-10,10],[-10,-10.],[10,-10]])
    ant_phase = ant*np.pi*2/lamb
    A = np.ones((4,3))
    A[:,:2] = ant_phase
    angles = np.dot(A,phase)
    cr = np.cos(angles)
    ci = np.sin(angles)
    if x.ndim > 1:
        c = np.zeros((8,N))
        c[:4,:] = cr
        c[4:,:] = ci
    else:
        c = np.zeros((8,))
        c[:4] = cr
        c[4:] = ci
    return c

def reset():
    global send_azi
    global send_elv
    global mean_azi
    global mean_elv
    global sum_azi
    global sum_elv
    global tick_azi
    global tick_elv
    global a

    a = 0
    send_azi = [0]*3
    send_elv = [0]*3
    mean_azi = 0
    mean_elv = 0
    sum_azi = 0
    sum_elv = 0
    tick_azi = 0
    tick_elv = 0
    #print send_elv
def getData_three():
    global fre
    global send_azi
    global send_elv
    global mean_azi
    global mean_elv
    global sum_azi
    global sum_elv
    global send_elv
    global tick_azi
    global tick_elv
    global a


    reset()
    
    #print mean_azi
    #data = hydro_info()
    for a in range(0,3):
        getData(a)
    #print len(send_azi)
    a = 0

    mean_azi = (send_azi[0]+send_azi[1]+send_azi[2])/3
    mean_elv = (send_elv[0]+send_elv[1]+send_elv[2])/3
    for i in range(0,3):
        if np.abs(float(send_azi[i]-mean_azi))<= 45 :
            sum_azi = sum_azi + send_azi[i]
            tick_azi+=1

        if np.abs(float(send_elv[i]-mean_elv)) <= 45 :
            sum_elv = sum_elv + send_elv[i]
            tick_elv+=1

    if(tick_azi == 0):
        print "stop!"
    #    data.stop = True
    if(tick_elv == 0):
        print "stop!"
    #    data.stop = True

    #data.stop = False
    #data.azi = sum_azi/tick_azi
    #data.elv = sum_elv/tick_elv
    #data.distance = 0 
   # print "Mean azi : %f, Count azi : %f" %(sum_azi/tick_azi,tick_azi)
   # print "Mean elv : %f, Count elv : %f" %(sum_elv/tick_elv,tick_elv)
    send_azi = [0]*3
    send_elv = [0]*3  
    #return data

def getData(a):
    global send_azi 
    global send_elv
    global mean_azi
    global mean_elv
    global sum_azi
    global send_elv
    global tick_azi
    global tick_elv
    #global a
    global fre
    print "Test"
    x=0
	# x = ser.read(1)
	# print x
	# if x=='\xff':
	# 	x = ser.read(1)
	# 	if x=='\xff'
    ct = 0
    while True:
        x = ser.read(1)
        if x=='\xff':
            x = ser.read(1)
            if x=='\xff':
                x = ser.read(94)
                break
 
	#	print x	
    
    # print '======='*2
    # print len(x)
    seq = struct.unpack("<H", ''.join(x[0:2]))[0]
    azi = struct.unpack("<f", ''.join(x[2:6]))[0]
    elv = struct.unpack("<f", ''.join(x[6:10]))[0]
    itv = struct.unpack("<H", ''.join(x[10:12]))[0]
    pct = struct.unpack("<H", ''.join(x[12:14]))[0]
    po1 = struct.unpack("<f", ''.join(x[14:18]))[0]
    po2 = struct.unpack("<f", ''.join(x[18:22]))[0]
    po3 = struct.unpack("<f", ''.join(x[22:26]))[0]
    po4 = struct.unpack("<f", ''.join(x[26:30]))[0]
    stt = struct.unpack("<c", ''.join(x[30:31]))[0]
    fre = struct.unpack("<I", ''.join(x[31:35]))[0]
    # print '==='*9
    # print fre
    print "Power Avg From 4 CH."
    print po1
    print po2
    print po3
    print po4
    print "--------------------"
    #print "power " %(po1,po2,po3,po4)
    fthres = struct.unpack("<f", ''.join(x[35:39]))[0]
    pthres = struct.unpack("<f", ''.join(x[39:43]))[0]
    print "------Power------"
    print fthres
    print pthres
    print "-----------------"
    ph1 = struct.unpack("<f", ''.join(x[43:47]))[0]
    ph2 = struct.unpack("<f", ''.join(x[47:51]))[0]
    ph3 = struct.unpack("<f", ''.join(x[51:55]))[0]
    ph4 = struct.unpack("<f", ''.join(x[55:59]))[0]
    c0r = struct.unpack("<f", ''.join(x[59:63]))[0]
    c0i = struct.unpack("<f", ''.join(x[63:67]))[0]
    c1r = struct.unpack("<f", ''.join(x[67:71]))[0]
    c1i = struct.unpack("<f", ''.join(x[71:75]))[0]
    c2r = struct.unpack("<f", ''.join(x[75:79]))[0]
    c2i = struct.unpack("<f", ''.join(x[79:83]))[0]
    c3r = struct.unpack("<f", ''.join(x[83:87]))[0]
    c3i = struct.unpack("<f", ''.join(x[87:91]))[0]
    resb = struct.unpack("<c", ''.join(x[91:92]))[0]
    csm = struct.unpack("<H", ''.join(x[92:94]))[0]
    			
        # phase_c = []
    c_obv = []
                    
    m1 = c0r+1j*c0i
    m2 = c1r+1j*c1i
    m3 = c2r+1j*c2i
    m4 = c3r+1j*c3i

    
    #set to get raw data

    print "c0r = %.6f, c0i = %.6f, c1r = %.6f, c1i = %.6f, c2r = %.6f, c2i = %.6f, c3r = %.6f, c3i = %.6f\n" % (c0r,c0i,c1r,c1i,c2r,c2i,c3r,c3i)
    
    #m_mx = max(max(np.abs(m1),np.abs(m2),max(np.abs(m3),np.abs(m4))))
    #if m_mx > 0:
    cp = np.array([c0r/np.abs(m1),c1r/np.abs(m2),c2r/np.abs(m3),c3r/np.abs(m4),c0i/np.abs(m1),c1i/np.abs(m2),c2i/np.abs(m3),c3i/np.abs(m4)])
    
    #c_obv.append(temp2)
        
    #print "Board"
    #print seq,azi,elv
	
    
    #for cp in c_obv:
        #cp = hx(np.array(np.array([-pi*0.7,pi/4.,0])))+np.random.normal(0,np.sqrt(0.01),(8,))
    v = np.random.rand(3,N)*2.0-1.0
    v[0] = v[0]*pi/16.0   #trace max speed 
    v[1] = v[1]*pi/16.0
    v[2] = v[2]*pi/3.0
    pf.predict(v)
    pf.update_weight(cp)
    pf.resampling()
    xk = pf.get_mmse()
    az_t = xk[0]*180./pi
    elv_t = xk[1]*180./pi
    c = xk[2]
    print "Particle Filter Az: %.2f,Elv: %.2f,Ot: %.3f ,Freq : %.0f" % (az_t, elv_t,c,fre/1000)
    print "Pcocess : %f, Interval : %f" % (pct,itv)
    #data = hydro_msg()
    #data.azi = az_t
    #data.elv = elv_t
    #return pf
    send_azi[a]=az_t #np.abs(az_t)
    send_elv[a]=elv_t #np.abs(elv_t)
    
def greet():
    #rospy.init_node('GUGU_IS_COMING_TO_TOWN')
    #s = rospy.Service('hydro', hydro_info, getData)
    #print "READY TO GET THE DATA"
    res = set(0x00,2000000)
    ser.write(res)
	#res = set(0x01,28000)
	#ser.write(res)
    res = set(0x02,0.25)
    ser.write(res)
    res = set(0x03,0.02)
    ser.write(res)
    res = set(0x04,1500)
    ser.write(res)
	
if __name__ == '__main__':
	# try:
	# 	greet()
	# except rospy.ROSInterruptException:
	# 	pass
    res = set(0x02,0.3)
    ser.write(res)
    res = set(0x03,0.02)
    ser.write(res)
    R = np.eye(8)*0.1
    pi = np.pi
    x_min = np.array([-pi,0,-pi])
    x_max = np.array([pi,pi/2.,pi])
    N= 500
    pf = particleFilter(N,x_min,x_max,fx2,hx,R)
    while True :
        getData_three()
        #pf = pf2 
