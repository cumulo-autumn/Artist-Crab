import sys
import smbus
import math
import time
import pigpio

##############   define constants   ########################################################################
bus = smbus.SMBus(1)
pi=pigpio.pi() #pigpio start

addr1 = 0x1d     #address of ADXL345
addr2 = 0x76    #address of MS5607 
addr3 = 0x1e    #address of HMC5883L
addr4=0x6b      #address of L3GD20

brushless_port=13  #brushless Pin
tailservo_port=18
wingservo_port=19
MU2=23
SV=24
FP1=4                        #for parachute open
FP2=12                     #for parachute separation
RXD=15
LED=11

count0=0
count1=0
count2=0
count3=0
count4=0
tw=5                                   #for sensors
tw2=50                              #for GPS
tw3=50                              #for MU2  
tw4=5                             #for control
ave = tw4 / tw
g = 9.798                          # g[m/s^2] at Tokyo
xtemp=[0]*1
ytemp=[0]*1
ztemp=[1]*1 
rolltemp=[0] *1
pitchtemp=[0] *1
yawtemp=[0] *1
xhtemp=[0]*1
yhtemp=[0]*1
zhtemp=[0]*1
sPRESSURE = 1011.8         #go to web and check the value
C = [0]*8                                 #declare a list for the coefficient c_1 to c_6
scale = 0.92                        #scale for Compass
GLat=40.87973                   #set Goal Latitude here    #35.606861
GLon=119.12180                 #set Goal Longtitude here  #139.684777
R=6378137                             #radius of the earth
sen = ["0"]*10
sentemp2=["0"]*10
phi=0
rotemp=[0]*1
pitatal=[0]*1
pitemp2=[0]*1
pitemp3=[0]*1
pw=[1530]*1
logtail=[-1000]*1
logwing=[-1000]*1
logbrushless=[-1000]*1
sample = 40                                                                                                     #the number of sample data
queue = [[0]*sample, [0]*sample, [0]*sample, [0]*sample]      # [x, y, z, height]
slope = [0]*4                                                                                                  # [x, y, z, height]
intercept = [0]*4                                                                                       # [x, y, z, height]
R_2 = [0]*4                                                                                                       # [x, y, z, height]
onoff1=[0]*1
onoff2=[0]*1
landed=[1]*1
comoff=[0]*3              #offset for compass comoff[0]...x compff[1]...y comoff[2]...z
comoff[0]=104.9 
comoff[1]=131.25
comoff[2]=106.7
deltax=0
deltay=0
GPSx=500
GPSy=500
deftatal = 0
wingfix = 0
tailfix = 0
ailst=1     #the time to start ailron control [min]
deftemp=-1000

h1=pi.serial_open("/dev/ttyAMA0",19200)                          #handle for MU2
pi.bb_serial_read_open(RXD, 9600, 8)                                       #software-serial port for GPS
pi.set_mode(MU2,pigpio.OUTPUT)                                                  #MU2 transistor
pi.set_mode(SV,pigpio.OUTPUT)                                                    #servo transistor
pi.set_mode(RXD, pigpio.INPUT)                                                   #RXD fot GPS
pi.set_mode(FP1,pigpio.INPUT)                                                    #flight pin for open
pi.set_mode(FP2,pigpio.INPUT)                                                    #flight pin for separation
pi.set_pull_up_down(FP2,pigpio.PUD_UP)
pi.set_mode(LED,pigpio.OUTPUT)                                                  #LED blink
#############################################################################################



## define  converting unsigned_2byte_integer to signed_2byte_integer function ##
def si_2b(x):
	sx = x-((x&0b1000000000000000)<<1)
	return sx
	
## define serialwrite_function for MU2 ##
def serial_string(s):
	i = 0
	while i < len(s):
		pi.serial_write_byte(h1,ord(s[i]))
		i+=1
	
## define serialwrite_DATA_function for MU2 ##
def serial_data3(d1,d2,d3,d4,d5,d6,d7):
	serial_string("@DT")
	ldata=len(d1)+len(d2)+len(d3)+len(d4)+len(d5)+len(d6)+len(d7)+5+5+6+7+6+5+5
	hexdata=hex(ldata)
	serial_string(hexdata[2:])
	serial_string(" LaT=")
	i = 0
	while i < len(d1):
		pi.serial_write_byte(h1,ord(d1[i]))
		i+=1
	serial_string(" Lon=")
	i = 0
	while i < len(d2):
		pi.serial_write_byte(h1,ord(d2[i]))
		i+=1
	serial_string(" roll=")
	i = 0
	while i < len(d3):
		pi.serial_write_byte(h1,ord(d3[i]))
		i+=1
	serial_string(" pitch=")
	i = 0
	while i < len(d4):
		pi.serial_write_byte(h1,ord(d4[i]))
		i+=1
	serial_string(" Alti=")
	i = 0
	while i < len(d5):
		pi.serial_write_byte(h1,ord(d5[i]))
		i+=1
	serial_string(" Com=")
	i = 0
	while i < len(d6):
		pi.serial_write_byte(h1,ord(d6[i]))
		i+=1
	serial_string(" phi=")
	i = 0
	while i < len(d7):
		pi.serial_write_byte(h1,ord(d7[i]))
		i+=1
	serial_string("\r\n")
	
## define  ADXL_function ##
def accel():
	lowpass1=0.7              #0.7
	lowpass2=0.3               #0.3
	x = (0.0039*si_2b(bus.read_word_data(addr1,0x32)))*(-1) + 0.0445    # - 0.03802-0.023
	y = (0.0039*si_2b(bus.read_word_data(addr1,0x34)) )*(-1)  -0.0434      #+ 0.06015+0.013
	z = 0.0039*si_2b(bus.read_word_data(addr1,0x36)) +(1-0.8249)         #+ (1-0.93496)+(1-0.889)  
	
	
	#low-pass filter
	x = lowpass1*xtemp[0] + (1-lowpass1)*x
	y = lowpass1*ytemp[0] + (1-lowpass1)*y
	z = lowpass1*ztemp[0] + (1-lowpass1)*z
	xtemp[0] = x
	ytemp[0] = y
	ztemp[0] = z
	
	roll = math.atan2(y,z) * 180 / math.pi
	pitch = math.atan2(x,z) * 180 / math.pi
	yaw = math.atan2(x,y) * 180 / math.pi
	
	roll = lowpass2*rolltemp[0] + (1-lowpass2)*roll
	pitch = lowpass2*pitchtemp[0] + (1-lowpass2)*pitch
	yaw = lowpass2*yawtemp[0] + (1-lowpass2)*yaw
	
	pitemp2[0]=pitch-pitchtemp[0]
	rolltemp[0] = roll
	pitchtemp[0] = pitch
	yawtemp[0] = yaw
	rotemp[0] += roll
	pitatal[0] += pitch
	pitemp3[0] += pitemp2[0]
	
	return (x,y,z,roll,pitch,yaw)         
	
## define  MS5607_function ##
def alti():
	## Read_ADC  D1 and D2 ##
	##D1
	bus.write_byte(addr2,0x48)  #start D1 conversion
	time.sleep(0.01)    #wait for conversion
	templist = bus.read_i2c_block_data(addr2,0x00,3)  #read D1 as list
	D1=(templist[0]<<16)+(templist[1]<<8)+templist[2]

	##D2
	bus.write_byte(addr2,0x58)  #start D1 conversion
	time.sleep(0.01)    #wait for conversion
	templist = bus.read_i2c_block_data(addr2,0x00,3)  #read D2 as list
	D2=(templist[0]<<16)+(templist[1]<<8)+templist[2]


	## calculate 1st order pressure and temperature ##
	dT = D2 - C[5]*(2**8)
	OFF = C[2]*(2**17) + (dT * C[4])/(2**6)
	SENS = C[1]*(2**16) + (dT*C[3])/(2**7)
	TEMPERATURE = 2000 + (dT*C[6])/(2**23)
    #PRESSURE = ((D1*SENS)/(2**21)-OFF)/(2**15)

	## calculate 2nd order pressure and temperature ##
	if TEMPERATURE < 2000:
	    T2 = (dT**2)/(2**31)
	    OFF2 = 61 * ((TEMPERATURE-2000)**2)/(2**4)
	    SENS2 = 2 * ((TEMPERATURE-2000)**2)
	    if TEMPERATURE < -1500:
	        OFF2 += 15 * ((TEMPERATURE+1500)**2)
	        SENS2 += 8*((TEMPERATURE+1500)**2)
	else:
	        T2 = 0
	        OFF2 = 0
	        SENS2 = 0
        
	TEMPERATURE =(TEMPERATURE-T2)/100.0
	OFF -= OFF2
	SENS -= SENS2
	PRESSURE = (((D1*SENS)/(2**21) - OFF)/(2**15))/100.0
	HEIGHT = ((1013.25/sPRESSURE)**(1/5.2526)) *((sPRESSURE/PRESSURE)**(1/5.2526)-1.0)*(TEMPERATURE+273.15)/0.0065
	
	return (HEIGHT,TEMPERATURE,PRESSURE)
	
## define  HMC5883L_function ##
def compass(roll,pitch):
	lowpass3 = 0.5
	xmagtemp = bus.read_word_data(addr3,0x03)
	ymagtemp= bus.read_word_data(addr3,0x07)
	zmagtemp = bus.read_word_data(addr3,0x05)
	
	xmag = si_2b(((xmagtemp&0xff00)>>8) | ((xmagtemp&0x00ff)<<8)) * scale*(-1)
	ymag = si_2b(((ymagtemp&0xff00)>>8) | ((ymagtemp&0x00ff)<<8)) * scale*(-1)
	zmag = si_2b(((zmagtemp&0xff00)>>8) | ((zmagtemp&0x00ff)<<8)) * scale
	
	##### calibration ###########
	
	## collect data for offset ##
	f5 = open("magdata.txt","a")
	f5.write(str(xmag)+","+str(ymag)+","+str(zmag)+" The time is:"+time.ctime())
	f5.write("\n")
	f5.close()
	
	## offset ##
	xmag=xmag-comoff[0]
	ymag=ymag-comoff[1]
	zmag=zmag-comoff[2]
	
	## low-pass filter ##
	xmag = lowpass3*xhtemp[0] + (1-lowpass3)*xmag
	ymag = lowpass3*yhtemp[0] + (1-lowpass3)*ymag
	zmag = lowpass3*zhtemp[0] + (1-lowpass3)*zmag
	xhtemp[0] = xmag
	yhtemp[0] = ymag
	zhtemp[0] = zmag
	
	## normalize ##
	magnitude=(xmag**2.0+ymag**2.0+zmag**2.0)**0.5
	xmag=xmag/magnitude
	ymag=ymag/magnitude
	zmag=zmag/magnitude
	
	## declinationAngle ##
	a=13           #a:deg 
	b=40         #b:min
	c=1          # E (positive)  W(negative)
	declinationAngle =c* (a + (b/60))*math.pi/180
	
	## tilt compensation ##
	xh = xmag *math.cos(pitch*math.pi/180) + zmag *math.sin(pitch*math.pi/180)
	yh = xmag*math.sin(roll*math.pi/180)*math.sin(pitch*math.pi/180) + ymag*math.cos(roll*math.pi/180) - zmag*math.sin(roll*math.pi/180)*math.cos(pitch*math.pi/180)
	
	heading = math.atan2(yh,xh)+declinationAngle
	
	## offset angle ##
	offangle=50+15
	
	theta = heading*180/math.pi-offangle
	
	if theta>180:
		theta -= 360
	elif theta<-180:
		theta += 360
	
	return theta
	
## define tailservo function ##
	
def tail_servo(x):
	offtemp=-10-11-8
	pulsewidth=1500+(x-offtemp)*500*2/90 
	pi.set_servo_pulsewidth(tailservo_port,pulsewidth)
	logtail[0]=x
	
## define wingservo function ##
def wing_servo(x):
	pulsewidth=1500+x*500*2/90
	pi.set_servo_pulsewidth(wingservo_port,pulsewidth)
	logwing[0]=x
	
## define brushlessmotor function ##
def brushless_calibration():
	pi.set_servo_pulsewidth(brushless_port,500)
	time.sleep(8)                               #defalt: 8 sec
	
def brushless(x):
	for i in range(500,x):
		pi.set_servo_pulsewidth(brushless_port,i)
		time.sleep(0.001)
	
def stopbrushless(x,y):
	for i in range(x,y):
		pi.set_servo_pulsewidth(brushless_port,x+y-i)
		time.sleep(0.001)
	
## define Landing_Detection ##
def value_change(value):  # value = [Accel[0], Accel[1], Accel[2], HEIGHT]
	## queue ##
	for i in range(4):
		del queue[i][0]
		queue[i].append(value[i])
	
	## calculate slope ##
	sum_x = [0]*4
	sum_y = [0]*4
	sum_xy = [0]*4
	sum_x_2 = [0]*4
	for i in range(4):
		for j in range(sample):
			sum_x[i] += j
			sum_y[i] += queue[i][j]
			sum_xy[i] += j*queue[i][j]
			sum_x_2[i] += j*j
		slope[i] = (sample*sum_xy[i] - sum_x[i]*sum_y[i])/(sample*sum_x_2[i] - sum_x[i]**2)
		intercept[i] = (sum_x_2[i]*sum_y[i] - sum_xy[i]*sum_x[i])/(sample*sum_x_2[i] - sum_x[i]**2)
	
	## calculate R_2 ##
	sum_y = [0]*4
	ave_y = [0]*4
	sum1 = [0]*4
	sum2 = [0]*4
	for i in range(4):
		for j in range(sample):
			sum_y[i] += queue[i][j]
		ave_y[i] = sum_y[i]/sample
		for j in range(sample):
			sum1[i] += (slope[i]*j+intercept[i] - ave_y[i])**2
			sum2[i] += (queue[i][j]-ave_y[i])**2
		if sum2[i] == 0:
			R_2[i]=1
		else:
			R_2[i] = abs(sum1[i]/sum2[i])
	
def stop_rotating(slope, R_2, HEIGHT):
	a = 1
	b =0.01
	c = -10000
	if abs(slope[0])<a and abs(slope[1])<a and abs(slope[2])<a and abs(slope[3])<a:
		if R_2[0]>b and R_2[1]>b and R_2[2]>b and R_2[3]>b:
			if HEIGHT<c:
				if onoff2[0]:
					stopbrushless(500,pw[0])
					f4 = open(f4name,"a")
					f4.write("landed")
					f4.write("\n")
					f4.close()
					onoff2[0]=0
					landed[0]=0

######  Start MS5607 configuration ############################################################################

## reset sequence ##
bus.write_byte(addr2,0x1e)      #send reset command
time.sleep(0.003)                               #wait for reset

## ReadProm sequence c_1~c_6 and crc ##
for i in range(8):
    C[i] = 0                                                                                          #clean the cash
    temp= bus.read_word_data(addr2,0xA0+2*i)      #request the coefficient of c_i and read it
    C[i] = ((temp&0x00ff)<<8) + ((temp&0xff00)>>8)

######  end MS5607 config      #################################################################################


###### start ADXL345 configuration    ##########################################################################

## set X,Y,X offset ##
bus.write_byte_data(addr1,0x1e,0b00000000)		#x_off
bus.write_byte_data(addr1,0x1f,0b00000000)		#y_off
bus.write_byte_data(addr1,0x20,0b00000000)		#z_off

## set BW_RATE ##
bus.write_byte_data(addr1,0x2c,0b00001110)    #High_Power_mode band width 800Hz (Output data rate 1600Hz)

## set DATA_FORMAT ##
bus.write_byte_data(addr1,0x31,0b00001011)     # FULL_RES on   / unjustified /   Range  +- 8g

## set POWER_CTL ##
bus.write_byte_data(addr1,0x2d,0b00001000)     #start measure mode

###### end ADXL345  config       ##################################################################################


######   start HMC5883L  configuration    ########################################################################

bus.write_byte_data(addr3,0x00,0b00010000)     #configA   1ave. 15Hz
bus.write_byte_data(addr3,0x01,0b00100000)     #configB   choose the gain 
bus.write_byte_data(addr3,0x02,0b00000000)     #Mode

######   end HMC5883L  config  ####################################################################################


try:
	pi.write(MU2,0)
	pi.write(SV,0)
	fname="sensors"+time.ctime() + ".txt"
	f2name="GPSlog"+time.ctime() + ".txt"
	f3name="GPSbuffer"+time.ctime() + ".txt"
	f4name="control"+time.ctime() + ".txt"
	f = open(fname,"a")
	f.write("start!\n")
	f.close()
	f2 = open(f2name,"a")
	f.close()
	f3 = open(f3name,"a")
	f3.close()
	f4 = open(f4name,"a")
	f4.write("start!\n")
	f4.close()
	
	brushless_calibration()
	
	## detect parachute open #
	while True:
		if pi.read(FP1):
			pi.write(MU2,1)
			f4 = open(f4name,"a")
			f4.write("parachute has opened!\n")
			f4.close()
			break
		elif count0%100==0:
			pi.write(LED,1)
			count0 += 1
		else:
			pi.write(LED,0)
			count0 += 1
	
	while True:
		
		pi.write(LED,count1%2)
		
		#sent the initiating setup to MU2
		t1=500
		if count1<t1:
			serial_string("@UI0042,B05E\r\n")
			serial_string("@CH07\r\n")
			if count1%100==0:
					f4 = open(f4name,"a")
					f4.write("setup MU2\n")
					f4.close()
		
		## detect parachute Separation ##
		if landed[0]:
			if pi.read(FP2):
					onoff2[0]=1
					f4 = open(f4name,"a")
					f4.write("separation executed!\n")
					f4.close()
		
		#### read sensors section ####
		
		## read MS5607 ##
		if count1%tw2==0:
			Altitude = alti()
		
		if count1%tw==0:
			## read ADXL ##
			Accel = accel()
			
			## read HMC5883L ##
			Compass = compass(Accel[3],Accel[4])
			
			## read GPS from buffer and extract coordinate data ##
			f3 = open(f3name,"r")
			para = f3.readlines()
			lp=len(para)
			for i in range(0, lp-1):
				sentemp = para[i]
				sen = sentemp.split(",")
				if sen[0] == "$GPGGA":
					sentemp2=sen
					#print(sen[0]+sen[1]+sen[2]+sen[4])
					f3.close()
					f3 = open(f3name,"w")
					f3.close()
					if sen[6] == "1":
						Lattemp=float(sen[2])/100.0
						Lontemp=float(sen[4])/100.0
						Lat=(Lattemp-int(Lattemp))*100.0/60.0+int(Lattemp)
						Lon=(Lontemp-int(Lontemp))*100.0/60.0+int(Lontemp)
						#print("Lat="+sen[2]+"  Lon="+sen[4])
						#print("Lat="+str(Lat)+"  Lon="+str(Lon))
						GPSx=R*math.cos(GLat/180.0*math.pi)*(Lon/180.0*math.pi-GLon/180.0*math.pi)
						GPSy=-R*(Lat/180.0*math.pi-GLat/180.0*math.pi)
						deltax=GLon-Lon
						deltay=GLat-Lat
						phi=math.atan2(GPSx,GPSy)*180.0/math.pi
						phi2=math.atan2(deltax,deltay)*180.0/math.pi
						print("x= "+str(Lat)+"y= "+str(Lon)+"phi= "+str(phi))
						#print("dx= "+str(deltax)+"dy= "+str(deltay)+"phi2= "+str(phi2))
					else:
						print("no signal")
					break
					
			## save the data of sensors ##
			f = open(fname, "a")
			f.write(str(Accel[0])+","+str(Accel[1])+","+str(Accel[2])+","+str(Accel[3])+","+str(Accel[4])+","+str(Altitude[0])+","+str(Compass)+","+str(phi)+" The time is:"+time.ctime())
			f.write("\n")
			f.close()
			## debug ##
			#print("roll= "+str(Accel[3])+"pitch= "+str(Accel[4])+"gall= "+str((Accel[0]**2.0+Accel[1]**2.0+Accel[2]**2.0)**(0.50)))
		
		## read GPS and put it in log and buffer ##
		(count, data) = pi.bb_serial_read(RXD)
		if count:
			f2 = open(f2name,"a")
			f3 = open(f3name,"a")
			f2.write(data)
			f2.close()
			f3.write(data)
			f3.close()
		
		## send the data to MU2 ##
		if count1%tw3==0:
			if sentemp2[6] == "1":
				#serial_string("@DT03GPS\r\n")
				serial_data3(str(Lat),str(Lon),str(Accel[3]),str(Accel[4]),str(Altitude[0]),str(Compass),str(phi))
			else:
				#serial_string("@DT09no signal\r\n")
				serial_data3("no signal","no signal",str(Accel[3]),str(Accel[4]),str(Altitude[0]),str(Compass),"no signal")
		
		#### control section ####
		if count1%tw4==0:
			
			### control the pitch ###
			
			# send order to tail #
			tp0=100/tw4
			tp1=150/tw4
			tp2=200/tw4
			tp3=150/tw4
			
			if onoff2[0]:
				if count4<tp0:
					tail_servo(0)
				elif count4>=tp0 and count4<(tp0+tp1):
					tail_servo(-20)
				elif count4<(tp0+tp1+tp2) and count4>=(tp0+tp1):
					tail_servo(0)
				elif count4>=(tp0+tp1+tp2) and count4<(tp0+tp1+tp2+tp3):
					if tailfix==0:
						pitemp = pitatal[0]/tp2
						tailfix=1
					if abs(pitemp-4)<10:
						tail_servo(0)
					else:
						tail_servo((pitemp-4)*20/90)
				elif count4==(tp0+tp1+tp2+tp3):
					pitatal[0] = 0
					tailfix = 0
					count4 =tp1 -1
				count4 +=1
			
			### control the Yaw (roll) ###
			defangle=Compass-phi                                                          #unit [deg]
			dist=0.1*(GPSx**2.0+GPSy**2.0)**0.5                              #unit [m]
			ctemp=100
			k=math.exp(-(0.5/ctemp)*dist**2)
			stangle=15*(1-k)    #straight angle
			wingangle=60
			 
			# change the cordinate #
			if defangle>180:
				defangle -= 360
			elif defangle<-180:
				defangle += 360
			print("defangle= "+str(defangle)+" stangle= "+str(stangle)+" dist= "+str(dist*10)+"count2= "+str(count2))
			
			deftatal += defangle
			
			# send order to aileron #
			tr1=70/tw4
			tr2=70/tw4
			
			if onoff2[0] and count2>4000*ailst:
				if count3<tr1:
					wing_servo(0)
				elif count3>=tr1 and count3<(tr1+tr2):
					if wingfix==0:
						deftemp = deftatal/tr1
						sttemp =  stangle
						wingfix=1
					if deftemp>sttemp:
						wing_servo(wingangle)
					elif deftemp<-sttemp:
						wing_servo(-wingangle)
					else:
						wing_servo(0)
					
				elif count3==(tr1+tr2):
					deftatal = 0
					wingfix = 0
					count3 = -1
				count3 +=1
				
			rotemp[0]=0
			pitemp3[0]=0
		## save the control log ##
		f4 = open(f4name,"a")
		f4.write("tail= "+str(logtail[0])+" deg"+" aileron="+str(logwing[0])+" deg"+" defangle= "+str(defangle) +" deg"+" deftemp= "+str(deftemp)+" deg"+" count1="+str(count1)+" count2="+str(count2)+" count3="+str(count3)+" count4="+str(count4)+"  The time is:"+time.ctime())
		f4.write("\n")
		f4.close()
		
			
		## brushless power max ##
		if onoff2[0]:
			count2 +=1
			
			if count2>120:
				pi.set_servo_pulsewidth(brushless_port,pw[0])
			else:
				pi.set_servo_pulsewidth(brushless_port,1000)
				
			
		## detect landing ##
		if count1%tw==0:
			value = [Accel[0], Accel[1], Accel[2], Altitude[0]]
			value_change(value)
			if count1 > sample:
				stop_rotating(slope, R_2, Altitude[0])
		
		## Emergency stop ##
		if dist*10>20000:
			if onoff2[0]:
				stopbrushless(500,pw[0])
				onoff2[0]=0
				landed[0]=0
				tail_servo(-50)
				f4 = open(f4name,"a")
				f4.write("Emergency stop!")
				f4.write("\n")
				f4.close()
		
		## get to goal ##
		if dist*10<45 and count2>4000*ailst:
			if onoff2[0]:
				stopbrushless(500,pw[0])
				onoff2[0]=0
				landed[0]=0
				tail_servo(-50)
				f4 = open(f4name,"a")
				f4.write("found the goal!")
				f4.write("\n")
				f4.close()
		
		count1 += 1


except KeyboardInterrupt:
        pass
    
finally:
	stopbrushless(500,pw[0])
	pi.bb_serial_read_close(RXD)
	f = open(fname,"a")
	f.close()
	f2 = open(f2name,"a")
	f2.close()
	f3 = open(f3name,"a")
	f3.close()
	f4 = open(f4name,"a")
	f4.close()
	f5 = open("magdata.txt","a")
	f5.close()
	
	tail_servo(0)
	wing_servo(0)
	#stop measure mode of ADXL345
	bus.write_byte_data(addr1,0x2d,0b00000000)
