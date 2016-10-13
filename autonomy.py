# Autonomy
#Raw file for Engagement system flight. Note: View in "raw" tab setting for an easier view           ^


from __future__ import division
import math
import msvcrt
import datetime
import dronekit_sitl
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import serial

#All imports and global variables needed for SIM run
global x
global y	
global z
global key
global correction_x
global correction_y
global correction_z
global trkx
global trky
key = 76
trkx = 7
trky = 7
correction_x = 0
correction_y = 0
correction_z = 0
accel = .9

#Connect to XBee
#ser = serial.Serial("com7", 9600, timeout = 0.5)


print "Start simulator (SITL)"
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()
# ^ Starts SIM

print("Connecting to vehicle on: %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)
print " Current-Location: Lat:%s, Lon:%s, Alt: %s" % (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt)
home_lat = float(vehicle.location.global_frame.lat)
int(home_lat)
home_lon = float(vehicle.location.global_frame.lon)
int(home_lon)
home_alt = 5
# ^ Connects SIM Drone and defines home Lat, Lon, and Alt for vehicle recovery



#print "Type Latitude"
#x=float(raw_input())
x=-35.3633
#int(x)
#print "Type Longitude"
#y=float(raw_input())
y=149.16522
#int(y)
#print "Type Altitude"
#z=float(raw_input())
z=7
#int(z)
# ^ Drone is waiting at the ready until coords are recieved

print "Arming"
time.sleep(2)
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True
	# Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)
# ^ Arms and rises to given Altitude

def send_global_velocity(velocity_x,velocity_y,velocity_z):
	msg = vehicle.message_factory.set_position_target_global_int_encode(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 0b0000111111000111,0,0,0,velocity_x,velocity_y,velocity_z,0,0,0,0,0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
# ^	Needed only for conditionyaw action

def conditionyaw(heading, relative = False):
	is_relative = 0
	# ^ states that heading is not relative to flight path
	send_global_velocity(0,0,0)
	msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,heading,0,1,is_relative, 0,0,0)
	vehicle.send_mavlink(msg)
# ^ Turns drone on z access


def gotoGPS(location):
	global x
	global y	
	global z
	global key
	global correction_x
	global correction_y
	global correction_z
	vehicle.simple_goto(location, groundspeed=20)
# ^ Flies drone to a given set of coords and alt

def tracking(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
# ^ Attempt to control drone directly from visual imputs


def track(heading):
	global trkx
	global trky
	if vehicle.heading == 0:
		trkx = -1
		trky = 0
	elif vehicle.heading < 15:
		trkx = -1
		trky = -.25
	elif vehicle.heading>=15 and vehicle.heading<=30:
		trkx = -1
		trky = -.5
	elif vehicle.heading>30 and vehicle.heading<37.5:
		trkx = -1
		trky = -.75
	elif vehicle.heading>=37.5 and vehicle.heading<=52.5:
		trkx = -1
		trky = -1
	elif vehicle.heading>52.5 and vehicle.heading<60:
		trkx = -.75
		trky = -1
	elif vehicle.heading>=60 and vehicle.heading<=75:
		trkx = -.5
		trky = -1	
	elif vehicle.heading>75 and vehicle.heading<90:
		trkx = -.25
		trky = -1
	elif vehicle.heading == 90:
		trkx = -0
		trky = -1
	elif vehicle.heading>90 and vehicle.heading < 105:
		trkx = .25
		trky = -1
	elif vehicle.heading>=105 and vehicle.heading<=120:
		trkx = .5
		trky = -1
	elif vehicle.heading>120 and vehicle.heading<127.5:
		trkx = .75
		trky = -1
	elif vehicle.heading>=127.5 and vehicle.heading<=142.5:
		trkx = 1
		trky = -1
	elif vehicle.heading>142.5 and vehicle.heading<150:
		trkx = 1
		trky = -.75
	elif vehicle.heading>=150 and vehicle.heading<=165:
		trkx = 1
		trky = -.5	
	elif vehicle.heading>165 and vehicle.heading<180:
		trkx = 1
		trky = -.25
	elif vehicle.heading == 180:
		trkx = 1
		trky = 0
	elif vehicle.heading>180 and vehicle.heading < 195:
		trkx = 1
		trky = .25
	elif vehicle.heading>=195 and vehicle.heading<=210:
		trkx = 1
		trky = .5
	elif vehicle.heading>210 and vehicle.heading<217.5:
		trkx = 1
		trky = .75
	elif vehicle.heading>=217.5 and vehicle.heading<=232.5:
		trkx = 1
		trky = 1
	elif vehicle.heading>232.5 and vehicle.heading<240:
		trkx = .75
		trky = 1
	elif vehicle.heading>=240 and vehicle.heading<=255:
		trkx = .5
		trky = 1	
	elif vehicle.heading>255 and vehicle.heading<270:
		trkx = .25
		trky = 1
	elif vehicle.heading == 270:
		trkx = 0
		trky = 1
	elif vehicle.heading > 270 and vehicle.heading< 285:
		trkx = -.25
		trky = 1
	elif vehicle.heading>=285 and vehicle.heading<=300:
		trkx = -.5
		trky = 1
	elif vehicle.heading>300 and vehicle.heading<307.5:
		trkx = -.75
		trky = 1
	elif vehicle.heading>=307.5 and vehicle.heading<=322.5:
		trkx = -1
		trky = 1
	elif vehicle.heading>322.5 and vehicle.heading<330:
		trkx = -1
		trky = .75
	elif vehicle.heading>=330 and vehicle.heading<345:
		trkx = -1
		trky = .5	
	elif vehicle.heading>345:
		trkx = -1
		trky = .25

		
#How to send data thru XBee
def send_full_data(Name, arg1, arg2, arg3):
	while True:
		ser.write("%s\n" % Name)
		time.sleep(.5)
		if Name == "MANUAL":
			vehicle.mode = VehicleMode("MANUAL")
			incoming = ser.readline().strip()
			if incoming == "Received":
				return
		time.sleep(.5)
		incoming = ser.readline().strip()
		if incoming == "Go":
			print "writing arg1"
			time.sleep(.5)
			ser.write("%s\n" % arg1)
			time.sleep(.5)
			becoming = ser.readline().strip()
			print becoming
			if becoming == "Received arg1":
				print "writing arg2"
				time.sleep(.5)
				ser.write("%s\n" % arg2)
				time.sleep(.5)
				#print "I got to here"
				becoming = ser.readline().strip()
				print becoming
				if becoming == "Received arg2":
					print "writing arg3"
					time.sleep(.5)
					ser.write("%s\n" % arg3)
					#print "I got to here"
					time.sleep(.5)
					becoming = ser.readline().strip()
					print becoming
					if becoming == "Received arg3":
						break
					print "woohoo"
					break
		if incoming == Name:
			print "I didn't get anything help me"
			break
						
#How to receive data thru XBee			
def rec_full_data(Name):
	while True:
		if ser.readline().strip() == Name:
			time.sleep(.5)
			print Name
			ser.write("%s\n" % Name)
			#Changemode settings
			if Name == "MANUAL":
				#vehicle.mode = VehicleMode("MANUAL")
				print "I would change my mode now"
				time.sleep(.5)
				ser.write("Received\n")
				return
				
			#ready for arg 1
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			if type(incoming) == str:
				try:
					arg1 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg1\n")
					print "Arg1 worked!"
					return arg1
				except ValueError, e:
					print "error", e
					arg1 = " "
			
			#ready for arg2		
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			if type(incoming) == str:
				try:
					arg2 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg2\n")
					print "Arg2 worked!"
					return arg2
				except ValueError, e:
					print "error", e
					arg2 = " "
			
			#ready for arg3
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			if type(incoming) == str:
				try:
					arg3 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg3\n")
					print "Arg3 worked!"
					return arg3
				except ValueError, e:
					print "error", e
					arg3 = " "
			
			
			return Name, arg1, arg2, arg3	

#How to receive chars thru XBee
def rec_char(Name):
	while True:
		if ser.readline().strip() == Name:
			time.sleep(.5)
			print Name
			ser.write("%s\n" % Name)
			time.sleep(.5)
			incoming = ser.readline().strip()
			print incoming
			if type(incoming) == str:
				try:
					arg1 = float(incoming)
					time.sleep(.5)
					ser.write("Received arg1\n")
					print "Arg1 worked!"
					return arg1
				except ValueError, e:
					print "error", e
					arg1 = " "
			return Name, arg1
		
#How to send chars thru XBee
def send_char(Name, arg1):
	while True:
		ser.write("%s\n" % Name)
		time.sleep(.5)
		incoming = ser.readline().strip()
		if incoming == "Go":
			print "writing arg1"
			time.sleep(.5)
			ser.write("%s\n" % arg1)
			time.sleep(.5)
			becoming = ser.readline().strip()
			print becoming
			if becoming == "Received arg1":
				return
		if incoming == Name:
			print "I didn't get anything, help me!"
			break
#Full Action Code Starts Below


#Get WP and enemy's WP
"""
while True:
	ser.write("WP\n")
	time.sleep(1)
	bc = ser.readline().strip()
	if bc == "Go":
		[Name, x, y, z] = rec_full_data("WP")
		break
print "Enemy Latitude is: ", x
print "Enemy Longitude is: ", y
print "Enemy Altitude is: ", z
print "Type is: ", Name

while True:
	ser.write("EnemyWP\n")
	time.sleep(1)
	bc = ser.readline().strip()
	if bc == "Go":
		[Name, elat, elon, ealt] = rec_full_data("EnemyWP")
		break
print "Enemy Latitude is: ", elat
print "Enemy Longitude is: ", elon
print "Enemy Altitude is: ", ealt
print "Type is: ", Name
"""

arm_and_takeoff(5)
# ^ Arms and climbs to 5 meters	
time.sleep(3)	

print "Going to Waypoint"		
while True:
	global tgtx
	global tgty
	a = float(x - vehicle.location.global_relative_frame.lat)
	b = float(y - vehicle.location.global_relative_frame.lon)		
	c = float(z - vehicle.location.global_relative_frame.alt)
	int(a)
	int(b)
	int(c)
		
	newLoc = LocationGlobal (vehicle.location.global_frame.lat + a + correction_x, vehicle.location.global_frame.lon + b + correction_y, vehicle.location.global_frame.alt + c + correction_z)
	gotoGPS(newLoc)		
	
	print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
	print " Enroute to Lat:%s, Lon:%s, Alt:%s" % (x,y,z)
	time.sleep(3)
	print " Check Current Alt:%s" % vehicle.location.global_relative_frame.alt
	print " Heading: {}".format(vehicle.heading)
	print " Distance to Waypoint: %s" % (math.sqrt(((x*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*y)-((100000*vehicle.location.global_relative_frame.lon)))**2+(z-(vehicle.location.global_relative_frame.alt))**2))
	time.sleep(3)
	
#	if (math.sqrt(((x*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*y)-((100000*vehicle.location.global_relative_frame.lon)))**2+(z-(vehicle.location.global_relative_frame.alt))**2))<10:
#######Change, above to get tgt coords
#		La = vehicle.location.global_relative_frame.lat
#		la = vehicle.location.global_relative_frame.lon
#		Lb = tgtx
#		lb = tgty
#		U = math.cos(math.radians(Lb))*math.sin(math.radians(lb-la))
#		T = math.cos(math.radians(La))*math.sin(math.radians(Lb))-math.sin(math.radians(La))*math.cos(math.radians(Lb))*math.cos(math.radians(lb-la))
#		Bearing = math.atan2(U,T)
#		Hdg = math.degrees(Bearing)
#		conditionyaw(Hdg)
#######the above is the distance formula for two points, used to know and turn in the direction of tgt A/C
	"""
	while True:
		ser.write("EnemyWP\n")
		time.sleep(1)
		bc = ser.readline().strip()
		if bc == "Go":
			[Name, elat, elon, ealt] = rec_full_data("EnemyWP")
			break
	print "Enemy Latitude is: ", elat
	print "Enemy Longitude is: ", elon
	print "Enemy Altitude is: ", ealt
	print "Type is: ", Name
	"""
	print "Do you see the threat? Y/N?"
	key=msvcrt.getch()
	if key == "Y" or key == "y":
		d1 = 10# <-- replace value with -> math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))
		#get coords of tgt drone
		start = time.time()
		time.sleep(1)
		d2 = 15# <-- replace value with -> math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))
		#get updated coords of tgt drone
		end = time.time()
		tgt = d2 - d1
		vel = (tgt/(end-start))*.90
		print trkx
		print trky
		print vel
		#tracking(vel*trkx, vel*trky, 0, 5)
		print "Switching to manual in..."
		count = 5
		while not count == 0:
			print count 
			count = count-1
			time.sleep(1)
		print "Good Luck!"
		break
	
	else:# Can't find target and gets updated coords
		
		print "Type Latitude"
		x=float(raw_input())
		int(x)
		print "Type Longitude"  #######Change so that coords are not manual inputs
		y=float(raw_input())
		int(y)
		print "Type Altitude"
		z=float(raw_input())
		int(z)
		"""
		while True:
			ser.write("WP\n")
			time.sleep(1)
			bc = ser.readline().strip()
			if bc == "Go":
				[Name, x, y, z] = rec_full_data("WP")
				break
		print "Enemy Latitude is: ", x
		print "Enemy Longitude is: ", y
		print "Enemy Altitude is: ", z
		print "Type is: ", Name
		"""
		
print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)	
print " Heading: {}".format(vehicle.heading)

while True:
	global tgtx
	global tgty
	global vel
	global yaw
	global turn
	global accel
	#time.sleep(3)
	#time.sleep(3)
	print "Key"
	key=msvcrt.getch() # This command may be moved to the bottom of the while statement since we already get one command prior to the ask here
	"""
	while True:
			ser.write("Key\n")
			time.sleep(1)
			bc = ser.readline().strip()
			if bc == "Go":
				[Name, key] = rec_char("Key")
				break
	print key
	"""
	yaw = vehicle.heading
	heading = vehicle.heading
	d1 = 10# <-- replace value with -> math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))
	#get coords of tgt drone
	start = time.time()
	time.sleep(1)
	d2 = 15# <-- replace value with -> math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))
	#get updated coords of tgt drone
	end = time.time()
	tgt = d2 - d1
	vel = (tgt/(end-start))*accel
	turn = 25
	
	print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
	#if math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))<10 and math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))>5:
		#turn = 15
		#time = 1
	#elif math.sqrt(((tgtx*100000)-((100000*vehicle.location.global_relative_frame.lat)))**2+((100000*tgty)-((100000*vehicle.location.global_relative_frame.lon)))**2+(tgtz-(vehicle.location.global_relative_frame.alt))**2))<5:
		#turn = 5
		#time = .5
	if key == "a": #Change key so that the value here represents the degrees to yaw ex. key -1 is 1 degree left while (+)2 is 2 degrees right
		conditionyaw(yaw-turn)
		time.sleep(2)
		print " Heading: {}".format(vehicle.heading)
		track(heading)
		#print trkx
		#print trky
		#print vel
		#tracking(vel*trkx, vel*trky, 0, 1)
		
	elif key == "d":
		conditionyaw(yaw+turn)
		time.sleep(2)
		print " Heading: {}".format(vehicle.heading)
		track(heading)
		#print trkx
		#print trky
		#print vel
		#tracking(vel*trkx, vel*trky, 0, 1)
		print vehicle.location.global_relative_frame.alt	
	elif key == "w":
		print " Heading: {}".format(vehicle.heading)
		accel = accel-.1
		vel = (tgt/(end-start))*accel
		#tracking(vel*trkx, vel*trky, 0, 1)
		time.sleep(.5)
		print vel
		
		
	elif key == "s":
		print " Heading: {}".format(vehicle.heading)
		accel = accel+.1
		vel = (tgt/(end-start))*accel
		#tracking(vel*trkx, vel*trky, 0, 1)
		time.sleep(.5)
		print vel
		

	elif key == "z":
		#print " Heading: {}".format(vehicle.heading)
		newLoc = LocationGlobal (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt+1)
		gotoGPS(newLoc)
		#tracking(vel*trkx, vel*trky, 0, 1)
		time.sleep(5)
		print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		#tracking(vel*trkx, vel*trky, 0, 1)	
		#time.sleep(2)
		

	elif key == "x":
		#print " Heading: {}".format(vehicle.heading)
		newLoc = LocationGlobal (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.location.global_frame.alt-1)
		gotoGPS(newLoc)
		#tracking(vel*trkx, vel*trky, 0, 1)
		time.sleep(5)
		print " Current Location: Lat:%s, Lon:%s, Alt:%s" % (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.alt)
		#tracking(vel*trkx, vel*trky, 0, 1)	
		#time.sleep(2)
	
	elif key == "p":
		break
	tracking(vel*trkx, vel*trky, 0, 1)
# Ending Sim command
sitl.stop()
