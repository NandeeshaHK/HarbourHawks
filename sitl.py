#!/usr/bin/env python3
import os
import asyncio
import numpy as np
import pandas as pd
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from pymavlink import mavwp
from pymavlink import mavutil
from math import cos,sin,atan2,radians,sqrt

# set the directory where the .waypoints files are located
waypoints_dir = '/home/nerdnhk/HarbourHawks/waypoints'

# set the directory where the output .csv files will be saved
csv_dir = '/home/nerdnhk/HarbourHawks/waypoints/csv'

wp_list = []
wp_file_list = []
file_dis_list = []
csv_file_list = []
curr_lat = np.float32(0)
curr_lon = np.float32(0)
alt = np.float32(0)
wpf_final = ""


# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
wp = mavwp.MAVWPLoader()  
    
#connection initiating 
def init_conn(the_connection):
    # Wait for the first heartbeat 
    print("Waiting For HeartBeat")
    the_connection.wait_heartbeat()
    # This sets the system and component ID of remote system for the link
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    #print("System is armed.")
    msg = the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking = True)
    print(msg)
    curr_lat = (msg.lat)/1e7
    curr_lon = (msg.lon)/1e7
    return (curr_lat,curr_lon)
#to get home gps 
async def get_home_gps(drone):
    print("Waitng to ARM the drone\n")
    async for terrain_info in drone.telemetry.home():
        curr_lat = terrain_info.latitude_deg
        curr_lon = terrain_info.longitude_deg
        break
    print("Drone is ARMED for 5 secs")
    print(curr_lat,curr_lon)
    return (curr_lat,curr_lon)

# to calculate distance btw two lat,lons
def distance_lat_lon(lat1, lon1, lat2, lon2):
    '''distance between two points'''
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)
    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(abs(a)), sqrt(abs(1.0-a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist

#to convert .waypoints to csv
def wpToCSV(waypoints_dir):
    for filename in os.listdir(waypoints_dir):
        if filename.endswith('.waypoints'):
            #uncomment below once to create a new directory, then uncomment it
            #os.mkdir(csv_dir)
            #time.sleep(2)
            
            # read the file into a pandas dataframe
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['seq', 'current','frame', 'command','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','mission_type'])

            # construct the output filename
            output_filename = os.path.splitext(filename)[0] + '.csv'

            # write the selected columns to a new .csv file
            df.to_csv(os.path.join(csv_dir, output_filename), index=False)

# to extract gps from csv file
def ext_gps(csv_file_path,gps):
    # Extract 3 columns from CSV file
    df = pd.read_csv(csv_file_path, usecols=['command', 'latitude','longitude'])
    f_lat = np.float32(0)
    f_lon = np.float32(0)
    
    for i in range(1,3):
        row = df.loc[i, :]
        if row['command'] !=22 and row.loc['latitude'] != 0.0 and row.loc['latitude'] !=0.0:
            f_lat = row.loc['latitude']
            f_lon = row.loc['longitude']
            break
    dis = distance_lat_lon(f_lat,f_lon,np.float32(gps[0]),np.float32(gps[1]))
    print("Found distance is "+str(dis/1000)+"km")
    file_dis_list.append(dis)
    

# to find the shortest distance
def final_wpf(GPS):
    file_name_list = os.listdir(csv_dir)
    print(file_name_list)
    for file_name in file_name_list:
        csv_file_path = os.path.join(csv_dir,file_name)
        if os.path.isfile(csv_file_path):
            print(file_name+" CSV file found")
            (ext_gps(csv_file_path,GPS))
        else:
            print("CSV file not found")
    #to find minium distance WP file
    min_dis_wpf = min(file_dis_list)
    # convert Pandas Series for easy conversion
    pd_series = pd.Series(file_dis_list)
    min_dis_index = pd_series.index[pd_series == min_dis_wpf][0]
    print("The shortest distance wp file:"+str(min_dis_index))
    # now file _name_list consists only waypoints
    file_name_list = os.listdir(csv_dir)
    wpf_final = file_name_list[min_dis_index]
    print("Final WP file seleted: "+wpf_final)

    # delete the directory
    file_name_list = os.listdir(csv_dir)
    file_name_list.remove(wpf_final)
    print("after deletion of csv files excluded req"+str(file_name_list))
    for file_name in file_name_list:
        csv_file_path = os.path.join(csv_dir,file_name)
        os.remove(csv_file_path)
    return wpf_final
'''
GPS = init_conn(the_connection)
wpToCSV(waypoints_dir)
wpf_final = final_wpf(GPS)
'''
#to upload mission/home/nerdnhk/pymavlink/csv_files/
async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    usr_read = 'n'
    #wait for usr input
    while usr_read != 'y':
        usr_read = input("-- Is drone is ready to be armed?(y/n):")
        if usr_read == 'y':
            wpToCSV(waypoints_dir)
            await drone.action.arm()
            print("-- Drone is ARMED")
    

            #print("-- Changed GPS coordinates for Simulation: ")
            GPS = init_conn(the_connection)
            print("-- Coordinates of drone: lat:"+str(GPS[0])+" and lon:"+str(GPS[1]))
            wpf_final = final_wpf(GPS)
        else:
            asyncio.sleep(0.1)
    
    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []
    final_path = os.path.join(csv_dir, wpf_final)
    print("CSV file Path:"+str(final_path))
    # final_path = "/home/nerdnhk/controlsystems/college.csv"
    speed = int(input("Input Desired Drone Speed(m/s):"))
    # to make mission items 
    with open(final_path) as f:
        next(f)  # Skip the first line
        next(f)  # Skip the second line
        for i,line in enumerate(f):
            fields = line.strip().split(",")
            seq = int(fields[0])
            latitude = float(fields[8])
            longitude = float(fields[9])
            altitude = float(fields[10])
            print(f"Waypoint: ({seq}, {latitude}, {longitude}, {altitude})")
            mission_items.append(MissionItem(latitude,
                                     longitude,
                                     altitude,
                                     speed,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    
    #async for flight_mode in drone.telemetry.flight_mode():
        #print("FlightMode:", flight_mode)    
    #await print_flight_mode(drone)
    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    await asyncio.sleep(2)
    
    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

async def print_flight_mode(drone):
    """ Prints the flight mode when it changes """

    previous_flight_mode = None

    async for flight_mode in drone.telemetry.flight_mode():
        if flight_mode != previous_flight_mode:
            previous_flight_mode = flight_mode
            print(f"Flight mode: {flight_mode}")

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")

def changetoauto(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     176, 0, 1, 3, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == 0:
        print("Command 'AUTO' executed successfully")
    else:
        print("Command failed with error code:", msg.result)
        
async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False
    async for terrain_info in drone.telemetry.home():
        curr_lat = terrain_info.latitude_deg
        curr_lon = terrain_info.longitude_deg
        break
    
    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air
        msg = the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking = True)
        curr_lat = (msg.lat)/1e7
        curr_lon = (msg.lon)/1e7
        print(curr_lat,curr_lon)
        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

# Ending connection
the_connection.close()