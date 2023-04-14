
"""
Created on Wed Mar 16

@author: Dennis Kibalama (kibalama.3)
"""

from __future__ import absolute_import
from __future__ import print_function
from termcolor import colored

import os
import sys
import numpy as np

# check SUMO_HOME 
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from sumolib import net, checkBinary

class SumoEnv:
    """
    Class to define the SUMO environment 
    """
    def __init__(self,
        map_area : str,
        use_gui = True,
        config_file = "traffic_sim.sumocfg"
    ):
        # super(SumoEnv,self).__init__()
        self.sumo_map = map_area
        self.cwd = os.getcwd()
        self.config_file = config_file
    
        self.sumo_config = (f"{self.cwd}/env/traffic_model/{self.sumo_map}/{self.config_file}")
        self.net_file = net.readNet(f"{self.cwd}/env/traffic_model/{self.sumo_map}/{self.sumo_map}.net.xml")
        self.use_gui = use_gui
        self.step = 0
        
        # self.sumo_config = self.cwd + "/env/traffic_model/" + self.sumo_map + "/" + self.config_file

        # initialize the host vehicle
        self.host_id = "host0"
        self.host_vehicle_present = False
        self.delta_t = 1 # simulation time step [secs]
        self.vehicle_velocity = 0

        # SUMO states
        self.distance_travelled = 0
        self.distance_remained = 0
        self.current_position = []
        self.current_speed_limit = 0
        self.next_speed_limit = 0
        self.next_speed_limit_distance = 1000
        self.next_TLS = []

        # self.next_tls_id = []
        self.next_tls_distance = 0
        self.next_tls_state = "g"

        # Traffic light information
        self.tls_list = []
        self.last_next_tls_program = []
        self.last_next_tls_link_num = []
        self.next_tls_info = None
        self.next_tls_index = []
        

        # Route information
        self.edge_list = []
        self.edge_length_list = []
        self.departure_position = []
        self.speed_limit_list = []
        self.current_edge_id = []
        self.current_edge_index = 0

        self.done = False # tag for simulation complete status
        self.t = -1
        self.time = 0

    def _sumo_config_file(self):
        # configuration file needed for SUMO (if not alreadyt created)
        pass

    def reset(self):
        self.step = 0

        # Reset the simulation
        try:
            traci.close(False)
        except traci.exceptions.FatalTraCIError:
            print(colored("TraCi already closed", "yellow"))

    
    def setup_sumo(self):
        """
        If the user wants to use the GUI, then the sumo_binary is set to the path of the sumo-gui.exe file.
        If the user doesn't want to use the GUI, then the sumo_binary is set to the path of the sumo.exe
        file.
        This code accounts for linux, mac and assumes the only other platform is Windows, if running on 
        another platform, consider adding the <platform value> to the included list
        """

        if self.use_gui:
            if sys.platform in ['linux','Darwin']:
                self.sumo_binary = "/usr/share/sumo/bin/sumo-gui"
            else:
                self.sumo_binary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe" 
        #     self.sumo_binary = (
        #         "/usr/share/sumo/bin/sumo-gui"
        #         if sys.platform in ['linux', 'Darwin', 'darwin']
        #         else r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
        #    )
        else:
            if sys.platform in ['linux','Darwin']:
                self.sumo_binary = "/usr/share/sumo/bin/sumo"
            else:
                self.sumo_binary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo.exe"
        #     self.sumo_binary = (
        #         "/usr/share/sumo/bin/sumo"
        #         if sys.platform in ['linux', 'Darwin', 'darwin']
        #         else r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo.exe"
        #    )

    
    def start_sumo(self):
        """
        If TraCI is loaded, close the connection, then start SUMO via TraCI.
        """

        self.sumo_command = [
            self.sumo_binary,
            "-c",
            self.sumo_config,
            "--tripinfo-output",
            "tripinfo.xml",
            "--time-to-teleport",
            "-1",
            "--no-warnings",
        ]

        if traci.isLoaded():
            traci.close(False) # close a TraCi connection if TraCI is Loaded

        traci.start(self.sumo_command) # make call to start SUMO via TraCI


    def get_state(self):
        """
        The function gets the current speed of the vehicle, the current speed limit, the next speed
        limit and the distance to the next speed limit change
        :return: The current state of the vehicle.
        """

        self.vehicle_velocity = traci.vehicle.getSpeed(self.host_id)

        current_edge_id = traci.vehicle.getRoadID(self.host_id)

        if current_edge_id not in self.edge_list:
            current_edge_index = self.current_edge_index + 1
        else:
            current_edge_index = self.edge_list.index(current_edge_id)
            self.current_edge_index = current_edge_index
        current_speed_limit = self.speed_limit_list[current_edge_index]
        speed_limit_change = [
            (self.speed_limit_list[i] != current_speed_limit)
            for i in range(current_edge_index, len(self.edge_list))
        ]

        if any(speed_limit_change):
            next_speed_limit_index = current_edge_index + speed_limit_change.index(True)
            next_speed_limit = self.speed_limit_list[next_speed_limit_index]
            next_speed_limit_distance = traci.vehicle.getDrivingDistance(
                self.host_id,
                self.edge_list[next_speed_limit_index - 1],
                self.edge_length_list[next_speed_limit_index -1] - 0.5
            )
            if(next_speed_limit_distance > self.total_distance or next_speed_limit_distance < 0):
                next_speed_limit_distance = 0
        else:
            next_speed_limit = current_speed_limit
            next_speed_limit_distance = (self.total_distance - self.distance_travelled - self.vehicle_velocity)

        # Current speed limit, next speed limit and distacne to next speed limit change
        self.current_speed_limit = current_speed_limit
        self.next_speed_limit = next_speed_limit
        self.next_speed_limit_distance = max(0,next_speed_limit_distance)

        return[
            self.vehicle_velocity,
            self.current_speed_limit,
            self.next_speed_limit,
            self.next_speed_limit_distance
        ]

    def get_traffic_light_location(self):

        '''
        The function is designed to collect the traffic light locations along the 
        route. It is designed to be called only once when the ego vehicle has just spawned
        in the simulation. 
        
        It iterates through the length of the list returned by "traci.vehicle.getNextTLS(self.host_id)".
        This list changes length as the simulation progresses.
        
        Output argument
        :return: tl_route, the location of traffic lights along the route in meters 
        '''

        next_tls = traci.vehicle.getNextTLS(self.host_id)

        tls = len(next_tls)

        tl_route = [next_tls[tl][2] for tl in range(tls)]

        self.tl_route = tl_route

        print(colored(f"Traffic Light Locations [m]: {np.around(self.tl_route, decimals=2)}", "red"))

        return[
            self.tl_route
        ]

    def get_stop_sign_location(self):

        '''
        The function is designed to collect the stops in the simulation. The stops have 
        various stopFlags that define the type of stop. 

        Current version: Designed to locate stops only
        The stopFlag identifier can represent 
        1: stops
        2: parking
        4: person triggered
        8: container triggered
        16: is bus stop
        32: is container stop
        64: charging station
        128: parking area

        NOTE: Performs call to deprecated function getNextStops()
        '''

        next_stops = traci.vehicle.getNextStops(self.host_id)
        print(len(next_stops))



    def get_observations(self):
        self.distance_covered = traci.vehicle.getDistance(self.host_id)
        # states related to the vehicle itself
        veh_spd = traci.vehicle.getSpeed(self.host_id)
        veh_acc = traci.vehicle.getAcceleration(self.host_id)

        # Extract the lat-long coordinates of the route
        # self.lat_long = traci.simulation.convert2D(self.host_id, self.current_edge_id, self.current_position, toGeo=True)

        # Collect the position of the named vehicle (x,y) within the last step
        x, y = traci.vehicle.getPosition(self.host_id)
        # Extract the geo-coordinates (lat-long) of the ego vehicle
        self.long, self.lat = traci.simulation.convertGeo(x,y)

        # Rudimentary implementation to acquire traffic light information
        # next_tls = traci.vehicle.getNextTLS(self.hostID)

        # if next_tls:= traci.vehicle.getNextTLS(self.host_id):
        #     self.next_tls_id = next_tls[0][0]
        #     self.next_tls_distance = next_tls[0][2]
        #     self.next_tls_state = next_tls[0][3]

        # else:
        #     self.next_tls_id = 0
        #     self.next_tls_distance = self.total_distance - self.distance_travelled - self.vehicle_velocity
        #     self.next_tls_state = "g"

        '''Additional section for collecting traffic light information'''

        next_tls = traci.vehicle.getNextTLS(self.host_id)
        link_num = -1
        if next_tls:
            next_tls_id = next_tls[0][0]
            next_tls_distance = next_tls[0][2]
            next_tls_state = next_tls[0][3]
            # next_tls_program = traci.trafficlight.getCompleteRedYellowGreenDefinition(next_tls_id)  
            next_tls_program = traci.trafficlight.getAllProgramLogics(next_tls_id)   
            link_num = next_tls[0][1]
            next_tls_current_phase = traci.trafficlight.getPhase(next_tls_id)
            next_tls_next_switch = traci.trafficlight.getNextSwitch(next_tls_id) - traci.simulation.getTime()
            
            if next_tls_state == 'r':
                passing_period_start = next_tls_next_switch + 1

                if next_tls_current_phase + 1 == len(next_tls_program[0].phases):
                    iter_phase = 0
                else:
                    iter_phase = next_tls_current_phase + 1

                while (
                    next_tls_program[0].phases[iter_phase].state[link_num] != "g"
                    and next_tls_program[0].phases[iter_phase].state[link_num] != "G"
                ):
                    passing_period_start += (
                        next_tls_program[0].phases[iter_phase].duration
                    )
                    if iter_phase + 1 == len(next_tls_program[0].phases):
                        iter_phase = 0
                    else:
                        iter_phase += 1

                passing_period_end = passing_period_start
                while next_tls_program[0].phases[iter_phase].state[link_num] != "r":
                    passing_period_end += (
                        next_tls_program[0].phases[iter_phase].duration
                    )
                    if iter_phase + 1 == len(next_tls_program[0].phases):
                        iter_phase = 0
                    else:
                        iter_phase += 1
            

            elif next_tls_state == "y":
                has_red = False
                for i in range(len(next_tls_program[0].phases)):
                    if next_tls_program[0].phases[i].state[link_num] == "r":
                        has_red = True
                if has_red:
                    passing_period_start = 0
                    passing_period_end = next_tls_next_switch + 1
                else:
                    passing_period_start = 0
                    passing_period_end = 100

            
            elif next_tls_state == "g" or next_tls_state == "G":
                passing_period_start = 0

                has_red = False
                for i in range(len(next_tls_program[0].phases)):
                    if next_tls_program[0].phases[i].state[link_num] == "r":
                        has_red = True
                if has_red:
                    passing_period_end = next_tls_next_switch + 1

                    if next_tls_current_phase == len(next_tls_program[0].phases):
                        iter_phase = 0
                    else:
                        iter_phase = next_tls_current_phase + 1

                    while next_tls_program[0].phases[iter_phase].state[link_num] != "r":
                        passing_period_end += (
                            next_tls_program[0].phases[iter_phase].duration
                        )
                        if iter_phase + 1 == len(next_tls_program[0].phases):
                            iter_phase = 0
                        else:
                            iter_phase += 1
                else:
                    passing_period_end = 100

            offset = 0
            for i in range(next_tls_program[0].currentPhaseIndex):
                offset += next_tls_program[0].phases[i].duration
            total_switch_time = (
                next_tls_program[0]
                .phases[next_tls_program[0].currentPhaseIndex]
                .duration
            )

            offset += total_switch_time - next_tls_next_switch

            next_tls_info = {
                "Phases": [
                    phase.state[link_num] for phase in next_tls_program[0].phases
                ],
                "Duration": [phase.duration for phase in next_tls_program[0].phases],
                "Distance": next_tls_distance,
                "Offset": offset,
            }

        else:
            next_tls_distance = self.total_distance - self.distance_travelled - self.vehicle_velocity
            next_tls_state = "g"
            passing_period_start = 0
            passing_period_end = 100
            next_tls_program = []
            next_tls_info = {
                "Phases": ["g"],
                "Duration": [100],
                "Distance": 1000,
                "Offset": 0,
            }

        if (
            next_tls_distance > self.next_tls_distance
            and self.step != 0
            and self.next_tls_distance < 30
            and len(self.last_next_tls_program) != 0
        ):
            total_duration = sum(self.next_tls_info["Duration"])
            tmp = (self.next_tls_info["Offset"] - self.step) % total_duration
            tls_info = {
                "Phases": self.next_tls_info["Phases"],
                "Duration": self.next_tls_info["Duration"],
                "Distance": self.distance_travelled,
                "Offset": tmp,
            }
            self.tls_list.append(tls_info)

        # self.next_tls_index = next_tls_index
        self.next_tls_distance = next_tls_distance
        self.next_tls_state = next_tls_state
        self.next_tls_info = next_tls_info
        self.distance_travelled += veh_spd
        self.distance_remained = self.total_distance - self.distance_travelled
        self.last_next_tls_program = next_tls_program
        self.last_next_tls_link_num = link_num

        return [
            self.distance_covered,
            self.next_tls_distance,
            self.next_tls_state,
            self.long,
            self.lat,
            self.tls_list
        ]


    def get_edge_attributes(self):
        edge_list = self.edge_list
        edge_length_list = self.edge_length_list

        return[
            edge_list,
            edge_length_list
        ]

    def simulation_step(self):

        if self.host_vehicle_present:
            self.time += self.delta_t
            

            # One step forward update in SUMO
            traci.simulationStep()
            # self.get_traffic_light_location()

            # print(str(self.t) + ' host vehicle = ' + str(self.host_vehicle_present))
            self.t +=  1
            self.step += 1

            # Checking if host vehicle is still present
            if self.host_id in traci.vehicle.getIDList():
                self.host_vehicle_present = True

                self.edge_list = traci.vehicle.getRoute(self.host_id)
                self.edge_length_list = [self.net_file.getEdge(self.edge_list[i]).getLength()
                for i in range(len(self.edge_list))]

                self.speed_limit_list = [self.net_file.getEdge(self.edge_list[i]).getSpeed()
                for i in range(len(self.edge_list))]

                # calculate the total distance
                self.total_distance = traci.vehicle.getDrivingDistance(
                    self.host_id, self.edge_list[-1], self.edge_length_list[-1] - 0.5
                )

            else:
                self.host_vehicle_present = False
                self.done = True
                print("Simulation done!")
                # traci.close(False)

                # print(str(self.t) + ' host vehicle = ' + str(self.hostVehiclePresent))

        else:
            # Run SUMO until the host vehicle spawns
            while not(self.host_vehicle_present):
                traci.simulationStep()

                # print(str(self.t)+ ' host present = '+ str(self.hostVehiclePresent))
                self.t += 1
                self.step += 1

                # Check for spawning of the host vehicle
                if self.host_id in traci.vehicle.getIDList():
                    print("Vehicle " + self.host_id + " spawned!!")
                    self.host_vehicle_present = True
                    self.edge_list = traci.vehicle.getRoute(self.host_id)
                    self.edge_length_list = [self.net_file.getEdge(self.edge_list[i]).getLength()
                    for i in range(len(self.edge_list))]

                    self.speed_limit_list = [self.net_file.getEdge(self.edge_list[i]).getSpeed()
                    for i in range(len(self.edge_list))]

                    # calculate the total distance
                    self.total_distance = traci.vehicle.getDrivingDistance(
                        self.host_id, self.edge_list[-1], self.edge_length_list[-1] - 0.5
                    )

                    if self.use_gui:
                        traci.gui.setZoom('View #0', 75*traci.gui.getZoom());
                        traci.gui.trackVehicle('View #0', self.host_id);

                    # Enforce the departure speed of the host as 0 m/s
                    traci.vehicle.setSpeed(self.host_id, -1)