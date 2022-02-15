import os
import sys
import inspect

import lgsvl
from lgsvl.agent import AgentType, AgentState
from lgsvl.geometry import Transform, Vector
from lgsvl.utils import rad2deg

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from waymo_open_dataset.protos import scenario_pb2

class Object:
    def __init__(self, tracks, is_ego=False):
        self.id          = tracks['id']
        self.states      = tracks['states']
        self.is_ego      = is_ego 

        # agent type for both lgsvl and waymo
        self.agent_type = self.define_agent_type(tracks)

        # create drive waypoints and assign spawn
        self.spawn = lgsvl.AgentState()
        self.spawn.transform = self.state_to_transform(self.states[0])
        self.drive_waypoints = self.create_DriveWaypoints()
    
    def state_to_transform(self, state):
        pos = Vector(x=state['centerY'], y=0.1, z=state['centerX'])
        # pos *= 0.10

        # TODO: add angle due to elevation
        rot = Vector(x=0, y=rad2deg(state['heading']), z=0)
        return Transform(position=pos, rotation=rot)
    
    def create_DriveWaypoints(self):
        waypoints = []
        for i, state in enumerate(self.states):
            transform = self.state_to_transform(state)
            if i == 0:
                continue
            wp = lgsvl.DriveWaypoint(transform.position, 5, angle=transform.rotation, idle=0)
            waypoints.append(wp)     
        return waypoints

    def define_agent_type(self, track):
        agent_type = {}
        agent_type['waymo'] = track['objectType']

        if agent_type['waymo'] == 'TYPE_VEHICLE' and self.is_ego:
            agent_type['lgsvl'] = lgsvl.AgentType.EGO
        if agent_type['waymo'] in ['TYPE_VEHICLE', 'TYPE_CYCLIST', 'TYPE_OTHER']:
            agent_type['lgsvl'] = lgsvl.AgentType.NPC
        elif agent_type['waymo'] == 'TYPE_PEDESTRIAN':
            agent_type['lgsvl'] = lgsvl.AgentType.PEDESTRIAN
        else:
            raise TypeError(f'The type of vehicle with id {self.id} is unset')
        return agent_type

    # TODO
    def state_to_speed(self, state):
        pass

    def create_vehicle_model(self):
        # dict that has uuid
        pass
