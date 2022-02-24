import copy
from environs import Env
import numpy as np
import lgsvl

from components.scenario import Scenario
from components.object import Object
from components.visualize import Plot
from lgsvl.geometry import Transform, Vector


if __name__ == "__main__":
    # wod_file = "D:\Datasets\WaymoMotion\scenario\\1.tfrecord-00000-of-01000" 
    json_scenario = "./scenarios/scenario.json"
    scenario = Scenario(json_scenario)

    # print("ProjectVR simulation test: NPC following waypoints")
    # env = Env()
    # san_fran = '1f903cb2-477e-44aa-a8c0-a29af6d5e433'

    # sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
    # if sim.current_scene == san_fran:
    #     sim.reset()
    # else:
    #     sim.load(san_fran)

    
    # # LOAD EGO
    # ego_spawn = scenario.tracks[0].spawn
    # ego = sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_spawn)

    # # LOAD NPC
    # npc = scenario.tracks[1]
    # npc_sim = sim.add_agent("Sedan", npc.agent_type['lgsvl'], npc.spawn)

    # # agent goal
    # waypoints = npc.drive_waypoints

    # # When the NPC is within 0.5m of the waypoint, this will be called
    # def on_waypoint(agent, index):
    #     print("waypoint {} reached".format(index))


    # # The above function needs to be added to the list of callbacks for the NPC
    # npc_sim.on_waypoint_reached(on_waypoint)

    # # The NPC needs to be given the list of waypoints.
    # # A bool can be passed as the 2nd argument that controls whether or not the NPC loops over the waypoints (default false)
    # npc_sim.follow(waypoints)

    # input("Press Enter to run the simulation for 30 seconds")

    # sim.run(20)


