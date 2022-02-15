import copy
from environs import Env
import numpy as np
import lgsvl

from components.scenario import Scenario
from components.object import Object
from components.visualize import Plot
from lgsvl.geometry import Transform, Vector


if __name__ == "__main__":
    wod_file = "D:\Datasets\WaymoMotion\scenario\\1.tfrecord-00000-of-01000" 
    scenario = Scenario(wod_file)

    plt = Plot()
    plt.plot_map(scenario.scenario)

    # print("ProjectVR simulation test: NPC following waypoints")
    # env = Env()
    # flat_world = 'e6dc21da-0105-4534-a30d-c1939a8a4ff6'

    # sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
    # if sim.current_scene == flat_world:
    #     sim.reset()
    # else:
    #     sim.load(flat_world)

    # npc = scenario.tracks[0]
    # # LOAD EGO
    # ego_spawn = copy.deepcopy(npc.spawn)
    # ego_spawn.transform.position.x += 5
    # ego_spawn.transform.position.z += 5
    # ego = sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_spawn)

    # # LOAD NPC
    
    # print(npc.spawn.transform)
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

    # # # Creating state object to define state for Ego and NPC
    # # ego_spawn = lgsvl.AgentState()
    # # position = Vector(x=0, y=0.1, z=10000)
    # # ego_spawn.transform = Transform(position=position)

    # # # Ego
    # # ego_state = copy.deepcopy(ego_spawn)
    # # ego = sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, ego_spawn)

    # input("Press Enter to run the simulation for 30 seconds")

    # sim.run(20)


