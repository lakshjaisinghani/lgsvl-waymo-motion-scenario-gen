import os
import sys
import inspect
import json

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from google.protobuf import json_format
from waymo_open_dataset.protos import scenario_pb2
from .object import Object

class Scenario:
    def __init__(self, scenario_path):
        
        self.scenario = None
        if scenario_path.endswith('.json'):
            self.scenario = self.load_json_scenario(path=scenario_path)

            # scenario information
            self.scenario_id        = 0
            self.map                = self.scenario['map']
            self.tracks             = self.load_objects(self.scenario['agents'], VSE=True)

        else:
            self.scenario = self.load_waymo_scenario(path=scenario_path)

            # scenario information
            self.scenario_id        = self.scenario['scenarioId']
            self.timestamp_seconds  = self.scenario['timestampsSeconds']
            self.dynamic_map_states = self.scenario['dynamicMapStates']
            self.map_features       = self.scenario['mapFeatures']
            self.sdc_track_index    = self.scenario['sdcTrackIndex']
            self.tracks             = self.load_objects(self.scenario['tracks'], VSE=False)

    def load_waymo_scenario(self, path):
        # load tf record data
        scenario_msg = scenario_pb2.Scenario()
        dataset = tf.data.TFRecordDataset(path, compression_type='')
        data = next(dataset.as_numpy_iterator())

        # stored data in scenario message class
        sucess = scenario_msg.ParseFromString(data)
        parsed_dict = json_format.MessageToDict(scenario_msg) # convert to dict
        return parsed_dict

    def load_json_scenario(self, path):
        # Opening JSON file
        f = open(path)  
        data = json.load(f)
        return data
    
    def load_objects(self, tracks, VSE=True):
        tmp_tracks = []

        if not VSE:
            for idx, track in enumerate(tracks):
                is_ego = True if idx == self.sdc_track_index else False
                tmp_tracks.append(Object(track, is_ego=is_ego))
                break
            return tmp_tracks
        else:
            print(tracks)
            for agent in tracks:
                if agent['variant'] in ['Jaguar2015XE']:
                    is_ego = True
                    tmp_tracks.append(Object(track, is_ego=is_ego, VSE=VSE))
                else:
                    is_ego = False
                    tmp_tracks.append(Object(track, is_ego=is_ego, VSE=VSE))
            return tmp_tracks
        