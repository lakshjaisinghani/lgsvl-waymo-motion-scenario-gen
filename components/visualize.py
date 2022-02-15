import matplotlib.pyplot as plt
import numpy as np

class Plot:
    def find_color(self, road_type):
        if road_type == 'roadEdge':
            return 'k.'
        elif road_type == 'lane':
            return 'b.'
        elif road_type == 'roadLine':
            return 'r.'
        elif road_type == 'stopSign':
            return 'g.'
        elif road_type == 'crosswalk':
            return 'y.'
        else:
            return 'm.'

    def plot_map(self, parsed_scene):
        plt.figure(figsize=(20, 20), dpi=100)
        for map_feat in parsed_scene['mapFeatures']:
            road_type = list(map_feat.keys())[-1]
            print(road_type)
            if road_type in ['lane', 'roadEdge', 'roadLine']:
                poly = map_feat[road_type]['polyline']
            elif road_type in ['crosswalk', 'speedBump']:
                poly = map_feat[road_type]['polygon']
            else:
                continue
                
            for pnt in poly:
                x = pnt['x']
                y = pnt['y']
                plt.plot(x, y, self.find_color(road_type))
        plt.show()