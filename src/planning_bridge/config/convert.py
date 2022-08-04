import json
import os
import numpy as np

rotation_z = 135 * np.pi / 180
scale_y = 1.0925

translation = np.array([-32.5326, -18.6715, 0])

current_folder = os.path.dirname(os.path.abspath(__file__)) 
original_name = current_folder + '/waypoints.json'
dict_name = current_folder + '/waypoints_dict.json'

with open(original_name) as f:
    original_data = json.load(f) 

dict_data = dict()

rotation_matrix = np.array([[np.cos(rotation_z), -np.sin(rotation_z), 0],
                            [np.sin(rotation_z), np.cos(rotation_z), 0],
                            [0, 0, 1]])

scale_matrix = np.array([[1, 0, 0],
                        [0, scale_y, 0],
                        [0, 0, 1]])

roto_scaling = np.dot(rotation_matrix, scale_matrix)

for obj in original_data:
    # rotate, scale and then translate and convert to list
    dict_data[obj['label']] = {k: (np.dot(roto_scaling,np.array(v)*np.array([-1,-1,1]))-translation).tolist() for k, v in obj.items() if k!='label'} 

with open(dict_name, 'w') as f:
    json.dump(dict_data, f, indent=4)