import json
import os
import numpy as np

translation = np.array([6.8210, 8.7180, 0])
rotation_z = 135 * np.pi / 180
scale_y = 1.0925

current_folder = os.path.dirname(os.path.abspath(__file__))
original_name = current_folder + "/waypoints_original.json"
dict_name = current_folder + "/waypoints.json"

with open(original_name) as f:
    original_data = json.load(f)

rot_matrix = np.array(
    [
        [np.cos(rotation_z), -np.sin(rotation_z), 0],
        [np.sin(rotation_z), np.cos(rotation_z), 0],
        [0, 0, 1],
    ]
)

scale_matrix = np.array([[1, 0, 0], [0, scale_y, 0], [0, 0, 1]])

roto_scaling = np.dot(rot_matrix, scale_matrix)

dict_data = dict()

for obj in original_data:
    dict_data[obj["label"]] = {
        k: (-translation - np.dot(-roto_scaling, np.array(v)))[0:2].tolist()
        for k, v in obj.items()
        if k != "label"
    }

with open(dict_name, "w") as f:
    json.dump(dict_data, f, indent=4)
