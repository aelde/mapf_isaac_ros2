import os
import yaml
from ex_convert import convert_normal_to_pos_w,convert_normal_to_pos_p,convert_pos_to_normal_w,convert_pos_to_normal_p
from map._map import obstacles_w,obstacles_p

WS_DIR = f'/home/eggs/humble_mapf/src/mapf_isaac'
M_PEN =  'isaac_pendora.py'
M_WAREH = 'isaac_warehouse.py'
RESULT_DIR = '/home/eggs/humble_mapf/src/mapf_isaac/result/path'

def load_uneven_astar_config():
    config_dir = 'config/uneven_astar.yaml'
    config_d = os.path.join(f'{WS_DIR}', config_dir)
    print(f'astar config dir: {config_d}')
    with open(config_d, 'r') as file:
        config = yaml.safe_load(file)
        global MAP_NAME,TOTAL_ROBOTS,ROBOT_START,CONV_NORMAL_TO_POS,CONV_POS_TO_NORMAL,MAP_USE,OBSTACLES_MAP,DIR_COST
        MAP_NAME = config['map']['map_name']
        if MAP_NAME == 'map_wareh':
            MAP_USE = f'{WS_DIR}/isaac/{M_WAREH}'
            CONV_NORMAL_TO_POS = convert_normal_to_pos_w
            CONV_POS_TO_NORMAL = convert_pos_to_normal_w
            ROBOT_START = config['robot']['robot_start_w']
            OBSTACLES_MAP = obstacles_w
        elif MAP_NAME == 'map_pen':
            MAP_USE = f'{WS_DIR}/isaac/{M_PEN}'
            CONV_NORMAL_TO_POS = convert_normal_to_pos_p
            CONV_POS_TO_NORMAL = convert_pos_to_normal_p
            ROBOT_START = config['robot']['robot_start_p']
            OBSTACLES_MAP = obstacles_p
        print(f'MAP: {MAP_NAME}')
        TOTAL_ROBOTS = config['robot']['total_robots']
        DIR_COST = config['costs']
load_uneven_astar_config()
