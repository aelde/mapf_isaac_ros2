import random
import cbs_basic, a_star_class ,single_agent_planner, visualize
import numpy as np
from DICISION import ANI_VISUAL
D_HISOVLER = 'CBS'
D_LOWSOLVER = 'Astar'
import cbs_basic_old
import cbs_basic_with_rot_state
class PlannerControl:
    def __init__(self,HISOVLER=D_HISOVLER,LOWSOLVER=D_LOWSOLVER):
        self.HISOVLER = HISOVLER
        self.LOWSOLVER = LOWSOLVER
        self.print_config()
        # self.astar_planner = AstarPlanner()
        # self.cbs_planner = CBSPlanner()

    @staticmethod
    def plan_paths(map,start_pos,goal_pos):
        cbs = cbs_basic.CBSSolver(map,start_pos,goal_pos)
        solution = cbs.find_solution(False)
        paths, nodes_gen, nodes_exp, head_to = solution[:4]
        # PlannerControl.print_solution(solution)
        # PlannerControl.show_animation(map, start_pos, goal_pos, paths)
        return solution
    
    @staticmethod
    def plan_paths_old(map,start_pos,goal_pos):
        cbs = cbs_basic_old.CBSSolver(map,start_pos,goal_pos)
        solution = cbs.find_solution(False)
        paths, nodes_gen, nodes_exp = solution[:3]
        # PlannerControl.print_solution(solution)
        # PlannerControl.show_animation(map, start_pos, goal_pos, paths)
        return solution
    
    @staticmethod
    def plan_paths_with_rot_state(map,start_pos,goal_pos,initial_angles):
        cbs = cbs_basic_with_rot_state.CBSSolver(map,start_pos,goal_pos,initial_angles)
        solution = cbs.find_solution(False)
        paths, nodes_gen, nodes_exp, head_to = solution[:4]
        # PlannerControl.print_solution(solution)
        # PlannerControl.show_animation(map, start_pos, goal_pos, paths)
        return solution
    
    @staticmethod
    def print_solution(solution):
        paths, nodes_gen, nodes_exp = solution[:3]
        print(f'found solution!!!!!*/*/!')
        print()
        
    @staticmethod   
    def show_animation(map, start_pos, goal_pos, paths,head_to):
        # animation = visualize.Animation(map, start_pos, goal_pos, paths)
        animation = ANI_VISUAL.Animation(map, start_pos, goal_pos, paths,head_to)
        animation.show()
    
    def print_config(self):
        print(f'HI: {self.HISOVLER} LOW: {self.LOWSOLVER}')