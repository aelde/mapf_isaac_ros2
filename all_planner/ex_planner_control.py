import random
import cbs_basic, a_star_class ,single_agent_planner, visualize
import numpy as np

D_HISOVLER = 'CBS'
D_LOWSOLVER = 'Astar'
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
        paths, nodes_gen, nodes_exp = solution[:3]
        # PlannerControl.print_solution(solution)
        # PlannerControl.show_animation(map, start_pos, goal_pos, paths)
        return solution
    
    @staticmethod
    def print_solution(solution):
        paths, nodes_gen, nodes_exp = solution[:3]
        print(f'found solution!!!!!*/*/!')
        print()
        
    @staticmethod   
    def show_animation(map, start_pos, goal_pos, paths):
        animation = visualize.Animation(map, start_pos, goal_pos, paths)
        animation.show()
    
    def print_config(self):
        print(f'HI: {self.HISOVLER} LOW: {self.LOWSOLVER}')