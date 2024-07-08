import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

class Visualizer:
    def __init__(self):
        self.obstacles = np.array([
                    [-10.5, 4.5, 0], [-10.5, 7.5, 0], [-10.5, 10.5, 0], [-10.5, 13.5, 0],
                    [-10.5, 16.5, 0], [-10.5, 19.5, 0], [-10.5, 22.5, 0],
                    [-4.5, 4.5, 0], [-4.5, 7.5, 0], [-4.5, 10.5, 0], [-4.5, 13.5, 0],
                    [-4.5, 16.5, 0], [-4.5, 19.5, 0], [-4.5, 22.5, 0],
                    [-1.5, 4.5, 0], [-1.5, 7.5, 0], [-1.5, 10.5, 0], [-1.5, 13.5, 0],
                    [-1.5, 16.5, 0], [-1.5, 19.5, 0], [-1.5, 22.5, 0],
                    [4.5, 4.5, 0], [4.5, 7.5, 0], [4.5, 10.5, 0], [4.5, 13.5, 0],
                    [4.5, 16.5, 0], [4.5, 19.5, 0], [4.5, 22.5, 0],
                    [7.5, 4.5, 0], [7.5, 7.5, 0], [7.5, 10.5, 0],
                    [10.5, 4.5, 0], [10.5, 7.5, 0], [10.5, 10.5, 0], [10.5, 13.5, 0],
                    [10.5, 16.5, 0], [10.5, 19.5, 0], [10.5, 22.5, 0],
                    [13.5, 4.5, 0], [13.5, 7.5, 0], [13.5, 10.5, 0], [13.5, 13.5, 0],
                    [13.5, 16.5, 0], [13.5, 19.5, 0], [13.5, 22.5, 0],
                    # frame bottom
                    [16.5, -25.5, 0], [16.5, -22.5, 0], [16.5, -19.5, 0], [16.5, -16.5, 0],
                    [16.5, -13.5, 0], [16.5, -10.5, 0], [16.5, -7.5, 0], [16.5, -4.5, 0],
                    [16.5, -1.5, 0], [16.5, 1.5, 0], [16.5, 4.5, 0], [16.5, 7.5, 0],
                    [16.5, 10.5, 0], [16.5, 13.5, 0], [16.5, 16.5, 0], [16.5, 19.5, 0],
                    [16.5, 22.5, 0], [16.5, 25.5, 0], [16.5, 28.5, 0],
                    # frame right
                    [16.5, 28.5, 0], [13.5, 28.5, 0], [10.5, 28.5, 0], [7.5, 28.5, 0],
                    [4.5, 28.5, 0], [1.5, 28.5, 0], [-1.5, 28.5, 0], [-4.5, 28.5, 0],
                    [-7.5, 28.5, 0], [-10.5, 28.5, 0], [-13.5, 28.5, 0], [-16.5, 28.5, 0],
                    [-19.5, 28.5, 0], [-22.5, 28.5, 0], [-25.5, 28.5, 0],
                    # frame top
                    [-16.5, 25.5, 0],  [-16.5, 22.5, 0],  [-16.5, 19.5, 0],  [-16.5, 16.5, 0],
                    [-16.5, 13.5, 0],  [-16.5, 10.5, 0],  [-16.5, 7.5, 0],   [-16.5, 4.5, 0],
                    [-16.5, 1.5, 0],   [-16.5, -1.5, 0],  [-16.5, -4.5, 0],  [-16.5, -7.5, 0],
                    [-16.5, -10.5, 0], [-16.5, -13.5, 0], [-16.5, -16.5, 0], [-16.5, -19.5, 0],
                    [-16.5, -22.5, 0], [-16.5, -25.5, 0], [-16.5, -28.5, 0],
                    # frame left
                    [-25.5, -28.5, 0], [-22.5, -28.5, 0], [-19.5, -28.5, 0], [-16.5, -28.5, 0],
                    [-13.5, -28.5, 0], [-10.5, -28.5, 0], [-7.5, -28.5, 0], [-4.5, -28.5, 0],
                    [-1.5, -28.5, 0], [1.5, -28.5, 0], [4.5, -28.5, 0], [7.5, -28.5, 0],
                    [10.5, -28.5, 0], [13.5, -28.5, 0], [16.5, -28.5, 0], [19.5, -28.5, 0],
                    [22.5, -28.5, 0], [25.5, -28.5, 0], [28.5, -28.5, 0],
                ])

    def visualize_astar_path(self, tb_id, start_pos, goal_pos, current, open_set, closed_set, g_score, f_score, came_from, step):
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_title(f'A* Path Planning for TB_{tb_id} - Step {step}')
        
        ax.set_xlim(-28.5, 28.5)
        ax.set_ylim(-16.5, 16.5)
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        obstacles = self.obstacles[:, :2]
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        ax.scatter(start_pos[1], start_pos[0], c='c', marker='o', s=200, label='Start')
        ax.scatter(goal_pos[1], goal_pos[0], c='m', marker='*', s=200, label='Goal')
        
        open_set_coords = np.array([pos for _, _, pos in open_set])
        closed_set_coords = np.array(list(closed_set))
        if open_set_coords.size > 0:
            ax.scatter(open_set_coords[:, 1], open_set_coords[:, 0], c='g', marker='s', s=100, alpha=0.3, label='Open Set')
        if closed_set_coords.size > 0:
            ax.scatter(closed_set_coords[:, 1], closed_set_coords[:, 0], c='r', marker='s', s=100, alpha=0.3, label='Closed Set')
        
        ax.scatter(current[1], current[0], c='y', marker='o', s=150, label='Current')
        
        for pos in set(list(g_score.keys()) + list(f_score.keys())):
            x, y = pos[1], pos[0]
            g = g_score.get(pos, float('inf'))
            f = f_score.get(pos, float('inf'))
            h = f - g
            ax.add_patch(plt.Rectangle((x-1.5, y-1.5), 3, 3, fill=False, edgecolor='gray'))
            ax.text(x, y+1.3, f'f={f:.1f}', ha='center', va='bottom', fontsize=8)
            ax.text(x, y+0.2, f'g={g:.1f}', ha='center', va='top', fontsize=8)
            ax.text(x, y-0.9, f'h={h:.1f}', ha='center', va='center', fontsize=8)
        
        path = []
        current_pos = current
        while current_pos in came_from:
            path.append(current_pos)
            current_pos = came_from[current_pos]
        path.append(start_pos)
        path = path[::-1]
        path_array = np.array(path)
        ax.plot(path_array[:, 1], path_array[:, 0], 'b-', linewidth=2, label='Path')
        
        ax.set_aspect('equal', 'box')
        ax.set_ylabel('X')
        ax.set_xlabel('Y')
        ax.legend(loc='lower right')
        
        ax.text(-27.5, 15.5, f'Step: {step}', fontsize=12, fontweight='bold', 
                bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))
        
        plt.gca().invert_yaxis()
        
        result_dir = 'result/astar_steps'
        if not os.path.exists(result_dir):
            os.makedirs(result_dir)
        
        filename = os.path.join(result_dir, f'TB_{tb_id}_step_{step:03d}.png')
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f'A* step visualization saved as {filename}')

    def visualize_path(self, tb_id, start_pos, goal_pos, path):
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_title(f'Path Planning for TB_{tb_id}')
        
        start_pos = np.array(start_pos)
        goal_pos = np.array(goal_pos)
        path = np.array(path)
        
        ax.set_xlim(-28.5, 28.5)
        ax.set_ylim(-16.5, 16.5)
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        obstacles = self.obstacles[:, :2]
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        ax.scatter(start_pos[1], start_pos[0], c='c', marker='o', s=200, label='Start')
        ax.scatter(goal_pos[1], goal_pos[0], c='m', marker='*', s=200, label='Goal')
        
        ax.plot(path[:, 1], path[:, 0], 'y-', linewidth=2, label='Path')
        
        ax.set_aspect('equal', 'box')
        ax.set_ylabel('X')
        ax.set_xlabel('Y')
        ax.legend()
        
        plt.gca().invert_yaxis()
        
        result_dir = 'result/path'
        if not os.path.exists(result_dir):
            os.makedirs(result_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(result_dir, f'TB_{tb_id}_path_{timestamp}.png')
        
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f'Path visualization saved as {filename}')

    def visualize_all_paths(self, all_paths, all_starts, all_goals):
        fig, ax = plt.subplots(figsize=(16, 12))
        ax.set_title('Path Planning for All TBs')
        
        ax.set_xlim(-28.5, 28.5)
        ax.set_ylim(-16.5, 16.5)
        ax.set_xticks(np.arange(-25.5, 26.5, 3))
        ax.set_yticks(np.arange(-13.5, 14.5, 3))
        ax.grid(True)

        obstacles = self.obstacles[:, :2]
        ax.scatter(obstacles[:, 1], obstacles[:, 0], c='k', marker='s', s=100, label='Obstacles')
        
        colors = ['g', 'r', 'b']
        
        for i, (path, start, goal) in enumerate(zip(all_paths, all_starts, all_goals)):
            print(f'tb number: {i+1} :: color: {colors[i]} :: start: {start}')
            path = np.array(path)
            if path.size > 0:
                ax.plot(path[:, 1], path[:, 0], color=colors[i], linestyle='-', linewidth=2, label=f'Path TB_{i+1}')
            ax.scatter(start[1], start[0], c='c', marker='o', s=200)
            ax.scatter(goal[1], goal[0], c='m', marker='*', s=200)
        
        ax.set_aspect('equal', 'box')
        ax.set_xlabel('Y')
        ax.set_ylabel('X')
        ax.legend()
        
        plt.gca().invert_yaxis()
        
        result_dir = 'result/path'
        if not os.path.exists(result_dir):
            os.makedirs(result_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(result_dir, f'All_TBs_paths_{timestamp}.png')
        
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        
        print(f'All paths visualization saved as {filename}')