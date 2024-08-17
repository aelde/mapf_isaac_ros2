# #!/usr/bin/env python3
# from matplotlib.patches import Circle, Rectangle
# import matplotlib.pyplot as plt
# import numpy as np
# from matplotlib import animation
# import math

# Colors = ['g', 'r', 'b','c','y','m','w','k']

# class Animation:
#     def __init__(self, my_map, starts, goals, paths, head_to):
#         self.my_map = np.flip(np.transpose(my_map), 1)
#         self.starts = []
#         for start in starts:
#             self.starts.append((start[1], len(self.my_map[0]) - 1 - start[0]))
#         self.goals = []
#         for goal in goals:
#             self.goals.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
#         self.paths = []
#         if paths:
#             for path in paths:
#                 self.paths.append([])
#                 for loc in path:
#                     self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))
        
#         # Process head_to similar to paths
#         self.head_to_paths = []
#         if head_to:
#             for path in head_to:
#                 self.head_to_paths.append([])
#                 for loc in path:
#                     self.head_to_paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))

#         aspect = len(self.my_map) / len(self.my_map[0])

#         self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
#         self.ax = self.fig.add_subplot(111, aspect='equal')
#         self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

#         self.patches = []
#         self.artists = []
#         self.agents = dict()
#         self.agent_names = dict()
#         self.head_to = dict()
#         self.head_to_indicators = dict()  # New dictionary for head_to arrows
        
#         self.current_angles = dict()
#         self.target_angles = dict()
#         self.initial_positions = dict()
#         for i in range(len(self.paths)):
#             self.current_angles[i] = 0
#             self.target_angles[i] = 0
#             self.initial_positions[i] = (starts[i][1], len(self.my_map[0]) - 1 - starts[i][0])
        
#         # create boundary patch
#         x_min = -0.5
#         y_min = -0.5
#         x_max = len(self.my_map) - 0.5
#         y_max = len(self.my_map[0]) - 0.5
#         plt.xlim(x_min, x_max)
#         plt.ylim(y_min, y_max)

#         self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
#         for i in range(len(self.my_map)):
#             for j in range(len(self.my_map[0])):
#                 if self.my_map[i][j]:
#                     self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

#         # create agents:
#         self.T = 0
#         # draw goals first
#         for i, goal in enumerate(self.goals):
#             self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],
#                                           edgecolor='black', alpha=0.5))
#         for i in range(len(self.paths)):
#             name = str(i+1)
#             self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],
#                                     edgecolor='black')
#             self.agents[i].original_face_color = Colors[i % len(Colors)]
            
#             self.patches.append(self.agents[i])
            
#             self.T = max(self.T, len(paths[i]) - 1)
#             self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.25, name)
#             self.agent_names[i].set_horizontalalignment('center')
#             self.agent_names[i].set_verticalalignment('center')
#             self.artists.append(self.agent_names[i])

#             self.head_to[i] = self.ax.text(starts[i][0] + 0.25, starts[i][1], ">")
#             self.head_to[i].set_horizontalalignment('center')
#             self.head_to[i].set_verticalalignment('center')
#             self.artists.append(self.head_to[i])

#             # Add head_to arrow indicator
#             self.head_to_indicators[i] = self.ax.arrow(starts[i][0], starts[i][1], 0, 0, 
#                                                        head_width=0.2, head_length=0.2, 
#                                                        fc=Colors[i % len(Colors)], ec='black')
#             self.artists.append(self.head_to_indicators[i])

#         self.animation = animation.FuncAnimation(self.fig, self.animate_func,
#                                                  init_func=self.init_func,
#                                                  frames=int(self.T + 1) * 10,
#                                                  interval=100,
#                                                  blit=True)

#     def save(self, file_name, speed):
#         self.animation.save(
#             file_name,
#             fps=10 * speed,
#             dpi=200,
#             savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

#     @staticmethod
#     def show():
#         plt.show()

#     def init_func(self):
#         for p in self.patches:
#             self.ax.add_patch(p)
#         for a in self.artists:
#             self.ax.add_artist(a)
#         return self.patches + self.artists

#     def animate_func(self, t):
#         current_timestep = int(t / 10)
        
#         # Check if animation has restarted
#         if current_timestep == 0:
#             self.reset_indicators()
        
#         for k in range(len(self.paths)):
#             path = self.paths[k]
#             head_to_path = self.head_to_paths[k] if k < len(self.head_to_paths) else path
#             current_pos = np.array(self.get_state(t / 10, path))
#             head_to_pos = np.array(self.get_state(t / 10, head_to_path))
            
#             self.agents[k].center = (current_pos[0], current_pos[1])
#             self.agent_names[k].set_position((current_pos[0], current_pos[1] + 0.5))
            
#             # Calculate the target angle for rotation (original functionality)
#             dx = head_to_pos[0] - current_pos[0]
#             dy = head_to_pos[1] - current_pos[1]
            
#             if dx != 0 or dy != 0:
#                 self.target_angles[k] = math.degrees(math.atan2(dy, dx))
            
#             # Smoothly interpolate between current angle and target angle
#             interpolation_factor = 0.2
#             angle_diff = (self.target_angles[k] - self.current_angles[k] + 180) % 360 - 180
#             self.current_angles[k] += angle_diff * interpolation_factor
            
#             # Rotate and position the direction indicator (original functionality)
#             angle_rad = math.radians(self.current_angles[k])
#             self.head_to[k].set_position((current_pos[0] + 0.4 * math.cos(angle_rad), 
#                                           current_pos[1] + 0.4 * math.sin(angle_rad)))
#             self.head_to[k].set_rotation(self.current_angles[k])
            
#             # Update head_to arrow indicator
#             self.head_to_indicators[k].remove()
#             self.head_to_indicators[k] = self.ax.arrow(current_pos[0], current_pos[1], 
#                                                        dx * 0.4, dy * 0.4, 
#                                                        head_width=0.2, head_length=0.2, 
#                                                        fc=Colors[k % len(Colors)], ec='black')
#             self.artists[k*3+2] = self.head_to_indicators[k]
        
#         # reset all colors
#         for _, agent in self.agents.items():
#             agent.set_facecolor(agent.original_face_color)

#         # check drive-drive collisions
#         agents_array = [agent for _, agent in self.agents.items()]
#         for i in range(0, len(agents_array)):
#             for j in range(i + 1, len(agents_array)):
#                 d1 = agents_array[i]
#                 d2 = agents_array[j]
#                 pos1 = np.array(d1.center)
#                 pos2 = np.array(d2.center)
#                 if np.linalg.norm(pos1 - pos2) < 0.7:
#                     d1.set_facecolor('red')
#                     d2.set_facecolor('red')
#                     print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

#         return self.patches + self.artists

#     def reset_indicators(self):
#         for k in range(len(self.paths)):
#             self.current_angles[k] = 0
#             self.target_angles[k] = 0
#             initial_pos = self.initial_positions[k]
#             self.head_to[k].set_position((initial_pos[0] + 0.4, initial_pos[1]))
#             self.head_to[k].set_rotation(0)
            
#             # Reset head_to arrow indicator
#             self.head_to_indicators[k].remove()
#             self.head_to_indicators[k] = self.ax.arrow(initial_pos[0], initial_pos[1], 0.4, 0, 
#                                                        head_width=0.2, head_length=0.2, 
#                                                        fc=Colors[k % len(Colors)], ec='black')
#             self.artists[k*3+2] = self.head_to_indicators[k]
    
#     @staticmethod
#     def get_state(t, path):
#         if int(t) <= 0:
#             return np.array(path[0])
#         elif int(t) >= len(path):
#             return np.array(path[-1])
#         else:
#             pos_last = np.array(path[int(t) - 1])
#             pos_next = np.array(path[int(t)])
#             pos = (pos_next - pos_last) * (t - int(t)) + pos_last
#             return pos
#!/usr/bin/env python3
from matplotlib.patches import Circle, Rectangle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import math


# Colors = ['green', 'red', 'blue','cyan']
Colors = ['g', 'r', 'b','c','y','m','w','k']

class Animation:
    def __init__(self, my_map, starts, goals, paths,head_to):
        self.my_map = np.flip(np.transpose(my_map), 1)
        self.starts = []
        for start in starts:
            self.starts.append((start[1], len(self.my_map[0]) - 1 - start[0]))
        self.goals = []
        for goal in goals:
            self.goals.append((goal[1], len(self.my_map[0]) - 1 - goal[0]))
        self.paths = []
        if paths:
            for path in paths:
                self.paths.append([])
                for loc in path:
                    self.paths[-1].append((loc[1], len(self.my_map[0]) - 1 - loc[0]))

        aspect = len(self.my_map) / len(self.my_map[0])

        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.agents = dict()
        self.agent_names = dict()
        self.head_to = dict()
        
        self.current_angles = dict()
        self.target_angles = dict()
        self.initial_positions = dict()
        for i in range(len(self.paths)):
            self.current_angles[i] = 0
            self.target_angles[i] = 0
            self.initial_positions[i] = (starts[i][1], len(self.my_map[0]) - 1 - starts[i][0])
        
        # create boundary patch

        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        self.patches.append(Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor='none', edgecolor='gray'))
        for i in range(len(self.my_map)):
            for j in range(len(self.my_map[0])):
                if self.my_map[i][j]:
                    self.patches.append(Rectangle((i - 0.5, j - 0.5), 1, 1, facecolor='gray', edgecolor='gray'))

        # create agents:
        self.T = 0
        # draw goals first
        for i, goal in enumerate(self.goals):
            self.patches.append(Rectangle((goal[0] - 0.25, goal[1] - 0.25), 0.5, 0.5, facecolor=Colors[i % len(Colors)],
                                          edgecolor='black', alpha=0.5))
        for i in range(len(self.paths)):
            name = str(i+1)
            self.agents[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],
                                    edgecolor='black')
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            
            # self.head_to[i] = Circle((starts[i][0], starts[i][1]), 0.3, facecolor=Colors[i % len(Colors)],edgecolor='black')
            # self.head_to[i].original_face_color = Colors[i % len(Colors)]

            
            self.patches.append(self.agents[i])
            # self.patches.append(self.head_to[i])
            
            # self.T = max(self.T+1, len(paths[i]))
            self.T = max(self.T, len(paths[i]) - 1)
            self.agent_names[i] = self.ax.text(starts[i][0], starts[i][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment('center')
            self.agent_names[i].set_verticalalignment('center')
            self.artists.append(self.agent_names[i])

            self.head_to[i] = self.ax.text(starts[i][0] + 0.25, starts[i][1], ">")
            self.head_to[i].set_horizontalalignment('center')
            self.head_to[i].set_verticalalignment('center')
            self.artists.append(self.head_to[i])

            # Create an arrow with zero length initially
            # self.head_to[i] = self.ax.arrow(starts[i][0], starts[i][1], 0, 0, 
            #                                 head_width=0.2, head_length=0.2, fc=Colors[i % len(Colors)], ec='black')
            # self.artists.append(self.head_to[i])

        self.animation = animation.FuncAnimation(self.fig, self.animate_func,
                                                 init_func=self.init_func,
                                                 frames=int(self.T + 1) * 10,
                                                 interval=100,
                                                 blit=True)

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

    @staticmethod
    def show():
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, t):
        current_timestep = int(t / 10)
        
        # Check if animation has restarted
        if current_timestep == 0:
            self.reset_indicators()
        
        # Check if animation has restarted
        if current_timestep == 0:
            self.reset_indicators()
        
        for k in range(len(self.paths)):
            path = self.paths[k]
            current_pos = np.array(self.get_state(t / 10, path))
            
            self.agents[k].center = (current_pos[0], current_pos[1])
            self.agent_names[k].set_position((current_pos[0], current_pos[1] + 0.5))
            
            # Look ahead to find the next different position
            next_pos = current_pos
            next_timestep = current_timestep
            while next_timestep + 1 < len(path):
                next_timestep += 1
                if not np.array_equal(path[next_timestep], current_pos):
                    next_pos = np.array(path[next_timestep])
                    break
            
            # Calculate the target angle for rotation
            dx = next_pos[0] - current_pos[0]
            dy = next_pos[1] - current_pos[1]
            
            if dx != 0 or dy != 0:  # Update target angle if there's a future movement
                self.target_angles[k] = math.degrees(math.atan2(dy, dx))
            
            # Smoothly interpolate between current angle and target angle
            interpolation_factor = 0.2  # Adjust this value to control rotation speed
            angle_diff = (self.target_angles[k] - self.current_angles[k] + 180) % 360 - 180
            self.current_angles[k] += angle_diff * interpolation_factor
            
            # Rotate and position the direction indicator
            angle_rad = math.radians(self.current_angles[k])
            self.head_to[k].set_position((current_pos[0] + 0.4 * math.cos(angle_rad), 
                                          current_pos[1] + 0.4 * math.sin(angle_rad)))
            self.head_to[k].set_rotation(self.current_angles[k])
            
        # reset all colors
        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        # check drive-drive collisions
        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(0, len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)
                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor('red')
                    d2.set_facecolor('red')
                    print("COLLISION! (agent-agent) ({}, {}) at time {}".format(i, j, t/10))

        return self.patches + self.artists

    def reset_indicators(self):
        for k in range(len(self.paths)):
            self.current_angles[k] = 0
            self.target_angles[k] = 0
            initial_pos = self.initial_positions[k]
            self.head_to[k].set_position((initial_pos[0] + 0.4, initial_pos[1]))
            self.head_to[k].set_rotation(0)
    
    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        elif int(t) >= len(path):
            return np.array(path[-1])
        else:
            pos_last = np.array(path[int(t) - 1])
            pos_next = np.array(path[int(t)])
            pos = (pos_next - pos_last) * (t - int(t)) + pos_last
            return pos