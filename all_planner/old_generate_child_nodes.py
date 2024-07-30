def generate_child_nodes(self, curr):
        
        children = []
        ma_dirs = product(list(range(5)), repeat=len(self.agents)) # directions for move() for each agent: 0, 1, 2, 3, 4
        
        for dirs in ma_dirs: 
            # print(dirs)
            invalid_move = False
            child_loc = []
            # move each agent for new timestep & check for (internal) conflicts with each other
            for i, a in enumerate(self.agents):           
                    aloc = move(curr['loc'][i], dirs[i])
                    # vertex collision; check for duplicates in child_loc
                    if aloc in child_loc:
                        invalid_move = True
                        # print("internal conflict")
                        break
                    child_loc.append(move(curr['loc'][i], dirs[i]))   


            if invalid_move:
                continue


            for i, a in enumerate(self.agents):   
                # edge collision: check for matching locs in curr_loc and child_loc between two agents
                for j, a in enumerate(self.agents):   
                    if i != j:
                        # print(ai, aj)
                        if child_loc[i] == curr['loc'][j] and child_loc[j] == curr['loc'][i]:
                            invalid_move = True             
            
            if invalid_move:
                continue

            # check map constraints and external constraints
            for i, a in enumerate(self.agents):  
                next_loc= child_loc[i]
                # agent out of map bounds
                if next_loc[0]<0 or next_loc[0]>=len(self.my_map) or next_loc[1]<0 or next_loc[1]>=len(self.my_map[0]):
                    invalid_move = True
                # agechild_locnt collison with map obstacle
                elif self.my_map[next_loc[0]][next_loc[1]]:
                    invalid_move = True
                # agent is constrained by a negative external constraint
                elif self.constraint_violated(curr['loc'][i],next_loc,curr['timestep']+1,self.c_table[i], self.agents[i]):
                    invalid_move = True
                if invalid_move:
                    break

            if invalid_move:
                continue

            # find h_values for current moves
            h_value = 0
            for i in range(len(self.agents)):
                    h_value += self.heuristics[i][child_loc[i]]

            h_test = sum([self.heuristics[i][child_loc[i]] for i in range(len(self.agents))])

            assert h_value == h_test

            # g_value = curr['g_val']+ curr['reached_goal'].count(False)
            num_moves = curr['reached_goal'].count(False)
            # print("(edge) cost (curr -> child) in a* tree == ", num_moves)
            
            
            '''*************** new g cost cal ***************'''
            if curr['parent'] is not None:
                # print(f'curr_p: {curr["parent"]}')
                parent_locc = np.array(curr['parent']['loc'])            
               

            else:
                # parent_locc =  np.array([self.starts[0][0],self.starts[0][1]-3]),
                # parent_locc = np.zeros(2)
                parent_locc = np.array([self.starts[0][0],self.starts[0][1]-1])

                print(f'PASENT is NONE!!!')
                # parent_locc = np.array((curr['parent']['loc'][0],curr['parent']['loc'][1]-3))
                # parent_locc = None  # or some default value
            curr_locc = np.array(curr['loc'][0])
            child_locc = np.array(child_loc[0])
            print(f'family-> parent: {parent_locc}, curr: {curr_locc}, child: {child_locc}, cur_head: {curr["head_to"]}')
            
            direc = curr_locc - parent_locc
            new_direction_str = np.array(child_loc) - np.array(curr_locc)
            
            
            needs_rotation = not np.array_equal(direc, new_direction_str) and not np.array_equal(direc, np.zeros(2))
            cross_product = np.cross(direc, new_direction_str)
            
            
            # print(f'str_g: {g_cost["straight"]}')
            # print(f'le_g: {g_cost["rotate_left"]}')
            # print(f'ri_g: {g_cost["rotate_right"]}')
            
            backward_cost = self.g_cost["backward"]
            stop_cost = self.g_cost["stop"]
            straight_cost = self.g_cost["straight"]
            rotate_left_cost = self.g_cost["rotate_left"]
            rotate_right_cost = self.g_cost["rotate_right"]
            
            custom_g_cost = None
            
            HEAD_TO = child_loc[0]
            
            if np.array_equal(parent_locc, curr_locc) and np.array_equal(curr["head_to"],HEAD_TO) and dir[0]==4: 
                print('H: stop')
            elif np.array_equal(parent_locc, curr_locc) and np.array_equal(curr["head_to"],HEAD_TO) : 
                print('H: After Rotate Go STR')
            elif np.array_equal(parent_locc, curr_locc) and not np.array_equal(curr["head_to"],HEAD_TO) : 
                print('H: need to Rotate')
            elif np.array_equal(curr["head_to"],HEAD_TO):
                print('H: same as head_to GO STR')
            if dirs[0] == 4:
                print(f'same loc, no move, stop!!! use: {stop_cost}')
                custom_g_cost = stop_cost
            elif np.array_equal(parent_locc, HEAD_TO):
                print(f'H: going to Parent! BACKWARD!! use: {backward_cost}')
                custom_g_cost = backward_cost
            elif cross_product == 0:
                pass
                print(f'parent loc,bACkward??? , no need rot! use: {straight_cost}')
                custom_g_cost = straight_cost
                # continue
            elif needs_rotation:			
                # print(f'cross_product: {cross_product}')
                rotation_cost = rotate_left_cost if cross_product > 0 else rotate_right_cost
                custom_g_cost = rotation_cost
                print(f"need rotation! , NoMove Just rOtatet use: {rotation_cost}")
                child_loc[0] = (curr_locc[0],curr_locc[1])
            else : 
                pass
                print(f"STR, no need rotation! use: {straight_cost}")
                custom_g_cost = straight_cost
                # continue
            
            
            # g_value = curr['g_val'] + num_moves
            g_value = curr['g_val'] + custom_g_cost
            '''***************"***************"***************'''
            print(f'child_loc: {child_loc}')
            print(f'h: {h_value}, g: {g_value}, num_mov:{num_moves}, loc: {child_loc}, {convert_normal_to_pos_p(child_loc[0])}')

            reached_goal = [False for i in range(len(self.agents))]

            print(f'reach_go: {reached_goal[i]}, child_loc: {child_loc[i]}, goals[i]: {self.goals[i]}')
            for i, a in enumerate(self.agents):
                
                if not reached_goal[i] and child_loc[i] == self.goals[i]:

                    if curr['timestep']+1 <= self.max_constraints[i]:
                        if not self.future_constraint_violated(child_loc[i], curr['timestep']+1, self.max_constraints[i] ,self.c_table[i], self.agents[i]):
                    # print("agent ", a, 'has found solution at timestep ', curr['timestep'] + 1)
                    # print ('MAX CONSTRIANT:', self.max_constraints[i])
                            reached_goal[i] = True
                            # self.max_constraints[i] differs for each node
                    else:
                        reached_goal[i] = True


            child = {'loc': child_loc,
                    'g_val': g_value, # number of new locs (cost) added
                    'h_val': h_value,
                    'parent': curr,
                    'timestep': curr['timestep']+1,
                    'reached_goal': copy.deepcopy(reached_goal),
                    'head_to': HEAD_TO
                    } 

            children.append(child)

            # print(f'children: \n{children}')
        return children