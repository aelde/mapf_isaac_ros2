# -16.5 , -28.5 = 0 , 0
# -13.5 , -25.5 = 1 , 1
# 
def convert_normal_to_pos(pos):
    return (pos[0]*3 - 16.5, pos[1]*3 - 28.5)

def convert_pos_to_normal(normal):
    return ((normal[0] + 16.5) / 3, (normal[1] + 28.5) / 3)

# print(convert_normal_to_pos((10,2)))
# print(convert_pos_to_normal((1.5,-13.5)))

#manhattan_eulidean
tb_1 = (-1.5, -16.5)
tb_1_goal = (-13.5, -4.5)
tb_2 = (7.5, 19.5)
tb_2_goal = (7.5, 1.5)
tb_3 = (1.5, -13.5)
tb_3_goal = (7.5, -7.5)
# tb_1: (5.0, 4.0)
# tb_1_goal: (1.0, 8.0)
# tb_2: (8.0, 16.0)
# tb_2_goal: (8.0, 10.0)
# tb_3: (6.0, 5.0)
# tb_3_goal: (8.0, 7.0)

#cbs_1
tb_1 = (-1.5, -16.5)
tb_1_goal = (4.5, -10.5)
tb_2 = (7.5, 19.5)
tb_2_goal = (7.5, 1.5)
tb_3 = (4.5, -16.5)
tb_3_goal = (-1.5, -10.5)
# tb_1: (5.0, 4.0)
# tb_1_goal: (7.0, 6.0)
# tb_2: (8.0, 16.0)
# tb_2_goal: (8.0, 10.0)
# tb_3: (7.0, 4.0)
# tb_3_goal: (5.0, 6.0)

#cbs_advanced
tb_1 = (-1.5, -16.5)
tb_1_goal = (-1.5, 25.5)
tb_2 = (7.5, 19.5)
tb_2_goal = (7.5, 1.5)
tb_3 = (4.5, -16.5)
tb_3_goal = (1.5, 25.5)
# tb_1: (5.0, 4.0)
# tb_1_goal: (5.0, 18.0)
# tb_2: (8.0, 16.0)
# tb_2_goal: (8.0, 10.0)
# tb_3: (7.0, 4.0)
# tb_3_goal: (6.0, 18.0)

#cbs_advanced_2
tb_1 = (-7.5, 22.5)
tb_1_goal = (7.5, 13.5)
tb_2 = (7.5, 19.5)
tb_2_goal = (7.5, 1.5)
tb_3 = (4.5, -16.5)
tb_3_goal = (7.5, 16.5)
# tb_1: (3.0, 17.0)
# tb_1_goal: (8.0, 14.0)
# tb_2: (8.0, 16.0)
# tb_2_goal: (8.0, 10.0)
# tb_3: (7.0, 4.0)
# tb_3_goal: (8.0, 15.0)

print(f'tb_1: {convert_pos_to_normal(tb_1)}')
print(f'tb_1_goal: {convert_pos_to_normal(tb_1_goal)}')

print(f'tb_2: {convert_pos_to_normal(tb_2)}')
print(f'tb_2_goal: {convert_pos_to_normal(tb_2_goal)}')

print(f'tb_3: {convert_pos_to_normal(tb_3)}')
print(f'tb_3_goal: {convert_pos_to_normal(tb_3_goal)}')
