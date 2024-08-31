import numpy as np

class Robot_State_Check:

    # this fn get only position state
    def get_position_state(self, input_list):
        return [tuple(item[:2]) for item in input_list]

    # this fn get only angle state
    def get_angle_state(self, input_list):
        return [item[-1] for item in input_list]

    # this fn combine robot rot and robot pos to robot state list
    def combine_robot_state_list(self, values, positions):
        assert len(values) == len(positions), "len angle state != len positions state"
        
        combined = [(pos[0], pos[1], val) for pos, val in zip(positions, values)]
        print(f'len before: {len(combined)}')
        return combined
    
    def make_angle_correct(self, input_list):
        result = []
        for i in range(len(input_list)):
            current = input_list[i]
            
            if i < len(input_list) - 1:
                next_state = input_list[i+1]
                
                # Check if both current and next_state have at least 3 elements
                if len(current) >= 3 and len(next_state) >= 3:
                    # Check if position changed
                    if current[:2] != next_state[:2]:
                        # If position changed, make current angle match the next one
                        corrected = (current[0], current[1], next_state[2])
                    else:
                        corrected = current
                else:
                    # If either doesn't have 3 elements, keep current as is
                    corrected = current
            else:
                # For the last element, keep it as is
                corrected = current
            
            result.append(corrected)
        
        return result
        
    # this fn for detect non repeat rot state
    def detect_add_state(self, input_list):
        result = []
        for i in range(len(input_list)):
            result.append(tuple(input_list[i]))  
            
            if i < len(input_list) - 1:  # handle out of index err
                current_state = input_list[i][2]
                next_state = input_list[i+1][2]
                
                if current_state != next_state:
                    check_rotate = abs(next_state-current_state)
                    print(f'{abs(next_state-current_state)} : {[input_list[i][0], input_list[i][1]]}')
                    if check_rotate != 180.0:  # rot 1 time
                        new_entry = [(input_list[i][0], input_list[i][1], next_state)]
                    else:  # rot 2 times
                        first_rotate = -(current_state)+90
                        second_rotate = next_state
                        print(f'first: {first_rotate}, second: {second_rotate}')
                        new_entry = [
                            (input_list[i][0], input_list[i][1], first_rotate),
                            (input_list[i][0], input_list[i][1], second_rotate)
                        ]
                    result.extend(new_entry)
        
        print(f'len after: {len(result)}')        
        return result  # return the list of tuples

if __name__ == "__main__":
    rotation_checker = Robot_State_Check()  # for test
    values = [90.0, 180.0, 180.0, 90.0, 90.0, 90.0, 90.0, 90.0, 180.0]
    pos = [(4, 2), (3, 2), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6), (2, 7), (1, 7)]
    result = rotation_checker.combine_robot_state_list(values, pos)
    # print(f'before detect \n{result}')
    input_array = result
    result2 = rotation_checker.detect_add_state(input_array)
    # print(f'after detect \n{result2}')
    pos_state_af = rotation_checker.get_position_state(result2)
    angle_af = rotation_checker.get_angle_state(result2)
    pos_state_bf = rotation_checker.get_position_state(result)
    angle_bf = rotation_checker.get_angle_state(result)

    aaa = [(4, 3, 90.0), (4, 3, 90.0), (4, 3, 180.0), (3, 3, 180.0), (2, 3, 180.0), (2, 3, -90.0), (2, 4, 90.0), (2, 5, 90.0), (2, 6, 90.0), (2, 7, 90.0), (2, 8, 90.0), (2, 8, 0.0), (1, 8, 180.0)]    
    r = rotation_checker.make_angle_correct(aaa)
    print(f'r : \n{r}')
    # print(f'get only pos state: \nAFTER{pos_state_af}')
    # print(f'BEFORE{pos_state_bf}')
    # print(f'AFTER{angle_af}')
    # print(f'BEFORE{angle_bf}')
