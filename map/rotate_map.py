def rotate_map_90_cc(input_file, output_file):
    # Read the input file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Parse the dimensions and map
    rows, cols = map(int, lines[0].split())
    map_data = [line.strip() for line in lines[1:]]
    
    # Rotate the map
    rotated_map = [''.join(map_data[i][j] for i in range(rows-1, -1, -1)) for j in range(cols)]
    
    # Write the rotated map to the output file
    with open(output_file, 'w') as f:
        f.write(f"{cols} {rows}\n")
        for row in rotated_map:
            f.write(row + '\n')

# Use the function
rotate_map_90_cc('map1_real.txt', 'map1_real.txt')

def flip_map_vertically(input_file, output_file):
    # Read the input file
    with open(input_file, 'r') as f:
        lines = f.readlines()
    
    # Parse the dimensions and map
    rows, cols = map(int, lines[0].split())
    map_data = [line.strip() for line in lines[1:]]
    
    # Flip the map vertically
    flipped_map = map_data[::-1]
    
    # Write the flipped map to the output file
    with open(output_file, 'w') as f:
        f.write(f"{rows} {cols}\n")
        for row in flipped_map:
            f.write(row + '\n')

# Use the function
# flip_map_vertically('map1_real.txt', 'map1_real.txt')