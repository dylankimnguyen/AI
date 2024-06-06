import numpy as np
import sys

def parse_input(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    rows, cols = map(int, lines[0].strip().split(" "))
    map_data = [line.strip().split() for line in lines[1:rows+1]]
    num_observations = int(lines[rows+1].strip())
    observations = [lines[rows+2+i].strip() for i in range(num_observations)]
    error_rate = float(lines[rows+2+num_observations].strip())
    return rows, cols, map_data, num_observations, observations, error_rate