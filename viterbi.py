import numpy as np
import sys

def parse_input(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    r, c = map(int, lines[0].strip().split(" "))

    map_data = [line.strip().split() for line in lines[1:r+1]]
    num_obs = int(lines[r+1].strip())
    obs = [lines[r+2+i].strip() for i in range(num_obs)]
    error_rate = float(lines[r+2+num_obs].strip())
    return r, c, map_data, num_obs, obs, error_rate

class Viterbi:
    def __init__(self,r,c,map_data,num_obs,obs,error_rate):
        self.row_num=r
        self.col_num=c

        self.map=map_data
        self.error_rate = error_rate
        self.num_obs=num_obs
        self.Y=obs

        self.state_space=[]
        self.probabilities=[]
        self.tranversable_pos=0
        self.Transition_m = []
        self.emis_m= []

        self.set_state_space()
        self.tranversable_pos = len(self.state_space)
        self.change_probability()
        self.set_transition_m()
        self.set_emis_m()

    def set_state_space(self):
        for i in range(self.row_num):
            for j in range(self.col_num):
                if self.map[i][j] == '0':
                    self.state_space.append((i, j))
    
    def change_probability(self):
        initial_prob = 1 / self.tranversable_pos
        for i in range(self.row_num):
            for j in range(self.col_num):
                if self.map[i][j] == "0":
                    self.probabilities.append(initial_prob)
                else:
                    self.probabilities.append(0)


    def get_neighbors(self, state):
        y, x = state
        neighbors = []
        # directions
        if y > 0 and self.map[y-1][x] == "0": 
            neighbors.append((y-1, x))
        if y < self.row_num-1 and self.map[y+1][x] == "0": 
            neighbors.append((y+1, x))
        if x > 0 and self.map[y][x-1] == "0":  
            neighbors.append((y, x-1))
        if x < self.col_num-1 and self.map[y][x+1] == "0":  
            neighbors.append((y, x+1))
        return neighbors
    
    def get_surroundings(self, state):
        y, x = state
        N = '1' if y == 0 or self.map[y-1][x] == 'X' else '0'
        S = '1' if y == self.row_num-1 or self.map[y+1][x] == 'X' else '0'
        W = '1' if x == 0 or self.map[y][x-1] == 'X' else '0'
        E = '1' if x == self.col_num-1 or self.map[y][x+1] == 'X' else '0'
        return N, S, W, E
    
    def set_transition_m(self):
        self.Transition_m = np.zeros((self.tranversable_pos, self.tranversable_pos))
        for k, state in enumerate(self.state_space):
            y, x = state
            neighbors = self.get_neighbors(state)
            valid_neighbors = len(neighbors)
            for neighbor in neighbors:
                neighbor_index = self.state_space.index(neighbor)
                self.Transition_m[k, neighbor_index] = 1 / valid_neighbors

    def set_emis_m(self):
        self.emis_m = np.zeros((self.tranversable_pos, self.num_obs))
        for i, state in enumerate(self.state_space):
            surroundings = self.get_surroundings(state)
            for j in range(self.num_obs):
                error_count = sum(1 for k in range(4) if surroundings[k] != self.Y[j][k])
                emission_prob = ((1 - self.error_rate) ** (4 - error_count)) * (self.error_rate ** error_count)
                self.emis_m[i, j] = emission_prob


    def Viterbi_algorithm(self):
        trellis = np.zeros((self.num_obs, self.row_num, self.col_num))
        K=0
        for i in range(self.row_num):
            for j in range(self.col_num):
                if self.probabilities[j + self.col_num * i] != 0:
                    trellis[0,i,j] = self.probabilities[j+self.col_num * i] * self.emis_m[K,0]
                    K+=1
                else:
                    trellis[0,i,j]=0

        for t in range(1, self.num_obs):
            for i, state in enumerate(self.state_space):
                max_prob = 0
                y, x = state
                for k in range(self.tranversable_pos):
                    y1, x1 = self.state_space[k]
                    prob = trellis[t-1, y1, x1] * self.Transition_m[k, i] * self.emis_m[i, t]
                    if prob > max_prob:
                        max_prob = prob
                trellis[t, y, x] = max_prob

        return trellis

if __name__ == "__main__":
    input_file = sys.argv[1] #get input

    r, c, map_data, num_obs, obs, error_rate = parse_input(input_file)
    viterbi_solver = Viterbi(r, c, map_data, num_obs, obs, error_rate)
    result = viterbi_solver.Viterbi_algorithm()
   
    print(len(result))
    np.savez('output.npz', *result) #save in output file
