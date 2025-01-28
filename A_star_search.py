import copy

with open('astar_in.txt') as file:
    lines = [line.rstrip('\n') for line in file]

initial_state = []
goal_state = []

for state_label in lines[::4]:
    if state_label == 'start':
        for line in lines[1:4]:
            tiles = line.split()
            tiles = [int(tile) if tile != '*' else 0 for tile in tiles]
            initial_state.append(tiles)

    else:
        for line in lines[5:8]:
            tiles = line.split()
            tiles = [int(tile) if tile != '*' else 0 for tile in tiles]
            goal_state.append(tiles)


class Node:
    total_nodes = 0

    def __init__(self, state, g_n, h_n, from_node):
        self.state = state
        self.g_n = g_n
        self.h_n = h_n
        self.f_n = self.g_n + self.h_n
        self.from_node = from_node
        Node.total_nodes += 1

        # path max equation
        if self.g_n != 0:
            self.f_n = max(self.from_node.f_n, self.g_n + self.h_n)


    def __eq__(self, other):
        return self.state == other.state
    
    def __lt__(self, other):
        if self.f_n == other.f_n:
            return self.h_n < other.h_n
        return self.f_n < other.f_n


def misplaced_tiles(state):
    total_tiles = 0

    for row in range(3):
        for col in range(3):
            tile = state[row][col]

            if tile != 0 and tile != goal_state[row][col]:
                total_tiles += 1
    
    return total_tiles

def manhattan_d(state):

    # A dictionary for checking the positions of tiles in goal state
    goal_dict = {}
    for row in range(3):
        for col in range(3):
            if goal_state[row][col] != 0:
                goal_dict[goal_state[row][col]] = (row, col)
    
    # Calculating the manhattan distance (blank tiles are not included)
    total_d = 0
    for state_row in range(3):
        for state_col in range(3):
            tile = state[state_row][state_col]
            
            if tile != 0:
                goal_row = goal_dict[tile][0]
                goal_col = goal_dict[tile][1]
                total_d += abs(goal_row - state_row) + abs(goal_col - state_col)
    
    return total_d

def nilsson_score(state):
    """Checks the Nilsson sequence score of a given state.
    This may not be an admissible heuristic."""
    
    # Flatten the state list to check for nilsson score (it's hard to check with 2D list)
    state_list = []
    goal_state_list = []
    for row in range(3):
        for col in range(3):
            state_list.append(state[row][col])
            goal_state_list.append(goal_state[row][col])

    # This is for mapping of list index to nilsson sequence numbering
    i_nilssonpos = {}
    for i in range(len(goal_state_list)):
        if goal_state_list[i] == 0: # for blank in center
            i_nilssonpos[4] = 8
        else:
            i_nilssonpos[i] = goal_state_list[i] - 1

    # Calculate Nilsson score for each tile
    total_score = 0
    for tile in state_list:
        if tile == 0:
            pass
        elif state_list.index(tile) == 4: # if center tile
            total_score += 1

        # Below is formula obtained from the given link for nilsson sequence score
        elif i_nilssonpos[state_list.index((tile % 8) + 1)] != (i_nilssonpos[state_list.index(tile)] + 1) % 8:
            total_score += 2
            
    return 3 * total_score + manhattan_d(state)

def heuristic(state, h_option):
    """Returns the value of preferred h(n) for a state.
    1 - misplaced tiles, 2 - Manhattan distance, 3 - Nilsson score"""
    if h_option == 1:
        return misplaced_tiles(state)
    elif h_option == 2:
        return manhattan_d(state)
    elif h_option == 3:
        return nilsson_score(state)

def expand_node(state):
    child_states = []

    # List of all movements of blank tile (up, down, left, right)
    possible_moves = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # Get the index of blank tile
    for row in range(3):
        for col in range(3):
            if state[row][col] == 0:
                blank_r, blank_c = row, col
    
    for move in possible_moves:
        adj_r, adj_c = blank_r + move[0], blank_c + move[1] # Get position of adjacent tile
        if 0 <= adj_r <= 2 and 0 <= adj_c <= 2:

            # I had to make a deepcopy of state because this function keeps modifying the original parameter
            new_state = copy.deepcopy(state)

            # Switching blank tile with the adjacent tile in the new state
            new_state[blank_r][blank_c], new_state[adj_r][adj_c] = new_state[adj_r][adj_c], new_state[blank_r][blank_c]

            child_states.append(new_state)

    return child_states

def a_star(initial_state, goal_state, h_option):
    global open_queue, closed_nodes, solution_path, nodes_explored
    open_queue = []
    closed_nodes = []
    solution_path = []
    nodes_explored = 0
    Node.total_nodes = 0

    starting_node = Node(initial_state, 0, heuristic(initial_state, h_option), None)
    put_openqueue(starting_node)

    while open_queue:
        node_n = pop_openqueue()
        closed_nodes.append(node_n)

        if node_n.state == goal_state:
            print("Solution found: \n")
            
            while node_n:
                solution_path.append(node_n)
                node_n = node_n.from_node

            solution_path.reverse()
            for solution_node in solution_path:
                for row in solution_node.state:
                    print(f'{row}')
                    
                print(f'F(n) = {solution_node.f_n}')
                print(f'G(n) = {solution_node.g_n}')
                print(f'H(n) = {solution_node.h_n}')

                if h_option == 3:
                    p_n = manhattan_d(solution_node.state)
                    s_n = int((nilsson_score(solution_node.state) - p_n) / 3)
                    print(f'P(n) = {p_n}')
                    print(f'S(n) = {s_n}')
                print()
            
            print(f'Depth of solution: {len(solution_path) - 1}')
            print(f'Number of nodes expanded: {Node.total_nodes}')
            print(f'Number of nodes explored: {nodes_explored}')

            return
        
        new_states = expand_node(node_n.state)
        if new_states:
            for child_state in new_states:
                g_n = node_n.g_n + 1
                new_n = Node(child_state, g_n, heuristic(child_state, h_option), node_n)

                if new_n not in closed_nodes:
                    put_openqueue(new_n)
                else:
                    for node_c in closed_nodes:
                        if node_c == new_n and new_n.f_n < node_c.f_n:
                            node_c = new_n
                            put_openqueue(node_c)
                            closed_nodes.remove(node_c)

    print('No solution was found.')
    return

                         
def put_openqueue(node):
    for i in range(len(open_queue)):
        if open_queue[i][1] == node and node.f_n < open_queue[i][1].f_n: # if node in open, and has less f_n 
            open_queue[i][1] = node # update the node in open
            open_queue[i][0] = node.f_n # update the f_n in open
            open_queue.sort(reverse=True)
            return
        elif open_queue[i][1] == node and node.f_n >= open_queue[i][1].f_n:
            return
        
    open_queue.append([node.f_n, node])
    open_queue.sort(reverse=True)


def pop_openqueue():
    if not open_queue:
        return None
    
    global nodes_explored
    nodes_explored += 1
    return open_queue.pop()[1]


a_star(initial_state, goal_state, 1)
