import numpy as np


class Node:
    def __init__(self, state, prev=None, g=0, h=0):
        self.state = state
        self.prev = prev  # parent
        self.g = g  # moves from start
        self.h = h  # heuristic
        self.f = 0


class Puzzle:
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal

    def possible_moves(self, state):
        empty_position = np.where(state == 0)
        empty_pos_row, = empty_position[0]
        empty_pos_col = empty_position[1]

        moves = []

        if empty_pos_row > 0:
            moves.append((-1, 0))  # UP
        if empty_pos_row < 2:
            moves.append((1, 0))  # DOWN

        if empty_pos_col > 0:
            moves.append((0, -1))  # LEFT
        if empty_pos_col < 2:
            moves.append((0, 1))  # RIGHT

        return moves

    def move(self, state, move):
        empty_position = np.where(state == 0)

        new_state = state.copy()
        # move empty pos
        new_row = empty_position[0] + move[0]
        new_col = empty_position[1] + move[1]
        new_empty_position = (new_row, new_col)
        # swap positions
        new_state[empty_position], new_state[new_empty_position] = \
            new_state[new_empty_position], new_state[empty_position]

        return new_state

    # Manhattan distance to correct positions
    def distance(self, state):
        distance = 0
        # 3x3 puzzle
        for i in range(3):
            for j in range(3):
                value = state[i][j]
                # 0 is empty position
                if value != 0:
                    goal_position = np.where(self.goal == value)  # get correct position
                    distance += abs(goal_position[0] - i) + abs(goal_position[1] - j)

        return distance

    def a_star(self):
        open = []
        closed = []
        # found = False

        first_node = Node(self.start, h = self.distance(self.start))
        first_node.f = first_node.g + first_node.h

        open.append(first_node)

        while len(open):
            open.sort(key = lambda node: node.f)
            current = open.pop(0)

            # Found solution
            if np.array_equal(current.state, self.goal):
                return current

            closed.append(current)
            # print(f'close:', len(closed))

            # Expand
            moves = self.possible_moves(current.state)
            for move in moves:
                new = self.move(current.state, move)

                # add new nodes to open
                state_in_closed = any(np.array_equal(new, node.state) for node in closed)
                if not state_in_closed:
                    next_node = Node(new, current, g = current.g + 1, h = self.distance(new))
                    next_node.f = next_node.g + next_node.h
                    open.append(next_node)
                    # print(f'open', len(open))

        return None

    def print(self, goal_node):
        path = []

        while goal_node.prev:
            path.append(goal_node.state)
            goal_node = goal_node.prev

        path.reverse()

        for state in path:
            print(state)
            print('========')

        print(f"Moves:", len(path))


if __name__ == "__main__":
    initial_state = np.array([[2, 3, 0], [1, 5, 6], [4, 7, 8]])  # 6 moves
    # initial_state = np.array([[8, 6, 7], [2, 5, 4], [3, 0, 1]])  # Have solution but not worth to wait with A*
    # initial_state = np.array([[1, 3, 6], [5, 8, 2], [0, 7, 4]])  # 18 moves
    gl_state = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 0]])
    solver = Puzzle(initial_state, gl_state)
    solution = solver.a_star()

    if solution is not None:
        print(initial_state)
        print('========')
        solver.print(solution)
    else:
        print("Can't find soulution")

