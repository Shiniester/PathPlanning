"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
from typing import Tuple, List, Dict

# Add the path for external modules if needed
sys.path.append(
    os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/"
)

from Search_2D import plotting, env


class AStar:
    """
    AStar class implements the A* pathfinding algorithm.
    """

    def __init__(
        self, s_start: Tuple[int, int], s_goal: Tuple[int, int], heuristic_type: str
    ):
        """
        Initialize A* algorithm with start and goal nodes and heuristic type.

        :param s_start: Starting point as a tuple (x, y)
        :param s_goal: Goal point as a tuple (x, y)
        :param heuristic_type: Type of heuristic to use ('manhattan' or 'euclidean')
        """
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env.Env()  # Environment instance

        self.u_set = self.Env.motions  # Feasible input motions
        self.obs = self.Env.obs  # Obstacle positions

        self.OPEN = []  # Priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / visited nodes
        # self.PARENT = dict()  # recorded parent
        # self.g = dict()  # cost to come
        self.PARENT: Dict[
            Tuple[int, int], Tuple[int, int]
        ] = {}  # Parent dictionary for path reconstruction
        self.g: Dict[Tuple[int, int], float] = {}  # Cost-to-come for each node

    def searching(self) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
        """
        Perform A* searching.

        :return: Tuple containing the path and visited nodes in the order they were processed.
        """
        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN, (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # Stop if goal is reached
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # Condition for updating cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def get_neighbor(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Find neighbors of state s that are not obstacles.

        :param s: Current state as a tuple (x, y)
        :return: List of neighboring states.
        """
        return [
            (s[0] + u[0], s[1] + u[1])
            for u in self.u_set
            if (s[0] + u[0], s[1] + u[1]) not in self.obs
        ]

    def cost(self, s_start: Tuple[int, int], s_goal: Tuple[int, int]) -> float:
        """
        Calculate the movement cost between two nodes.

        :param s_start: Starting node
        :param s_goal: Goal node
        :return: Cost of moving from s_start to s_goal. If in collision, return infinity.
        """
        if self.is_collision(s_start, s_goal):
            return math.inf
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start: Tuple[int, int], s_end: Tuple[int, int]) -> bool:
        """
        Check if the line segment (s_start, s_end) is in collision with obstacles.

        :param s_start: Starting node
        :param s_end: End node
        :return: True if there is a collision, otherwise False.
        """
        if s_start in self.obs or s_end in self.obs:
            return True
        # More detailed checks can be added here if necessary
        return False

    def f_value(self, s: Tuple[int, int]) -> float:
        """
        Calculate f = g + h, where g is the cost-to-come and h is the heuristic.

        :param s: Current state
        :return: Calculated f-value.
        """
        return self.g[s] + self.heuristic(s)

    def extract_path(
        self, PARENT: Dict[Tuple[int, int], Tuple[int, int]]
    ) -> List[Tuple[int, int]]:
        """
        Extract the path from start to goal based on the PARENT dictionary.

        :param PARENT: Dictionary containing parent pointers for each node
        :return: List of nodes in the path from start to goal.
        """
        path = [self.s_goal]
        s = self.s_goal

        while s != self.s_start:
            s = PARENT[s]
            path.append(s)

        path.reverse()
        return path

    def heuristic(self, s: Tuple[int, int]) -> float:
        """
        Calculate the heuristic value based on the chosen heuristic type.

        :param s: Current state
        :return: Heuristic cost to reach the goal.
        """
        goal = self.s_goal

        if self.heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        return math.hypot(goal[0] - s[0], goal[1] - s[1])


def main():
    s_start = (5, 5)
    s_goal = (45, 25)

    astar = AStar(s_start, s_goal, "euclidean")
    plot = plotting.Plotting(s_start, s_goal)

    path, visited = astar.searching()
    plot.animation(path, visited, "A*")  # Animation


if __name__ == "__main__":
    main()
