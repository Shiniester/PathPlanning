"""
Environment configuration for A* pathfinding
"""

from typing import List, Tuple, Set


class Env:
    """
    Env class defines the environment in which the A* algorithm operates,
    including obstacles and grid boundaries.
    """

    def __init__(self):
        """
        Initialize the environment with grid size, motions, and obstacle positions.
        """
        self.x_range = 51  # X-axis size of the grid
        self.y_range = 31  # Y-axis size of the grid
        # Possible moves (8 directions)
        self.motions: List[Tuple[int, int]] = [
            (-1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
            (1, 0),
            (1, -1),
            (0, -1),
            (-1, -1),
        ]
        self.obs: Set[Tuple[int, int]] = self.obs_map()  # Obstacle positions

    def update_obs(self, obs: Set[Tuple[int, int]]) -> None:
        """
        Update the obstacle positions in the environment.

        :param obs: Set of obstacle coordinates
        """
        self.obs = obs

    def obs_map(self) -> Set[Tuple[int, int]]:
        """
        Generate the obstacle map with predefined obstacles.

        :return: Set of coordinates representing obstacle positions.
        """
        x, y = self.x_range, self.y_range
        obs = set()

        # Boundary obstacles
        for i in range(x):
            obs.add((i, 0))  # Bottom boundary
            obs.add((i, y - 1))  # Top boundary

        for i in range(y):
            obs.add((0, i))  # Left boundary
            obs.add((x - 1, i))  # Right boundary

        # Internal obstacles
        for i in range(10, 21):
            obs.add((i, 15))  # Horizontal line of obstacles
        for i in range(15):
            obs.add((20, i))  # Vertical line of obstacles

        for i in range(15, 30):
            obs.add((30, i))  # Another vertical line of obstacles
        for i in range(16):
            obs.add((40, i))  # Another vertical line of obstacles

        return obs
