"""
Plot tools 2D
@author: huiming zhou
"""

import os
import sys
import matplotlib.pyplot as plt

# 将项目根目录添加到系统路径，方便模块导入
sys.path.append(
    os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/"
)

from Search_2D import env


class Plotting:
    """
    A class for plotting and visualizing 2D pathfinding animations and grids.

    Attributes:
        xI (tuple[int, int]): Starting point coordinates.
        xG (tuple[int, int]): Goal point coordinates.
        env (env.Env): Environment instance containing obstacle information.
        obs (list[tuple[int, int]]): List of obstacle coordinates.
    """

    def __init__(self, xI: tuple[int, int], xG: tuple[int, int]):
        """
        Initialize the Plotting class with start and goal points.

        Args:
            xI (tuple[int, int]): Starting point (x, y) coordinates.
            xG (tuple[int, int]): Goal point (x, y) coordinates.
        """
        self.xI, self.xG = xI, xG
        self.env = env.Env()
        self.obs = self.env.obs_map()

    def update_obs(self, obs: list[tuple[int, int]]) -> None:
        """
        Update the list of obstacles.

        Args:
            obs (list[tuple[int, int]]): New list of obstacle coordinates.
        """
        self.obs = obs

    def animation(
        self, path: list[tuple[int, int]], visited: list[tuple[int, int]], name: str
    ) -> None:
        """
        Display animation of the pathfinding process.

        Args:
            path (list[tuple[int, int]]): List of coordinates in the found path.
            visited (list[tuple[int, int]]): List of visited nodes during search.
            name (str): Title of the plot.
        """
        self.plot_grid(name)
        self.plot_visited(visited)
        self.plot_path(path)
        plt.show()

    def animation_lrta(
        self,
        path: list[list[tuple[int, int]]],
        visited: list[list[tuple[int, int]]],
        name: str,
    ) -> None:
        """
        Display animation specifically for LRTA* (Learning Real-Time A*) algorithm.

        Args:
            path (list[list[tuple[int, int]]]): A list of lists of paths for each LRTA* iteration.
            visited (list[list[tuple[int, int]]]): A list of lists of visited nodes for each iteration.
            name (str): Title of the plot.
        """
        self.plot_grid(name)
        cl = self.color_list_2()
        path_combine = []

        for k in range(len(path)):
            self.plot_visited(visited[k], cl[k])
            plt.pause(0.2)
            self.plot_path(path[k])
            path_combine += path[k]
            plt.pause(0.2)
        if self.xI in path_combine:
            path_combine.remove(self.xI)
        self.plot_path(path_combine)
        plt.show()

    def animation_ara_star(
        self,
        path: list[list[tuple[int, int]]],
        visited: list[list[tuple[int, int]]],
        name: str,
    ) -> None:
        """
        Display animation specifically for ARA* (Anytime Repairing A*) algorithm.

        Args:
            path (list[list[tuple[int, int]]]): A list of lists of paths for each ARA* iteration.
            visited (list[list[tuple[int, int]]]): A list of lists of visited nodes for each iteration.
            name (str): Title of the plot.
        """
        self.plot_grid(name)
        cl_v, cl_p = self.color_list()

        for k in range(len(path)):
            self.plot_visited(visited[k], cl_v[k])
            self.plot_path(path[k], cl_p[k], True)
            plt.pause(0.5)

        plt.show()

    def animation_bi_astar(
        self,
        path: list[tuple[int, int]],
        v_fore: list[tuple[int, int]],
        v_back: list[tuple[int, int]],
        name: str,
    ) -> None:
        """
        Display animation specifically for Bidirectional A* algorithm.

        Args:
            path (list[tuple[int, int]]): Final path found between start and goal.
            v_fore (list[tuple[int, int]]): Nodes visited in the forward search.
            v_back (list[tuple[int, int]]): Nodes visited in the backward search.
            name (str): Title of the plot.
        """
        self.plot_grid(name)
        self.plot_visited_bi(v_fore, v_back)
        self.plot_path(path)
        plt.show()

    def plot_grid(self, name: str) -> None:
        """
        Plot the grid with start, goal, and obstacle locations.

        Args:
            name (str): Title of the plot.
        """
        obs_x = [x[0] for x in self.obs]
        obs_y = [x[1] for x in self.obs]

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")
        plt.plot(obs_x, obs_y, "sk")
        plt.title(name)
        plt.axis("equal")

    def plot_visited(self, visited: list[tuple[int, int]], cl: str = "gray") -> None:
        """
        Plot the visited nodes during search.

        Args:
            visited (list[tuple[int, int]]): List of visited nodes.
            cl (str): Color of the visited nodes.
        """
        if self.xI in visited:
            visited.remove(self.xI)

        if self.xG in visited:
            visited.remove(self.xG)

        count = 0

        for x in visited:
            count += 1
            plt.plot(x[0], x[1], color=cl, marker="o")
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )

            # Adjust the speed of the animation based on count
            if count < len(visited) / 3:
                length = 20
            elif count < len(visited) * 2 / 3:
                length = 30
            else:
                length = 40

            if count % length == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    def plot_path(
        self, path: list[tuple[int, int]], cl: str = "r", flag: bool = False
    ) -> None:
        """
        Plot the path from start to goal.

        Args:
            path (list[tuple[int, int]]): List of coordinates in the path.
            cl (str): Color of the path line.
            flag (bool): Whether to apply the specified color `cl` to the path.
        """
        path_x = [path[i][0] for i in range(len(path))]
        path_y = [path[i][1] for i in range(len(path))]

        if not flag:
            plt.plot(path_x, path_y, linewidth="3", color="r")
        else:
            plt.plot(path_x, path_y, linewidth="3", color=cl)

        plt.plot(self.xI[0], self.xI[1], "bs")
        plt.plot(self.xG[0], self.xG[1], "gs")

        plt.pause(0.01)

    def plot_visited_bi(
        self, v_fore: list[tuple[int, int]], v_back: list[tuple[int, int]]
    ) -> None:
        """
        Plot the visited nodes in the bidirectional search.

        Args:
            v_fore (list[tuple[int, int]]): Nodes visited in the forward search.
            v_back (list[tuple[int, int]]): Nodes visited in the backward search.
        """
        if self.xI in v_fore:
            v_fore.remove(self.xI)

        if self.xG in v_back:
            v_back.remove(self.xG)

        len_fore, len_back = len(v_fore), len(v_back)

        for k in range(max(len_fore, len_back)):
            if k < len_fore:
                plt.plot(
                    v_fore[k][0], v_fore[k][1], linewidth="3", color="gray", marker="o"
                )
            if k < len_back:
                plt.plot(
                    v_back[k][0],
                    v_back[k][1],
                    linewidth="3",
                    color="cornflowerblue",
                    marker="o",
                )

            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )

            if k % 10 == 0:
                plt.pause(0.001)
        plt.pause(0.01)

    @staticmethod
    def color_list() -> tuple[list[str], list[str]]:
        """
        Provides color lists for visualizing different search paths and visited nodes.

        Returns:
            tuple[list[str], list[str]]: Two lists of colors, one for visited nodes and one for paths.
        """
        cl_v = ["silver", "wheat", "lightskyblue", "royalblue", "slategray"]
        cl_p = ["gray", "orange", "deepskyblue", "red", "m"]
        return cl_v, cl_p

    @staticmethod
    def color_list_2() -> list[str]:
        """
        Provides an alternative color list for LRTA* visualization.

        Returns:
            list[str]: List of colors.
        """
        cl = [
            "silver",
            "steelblue",
            "dimgray",
            "cornflowerblue",
            "dodgerblue",
            "royalblue",
            "plum",
            "mediumslateblue",
            "mediumpurple",
            "blueviolet",
        ]
        return cl
