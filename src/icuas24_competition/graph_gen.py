#!/usr/bin/env python

"""
graph_gen.py: Generates a graph of the environment given a set of parameters.
"""

from __future__ import annotations
from dataclasses import dataclass, field
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import yaml


@dataclass
class PlantBed:
    """PlantBed: A class that represents a plant bed.

    Attributes:
        id: The unique identifier of the plant bed.
        yaw: The yaw of the plant bed.
    """
    id: int
    yaw: float


@dataclass
class Node:
    """Node: A class that represents a node in a graph.

    Attributes:
        uuid: The unique identifier of the node.
        color: The color of the node.
        position: The position of the node.
        neightbours: The list of neighbours of the node.
    """
    uuid: int
    color: str
    position: tuple[float, float, float]
    neighbors: list[Node] = field(default_factory=list)
    plant_ids: list[PlantBed] = field(default_factory=list)


@dataclass
class IndoorFarm:
    """IndoorFarm: A class that represents an indoor farm.

    Attributes:
        row_count: The number of rows of plant beds.
        col_count: The number of columns of plant beds.
        height_count: The number of levels of plant beds.
        plant_bed_width: The width of each plant bed. Width is the row direction.
        plant_bed_length: The length of each plant bed. Length is the column direction.
        plant_bed_height: The height of each plant bed. Height is the vertical direction.
        row_spacing: The spacing between each row of plant beds.
        col_spacing: The spacing between each column of plant beds.
        height_spacing: The spacing between each level of plant beds.
        x_offset: The x offset of the indoor farm from the origin (0, 0, 0).
        y_offset: The y offset of the indoor farm from the origin (0, 0, 0).
        z_offset: The z offset of the indoor farm from the origin (0, 0, 0).
        safety_distance: The safety distance between the drone and the plant bed.
    """
    row_count: int
    col_count: int
    height_count: int
    plant_bed_width: float
    plant_bed_length: float
    plant_bed_height: float
    row_spacing: float
    col_spacing: float
    height_spacing: float
    x_offset: float
    y_offset: float
    z_offset: float
    safety_distance: float = 0.5
    enable_diagonals: bool = False

    @classmethod
    def from_yaml(cls, filename: str):
        """from_yaml: Creates an IndoorFarm object from a YAML file."""
        with open(filename, 'r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
            return cls(
                row_count=config['row_count'],
                col_count=config['col_count'],
                height_count=config['height_count'],
                plant_bed_width=config['plant_bed_width'],
                plant_bed_length=config['plant_bed_length'],
                plant_bed_height=config['plant_bed_height'],
                row_spacing=config['row_spacing'],
                col_spacing=config['col_spacing'],
                height_spacing=config['height_spacing'],
                x_offset=config['x_offset'],
                y_offset=config['y_offset'],
                z_offset=config['z_offset'],
                safety_distance=config['safety_distance'],
                enable_diagonals=config['enable_diagonals']
            )

    def neighbors(self, uuid: int, col: int, row: int, z: int) -> list[int]:
        """neighbors: Returns the neighbors of a node."""
        layer_node_size = (self.row_count + 2) * (self.col_count + 1)
        ns = []
        if col == -1:
            ns.append(uuid+1)
            if self.enable_diagonals:
                ns.append(uuid+layer_node_size+1)
                ns.append(uuid-layer_node_size+1)
        if -1 < col < self.col_count:
            ns.append(uuid-1)
            ns.append(uuid+1)
            if self.enable_diagonals:
                ns.append(uuid+layer_node_size+1)
                ns.append(uuid-layer_node_size+1)
                ns.append(uuid+layer_node_size-1)
                ns.append(uuid-layer_node_size-1)
        if col == self.col_count:
            ns.append(uuid-1)
            if self.enable_diagonals:
                ns.append(uuid+layer_node_size-1)
                ns.append(uuid-layer_node_size-1)

        # Row connected in first and last column only
        if col in (-1, self.col_count):
            if row == 0:
                ns.append(uuid+self.row_count+2)
            if 0 < row < self.row_count:
                ns.append(uuid-self.row_count-2)
                ns.append(uuid+self.row_count+2)
            if row == self.row_count:
                ns.append(uuid-self.row_count-2)

        if z == 0:
            ns.append(uuid+layer_node_size)
        if 0 < z < self.height_count:
            ns.append(uuid-layer_node_size)
            ns.append(uuid+layer_node_size)
        if z == self.height_count:
            ns.append(uuid-layer_node_size)
        return ns

    def plant_ids(self, col: int, row: int, z: int) -> list[PlantBed]:
        """plant_ids: Returns the plant ids of a node."""
        ids = []
        if row < self.row_count:
            pid = 1 + z + col * self.height_count + \
                row * self.row_count * self.height_count
            ids.append(PlantBed(pid, 0.0))
        if row > 0:
            pid = 1 + z + col * self.height_count + \
                (row-1) * self.row_count * self.height_count
            ids.append(PlantBed(pid, 180.0))
        return ids

    @property
    def graph(self) -> list[Node]:
        """graph: Returns a graph of the indoor farm."""
        nodes: list[Node] = []
        uuid = 0
        for z in range(self.height_count):
            height = self.z_offset + (z+1) * self.plant_bed_height + \
                (z+1) * self.height_spacing/2 + z * self.height_spacing/2
            for row in range(self.row_count + 1):
                for col in range(self.col_count + 1):
                    x = self.y_offset - self.row_spacing/2 + row * \
                        (self.plant_bed_length + self.row_spacing)
                    y = self.x_offset + col * self.plant_bed_width + \
                        (col-1/2) * self.col_spacing
                    y2 = self.x_offset + self.plant_bed_width/2 + col * \
                        (self.plant_bed_width + self.col_spacing)

                    # Red dots are interconnection points
                    # nodes.append(Node('r', (x, y, height)))
                    if col == 0:
                        y -= self.safety_distance
                        # Yellow left safety points
                        nodes.append(
                            Node(uuid, 'y', (x, y, height), self.neighbors(uuid, col-1, row, z)))
                        uuid += 1
                    if col < self.col_count:
                        # Green points are the center of the plant beds
                        nodes.append(
                            Node(uuid, 'g', (x, y2, height), self.neighbors(uuid, col, row, z),
                                 self.plant_ids(col, row, z)))
                        uuid += 1
                    if col == self.col_count:
                        y += self.safety_distance
                        # Yellow right safety points
                        nodes.append(
                            Node(uuid, 'y', (x, y, height), self.neighbors(uuid, col, row, z)))
                        uuid += 1

        return nodes


def distance(node1: Node, node2: Node) -> float:
    """distance: Returns the distance between two nodes."""
    return ((node1.position[0] - node2.position[0])**2
            + (node1.position[1] - node2.position[1])**2 +
            (node1.position[2] - node2.position[2])**2)**0.5


def draw_2d_indoor_farm(indoor_farm: IndoorFarm):
    """draw_indoor_farm: Draws a plot of the indoor farm."""
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for row in range(indoor_farm.row_count):
        for col in range(indoor_farm.col_count):
            x = indoor_farm.x_offset + col * \
                (indoor_farm.plant_bed_width + indoor_farm.col_spacing)
            y = indoor_farm.y_offset + row * \
                (indoor_farm.plant_bed_length + indoor_farm.row_spacing)
            rectangle = Rectangle(
                (x, y), indoor_farm.plant_bed_width, indoor_farm.plant_bed_length)
            ax.add_patch(rectangle)

    for node in indoor_farm.graph:
        ax.scatter(node.position[0], node.position[1], color=node.color)
    ax.set_xlim([0, 30])
    ax.set_ylim([0, 30])
    plt.show()


def draw_3d_indoor_farm(indoor_farm: IndoorFarm):
    """draw_indoor_farm: Draws a plot of the indoor farm."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for row in range(indoor_farm.row_count):
        for col in range(indoor_farm.col_count):
            x = indoor_farm.x_offset + col * \
                (indoor_farm.plant_bed_width + indoor_farm.col_spacing)
            y = indoor_farm.y_offset + row * \
                (indoor_farm.plant_bed_length + indoor_farm.row_spacing)
            z = 0
            # ax.bar3d(x, y, z, indoor_farm.plant_bed_width,
            #          indoor_farm.plant_bed_length, indoor_farm.plant_bed_height,
            #          color='b', alpha=0.8)

    for node in indoor_farm.graph:
        ax.scatter(node.position[0], node.position[1],
                   node.position[2], color=node.color)
        if node.plant_ids:
            ax.text(node.position[0], node.position[1],
                    node.position[2], f'{node.plant_ids}')

        for node_id in node.neighbors:
            if node_id < 0 or node_id >= len(indoor_farm.graph):
                continue
            node_n = indoor_farm.graph[node_id]
            ax.plot([node.position[0], node_n.position[0]],
                    [node.position[1], node_n.position[1]],
                    [node.position[2], node_n.position[2]], color='b')

    # Set labels and title
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_title('Indoor Farm')
    ax.set_xlim([0, 30])
    ax.set_ylim([0, 30])
    plt.show()


if __name__ == "__main__":
    an_indoor_farm = IndoorFarm(
        row_count=3,
        col_count=3,
        height_count=3,
        plant_bed_width=6.0,
        plant_bed_length=2.0,
        plant_bed_height=0.8,
        row_spacing=4.0,
        col_spacing=1.5,
        height_spacing=2.0,
        x_offset=3.0,
        y_offset=3.0,
        z_offset=0,
        safety_distance=1.5,
        enable_diagonals=True
    )
    # draw_2d_indoor_farm(an_indoor_farm)

    draw_3d_indoor_farm(an_indoor_farm)
