#!/usr/bin/env python3
"""
graph_server.py: Graph Server Node for the ICUAS 24 Competition.
"""
from itertools import chain
import rospy
from geometry_msgs.msg import Point
from icuas_msgs.msg import Graph, Node, PlantBed
from icuas_msgs.srv import GetGraph, GetGraphResponse
import numpy as np
from icuas24_competition.graph_gen import IndoorFarm, distance


# TODO: add plant_ids to each node
class GraphServer:
    """Graph Server"""

    def __init__(self):
        rospy.init_node('graph_server', anonymous=True)

        indoor_farm_file = rospy.get_param('~indoor_farm_file')
        self.environment_model = IndoorFarm.from_yaml(indoor_farm_file)

        self.graph_serv = rospy.Service(
            'get_graph', GetGraph, self.get_graph_callback)

        rospy.loginfo('Graph Server Started')
        rospy.loginfo(self.environment_model.graph)
        rospy.spin()

    def get_graph_callback(self, req):
        """get_graph_callback: Returns the graph."""
        _ = req
        dummy_graph = Graph()
        distances = np.identity(len(self.environment_model.graph))
        distances[distances == 0] = np.inf
        distances[distances == 1] = 0
        for node in self.environment_model.graph:
            a_node = Node()
            a_node.uuid = node.uuid
            a_node.position = Point(
                node.position[0], node.position[1], node.position[2])
            for plant in node.plant_ids:
                a_node.plant_ids.append(PlantBed(id=plant.id, yaw=plant.yaw))
            dummy_graph.nodes.append(a_node)
            for neighbor in node.neighbors:
                if neighbor in range(len(self.environment_model.graph)):
                    distances[node.uuid][neighbor] = distance(
                        node, self.environment_model.graph[neighbor])
                    distances[neighbor][node.uuid] = distance(
                        node, self.environment_model.graph[neighbor])
        dummy_graph.distances = list(chain.from_iterable(distances.tolist()))
        return GetGraphResponse(True, dummy_graph)


if __name__ == '__main__':
    GraphServer()
