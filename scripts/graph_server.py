#!/usr/bin/env python3
"""
graph_server.py: Graph Server Node for the ICUAS 24 Competition.
"""
from itertools import chain
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from icuas_msgs.msg import Graph, Node, PlantBed
from icuas_msgs.srv import GetGraph, GetGraphResponse
import numpy as np
from icuas24_competition.graph_gen import IndoorFarm, distance


class GraphServer:
    """Graph Server"""

    def __init__(self):
        rospy.init_node('graph_server', anonymous=True)

        indoor_farm_file = rospy.get_param('~indoor_farm_file')
        self.environment_model = IndoorFarm.from_yaml(indoor_farm_file)

        self.graph_serv = rospy.Service(
            'get_graph', GetGraph, self.get_graph_callback)

        self.viz_topic = rospy.Publisher(
            'debug/graph', Marker, queue_size=10)
        self.viz_timer = rospy.Timer(rospy.Duration(1), self.viz_callback)

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
                if neighbor == node.uuid:
                    continue
                if neighbor in range(len(self.environment_model.graph)):
                    distances[node.uuid][neighbor] = distance(
                        node, self.environment_model.graph[neighbor])
                    distances[neighbor][node.uuid] = distance(
                        node, self.environment_model.graph[neighbor])
        dummy_graph.distances = list(chain.from_iterable(distances.tolist()))
        return GetGraphResponse(True, dummy_graph)

    def viz_callback(self, event=None):
        """viz_callback: Publishes the graph for visualization."""
        # Plotting nodes
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "graph"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        for node in self.environment_model.graph:
            marker.points.append(Point(
                node.position[0], node.position[1], node.position[2]))
        self.viz_topic.publish(marker)

        # Plotting path between nodes
        path = Marker()
        path.header.frame_id = "world"
        path.header.stamp = rospy.Time.now()
        path.ns = "graph"
        path.id = 1
        path.type = Marker.LINE_LIST
        path.action = Marker.ADD
        path.pose.orientation.w = 1.0
        path.scale.x = 0.05
        path.color.a = 1.0
        path.color.r = 0.0
        path.color.g = 0.0
        path.color.b = 1.0
        path.points = []
        for node in self.environment_model.graph:
            for neighbor in node.neighbors:
                try:
                    neighbor_pt = self.environment_model.graph[neighbor]
                    path.points.append(Point(*node.position))
                    path.points.append(Point(*neighbor_pt.position))
                except IndexError:
                    pass
        self.viz_topic.publish(path)

        # Plotting indoor farm
        farm = Marker()
        farm.header.frame_id = "world"
        farm.header.stamp = rospy.Time.now()
        farm.ns = "graph"
        farm.id = 2
        farm.type = Marker.MESH_RESOURCE
        farm.action = Marker.ADD
        farm.mesh_resource = "file:///root/sim_ws/src/icuas24_competition/models/icuas24/icuas24_wrld_notxt.dae"
        farm.pose.orientation.w = 1.0
        farm.scale.x = 1.0
        farm.scale.y = 1.0
        farm.scale.z = 1.0
        farm.color.a = 1.0
        farm.color.r = 1.0
        farm.color.g = 0.7
        self.viz_topic.publish(farm)


if __name__ == '__main__':
    GraphServer()
