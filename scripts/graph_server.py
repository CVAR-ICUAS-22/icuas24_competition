import rospy
from geometry_msgs.msg import Point
from icuas_msgs.msg import Graph, Node
from icuas_msgs.srv import GetGraph, GetGraphResponse
# from indoor_farm import IndoorFarm


class GraphServer:
    """Graph Server"""

    def __init__(self):
        rospy.init_node('graph_server', anonymous=True)

        # self.environment_model = IndoorFarm.from_yaml(
        #     'scripts/indoor_farm.yaml')

        self.graph_serv = rospy.Service(
            'get_graph', GetGraph, self.get_graph_callback)

        rospy.spin()

    def get_graph_callback(self, req):
        """get_graph_callback: Returns the graph."""
        _ = req
        dummy_graph = Graph()
        node0 = Node()
        node0.uuid = 0
        node0.position = Point(0, 0, 0)
        node1 = Node()
        node1.uuid = 1
        node1.position = Point(1, 0, 0)
        dummy_graph.nodes.append(node0)
        dummy_graph.nodes.append(node1)
        dummy_graph.distances.append(0)
        dummy_graph.distances.append(0.5)
        dummy_graph.distances.append(0.5)
        dummy_graph.distances.append(0)
        return GetGraphResponse(True, dummy_graph)
        # return GetGraphResponse(self.environment_model.graph)


if __name__ == '__main__':
    GraphServer()
