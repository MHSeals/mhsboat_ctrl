#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from sensor_msgs.msg import PointCloud2, PointField, NavSatFix, Imu
from geometry_msgs.msg import Quaternion
import math
import numpy as np
from boat_interfaces.msg import AiOutput
from boat_interfaces.msg import BuoyCoordinates
import matplotlib.pyplot as plt
import networkx as nx
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

class locate_buoys(Node):
    def __init__(self):
        super().__init__("TaskTwo")
        self.get_logger().info("Task Two: its working")
        self.latitudes = []
        self.longitudes = []
        self.types = []
        self.boat_latitude = 0
        self.boat_longitude = 0
        #subscribes to AI Output
        self.AiOutput_subscriber = self.create_subscription(
            BuoyCoordinates, "/AverageBuoyCoordinates", self.buoy_coordinates_callback, 10)
        
        #subscribes to the GPS location
        self.gps_subscriber = self.create_subscription(
                NavSatFix, "/wamv/sensors/gps/gps/fix", self.gps_callback,10)

    def gps_callback(self, gps):
        self.boat_latitude = gps.latitude
        self.boat_longitude = gps.longitude

    def buoy_coordinates_callback(self, BuoyCoordinatesOutput):
        self.latitudes = BuoyCoordinatesOutput.latitudes
        self.longitudes = BuoyCoordinatesOutput.longitudes
        self.types = BuoyCoordinatesOutput.types
        self.ids = BuoyCoordinatesOutput.ids
        taskTwo(self,self.latitudes, self.longitudes, self.types)
        
class Buoy:
    def __init__(self, t, long, lat):
        self.type = t
        self.longitude = long
        self.latitude = lat
    def __str__(self):
        return f"{self.type} {self.latitude} {self.longitude}"

    def __repr__(self):
        return f"{self.type} {self.latitude} {self.longitude}"
    def __eq__(self,other):
        return self.type==other.type and self.latitude==other.latitude and self.longitude==other.longitude

def taskTwo(self, latitudes, longitudes, types):
    #find the closest green buoy to every red buoy
    #compute the midpoint between the buoy pairs
    #also find the midpoint between the closest yellow or black buoy and the first green and red buoy
    #find closeset midpoint, navigate to it
    redBallList = []
    greenBallList = []
    yellowBallList = []
    blackBallList = []
    midpointList = []
    print("types: "+str(types))
    for i, b in enumerate(types):
        if b == "red_buoy":
            redBallList.append(Buoy(b,longitudes[i],latitudes[i]))
        elif b== "green_buoy":
            greenBallList.append(Buoy(b,longitudes[i],latitudes[i]))
        elif b == "yellow_buoy":
            yellowBallList.append(Buoy(b,longitudes[i],latitudes[i]))
        elif b == "black_buoy":
            blackBallList.append(Buoy(b,longitudes[i],latitudes[i]))
    

    print(redBallList)
    print(greenBallList)
    closest = Buoy("blank",0,0)
    distance = 100000
    for yellowBall in yellowBallList:
        for redBall in redBallList:
            x = yellowBall.latitude - redBall.latitude
            y = yellowBall.longitude - redBall.longitude
            print("redDist: "+str(math.sqrt(x*x+y*y)))
            if(math.sqrt(x*x+y*y)<distance):
                closest = redBall
                distance = math.sqrt(x*x+y*y)
        for greenBall in greenBallList:
            x = yellowBall.latitude - greenBall.latitude
            y = yellowBall.longitude - greenBall.longitude
            print("greenDist: "+str(math.sqrt(x*x+y*y))+"dist: "+str(distance))
            if(math.sqrt(x*x+y*y)<distance):
                closest = greenBall
                distance = math.sqrt(x*x+y*y)
        #print("closest: "+str(closest))
        #print("in:"+str(redBallList.index(closest)))
        if closest.type=="red_buoy":
            redBallList.remove(closest)
            print("redList: "+str(redBallList))
        elif closest.type=="green_buoy":
            greenBallList.remove(closest)
        distance = 10000     
   

    distance = 10000
    for blackBall in blackBallList:
        for redBall in redBallList:
            x = blackBall.latitude - redBall.latitude
            y = blackBall.longitude - redBall.longitude
            if(math.sqrt(x*x+y*y)<distance):
                closest = redBall
                distance = math.sqrt(x*x+y*y)
        for greenBall in greenBallList:
            x = blackBall.latitude - greenBall.latitude
            y = blackBall.longitude - greenBall.longitude
            if(math.sqrt(x*x+y*y)<distance):
                closest = greenBall
                distance = math.sqrt(x*x+y*y)
        if closest.type=="red_buoy":
            redBallList.remove(closest)
        elif closest.type=="green_buoy":
            greenBallList.remove(closest)
        distance = 10000    
    print("redList: "+str(redBallList))
    buoyList = redBallList + greenBallList + yellowBallList + blackBallList

    nodes = {}
    for i, buoy in enumerate(buoyList):
        print("type: "+str(buoy.type))
        if buoy.type=="red_buoy":
            nodes["red"+str(self.ids[i])]=(buoy.longitude,buoy.latitude)
        elif buoy.type=="green_buoy":
            nodes["green"+str(self.ids[i])]=(buoy.longitude,buoy.latitude)
        elif buoy.type=="black_buoy":
            nodes["black"+str(self.ids[i])]=(buoy.longitude,buoy.latitude)
        elif buoy.type=="yellow_buoy":
            nodes["yellow"+str(self.ids[i])]=(buoy.longitude,buoy.latitude)
    nodes["boat"] = (self.boat_longitude,self.boat_latitude)
    print("nodes: "+str(nodes))

    G = nx.Graph()
    #G = nx.barabasi_albert_graph(len(nodes),2)
    G.add_nodes_from(nodes)

    pos = nx.spring_layout(G, seed=42)

    nx.draw(G,pos=nodes,with_labels=True, node_color="skyblue",edge_color="gray")
    plt.show()
    for i, buoy in enumerate(buoyList):
        for buoy2 in buoyList[(i+1):]:
            #print("buoy1: "+str(buoy))
            #print("buoy2: "+str(buoy2))
            #if distance between buoy and buoy2 is less than some maxdistance
            #no overlapping with another preexisting midpoint or buoy
            #
            midpointList.append(Buoy("midpoint", (buoy.longitude+buoy2.longitude)/2, (buoy.latitude+buoy2.latitude)/2))
    print("buoyList: "+str(buoyList))
    print("midpointList: "+str(midpointList))
    print()
    
    endPoint = Buoy("test",0,0)
    endPointIndex = -1
    maxDistance = 0

    startPoint = Buoy("test",0,0)
    startPointIndex = -1
    minDistance = 10000
    for i, midpoint in enumerate(midpointList):
        lat_dist = self.boat_latitude-midpoint.latitude
        long_dist = self.boat_longitude-midpoint.longitude
        
        dist = math.sqrt(lat_dist**2+long_dist**2)
        if(dist>maxDistance):
            maxDistance = dist
            endPoint = midpoint
            endPointIndex = i
        if(dist<minDistance):
            minDistance = dist
            startPoint = midpoint
            startPointIndex = i
    print("endPoint: "+str(endPoint))
    
    adj_mat = np.empty((len(midpointList),len(midpointList)))
    
    for i, point1 in enumerate(midpointList):
        minDist1 = 10000
        minDist1Index = -1
        minDist2 = 100000
        minDist2Index = -1
        for j, point2 in enumerate(midpointList):
            if i==j:
                continue
            dist = math.sqrt((point1.latitude-point2.latitude)**2+(point1.longitude-point2.longitude)**2)
            point1_to_endPoint = math.sqrt((point1.latitude-endPoint.latitude)**2+(point1.longitude-endPoint.longitude)**2)
            point2_to_endPoint = math.sqrt((point2.latitude-endPoint.latitude)**2+(point2.longitude-endPoint.longitude)**2)
            #find distance between end buoy
            #if distance closer to end buoy than point1 is to end buoy then its a valid connection
            if point2_to_endPoint < point1_to_endPoint and dist < minDist1:
                minDist2 = minDist1
                minDist2Index = minDist1Index

                minDist1 = dist
                minDist1Index = j
            elif point2_to_endPoint < point1_to_endPoint and dist<minDist2:
                minDist2 = dist
                minDist2Index = j
            #print("minDist1: "+str(minDist1))
            #print("minDist2: "+str(minDist2))
        if minDist1Index != -1:
            adj_mat[i][minDist1Index] = minDist1
        if minDist2Index != -1:
            adj_mat[i][minDist2Index] = minDist2
        #print(adj_mat[i])
        
    #G = nx.from_numpy_array(adj_mat)

    #pos = nx.spring_layout(G)
    #nx.draw(G, pos, with_labels=True, cmap='viridis', node_color="skyblue", font_color="black", font_weight = "bold")

    print("start index: "+str(startPointIndex))
    print("end index: "+str(endPointIndex))
    #plt.show()
    #path = nx.shortest_path(G,startPointIndex,endPointIndex,weight="weight")
    #print(path)

    #for x in path:
        #print(str(midpointList[x]))

    #dist_matrix, predecessors = dijkstra(csgraph=adj_mat, directed=True, indices=startPointIndex, return_predecessors=True)

    #print("dist_matrix: "+str(dist_matrix))
    #print("predecessors: "+str(predecessors))
    #print("path length: "+str(dist_matrix[endPointIndex]))
#for x in adj_mat:
        #print(x)
    #print(adj_mat)
def main(args=None):
    rclpy.init(args=args)
    node = locate_buoys()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
