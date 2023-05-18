Here's an example Python code for using pre-generated route GPS coordinates from a file to generate a SUMO route file:

python

import csv
from sumolib import net

# define the path to the SUMO network file
net_file = 'path/to/your/sumo/network/file.net.xml'

# define the path to the GPS coordinates file
gps_file = 'path/to/your/gps/coordinates/file.csv'

# create a SUMO route file and write the header information
route_file = open('route_file.rou.xml', 'w')
route_file.write('<routes>\n')

# load the network file and extract the node IDs
net_data = net.readNet(net_file)
node_ids = [node.getID() for node in net_data.getNodes()]

# read the GPS coordinates from the file
with open(gps_file, 'r') as csvfile:
    reader = csv.reader(csvfile)
    coords = [tuple(map(float, row)) for row in reader]

# iterate through the GPS coordinates and find the closest node to each point
node_ids = []
for i in range(len(coords)):
    node = min(node_ids, key=lambda x: net_data.getDistance(x, coords[i]))
    node_ids.append(node)

# write the route information to the route file
route_file.write('    <vehicle id="vehicle_1" depart="0.0" departLane="best" departSpeed="max">\n')
route_file.write('        <route edges="{}"/>\n'.format(' '.join(net_data.getShortestPath(node_ids))))
route_file.write('    </vehicle>\n')

# write the closing tag and close the file
route_file.write('</routes>\n')
route_file.close()
