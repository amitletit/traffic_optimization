#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import colorsys
import pyqubo

import sumolib
from sumolib import checkBinary  # noqa

# Generate toy network system
def gen_grid(fname, lanes, grids = 2):
	#os.system("netgenerate  -j traffic_light -S 20 -o {} --grid --grid.number=2  -L{} --grid.length=200 --grid.attach-length=200".format(fname, lanes))  
    os.system("netgenerate --tls.guess -S 20 -o {} --grid --grid.number={}  -L{} --grid.length=700 --grid.attach-length=300 --turn-lanes 1 --turn-lanes.length 100 --tls.left-green.time 10".format(fname, grids, lanes))
    # max 20 m/s
		
# Generate traffic zone file
def gen_taz(afile):
    with open(afile, "w") as f:
        print("""<additional>
		         <tazs>
                   <taz id="left">
                     <tazSource id="left0A0" weight="0.9"/><tazSource id="left1A1" weight="0.5"/>
                     <tazSink id="A0left0" weight="0.5"/>  <tazSink id="A1left1" weight="0.5"/>
                   </taz>
                   <taz id="top">
                     <tazSource id="top0A1" weight="0.5"/><tazSource id="top1B1" weight="0.5"/>
                     <tazSink id="A1top0" weight="0.5"/>  <tazSink id="B1top1" weight="0.5"/>
                   </taz>
                   <taz id="right">
                     <tazSource id="right0B0" weight="0.5"/><tazSource id="right1B1" weight="0.5"/>
                     <tazSink id="B0right0" weight="0.5"/>  <tazSink id="B1right1" weight="0.5"/>
                   </taz>
                   <taz id="bottom">
                     <tazSource id="bottom0A0" weight="0.5"/><tazSource id="bottom1B0" weight="0.5"/>
                     <tazSink id="A0bottom0" weight="0.5"/>  <tazSink id="B0bottom1" weight="0.5"/>
                   </taz>
                 </tazs>
				 </additional>""", file=f)

# Generate traffic flow from OD input file
def gen_flow_from_od(outflow, taz, od):
	os.system("od2trips.exe --taz-files {} --od-matrix-files {} --flow-output {}".format(taz, od, outflow))	
    ### generate trip instead of flow:   od2trips  -d .\od.txt -n .\taz.xml -o trip.gen.xml 
    ### duarouter --route-files=.\trip.gen.xml --net-file=.\net.net.xml --output-file=route.gen.xml --taz-files=.\taz.xml --with-taz

# build lane2mode table for easily construct QUBO coeff.
# We assume each lane only active for one mode.
def genLane2modeTbl(modes):
    lane2mode = dict()
    for mi, m in zip(range(len(modes)), modes):
        state = m.state   ## return, say, "GGGrrrrrrrGGGrrrrrrr" for 20 lanes
        for i in range(len(state)):
            if state[i] == 'G' or state[i] == 'g':
                lane2mode[i] =  mi
    return lane2mode
    

if __name__ == "__main__":

    lanes = 2
    grids = 2
    ODFILE   = "od.txt"          ## in
    NETFILE  = "gen.net.xml"
    TAZFILE  = "gen.taz.xml"
    FLOWFILE = "gen.flow.xml"
    
    gen_grid(NETFILE, lanes, grids)
    gen_taz(TAZFILE)
    gen_flow_from_od(FLOWFILE, TAZFILE, ODFILE)

    net = sumolib.net.readNet(NETFILE, withPrograms = True, withInternal = False)
    tls   = net.getTrafficLights()
    ntls  = len(tls)
    
    print("====== Network summary ======")
    print("Network boundary: ", net.getBBoxXY())
    print("Network diameter: ", net.getBBoxDiameter())
    
    print("Found {} traffic lights:".format(ntls))
    for tl in tls: 
        print ("  ", tl.getID())
    
    #nodes = net.getNodes()
    #for node in nodes: print (node.getID())
    #edges = net.getEdges()
    #for edge in edges: print (edge.getID())
    #links = tl.getLinks()
    #for l in links:             print(l)

    cij = dict()
    for ntl, tl in zip(range(len(tls)), tls):
        tid    = tl.getID()
        conns = tl.getConnections()
        modes = tl.getPrograms()['0'].getPhases()
        
        lane2mode = genLane2modeTbl(modes)
 
        # for each connection of this junction, add (up, down)-stream lane for QUBO coeff ....
        for c in conns:
            I, J = ntl, lane2mode[c[2]]   # index for QUBO coeff C_{I,J}
            if (I,J) not in cij: cij[(I,J)]=[]
            cij[(I,J)] += [ (c[0].getID(), c[1].getID()) ]
    
    # From the generated table, we see that only the upstream lane is needed to build a minimal QUBO system.
    # More sophisticated QUBO cost function may be built from including also downstream lane.
    # To including nearest neighbor correlation of a junction X, we may have to trace the network to find junctions Y with uptream lane equal to the downstream of X.
    # A challange is that if the network contain may virtual node (w/o TL), the the tracing may not be so easy.
    print("=== QUBO coeff table ===")
    for i in cij:
        print (i)
        for j in cij[i]:
            print ("  ", j)
