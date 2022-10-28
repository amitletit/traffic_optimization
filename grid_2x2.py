#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import numpy as np
import random
import optparse
import colorsys
import pyqubo
from sumolib import checkBinary  # noqa
import traci  # noqa
import neal
from pyqubo import Binary
from pyqubo import Array
import re


# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa


def gen_grid(fname, lanes):
	#os.system("netgenerate  -j traffic_light -S 20 -o {} --grid --grid.number=2  -L{} --grid.length=200 --grid.attach-length=200".format(fname, lanes))  
    os.system("netgenerate --tls.guess -S 20 -o {} --grid --grid.number=2  -L{} --grid.length=700 --grid.attach-length=300 --turn-lanes 1 --turn-lanes.length 100 --tls.left-green.time 10".format(fname, lanes))
    # max 20 m/s
		
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

def gen_flow_from_od(outflow, taz, od):
	os.system("od2trips.exe --taz-files {} --od-matrix-files {} --flow-output {}".format(taz, od, outflow))	
    ### generate trip instead of flow:   od2trips  -d .\od.txt -n .\taz.xml -o trip.gen.xml 
    ### duarouter --route-files=.\trip.gen.xml --net-file=.\net.net.xml --output-file=route.gen.xml --taz-files=.\taz.xml --with-taz

def info():

    edges=traci.edge.getIDList()
    for e in edges:
        if e[0]!=':': 
            print(" E: ", e)
            
    tls = traci.trafficlight.getIDList()
    for t in tls:
       #clanes = traci.trafficlight.getControlledLanes(t)
       #for clane in clanes: print("    Lane: ", clane)
       print("=== TL: ", t)
       phases = traci.trafficlight.getAllProgramLogics(t)[0].getPhases()
       for p in phases:
            print(p.state)
       
       clinks = traci.trafficlight.getControlledLinks(t)
       for cl in clinks:
            print("    Link: ", cl)
            
    juncs = traci.junction.getIDList()
    for j in juncs:
        if j[0]!=':': print(" J: ",j)
        
def update_carstat():
   cars = traci.vehicle.getIDList()
   for c in cars:
     v = traci.vehicle.getSpeed(c)/ traci.vehicle.getMaxSpeed(c)
     r,g,b = colorsys.hsv_to_rgb(1-v, 1.0, 1.0)
     traci.vehicle.setColor(c, (255*r, 255*g, 255*b))	  	

def measure():
    #edges = traci.edge.getIDList()
    #lanes = traci.lane.getIDList()
    coeff_list = np.zeros((4,4))
    
    
    ## waiting TD at A1          
    coeff_list[0][0] = traci.lane.getLastStepVehicleNumber('top0A1.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('top0A1.200.00_1') + \
                    traci.lane.getLastStepVehicleNumber('A0A1.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('A0A1.600.00_1') +\
                    traci.lane.getLastStepVehicleNumber('top0A1_0')
    ## waiting TD left-turn at A1                   
    coeff_list[0][1] = traci.lane.getLastStepVehicleNumber('top0A1.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('A0A1.600.00_2') +\
                    traci.lane.getLastStepVehicleNumber('top0A1_1')
    ## waiting LR   at A1           ## more car stuck behind, more probable the phase is
    coeff_list[0][2] = traci.lane.getLastStepVehicleNumber('left1A1.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('left1A1.200.00_1') + \
                    traci.lane.getLastStepVehicleNumber('B1A1.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('B1A1.600.00_1') +\
                    traci.lane.getLastStepVehicleNumber('left1A1_0')
    ## waiting LR left-turn  at A1  
    coeff_list[0][3] = traci.lane.getLastStepVehicleNumber('left1A1.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('B1A1.600.00_2') +\
                    traci.lane.getLastStepVehicleNumber('left1A1_1')
    

    ## waiting TD at B1         
    coeff_list[1][0] = traci.lane.getLastStepVehicleNumber('top1B1.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('top1B1.200.00_1') + \
                    traci.lane.getLastStepVehicleNumber('B0B1.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('B0B1.600.00_1') +\
                    traci.lane.getLastStepVehicleNumber('top1B1_0')
    ## waiting TD left-turn at B1                   
    coeff_list[1][1] = traci.lane.getLastStepVehicleNumber('top1B1.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('B0B1.600.00_2') +\
                    traci.lane.getLastStepVehicleNumber('top1B1_1')
    ## waiting LR   at B1            ## more car stuck behind, more probable the phase is
    coeff_list[1][2] = traci.lane.getLastStepVehicleNumber('A1B1.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('A1B1.600.00_1') + \
                    traci.lane.getLastStepVehicleNumber('right1B1.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('right1B1.200.00_1') +\
                    traci.lane.getLastStepVehicleNumber('right1B1_0')
    ## waiting LR left-turn  at B1  
    coeff_list[1][3] = traci.lane.getLastStepVehicleNumber('A1B1.600.00_2') + \
                    traci.lane.getLastStepVehicleNumber('right1B1.200.00_2') +\
                    traci.lane.getLastStepVehicleNumber('right1B1_1')
                    
                                      
                    
    ## waiting TD at A0           
    coeff_list[2][0] = traci.lane.getLastStepVehicleNumber('A1A0.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('A1A0.600.00_1') + \
                    traci.lane.getLastStepVehicleNumber('bottom0A0.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('bottom0A0.200.00_1') +\
                    traci.lane.getLastStepVehicleNumber('bottom0A0_0')
    ## waiting TD left-turn at A0                    
    coeff_list[2][1] = traci.lane.getLastStepVehicleNumber('A1A0.600.00_2') + \
                    traci.lane.getLastStepVehicleNumber('bottom0A0.200.00_2') +\
                    traci.lane.getLastStepVehicleNumber('bottom0A0_1')
    ## waiting LR   at A0           ## more car stuck behind, more probable the phase is
    coeff_list[2][2] = traci.lane.getLastStepVehicleNumber('left0A0.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('left0A0.200.00_1') + \
                    traci.lane.getLastStepVehicleNumber('B0A0.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('B0A0.600.00_1') +\
                    traci.lane.getLastStepVehicleNumber('left0A0_0')
    ## waiting LR left-turn  at A0  
    coeff_list[2][3] = traci.lane.getLastStepVehicleNumber('left0A0.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('B0A0.600.00_2') + \
                    traci.lane.getLastStepVehicleNumber('left0A0_1')

    ## waiting TD at B0          
    coeff_list[3][0] = traci.lane.getLastStepVehicleNumber('B1B0.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('B1B0.600.00_1') + \
                    traci.lane.getLastStepVehicleNumber('bottom1B0.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('bottom1B0.200.00_1') +\
                    traci.lane.getLastStepVehicleNumber('bottom1B0_0')
    ## waiting TD left-turn at B0                   
    coeff_list[3][1] = traci.lane.getLastStepVehicleNumber('B1B0.600.00_2') + \
                    traci.lane.getLastStepVehicleNumber('bottom1B0.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('bottom1B0_1')
    ## waiting LR   at B0           ## more car stuck behind, more probable the phase is
    coeff_list[3][2] = traci.lane.getLastStepVehicleNumber('A0B0.600.00_0') + \
                    traci.lane.getLastStepVehicleNumber('A0B0.600.00_1') + \
                    traci.lane.getLastStepVehicleNumber('right0B0.200.00_0') + \
                    traci.lane.getLastStepVehicleNumber('right0B0.200.00_1') +\
                    traci.lane.getLastStepVehicleNumber('right0B0_0')
    ## waiting LR left-turn  at B0   
    coeff_list[3][3] = traci.lane.getLastStepVehicleNumber('A0B0.600.00_2') + \
                    traci.lane.getLastStepVehicleNumber('right0B0.200.00_2') + \
                    traci.lane.getLastStepVehicleNumber('right0B0_1')
                    
    
    
    return coeff_list
    
    
    #n = list(map(traci.edge.getLastStepVehicleNumber, edges))
    #print(" -- #car: ",  list(map(traci.edge.getLastStepVehicleNumber, edges)))
    
def qubo(Coeff):
        lamda1 = -1; 
        #lamda2 = 60; 
        #lamda3 = 20; 
        lamda4 = 100;
        x = Array.create('x', shape=(4), vartype='BINARY')
        Q_mode_constrain = (1 - sum(x))**2
        #print("      Coeff: {}".format(Coeff[i]) )
        Q1 = sum(x*Coeff)
        H = lamda1*Q1 + lamda4*Q_mode_constrain
        model = H.compile()
        bqm = model.to_bqm()
        sa = neal.SimulatedAnnealingSampler()
        sampleset = sa.sample(bqm, num_reads=20)
        decoded_samples = model.decode_sampleset(sampleset)
        best_sample = min(decoded_samples, key=lambda x: x.energy)
        opt = best_sample.sample
        
        return opt

    
def qubo2phase(qubo):
    for key in qubo:   ## x_{0j}
        if qubo[key] == 1:
            # return 0,2,4,6        
            return 2*int(key[2:-1])    
        
def quboopt(opt_qubo): ## output the optimized signal number
    for key in opt_qubo:   
        if opt_qubo[key] == 1:
            key = re.sub("\D", "", key)
            key = int (key)
            return key 
           
    
tls = ["A0", "A1", "B0", "B1"]		
def control():
    print(" -- TL modes: ", list(map(traci.trafficlight.getPhase, tls)))

def run():
    """execute the TraCI control loop"""

    step = 0
    ss = 5; ss1 = 5; ss2 = 5; ss3 = 5;
    
    while step < 3600:
        traci.simulationStep()
        update_carstat()
        #NUM_PHASE=8
        TLSs = traci.trafficlight.getIDList()
        print(TLSs)
        dt = dict()
        gt = dict()
        nextphase = dict()
        duration  = dict()
        Coeff = measure();
        for s in TLSs:
            dt[s]        = 5     ## transition time
            gt[s]        = 30    ## minimal green/red  time
            nextphase[s] = 0     ## next phase
            duration[s]  = 0     ## how long it has been in "cphase"
        
        
        if step%10 == 0: 
          print("Step: {:5}  # Car: {}".format(step, traci.vehicle.getIDCount()))       
        
        # for junction 0
        if step%ss == 0: 
          
          opt0 = qubo(Coeff[0]);
          opt_phase0 = qubo2phase(opt0)
          print(opt_phase0)
          opt_signal0 = quboopt(opt0)
          current_phase0 = traci.trafficlight.getPhase('A1')
          print(Coeff[(0,opt_signal0)])
          # if the qubo tell signle should be changed
          if abs(opt_phase0 - current_phase0) != 0:
              traci.trafficlight.setPhase("A1",opt_phase0)    ##setting phase
              traci.trafficlight.setPhaseDuration('A1', int(Coeff[(0,opt_signal0)])) ##we can also change the time of traffic signals
              
              
              #duration['A1'] = 0
              #print(current_phase0,opt_phase0)
              #gt['A1'] = Coeff[(0,opt_signal0)]*0.8;    ## minimal green/red  time
              #measure()   
          #time0 = traci.trafficlight.getPhaseDuration("A1") 
          time0 = int(Coeff[(0,opt_signal0)])
          print(time0)
          ss = 5 + time0+step;
          


        # for junction 1
        if step%ss1 == 0: 
          opt1 = qubo(Coeff[1]);
          print(opt1)
          opt_phase1 = qubo2phase(opt1)
          print(opt_phase1)
          opt_signal1 = quboopt(opt1)
          current_phase1 = traci.trafficlight.getPhase('B1')
          print(Coeff[(1,opt_signal1)])
          # if the qubo tell signle should be changed
          if abs(opt_phase1 - current_phase1) != 0:
              traci.trafficlight.setPhase("B1",opt_phase1)    ##setting phase
              traci.trafficlight.setPhaseDuration('B1', int(Coeff[(1,opt_signal1)]))
              
              
              #traci.trafficlight.setPhaseDuration("A1", int(Coeff[(0,opt_signal0)]*0.8)) ##we can also change the time of traffic signals
              
              
              #duration['A1'] = 0
              #print(current_phase0,opt_phase0)
              #gt['A1'] = Coeff[(0,opt_signal0)]*0.8;    ## minimal green/red  time
              #measure()   
          #time1 = traci.trafficlight.getPhaseDuration("B1") 
          time1 = int(Coeff[(1,opt_signal1)])
          print(time1)
          ss1 = 5 + time1 +step;
          #measure()


        # for junction 2
        if step%ss2 == 0: 
          opt2 = qubo(Coeff[2]);
          print(opt2)
          #measure()
          opt_phase2 = qubo2phase(opt2)
          print(opt_phase2)
          opt_signal2 = quboopt(opt2)
          current_phase2 = traci.trafficlight.getPhase('A0')
          print(Coeff[(2,opt_signal2)])
          # if the qubo tell signle should be changed
          if abs(opt_phase2 - current_phase2) != 0:
              traci.trafficlight.setPhase("A0",opt_phase2)    ##setting phase
              traci.trafficlight.setPhaseDuration('A0', int(Coeff[(2,opt_signal2)]))
              #traci.trafficlight.setPhaseDuration("A1", int(Coeff[(0,opt_signal0)]*0.8)) ##we can also change the time of traffic signals
              
              
              #duration['A1'] = 0
              #print(current_phase0,opt_phase0)
              #gt['A1'] = Coeff[(0,opt_signal0)]*0.8;    ## minimal green/red  time
              #measure()   
          #time2 = traci.trafficlight.getPhaseDuration("A0") 
          time2 = int(Coeff[(2,opt_signal2)])
          print(time2)
          ss2 = 5 + time2 +step;
          #measure()


        # for junction 3
        if step%ss3 == 0: 
          opt3 = qubo(Coeff[3]);
          print(opt3)
          #measure()
          
          opt_phase3 = qubo2phase(opt3)
          print(opt_phase3)
          opt_signal3 = quboopt(opt3)
          current_phase3 = traci.trafficlight.getPhase('B0')
          print(Coeff[(3,opt_signal3)])
          # if the qubo tell signle should be changed
          if abs(opt_phase3 - current_phase3) != 0:
              traci.trafficlight.setPhase("B0",opt_phase3)    ##setting phase
              traci.trafficlight.setPhaseDuration('B0', int(Coeff[(3,opt_signal3)]))
              
              
              #traci.trafficlight.setPhaseDuration("A1", int(Coeff[(0,opt_signal0)]*0.8)) ##we can also change the time of traffic signals
              
              
              #duration['A1'] = 0
              #print(current_phase0,opt_phase0)
              #gt['A1'] = Coeff[(0,opt_signal0)]*0.8;    ## minimal green/red  time
              #measure()   
          #time3 = traci.trafficlight.getPhaseDuration("B0") 
          time3 = int(Coeff[(3,opt_signal3)])
          print(time3)
          ss3 = 5 + time3 +step;
          
        
        #if step%10 == 0: control()
        step += 1
    
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    lanes = 2
    gen_grid("gen.net.xml", lanes)
    gen_taz("gen.taz.xml")
    gen_flow_from_od("gen.flow.xml", "gen.taz.xml", "od.txt")

    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
     
    traci.start([sumoBinary, "-c", "main.sumocfg", "--statistic-output" , "stat.log"])

    info()

    run()
	
