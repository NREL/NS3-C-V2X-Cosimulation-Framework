# N-S3 Cellular Vehicle-to-Everything (C-V2X) Cosimulation Framework
## Contributors:
Maxwell McManus <Maxwell.McManus@nrel.gov>
<br>
Qichao Wang <Qichao.Wang@nrel.gov>
<br>
Nicholas Accurso <Nicholas.Accurso@nrel.gov>
### Publication
This software is identified under NREL Software Record Number SWR-24-54. 

### Purpose
To provide a configurable, scalable C-V2X Mode 4 simulator intended for synchronous co-simulation with the Aimsun NEXT vehicle traffic simulator. 
The simulation federate configuration parameters are vehicle scenario area, number of vehicles, vehicle coordinates, maximum communication range between nodes, and network protocol configuration. For co-simulation, the input will also include the IP address and port of the associated HELICS broker. The outputs include: number of ns-3 instances required to simulate the networking area, network-level packet reception ratio and drop rate, and packet-level network performance, including message receptions, message drops, and latency. In each synchronous simulation step, the network simulation is designed to receive node ID, coordinates, and relevant transmission triggers from the vehicle simulator, and will return the associated transmission/reception results to the traffic simulator. The C-V2X protocol stack integrated with this framework can be found at https://github.com/FabianEckermann/ns-3_c-v2x.  


### This repo contains the following items:
- Core code for the software
	* For synchronous co-simulation: workspace_code/ns3-starter.py
	* For standalone operation (network simulation only): workspace_code/solo-ns3-starter.py
- Input data
	* For synchronous co-simulation, input is provided by Aimsun NEXT V2X SDK.
	* For co-simulation testing without independent vehicle simulation: workspace_code/aimsun-proxy.py
	* For standalone operation, simulation configuration is input via command line: workspace_code/solo-ns3-starter.py

All network simulation parameters (i.e. channel allocation, TX power, V2X message set, etc.) and co-simulation default parameters (i.e. runtime, simulation time per step, number of vehicles, etc.) found in ~/cv2x-cosim/workspace_code/scratch/v2x_sim/v2x_sim.cc. The test program (aimsun-proxy.py)  replicates the data transfer expected from external vehicle simulation, including vehicle IDs, vehicle location data, and vehicle broadcast messages. Note that the test program does not simulate any vehicle traffic - the generated data points are placeholders designed to enable standalone execution of the network simulation framework.  


## Installation instructions
### 0. Dependencies
This program requires both Python 2.7 and Python 3.10. The recommended configuration is to run the network simulation starter from a conda environment with Python 3.10 and HELICS 3.4.0 installed, and create a new environment named "cv2x" with Python 2.7 and C++/G++=11 installed. The starter script will deploy ns-3 instances in the "cv2x" environment. If there are any issues with dependencies for specific modules of ns-3, more details can be found here: https://www.nsnam.org/wiki/Installation, following recommendations for ns-3.28. Some knowledge of the HELICS co-simulation framework may be required, and can be found here: https://helics.org.
## Installation instructions
### 1. Clone this repo
In terminal, type
``` bash
git clone https://github.nrel.com/CDA/cv2x-cosim 
```
### 2. Build and configure
``` bash
$ cd cv2x-cosim  
$ ./waf clean  
$ ./waf configure --build-profile=optimized --disable-python --disable-gtk   
```
Optional flags:  
"--enable-examples --enable-tests": enable ns-3 examples and tests   
"--enable-mpi": enable MPI   
"--disable-gtk": needed for running on HPC with ns-3 < 3.29   
"--build-profile=debug": configure ns-3 in debug mode, which is required for integrating optional modules   

If build fails with message “__all warnings being treated as errors__”, try:  
``` bash
$ CXXFLAGS=“-Wall” ./waf configure --build-profile=optimized --disable-python --enable-examples --enable-tests  
```

### 2. Compile
``` bash   
$ ./waf  
```

### 3. Start simulation
Depending on the observed simulation area specified by the vehicle traffic simulator, multiple instances of ns-3 may start on dedicated hardware threads. There are three supported co-simulation modes - local broker, remote broker via E-broker, and alternate remote broker. Local broker is intended for co-simulation on a single machine, and uses ZMQ for message passing across the HELICS broker. Remote broker via E-broker specifies the HELICS broker located on the E-broker node of the NREL Eagle (soon Kestrel) HPC system. Specifying an alternate remote broker via IP address and port number is also supported.  

#### First configuration: using E-Broker
To run the co-simulation using the HPC E-broker, SSH into the HPC E-Broker node:
``` bash
ssh [username]@ebroker.hpc.nrel.gov
```
Once in E-broker, run the following command: 
``` bash
helics_broker -f 2 -t tcp-ss
```
On another HPC node or local machine, start the vehicle traffic simulator federate. 
To start the synchronous network simulation, run the following: 
``` bash  
python3 ns3-starter.py -r true  
```
Note: it is recommended to run the network federate on HPC when simulating large vehicle scenarios. 

#### Second configuration: using non-HPC remote broker
On the desired broker node, run the following command:
``` bash
helics_broker -f 2 -t tcp-ss
```
Start the vehicle traffic simulator federate. 
To start the synchronous network simulation, run the following: 
``` bash  
python3 ns3-starter.py -r new  
```
The program will issue prompts for the IP address and port of the remote broker. 

#### Third configuration: local execution
To run the co-simulation locally, run the following command in a terminal window:
``` bash
helics_broker -f 2  
```
Start the vehicle traffic simulator federate. Alternatively, start the traffic simulation proxy federate in a second terminal:
``` bash
python3 aimsun-proxy.py
```
To start the synchronous network simulation, run the following in a third terminal: 
``` bash  
python3 ns3-starter.py -r false  
```
<!--
NOTE: this software is designed to interface with the HELICS co-simulation framework, and as such both Python 2.7 and Python 3.10 will need to be installed. If ns-3 cannot find Python in "/usr/bin/env/" after the installation, the system may not have an executable named "python", which ns-3 requires. Double-check that both "python2" and "python3" executables are available in "/usr/bin/env/", then create a symbolic link to "python2" (or update ns-3 path variables, if preferred): 
``` bash
$ sudo ln -s /usr/bin/python2 /usr/bin/python
```
-->
