from contextlib import closing
import argparse 
import json
import math 
import multiprocessing
# import os 
import random 
import signal
import socket 
import struct
import subprocess
import sys
import threading 
import time

# --> NOTE: this version of ns3-starter.py only starts network simulation and does not interface with HELICS



def ns3_process(coord_tuple):
    """
    This is where we will define unique aspects of each ns-3 instance via "args" 

    Runtime is measured using "instance_return", but other metrics are extracted via HELICS 

    For newer versions of Python (i.e. 3.5+), recommend refactor code to use "run()" 
    NOTE: recommend avoid PIPE for stdin/stderr/stdout for launching ns-3 - process will block when PIPE is full unless carefully managed 
    NOTE: for any subprocess command (i.e. .call, .Popen, .run, etc.), recommend never use "shell=True" - could lead to shell injection 
    """

    # extract coord list 
    inst_idx = coord_tuple[0]
    io_port = coord_tuple[1]
    wrapper_port = coord_tuple[2]
    og_x = coord_tuple[3]
    og_y = coord_tuple[4]

    # compile bash string 
    prog_w_args = "v2x_sim --inst_idx={} --io_port={} --wrapper_port={} --origin_x={} --origin_y={} --reporting=1".format(inst_idx, io_port, wrapper_port, og_x, og_y) 

    print("Starting ns-3 instance {}".format(inst_idx)) 

    # subprocess call will block function execution (intended) to prevent early program exit 
    # --> usually ".call()" should hold up until process finishes - does not do that here, likely returns after ns-3 instance is started 
    # return_val = subprocess.call(["gnome-terminal", "-x", "./waf", "--run", prog_w_args]) 
    # return return_val 

    # subprocess.call(["gnome-terminal", "-x", "./waf", "--run", prog_w_args]) # works, but terminal closes immediately after ns-3 executes 
    # subprocess.call(["xterm", "-hold", "-e", "./waf", "--run", prog_w_args]) 

    subprocess.call(["conda", "run", "-n", "cv2x", "./waf", "--run", prog_w_args]) 

    # print("[ns3_process] Sim instance {} finished! Returning with value {}".format(inst_idx, return_val)) 

    return 





# automatically determine number of required simulation instances based on observed area 
def num_req_instances(obs_x, obs_y, min_ns3_dim): 
    """
    For simplicity - simulation area needs to overlap 50% with all adjacent instances 
    - this way, all possible V2V links will exist in at least 1 simulation 
    """
    min_ns3_dim = 300   # double maximum range for geonetworking range limits (i.e. double "valid" packet range for V2X)
    overlap = 0.5       # amount of overlap between simulation instances 

    # number of instances required to guarantee exactly 50% overlap between them
    num_x = int(math.ceil((2 * obs_x) / float(min_ns3_dim)) - 1) 
    num_y = int(math.ceil((2 * obs_y) / float(min_ns3_dim)) - 1) 

    # make sure we never recommend 0 instances 
    num_x = max(num_x, 1) 
    num_y = max(num_y, 1) 

    num_instances = num_x * num_y 

    # Calculate origin of each simulation to coordinate geographic coverage

    # Overlapped coverage can be maximum 50% along any axis, any more will cause unnecessary redundancies
    # - set to exactly 50% for convenience 

    list_sim_origin_coords = []

    for ix in range(num_x):
        for iy in range(num_y):
            coord_x = int(math.ceil(ix * overlap * min_ns3_dim)) # direct conversion should be safe 
            coord_y = int(math.ceil(iy * overlap * min_ns3_dim)) # should be int anyways, and ceil helps prevent redunant sim area coverage 
            list_sim_origin_coords.append([coord_x,coord_y])

    print(list_sim_origin_coords[-1][0] + min_ns3_dim) 

    calc_sim_coverage = (list_sim_origin_coords[-1][0] + min_ns3_dim) * (list_sim_origin_coords[-1][1] + min_ns3_dim) 

    print("Requested simulation area: {} $m^2$".format(obs_x * obs_y)) 
    print("Calculated simulation area: {} $m^2$".format(calc_sim_coverage)) 

    return int(num_instances), list_sim_origin_coords 





def check_ports_open(main_port, listener_port_start, sim_inst_port_start, num_inst):
    """
    Check that all desired ports are available (up to 10k instances, based on current port config)
    --> good idea to dedicate these in a config file if these will always be open on target machine - this part is designed for portability 
    
    NOTE: port ranges are designed to be separated by 10000, i.e. for listener port 50002, the corresponding ns-3 port is 60002
    --> if different port ranges are selected in "__main__", need to update this function accordinly 
    """

    # first, check all ports in the intended ranges to make sure we can sync everything up 

    available_listener_ports = [] 
    available_sim_inst_ports = [] 

    lclport_iter = listener_port_start 
    simport_iter = sim_inst_port_start 

    with closing (socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as mainsock: 
        mainsock.settimeout(1)
        while mainsock.connect_ex(("", main_port)) != 0: 
            main_port -= 1


    # needs to be done as a loop since sockets cannot re-bind and we want to keep ports "adjacent" (constant difference)
    for inst_idx in range(num_inst):
        # open listener socket and make sure we close our connection after we check 
        with closing (socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as checksock1: 
            with closing (socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as checksock2: 
                checksock1.settimeout(1) # avoid hanging if we cannot connect - should be pretty instantaneous
                checksock2.settimeout(1) 
                if checksock1.connect_ex(("", lclport_iter)) != 0: # "connect_ex" returns 0 if connection is successful 
                    # do not record port - port not available 
                    pass
                elif checksock2.connect_ex(("", simport_iter)) != 0: 
                    # do not record port - port not available 
                    pass
                else: 
                    # both ports are open -> record available port number and close when "with" statements terminate 
                    available_listener_ports.append(lclport_iter)
                    available_sim_inst_ports.append(simport_iter)
                # iterate both port numbers to maintain constant difference 
                lclport_iter += 1
                simport_iter += 1
                # quick catch for network config problems, i.e. not enough open ports (accounting for ~10k instances seems sufficient, just change values if not)
                if lclport_iter == 60000 or simport_iter == 70000: 
                    print("No suitable ports for process configuration. Recommend careful examination of network parameters. Exiting.")
                    sys.exit(1)
                else: pass 

    # another quick check that we have enough available sockets  
    if len(available_listener_ports) != num_inst or len(available_sim_inst_ports) != num_inst:
        print("Port detection code failed - not enough ports reserved for simulation. Recommend debug. Exiting.")
        sys.exit(1)
    else: pass 

    # need to wait until socket reconnect timeout finishes up (can use "REUSE_ADDR" to avoid this, but not recommended default behavior)
    # --> NOTE: UDP sockets apparenty have no "TIME_WAIT" parameter, so calling "bind()" again should succeed 
    # time.sleep(2) 

    return main_port, available_listener_ports, available_sim_inst_ports



class instance_server(): 
    """
    Class object for listener sockets to improve asynchronous data collection and retrieval 
    - listener server object will be generated per ns-3 instance to run a threaded socket to receive data updates. 
    - member functions will enable retrieval of stored data without interrupting thread process. 
    """
    def __init__(self, inst_idx, src_port, dest_port):
        self.inst_idx = inst_idx
        self.listener_socket = None
        self.thread = None
        self.listener_port = src_port
        self.sim_inst_port = dest_port

        # set to 1 when ns-3 startup is finished and simulation is waiting for init params 
        self.instance_waiting = 0

        # "ready" flag for each instance - need to set to 1 when ns-3 finishes setup (based on initialization params)
        self.instance_ready = 0 

        # list of node IDs from Aimsun - ID translation is deployed at control plane (otherwise we need extra exchanges)
        self.curr_node_ids  = [] # list of node IDs associated with this instance in this timestep 
        self.curr_node_locs = [] # list of node locations associated with this instance in this timestep, index-matched to IDs 
        self.curr_node_msgs = [] # list of messages sent by each node in this instance in this timestep, index-matched to IDs 

        # TODO: update "collected_data" with required parameters 
        self.sim_message = None
        self.collected_data = [] # empty list for now - will be replaced with reported data 

        self.sim_runtime = None
        self.sim_timestep_ms = None 



    def start_listener(self): 
        """
        Start listener thread 
        """
        self.thread = threading.Thread(target = self.listener_loop)
        self.thread.setDaemon = True 
        self.thread.start()



    def listener_loop(self):
        """
        Generic listener thread for each ns-3 instance 
        - this thread hosts a non-blocking socket receive loop used to receive info from ns-3
        - assums ports are available - make sure port detection is successful before declaring listeners 
        """    
        sock = socket.socket(family = socket.AF_INET, type = socket.SOCK_STREAM)
        try: 
            # TCP socket setup procedure - exit with error message if any part fails 
            sock.bind(("", self.listener_port))
            sock.listen(4) # TODO: "3" not reached! 
            self.listener_socket, addr = sock.accept() # "accept()" returns actual socket object we will use - no need for "addr"
        except Exception as e: 
            print("Socket could not bind to available port, with exception: {}".format(e))
            print("\n--> Debug function 'check_ports_open' and/or reset network parameters\n")
            # exit thread, since we cannot proceed with this error 
            self.thread.exit(1) 

        print("[instance_server.listener_loop] Listening on port {}...".format(self.listener_port))

        while True: 
            b_msg = b'' # flush buffer before receiving more data - we want to isolate each byte stream 
            b_msg = self.listener_socket.recv(4096) # may need to be larger depending on performance - TCP allows for much, much larger chunks if needed 

            # detect receive EOF (i.e. connection closed on client side) 
            if not b_msg: 
                print("[instance_server.listener_loop] Inst {} connection lost!".format(self.inst_idx))
                break
            else: 
                msg = b_msg.decode()
                print("[instance_server.listener_loop] Inst {} message: {}".format(self.inst_idx, msg))

            # user-defined state-based message exchanges 
            if msg == "WAIT": 
                # instance is now ready to receive initialization data 
                self.instance_waiting = 1
                self.sim_message = [msg]
            
            elif msg == "READY": 
                # "ready" signal should only be sent once, when instance setup is complete and simulation ready to advance 
                # --> after this, we expect formatted data reporting for each subsequent timestep 
                if self.instance_ready == 1: 
                    print("Duplicate 'ready' signal from instance {}.".format(self.inst_idx))
                else: 
                    self.instance_ready = 1
                self.sim_message = [msg]

            elif msg[0:4] == "DATA": 
                # message passed in known format - process into object then 
                self.parse_reported_data(msg[5:]) # no reason to pass header "DATA:"
                self.sim_message = [msg]
            else: 
                # after 
                pass 


    def parse_reported_data(self, msg):
        """
        Format needs to be coordinated with "control-plane.cc:HelicsCtrlPlane.ReportData(...)"

        --> for now, we only report successful packet reception events 
        """
        
        # list_of_pkt_events = msg.split("+")[1:] 

        dict_rx_ok = {}
        print("Reported data: <{}>".format(msg))

        if len(msg) == 0:
            # make sure instance is reporting something 
            self.collected_data = "No data reported!"
        else: 
            good_rxs, all_txs = msg.split("+")
            dict_rx_ok = {"rx_valid": good_rxs, "num_tx": all_txs}

            self.collected_data = dict_rx_ok




    def update_ns3_state(self, init): 
        """
        Generate string to pass to ns-3 based on received data 
        --> should be thread-safe, since we have ".recv()" in own thread and we will borrow main thread for this 

        Init string format = 
        --> sent before ns-3 execution starts 

        Update string format = 
        --> sent every timestep 
        """

        id_string = ""
        loc_string = ""
        msg_string = ""

        # print("\n\nERROR")
        # print(init, len(self.curr_node_ids), len(self.curr_node_locs), self.curr_node_msgs)

        if (init == True) and (len(self.curr_node_ids) != len(self.curr_node_locs)): 
            print("Error - init data is not index-matched for instance {}! Data will be lost for this timestep.".format(self.inst_idx))
            print(f"\t> len current ids: {len(self.curr_node_ids)}, len locs: {len(self.curr_node_locs)}")

        elif (init == False) and (len(set([len(self.curr_node_ids), len(self.curr_node_locs), len(self.curr_node_msgs)])) != 1): 
            print("Error - state data is not index-matched for instance {}! Data will be lost for this timestep.".format(self.inst_idx))

        elif (init == False) and (self.instance_ready == 0):
            # cannot update data, simulation has not finished setup 
            print("Error - simulation instance {} not ready! Data will be lost for this timestep.".format(self.inst_idx))

        else: 
            # compile all lists into string of format expected by ns-3 control plane 
            # --> format: "id,id,...,id/locx:locy,locx:locy,.../msg1msg2,msg3msg0,msg1..../!"
            if init == True: 
                # simulation already initialized, use normal format 
                for elmt in range(len(self.curr_node_ids)):
                    id_string += str(self.curr_node_ids[elmt]) + ","
                    loc_string += str(self.curr_node_locs[elmt]) + ","
                full_string_update = self.sim_runtime + "/" + self.sim_timestep_ms + "/" + id_string[:-1] + "/" + loc_string[:-1] + "/!"
            else:  
                # simulation not initialized, used initialization format 
                for elmt in range(len(self.curr_node_ids)):
                    id_string += str(self.curr_node_ids[elmt]) + ","
                    loc_string += self.curr_node_locs[elmt] + "," # already strings 
                    msg_string += self.curr_node_msgs[elmt] + "," # already strings 

                full_string_update = id_string[:-1] + "/" + loc_string[:-1] + "/" + msg_string[:-1] + "/!"



            # need to pack to 4096 bytes to match TCP buffer size (or else "recv" will not work) 
            packed_string = struct.pack('4096s', full_string_update.encode()) 

            print("Sending ns3 state update: {} ({} byte object packed to {} bytes)".format(full_string_update, len(full_string_update.encode()), len(packed_string)))

            self.listener_socket.send(packed_string) # encode string to bytes for C++ - UTF-8 encoding is default 


    def retrieve_data(self):
        """
        Main thread can call this to return data collected during listening period
        """
        return self.collected_data


    def flush_data(self):
        """
        Main thread can call this to clear data list in preparation for next listening period
        """
        self.collected_data = []



def process_helics_strings(string_list): 
    """
    offload some parsing task to reduce complexity of main/init loops 

    "string_list" must be list of strings in order: [ack_string, id_string, coord_string, vel_string, msg_string]

    Returns the following index-matched lists: 
    - aimsun_ids: list of INT ID's of each node 
    - aimsun_coords_string_list: list of STRING coordinates of each node - passed directly to ns-3
    - aimsun_coords_int_list: list of INT coordinates of each node - used to sort nodes into instances 
    - aimsun_msgs: list of strings, each containing msgs sent by a node during this timestep 
    """

    if len(string_list) != 5: 
        print("[process_helics_strings:ns3-starter.py] String parsing error - missing fields.")
    else: pass 


    # quickly check if "/!" delimiter is present and strip if needed 
    ack_string, ids_string, crd_string, vel_string, msg_string = [string[:-2]  if string[-2:] == ",!"  else string for string in string_list]

    # preview received strings  
    print(f'received ACK: {ack_string}') 
    print(f'received Ids: {ids_string}')  
    print(f'received Coords: {crd_string}')  
    # print(f'received Speeds: {vel_string}')  
    print(f'received Messages: {msg_string}')  

    # list of integer IDs from Aimsun (assuming node IDs are expressed as integers)
    aimsun_ids = [int(x) for x in ids_string.split(",")] # int is easier to work with 

    # list of coordinates index-matched to node ID list --> we will pass as string through socket to ns-3 - no need to convert yet
    aimsun_coords_string_list = [coord_pair for coord_pair in crd_string.split(",")]  

    # same list of coordinates as integers --> converted to int values to sort nodes into simulation instance easier 
    aimsun_coords_int_list = [[int(coord) for coord in coord_pair.split(":")] for coord_pair in crd_string.split(",")] 

    # list of strings denoting which messages are sent by each node in each timestep (also index-matched with ID list)
    aimsun_msgs = msg_string.split(",") # easier if it is just list of strings, we will parse in ns-3 when we add to queue (any pre-processing at all?) 

    return aimsun_ids, aimsun_coords_string_list, aimsun_coords_int_list, aimsun_msgs 


def ns3_instances_complete(inst_return_infos):
    """
    Function executes once all NS3 instances have completed
    Set a flag equal to 1, so that the main loop knows when to stop
    """
    global ns3_completed
    ns3_completed = 1
    





# main program  
if (__name__ == "__main__"):    
    # get command line args 
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--num_nodes", type = int, help = "select number of nodes in simulation", default = 20)
    parser.add_argument("-t", "--sim_duration", type = int, help = "define simulation duration (in seconds)", default = 10)
    parser.add_argument("-p", "--sim_timestep_ms", type = int, help = "define time between ns-3 reports (in ms of sim time)", default = 1000)
    parser.add_argument("-x", "--dim_x", type = int, help = "x-dimension of networking area (in meters)", default = 300)
    parser.add_argument("-y", "--dim_y", type = int, help = "y-dimension of networking area (in meters)", default = 300)
    args = parser.parse_args()

    sim_runtime = args.sim_duration
    num_nodes = args.num_nodes   
    if args.sim_timestep_ms > (args.sim_duration * 1000):
        print("WARNING: sim timestep is longer than sim duration - defaulting to 1/2 of duration.");
        sim_timestep_ms = (args.sim_duration // 2) * 1000
    else: 
        sim_timestep_ms = args.sim_timestep_ms

    # total observed networking area 
    observed_area_x = args.dim_x 
    observed_area_y = args.dim_y 


    # maximum coverage of 1 ns-3 instance (total_ntwk_area = min_ntwk_dim**2)
    min_ntwk_dim = 300 # defined by 2 * maximum valid V2V communication range  
    num_inst, origin_coords_list = num_req_instances(observed_area_x, observed_area_y, min_ntwk_dim)


    # Determine number of instances required for parallel execution 
    processors = multiprocessing.cpu_count() - 1 # reserve one for control plane (i.e. comms threads/sim interface)

    # ### DEBUG ### Test config designed for VM environment - TODO: remove this before testing on kestrel 
    # processors = 2 

    # prevent hardware overflow - may need to update logic to interact with kestrel  
    if num_inst > processors: 
        print("Insufficient hardware resources! Number of requested instances greater than available CPU cores. Limiting scope...")
        num_inst = processors 
        origin_coords_list = origin_coords_list[:num_inst] # currently, just truncate simulation set 
        # sys.exit(0) # no need to exit, just limit reserved resources 
        # TODO: how do we want to handle this? Could make each simulation larger to cover obs area (at the cost of speed) 
    else: 
        pass 


    # internal netcfg
    main_comms_port = 50000     # dedicated comms port for "__main__"
    first_listener_port = 50001 # ports for listener sockets in this file (if this value changes, need to check loop error detection)

    # external netcfg
    helics_broker_port = 54321  # NOTE: needs to match broker! update when a common value chosen between simulations  
    first_sim_inst_port = 60001 # ports for interface sockets in simulation 


    # check netcfg against series of available ports (50000-59999, 60000-69999), and return available ports 
    main_comms_port, avail_listener_ports, avail_sim_ports = check_ports_open(main_comms_port, first_listener_port, first_sim_inst_port, num_inst)

    instance_args = [] # list of "command line" arg tuples for instance subprocess call 
    # current CLI vars needed in "instance_args" list: 
    # --inst_idx [uint32_t]        // integer index of ns-3 instance - will be used to maintain node association at control plane 
    # --io_port [uint32_t]         // port number assigned to ns-3 blocking/reporting socket  
    # --wrapper_port [uint32_t]    // port number assigned to wrapper thread dedicated to ns-3 instance
    # --origin_x [uint32_t]        // x-coordinate of interface origin, for accurate node geolocation relative to Aimsun
    # --origin_y [uint32_t]        // y-coordinate of interface origin, for accurate node geolocation relative to Aimsun 

    # array of listener objects 
    listener_object_list = []

    print("[MAIN] Starting <{}> listeners...".format(num_inst))

    # finally - bind to sockets, start listener threads, and declare instance parameters 
    for idx in range(num_inst):
        time.sleep(0.1) # avoid potential collisions in waf 
        # create listener server for this instance index 
        tmp_listener = instance_server(idx, avail_listener_ports[idx], avail_sim_ports[idx])

        # attach handle to list (reference, not copy)
        listener_object_list.append(tmp_listener) 

        # start instance server (includes socket and thread)
        tmp_listener.start_listener()

        # declare "command line" args for instance subprocess call 
        arg_tup = (idx, avail_sim_ports[idx], avail_listener_ports[idx], origin_coords_list[idx][0], origin_coords_list[idx][1]) # argument list declared as interim variable before appending for clarity 
        instance_args.append(arg_tup)

    print("[MAIN] Starting <{}> CPU processes...".format(num_inst))

    # print(instance_args)

    # Open and initialize CPU pool 
    # --> initializer will ignore Ctrl+C for daughter processes, since program may not stop otherwise - known bug in Python 2.X
    # --> still requires two instances of "KeyboardInterrupt" (i.e. Ctrl+C twice) to exit program (and kill child processes)
    cpu_pool = multiprocessing.Pool(processes = num_inst, initializer = signal.signal, initargs = (signal.SIGINT, signal.SIG_IGN)) 
    # cpu_pool = multiprocessing.Pool(processes = num_inst) 

    # Start ns-3 instances using CPU pool, then close pool once all instances are started 
    proc_outputs = cpu_pool.map_async(ns3_process, instance_args, callback=ns3_instances_complete)



######################################### START INIT LOOP #########################################

    # timeout flags for init loop 
    init_timeout = 0 
    max_init_timeout = 3000 # (sec) arbitrarily long time to wait until Aimsun sends init data (with steady state flag)

    startup_timeout = 0 
    max_startup_timeout = 300 # (sec) time to wait for simulations to compile and open sockets to receive init data 

    # flags to monitor progress and record init checkpoints 
    all_instances_waiting_for_init = False # set to 1 after instances finish compilation and are ready to receive initialization data 
    all_instances_ready = 0 # set to 1 after instances finished node deployment and network startup and are ready to advance co-simulation 


    # generate node IDs from user-defined number of nodes 
    aimsun_ids = [i for i in range(0,num_nodes)]

    # distribute nodes in quantized networking area, using uniform random distribution (with no repeats) 
    # --> NOTE: initial quantization assumes minimum 25 m^2 (5 m * 5 m) per vehicle 
    min_dim_per_veh = 5 # assuming square for each vehicle

    # (1) get indices of locations per vehicle (i.e. x/y indices for each 5m * 5m vehicle "cell")
    quant_x = observed_area_x // min_dim_per_veh # integer floor division, req. Python3 
    quant_y = observed_area_y // min_dim_per_veh

    # determined possible locs by (1) finding index of vehicle cell, (2) converting to meters, and (3) centering vehicle (roughly) 
    possible_locs = [[x * min_dim_per_veh + (min_dim_per_veh // 2), y * min_dim_per_veh + (min_dim_per_veh // 2)] for x in range(quant_x) for y in range(quant_y)]

    if num_nodes > len(possible_locs):
        print("WARNING: network area too small for requested number of nodes - nodes may overlap, performance will be affected!") 
        # sample with repeats
        aimsun_coords_int_list = random.choices(possible_locs, k = num_nodes)
    else: 
        # sample without repeats
        aimsun_coords_int_list = random.sample(possible_locs, k = num_nodes)

    aimsun_coords_string_list = []
    for coord in aimsun_coords_int_list:
        aimsun_coords_string_list.append(str(coord[0]) + ":" + str(coord[1]))


    # start to process received data before checking if ns-3 instances are ready to receive it (minor issue, but compilation may be slow)
    sort_index_to_instance = [[] for inst in range(num_inst)] # provides list of indices to map node info to instances w.r.t ssort_nodes[idx].append(new_node) 


    # associate data into target instances by node ID 
    for node_index in range(num_nodes): 
        # get location of each node 
        curr_node_coords_int_list = aimsun_coords_int_list[node_index] # expressed as [int_x,int_y]

        # check boundaries of each ns-3 instance 
        for inst_ctr in range(num_inst): 
            if (
                (curr_node_coords_int_list[0] >= origin_coords_list[inst_ctr][0]) and       
                (curr_node_coords_int_list[0] <= origin_coords_list[inst_ctr][0] + min_ntwk_dim) and 
                (curr_node_coords_int_list[1] >= origin_coords_list[inst_ctr][1]) and 
                (curr_node_coords_int_list[1] <= origin_coords_list[inst_ctr][1] + min_ntwk_dim)
                ): 
                sort_index_to_instance[inst_ctr].append(node_index)

            else: 
                # print(f'Node {aimsun_coords_int_list[node_index]} not located within instance {inst_ctr} boundaries [{origin_coords_list[inst_ctr][0]},{origin_coords_list[inst_ctr][1]}] * [{origin_coords_list[inst_ctr][0] + min_ntwk_dim},{origin_coords_list[inst_ctr][1] + min_ntwk_dim}]')
                # print(f'Node {aimsun_coords_int_list[node_index]} actually located at {curr_node_coords_int_list[0]}, {curr_node_coords_int_list[1]}')
                pass # node not located in this instance 

    # quick check before proceeding 
    for instance in range(num_inst):
        print("Nodes in instance {}: {}".format(instance, sort_index_to_instance[instance]))


    # NOTE: reaching this point means all INIT data has been processed, and we need to see if ns-3 is ready to receive it yet 


    # check that all instances are finished compiling and ready to receive INIT data before advancing 
    # --> cannot process init data until instance sockets are open and monitored 
    while all_instances_waiting_for_init == False: 
        # check startup timeout - should only take a few minutes 
        if startup_timeout >= max_startup_timeout: 
            print("ERROR: Simulation startup timeout reached (ns-3 took too long to compile). Exiting.")
            sys.exit(1)
            
        else: 
            # add "1" if any instances have not reported that they are expecting init params 
            waiting = [0 if listener.instance_waiting == 1 else 1 for listener in listener_object_list]
            if sum(waiting) == 0: 
                # all instances are waiting for init, so we can now engage with Aimsun 
                all_instances_waiting_for_init = True 
            else: 
                # nodes not ready for init_data yet - 
                time.sleep(1) 
                startup_timeout += 1


    # NOTE: reaching this point means all instances have finished compiling/setting up and are ready to receive initial state data 

    for inst_ctr in range(num_inst): 
        # push all init data to associated instances 
        listener_object_list[inst_ctr].curr_node_ids  = [aimsun_ids[idx] for idx in sort_index_to_instance[inst_ctr]] # initial nodes 
        listener_object_list[inst_ctr].curr_node_locs = [aimsun_coords_string_list[idx] for idx in sort_index_to_instance[inst_ctr]]  # initial node locs 
        listener_object_list[inst_ctr].curr_node_msgs = [] # no initial messages for ns-3 initialization - network not established yet 

        # generate string for each instance to send to corresponding ns-3 control plane 
        listener_object_list[inst_ctr].sim_runtime = str(sim_runtime)
        listener_object_list[inst_ctr].sim_timestep_ms = str(sim_timestep_ms) 
        listener_object_list[inst_ctr].update_ns3_state(init = True)


    # NOTE: reaching this point means all instances have received initial state data and are beginning network execution 
    # --> we need to let the network advance a little bit to ensure all nodes are connected and active before proceeding  
        



    # wait for all instances to report init finished and ready to advance - may take some time for network to be established, but needed to advance 
    ready = sum([0 if listener.instance_ready == 1 else 1 for listener in listener_object_list])
    startup_timeout = 0 # re-use timeout here 

    # need to make sure all network processes are running and all nodes are connected before advancing 
    while ready != 0: 
        if startup_timeout >= max_startup_timeout:
            print("ERROR: Network startup timeout reached (ns-3 took too long to report ready). Exiting.")
            sys.exit(1)
        
        ready = sum([0 if listener.instance_ready == 1 else 1 for listener in listener_object_list])
        time.sleep(1) 
        startup_timeout += 1


######################################### START MAIN LOOP #########################################

    # timeout flags for main loop 
    global_timeout = 0 
    max_global_timeout = sim_runtime * sim_timestep_ms + 10000 # iterations - moot if we have flag for when child processes finish 

    reporting_timeout = 0  
    max_reporting_timeout = num_nodes * 30 # (seconds) max time to wait for simulations to advance sim step and report data (scaled with simulation size)

    # used to store collected data - declared outside of loop for efficiency 
    agg_reported_data_dict = {}

    # flag variable - 0 if any ns3 instances still running, 1 if all are complete
    global ns3_completed
    ns3_completed = 0

    while (global_timeout < max_global_timeout) and not ns3_completed: 
        global_timeout += 1

        # NOTE: at this point we still have the list of nodes associated with each instance: sort_nodes_by_index[instance_index][node_index]

        for inst_ctr in range(num_inst): 
            
            # cross-reference with previous values in case we want to monitor node traversal externally 
            if len(listener_object_list[inst_ctr].curr_node_ids) != 0: 
                # print(" - Inst. {} incoming nodes: {}".format(inst_ctr, list(set(updated_nodes) - set(listener_object_list[inst_ctr].curr_node_ids))))
                # print(" - Inst. {} outgoing nodes: {}".format(inst_ctr, list(set(listener_object_list[inst_ctr].curr_node_ids) - set(updated_nodes))))
                pass
            else: # list is empty - leave as "pass" if we want to ignore initial placement s 
                pass 

            # no change, so just use the same values as earlier 
            listener_object_list[inst_ctr].curr_node_ids  = [aimsun_ids[idx] for idx in sort_index_to_instance[inst_ctr]] # initial nodes 
            listener_object_list[inst_ctr].curr_node_locs = [aimsun_coords_string_list[idx] for idx in sort_index_to_instance[inst_ctr]]  # initial node locs 
            # listener_object_list[inst_ctr].curr_node_msgs = [] # nodes will send CAM by default when queue is empty 
            listener_object_list[inst_ctr].curr_node_msgs = ["0" for idx in sort_index_to_instance[inst_ctr]] # nodes will send CAM by default when queue is empty 

            listener_object_list[inst_ctr].update_ns3_state(init = False) 

        # NOTE: at this point, all state data has been sent to ns-3 - now we wait for all instances to publish network updates 
        # --> we expect ns-3 to take some time to advance   
        time.sleep(2) 

        # start checking for data reported from ns-3 simulations 
        reporting = [0 if (len(listener.retrieve_data()) == 0) else 1 for listener in listener_object_list]
        # --> append 1 if listeners have collected data for this time step 

        while sum(reporting) == 0:
            # print(f"Waiting on {len(reporting)} instances to report data...")
            # check timeout 
            if reporting_timeout == max_reporting_timeout: 
                # data reporting timeout scales with number of nodes, so this should never be reached without critical error 
                print("ERROR: Data collection timeout reached (ns-3 took too long to report data). Exiting.") 
                sys.exit(1)
            else:  
                # check if all listeners have some data to report - append "1" if any listener data list is empty 
                reporting = [0 if (len(listener.retrieve_data()) == 0) else 1 for listener in listener_object_list]
                reporting_timeout += 1
                time.sleep(1) 
                if reporting_timeout % 10 == 0:
                    print(f"Waiting for {reporting_timeout} on {len(reporting)} instances to report data...")

        
        send_string = "FAKE_DATA" # placeholder to send to Aimsun - should be replaced by data dict which can be processed directly 

        print("Data reported from listeners: ")

        # collect data from all instances and dump into json object 

        for listener in listener_object_list: 
            # copy data from listener buffer 
            agg_reported_data_dict[str(listener.inst_idx)] = listener.retrieve_data()

            # clear listener buffer - we will enable any persistent monitoring in ns-3 directly 
            listener.flush_data()

        send_string = json.dumps(agg_reported_data_dict)

        print("ns-3 state: {}".format(send_string)) 

        # optional: sleep for a small period 
        time.sleep(0.5)  
        
#########################################  END MAIN LOOP  #########################################

    proc_outputs.wait() # see if child processes waiting to finish (i.e. ns-3 not done but Aimsun is)

    cpu_pool.terminate()
    cpu_pool.join() # safely exit multiprocessing 
