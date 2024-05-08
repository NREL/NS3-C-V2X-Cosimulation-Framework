"""
Temporary specialized federate to enable integration testing with Aimsun. 

This federate will produce strings in accordance with expected formatting from Aimsun. 

Last tested with Python 3.11.0. 
"""

import helics as h 
import json
import random 
import sys 
import time 



# make sure we have access to HELICS during runtime (debug) 
h.helicsGetVersion()

class aimsun_test_set():
    """
    Generate strings which we would expect from Aimsun V2X federate 

    Designed to interface with ns-3 wrapper suite through ZMQ-based HELICS broker 
    """
    def __init__(self):
        # ground truths 
        self.coord_bounds = 10   # x/y coordinate boundaries (assume small, square area for now)
        self.candidate_messages = [0,1,2,3,4] # keys for 4 packet types (1,2,3,4) + no transmission (0)
        self.assumed_runtime = 10 # s
        # self.assumed_timestep = 200 # ms
        self.assumed_timestep = 100 # ms

        # "current state" of Aimsun network 
        self.node_ids_by_idx = [0,1,2,3]
        self.node_coords_by_idx = [[1,1], [3,3], [5,5], [7,7]]
        self.node_msgs_by_idx = [0,0,0,0] 

    def generate_init_strings(self):
        """
        Generate strings for ns-3 simulation initialization
        
        Only 3 federates will send data, others will send escape character ("!") 
        --> ACK federate      : must contain "init:[sim_runtime]:[sim_timestep]" to tell wrapper how to sort data 
        --> Node ID federate  : same as normal update 
        --> Coords federate   : same as normal update, but locations should not change if repeated 
        > Velocity federate   : send "!" 
        > Message federate    : send "!"
        """     
        ack_string = "INIT:" + str(self.assumed_runtime) + ":" + str(self.assumed_timestep) 
        node_string = "" 
        loc_string = "" 
        msg_string = "!" 
        vel_string = "not_used" 

        for veh in range(len(self.node_ids_by_idx)): 
            node_string += str(self.node_ids_by_idx[veh]) + "," 
            loc_string += str(self.node_coords_by_idx[veh][0]) + ":" + str(self.node_coords_by_idx[veh][1]) + ","

        node_string += "!"
        loc_string += "!"

        return ack_string, node_string, loc_string, msg_string, vel_string


    def generate_dummy_strings(self, timestep):
        """
        TEST FUNCTION - make dummy strings following expected formatting from Aimsun to design instance I/O 

        --> assume 4 moving nodes, each may or may not send a random message in each timestep 
        """

        # NOTE: Fake Aimsun data for test suite to design instance I/O 
        # --> assume 4 moving nodes, each may or may not send a random message in each timestep 

        # reset passed strings each step 
        ack_string = ""
        node_string = ""
        loc_string = ""
        msg_string = "" 

        for node in range(len(self.node_ids_by_idx)): # modify in-place favorable since list comprehension makes new object 
            
            # update coords - every node just moves 1 step diagonally, separated by (2,2) and resetting to (0,0) once reached (coord_bounds, coord_bounds)
            if self.node_coords_by_idx[node][0] < self.coord_bounds: # only check X coord since X and Y are the same 
                self.node_coords_by_idx[node][0] += 1
                self.node_coords_by_idx[node][1] += 1
            else: 
                self.node_coords_by_idx[node][0] = 0
                self.node_coords_by_idx[node][1] = 0

            # update message list - random number of message keys in each timestep (key 0 == no message --> since we align vehicle ID with list index, field cannot be empty)
            for num_msgs_this_step in range(random.randint(1,3)): 
                msg_string += str(random.choice(self.candidate_messages)) # random choice from message set 

            # update strings 
            node_string += str(self.node_ids_by_idx[node]) + "," # will not change yet but left here for cohesion 
            loc_string += str(self.node_coords_by_idx[node][0]) + ":" + str(self.node_coords_by_idx[node][1]) + ","
            msg_string += "," # messages are single-digit keys, so we can separate groups of messages sent by a single node using ","

        node_string += "!"
        loc_string += "!"
        msg_string += "!"

        vel_string = "not_used"

        print("LOC STRING: {}".format(loc_string))

        return ack_string, node_string, loc_string, msg_string, vel_string




# line below has specifications only needed for multi-machine connection 
# helics_broker --broker_address=54.67.2.187 --localinterface=10.81.10.102 -f 1 --loglevel=trace -t zmq_ss  

fedinfo = h.helicsCreateFederateInfo() 

fedinit_str = '--federates=1 --broker_address=127.0.0.1' # use for local (zmq) testing - default 

fed_name = 'aimsunFederate' 

h.helicsFederateInfoSetCoreName(fedinfo, fed_name) 

h.helicsFederateInfoSetIntegerProperty(fedinfo, h.helics_property_int_log_level, 6) #use highest logging level 

h.helicsFederateInfoSetCoreTypeFromString(fedinfo, "zmq") # only used locally 

h.helicsFederateInfoSetCoreInitString(fedinfo, fedinit_str) 

h.helicsFederateInfoSetTimeProperty(fedinfo, h.helics_property_time_delta, 0.01) # make sure the time delta is less than the resolution of your co-simulation 

cfed = h.helicsCreateCombinationFederate(fed_name, fedinfo) # a combination federate is one that can send values or messages as opposed to only values print(f'federate created as {cfed}')    

pubAck = h.helicsFederateRegisterPublication(cfed, "ACK",h.helics_data_type_string, '')
pubIds = h.helicsFederateRegisterPublication(cfed, 'vehicleIds', h.helics_data_type_string, '') 
pubCrd = h.helicsFederateRegisterPublication(cfed, 'vehicleCoordinates', h.helics_data_type_string, '') 
pubMsg = h.helicsFederateRegisterPublication(cfed, 'vehicleMessages', h.helics_data_type_string, '') 
pubVel = h.helicsFederateRegisterPublication(cfed, 'vehicleSpeeds', h.helics_data_type_string, '') 

sub1 = h.helicsFederateRegisterSubscription(cfed, 'ns3/ns3pub', '') 


try:
    h.helicsFederateEnterExecutingMode(cfed) 
except KeyboardInterrupt:
    sys.exit(1)

print('test_fed connected and entering executing mode')    

i = 0
step = 1

aimsun_instance = aimsun_test_set()

initialized = 0


while i < 1000:   
    # update "Aimsun" state
    if initialized == 0: 
        print("Generating init string...")
        # send initialization data without update simulation state - repeat sending until wrapper ACKs init data 
        s_ack, s_ids, s_crd, s_msg, s_vel = aimsun_instance.generate_init_strings()
    else:
        print("Generating update string...")
        # normal dynamic state update 
        s_ack, s_ids, s_crd, s_msg, s_vel = aimsun_instance.generate_dummy_strings(i)

    h.helicsPublicationPublishString(pubAck, s_ack)   
    h.helicsPublicationPublishString(pubIds, s_ids)   
    h.helicsPublicationPublishString(pubCrd, s_crd)   
    h.helicsPublicationPublishString(pubMsg, s_msg)   
    h.helicsPublicationPublishString(pubVel, s_vel)   

    cfed.request_time(i)    
    i += step

    # subscribe to any subscriptions     
    ns3_update = h.helicsInputGetString(sub1)     

    # and do any processing that this federate needs to do    
    try: 
        json_data = json.loads(ns3_update)
        print('data recognized as json: ')   
        print(json_data)
    except:
        json_data = "did not receive json string"


    if ns3_update == "INIT_OKAY": # indicates to advance simulation to next timestep 
        initialized = 1
    elif ns3_update == "INIT_FAIL": # indicates to repeat init data
        break
    else: pass 

    cfed.request_time(i)      
    i += step

    time.sleep(1)  

# at the end close out your federate by disconnecting from the broker and freeing any memory used by the federate 
print('closing {}'.format(fed_name)) 
h.helicsFederateDisconnect(cfed) 
h.helicsFederateFree(cfed)  

## for testing can use helics_app echo  
# can connect with -t tcp_ss --ipv4 --port=26404 --broker_address=ebroker.hpc.nrel.gov