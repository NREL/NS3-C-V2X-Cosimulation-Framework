/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * 
 * Modified by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 */




#include "ns3/applications-module.h"
#include <ns3/buildings-helper.h>
#include <ns3/cni-urbanmicrocell-propagation-loss-model.h>
#include "ns3/config-store.h"
#include <ns3/constant-position-mobility-model.h>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/lte-helper.h"
#include "ns3/lte-hex-grid-enb-topology-helper.h"
#include "ns3/lte-module.h"
#include "ns3/lte-v2x-helper.h"
#include "ns3/mobility-module.h"
#include <ns3/multi-model-spectrum-channel.h>
#include "ns3/network-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tag.h"
#include <ns3/spectrum-analyzer-helper.h>

#include "helics-control-app.h"
#include "v2x-packet-tag.h"
#include "control-plane.h"


#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <cfloat>
#include <fstream>
#include <netdb.h>
#include <netinet/in.h>
#include <sstream>
#include <stdio.h> 
#include <stdlib.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <typeinfo>
#include <unistd.h>
#include <vector>
#include <chrono> 


using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("v2x_communication_mode_4");



// Global variables - declared for convenience 
uint32_t origin_x = 0;
uint32_t origin_y = 0;

// Baseline distance in meter (150m for urban, 320m for freeway) - range limit 
double baseline = 150.0;     

// communication ports for each node 
uint32_t application_port = 8000; // default packet destination
uint32_t measurement_port = 8001; // custom packet destination for on-demand packets 



Ipv4InterfaceContainer ueIpIface; // needed to manipulate interfaces outside of "main()", i.e. to "remove" nodes

// Map packet statistics to each node in this simulation 
// --> these will be reset when nodes are added from initialized object pool, but not when they are removed from this instance 
// --> statistics will be aggregated for entire fleet in Python wrapper 

uint32_t ctr_totRx = 0; 	  // Counter for total received packets
uint32_t ctr_valRx = 0;       // Counter for packets received within packet valid window 
uint32_t ctr_totTx = 0; 	  // Counter for total transmitted packets
uint32_t ctr_outOfRange = 0;  // Count distance-based packet failures 



std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > candidate_rx_nodes; 

std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > packet_monitor;

std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > receiver_drops_by_packet; // NOTE: refreshed every iteration 

// std::vector< std::tuple<uint32_t, uint32_t, uint32_t> > globally_dropped_packets; --> no need for details yet, we can just count first 

uint32_t globally_dropped_packets;

uint32_t individually_dropped_packets; 

// std::map<std::tuple<uint32_t, uint32_t, uint32_t>, std::string> late_packets; --> left for future work 

// std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::tuple<uint32_t, uint32_t> > rx_loc_by_dropped_packet; --> left for future work 



/*
Packet monitoring:  
- for each transmitted packet, need (1) list of nodes which correctly receive it, and (2) list of nodes which do not 
- reception needs to be configurable to N sim time steps, i.e. nodes can correctly receive a packet as long as it was sent within N sim time steps (initially 3)
---> this was not selected based on LTE-V2X latency requirements, but we can calculate this easily



Variables: 

// common_packet_identifier_key = std::tuple<TX_time_ms, TX_id, packet_type_key> 

- packet_monitor: record of all packets sent which are still valid with assoc receivers 
---> maps (TIME, TX_ID, KEY) tuple to vector of RX_IDs  
---> packets expire after N sim time steps (initially, N = 3) 
---> latency calculated as (rx_TIME - tx_TIME) 

- candidate_rx_nodes: record of all nodes within range of transmitter when packet was transmitted 
---> maps (TIME, TX_ID, KEY) tuple to vector of RX_IDs 
---> appends all possible receivers - used to check which packets were fully dropped vs received late, and record location of drops 
---> can be used to generate heatmap for receiver locations when packets are dropped (can do same for TX by including TX_LOC in map key tuple) 
---> NOTE: notes will remove themselves from this list when the packet is received - that way, anything left after timeout shows packet drop with location 

- receiver_drops_by_packet: record of all receivers which did not receive a packet before timeout 
---> maps (TIME, TX_ID, KEY) tuple to list of (RX_IDs, LOC) 

- globally_dropped_packets: count number of packets which were not received by ANY node (i.e. resource-based packet collision)

- individually_dropped_packets: count number of packets which were received by SOME nodes (i.e. channel-based packet loss)



*/



// Responders users 
NodeContainer ueVeh;

// Pile of UE vehicles to draw from for dynamic topology 
// --> created using known upper-bound number of vehicles in simulation (initial guess = 500)
NodeContainer inactiveUeVeh; // node list to pull from when instantiating new nodes or add to when deactivating nodes
// ** initialized first - all nodes here will be placed at [0,0], get V2X protocol stack installed/connected, and provided L2/L3 addresses  

NodeContainer activeUeVeh;   // node list of active (unavailable) nodes 
// ** nodes will be added to this container when TFT is installed/replaced and sidelink is activated 

NetDeviceContainer availableDevices; // list of all unused network devices 
NetDeviceContainer activeTxUes;      // list of all unavailable/active network devices 






// struct to hold pointers to objects needed to "add/remove" (enable/disable) nodes during runtime 
struct DynamicTopoManager
{
    std::vector<uint32_t>          node_ids; 
    std::vector<Vector>            loc_updates;
    Ptr<LteV2xHelper>              preconfig_lteV2xHelper; 
};





// might be better practice to define this class in a public header 
class PacketLib 
{ 
public: 
  // define all packet types that can be triggered by scenarios in Aimsun - we need Aimsun to select TX'd packets by type based on scenario 
  uint32_t lenCAM = 190;   
  uint32_t lenDENM = 800; 
  uint32_t lenBSM1 = 120;  
  uint32_t lenBSM2 = 320; 
  uint32_t lenBSM3 = 520;  
//uint32_t lenBSM4... 

  // packets initialized this way have zero-filled payload (uninitialized), but that's fine - we will use tags instead of payload 
  Ptr<Packet> packetCam  = Create<Packet>(lenCAM);   // pkt_key = "cam0", pkt_idx = 1 (pkt_key should always have len=4 for convenience)
  Ptr<Packet> packetDenm = Create<Packet>(lenDENM);  // pkt_key = "denm", pkt_idx = 2
  Ptr<Packet> packetBSM1 = Create<Packet>(lenBSM1);  // pkt_key = "bsm1", pkt_idx = 3
  Ptr<Packet> packetBSM2 = Create<Packet>(lenBSM2);  // pkt_key = "bsm2", pkt_idx = 4
  Ptr<Packet> packetBSM3 = Create<Packet>(lenBSM3);  // pkt_key = "bsm3", pkt_idx = 5
//Ptr<Packet> packetBSM4... -> make sure to add parsing to "helics-control-app" 

  std::map<uint32_t, Ptr<Packet> > const key_map = { 
    {1, packetCam}, 
    {2, packetDenm}, 
    {3, packetBSM1},
    {4, packetBSM2},
    {5, packetBSM3}
  };
};

const PacketLib packetLib;


// function declarations 
void PlaceNodes(NodeContainer *nodes, std::vector<uint32_t> node_ids, std::vector<Vector> loc_updates, uint32_t reset);
void AddNodes(DynamicTopoManager * topoMngr);
void RemoveNodes(DynamicTopoManager * topoMngr);
void stateUpdateReport (Ptr<HelicsCtrlPlane> controlPlane, uint32_t simTimeStep, DynamicTopoManager * topoMngr);



void
stateUpdateReport (Ptr<HelicsCtrlPlane> controlPlane, uint32_t simTimeStep, DynamicTopoManager * topoMngr) 
/**
 * Pause all processes in simulation (ideally let them finish) and aggregate state info from all nodes 
 * 
 * Individual nodes should not compete to access shared variables, so nodes will record their own metrics at the APP layer
 * 
 * Goals: 
 * --> aggregate all APP-layer data while simulation is "paused" (i.e. no events allowed to schedule until this function returns)
 * --> block simulation from executing until allowed to proceed (i.e. sync finished with HELICS)
 * 
 * APP-layer data will be aggregated here, then "blockingRuntimeSync()" will report info and block execution until allowed to proceed 
*/
{
    // std::cout << "\n * Network state update * " << std::endl;
    NS_LOG_INFO("\n * Network state update * ");
    
    // // DEBUG: this block ensures that node operations are paused during runtime I/O processes 
    // // --> context will return the tracked node ID which is currently executing a process 
    // // --> if there is no node assigned to the process, the "context" should return 0xFFFFFFFF
    //
    // uint32_t context = Simulator::GetContext(); 
    // if (context == 0xFFFFFFFF) {
    //     std::cout << "Reporting at context 0xFFFFFFFF" << std::endl;
    // } 
    // else {
    //     std::cout << "Reporting non-zero context - this function may not have priority! Events are not being scheduled correctly!" << std::endl;
    // }
    

    // // DEBUG: print all transmit queues - some nodes will probably still have packets in TX queue when state update triggers 
    // NodeContainer::Iterator temp_nIt;
    // for (temp_nIt = activeUeVeh.Begin(); temp_nIt != activeUeVeh.End(); temp_nIt++){
    //     std::deque<uint32_t> copy_queue = (*temp_nIt)->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_pktDeque;
    //     std::cout << "Node " << (*temp_nIt)->GetId() << " packet queue: ";
    //     for (auto checker:copy_queue) {
    //         std::cout << checker << ", ";
    //     }
    //     std::cout << std::endl;
    // }

    std::map<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > reception_events_this_timestep;


    if (controlPlane->m_sim_ready == 0) { 
        // for first synchronous step, we only report "ready" signal 
        controlPlane->m_sim_ready = 1; 
        controlPlane->SendReady(); 

        // remove any statistics collected before simulation period start 
        packet_monitor.clear();
        candidate_rx_nodes.clear();
        ctr_totRx = 0; 	   
        ctr_valRx = 0;        
        ctr_totTx = 0; 	    
        ctr_outOfRange = 0; 

        // flush message queues so nodes do not enter "valid" simtime with any queued packets 
        NetDeviceContainer::Iterator ndIt;
        for (ndIt = activeTxUes.Begin(); ndIt != activeTxUes.End(); ndIt++) {
            (*ndIt)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->FlushQueue();
        }
    } 
    else { // after first synchronous step, we will have real data to report from the current network state  

        // iterate through transmitted packets logged by packet_monitor 
        std::map<std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> >::iterator packet_log_iter; 
        std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> invalid_packets; 
        
        uint32_t validity_window = (3 * controlPlane->sim_time_step); // packets sent > (3 * simTimeStep) ago is invalid (TODO: match this to protocol)
        uint32_t send_time;

        for (packet_log_iter = packet_monitor.begin(); packet_log_iter != packet_monitor.end(); packet_log_iter++) {
            // entry format: TIME, TX_ID, KEY (i.e. get<0> = time, get<1> = TX_ID, get<2> = KEY)
            send_time = std::get<0>(packet_log_iter->first); 

            if (send_time < (Simulator::Now().GetMilliSeconds() - validity_window)) { // strictly "less than" (i.e. before) so we can try to hit 3 iterations exactly (we will probably need to adjust this anyways)
                // packet was sent too long ago - data may no longer reflect current simulation state 
                // --> record packet to purge from registry after loop (erase-remove idiom does NOT work with maps)
                invalid_packets.push_back(packet_log_iter->first);
                
                // std::cout << ">> Removing packet from registry (window: " << send_time << " to " << send_time + validity_window << " ms, currently " << Simulator::Now().GetMilliSeconds() << " ms)" << std::endl;

                // record any remaining candidate receivers as drops 
                receiver_drops_by_packet[packet_log_iter->first] = candidate_rx_nodes[packet_log_iter->first];
                candidate_rx_nodes.erase(packet_log_iter->first); // erase map entry 

                if (packet_log_iter->second.size() == 0) {
                    // zero nodes received this packet 
                    globally_dropped_packets++; 
                }
                else {
                    // some nodes received this packet 
                    individually_dropped_packets++;
                }
            } 
            else { 
                // packet is still valid - report which (if any) nodes have received this so far (with latency)
                std::vector<uint32_t> receiver_vector = packet_log_iter->second;
            } 
        } 

        // remove invalid packets from registries   
        for (auto packet_keys:invalid_packets) {
            packet_monitor.erase(packet_keys);
        }

        invalid_packets.clear();
        // remove any entries in candidate node vector with 0 nodes left 
        for (packet_log_iter = candidate_rx_nodes.begin(); packet_log_iter != candidate_rx_nodes.end(); packet_log_iter++) {
            if (packet_log_iter->second.size() == 0) {
                invalid_packets.push_back(packet_log_iter->first);
            }
        }
        // remove invalid packets from registries   
        for (auto packet_keys:invalid_packets) {
            candidate_rx_nodes.erase(packet_keys);
            // std::cout << "show remove invalid dropped packet" << std::endl;
            NS_LOG_INFO("show remove invalid dropped packet");
        }
        invalid_packets.clear();

        // report data to wrapper  
        // --> packet reception data = packet_monitor
        // --> packet drop data = receiver_drops_by_packet
        // controlPlane->ReportData(packet_monitor, receiver_drops_by_packet); --> not reporting packet drops to Aimsun 
        if (controlPlane->m_data_reporting_fidelity == 1) {
            controlPlane->ReportAbbrevData(ctr_totTx, ctr_valRx);
        }
        else {
            controlPlane->ReportData(packet_monitor); 
        }


        receiver_drops_by_packet.clear();

        // // after reporting, remove all nodes from received nodes vector for each packet 
        // --> can be done at control plane, to minimize number of loop operations, but harder to debug 
        for (packet_log_iter = packet_monitor.begin(); packet_log_iter != packet_monitor.end(); packet_log_iter++) {
            packet_log_iter->second.clear();
        }

    } 

    // wait for traffic state update before advancing simulation time 
    UpdateData new_data = controlPlane->WaitForData();

    std::vector<uint32_t>::iterator idIt; // iterator for list of Aimsun IDs in new_data 
    std::map<uint32_t, uint32_t>::iterator mapIt; // iterator for Control Plane map 
    std::vector<uint32_t>::iterator vecIt; // iterator for list of ns-3 IDs in currTopo 

    // record which node IDs to add/remove from the simulation 
    std::vector<uint32_t> temp_entering_node_aimIds;
    std::vector<uint32_t> temp_leaving_node_aimIds;

    // empty curr_topo vectors and reserve new sizes (keeping pointer to preconfig V2X helper)
    topoMngr->node_ids.clear();
    topoMngr->loc_updates.clear();


    // topoMngr->node_ids.reserve(new_data.node_ids.size()); // pre-allocate expecting updates for all nodes - no problem if we reserve too much 
    // topoMngr->loc_updates.reserve(new_data.node_ids.size());

    NS_LOG_INFO("Node IDs from Aimsun update: ");
    // std::cout << "Node IDs from Aimsun update: ";
    for (auto i:new_data.node_ids) {
        NS_LOG_INFO(i << ", ");
        // std::cout << i << ", ";
    }
    // std::cout << std::endl;

    NS_LOG_INFO("Current map: ");
    // std::cout << "Current map: ";
    for (auto i:controlPlane->id_map) {
        NS_LOG_INFO(i.first << " = " << i.second << ", ");
        // std::cout << i.first << " = " << i.second << ", ";
    }
    // std::cout << std::endl;

    // 1. Iterate through aimsun node IDs from passed data 

    for (idIt = new_data.node_ids.begin(); idIt != new_data.node_ids.end(); idIt++) {
            
        // 2. Check if this Aimsun ID is registered in current topology (via control plane map)
        // --> if YES: do nothing
        // --> if NO:  need to add it 

        mapIt = controlPlane->id_map.find((*idIt)); 

        if (mapIt == controlPlane->id_map.end()) {
            // std::cout << "Node " << (*idIt) << " not found - adding..." << std::endl;
            NS_LOG_INFO("Node " << *idIt <<  " not found - adding...");
            // this aimsun ID has not been mapped yet - must be added - record all new nodes to add as batch 
            temp_entering_node_aimIds.push_back((*idIt));
        }

        else {
            // we have found the key in the map - update is provided for existing node - record ID directly 
            topoMngr->node_ids.push_back(controlPlane->id_map.at(*idIt)); 
            topoMngr->loc_updates.push_back(new_data.loc_updates[idIt - new_data.node_ids.begin()]); // use iterator to get matched index of location update 
        }
    }

    // 3. Iterate through current topology nodes to see if we update or remove them 

    for (mapIt = controlPlane->id_map.begin(); mapIt != controlPlane->id_map.end(); mapIt++) {
        // 4. Check if current topology contains aimsun IDs which are not in Aimsun update set
        // --> if YES: need to remove it 
        // --> if NO:  do nothing 
        vecIt = std::find(new_data.node_ids.begin(), new_data.node_ids.end(), mapIt->first); 

        if (vecIt == new_data.node_ids.end()) {
            // std::cout << "Node " << (*mapIt).second << " not updated - removing..." << std::endl;
            NS_LOG_INFO("Node " << (*mapIt).second << " not updated - removing...");
            // this aimsun ID is no longer in simualtion - need to remove 
            temp_leaving_node_aimIds.push_back(mapIt->first); // record Aimsun ID since it is easiest to remove map elements by key 
        }
        else {
            // we found the existing aimsun ID in the update list - node is still in simulation - do nothing for now 
        }
    }

    // Update location of nodes we know are active in both ns-3 (activeUeVeh) and aimsun (new_data.node_ids)

    // NOTE this will try to place nodes that are about to be removed - does not cause any problems, just may be unnecessary  
    // --> implemented fix: iterate through topoMngr->node_ids again and cross-reference with temp_leaving_node_aimIds 
    
    PlaceNodes(&activeUeVeh, topoMngr->node_ids, topoMngr->loc_updates, 0); 


    // empty curr_topo vectors and reserve new sizes (keeping pointer to preconfig V2X helper)
    topoMngr->node_ids.clear();
    topoMngr->loc_updates.clear();

    // 5. Remove any exiting nodes first to free up resources 
    if (temp_leaving_node_aimIds.size() != 0) {

        topoMngr->node_ids.reserve(temp_leaving_node_aimIds.size());
        topoMngr->loc_updates.reserve(0); // we reset lcoations - only need to satisfy function call 

        for (vecIt = temp_leaving_node_aimIds.begin(); vecIt != temp_leaving_node_aimIds.end(); vecIt++) {

            // 5.1 Add ns-3 IDs of exiting nodes to available ID list (get ns-3 ID from map using Aimsun ID)
            controlPlane->available_ns3_ids.push_back(controlPlane->id_map[(*vecIt)]); 

            // 5.2 Translate node IDs from aimsun to ns-3 reference 
            topoMngr->node_ids.push_back(controlPlane->id_map[(*vecIt)]); 

            // 5.3 Remove map entries by key 
            controlPlane->id_map.erase(*vecIt);
        }

        // 5.4. Call remove nodes on exiting ns-3 IDs 
        RemoveNodes(topoMngr); // no need to change whatever is in memory here- we are resetting all to (0,0)

        // empty curr_topo vectors and reserve new sizes (keeping pointer to preconfig V2X helper)
        topoMngr->node_ids.clear();
        topoMngr->loc_updates.clear();
    }
    else {
        // no nodes to remove 
    }




    // 6. Add any nodes not yet in simulation 

    if (temp_entering_node_aimIds.size() != 0) {

        topoMngr->node_ids.reserve(temp_entering_node_aimIds.size());
        topoMngr->loc_updates.reserve(temp_entering_node_aimIds.size());

        for (vecIt = temp_entering_node_aimIds.begin(); vecIt != temp_entering_node_aimIds.end(); vecIt++) {

            // std::cout << "Check A: adding aimsun ID " << (*vecIt) << " as key to map with value " << controlPlane->available_ns3_ids.front() << std::endl;

            // 6.1 Update control plane map to map incoming aimsun ID to unused ns-3 ID 
            controlPlane->id_map[(*vecIt)] = controlPlane->available_ns3_ids.front();

            // 6.2 Push_back currTopoManager.node_ids with ns-3 ID  
            topoMngr->node_ids.push_back(controlPlane->available_ns3_ids.front()); 

            // 6.3 Pop ns-3 ID from "avaiable" IDs 
            controlPlane->available_ns3_ids.pop_front();
        }

        // 6.4 Update location of new nodes 
        topoMngr->loc_updates = new_data.loc_updates;

        NS_LOG_INFO("Adding nodes: " );

        // std::cout << "Adding nodes: ";
        for (auto i:topoMngr->node_ids){
            NS_LOG_INFO(i << ", ");
            // std::cout << i << ", ";
        }
        // std::cout << std::endl;

        // 6.5 Call AddNodes on currTopoManager (AddNodes will already ignore exiting "active" nodes) 
        AddNodes(topoMngr);

        topoMngr->node_ids.clear();
        topoMngr->loc_updates.clear();
    }
    else {
        // no nodes to add 
    }

    // std::cout << "Aimsun update ID stream: ";
    NS_LOG_INFO("Aimsun update ID stream: ");
    for (auto i:new_data.node_ids){
        // std::cout << i << ", ";
        NS_LOG_INFO(i << ", ");
    }
    // std::cout << std::endl;

    // std::cout << "Updated ns-3 topology: ";
    NS_LOG_INFO("Updated ns-3 topology: ");
    for (auto i:controlPlane->id_map){
        NS_LOG_INFO(i.first << " = " << i.second << ", ");
        // std::cout << i.first << " = " << i.second << ", ";
    }
    // std::cout << std::endl;


    // Queue up messages for each node 
    NodeContainer::Iterator nIt;
    for (nIt = activeUeVeh.Begin(); nIt != activeUeVeh.End(); nIt++){

        // need to nest loops since maps cannot search key from value - can replace with lambda function if needed 
        for (vecIt = new_data.node_ids.begin(); vecIt != new_data.node_ids.end(); vecIt++){
            // if this node matches this index
            if ((*nIt)->GetId() == controlPlane->id_map.at(*vecIt)) {
                // vecIt points to index of Aimsun ID associated with ns-3 ID 
                (*nIt)->GetApplication(0)->GetObject<HelicsCtrlApp>()->QueuePacketSet(new_data.new_msgs[vecIt - new_data.node_ids.begin()].c_str());
            }
        }
    }

    // NOTE: can use "::ScheduleNow()" if needed to schedule an event whenever the "current" event is finished, before simtime is iterated for next event 
    // --> Simulator::ScheduleNow(&foo); // i.e. this should enter an infinite loop which permanently stops simtime from advancing 
    // --> Ideally - Simulator::ScheduleNow(&blockingRuntimeSync); - to execute sync events in order (without simtime advancing in background)


    // print status this step 

    // make sure we don't receive more packets than we transmit 
    if (ctr_totRx > ctr_totTx) {
        // std::cout << "*** Logical error - RX packets > TX packets!" << std::endl; 
        NS_LOG_INFO("*** Logical error - RX packets > TX packets!");
        ctr_totRx = ctr_totTx; 
    } 
    else if (ctr_valRx > ctr_totRx) {
        // std::cout << "*** Logical error - valid RX packets > total RX packets!" << std::endl; 
        NS_LOG_INFO("*** Logical error - valid RX packets > total RX packets!");
        ctr_valRx = ctr_totRx; 
    }


    float prr = (float)ctr_valRx / (float) ctr_totTx; // only count valid packets 
    uint32_t late_pkts = ctr_totRx - ctr_valRx; // count packets received after expiration 
    // uint32_t dropped_pkts = ctr_totTx - ctr_totRx; --> cannot calculate like this, since validity timeout spans multiple time steps 

    // log current PRR in each timestep  
    // printf("\n[%.3f s] PRR: %.2f - %d sent / %d OK / %d late / %d out-of-range \n\n", Simulator::Now().GetSeconds(), prr, ctr_totTx, ctr_valRx, late_pkts, ctr_outOfRange);
    NS_LOG_INFO("[" << Simulator::Now().GetSeconds() << "] PRR: " << prr << " - " <<  ctr_totTx << " sent / " << ctr_valRx << " OK / " << late_pkts << " late / " << ctr_outOfRange << " out-of-range.");

    // schedule next update/reporting  
    Simulator::Schedule(MilliSeconds(simTimeStep), &stateUpdateReport, controlPlane, simTimeStep, topoMngr); // call self to execute after sync period 
    // std::cout << "Simulation advancing...\n" << std::endl;
}







void
SidelinkV2xAnnouncementMacTrace(Ptr<Socket> socket)
{    
    Ptr <Node> node = socket->GetNode(); 

    uint32_t packet_key = node->GetApplication(0)->GetObject<HelicsCtrlApp>()->GetQueuedPacket();

    if (packet_key == 0) {
        // not allowed to send packets at this node, just exit 
        return;
    }

    Ptr<HelicsCtrlApp> node_app = node->GetApplication(0)->GetObject<HelicsCtrlApp>(); 
    
    Ptr<Packet> curr_pkt = packetLib.key_map.at(packet_key)->Copy(); 

    // std::cout << "[" << simTime/1000000.  << "] " << "MAC sidelink announcement trace called at node: " << id << std::endl;

    uint32_t id = node->GetId();
    Vector posTx = node->GetObject<MobilityModel>()->GetPosition();

    Time send_time_now = Simulator::Now();

    // add relevant tags to packet and send 
    v2xTag newTag;
    newTag.SetSenderId(id);
    newTag.SetSenderLoc(posTx);
    newTag.SetSendTime(send_time_now);
    newTag.SetPacketType(packet_key);
    curr_pkt->AddPacketTag(newTag);

    // std::cout << "Adding entry to packet monitor registries: " << newTag.GetSendTime().GetMilliSeconds() << ", " << newTag.GetSenderId() << ", " << newTag.GetPacketType() << std::endl;
    NS_LOG_INFO("Adding entry to packet monitor registries: " << newTag.GetSendTime().GetMilliSeconds() << ", " << newTag.GetSenderId() << ", " << newTag.GetPacketType());

    auto map_entry_key = std::make_tuple(newTag.GetSendTime().GetMilliSeconds(), newTag.GetSenderId(), newTag.GetPacketType());

    // aggregate packet info to send to wrapper when needed 
    packet_monitor[map_entry_key]; // create entry with default constructor at value (i.e. map entry associates this key with an empty vector)

    // keep track of IDs of all nodes within range of this transmitter - all should receive packet 
    std::vector<uint32_t> receivers_in_range; 

    // std::cout << "Transmitter " << node->GetId() << " tagged loc: " << newTag.GetSenderLoc().x << "," << newTag.GetSenderLoc().y << std::endl;

    // send tagged packet 
    socket->SendTo(curr_pkt, 0, InetSocketAddress(node_app->m_bcastAddr, node_app->m_port));

    // check for each UE distance to transmitter
    for (uint8_t i=0; i<activeUeVeh.GetN();i++) {
        Ptr<MobilityModel> mob = activeUeVeh.Get(i)->GetObject<MobilityModel>(); 
        Vector posRx = mob->GetPosition();
        
        // add 1 to "sent" packet list for each vehicle that SHOULD receive this packet 
        double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.y - posRx.y),2.0));

        if  (distance > 0 && distance <= baseline) { // if self, distance == 0
            // tick up expected number of packet reception events (1 for each node within broadcast range)
            ctr_totTx++;

            // add ID of all nodes within range to candidate RX vector 
            receivers_in_range.push_back(activeUeVeh.Get(i)->GetId());
        }
    }
    // create entry for this packet and associate with all possible receivers (do this at the end to avoid dubious edit-in-place operations on the map) 
    candidate_rx_nodes[map_entry_key] = receivers_in_range; 
}





static void
ReceivePacket(Ptr<Socket> socket)
{   
    // get receiver node from socket info (socket is expected to be on receiver?)
    Ptr<Node> node = socket->GetNode();
    // int id = node->GetId();

    Ptr<MobilityModel> posMobility = node->GetObject<MobilityModel>();
    Vector posRx = posMobility->GetPosition();
    // Vector posTx; 

    // asynchronous, non-blocking socket receive function 
    // --> callback is called when there is data to be read, so this will return that data 
    Ptr<Packet> packet = socket->Recv (); 

    v2xTag tagCopy;

    bool foundTag = packet->RemovePacketTag(tagCopy);

    if (foundTag) {
        // std::cout << "Packet tag received successfully from transmitter " << tagCopy.GetSenderId() << std::endl;
        // posTx = tagCopy.GetSenderLoc();
        // printf("Recovered location from packet tag: %f, %f", tagCopy.GetSenderLoc().x, tagCopy.GetSenderLoc().y);
    }
    else {
        // std::cout << "Packet corrupt at node " << node->GetId() << " - dropping..." << std::endl;
        NS_LOG_INFO("Packet corrupt at node " << node->GetId() << " - dropping...");
    }
    
    // bool found = packet->PeekPacketTag(tagCopy); // just returns true if it exists - not needed
    // packet->PrintPacketTags(std::cout); // prints ALL packet tags, also not needed (or wanted)
    // bool found = packet->RemovePacketTag(tagCopy); 
    // if (found) {
    //     std::cout << tagCopy.GetSenderId() << std::endl; // works, but immense waste of console space 
    // }

    // NetDeviceContainer::Iterator ncIt;

    // double distance = sqrt(pow((posTx.x - posRx.x),2.0)+pow((posTx.x - posRx.y),2.0));
    double distance = sqrt(pow((tagCopy.GetSenderLoc().x - posRx.x),2.0) + pow((tagCopy.GetSenderLoc().x - posRx.y),2.0));

    if (distance > 0 && distance <= baseline) {         
        // uint32_t id = node->GetId();
        // uint32_t simTime = Simulator::Now().GetMilliSeconds();

        // COMPOUND MESSAGE (1/5)
        // std::cout << "Accessing entry: " << tagCopy.GetSendTime().GetMilliSeconds() << ", " << tagCopy.GetSenderId() << ", " << tagCopy.GetPacketType();
        NS_LOG_INFO("Accessing entry: " << tagCopy.GetSendTime().GetMilliSeconds() << ", " << tagCopy.GetSenderId() << ", " << tagCopy.GetPacketType());

        // record that this node received the packet
        // --> packets may still be "in the air" when reporting data 
        // try {
        //     std::string temp = packet_monitor.at(std::make_tuple(tagCopy.GetSendTime().GetMilliSeconds(), tagCopy.GetSenderId(), tagCopy.GetPacketType()));
        //     temp.append(std::to_string(node->GetId()));
        //     temp.append(",");
        //     packet_monitor.at(std::make_tuple(tagCopy.GetSendTime().GetMilliSeconds(), tagCopy.GetSenderId(), tagCopy.GetPacketType())) = temp;
        //     std::cout << "error cleared" << std::endl;
        // }
        // catch (std::out_of_range& const e) {
        //     std::cout << "Received a packet sent prior to data reporting - reception not counted." << std::endl;
        // }

        // generate common key for packet registry maps 
        auto registry_key = std::make_tuple(tagCopy.GetSendTime().GetMilliSeconds(), tagCopy.GetSenderId(), tagCopy.GetPacketType());

        // check list of valid packets for packet we just received 
        auto find_packet_entry = packet_monitor.find(registry_key);
        

        // TODO: maybe we can replace some of these "find...if..." sequences with "try...catch..." for efficiency? 

        if (find_packet_entry != packet_monitor.end()) {
            // add node ID to vector assoc with this key - "at" throws exception when key isn't found (guarantee we are accessing an existing packet entry and not creating a new one)
            packet_monitor.at(registry_key).push_back(node->GetId());

            // now that packet is received correctly, remove self from candidate receivers list 
            // --> first find entry for this packet in candidate nodes map 
            auto find_candidates_entry = candidate_rx_nodes.find(registry_key); 

            if (find_candidates_entry != candidate_rx_nodes.end()) {

                // copy candidate nodes into new vector for clarity 
                std::vector<uint32_t> copy_candidate_nodes = find_candidates_entry->second;
                
                // then find own ID in vector associated with key from this packet 
                std::vector<uint32_t>::iterator find_self_in_candidates = std::find(copy_candidate_nodes.begin(), copy_candidate_nodes.end(), node->GetId());

                if (find_self_in_candidates != copy_candidate_nodes.end()) {
                    // erase own ID from vector of candidate receivers associated with this node 
                    copy_candidate_nodes.erase(find_self_in_candidates);

                    // COMPOUND MESSAGE (2/5)
                    // std::cout << " -- ok." << std::endl;
                    NS_LOG_INFO(" -- ok.");

                    // update received packet count here - we do not want to update count for expired packets or duplicate receptions 
                    ctr_totRx++; 
                    ctr_valRx++;

                }
                else {
                    // packet received and valid but "self" not found in candidates  
                    // --> packet received twice? - not sure how this would happen, but can ignore for now since first packet would have already been received 
                    // --> node moved into reception range before packet timeout? - possible, and no action needed since packet was already received (i.e. no drops)
                    
                    // COMPOUND MESSAGE (3/5) - rest left active to show error states 
                    // std::cout << " -- pkt received, valid, but SELF not found." << std::endl;
                    NS_LOG_INFO(" -- pkt received, valid, but SELF not found.");
                    ctr_totRx++;    
                }
            }
            else {
                // packet received but assoc key not found in candidates map 
                // --> much more unlikely, but still could be caused by duplicate transmission of a packet (with same tags) - safe to ignore for now 
                
                // COMPOUND MESSAGE (4/5)
                // std::cout << " -- pkt received, valid, but ASSOC KEY not found." << std::endl;
                NS_LOG_INFO(" -- pkt received, valid, but ASSOC KEY not found.");
                ctr_totRx++; 
            }

        }
        else {
            // map access failed because packet validity expired before reception event 
            // COMPOUND MESSAGE (5/5)
            // std::cout << " -- pkt failed (packet received " << ctr_totRx << "ms after data timeout). "  << std::endl;
            NS_LOG_INFO(" -- pkt failed (packet received " << ctr_totRx << "ms after data timeout). ");
            ctr_totRx++; 
            // TODO: add more robust handling for this if needed 
        }
    }
    else {

        // double-check, since distance == 0 is trivial but possible 
        if (distance > baseline) {
            ctr_outOfRange++; 
        }
        else {
            // this should never be reached (ideally) - only should trigger when distance <= 0 
            // --> this condition is also trivial, since packets could be invalid anyways 
            // --> this was observed when a node received a packet at the transmitting node's location at time of transmit (well after packet timeout) and calculated 0 distance 
            // printf("Invalid TX/RX distance detected: TX %d = %.0f,%.0f - RX %d = %.0f,%.0f, dist = %.0f\n", tagCopy.GetSenderId(), tagCopy.GetSenderLoc().x, tagCopy.GetSenderLoc().y, node->GetId(), posRx.x, posRx.y, distance);
        }
    }
    // uint32_t latency = Simulator::Now().GetMicroSeconds() - transmit_time;
}



void 
PlaceNodes(NodeContainer *nodes, std::vector<uint32_t> node_ids, std::vector<Vector> loc_updates, uint32_t reset)
{
    // Install constant random positions 
    MobilityHelper mobVeh;
    mobVeh.SetMobilityModel("ns3::ConstantPositionMobilityModel"); 
    Ptr<ListPositionAllocator> staticVeh[nodes->GetN()];

    // Loop through number of nodes (vehicles) in nodeContainer "ueVeh"
    for (uint32_t i = 0; i < nodes->GetN(); i++) {
        staticVeh[i] = CreateObject<ListPositionAllocator>();
        uint32_t x = 0;
        uint32_t y = 0;
        float z = 1.5; // constant placement height for all nodes 

        // get ID of node "i" 
        uint32_t id_this_node = nodes->Get(i)->GetId();
        
        // get index of ID in ID vector 
        std::vector<uint32_t>::iterator posCoords = std::find(node_ids.begin(), node_ids.end(), id_this_node);

        if (reset == 0){
            // check if each node is in "node_ids" vector: set [x,y] to user-defined values if so, set [x,y] to random if not 
            if (posCoords != node_ids.end() ) {
                // // print node ID for debug 
                // std::cout << "Installing user-defined location for ns3 node: " << key << std::endl;

                // loc_updates is parallel to node_ids --> index of node == index of node coordinates 
                x = loc_updates[posCoords - node_ids.begin()].x;
                y = loc_updates[posCoords - node_ids.begin()].y;

            }
            else {
                // no user-defined location for this node, indicate random placement in case this was in error 
                // std::cout << "[PlaceNodes] No location provided - installing random location for node " << id_this_node << std::endl; 
                Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable> ();
                x = rand->GetValue (0,100); 
                y = rand->GetValue (0,100); 
            }
        }
        else {
            // place at [0,0] 
            x = y = 0;
        }

        staticVeh[i]->Add(Vector(x + origin_x,y + origin_y,z)); 
        mobVeh.SetPositionAllocator(staticVeh[i]);
        mobVeh.Install(nodes->Get(i));

        // // print placement of each node - verify placement by accessing current node mobility model directly 
        Ptr<MobilityModel> posMobility = nodes->Get(i)->GetObject<MobilityModel>();
        Vector posCurVeh = posMobility->GetPosition();

        // only print new locations of active nodes - we don't need to see reset loc 
        if (reset == 0){
            // std::cout << "Node " << nodes->Get(i)->GetId() << " placed at " << posCurVeh.x << "," << posCurVeh.y << std::endl;
            NS_LOG_INFO("Node " << nodes->Get(i)->GetId() << " placed at " << posCurVeh.x << "," << posCurVeh.y);
        }
    }
}



void   
AddNodes(DynamicTopoManager * topoManager)
/**
 * Used to add nodes to the simulation during runtime, based on context received from co-simulation 
 * + Since "removing" nodes is impractical, we will "add" disabled nodes first before calling node object constructor 
 *   - We will need to reinstall sidelink config and location anyways, so this does not increase complexity 
 *  
 * 
 * KEY ASSUMPTION: 1 UE NET DEVICE = 1 NODE -> logically should never have >1 UE netDevice/OBU per vehicle anyways 
*/
{

// 1. make sure "added" nodes are not already active - if so, remove from incoming node vector 
    NetDeviceContainer::Iterator ndIt;

    // copy vector of intended indices for "added" nodes 
    std::vector<uint32_t> added_node_ids(topoManager->node_ids);

    // no need to do anything here if we don't have new nodes to add 
    if (added_node_ids.size() == 0) {
        // std::cout << "No nodes to add - returning" << std::endl;
        NS_LOG_INFO("No nodes to add - returning");
        return; 
    }

    // print number of nodes to add 
    // std::cout << "Adding " << added_node_ids.size() << " nodes..." << std::endl; 
    NS_LOG_INFO("Adding " << added_node_ids.size() << " nodes...");

    // std::cout << "Seg fault check - num active TX UEs: " << activeTxUes.GetN() << std::endl;

    // loop through "active" net devices 
    for (ndIt = activeTxUes.Begin(); ndIt != activeTxUes.End(); ndIt++) { 
        // get node ID of each "active" net device 
        uint32_t active_node_id = (*ndIt)->GetNode()->GetId(); 

        // std::cout << "1a. Checking active node " << active_node_id << " (total pool: " << activeTxUes.GetN() << ")" << std::endl; 

        // get index of "active" node ID if contained within vector of incoming node IDs
        auto check = std::find(added_node_ids.begin(), added_node_ids.end(), active_node_id); 

        // if the index is ".end()", move on -- if not, then there is a conflict and we will not add the "active" node 
        if (check != added_node_ids.end()) {
            // active node ID FOUND in list of incoming nodes 

            // std::cout << "1b. Node " << *check << " is already active. Skipping activation." << std::endl;

            // --> remove from copy of vector using "remove/erase" idiom 
            added_node_ids.erase(std::remove(added_node_ids.begin(), added_node_ids.end(), *check), added_node_ids.end()); // TODO: double-check logic here - not familiar with idiom 
        }
        else { 
            // active node ID NOT FOUND in list of incoming nodes 
            // --> do nothing for now (can probably optimize loops later)
        }
    }
    // get number of nodes we are about to add 
    uint32_t num_new_vehicles = added_node_ids.size();

    // std::cout << "Seg fault check - num non-redundant vehs: " << num_new_vehicles << std::endl;

    // std::cout << "1c. Double-checking node list - adding  " << num_new_vehicles << " nodes after duplicate removal." << std::endl;

// 2. make sure we have N net devices in net device pile - if not, just return with error message 
    if (availableDevices.GetN() < num_new_vehicles) {
        // std::cout << "Insufficient available devices - selected nodes cannot be added until resources become available. " << std::endl;
        NS_LOG_DEBUG("Insufficient available devices - selected nodes cannot be added until resources become available. ");
        return; 
    }
    else {
        // pass (or add all following code here)
    }



// 3. get first N net devices from net device pile, remove them from net device pile, and add them to new container 
    // --> need a loop to do this since NetDeviceContainer is just vector class without vector methods 
    NetDeviceContainer new_broadcast_devices; 
    NodeContainer new_vehicle_nodes;

    // std::cout << "Seg fault check - num prealloc nodes: " << availableDevices.GetN() << std::endl;


    for (uint32_t ctr = 0; ctr < num_new_vehicles; ctr++) {
        // add current net device to new container 
        new_broadcast_devices.Add(availableDevices.Get(0));
        new_vehicle_nodes.Add(availableDevices.Get(0)->GetNode());

        // remove current net device from list of available devices 
        availableDevices = topoManager->preconfig_lteV2xHelper->RemoveNetDevice(availableDevices, availableDevices.Get(0));

        // set device app layer to "running"
        // availableDevices.Get(0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_running = true;

        // std::cout << "2a. Moved device from available list [" << availableDevices.GetN() << "] to broadcast list [" << new_broadcast_devices.GetN() << "]" << std::endl; 
    }



    // sanity check 
    if (new_broadcast_devices.GetN() == num_new_vehicles) {
        // std::cout << "Sanity check OK - all requested devices available [AddNodes]" << std::endl;
    }
    else {
        // std::cout << "Sanity check NOT OK - unexpected number of devices selected from device pool [AddNodes]" << std::endl;
        NS_LOG_DEBUG("Sanity check NOT OK - unexpected number of devices selected from device pool [AddNodes]");
    }

 
// 1.5. place new nodes nodes 
    PlaceNodes(&new_vehicle_nodes, topoManager->node_ids, topoManager->loc_updates, 0);
    
// 2. add to active UE list and associate all active UEs for broadcast 

    activeTxUes.Add(new_broadcast_devices); 
    activeUeVeh.Add(new_vehicle_nodes);
    std::vector<NetDeviceContainer> newTxGroups = topoManager->preconfig_lteV2xHelper->AssociateForV2xBroadcast(activeTxUes, activeTxUes.GetN()); 

    // std::cout << "3. Configuring broadcast: " << activeTxUes.GetN() << " total active devices" << std::endl; 

    // topoManager->preconfig_lteV2xHelper->PrintGroups(newTxGroups); 

    // std::cout << "Seg fault check: " << availableDevices.GetN() << " inactive devices, " << activeTxUes.GetN() <<  " active UEs, and " << activeUeVeh.GetN() << " active nodes, with " << new_vehicle_nodes.GetN() << " on the way" << std::endl;


// 3. loop through broadcast association list, generate flow templates for each transmitter, and activate sidelink 
    std::vector<NetDeviceContainer>::iterator gIt; // group iterator 
    for(gIt = newTxGroups.begin(); gIt != newTxGroups.end(); gIt++)  {
        // first node in each group is (implicitly) designated as transmitter 
        uint32_t curr_tx_id = gIt->Get(0)->GetNode()->GetId();

        // check which flow templates are needed
        std::vector<uint32_t>::iterator is_new = std::find(added_node_ids.begin(), added_node_ids.end(), curr_tx_id);
        // --> if current TX UE is in list of added UEs, need to install new TFT set for all 
        // --> if not, then only need to install "RECEIVE" template on new UEs 

        if (is_new != added_node_ids.end()) {
            // new UE - generate full set of tx/rx flow templates for new UE 

            // separate TX node and RX nodes (by net device)
            NetDeviceContainer txUe ((*gIt).Get(0));
            NetDeviceContainer rxUes = topoManager->preconfig_lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));

            // extract L2/L3 addresses from TX node to define flow template 
            uint32_t     tx_L2_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_L2Addr;
            Ipv4Address  tx_responders_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcastAddr;

            // std::cout << "3a. New transmitter: " << curr_tx_id << " with IP/L2" << tx_responders_address << "/" << tx_L2_address << std::endl; 

            // while we're here, activate APP (activation function is trivial for now -> TODO: reorganize processes for dormant nodes to minimize resource consumption)
            txUe.Get(0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->StartApplication();

            // define flow templates for current transmitter 
            Ptr<LteSlTft> tft_tx = Create<LteSlTft> (LteSlTft::TRANSMIT, tx_responders_address, tx_L2_address);
            Ptr<LteSlTft> tft_rx = Create<LteSlTft> (LteSlTft::RECEIVE, tx_responders_address, tx_L2_address);

            // activate sidelink and install flow template for current transmitter on all new nodes 
            topoManager->preconfig_lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), txUe,  tft_tx); // activate sidelink "0.0 seconds from now"
            topoManager->preconfig_lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), rxUes, tft_rx); // " " " " "

            // activate MAC trace to enable packet trasmission
            Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>(txUe.Get(0)->GetObject<LteUeNetDevice> ()->GetMac ());

            ueMac->TraceConnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, txUe.Get(0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcast_socket)); // may contribute to high RTF 
            // available: 
            // ueMac->TraceDisconnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, host)); 

        }
        else { 
            // existing UE - only generate rx flow template for new devices 

            // separate TX node and RX nodes (by net device) 
            NetDeviceContainer txUe ((*gIt).Get(0)); 
            // NetDeviceContainer rxUes = new_broadcast_devices; --> can just use directly 

            // extract L2/L3 addresses from TX node to define flow template 
            uint32_t tx_L2_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_L2Addr;
            Ipv4Address tx_responders_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcastAddr;

            // std::cout << "3b. Existing transmitter: " << curr_tx_id << " with IP/L2" << tx_responders_address << tx_L2_address << std::endl; 

            // define flow templates for current transmitter 
            Ptr<LteSlTft> tft_rx = Create<LteSlTft> (LteSlTft::RECEIVE, tx_responders_address, tx_L2_address);

            // activate sidelink and install flow template for current transmitter on all new nodes 
            topoManager->preconfig_lteV2xHelper->ActivateSidelinkBearer (Seconds(0.0), new_broadcast_devices, tft_rx); 

        }
    }
    // verify nodes added 
    // std::cout << "Nodes added!" << std::endl;
    // std::cout << "Seg fault check - end of AddNodes" << std::endl;
}




void
RemoveNodes(DynamicTopoManager * topoManager)
/**
 * Used to "remove" a node from the current network topology 
 * - In practice, we just remove all network configuration info and probably "uninstall" app 
 * - Need to maintain pool of "disabled" nodes, since recycling objects should improve computational efficiency 
 * 
 * ==> first pass will just be adding nodes, we will disable nodes in the future 
 *     - might be sufficient to remove from flow templates, try this first 
 *     - otherwise, leave all TX/RX control to app layer and just disable via flag (probably less efficient)
*/
{
// 1. make sure selected nodes are already active
//    --> if they are, add to new node/netDevice containers
//    --> if not, remove from incoming node vector 
    NetDeviceContainer::Iterator ndIt;

    // copy vector of intended indices for selected nodes 
    std::vector<uint32_t> nodes_to_remove(topoManager->node_ids);
    std::vector<uint32_t> valid_targets;


    // no need to do anything here if we don't have new nodes to remove 
    if (nodes_to_remove.size() == 0) {
        return; 
    }

    // containers to sort through removed/remaining devices 
    NetDeviceContainer exiting_devices; 
    NodeContainer exiting_nodes;
    NetDeviceContainer remaining_devices; 
    NodeContainer remaining_veh;

    // loop through current "active" net devices 
    for (ndIt = activeTxUes.Begin(); ndIt != activeTxUes.End(); ndIt++) {
        // get node ID of each "active" net device 
        uint32_t active_node_id = (*ndIt)->GetNode()->GetId();

        // get index of "active" node ID if contained within vector of incoming node IDs
        auto check = std::find(nodes_to_remove.begin(), nodes_to_remove.end(), active_node_id);


        // if the index is ".end()", the current node was not designated for removal (add to "remaining" UE container) -- otherwise, add to "exiting" container 
        if (check == nodes_to_remove.end()) {
            // active node ID NOT FOUND in list of selected nodes 

            // std::cout << "Node " << active_node_id << " will remain active." << std::endl;
            
            // add to list of "remaining" devices after selected UEs are removed --> likely safer than removing elements from "activeTxUes" during iteration 
            remaining_devices.Add(*ndIt);
            remaining_veh.Add((*ndIt)->GetNode());
        }
        else { 
            // active node ID FOUND in list of selected nodes 
            // --> add "this" net device AND node to new containers 
            exiting_devices.Add(*ndIt);
            exiting_nodes.Add((*ndIt)->GetNode());

            valid_targets.push_back(active_node_id); // found a match, so it doesn't matter where we pull the ID 

            // shut down IPV4 interface 
            (*ndIt)->GetNode()->GetObject<Ipv4>()->SetDown(1);
            // std::cout << "Node " << (*ndIt)->GetNode()->GetId() << " IPV4 interface down" << std::endl;
            
            // disable app layer (for now, just packet generation)
            // (*ndIt)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_running = true;

            // remove node ID from user vector now that it is added to "exiting" container (using same "remove/erase" idiom) 
            nodes_to_remove.erase(std::remove(nodes_to_remove.begin(), nodes_to_remove.end(), *check), nodes_to_remove.end()); // TODO: double-check logic here - not familiar with idiom 
        }
    }

    if (nodes_to_remove.size() == 0) {
        // all selected nodes were found in active UE list 
    }
    else {
        // not all selected nodes were active
        // std::cout << "Not all selected nodes are active in the simulation. Disabling only active nodes." << std::endl; 
        NS_LOG_INFO("Not all selected nodes are active in the simulation. Disabling only active nodes.");
    }

    // get number of nodes we are about to remove 
    uint32_t num_exiting_vehicles = valid_targets.size();

// 2. make sure we have at least N active net devices - if not, just return with error message 
    if (activeTxUes.GetN() < num_exiting_vehicles) {
        // std::cout << "Not enough active nodes in network scenario - select fewer nodes to remove. " << std::endl;
        NS_LOG_DEBUG("Not enough active nodes in network scenario - select fewer nodes to remove. ");
        return; 
    }
    else {
        // pass (or add all following code here)
    }


    // sanity check 
    if (exiting_devices.GetN() == num_exiting_vehicles) {
        // std::cout << "Sanity check OK - all requested devices active and ready to be deactivated [L.RemoveNodes]" << std::endl;
    }
    else {
        // std::cout << "Sanity check NOT OK - unexpected number of devices selected from device pool [L.RemoveNodes]" << std::endl;
        NS_LOG_DEBUG("Sanity check NOT OK - unexpected number of devices selected from device pool [L.RemoveNodes]");
    }

// 1.5. move removed nodes back to arbitrary null coordinates (0,0)  
    PlaceNodes(&exiting_nodes, topoManager->node_ids, topoManager->loc_updates, 1);


// 2. generate broadcast association list to remove flow templates 
    std::vector<NetDeviceContainer> oldTxGroups = topoManager->preconfig_lteV2xHelper->AssociateForV2xBroadcast(activeTxUes, activeTxUes.GetN()); 


// 3. loop through broadcast association list, generate flow templates for each transmitter, and activate sidelink 
    std::vector<NetDeviceContainer>::iterator gIt; // group iterator 
    for(gIt = oldTxGroups.begin(); gIt != oldTxGroups.end(); gIt++)  {
        // first node in each group is (implicitly) designated as transmitter 
        uint32_t curr_tx_id = gIt->Get(0)->GetNode()->GetId();

        // check which flow templates are needed
        auto is_new = std::find(valid_targets.begin(), valid_targets.end(), curr_tx_id);
        // --> if current TX UE is in list of UEs to remove, need to deactivate existing TFT set for all 
        // --> if not, then only need to deactivate tall "RECEIVE" template on removed UEs 

        if (is_new != valid_targets.end()) {
            // removed UE - need to remove all associated flow templates 

            // std::cout << "Deactivating sidelink: removing TX/RX flow template for node " << curr_tx_id << std::endl;

            // separate TX node and RX nodes (by net device)
            NetDeviceContainer txUe ((*gIt).Get(0));
            NetDeviceContainer rxUes = topoManager->preconfig_lteV2xHelper->RemoveNetDevice ((*gIt), txUe.Get (0));

            // extract L2/L3 addresses from TX node to define flow template 
            uint32_t     tx_L2_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_L2Addr;
            Ipv4Address  tx_responders_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcastAddr;

            // while we're here, deactivate APP (deactivation function is trivial for now -> TODO: reorganize processes for dormant nodes to minimize resource consumption)
            txUe.Get(0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->StopApplication();

            // define flow templates for current transmitter 
            Ptr<LteSlTft> tft_tx = Create<LteSlTft> (LteSlTft::TRANSMIT, tx_responders_address, tx_L2_address);
            Ptr<LteSlTft> tft_rx = Create<LteSlTft> (LteSlTft::RECEIVE, tx_responders_address, tx_L2_address);

            // deactivate sidelink and remove flow template for departing transmitter on all nodes 
            topoManager->preconfig_lteV2xHelper->DeactivateSidelinkBearer (Seconds(0.0), txUe,  tft_tx); // activate sidelink "0.0 seconds from now"
            topoManager->preconfig_lteV2xHelper->DeactivateSidelinkBearer (Seconds(0.0), rxUes, tft_rx); // " " " " "

            // disconnect MAC trace to try and save computational resources 
            // --> works, but does not seem to save much execution time at the moment 
            // Ptr<LteUeMac> ueMac = DynamicCast<LteUeMac>(txUe.Get(0)->GetObject<LteUeNetDevice> ()->GetMac ());
            // ueMac->TraceDisconnectWithoutContext ("SidelinkV2xAnnouncement", MakeBoundCallback (&SidelinkV2xAnnouncementMacTrace, txUe.Get(0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcast_socket)); // may contribute to high RTF 

        }
        else { 
            // remaining UE - should only need to remove "RECIEVE" template from departing nodes 

            // separate TX node and RX nodes (by net device)
            NetDeviceContainer txUe ((*gIt).Get(0));
            // NetDeviceContainer rxUes = new_broadcast_devices; --> can just use directly 

            // extract L2/L3 addresses from TX node to define flow template 
            uint32_t     tx_L2_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_L2Addr;
            Ipv4Address  tx_responders_address = txUe.Get (0)->GetNode()->GetApplication(0)->GetObject<HelicsCtrlApp>()->m_bcastAddr;

            // define flow templates for current transmitter 
            Ptr<LteSlTft> tft_rx = Create<LteSlTft> (LteSlTft::RECEIVE, tx_responders_address, tx_L2_address);

            // std::cout << "Removing node " << curr_tx_id << "/" << tx_responders_address << "/" << tx_L2_address << " RX flow template on " << exiting_devices.GetN() << " exiting devices" << std::endl;

            // deactivate sidelink and remove flow template for current transmitter on all departing nodes 
            topoManager->preconfig_lteV2xHelper->DeactivateSidelinkBearer (Seconds(0.0), exiting_devices, tft_rx); 

        }
    }
    // update global list of active devices (by assignment, since we cannot "safely" remove net devices from container)
    activeTxUes = remaining_devices;
    activeUeVeh = remaining_veh;
}




int 
main (int argc, char *argv[])
{
    LogComponentEnable ("v2x_communication_mode_4", LOG_DEBUG);
        
    // declare user config struct and support for dynamic topology 
    // --> created here as pointer to follow ns-3 formatting without requiring adherence to ns3 "Ptr< >" class 
    DynamicTopoManager currTopoManager;

    // Initialize simulation parameters 
    uint32_t inst_idx = 0;                  // Index ID of current simulation instance (necessary when running multiple instances in parallel)
    uint32_t reporting = 9;                 // Verbosity of network metrics (use 1 for longer timesteps, 9 for shorter timesteps i.e. synchronous co-simulation)
    uint32_t simTime = 10;                  // Maximum simulation time in seconds (dummy value for CMD - will be overwritten by HELICS input later) 
    uint32_t simTimeStep = 200;             // Timestep of simulation (in milliseconds) - MUST MATCH AIMSUN - will be overwritten by init data 
    uint32_t io_sock_port = 60000;          // Client (source) port for simulation I/O socket  
    uint32_t wrapper_port = 50000;          // Server (destination) port of wrapper socket 
    std::string tracefile;                  // Name of the tracefile 

    // Initialize vehicle network parameters  
    uint32_t numVeh = 10;                   // Maximum supported number of vehicles in dynamic topology simulation 
    uint32_t lenCam_default = 190;          // Length of cooperative awareness message (CAM) in bytes - ignored in favor of pre-defined message set 
    uint32_t mcs = 20;                      // Modulation and Coding Scheme
    uint32_t sizeSubchannel = 10;           // Number of RBs per subchannel
    uint32_t numSubchannel = 3;             // Number of subchannels per subframe
    uint32_t startRbSubchannel = 0;         // Index of first RB corresponding to subchannelization
    uint32_t pRsvp = 100;				    // Resource reservation interval (period between successive TX schedules after resources selected)
    uint32_t t1 = 4;                        // T1 value of selection window (how fast to select resources after sensing window, ms)
    uint32_t t2 = 100;                      // T2 value of selection window (size of resource selection window, ms)
    uint32_t slBandwidth;                   // Sidelink bandwidth
    double ueTxPower = 23.0;                // Transmission power in dBm
    double probResourceKeep = 0.0;          // Probability to select the previous resource again [0.0-0.8]
    bool harqEnabled = false;               // Retransmission enabled 
    bool adjacencyPscchPssch = true;        // Subchannelization scheme
    bool partialSensing = false;            // Partial sensing enabled (actual only partialSensing is false supported)


    // Command line arguments
    CommandLine cmd;

    // simulation setup variables 
    cmd.AddValue("time", "Simulation Time", simTime);
    cmd.AddValue("inst_idx", "integer ID (index) of current simulation", inst_idx); 
    cmd.AddValue("reporting", "integer indicator of control plane report verbosity", reporting);
    cmd.AddValue("origin_x", "x-value of location origin of this simulation instance", origin_x);
    cmd.AddValue("origin_y", "y-value of location origin of this simulation instance", origin_y);
    cmd.AddValue("io_port", "port number for blocking sync port at control plane", io_sock_port); 
    cmd.AddValue("wrapper_port", "port number for blocking sync port at control plane", wrapper_port); 
    // cmd.AddValue ("log_simtime", "name of the simtime logfile", simtime);
    // cmd.AddValue ("log_rx_data", "name of the rx data logfile", rx_data);
    // cmd.AddValue ("log_tx_data", "name of the tx data logfile", tx_data);
    // cmd.AddValue ("tracefile", "Path of ns-3 tracefile", tracefile); 

    // vehicle network variables 
    cmd.AddValue ("numVeh", "Number of Vehicles", numVeh);
    cmd.AddValue ("adjacencyPscchPssch", "Scheme for subchannelization", adjacencyPscchPssch); 
    cmd.AddValue ("sizeSubchannel", "Number of RBs per Subchannel", sizeSubchannel);
    cmd.AddValue ("numSubchannel", "Number of Subchannels", numSubchannel);
    cmd.AddValue ("startRbSubchannel", "Index of first subchannel index", startRbSubchannel); 
    cmd.AddValue ("T1", "T1 Value of Selection Window", t1);
    cmd.AddValue ("T2", "T2 Value of Selection Window", t2);
    cmd.AddValue ("harqEnabled", "HARQ Retransmission Enabled", harqEnabled);
    cmd.AddValue ("partialSensingEnabled", "Partial Sensing Enabled", partialSensing);
    cmd.AddValue ("lenCam_default", "Packetsize in Bytes (deprecated)", lenCam_default);
    cmd.AddValue ("mcs", "Modulation and Coding Scheme", mcs);
    cmd.AddValue ("pRsvp", "Resource Reservation Interval", pRsvp); 
    cmd.AddValue ("probResourceKeep", "Probability for selecting previous resource again", probResourceKeep); 
    cmd.AddValue ("baseline", "Distance in which messages are transmitted and must be received", baseline);

    cmd.Parse (argc, argv);
    
    // std::cout << "\nStarting Control Plane... " << std::endl;
    Ptr<HelicsCtrlPlane> controlPlane = CreateObject<HelicsCtrlPlane>();
    controlPlane->Setup(inst_idx, io_sock_port, wrapper_port, reporting);
    // std::cout << "Control Plane Started!" << std::endl;

    // std::cout << "\nWaiting for INIT data..." << std::endl;
    NS_LOG_DEBUG("\nWaiting for INIT data...");
    InitializationData init_data = controlPlane->GetInitParams();
    // numVeh = int(init_data.node_ids.size() * 1.2); // maximum number of nodes = integer floor of number of nodes in node ID vector * 1.2
    // TODO: figure out how to trigger ACK for Aimsun steady-state 
    numVeh = 30; 
    
    simTime = init_data.sim_runtime; 
    simTimeStep = init_data.sim_timestep_ms;

    controlPlane->sim_time_step = simTimeStep;

    // std::cout << "INIT data received! Setting up simulation..." << std::endl;
    NS_LOG_DEBUG("INIT data received! Setting up simulation...");

    // AsciiTraceHelper ascii;
    // log_simtime = ascii.CreateFileStream(simtime);
    // log_rx_data = ascii.CreateFileStream(rx_data);
    // log_tx_data = ascii.CreateFileStream(tx_data);
    // log_connections = ascii.CreateFileStream(connections);
    // log_positions = ascii.CreateFileStream(positions); 

    NS_LOG_INFO ("Starting network configuration..."); 

    // Set the UEs power in dBm
    Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePhy::RsrpUeMeasThreshold", DoubleValue (-10.0));
    // Enable V2X communication on PHY layer
    Config::SetDefault ("ns3::LteUePhy::EnableV2x", BooleanValue (true));

    // Set power
    Config::SetDefault ("ns3::LteUePowerControl::Pcmax", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PsschTxPower", DoubleValue (ueTxPower));
    Config::SetDefault ("ns3::LteUePowerControl::PscchTxPower", DoubleValue (ueTxPower));

    if (adjacencyPscchPssch) {
        slBandwidth = sizeSubchannel * numSubchannel;
    }
    else {
        slBandwidth = (sizeSubchannel+2) * numSubchannel; 
    }

    // Configure for UE selected
    Config::SetDefault ("ns3::LteUeMac::UlBandwidth", UintegerValue(slBandwidth));
    Config::SetDefault ("ns3::LteUeMac::EnableV2xHarq", BooleanValue(harqEnabled));
    Config::SetDefault ("ns3::LteUeMac::EnableAdjacencyPscchPssch", BooleanValue(adjacencyPscchPssch));
    Config::SetDefault ("ns3::LteUeMac::EnablePartialSensing", BooleanValue(partialSensing));
    Config::SetDefault ("ns3::LteUeMac::SlGrantMcs", UintegerValue(mcs));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelSize", UintegerValue (sizeSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlSubchannelNum", UintegerValue (numSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlStartRbSubchannel", UintegerValue (startRbSubchannel));
    Config::SetDefault ("ns3::LteUeMac::SlPrsvp", UintegerValue(pRsvp));
    Config::SetDefault ("ns3::LteUeMac::SlProbResourceKeep", DoubleValue(probResourceKeep));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT1", UintegerValue(t1));
    Config::SetDefault ("ns3::LteUeMac::SelectionWindowT2", UintegerValue(t2));
    //Config::SetDefault ("ns3::LteUeMac::EnableExcludeSubframe", BooleanValue(excludeSubframe)); 

    ConfigStore inputConfig; 
    inputConfig.ConfigureDefaults(); 

    // Create node container to hold all UEs 
    NodeContainer ueAllNodes; 




/////////////////////////////////////////////////////////////////////////////////////////////
// 1. Place all nodes (based on maximum estimate, not initialized topology)

    // NS_LOG_UNCOND("Starting simulation, please send COORDS info...\n");
    // ParseHelMsg(&userData);
    // NS_LOG_UNCOND("COORDS info received!\n");

    ueVeh.Create (numVeh);
    ueAllNodes.Add (ueVeh);

    // PlaceNodes will ignore node ID vector and node location vector if reset flag is "1" - just need to pass correct type  
    std::vector<uint32_t> dummy_id_vector;
    std::vector<Vector> dummy_loc_vector;
    NS_LOG_INFO ("Installing Mobility Model...");
    // control plane function to place nodes at user-defined locations: 
    PlaceNodes(&ueVeh, dummy_id_vector, dummy_loc_vector, 1); // reset = 1 for initial location of all nodes (all placed at [0,0] initially - we will assign active nodes later) 

    inactiveUeVeh.Add(ueVeh); // all nodes initialized as "inactive" for now 

// end node placement 
/////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////
// 2. Construct LTE network and protocol stack helpers 

    NS_LOG_INFO ("Creating helpers...");
    // EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // LTE Helper
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->DisableNewEnbPhy(); // Disable eNBs for out-of-coverage modelling
    
    // V2X 
    Ptr<LteV2xHelper> lteV2xHelper = CreateObject<LteV2xHelper> ();
    lteV2xHelper->SetLteHelper (lteHelper); 

    // Configure eNBs' antenna parameters before deploying them.
    lteHelper->SetEnbAntennaModelType ("ns3::NistParabolic3dAntennaModel");

    // Set pathloss model
    // FIXME: InstallEnbDevice overrides PathlossModel Frequency with values from Earfcn
    // 
    lteHelper->SetAttribute ("UseSameUlDlPropagationCondition", BooleanValue(true));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlEarfcn", StringValue ("54990"));
    //Config::SetDefault ("ns3::CniUrbanmicrocellPropagationLossModel::Frequency", DoubleValue(5800e6));

    // TODO: should we switch this with WINNER+B1? what is the difference? --> look at paper 
    lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel")); 
    

    // Create eNB Container
    NodeContainer eNodeB;
    eNodeB.Create(1); 

    // Topology eNodeB
    Ptr<ListPositionAllocator> pos_eNB = CreateObject<ListPositionAllocator>(); 
    pos_eNB->Add(Vector(5,-10,30));

    //  Install mobility eNodeB
    MobilityHelper mob_eNB;
    mob_eNB.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob_eNB.SetPositionAllocator(pos_eNB);
    mob_eNB.Install(eNodeB);

    // Install Service
    NetDeviceContainer enbDevs = lteHelper->InstallEnbDevice(eNodeB);

    // Required to use NIST 3GPP model
    BuildingsHelper::Install (eNodeB);
    BuildingsHelper::Install (ueAllNodes);
    BuildingsHelper::MakeMobilityModelConsistent (); 


// end LTE deployment 
/////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////
// 3. Install protocol stack on all nodes, attach to LTE network 

    // Install LTE devices to all UEs 
    NS_LOG_INFO ("Installing UE's network devices...");
    lteHelper->SetAttribute("UseSidelink", BooleanValue (true));
    NetDeviceContainer ueRespondersDevs = lteHelper->InstallUeDevice (ueVeh);
    NetDeviceContainer ueDevs;
    ueDevs.Add (ueRespondersDevs); 

    availableDevices.Add (ueRespondersDevs); // all devices are initially "available" (inactive)

    // Install the IP stack on the UEs
    NS_LOG_INFO ("Installing IP stack..."); 
    InternetStackHelper internet;
    internet.Install (ueAllNodes); 

    // Assign IP adress to UEs
    NS_LOG_INFO ("Allocating IP addresses and setting up network route...");
    // Ipv4InterfaceContainer ueIpIface; // global for convenience 
    ueIpIface = epcHelper->AssignUeIpv4Address (ueDevs);
    Ipv4StaticRoutingHelper Ipv4RoutingHelper;

    for(uint32_t u = 0; u < ueAllNodes.GetN(); ++u) {
        Ptr<Node> ueNode = ueAllNodes.Get(u);

        // while we're iterating through all nodes, we may as well update control plane with concrete ns-3 IDs assigned to each one 
        // --> likely better practice than pre-allocating using "iota" based on known rules 
        controlPlane->available_ns3_ids.push_back(ueNode->GetId());

        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting = Ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress(), 1);

        // // print IP address of each node 
        // std::cout << "Node "<< ueNode->GetId () << " has been assigned an IP address: " << ueNode->GetObject<Ipv4> ()->GetAddress(1,0).GetLocal() << std::endl;
    }

    NS_LOG_INFO("Attaching UE's to LTE network...");
    // Attach each UE to the best available eNB 
    lteHelper->Attach(ueDevs); 


// end network attach 
/////////////////////////////////////////////////////////////////////////////////////////////







/////////////////////////////////////////////////////////////////////////////////////////////
// 4. Create sidelink groups for broadcast 

    NS_LOG_INFO ("Creating sidelink groups...");
    std::vector<NetDeviceContainer> txGroups;

    txGroups = lteV2xHelper->AssociateForV2xBroadcast(ueRespondersDevs, numVeh); 
    // NS_LOG_UNCOND ("DEBUG: Printing txGroups: ");
    // lteV2xHelper->PrintGroups(txGroups); 

    NS_LOG_INFO ("Installing applications..."); 

    // Application Setup for Responders
    std::vector<uint32_t> groupL2Addresses; 
    uint32_t groupL2Address = 0x00; 
    Ipv4AddressGenerator::Init(Ipv4Address ("225.0.0.0"), Ipv4Mask("255.0.0.0"));
    Ipv4Address clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));


// end create sidelink groups 
/////////////////////////////////////////////////////////////////////////////////////////////






/////////////////////////////////////////////////////////////////////////////////////////////
// 5. (new) Assign L2/L3 addresses to ueResponderDevs - do not create flow templates in this step 


    std::vector<HelicsCtrlApp> allApps; 
    NetDeviceContainer::Iterator ndIt;

    for (ndIt = ueRespondersDevs.Begin(); ndIt != ueRespondersDevs.End(); ndIt++) {
        uint32_t curr_tx_id = (*ndIt)->GetNode()->GetId();

        Ptr<Socket> host = Socket::CreateSocket((*ndIt)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        host->Bind();
        host->Connect(InetSocketAddress(clientRespondersAddress, application_port));
        host->SetAllowBroadcast(true);
        host->ShutdownRecv(); 

        // existing sink socket 
        Ptr<Socket> sink = Socket::CreateSocket((*ndIt)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), application_port));
        sink->SetRecvCallback (MakeCallback (&ReceivePacket)); // SetRecvCallback indicates when new data is available to be read

        // custom sink socket at different port 
        Ptr<Socket> meas_sink = Socket::CreateSocket((*ndIt)->GetNode(),TypeId::LookupByName ("ns3::UdpSocketFactory"));
        meas_sink->Bind(InetSocketAddress (Ipv4Address::GetAny (), measurement_port));

        // declare app instance for this node 
        Ptr<HelicsCtrlApp> app = CreateObject<HelicsCtrlApp>();
        app->Setup (curr_tx_id, host, meas_sink, measurement_port, clientRespondersAddress, groupL2Address); // ns-3 should not care about Aimsun ID - should be handled at wrapper
        (*ndIt)->GetNode()->AddApplication (app);
        app->m_sink_socket->SetRecvCallback (MakeCallback (&ReceivePacket)); // set receive callback at APP layer to local function (for convenience) 

        //store and increment MAC addresses 
        groupL2Addresses.push_back (groupL2Address);
        groupL2Address++;
        clientRespondersAddress = Ipv4AddressGenerator::NextAddress (Ipv4Mask ("255.0.0.0"));
    }


// end create flow templates 
/////////////////////////////////////////////////////////////////////////////////////////////





/////////////////////////////////////////////////////////////////////////////////////////////
// 8. Define and install sidelink configuration 


    NS_LOG_INFO ("Creating Sidelink Configuration...");
    Ptr<LteUeRrcSl> ueSidelinkConfiguration = CreateObject<LteUeRrcSl>();
    ueSidelinkConfiguration->SetSlEnabled(true);
    ueSidelinkConfiguration->SetV2xEnabled(true);

    LteRrcSap::SlV2xPreconfiguration preconfiguration;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.carrierFreq = 54890;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommPreconfigGeneral.slBandwidth = slBandwidth;
    
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.nbPools = 1;
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.nbPools = 1;

    SlV2xPreconfigPoolFactory pFactory;
    pFactory.SetHaveUeSelectedResourceConfig (true);
    pFactory.SetSlSubframe (std::bitset<20> (0xFFFFF));
    pFactory.SetAdjacencyPscchPssch (adjacencyPscchPssch);
    pFactory.SetSizeSubchannel (sizeSubchannel);
    pFactory.SetNumSubchannel (numSubchannel);
    pFactory.SetStartRbSubchannel (startRbSubchannel);
    pFactory.SetStartRbPscchPool (0);
    pFactory.SetDataTxP0 (-4);
    pFactory.SetDataTxAlpha (0.8);

    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommTxPoolList.pools[0] = pFactory.CreatePool ();
    preconfiguration.v2xPreconfigFreqList.freq[0].v2xCommRxPoolList.pools[0] = pFactory.CreatePool ();
    ueSidelinkConfiguration->SetSlV2xPreconfiguration (preconfiguration); 

    NS_LOG_INFO ("Installing Sidelink Configuration...");
    lteHelper->InstallSidelinkV2xConfiguration (ueRespondersDevs, ueSidelinkConfiguration);


// end sidelink definition 
/////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////
// 9. Deploy network nodes based on initialization data 

    // initialize map between Aimsun node IDs and ns-3 IDs 

    std::vector<uint32_t>::iterator idIt; // aimsun ID iterator

    for (idIt = init_data.node_ids.begin(); idIt < init_data.node_ids.end(); idIt++) {

        // get first available ns-3 ID from existing nodes  
        uint32_t candidate_ns3_id = controlPlane->available_ns3_ids.front();
        
        // update control plane map 
        controlPlane->id_map[(*idIt)] = candidate_ns3_id;

        // pop assigned ID from deque of available IDs 
        controlPlane->available_ns3_ids.pop_front();

        // add ns-3 ID corresponding to Aimsun node to current topology 
        currTopoManager.node_ids.push_back(candidate_ns3_id); 
    }

    // update the rest of the topology resource set using initialization data and V2X helper 
    // --> node order will be preserved - can use node location vector as-is 
    currTopoManager.loc_updates = init_data.node_init_locs; // TODO: does not work 
    currTopoManager.preconfig_lteV2xHelper = lteV2xHelper; 






    // deploy current Aimsun topology 
    AddNodes(&currTopoManager);

// end deployment 
/////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////
// 10. Schedule events and run the simulation 

    // schedule recursive I/O update function - 5 seconds is usually enough to make sure all nodes are activem, but we use 3 for now (faster)
    Simulator::Schedule(Seconds(3), &stateUpdateReport, controlPlane, simTimeStep, &currTopoManager);

    // Simulator::Schedule(Seconds(1), &PrintStatus, 10, start); // simtime argument is now in ms 


// --> 'adding' nodes after simulation starts:  

    // currTopoManager.node_ids = {4, 5};
    // currTopoManager.preconfig_lteV2xHelper = lteV2xHelper; 
    // currTopoManager.loc_updates = {Vector(8, 8, 2), Vector(10, 10, 2)};

    // Simulator::Schedule(Seconds(3), &AddNodes, &currTopoManager); 


// --> 'removing' nodes after simulation starts:  

    // DynamicTopoManager newTopoManager; // cannot change object before AddNodes is called 
    // newTopoManager.node_ids = {0, 1};
    // newTopoManager.preconfig_lteV2xHelper = lteV2xHelper; 
    // newTopoManager.loc_updates = {}; 
    // Simulator::Schedule(Seconds(4), &RemoveNodes, &newTopoManager);



    
    NS_LOG_INFO ("Starting Simulation...");
    Simulator::Stop(MilliSeconds(simTime*1000 + 40)); // why "+40" milliseconds? value left by original authors 

    // measure runtime 
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); // std::chrono::steady_clock::time_point wallClockStartTime = std::chrono::steady_clock::now();

    Simulator::Run();
    Simulator::Destroy();

    std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();
    std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    uint32_t runtime = duration.count();

    NS_LOG_INFO("Simulation done.");

    // std::cout << "> Duration: " << runtime << " milliseconds." << std::endl;
    NS_LOG_INFO("> Duration: " << runtime << " milliseconds.");

    //std::this_thread::sleep_for(std::chrono::seconds(100)); // terminal dies too fast when launched as daughter process

    return 0; 
    
// end simulation 
/////////////////////////////////////////////////////////////////////////////////////////////
}   


