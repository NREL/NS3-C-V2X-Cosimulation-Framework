/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Contribution 11/13/2023: HELICS Control App 
 * Author: Maxwell McManus
 *         National Renewable Energy Laboratory (NREL) 
 */
#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv6-address.h"
#include "ns3/internet-module.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/mobility-module.h"


#include <vector>
#include <string>
#include <deque> 

#include "helics-control-app.h"
// #include "user-packet-lib.h" // pre-defined packet objects 
#include "v2x-packet-tag.h"
#include "ns3/tag.h"

namespace ns3 {


NS_LOG_COMPONENT_DEFINE ("HelicsCtrlPlaneInterfaceApplication");

NS_OBJECT_ENSURE_REGISTERED (HelicsCtrlApp);



TypeId
HelicsCtrlApp::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::HelicsCtrlApp")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<HelicsCtrlApp> ()

    // motivation of this vs normal member parameter initialization is unclear - just for documentation?
    .AddAttribute ("Ns3Id",
                   "ID of this node within ns3",
                   UintegerValue (0),
                   MakeUintegerAccessor (&HelicsCtrlApp::m_ns3Id),
                   MakeUintegerChecker<uint32_t> ())
    // .AddAttribute ("ExtNodeId",
    //                "ID associated with this node in external co-simulation",
    //                UintegerValue (0),
    //                MakeUintegerAccessor (&HelicsCtrlApp::m_aimsunId),
    //                MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("AppPort",
                   "Port for on-demand APP-layer packets",
                   UintegerValue (0),
                   MakeUintegerAccessor (&HelicsCtrlApp::m_port),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("DefaultBroadcastAddr",
                   "Default peer address for APP-layer broadcast",
                   Ipv4AddressValue (),
                   MakeIpv4AddressAccessor (&HelicsCtrlApp::m_bcastAddr),
                   MakeIpv4AddressChecker ())
    .AddAttribute ("L2Addr",
                   "MAC-layer identifier, stored here for convenience",
                   UintegerValue (0),
                   MakeUintegerAccessor (&HelicsCtrlApp::m_L2Addr),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("PacketsSent",
                   "Number of packets sent on this node",
                   UintegerValue (0),
                   MakeUintegerAccessor (&HelicsCtrlApp::m_packetsSent),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("PacketsReceived",
                   "Number of packets received on this node",
                   UintegerValue (0),
                   MakeUintegerAccessor (&HelicsCtrlApp::m_packetsRcvd),
                   MakeUintegerChecker<uint32_t> ())
    
    // Add more trace sources as they become relevant 
    .AddTraceSource ("NewCommand", "A new command is provided by HELICS", // probably implemented in control plane class 
                     MakeTraceSourceAccessor (&HelicsCtrlApp::m_cmdTrace),
                     "ns3::Packet::TracedCallback")
  ;
  return tid;
}

// constructor/destructor: 

HelicsCtrlApp::HelicsCtrlApp ()
{
  NS_LOG_FUNCTION (this);
  m_running = false; 
  m_packetsSent = 0;
  m_packetsRcvd = 0;

  // std::ostringstream msgDenm;
  // msgDenm << 2 << ";" << 20. << ";" << "test;" << '\0';
  // m_packet = Create<Packet>((uint8_t*)msgDenm.str().c_str(), 800);
}

HelicsCtrlApp::~HelicsCtrlApp()
{
  NS_LOG_FUNCTION (this);
  m_bcast_socket = 0;
  m_sink_socket = 0;

  // TODO: do we need to manually destroy anything else to free up memory? 
  // --> no "external" socket at APP layer 
}


// functions: 

void
HelicsCtrlApp::Setup (uint32_t ns3Id, Ptr<Socket> bcast_socket, Ptr<Socket> sink_socket, uint32_t port, Ipv4Address bcastAddr, uint32_t L2Addr)
// HelicsCtrlApp::Setup (uint32_t ns3Id, uint32_t extId, Ptr<Socket> bcast_socket, Ptr<Socket> sink_socket, uint32_t port, Ipv4Address bcastAddr, uint32_t L2Addr)
{
    // NS_LOG_UNCOND ("HelicsApp: Entering Setup.");
    m_ns3Id = ns3Id;
    // m_aimsunId = extId;
    m_bcast_socket = bcast_socket;
    m_sink_socket  = sink_socket;
    m_port = port;
    m_bcastAddr = bcastAddr; // default "peer" is broadcast address 
    m_L2Addr = L2Addr; // added here for easy access to MAC address 


} 





// updated to accept expected message set format 
void
HelicsCtrlApp::QueuePacketSet (const char * pkt_key_set) 
/**
 * Assumes pkt_key is directly passed from wrapper as "string" of packet keys 
 * 
 * --> i.e. "233" == [msg2, msg3, msg3], "1" == [msg1], etc. 
*/
{
  std::stringstream placeholder;
  uint32_t int_pkt_key;

  // std::cout << "Queued packet set: " << pkt_key_set << std::endl;

  for (uint32_t numpkts = 0; numpkts != strlen(pkt_key_set); numpkts++) {
    placeholder << pkt_key_set[numpkts];
    placeholder >> int_pkt_key; 

    // std::cout << " > queued packet: " << int_pkt_key << std::endl;

    m_pktDeque.push_back(int_pkt_key);
  }
}



void
HelicsCtrlApp::FlushQueue (void) 
{
  if (m_pktDeque.empty()) {
    // nothing in queue - pass
  }
  else {
    m_pktDeque.clear();
  }
}




uint32_t
HelicsCtrlApp::GetQueuedPacket(void)
{
    // see if we are allowed to send a packet
    if (m_running == 0) {
      // return null packet key - node is not active in simulation and we use unsigned ints (i.e. no "-1" for traditional failed return value)
      return 0; 
    }
    
    // Ptr <Node> node = m_bcast_socket->GetNode(); 
    // int id_from_socket = node->GetId();

    uint32_t next_packet_key;

    // no packet in queue - send CAM 
    if (m_pktDeque.size() == 0) {
        next_packet_key = 1;
    }
    
    else {
      // check for highest priority packet, e.g. "packetBsm1" if this correlates to something like collision detection 
      std::deque<uint32_t>::iterator bsm1_first_index = std::find(m_pktDeque.begin(), m_pktDeque.end(), 3); 
      
      // if we find an index of highest-priority packet (e.g. BSM1), send that first 
      if (bsm1_first_index != m_pktDeque.end()) {

        // dereference iterator to get element ("3") at first index of BSM1 packet key 
        next_packet_key = *bsm1_first_index;

        // remove sent packet from queue
        m_pktDeque.erase(bsm1_first_index); 
      }
      // if we don't find an index of BSM1, default to FIFO
      else {
        // send first packet in queue (since we add new packets to back )
        next_packet_key = m_pktDeque.front(); 

        // delete first element
        m_pktDeque.pop_front();
      }
    }
    
    return next_packet_key;
}



void
HelicsCtrlApp::SockRecv(Ptr<Socket> socket)
{   
    // Ptr<Node> node = socket->GetNode();
    Ptr<Node> node = this->GetNode();

    // asynchronous, non-blocking socket receive function 
    Ptr<Packet> packet = socket->Recv(); 

    // copy data from packet to local buffer for parsing 
    uint8_t *buffer = new uint8_t[packet->GetSize()];
    packet->CopyData(buffer,packet->GetSize());
    std::string s = std::string((char*)buffer);  

    // size_t pos = 0; 
    // std::string copy = s; // copy of the buffer so we can parse less carefully 
    NS_LOG_INFO("[" << node->GetObject<Ipv4L3Protocol>()->GetAddress(1,0).GetLocal() << "] " << "Ack MEAS RX at Node " << node->GetId() << ": " << packet->GetSize() << " bytes");
    // std::cout << "[" << node->GetObject<Ipv4L3Protocol>()->GetAddress(1,0).GetLocal() << "] " << "Ack MEAS RX at Node " << node->GetId() << ": " << packet->GetSize() << " bytes" << std::endl;
    m_packetsRcvd++;
}



void
HelicsCtrlApp::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}




void
HelicsCtrlApp::StartApplication (void)
/**
 * Most setup is done at simulation level for now, can add more later 
 * No need to bind/connect sockets, since they are passed as pointers after all that  
 */
{
  // NS_LOG_UNCOND ("HelicsApp: Entering StartApplication.");
  m_running = true;
  m_packetsSent = 0;
  m_packetsRcvd = 0;


}




void
HelicsCtrlApp::StopApplication (void)
/**
 * Close all associated sockets - need to verify when/how to do that in dynamic topology 
 * 
 * NOTE: cannot safely re-use sockets after closing - need to declare new socket in StartApplication
 * 
 */
{
    NS_LOG_INFO ("HelicsApp: Entering StopApplication.");
    // set operation flag to false 
    m_running = false;

    // close socket 
    if (m_bcast_socket)  {
        m_bcast_socket->Close ();
    }

    // close socket 
    if (m_sink_socket)  {
        m_sink_socket->Close ();
    }
}

} // Namespace ns3



