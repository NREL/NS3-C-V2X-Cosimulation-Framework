/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Contribution 11/13/2023: HELICS Control App   
 * Author: Maxwell McManus
 *         National Renewable Energy Laboratory (NREL) 
 */

#ifndef HELICS_CTRL_APP_H
#define HELICS_CTRL_APP_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-address.h"
#include "ns3/traced-callback.h"
#include "deque"
#include "string"

namespace ns3 {

class Socket;
class Packet;


/**
 * \brief HELICS Control Plane Interface Application 
 *
 * Provides external control of simulation runtime to HELICS co-simulation framework
 */
class HelicsCtrlApp : public Application 
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  HelicsCtrlApp ();

  virtual ~HelicsCtrlApp ();

  /**
   * \brief initialize/reassign member variables of app instance based on context 
   * \param ns3Id identifier of this node used by ns3 simulation 
  //  * \param extId identifier of this node in external simulation 
   * \param bcast_socket internal (ns3) socket used for message broadcast 
   * \param sink_socket internal (ns3) socket used to receive APP-layer broadcast 
   * \param port port used for APP-layer signaling 
   * \param bcastAddr IP address associated for sending from this node (needed for V2X broadcast)
   */
  // void Setup (uint32_t ns3Id, uint32_t extId, Ptr<Socket> bcast_socket, Ptr<Socket> sink_socket, uint32_t port, Ipv4Address bcastAddr, uint32_t L2Addr);
  void Setup (uint32_t ns3Id, Ptr<Socket> bcast_socket, Ptr<Socket> sink_socket, uint32_t port, Ipv4Address bcastAddr, uint32_t L2Addr);

  /**
   * \brief add packet to queue at this node 
   * \param pkt_key string identifier of packet type to add to queue
   */
  void QueuePacket(const char * pkt_key);


  /**
   * \brief add packet set to queue at this node 
   * \param pkt_key string of integer keys of packet types to add to queue this timestep 
   */
  void QueuePacketSet(const char * pkt_key);


  /**
   * \brief delete contents of queue (i.e. for removing node from simulation)
   */
  void FlushQueue(void);

  /**
   * \brief send a packet on member socket "m_bcast_socket" - if none queued, send CAM
   * \returns integer key associated with user-defined packet type 
   */
  uint32_t GetQueuedPacket (void);

  /**
   * \brief receive callback for APP-layer signaling 
   * \param socket pointer to member socket "m_sink_socket" 
   */
  void SockRecv (Ptr<Socket> socket);

  // need these public to have control during runtime 
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  Ptr<Socket> m_bcast_socket; //!< default APP-layer sender socket
  Ptr<Socket> m_sink_socket;  //!< default APP-layer receive socket 
  uint32_t    m_ns3Id;        //!< ns3 identifier 
  uint32_t    m_aimsunId;        //!< external simulation identifier 
  bool        m_running;      //!< boolean indicating node activity on network 
  Ipv4Address        m_bcastAddr;    //!< default SendTo address 
  uint32_t           m_L2Addr;       //!< MAC-layer address, stored here for convenience 
  uint32_t           m_port;         //!< port for APP-layer signaling    

  Ptr<Packet> m_packet;
  std::deque<uint32_t> m_pktDeque; // deque favored for efficieny (fun fact: std::queue objects are internally stored in std::deque objects)


protected:

  virtual void DoDispose (void);

private: 

  uint32_t           m_packetsSent;  //!< number of packets sent by APP layer  
  uint32_t           m_packetsRcvd;  //!< number of packets received by APP layer s

  /// Callback for tracing user-plane command events - not implemented 
  TracedCallback<Ptr<const Packet> > m_cmdTrace;
};

} // namespace ns3

#endif /* HELICS_CTRL_APP_H */
