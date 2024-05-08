/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Authored by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 * 
 */


#ifndef HELICS_CTRL_PLANE
#define HELICS_CTRL_PLANE


#include "ns3/log.h"
#include "ns3/core-module.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <deque> 


namespace ns3 {


// static variables needed for socket communications 
// #define HELMAXLINE  4096


// struct definition for params required on startup (i.e. before simulation can begin execution)
struct InitializationData 
{
    // sim initialization data 
    uint32_t sim_runtime; // in seconds 
    uint32_t sim_timestep_ms; // in milliseconds

    // node initialization data 
    std::vector<uint32_t> node_ids; 
    std::vector<Vector>   node_init_locs;
};


// struct to hold pointers to objects needed to "add/remove" (enable/disable) nodes during runtime 
struct UpdateData
{
    std::vector<uint32_t>          node_ids; 
    std::vector<Vector>            loc_updates;
    std::vector<std::string>       new_msgs;   // parse this in main file so we can access node queues directly 
};




/**
 * \brief HELICS Control Plane
 *
 * Applies actions to simulation runtime based on passed messages from HELICS (via "ns3-starter.py" wrapper)
 * 
 * No specific reason to follow ns-3 object registration formality here, but simple enough and good practice 
 */
class HelicsCtrlPlane : public Object 
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * \brief constructor/destructor
   */
  HelicsCtrlPlane ();
  virtual ~HelicsCtrlPlane ();

  /**
   * \brief assign simulation parameters needed to establish basic I/O and data reporting 
   * \param sim_port port of the socket opened in this instance of ns-3
   * \param ctrl_port port of the socket defined in the Python wrapper dedicated to this instance 
   */
  void Setup(const uint32_t inst_idx, const uint32_t sim_port, const uint32_t ctrl_port, const uint32_t reporting);


  /**
   * \brief block simulation runtime until allowed to proceed 
   * \param XX XX
   */
  void SendAck ();


  /**
   * \brief send "ready" signal to wrapper to enable synchronous start time of all simulation instances 
   * \param XX XX
   */
  void SendReady ();



  // /**
  //  * \brief 
  //  * \param XX XX
  //  */
  // void ReportData (std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > &packet_reception, std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > &packet_drops);

  /**
   * \brief 
   * \param XX XX
   */
  void ReportData (std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > &packet_reception);

  /**
   * \brief 
   * \param XX XX
   */
  void ReportAbbrevData (uint32_t pkts_sent, uint32_t pkts_received_valid);

  /**
   * \brief block simulation runtime until sent data is acknowledged by wrapper instance - provide timeout flag in case we want to send data again 
   * \param XX XX
   * \return -1 if ACK was not received before timeout, else 1 if ACK was received 
   */
  uint32_t WaitForAck ();


  /**
   * \brief block simulation runtime until allowed to proceed 
   * \param XX XX
   * \return DynamicTopoResources struct object with all update data 
   */
  UpdateData 
  WaitForData ();



  /**
   * \brief block simulation setup until we get initialization params from Aimsun (via wrapper)
   * \param XX XX
   */
  InitializationData
  GetInitParams();




  // ParamSet keyParams;
  uint32_t m_inst_idx;
  uint32_t m_io_sock_port;
  uint32_t m_wrapper_port;
  uint32_t m_sockfd;

  uint32_t             m_recv_msg_len; 
  const uint32_t       m_enable = 1;    // flag for enabling reuse-addr or broadcast settings - will never be changed 
  struct sockaddr_in   m_io_sock_addr;  // address for own I/O socket this instance
  struct sockaddr_in   m_wrapper_addr;  // address for interface socket 
  socklen_t            m_socklen = sizeof(m_wrapper_addr); // size of wrapper address 

  struct hostent *     m_server; // server we will connect to via TCP client 

  uint32_t             m_sim_ready;

  // input buffer to read chars from socket 
  char                 m_buffer[4096];

  // command strings  
  char                 endmessage[4096] = "x\0";
  const char *         m_sig_ack = "ACK";
  const char *         m_sig_ready = "READY";
  const char *         m_sig_data = "DATA"; // dummy string to validate logic 

  const char *         m_sig_waiting = "WAIT"; // send to control plane when waiting for init params 

  // Node registry and map 
  std::deque<uint32_t>           available_ns3_ids; // keep track of which node IDs are available, since we will need to translate before adding them 
  std::map<uint32_t, uint32_t>   id_map; // mapping from "Aimsun ID" (key) to "ns-3 ID" (value) 

  uint32_t            sim_time_step; // keep track of how long 1 sim step is for various timeouts

  uint32_t            m_data_reporting_fidelity; // how much data to push through listener socket (1 = minimal, 9 = maximum)

// private: 
//   none 

}; 

} // namespace ns3


#endif