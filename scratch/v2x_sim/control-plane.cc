/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Authored by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 * 
 */

#include "ns3/vector.h"
#include "ns3/log.h"
#include "control-plane.h" // needed for access to packet operations 


namespace ns3 {



NS_LOG_COMPONENT_DEFINE ("HelicsCtrlPlane");
NS_OBJECT_ENSURE_REGISTERED (HelicsCtrlPlane);


TypeId
HelicsCtrlPlane::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::HelicsCtrlPlane")
    .SetParent<Object> ()
    .SetGroupName("Helics") // existing group, but both unrelated and insignificant 
    .AddConstructor<HelicsCtrlPlane> ()
  ;
  return tid;
}

// constructor/destructor: 

HelicsCtrlPlane::HelicsCtrlPlane () {
  NS_LOG_FUNCTION (this);
}


HelicsCtrlPlane::~HelicsCtrlPlane()
{
  NS_LOG_FUNCTION (this);

  // close socket, arg = 2 for "stop both reception and transmission"
  shutdown(m_sockfd, 2);
}


void 
HelicsCtrlPlane::Setup(const uint32_t inst_idx, const uint32_t io_sock_port, const uint32_t wrapper_port, const uint32_t reporting)
{
  NS_LOG_FUNCTION (this);

  // need to remember which instance this is 
  m_inst_idx = inst_idx; 
  m_data_reporting_fidelity = reporting;

  // declare TCP socket client for simulation I/O - wrapper provides server 

  m_io_sock_port = io_sock_port;
  m_wrapper_port = wrapper_port;

  // create EXTERNAL socket file descriptor 
  if ( (m_sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 ) {
      perror("socket creation failed");
      exit(EXIT_FAILURE);
  }
  // // set reuse-addr or broadcast flags, if needed - not necessary to leave up by default 
  // setsockopt(io_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(uint32_t));
  // setsockopt(io_sockfd, SOL_SOCKET, SO_BROADCAST, &enable, sizeof(uint32_t));
  
  memset(&m_io_sock_addr, 0, sizeof(m_io_sock_addr));
  memset(&m_wrapper_addr, 0, sizeof(m_wrapper_addr));
      
  // Filling "client" information - instance 
  m_io_sock_addr.sin_family    = AF_INET; // IPv4
  m_io_sock_addr.sin_addr.s_addr = INADDR_ANY; // IP address
  m_io_sock_addr.sin_port = htons(m_io_sock_port);

  // Known "server" information - wrapper
  m_wrapper_addr.sin_family    = AF_INET; // IPv4
  m_wrapper_addr.sin_addr.s_addr = INADDR_ANY; // IP address
  m_wrapper_addr.sin_port = htons(m_wrapper_port); 

  if (connect(m_sockfd, (struct sockaddr *) &m_wrapper_addr, sizeof(m_wrapper_addr)) < 0) {
      perror("connect failed");
      exit(EXIT_FAILURE);
  }
    
  // --> no need to bind since we are TCP client - bind is implicit 
  // // bind I/O socket with server address (provided by cmd line args)
  // if ( bind(m_sockfd, (const struct sockaddr *)&m_io_sock_addr, sizeof(m_io_sock_addr)) < 0 ) {
  //     perror("bind failed");
  //     exit(EXIT_FAILURE);
  // }
  // else {
  //   std::cout << "Bound to " << m_io_sock_port << " at " << m_sockfd << std::endl;
  // }

  // initialize sim_ready to 0 -> will set to 1 when all nodes are active and ready for co-simulation 
  m_sim_ready = 0;

}

void 
HelicsCtrlPlane::ReportData (std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> > &packet_reception) 
/**
 * Send data to wrapper (as byte string) 
 * 
 * Data should be split into two structures: 
 * - received packets: for each transmitted packet, provide a list of nodes which correctly received it 
 * - dropped packets:  for each transmitted packet, provide a list of nodes which did not correctly receive it before expiration
 * 
 * NOTE - users should define own reporting scheme based on target application 
 * 
*/
{
  // aggregate data and serialize into bytestring 
  std::string data = "DATA=";
  std::string good_rx = ""; // declare empty string so we can append easily 

  std::map< std::tuple<uint32_t, uint32_t, uint32_t>, std::vector<uint32_t> >::iterator collector; 

  std::vector<uint32_t>::iterator node_iter;

  std::ostringstream node_stream; 

  // first, iterate through successful RX events 
  // --> collector->first = packet key (TX_time, TX_id, packet_type)
  // --> collector->second = vector of node ID's which correctly received the packet 

  for (collector = packet_reception.begin(); collector != packet_reception.end(); collector++) {
      // log packet info ("ID:KEY:RX,RX,RX,...,")
      good_rx.append(std::to_string(std::get<1>(collector->first))); // TX ID <ID> 

      good_rx.append(":"); // field delimiter

      good_rx.append(std::to_string(std::get<2>(collector->first))); // packet type <key>

      good_rx.append(":"); // field delimiter

      // good_rx.append(std::to_string(std::get<0>(collector->first))); // transmit time <time>

      // good_rx.append(":"); // field delimiter

      if (collector->second.empty()) {
        good_rx.append("None");
      }
      else {
        // loop through all nodes in vector and compile into string-type 
        for (node_iter = collector->second.begin(); node_iter != collector->second.end(); node_iter++) {
          node_stream << (*node_iter) << ",";
        }
        good_rx.append(node_stream.str());

        node_stream.str(std::string(""));

        // remove trailing delimiter ","
        good_rx.pop_back(); 

        // delete vector contents now that we have copied them 
        // --> also done at control plane, but something isn't working 
        collector->second.clear();
      }

      good_rx.append("+"); // entry delimiter
  }

  // empty out stringstream 
  node_stream.str(std::string(""));

  // std::cout << "Node stream: " << node_stream.str() << std::endl;

  // std::cout << "Reported data length: " << good_rx.length() << " bytes" << std::endl;
  NS_LOG_INFO("Reported data length: " << good_rx.length() << " bytes");

  // remove trailing delimiter "+"
  if (good_rx.length() > 0) {
    good_rx.pop_back(); 
    data.append(good_rx);   
  }
  else {
    // no data has been reported 
    // data.append("None");
  }


  // may not be big enough socket size 
  // std::cout << "Reported data: " << data << std::endl;
  // std::cout << "Reported data size: " << sizeof(data) << " bytes" << std::endl;
  NS_LOG_INFO("Reported data: " << data);
  NS_LOG_INFO("Reported data size: " << sizeof(data) << " bytes");

  const char * serialized_data = data.c_str();

  // final format: "DATA=TX1:KEY:RX2,RX3,...+TX2:KEY:RX1,RX3,...+TX3:KEY:RX1,RX2,..."

  send(m_sockfd, (const char *) serialized_data, strlen(serialized_data), 0); // args: socket FD, data to send, size of data to send, flags

  // flush collected data to prevent overflows - not done here anymore!
  // collected_data.erase(collected_data.begin(), collected_data.end());

  return;
}


void 
HelicsCtrlPlane::ReportAbbrevData (uint32_t pkts_sent, uint32_t pkts_received_valid) 
/**
 * Send abbreviated data to wrapper (as byte string) 
 * 
 * 
 * NOTE - we only report total sent and total received packets 
 * 
*/
{
  // aggregate data and serialize into bytestring 
  std::string data = "DATA=";

  data.append(std::to_string(pkts_received_valid));
  data.append("+");
  data.append(std::to_string(pkts_sent));

  NS_LOG_INFO("Reported data: " << data);
  NS_LOG_INFO("Reported data size: " << sizeof(data) << " bytes");

  const char * serialized_data = data.c_str();

  // final format: "DATA=num_pkts_sent+num_pkts_rvcd"

  send(m_sockfd, (const char *) serialized_data, strlen(serialized_data), 0); // args: socket FD, data to send, size of data to send, flags

  return;
}


void 
HelicsCtrlPlane::SendAck () 
/**
 * Send ACK to listener thread 
 * 
*/
{
  // std::cout << "Reporting ACK to " << m_wrapper_port << std::endl;
  NS_LOG_INFO("Reporting ACK to " << m_wrapper_port);

  send(m_sockfd, (const char *)m_sig_ack, strlen(m_sig_ack), 0);


  return;
}


void 
HelicsCtrlPlane::SendReady () 
/**
 * Send ACK to listener thread 
 * 
*/
{
  // std::cout << "Reporting READY to " << m_wrapper_port << std::endl;
  NS_LOG_INFO("Reporting READY to " << m_wrapper_port);

  send(m_sockfd, (const char *)m_sig_ready, strlen(m_sig_ready), 0);

  return;
}


uint32_t 
HelicsCtrlPlane::WaitForAck () 
/**
 * Generic function for synchronization with wrapper 
 * 
 * Hold simulator execution until HELICS interface returns ACK 
 * 
*/
{

  m_recv_msg_len = recv(m_sockfd, (char *)m_buffer, strlen(m_buffer), 0);

  // std::cout << "Instance " << m_inst_idx << " received message: [" << m_buffer << "]" << std::endl;
  NS_LOG_INFO("Instance " << m_inst_idx << " received message: [" << m_buffer << "]");

  if (strcmp(m_buffer,m_sig_ack) == 0) {
      // ACK received successfully
      return 1;
  } 
  else {
    // string received was NOT ACK - unexpected --> cast received buffer to string in case we want to handle I/O errors like this 
    std::string buffstring(m_buffer);

  }

  return 1; 
}



InitializationData
HelicsCtrlPlane::GetInitParams()
/**
 * Hold simulator startup until all simulation setup params are received 
 * 
 * Init param structure is known, so we will parse string directly and return completed "struct InitializationData"
 * 
 * Expected format: simtime/timestep/id,id,.../x:y,x:y,.../! == "30/100/0,1,2,3/0:0,1:1,2:2,3:3/!"
 * 
 * NOTE: this will need to match formatting based on target application and vehicle simulator reporting 
*/
{
  // send signal to wrapper that this instance is finished compiling and is ready to process init params 
  send(m_sockfd, (const char *)m_sig_waiting, strlen(m_sig_waiting), 0);
  
  // std::cout << "Control plane for instance " << m_inst_idx << " waiting for init params..." << std::endl;

  // make sure the buffer is cleared 
  memset(&m_buffer, 0, sizeof(m_buffer));

  // std::cout << "Buffer length in ns-3 code: " << strlen(m_buffer) << std::endl; --> buffer length is 0...
 
  recv(m_sockfd, (char *)m_buffer, sizeof(m_buffer), 0);  


  // cast received message to string for further processing (more intuitive at least, if not more portable)
  std::string dataString(m_buffer);

  // std::cout << "Instance " << m_inst_idx << " received message: '" << dataString << "' of size " << dataString.length() << std::endl;
  NS_LOG_INFO("Instance " << m_inst_idx << " received message: '" << dataString << "' of size " << dataString.length());

  // // remove padding 
  // if (dataString.find("/") != std::string::npos) {
  //   pos = dataString.find("/");
  //   runtime = std::stoi(dataString.substr(0,pos));
  //   dataString.erase(0,pos+1);
  // }
  // else {
  //   std::cout << "\n\t> Error parsing simulation duration!\n" << std::endl;
  // }

  uint32_t runtime; 
  uint32_t timestep_ms;
  std::vector<uint32_t> ids_vector;
  std::vector<Vector> locs_vector;

  // placeholders for parsing string values - easier if we can subsect full string first 
  std::string ids_string;
  std::string locs_string;
  std::string sub_loc_string;
  Vector temp_loc_vec;

  size_t pos = 0;

  // destructive parsing - each delimited ("/") field is removed from dataString after it is copied (more straightforward than preserving original string)
  // --> expected format: simtime/timestep/id,id,.../x:y,x:y,.../! == "30/100/0,1,2,3/0:0,1:1,2:2,3:3/!"

  // --> parse simtime 
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    runtime = std::stoi(dataString.substr(0,pos));
    dataString.erase(0,pos+1);
  }
  else {
    // std::cout << "\n\t> Error parsing simulation duration!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing simulation duration!\n");
  }
  // std::cout << "\nInstance " << m_inst_idx << " parsed sim duration: " << runtime << " \nRemaining datastring: '" << dataString << "'" << std::endl;
  NS_LOG_INFO("\nInstance " << m_inst_idx << " parsed sim duration: " << runtime << " \nRemaining datastring: '" << dataString << "'");

  // --> parse timestep (dataString is now: "timestep/id,id,.../x:y,x:y,.../!")
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    timestep_ms = std::stoi(dataString.substr(0,pos));
    dataString.erase(0,pos+1);
  }
  else {
    // std::cout << "\n\t> Error parsing simulation timestep!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing simulation timestep!\n");
  }

  // std::cout << "\nInstance " << m_inst_idx << " parsed timestep: " << timestep_ms << " \nRemaining datastring: '" << dataString << "'" << std::endl;
  NS_LOG_INFO("\nInstance " << m_inst_idx << " parsed timestep: " << timestep_ms << " \nRemaining datastring: '" << dataString << "'");

  // --> parse node IDs (dataString is now: "id,id,.../x:y,x:y,.../!")
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    ids_string = dataString.substr(0,pos);
    dataString.erase(0,pos+1);
    // parse ID string to vector of int 
    while (ids_string.length() != 0) {
      // will find delimiter on every entry except the last one, which can be handled directly 
      if (ids_string.find(",") != std::string::npos) {
        pos = ids_string.find(",");
        ids_vector.push_back(std::stoi(ids_string.substr(0,pos)));
        ids_string.erase(0,pos+1);
      }
      else {
        // string should only consist of last remaining ID with no delimiter  
        ids_vector.push_back(std::stoi(ids_string)); 
        ids_string.erase(0,std::string::npos); // make sure string is empty 
      }
    }
  }
  else {
    // std::cout << "\n\t> Error parsing initial node IDs!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing initial node IDs!\n");
  }

  // std::cout << "\nInstance " << m_inst_idx << " parsed init nodes: " << ids_string << " \nRemaining datastring: '" << dataString << "'" << std::endl;
  NS_LOG_INFO("\nInstance " << m_inst_idx << " parsed init nodes: " << ids_string << " \nRemaining datastring: '" << dataString << "'");

  // --> parse node init locations (dataString is now: "x:y,x:y,.../!")
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    locs_string = dataString.substr(0,pos);
    dataString.erase(0,pos+1);
    // parse loc string to std::vector of ns3::Vector  

    // parse ID string to vector of int 
    while (locs_string.length() != 0) {
      // will find delimiter on every entry except the last one, which can be handled directly 
      if (locs_string.find(",") != std::string::npos) {
        pos = locs_string.find(",");
        // extract location value from front of locs_string 
        sub_loc_string = locs_string.substr(0,pos); // get single "x-coord:y-coord"
        // remove extracted value from locs_string - **need to do this first to re-use "pos", otherwise declare another int 
        locs_string.erase(0,pos+1); 

        // get x-coord
        pos = sub_loc_string.find(":");
        temp_loc_vec.x = stof(sub_loc_string.substr(0,pos)); 
        
        // remove "x-coord:" from loc string "x-coord:y-coord" - just left with y-coord
        sub_loc_string.erase(0,pos+1); 
        temp_loc_vec.y = stof(sub_loc_string); 

        locs_vector.push_back(temp_loc_vec);
      }
      else {
        // remaining locs_string should just be "x-coord:y-coord" - follow same logic as processing "sub_loc_string"
        pos = locs_string.find(":");
        temp_loc_vec.x = stof(locs_string.substr(0,pos)); 
        locs_string.erase(0,pos+1); 
        temp_loc_vec.y = stof(locs_string); 
        locs_string.erase(0,std::string::npos); // make sure string is empty 

        locs_vector.push_back(temp_loc_vec);
      }
    }
  }
  else {
    // std::cout << "\n\t> Error parsing initial node locations!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing initial node locations!\n");
  }

  // std::cout << "\nInstance " << m_inst_idx << " parsed node locations: " << locs_string << " \nRemaining datastring: '" << dataString << "'" << std::endl;
  NS_LOG_INFO("\nInstance " << m_inst_idx << " parsed node locations: " << locs_string << " \nRemaining datastring: '" << dataString << "'");

  // can check if parsing worked by printing remaining dataString - should be only "/!" 
  pos = dataString.find("/");
  dataString.erase(0,pos+1);
  // std::cout << "\nData string processed down to: " << dataString << std::endl;

  // initialize return struct in-place with parsed values 
  InitializationData initData = {.sim_runtime = runtime, .sim_timestep_ms = timestep_ms, .node_ids = ids_vector, .node_init_locs = locs_vector};

  // std::cout << "\nInstance " << m_inst_idx << " finished parsing init string!" << std::endl;

  return initData;
}






UpdateData
HelicsCtrlPlane::WaitForData () 
/**
 * Hold simulator execution until HELICS interface gives state update 
 * 
 * Dynamic topology update structure is known, so we will parse string directly and return completed "struct DynTopo"
 * 
 * Expected format: id,id,.../x:y,x:y,.../msgSet,msgSet,.../! == "0,1,2,3/0:0,1:1,2:2,3:3/00,1,222,3/!"
 * 
 * NOTE: this will need to match formatting based on target application and vehicle simulator reporting 
 * 
*/
{
  // std::cout << "Control plane for instance " << m_inst_idx << " waiting for data" << std::endl;

  m_recv_msg_len = recv(m_sockfd, (char *)m_buffer, sizeof(m_buffer), 0);


  // std::cout << "Instance " << m_inst_idx << " received update message: '" << m_buffer << "'" << std::endl;
  NS_LOG_INFO("Instance " << m_inst_idx << " received update message: '" << m_buffer << "'");

  // cast received message to string for further processing (more intuitive at least, if not more portable)
  std::string dataString(m_buffer);

  std::vector<uint32_t> ids_vector;
  std::vector<Vector> locs_vector;
  std::vector<std::string> msg_set_vector;

  // placeholders for parsing string values - easier if we can subsect full string first 
  std::string ids_string;
  std::string locs_string;
  std::string sub_loc_string;
  Vector temp_loc_vec;
  std::string msg_set_string;
  std::string per_node_msgs;

  size_t pos = 0;

  // destructive parsing - each delimited ("/") field is removed from dataString after it is copied (more straightforward than preserving original string)
  // --> expected format: id,id,.../x:y,x:y,.../msgSet,msgSet,.../! == "0,1,2,3/0:0,1:1,2:2,3:3/00,1,222,3/!"


  // --> parse node IDs (dataString is now: "id,id,.../x:y,x:y,.../msgSet,msgSet,.../!")
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    ids_string = dataString.substr(0,pos);
    dataString.erase(0,pos+1);
    
    // std::cout << "\nInstance " << m_inst_idx << " parsing node IDs: " << ids_string << "... \nRemaining datastring: '" << dataString << "'" << std::endl;

    // parse ID string to vector of int 
    while (ids_string.length() != 0) {
      // will find delimiter on every entry except the last one, which can be handled directly 
      if (ids_string.find(",") != std::string::npos) {
        pos = ids_string.find(",");
        ids_vector.push_back(std::stoi(ids_string.substr(0,pos)));
        ids_string.erase(0,pos+1);
      }
      else {
        ids_vector.push_back(std::stoi(ids_string)); // string should only consist of last remaining ID with no delimiter  
        ids_string.erase(0,std::string::npos);
      }
    }
  }
  else {
    // std::cout << "\n\t> Error parsing initial node IDs!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing initial node IDs!\n");
  }



  // --> parse node init locations (dataString is now: "x:y,x:y,.../msgSet,msgSet,.../!")
  if (dataString.find("/") != std::string::npos) {
    pos = dataString.find("/");
    locs_string = dataString.substr(0,pos);
    dataString.erase(0,pos+1);
    // parse loc string to std::vector of ns3::Vector  

    // std::cout << "\nInstance " << m_inst_idx << " parsing node locations: " << locs_string << "... \nRemaining datastring: '" << dataString << "'" << std::endl;

    // parse ID string to vector of int 
    while (locs_string.length() != 0) {
      // will find delimiter on every entry except the last one, which can be handled directly 
      if (locs_string.find(",") != std::string::npos) {
        pos = locs_string.find(",");
        // extract location value from front of locs_string 
        sub_loc_string = locs_string.substr(0,pos); // get single "x-coord:y-coord"
        // remove extracted value from locs_string - **need to do this first to re-use "pos", otherwise declare another int 
        locs_string.erase(0,pos+1); 

        // get x-coord
        pos = sub_loc_string.find(":");
        temp_loc_vec.x = stof(sub_loc_string.substr(0,pos)); 
        
        // remove "x-coord:" from loc string "x-coord:y-coord" - just left with y-coord
        sub_loc_string.erase(0,pos+1); 
        temp_loc_vec.y = stof(sub_loc_string); 

        locs_vector.push_back(temp_loc_vec);
      }
      else {
        // remaining locs_string should just be "x-coord:y-coord" - follow same logic as processing "sub_loc_string"
        pos = locs_string.find(":");
        temp_loc_vec.x = stof(locs_string.substr(0,pos)); 
        locs_string.erase(0,pos+1); 
        temp_loc_vec.y = stof(locs_string); 

        locs_vector.push_back(temp_loc_vec);

        locs_string.erase(0,std::string::npos); 
      }
    }
  }
  else {
    // std::cout << "\n\t> Error parsing initial node locations!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing initial node locations!\n");
  }


  // --> parse message sets as vector of strings (dataString is now: "msgSet,msgSet,.../!")
  if (dataString.find("/") != std::string::npos) {
    
    pos = dataString.find("/");
    msg_set_string = dataString.substr(0,pos);
    dataString.erase(0,pos+1);

    // std::cout << "\nInstance " << m_inst_idx << " parsing message sets: " << locs_string << "... \nRemaining datastring: '" << dataString << "'" << std::endl;
    
    // parse message string to vector of strings  
    while (msg_set_string.length() != 0) {
      // will find delimiter on every entry except the last one, which can be handled directly 
      if (msg_set_string.find(",") != std::string::npos) {
        pos = msg_set_string.find(",");
        // extract series of message keys from front of msg_set_string 
        per_node_msgs = msg_set_string.substr(0,pos); // get single "x-coord:y-coord"
        // remove extracted value from msg_set_string - **need to do this first to re-use "pos", otherwise declare another int 
        msg_set_string.erase(0,pos+1); 
        msg_set_vector.push_back(per_node_msgs);

      }
      else {
        // remaining "msg_set_string" should just be string of message keys for last node - no further processing 
        msg_set_vector.push_back(msg_set_string);
        msg_set_string.erase(0,std::string::npos);
      }
    }
  }
  else {
    // std::cout << "\n\t> Error parsing message set!\n" << std::endl;
    NS_LOG_DEBUG("\n\t> Error parsing message set!\n");
  }

  // can check if parsing worked by printing remaining dataString - should be only "/!" 
  pos = dataString.find("/");
  dataString.erase(0,pos+1);
  // std::cout << "\nData string processed down to: " << dataString << std::endl;

  // initialize return struct in-place with parsed values 
  UpdateData update = {.node_ids = ids_vector, .loc_updates = locs_vector, .new_msgs = msg_set_vector};

  return update;
}


} // namespace ns3 

