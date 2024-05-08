/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Authored by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 * 
 * NOTE - this code can be used with any >=C++17/g++17 - this was commented out b/c C++11/g++11 was used during development 
 */


// #ifndef V2X_CUSTOM_PACKETS
// #define V2X_CUSTOM_PACKETS


// #include "ns3/network-module.h" // needed for access to packet operations 




// namespace ns3 {

// class PacketLib 
// {
// public: 
//   // define all packet types that can be triggered by scenarios in Aimsun - we need Aimsun to select TX'd packets by type based on scenario 
//   uint32_t lenCAM = 190;   
//   uint32_t lenDENM = 800;
//   uint32_t lenBSM1 = 120;  
//   uint32_t lenBSM2 = 320;
//   uint32_t lenBSM3 = 520;  
// //uint32_t lenBSM4...

//   // packets initialized this way have zero-filled payload (uninitialized), but that's fine - we will use tags instead of payload 
//   Ptr<Packet> packetCam  = Create<Packet>(lenCAM);   // pkt_key = "cam0", pkt_idx = 0 (pkt_key should always have len=4 for convenience)
//   Ptr<Packet> packetDenm = Create<Packet>(lenDENM);  // pkt_key = "denm", pkt_idx  = 1
//   Ptr<Packet> packetBSM1 = Create<Packet>(lenBSM1);  // pkt_key = "bsm1", pkt_idx  = 2
//   Ptr<Packet> packetBSM2 = Create<Packet>(lenBSM2);  // pkt_key = "bsm2", pkt_idx  = 3
//   Ptr<Packet> packetBSM3 = Create<Packet>(lenBSM3);  // pkt_key = "bsm3", pkt_idx  = 4
// //Ptr<Packet> packetBSM4... -> make sure to add parsing to "helics-control-app" 

// //   std::map<const char *,uint32_t> key_map; // TODO: trouble initializing, don't really want .cc file - will just cascade if/else statements for now
// //   {"cam0", 0},
// //   {"denm", 1},
// //   {"bsm1", 2},
// //   {"bsm2", 3},
// //   {"bsm3", 4}
// // //{"bsm4"...
// //   };


//   std::map<uint32_t, Ptr<Packet> > const key_map = { 
//     {0, packetCam}, 
//     {1, packetDenm}, 
//     {2, packetBSM1},
//     {3, packetBSM2},
//     {4, packetBSM3}
//   };

// };

// // we want one PacketLib object of this to be shared among all instances of HelicsControlApp to prevent unnecessary packet creation 
// // inline PacketLib const packetLib; --> cannot use with C++>17
// extern PacketLib const packetLib; 


// } 


// #endif