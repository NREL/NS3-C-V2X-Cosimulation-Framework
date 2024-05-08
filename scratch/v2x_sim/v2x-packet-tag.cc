/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Authored by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 */



#include "ns3/tag.h"
#include "ns3/log.h"
#include "ns3/vector.h"
#include "ns3/simulator.h"
#include "v2x-packet-tag.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE("v2xTag");
NS_OBJECT_ENSURE_REGISTERED(v2xTag);


// constructors 

v2xTag::v2xTag() {
  // m_sendTime = Simulator::Now(); // should be overwritten right before sending
  m_senderId = -1; 
  m_senderLoc = Vector(0.0,0.0,0.0);
}

// v2xTag::v2xTag(uint32_t senderId) {
//   m_sendTime = Simulator::Now(); // should be overwritten right before sending
//   m_senderId = senderId; 
//   m_senderLoc = Vector(0.0,0.0,0.0);
// }


// destructor

v2xTag::~v2xTag() {
  // nothing to destroy manually
}


// constituent functions

TypeId
v2xTag::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::v2xTag")
  .SetParent<Tag> ()
  .AddConstructor<v2xTag> ();
  // not sure if attributes are needed - may help compiler, but should not impact runtime behaviors
  return tid;
}

TypeId 
v2xTag::GetInstanceTypeId (void) const
{
  return v2xTag::GetTypeId ();
}

uint32_t 
v2xTag::GetSerializedSize (void) const
{
  return sizeof(uint32_t) + sizeof(Time) + sizeof(Vector);
}

void 
v2xTag::Serialize (TagBuffer i) const // needs to match order of ::Deserialize()
{
  i.WriteDouble (m_sendTime.GetDouble());
  i.WriteU16 (m_senderId);  // safe to assume we will have <65000 nodes per simulation 
  i.WriteU8 (m_packetType); // max value of 8 bit int is 255, we will not need that many packet keys 
  i.WriteDouble (m_senderLoc.x);
  i.WriteDouble (m_senderLoc.y);
  // std::cout << "Serialized: " << "x = [" << m_senderLoc.x << "], y = [" << m_senderLoc.y << "], z = [" << m_senderLoc.z << "]" << std::endl;
  // printf("\nSerialized: time = [%.2f], id = [%d], x = [%.2f], y = [%.2f], z = [%.2f]", m_sendTime.GetDouble(), m_senderId,m_senderLoc.x, m_senderLoc.y, m_senderLoc.z);
}

void 
v2xTag::Deserialize (TagBuffer i) // needs to match order of ::Serialize()
{
  m_sendTime = Time::FromDouble(i.ReadDouble(), Time::NS);
  m_senderId = i.ReadU16 ();
  m_packetType = i.ReadU8 ();
  m_senderLoc.x = i.ReadDouble ();
  m_senderLoc.y = i.ReadDouble ();
  // fix this to send less data in tag - running into buffer problems 
  m_senderLoc.z = 1.5; // elevation will never change significantly - Aimsun reports coords in 2D anyways 
  // printf("\nDeserialized: time = [%.2f], id = [%d], x = [%.2f], y = [%.2f], z = [%.2f]", m_sendTime.GetDouble(), m_senderId,m_senderLoc.x, m_senderLoc.y, m_senderLoc.z); 
}

void 
v2xTag::Print (std::ostream &os) const
{
  os << "Node " << (uint32_t)m_senderId << " sent a packet at " << (Time)m_sendTime << " from " << (Vector)m_senderLoc;
}

Time 
v2xTag::GetSendTime (void) 
{
  return m_sendTime;
}


uint32_t 
v2xTag::GetSenderId (void) 
{
  return m_senderId;
}

Vector 
v2xTag::GetSenderLoc (void) 
{
  return m_senderLoc;
}

uint32_t 
v2xTag::GetPacketType (void) 
{
  return m_packetType;
}

void 
v2xTag::SetSendTime (Time t) 
{
  m_sendTime = t;
}

void 
v2xTag::SetSenderId (uint32_t senderId)
{
  m_senderId = senderId;
}

void 
v2xTag::SetSenderLoc (Vector loc)
{
  m_senderLoc = loc;
}

void 
v2xTag::SetPacketType (uint32_t packetType) 
{
  m_packetType = packetType;
}




}
