/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Authored by: Maxwell McManus <maxwell.mcmanus@nrel.gov> (NREL)
 */

#ifndef V2X_TAG_H
#define V2X_TAH_H


#include "ns3/tag.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"



namespace ns3 {

 // define this class in a public header
class v2xTag : public Tag
{
public:
  v2xTag();
  virtual ~v2xTag();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  Time     GetSendTime  (void);
  uint32_t GetSenderId  (void);
  Vector   GetSenderLoc (void);
  uint32_t GetPacketType(void);

  void SetSendTime (Time t);
  void SetSenderId (uint32_t senderId);
  void SetSenderLoc (Vector loc);
  void SetPacketType(uint32_t packetType);


private:
  Time m_sendTime;
  uint32_t m_senderId;
  Vector m_senderLoc;
  uint32_t m_packetType;

  // std::chrono::steady_clock::time_point sendtime = std::chrono::steady_clock::now(); // use this line when packet is tagged right before "socket->send(tagged_pkt)"
};
}
#endif
