/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   Author: Marco Miozzo <marco.miozzo@cttc.es>
*           Nicola Baldo  <nbaldo@cttc.es>
*
*   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
*                         Sourjya Dutta <sdutta@nyu.edu>
*                         Russell Ford <russell.ford@nyu.edu>
*                         Menglei Zhang <menglei@nyu.edu>
*/


#include "ns3/point-to-point-module.h"
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/packet.h>
#include <ns3/tag.h>
#include <ns3/queue-size.h>
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/traffic-control-module.h"
#include <cstdio>
/*#include <ns3/lte-helper.h>
#include <ns3/epc-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/lte-module.h>*/

//#include "ns3/gtk-config-store.h"


/**
 * A script to simulate the DOWNLINK TCP data over mmWave links
 * with the mmWave devices and the LTE EPC.
 */
NS_LOG_COMPONENT_DEFINE ("mmWaveTCPExample");


using namespace ns3;
using namespace mmwave;

Ptr<PacketSink> sink;                 // packetsink application 
Ptr<PacketSink> sink2;
Ptr<PacketSink> sink3;
Ptr<PacketSink> sink4;
Ptr<PacketSink> sink5;                 // packetsink application 
Ptr<PacketSink> sink6;
Ptr<PacketSink> sink7;
Ptr<PacketSink> sink8;
Ptr<PacketSink> sink9;                 // packetsink application 
Ptr<PacketSink> sink10;
Ptr<PacketSink> sink11;
Ptr<PacketSink> sink12;
Ptr<PacketSink> sink13;                 // packetsink application 
Ptr<PacketSink> sink14;
Ptr<PacketSink> sink15;
Ptr<PacketSink> sink16;
uint64_t lastTotalRx = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx2 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx3 = 0;  
uint64_t lastTotalRx4 = 0;  
uint64_t lastTotalRx5 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx6 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx7 = 0;  
uint64_t lastTotalRx8 = 0;  
uint64_t lastTotalRx9 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx10 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx11 = 0;  
uint64_t lastTotalRx12 = 0;  
uint64_t lastTotalRx13 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx14 = 0;             /* The value of the last total received bytes */
uint64_t lastTotalRx15 = 0;  
uint64_t lastTotalRx16 = 0;  

Ptr<RateErrorModel> em;

NodeContainer ueNodes;

PointToPointHelper p2ph;

static double cur;
static double curPass;

int version1=1;
int version2=2;
int start_flag=-1;

double fairnessIndex = 0, xSum = 0, x2Sum = 0;

 AsciiTraceHelper asciiTraceHelper1;
 std::string g_path="abc.txt";

Ptr<OutputStreamWrapper> stream5;



// Az file Wireless/wifi-tcp.cc.
//Throughpu, when used in the context of communication networks, such as Ethernet or packet radio, throughput or network throughput is the rate of successful message delivery over a communication channel. (per second)
void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
  
}

void
CalculateThroughput2 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink2->GetTotalRx () - lastTotalRx2) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx2 = sink2->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput2);
  
}


void
CalculateThroughput3 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink3->GetTotalRx () - lastTotalRx3) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx3 = sink3->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput3);
  
}



void
CalculateThroughput4 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink4->GetTotalRx () - lastTotalRx4) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx4 = sink4->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput4);
  
}


void
CalculateThroughput5 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink5->GetTotalRx () - lastTotalRx5) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx5 = sink5->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput5);
  
}

void
CalculateThroughput6 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink6->GetTotalRx () - lastTotalRx6) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx6 = sink6->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput6);
  
}


void
CalculateThroughput7 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink7->GetTotalRx () - lastTotalRx7) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx7 = sink7->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput7);
  
}



void
CalculateThroughput8 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink8->GetTotalRx () - lastTotalRx8) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx8 = sink8->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput8);
  
}


void
CalculateThroughput9 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink9->GetTotalRx () - lastTotalRx9) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx9 = sink9->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput9);
  
}


void
CalculateThroughput10 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink10->GetTotalRx () - lastTotalRx10) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx10 = sink10->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput10);
  
}

void
CalculateThroughput11 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink11->GetTotalRx () - lastTotalRx11) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx11 = sink11->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput11);
  
}

void
CalculateThroughput12 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink12->GetTotalRx () - lastTotalRx12) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx12 = sink12->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput12);
  
}


void
CalculateThroughput13 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink13->GetTotalRx () - lastTotalRx13) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx13 = sink13->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput13);
  
}

void
CalculateThroughput14 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink14->GetTotalRx () - lastTotalRx14) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx14 = sink14->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput14);
  
}


void
CalculateThroughput15 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink15->GetTotalRx () - lastTotalRx15) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx15 = sink15->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput15);
  
}

void
CalculateThroughput16 ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */


   
   cur = (sink16->GetTotalRx () - lastTotalRx16) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;

curPass =  cur * 1e6;


//Tcpfbtcp t1;
//t1.setThreshold(curPass);


//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx16 = sink16->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput16);
  
}


/*
void
errorrate ()
{

// OK -> 1 drop.
  if (Simulator::Now ().GetSeconds () == 2.2 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000002));
}
else if ( Simulator::Now ().GetSeconds () > 2.203 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}



  if (Simulator::Now ().GetSeconds () == 4 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000001));
}
else if ( Simulator::Now ().GetSeconds () > 4.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}



//Ok -> 1 drop.
  if (Simulator::Now ().GetSeconds () == 10 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000002));
}
else if ( Simulator::Now ().GetSeconds () > 10.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}


//OK -> 2 drops.
  if (Simulator::Now ().GetSeconds () == 16 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000005));
}
else if ( Simulator::Now ().GetSeconds () > 16.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}


//OK -> 1 drop.
  if (Simulator::Now ().GetSeconds () == 20 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000002));
}
else if ( Simulator::Now ().GetSeconds () > 20.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}



  if (Simulator::Now ().GetSeconds () == 37 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000004));
}
else if ( Simulator::Now ().GetSeconds () > 37.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}



  if (Simulator::Now ().GetSeconds () == 40 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000004));
}
else if ( Simulator::Now ().GetSeconds () > 40.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}



//OK -> 1 drop.
  if (Simulator::Now ().GetSeconds () == 45 ) {
  //NS_LOG_UNCOND ("Congestion happened. ");
  //NS_LOG_UNCOND (em);

  em->SetAttribute ("ErrorRate", DoubleValue (0.0000004));
}
else if ( Simulator::Now ().GetSeconds () > 45.001 ) {
  em->SetAttribute ("ErrorRate", DoubleValue (0));
}





  Simulator::Schedule (MilliSeconds (100), &errorrate);
}

*/


class MyAppTag : public Tag
{
public:
  MyAppTag ()
  {
  }

  MyAppTag (Time sendTs) : m_sendTs (sendTs)
  {
  }

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::MyAppTag")
      .SetParent<Tag> ()
      .AddConstructor<MyAppTag> ();
    return tid;
  }

  virtual TypeId  GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  virtual void  Serialize (TagBuffer i) const
  {
    i.WriteU64 (m_sendTs.GetNanoSeconds ());
  }

  virtual void  Deserialize (TagBuffer i)
  {
    m_sendTs = NanoSeconds (i.ReadU64 ());
  }

  virtual uint32_t  GetSerializedSize () const
  {
    return sizeof (m_sendTs);
  }

  virtual void Print (std::ostream &os) const
  {
    std::cout << m_sendTs;
  }

  Time m_sendTs;
};


class MyApp : public Application
{
public:
  MyApp ();
  virtual ~MyApp ();
  void ChangeDataRate (DataRate rate);
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);



private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_nPackets (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::ChangeDataRate (DataRate rate)
{
  m_dataRate = rate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  MyAppTag tag (Simulator::Now ());

  m_socket->Send (packet);
  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTx ();

    }



}



void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}






//Anvae Safha

void
DevicePacketsInQueueTrace (uint32_t oldValue, uint32_t newValue)
{
  //std::cout << "DevicePacketsInQueue " <<Simulator::Now ().GetSeconds () << "\t" << oldValue << " to " << newValue << std::endl;
}


//TC:Trafic Control
void
TcPacketsInQueueTrace (uint32_t oldValue, uint32_t newValue)
{
  //std::cout << "TcPacketsInQueue " << Simulator::Now ().GetSeconds () << "\t" << oldValue << " to " << newValue << std::endl;
}


void
SojournTimeTrace (Time sojournTime)
{
  std::cout << "Sojourn time " << Simulator::Now ().GetSeconds () << "\t" << sojournTime.ToDouble (Time::MS) << "ms" << std::endl;
}




static void
RttChange (Ptr<OutputStreamWrapper> stream, Time oldRtt, Time newRtt)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldRtt.GetSeconds () << "\t" << newRtt.GetSeconds () << std::endl;
}



static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize () << std::endl;
}





void printStats (FlowMonitorHelper &flowmon_helper, bool perFlowInfo) {
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon_helper.GetClassifier());
  std::string proto;
  Ptr<FlowMonitor> monitor = flowmon_helper.GetMonitor ();
  std::map < FlowId, FlowMonitor::FlowStats > stats = monitor->GetFlowStats();
  double totalTimeReceiving;
  uint64_t totalPacketsReceived, totalPacketsDropped, totalBytesReceived,totalPacketsTransmitted, totalPacketsLost;

  totalBytesReceived = 0, totalPacketsDropped = 0, totalPacketsReceived = 0, totalTimeReceiving = 0,totalPacketsTransmitted = 0, totalPacketsLost=0;
  for (std::map< FlowId, FlowMonitor::FlowStats>::iterator flow = stats.begin(); flow != stats.end(); flow++)
  {
    Ipv4FlowClassifier::FiveTuple  t = classifier->FindFlow(flow->first);
    switch(t.protocol)
     {
     case(6):
         proto = "TCP";
         break;
     case(17):
         proto = "UDP";
         break;
     default:
         exit(1);
     }
     totalBytesReceived += (double) flow->second.rxBytes * 8;
     totalTimeReceiving += flow->second.timeLastRxPacket.GetSeconds ();
     totalPacketsReceived += flow->second.rxPackets;
     totalPacketsDropped += flow->second.txPackets - flow->second.rxPackets;
     totalPacketsTransmitted += flow->second.txPackets;
     totalPacketsLost += flow->second.lostPackets;
     if (perFlowInfo)
     {
       std::cout << "FlowID: " << flow->first << " (" << proto << " "
                 << t.sourceAddress << " / " << t.sourcePort << " --> "
                 << t.destinationAddress << " / " << t.destinationPort << ")" << std::endl;
       std::cout << "  Tx Bytes: " << flow->second.txBytes << std::endl;
       std::cout << "  Rx Bytes: " << flow->second.rxBytes << std::endl;
       std::cout << "  Tx Packets: " << flow->second.txPackets << std::endl;
       std::cout << "  Rx Packets: " << flow->second.rxPackets << std::endl;
       std::cout << "  Time LastRxPacket: " << flow->second.timeLastRxPacket.GetSeconds () << "s" << std::endl;
       std::cout << "  Lost Packets: " << flow->second.lostPackets << std::endl;
       std::cout << "  Pkt Lost Ratio: " << ((double)flow->second.txPackets-(double)flow->second.rxPackets)/(double)flow->second.txPackets << std::endl;
       std::cout << "  Throughput: " << ( ((double)flow->second.rxBytes * 8) / (flow->second.timeLastRxPacket.GetSeconds ()) ) << "bits/s" << std::endl;
       std::cout << "  Mean{Delay}: " << (flow->second.delaySum.GetSeconds()/flow->second.rxPackets) << std::endl;
       std::cout << "  Mean{Jitter}: " << (flow->second.jitterSum.GetSeconds()/(flow->second.rxPackets)) << std::endl;
     }


   }

     std::cout<< "Total throughput of System: "<<
     (totalBytesReceived)/totalTimeReceiving<<" bps "<<std::endl;
     std::cout<<"Total packets transmitted: "<<totalPacketsTransmitted<<std::endl;
     std::cout<<"Total packets received: "<< totalPacketsReceived<<std::endl;
     std::cout<<"Total packets dropped: "<< totalPacketsDropped<<std::endl;
     std::cout<<"Total packets lost: "<< totalPacketsLost<<std::endl;
     std::cout << "Packet Lost Ratio: " << totalPacketsDropped / (double) (totalPacketsReceived + totalPacketsDropped) << std::endl;
     //count=totalPacketsLost;
}





void
ChangeSpeed (Ptr<Node>  n, Vector speed)
{
  n->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (speed);





}


void
getPosition (Ptr<Node>  n)
{
  
      Vector pos = n->GetObject<MobilityModel> ()->GetPosition();

std::cout <<"THE POSITION OF THE NODE IS: " << pos << std::endl;

Simulator::Schedule (MilliSeconds (100), &getPosition, ueNodes.Get (0));
  
}




void
congestionstate (Ptr<OutputStreamWrapper> stream,  TcpSocketState::TcpCongState_t oldvalue, TcpSocketState::TcpCongState_t newvalue)
{
 
  NS_LOG_UNCOND ( Simulator::Now ().GetSeconds () << "\t"  << "The old state is:" <<oldvalue << "\n" << "The new state is: " <<  newvalue  );

//*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldvalue << "\t" << newvalue << std::endl;;
}







int
main (int argc, char *argv[])
{
//	LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveFlexTtiMaxWeightMacScheduler", LOG_LEVEL_DEBUG);

  /*
   * scenario 1: 1 building;
   * scenario 2: 3 building;
   * scenario 3: 6 random located small building, simulate tree and human blockage.
   * */
  ns3::LogComponentEnableAll(ns3::LOG_LEVEL_WARN);
  int scenario = 3;
  std::string transport_prot = "TcpCubic"; //yo
  std::string path_to_res = "results/";
  std::string prefix_file_name = "Camera_Ready_Results";
  bool dcr = true;
  bool sack = true;
  uint16_t cong_var = 3;
  double stopTime = 50;
  double simStopTime = 50;
  bool harqEnabled = true;
  bool rlcAmEnabled = true;
  bool tcp = true;

  CommandLine cmd;
//	cmd.AddValue("numEnb", "Number of eNBs", numEnb);
//	cmd.AddValue("numUe", "Number of UEs per eNB", numUe);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simStopTime);
//	cmd.AddValue("interPacketInterval", "Inter-packet interval [us])", interPacketInterval);
  cmd.AddValue ("harq", "Enable Hybrid ARQ", harqEnabled);
  cmd.AddValue ("DCR", "Enable Selective ACKnowledgements", dcr);
  cmd.AddValue("sack", "Enable or disable SACK option", sack);
  cmd.AddValue ("cong_var", "congestion protocol to be used", cong_var);
  cmd.AddValue ("rlcAm", "Enable RLC-AM", rlcAmEnabled);
  cmd.Parse (argc, argv);

  if(dcr)
  {
    path_to_res = "results/Scenario3/Cubiwood_dcr/sixteen/";   
  }
  
  else
  {
    path_to_res = "results/Scenario3/Cubiwood/sixteen/";   
  } 
 
  freopen((path_to_res + prefix_file_name + "-res.txt").c_str(),"w",stdout);
  
  
  transport_prot = std::string ("ns3::") + transport_prot; //yo
  
  
   g_path=(path_to_res + prefix_file_name + "-throughput.txt").c_str();
   stream5=asciiTraceHelper1.CreateFileStream (g_path);






//RTO
  Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (1)));

//Slow Start Threshold
  //Config::SetDefault ("ns3::TcpSocket::InitialSlowStartThreshold",  UintegerValue (65500));


  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1400));

//	Config::SetDefault ("ns3::PointToPointNetDevice::Mtu", UintegerValue (3000));
//	Config::SetDefault ("ns3::VirtualNetDevice::Mtu", UintegerValue (3000));


  Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(sack));
  Config::SetDefault ("ns3::TcpSocketBase::DCR", BooleanValue (dcr)); 
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (1024 * 1024));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (131072 * 50));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (131072 * 50));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue (1));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (72));
  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveBeamforming::LongTermUpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcAm::PollRetransmitTimer", TimeValue (MilliSeconds (4.0)));
  //Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (MilliSeconds (2.0)));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (4.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (2.5 * 1024 * 1024));




//Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (Tcpfbtcp::GetTypeId ()));






  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();

  mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::BuildingsObstaclePropagationLossModel"));//File 3 vom dar src/mmwave/model.
  mmwaveHelper->Initialize ();
  mmwaveHelper->SetHarqEnabled (true);

  Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  /*
  Ptr<LteHelper> mmwaveHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
*/


//The PGW acts as the interface between the LTE network and other packet data networks, such as the Internet or SIP-based IMS networks.

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

//Dar src/network/utils
   em = CreateObject<RateErrorModel> ();
   em->SetAttribute ("ErrorRate", DoubleValue (0.0000001));


  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);//Node 0 i ke dar container remoteHostContainer hast ra gerefti va dar remoteHost migozarim. 
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);


//Config::SetDefault ("ns3::PfifoFastQueueDisc::MaxSize",QueueSizeValue (QueueSize ("1000p")));
                     
//Nahve tarif pfifo queues .
  TrafficControlHelper tchPfifo;
  tchPfifo.SetRootQueueDisc ("ns3::PfifoFastQueueDisc");

//Nahve tarif CoDel queue.
  TrafficControlHelper tchCoDel;
  tchCoDel.SetRootQueueDisc ("ns3::CoDelQueueDisc");



  // Create the Internet
 
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  internetDevices.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em));

  QueueDiscContainer qdiscs=tchPfifo.Install(internetDevices);
//src/utils/queue.cc
//Safi ke beyne PWG va remote host hast ra trace mikonim.
  Ptr<QueueDisc> q = qdiscs.Get (1);
  q->TraceConnectWithoutContext ("BytesInQueue", MakeCallback (&TcPacketsInQueueTrace));



  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr;
  remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);



  switch (scenario)
    {
    case 1:
      {
// 1.1. A small obstacle.
        Ptr < Building > building;
        building = Create<Building> ();
        building->SetBoundaries (Box (10,12,
                                      2, 3,
                                      0.0, 1.5));


// 1.2. A small obstacle.
        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (20.0,22.0,
                                      6.0, 7,
                                      0.0, 1.5));


// 1.3. A big obstacle.
        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (15.0,25.0,
                                      12.0, 14,
                                      0.0, 15.0));

//15 meters ==> a building.

        break;
      }
    case 2:
      {


// 2.1. A small obstacle.
        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (10,12,
                                      2, 3,
                                      0.0, 1.5));


//1.5 meters ==> a human or a tree.

// 2.2. A small obstacle.
        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (20.0,22.0,
                                      6.0, 7,
                                      0.0, 1.5));



// 2.3. A small obstacle.

        Ptr < Building > building3;
        building3 = Create<Building> ();
        building3->SetBoundaries (Box (5.0,7.0,
                                      8.0, 9,
                                      0.0, 1.5));




// 2.4. A small obstacle.

        Ptr < Building > building4;
        building4 = Create<Building> ();
        building4->SetBoundaries (Box (15.0,17.0,
                                       9.0, 10.0,
                                       0.0, 1.5));




// 2.5. An average obstacle.

        Ptr < Building > building5;
        building5 = Create<Building> ();
        building5->SetBoundaries (Box (13.0,20.0,
                                       6.0, 8.0,
                                       0.0, 3.0));

// 2.6. An average obstacle.

        Ptr < Building > building6;
        building6 = Create<Building> ();
        building6->SetBoundaries (Box (18.0,25.0,
                                       14.0, 16.0,
                                       0.0, 3.0));


      }
    case 3:
      {

/*
// 3.1 A small obstacle.
        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (38.0,38.5,
                                       5.0, 5.5,
                                       0.0, 1.5));
// 3.2 A small obstacle.
        Ptr < Building > building2;f
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (40.0,40.5,
                                       2.0, 2.5,
                                       0.0, 1.5));
*/


//3.1 The first tree.
//        Ptr < Building > building1;
//        building1 = Create<Building> ();
//       building1->SetBoundaries (Box (49.0,49.5,
//                                      2.0, 2.5,
//                                      0.0, 3));

/*
//3.1 The first tree.
        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (1.5,2,
                                       67.0, 67.5,
                                       0.0, 10));

//3.2 The second tree.
        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (3.5,4,
                                       67.0, 67.5,
                                       0.0, 10));


//3.3 The third tree.
        Ptr < Building > building3;
        building3 = Create<Building> ();
        building3->SetBoundaries (Box (5.5,6,
                                       67.0, 67.5,
                                       0.0, 10));


//3.4 The fourth tree.
        Ptr < Building > building4;
        building4 = Create<Building> ();
        building4->SetBoundaries (Box (7.5,8.0,
                                       67.0, 67.5,
                                       0.0, 10));
*/






//3.1 The first building
        Ptr < Building > building10;
        building10 = Create<Building> ();
        building10->SetBoundaries (Box (2.0,10.0,
                                       67.0, 74.0 ,
                                       0.0, 30));
/*       
uint16_t number_rooms  = building10->GetNRoomsX();
       uint16_t number_floors = building10->GetNFloors();
       std::cout << "Number of room: " << number_rooms << std::endl;
       std::cout << "Number of floors: " << number_floors << std::endl;
*/



//3.2 The second building
        Ptr < Building > building11;
        building11 = Create<Building> ();
        building11->SetBoundaries (Box (18.0,26.0,
                                       67.0, 74.0 ,
                                       0.0, 30));




//3.3 The third building
        Ptr < Building > building12;
        building12 = Create<Building> ();
        building12->SetBoundaries (Box (34.0,42.0,
                                       67.0, 74.0 ,
                                        0.0, 30));




/*

//3.4 The first tree.
        Ptr < Building > building4;
        building4 = Create<Building> ();
        building4->SetBoundaries (Box (51,51.5,
                                       67.0, 67.5,
                                       0.0, 30));



//3.5 The second tree.
        Ptr < Building > building5;
        building5 = Create<Building> ();
        building5->SetBoundaries (Box (56.5,57,
                                       67.0, 67.5,
                                       0.0, 10));

//3.6 The third tree.
        Ptr < Building > building6;
        building6 = Create<Building> ();
        building6->SetBoundaries (Box (62,62.5,
                                       67.0, 67.5,
                                       0.0, 10));







//3.3 The third building
        Ptr < Building > building7;
        building7 = Create<Building> ();
        building7->SetBoundaries (Box (67.5,75.5,
                                       67.0, 74.0 ,
                                       0.0, 30));



//3.3 The third building
        Ptr < Building > building8;
        building8 = Create<Building> ();
        building8->SetBoundaries (Box (85.5,93.5,
                                       67.0, 74.0 ,
                                       0.0, 30));



//3.3 The third building
        Ptr < Building > building9;
        building9 = Create<Building> ();
        building9->SetBoundaries (Box (103.5,111.5,
                                       67.0, 74.0 ,
                                       0.0, 30));
*/

        break;
        break;
      }
    default:
      {
        NS_FATAL_ERROR ("Invalid scenario");
      }
    }



  NodeContainer enbNodes;
  enbNodes.Create (1);
  ueNodes.Create (1);

  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 15.0));//Mogheyate enb.
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (enbNodes);
  BuildingsHelper::Install (enbNodes);

  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);

//Mogheyate ebtedayi ue(0).
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0, 68, 1.5));



  ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, 0, 0));




//To in zamanha sorate nodaro avaz mikonim.
   Simulator::Schedule (Seconds (1), &ChangeSpeed, ueNodes.Get (0), Vector (1.5, 0, 0));


   Simulator::Schedule (Seconds (4), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));


  Simulator::Schedule (Seconds (9), &ChangeSpeed, ueNodes.Get (0), Vector (1.5, 0, 0));



   Simulator::Schedule (Seconds (22), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));



  Simulator::Schedule (Seconds (27), &ChangeSpeed, ueNodes.Get (0), Vector (1.5, 0, 0));




   Simulator::Schedule (Seconds (37), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));


  Simulator::Schedule (Seconds (42), &ChangeSpeed, ueNodes.Get (0), Vector (1.5, 0, 0));



   Simulator::Schedule (Seconds (48), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));















//Simulator::Schedule (Seconds (30), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));


Simulator::Schedule (Seconds (0.2), &getPosition, ueNodes.Get (0));

/*
Simulator::Schedule (Seconds (1.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (1.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (2.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (2.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (3.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (3.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (4.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (4.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (5.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (5.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (6.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (6.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (7.0), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (7.5), &getPosition, ueNodes.Get (0));
Simulator::Schedule (Seconds (8.0), &getPosition, ueNodes.Get (0));
*/

 

//Dar shoro dar mogheyate (0, 0, 1) ast va dar sanie 2 shoro be harakat mikonad ba sorate (1.5, 0, 0) yani har sanie 1.5 metr be rast pas sanie 3 -> 1.3 va sanie 4 2.8. Nazdiktarin obstacle ->(60.0, 64.0,  0.0, 2.0,   0.0, 1.5)) yani y (0,2)  pas daorobare sanie 2 in mane blockage ijad mikonad. Baraye hamin ast ke throughput dar havalie sanie 2 be shedat oft mikonad.


  BuildingsHelper::Install (ueNodes);

  // Install LTE Devices to the nodes
  NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  // Assign IP address to UEs, and install applications
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  mmwaveHelper->AttachToClosestEnb (ueDevs, enbDevs);
  mmwaveHelper->EnableTraces ();

  // Set the default gateway for the UE
  Ptr<Node> ueNode = ueNodes.Get (0);
  Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);


//src/network/helper
ApplicationContainer sinkApps;
ApplicationContainer sinkApps2;
ApplicationContainer sinkApps3;
ApplicationContainer sinkApps4;
ApplicationContainer sinkApps5;
ApplicationContainer sinkApps6;
ApplicationContainer sinkApps7;
ApplicationContainer sinkApps8;
ApplicationContainer sinkApps9;
ApplicationContainer sinkApps10;
ApplicationContainer sinkApps11;
ApplicationContainer sinkApps12;
ApplicationContainer sinkApps13;
ApplicationContainer sinkApps14;
ApplicationContainer sinkApps15;
ApplicationContainer sinkApps16;

  if (tcp)
    {
      // Install and start applications on UEs and remote host (dar Ferestande)
      uint16_t sinkPort = 20000;
      uint16_t sinkPort2 = 20001;
      uint16_t sinkPort3 = 20002;
      uint16_t sinkPort4 = 20003;
      uint16_t sinkPort5 = 20004;
      uint16_t sinkPort6 = 20005;
      uint16_t sinkPort7 = 20006;
      uint16_t sinkPort8 = 20007;
      uint16_t sinkPort9 = 20008;
      uint16_t sinkPort10 = 20009;
      uint16_t sinkPort11 = 20010;
      uint16_t sinkPort12 = 20011;
      uint16_t sinkPort13 = 20012;
      uint16_t sinkPort14 = 20013;
      uint16_t sinkPort15 = 20014;
      uint16_t sinkPort16 = 20015;


      Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
      Address sinkAddress2 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort2));
      Address sinkAddress3 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort3));
      Address sinkAddress4 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort4));
      Address sinkAddress5 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort5));
      Address sinkAddress6 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort6));
      Address sinkAddress7 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort7));
      Address sinkAddress8 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort8));
      Address sinkAddress9 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort9));
      Address sinkAddress10 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort10));
      Address sinkAddress11 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort11));
      Address sinkAddress12 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort12));
      Address sinkAddress13 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort13));
      Address sinkAddress14 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort14));
      Address sinkAddress15 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort15));
      Address sinkAddress16 (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort16));

      PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
//Badan ezafe kardam.
 packetSinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps = packetSinkHelper.Install (ueNodes.Get (0));
sink = StaticCast<PacketSink> (sinkApps.Get (0));

      sinkApps.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput);
     //Simulator::Schedule (Seconds (0), &errorrate);



      sinkApps.Stop (Seconds (simStopTime));

     PacketSinkHelper packetSinkHelper2 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort2));
//Badan ezafe kardam.
 packetSinkHelper2.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps2 = packetSinkHelper2.Install (ueNodes.Get (0));
sink2 = StaticCast<PacketSink> (sinkApps2.Get (0));

      sinkApps2.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput2);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps2.Stop (Seconds (simStopTime));
      
      
       PacketSinkHelper packetSinkHelper3 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort3));
//Badan ezafe kardam.
 packetSinkHelper3.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps3 = packetSinkHelper3.Install (ueNodes.Get (0));
sink3 = StaticCast<PacketSink> (sinkApps3.Get (0));

      sinkApps3.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput3);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps3.Stop (Seconds (simStopTime));
      
      
       PacketSinkHelper packetSinkHelper4 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort4));
//Badan ezafe kardam.
 packetSinkHelper4.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps4 = packetSinkHelper4.Install (ueNodes.Get (0));
sink4 = StaticCast<PacketSink> (sinkApps4.Get (0));

      sinkApps4.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput4);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps4.Stop (Seconds (simStopTime));
      
      
        PacketSinkHelper packetSinkHelper5 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort5));
//Badan ezafe kardam.
 packetSinkHelper5.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps5 = packetSinkHelper5.Install (ueNodes.Get (0));
sink5 = StaticCast<PacketSink> (sinkApps5.Get (0));

      sinkApps5.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput5);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps5.Stop (Seconds (simStopTime));
      
        PacketSinkHelper packetSinkHelper6 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort6));
//Badan ezafe kardam.
 packetSinkHelper6.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps6 = packetSinkHelper6.Install (ueNodes.Get (0));
sink6 = StaticCast<PacketSink> (sinkApps6.Get (0));

      sinkApps6.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput6);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps6.Stop (Seconds (simStopTime));
      
        PacketSinkHelper packetSinkHelper7 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort7));
//Badan ezafe kardam.
 packetSinkHelper7.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps7 = packetSinkHelper7.Install (ueNodes.Get (0));
sink7 = StaticCast<PacketSink> (sinkApps7.Get (0));

      sinkApps7.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput7);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps7.Stop (Seconds (simStopTime));
      
        PacketSinkHelper packetSinkHelper8 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort8));
//Badan ezafe kardam.
 packetSinkHelper8.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps8 = packetSinkHelper8.Install (ueNodes.Get (0));
sink8 = StaticCast<PacketSink> (sinkApps8.Get (0));

      sinkApps8.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput8);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps8.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper9 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort9));
//Badan ezafe kardam.
 packetSinkHelper9.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps9 = packetSinkHelper9.Install (ueNodes.Get (0));
sink9 = StaticCast<PacketSink> (sinkApps9.Get (0));

      sinkApps9.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput9);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps9.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper10 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort10));
//Badan ezafe kardam.
 packetSinkHelper10.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps10 = packetSinkHelper10.Install (ueNodes.Get (0));
sink10 = StaticCast<PacketSink> (sinkApps10.Get (0));

      sinkApps10.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput10);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps10.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper11 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort11));
//Badan ezafe kardam.
 packetSinkHelper11.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps11 = packetSinkHelper11.Install (ueNodes.Get (0));
sink11 = StaticCast<PacketSink> (sinkApps11.Get (0));

      sinkApps11.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput11);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps11.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper12 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort12));
//Badan ezafe kardam.
 packetSinkHelper12.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps12 = packetSinkHelper12.Install (ueNodes.Get (0));
sink12 = StaticCast<PacketSink> (sinkApps12.Get (0));

      sinkApps12.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput12);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps12.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper13 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort13));
//Badan ezafe kardam.
 packetSinkHelper13.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps13 = packetSinkHelper13.Install (ueNodes.Get (0));
sink13 = StaticCast<PacketSink> (sinkApps13.Get (0));

      sinkApps13.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput13);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps13.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper14 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort14));
//Badan ezafe kardam.
 packetSinkHelper14.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps14 = packetSinkHelper14.Install (ueNodes.Get (0));
sink14 = StaticCast<PacketSink> (sinkApps14.Get (0));

      sinkApps14.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput14);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps14.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper15 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort15));
//Badan ezafe kardam.
 packetSinkHelper15.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps15 = packetSinkHelper15.Install (ueNodes.Get (0));
sink15 = StaticCast<PacketSink> (sinkApps15.Get (0));

      sinkApps15.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput15);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps15.Stop (Seconds (simStopTime));
      
      PacketSinkHelper packetSinkHelper16 ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort16));
//Badan ezafe kardam.
 packetSinkHelper16.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps16 = packetSinkHelper16.Install (ueNodes.Get (0));
sink16 = StaticCast<PacketSink> (sinkApps16.Get (0));

      sinkApps16.Start (Seconds (0.));

//Calle function e CalculateThroughput varaye mohasebe e throughput.
     Simulator::Schedule (Seconds (0), &CalculateThroughput16);
     //Simulator::Schedule (Seconds (0), &errorrate);
     
      sinkApps16.Stop (Seconds (simStopTime));
      
      
      TypeId tid1 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId1;
      nodeId1 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode1 = "/NodeList/" + nodeId1.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode1, TypeIdValue(tid1));
      Ptr<Socket> ns3TcpSocket1 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app1 = CreateObject<MyApp> ();
      app1->Setup (ns3TcpSocket1, sinkAddress, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app1);


      AsciiTraceHelper asciiTraceHelper;
      //AsciiTraceHelper asciiTraceHelper1;

      Ptr<OutputStreamWrapper> stream4 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt1.txt").c_str());
      ns3TcpSocket1->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream4));
      
      app1->SetStartTime (Seconds (0.1));
      app1->SetStopTime (Seconds (stopTime));
      
      //Config::SetDefault("ns3::TcpL4Protocol::SocketType", StringValue("ns3::TcpCubiwood"));//testing
      //Ptr<Socket> ns3TcpSocket2 = Socket::CreateSocket (remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId ());
      
      
      TypeId tid = TypeId::LookupByName("ns3::TcpCubic");
      //TypeId tid = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId;
      nodeId << remoteHostContainer.Get (0)->GetId();
      std::string specificNode = "/NodeList/" + nodeId.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode, TypeIdValue(tid));
      Ptr<Socket> ns3TcpSocket2 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app2 = CreateObject<MyApp> ();
      app2->Setup (ns3TcpSocket2, sinkAddress2, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app2);
      
      Ptr<OutputStreamWrapper> stream42 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt2.txt").c_str());
      ns3TcpSocket2->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream42));
      
      //app2->SetStartTime (Seconds (0.2));
      app2->SetStartTime (Seconds (0.1));
      app2->SetStopTime (Seconds (stopTime));
      
      
      
      TypeId tid3 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId3;
      nodeId3 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode3 = "/NodeList/" + nodeId3.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode3, TypeIdValue(tid3));
      Ptr<Socket> ns3TcpSocket3 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app3 = CreateObject<MyApp> ();
      app3->Setup (ns3TcpSocket3, sinkAddress3, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app3);
      
      Ptr<OutputStreamWrapper> stream43 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt3.txt").c_str());
      ns3TcpSocket3->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream43));
      
      app3->SetStartTime (Seconds (0.1));
      app3->SetStopTime (Seconds (stopTime));
      
      
      TypeId tid4 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId4;
      nodeId4 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode4 = "/NodeList/" + nodeId4.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode4, TypeIdValue(tid4));
      Ptr<Socket> ns3TcpSocket4 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app4 = CreateObject<MyApp> ();
      app4->Setup (ns3TcpSocket4, sinkAddress4, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app4);
      
      
      Ptr<OutputStreamWrapper> stream44 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt4.txt").c_str());
      ns3TcpSocket4->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream44));
      
      app4->SetStartTime (Seconds (0.1));
      app4->SetStopTime (Seconds (stopTime));
      
      TypeId tid5 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId5;
      nodeId5 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode5 = "/NodeList/" + nodeId5.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode5, TypeIdValue(tid5));
      Ptr<Socket> ns3TcpSocket5 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app5 = CreateObject<MyApp> ();
      app5->Setup (ns3TcpSocket5, sinkAddress5, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app5);
      
      
      Ptr<OutputStreamWrapper> stream45 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt5.txt").c_str());
      ns3TcpSocket5->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream45));
      
      app5->SetStartTime (Seconds (0.1));
      app5->SetStopTime (Seconds (stopTime));
      
      TypeId tid6 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId6;
      nodeId6 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode6 = "/NodeList/" + nodeId6.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode6, TypeIdValue(tid6));
      Ptr<Socket> ns3TcpSocket6 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app6 = CreateObject<MyApp> ();
      app6->Setup (ns3TcpSocket6, sinkAddress6, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app6);
      
      
      Ptr<OutputStreamWrapper> stream46 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt6.txt").c_str());
      ns3TcpSocket6->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream46));
      
      app6->SetStartTime (Seconds (0.1));
      app6->SetStopTime (Seconds (stopTime));
      
      
      TypeId tid7 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId7;
      nodeId7 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode7 = "/NodeList/" + nodeId7.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode7, TypeIdValue(tid7));
      Ptr<Socket> ns3TcpSocket7 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app7 = CreateObject<MyApp> ();
      app7->Setup (ns3TcpSocket7, sinkAddress7, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app7);
      
      
      Ptr<OutputStreamWrapper> stream47 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt7.txt").c_str());
      ns3TcpSocket7->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream47));
      
      app7->SetStartTime (Seconds (0.1));
      app7->SetStopTime (Seconds (stopTime));
      
      
      TypeId tid8 = TypeId::LookupByName("ns3::TcpCubic");
      std::stringstream nodeId8;
      nodeId8 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode8 = "/NodeList/" + nodeId8.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode8, TypeIdValue(tid8));
      Ptr<Socket> ns3TcpSocket8 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app8 = CreateObject<MyApp> ();
      app8->Setup (ns3TcpSocket8, sinkAddress8, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app8);
      
      
      Ptr<OutputStreamWrapper> stream48 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt8.txt").c_str());
      ns3TcpSocket8->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream48));
      
      app8->SetStartTime (Seconds (0.1));
      app8->SetStopTime (Seconds (stopTime));
      
      TypeId tid9 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId9;
      nodeId9 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode9 = "/NodeList/" + nodeId9.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode9, TypeIdValue(tid9));
      Ptr<Socket> ns3TcpSocket9 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app9 = CreateObject<MyApp> ();
      app9->Setup (ns3TcpSocket9, sinkAddress9, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app9);
      
      
      Ptr<OutputStreamWrapper> stream49 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt9.txt").c_str());
      ns3TcpSocket9->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream49));
      
      app9->SetStartTime (Seconds (0.1));
      app9->SetStopTime (Seconds (stopTime));
      
      TypeId tid10 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId10;
      nodeId10 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode10 = "/NodeList/" + nodeId10.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode10, TypeIdValue(tid10));
      Ptr<Socket> ns3TcpSocket10 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app10 = CreateObject<MyApp> ();
      app10->Setup (ns3TcpSocket10, sinkAddress10, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app10);
      
      
      Ptr<OutputStreamWrapper> stream410 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt10.txt").c_str());
      ns3TcpSocket10->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream410));
      
      app10->SetStartTime (Seconds (0.1));
      app10->SetStopTime (Seconds (stopTime));
      
      TypeId tid11 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId11;
      nodeId11 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode11 = "/NodeList/" + nodeId11.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode11, TypeIdValue(tid11));
      Ptr<Socket> ns3TcpSocket11 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app11 = CreateObject<MyApp> ();
      app11->Setup (ns3TcpSocket11, sinkAddress11, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app11);
      
      
      Ptr<OutputStreamWrapper> stream411 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt11.txt").c_str());
      ns3TcpSocket11->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream411));
      
      app11->SetStartTime (Seconds (0.1));
      app11->SetStopTime (Seconds (stopTime));
      
      TypeId tid12 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId12;
      nodeId12 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode12 = "/NodeList/" + nodeId12.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode12, TypeIdValue(tid12));
      Ptr<Socket> ns3TcpSocket12 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app12 = CreateObject<MyApp> ();
      app12->Setup (ns3TcpSocket12, sinkAddress12, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app12);
      
      
      Ptr<OutputStreamWrapper> stream412 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt12.txt").c_str());
      ns3TcpSocket12->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream412));
      
      app12->SetStartTime (Seconds (0.1));
      app12->SetStopTime (Seconds (stopTime));
      
      TypeId tid13 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId13;
      nodeId13 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode13 = "/NodeList/" + nodeId13.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode13, TypeIdValue(tid13));
      Ptr<Socket> ns3TcpSocket13 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app13 = CreateObject<MyApp> ();
      app13->Setup (ns3TcpSocket13, sinkAddress13, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app13);
      
      
      Ptr<OutputStreamWrapper> stream413 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt13.txt").c_str());
      ns3TcpSocket13->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream413));
      
      app13->SetStartTime (Seconds (0.1));
      app13->SetStopTime (Seconds (stopTime));
      
      TypeId tid14 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId14;
      nodeId14 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode14 = "/NodeList/" + nodeId14.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode14, TypeIdValue(tid14));
      Ptr<Socket> ns3TcpSocket14 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app14 = CreateObject<MyApp> ();
      app14->Setup (ns3TcpSocket14, sinkAddress14, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app14);
      
      
      Ptr<OutputStreamWrapper> stream414 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt14.txt").c_str());
      ns3TcpSocket14->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream414));
      
      app14->SetStartTime (Seconds (0.1));
      app14->SetStopTime (Seconds (stopTime));
      
      TypeId tid15 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId15;
      nodeId15 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode15 = "/NodeList/" + nodeId15.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode15, TypeIdValue(tid15));
      Ptr<Socket> ns3TcpSocket15 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app15 = CreateObject<MyApp> ();
      app15->Setup (ns3TcpSocket15, sinkAddress15, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app15);
      
      
      Ptr<OutputStreamWrapper> stream415 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt15.txt").c_str());
      ns3TcpSocket15->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream415));
      
      app15->SetStartTime (Seconds (0.1));
      app15->SetStopTime (Seconds (stopTime));
      
      TypeId tid16 = TypeId::LookupByName("ns3::TcpCubiwood");
      std::stringstream nodeId16;
      nodeId16 << remoteHostContainer.Get (0)->GetId();
      std::string specificNode16 = "/NodeList/" + nodeId16.str() + "/$ns3::TcpL4Protocol/SocketType";
      Config::Set(specificNode16, TypeIdValue(tid16));
      Ptr<Socket> ns3TcpSocket16 = Socket::CreateSocket(remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId());
     
      Ptr<MyApp> app16 = CreateObject<MyApp> ();
      app16->Setup (ns3TcpSocket16, sinkAddress16, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app16);
      
      
      Ptr<OutputStreamWrapper> stream416 = asciiTraceHelper.CreateFileStream ((path_to_res + prefix_file_name + "-rtt16.txt").c_str());
      ns3TcpSocket16->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream416));
      
      app16->SetStartTime (Seconds (0.1));
      app16->SetStopTime (Seconds (stopTime));


    }

//Agar if (tcp) dorost nabod.
  else
    {
      // Install and start applications on UEs and remote host
      uint16_t sinkPort = 20000;

      Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
      ApplicationContainer sinkApps = packetSinkHelper.Install (ueNodes.Get (0));



//
    sink = StaticCast<PacketSink> (sinkApps.Get (0));
     


     sinkApps.Start (Seconds (0.));


//
     Simulator::Schedule (Seconds (0), &CalculateThroughput);




      sinkApps.Stop (Seconds (simStopTime));

      Ptr<Socket> ns3UdpSocket = Socket::CreateSocket (remoteHostContainer.Get (0), UdpSocketFactory::GetTypeId ());
      Ptr<MyApp> app = CreateObject<MyApp> ();
      app->Setup (ns3UdpSocket, sinkAddress, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app);
      AsciiTraceHelper asciiTraceHelper;
      Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream ("mmWave-udp-data-am.txt");
      sinkApps.Get (0)->TraceConnectWithoutContext ("Rx",MakeBoundCallback (&Rx, stream2));

      app->SetStartTime (Seconds (0.1));
      app->SetStopTime (Seconds (stopTime));

    }


  //p2ph.EnablePcapAll("mmwave-sgi-capture");
  //BuildingsHelper::MakeMobilityModelConsistent ();
  Config::Set ("/NodeList/*/DeviceList/*/TxQueue/MaxSize", QueueSizeValue (QueueSize ("1000000p")));




 AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream ((path_to_res + prefix_file_name + "-mobility-trace.mob").c_str()));

  Simulator::Stop (Seconds (simStopTime));
  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();
  Simulator::Run ();
  printStats (flowmonHelper, true);
  Simulator::Destroy ();


  std::cout << std::endl << "*** Application statistics ***" << std::endl;
  double thr = 0;
  double thr2= 0;
  double thr3= 0;
  double thr4= 0;
  double thr5 = 0;
  double thr6= 0;
  double thr7= 0;
  double thr8= 0;
  double thr9 = 0;
  double thr10= 0;
  double thr11= 0;
  double thr12= 0;
  double thr13 = 0;
  double thr14= 0;
  double thr15= 0;
  double thr16= 0;
  double avg = 0;
  uint64_t totalPacketsThr = DynamicCast<PacketSink> (sinkApps.Get (0))->GetTotalRx ();
  thr = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For First sink app: " << thr << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps2.Get (0))->GetTotalRx ();
  thr2 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Second sink app: " << thr2 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps3.Get (0))->GetTotalRx ();
  thr3 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For third sink app: " << thr3 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps4.Get (0))->GetTotalRx ();
  thr4 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Fourth sink app: " << thr4 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps5.Get (0))->GetTotalRx ();
  thr5 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Fifth sink app: " << thr5 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps6.Get (0))->GetTotalRx ();
  thr6 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Sixth sink app: " << thr6 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps7.Get (0))->GetTotalRx ();
  thr7 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Seventh sink app: " << thr7 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps8.Get (0))->GetTotalRx ();
  thr8 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Eighth sink app: " << thr8 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps9.Get (0))->GetTotalRx ();
  thr9 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Ninth sink app: " << thr9 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps10.Get (0))->GetTotalRx ();
  thr10 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Tenth sink app: " << thr10 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps11.Get (0))->GetTotalRx ();
  thr11 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Eleventh sink app: " << thr11 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps12.Get (0))->GetTotalRx ();
  thr12 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Twelth sink app: " << thr12 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps13.Get (0))->GetTotalRx ();
  thr13 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Thirteenth sink app: " << thr13 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps14.Get (0))->GetTotalRx ();
  thr14 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Fourteenth sink app: " << thr14 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps15.Get (0))->GetTotalRx ();
  thr15 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Fifteenth sink app: " << thr15 << " Mbit/s" << std::endl;
  totalPacketsThr = DynamicCast<PacketSink> (sinkApps16.Get (0))->GetTotalRx ();
  thr16 = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For Sixteenth sink app: " << thr16 << " Mbit/s" << std::endl;
  std::cout << std::endl << "*** TC Layer statistics ***" << std::endl;
  xSum=thr+thr2+thr3+thr4+thr5+thr6+thr7+thr8+thr9+thr10+thr11+thr12+thr13+thr14+thr15+thr16;
  avg=xSum/16;
  x2Sum=thr*thr+thr2*thr2+thr3*thr3+thr4*thr4+thr5*thr5+thr6*thr6+thr7*thr7+thr8*thr8+thr9*thr9+thr10*thr10+thr11*thr11+thr12*thr12+thr13*thr13+thr14*thr14+thr15*thr15+thr16*thr16;
  fairnessIndex=(xSum*xSum)/(x2Sum*16);
  std::cout << std::endl << "*** Fairness Index ***" << std::endl;
  std::cout<<fairnessIndex<<std::endl;
  std::cout << std::endl << "*** Average Overall Goodput ***" << std::endl;
  std::cout<<avg<<std::endl;
  std::cout << q->GetStats () << std::endl;

  fclose(stdout);

  return 0;

}
