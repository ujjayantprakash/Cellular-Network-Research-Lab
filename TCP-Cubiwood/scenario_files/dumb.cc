/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"


#define NET_MASK   "255.255.255.0"
#define NET_ADD1   "10.1.1.0"
#define NET_ADD2   "10.2.1.0"
#define NET_ADD3   "10.3.1.0"


using namespace ns3;


NS_LOG_COMPONENT_DEFINE ("TcpDumbbellIncrease");

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



int
main (int argc, char *argv[])
{
  uint32_t nLeaf = 20;
  bool dcr = true;
  double simTime = 20;
  std::string animFile = "dumbbell-animation.xml";
  ns3::LogComponentEnableAll(ns3::LOG_LEVEL_WARN);

  CommandLine cmd;
  cmd.AddValue ("nLeaf", "PointToPointDumbbell network leafs number.", nLeaf);
  cmd.AddValue ("DCR", "Enable Selective ACKnowledgements", dcr);
  cmd.Parse (argc, argv);
  
  //Config::SetDefault ("ns3::TcpBbr::BBRVariant", EnumValue(TcpBbr::BBR_V3));
  //Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName ("ns3::TcpBbr")));
  
  Config::SetDefault ("ns3::TcpSocketBase::DCR", BooleanValue (dcr)); 
  Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(true));
  
  //Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpCubiwood::GetTypeId()));
  Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpCubic::GetTypeId()));

  // Create the point-to-point link helper
  PointToPointHelper p2pRouter;
  p2pRouter.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("10Mb/s")));
  p2pRouter.SetChannelAttribute ("Delay", TimeValue (Seconds (0.001)));
  PointToPointHelper p2pLeaf;
  p2pLeaf.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("20Mb/s")));
  p2pLeaf.SetChannelAttribute ("Delay", TimeValue (Seconds (0.001)));
 
  PointToPointDumbbellHelper net (nLeaf, p2pLeaf, nLeaf, p2pLeaf, p2pRouter);


  // Install Stack
  InternetStackHelper stack;
  net.InstallStack (stack);


  // Assign IP Addresses
  net.AssignIpv4Addresses (Ipv4AddressHelper (NET_ADD1, NET_MASK),
                           Ipv4AddressHelper (NET_ADD2, NET_MASK),
                           Ipv4AddressHelper (NET_ADD3, NET_MASK));


  /*for(uint32_t v = 0; v < nLeaf; ++v)
    {
      std::cout << "IP:" << net.GetLeftIpv4Address(v) << "\n";
    }
  */
  // install tcp source app on all left side nodes and sink app on all right side nodes
  uint32_t sinkPort = 8080;
  ApplicationContainer sourceApps;
  ApplicationContainer sinkApps;
  AddressValue remoteAddress;
  BulkSendHelper ftp ("ns3::TcpSocketFactory", Address());
  PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
  for(uint32_t u = 0; u < nLeaf; ++u)
    {
      remoteAddress.Set (InetSocketAddress (net.GetRightIpv4Address (u), sinkPort));
      ftp.SetAttribute ("Remote", remoteAddress);
      sourceApps.Add (ftp.Install (net.GetLeft (u)));
      sinkApps.Add (sink.Install (net.GetRight (u)));
    }
  sinkApps.Start (Seconds (0.0));
  sinkApps.Stop (Seconds (simTime));
  sourceApps.Start (Seconds (0.1));
  sourceApps.Stop (Seconds (simTime));
 
  
  // Set the bounding box for animation
  net.BoundingBox (1, 1, 100, 100);
 
  // Create the animation object and configure for specified output
  AnimationInterface anim (animFile);
  anim.EnablePacketMetadata (); // Optional
  anim.EnableIpv4L3ProtocolCounters (Seconds (0), Seconds (simTime)); // Optional
  //anim.SetNodeSize(2, 2);
  //anim.SetNodeDescriptionFontSize(14);
   
  // Set up the actual simulation
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
 
  std::cout << "Animation Trace file created:" << animFile.c_str ()<< std::endl;
 
  //FlowMonitorHelper flowmon;
  //Ptr<FlowMonitor> monitor = flowmon.InstallAll ();


  Simulator::Stop(Seconds(simTime));
  
  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();
  
  Simulator::Run();


  /*monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
      std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
      std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
      std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / (simTime) / 1000 / 1000  << " Mbps\n";
      std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
      std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
      std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / (simTime) / 1000 / 1000  << " Mbps\n";
    }*/
    
  printStats (flowmonHelper, true);


  Simulator::Destroy();
  
  std::cout << std::endl << "*** Application statistics ***" << std::endl;
  double thr = 0;
  double xyz=0;
  uint64_t totalPacketsThr;
  for(int i=0;i<=19;i=i+1)
  {
    totalPacketsThr = DynamicCast<PacketSink> (sinkApps.Get (i))->GetTotalRx ();
    thr = totalPacketsThr * 8 / (simTime * 1000000.0); //Mbit/s
    xyz=xyz+thr;
  }
  double res=xyz/20;
  std::cout << "  Average Goodput For NewProtocol: " << res << " Mbit/s" << std::endl;
  return 0;
}
