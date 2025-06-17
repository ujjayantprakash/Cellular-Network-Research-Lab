/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Natale Patriciello <natale.patriciello@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// Author: Ujjayant Prakash

#define NS_LOG_APPEND_CONTEXT \
  { std::clog << Simulator::Now ().GetSeconds () << " "; }

#include "tcp-cubiwood.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("TcpCubiwood");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (TcpCubiwood);

TypeId
TcpCubiwood::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TcpCubiwood")
    .SetParent<TcpSocketBase> ()
    .AddConstructor<TcpCubiwood> ()
    .SetGroupName ("Internet")
    .AddAttribute ("FastConvergence", "Enable (true) or disable (false) fast convergence",
                   BooleanValue (true),
                   MakeBooleanAccessor (&TcpCubiwood::m_fastConvergence),
                   MakeBooleanChecker ())
    .AddAttribute ("Beta", "Beta for multiplicative decrease",
                   DoubleValue (0.8),
                   MakeDoubleAccessor (&TcpCubiwood::m_beta),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("Alpha", "Weighting factor for TUSTIN filter",
                   DoubleValue (0.93),
                   MakeDoubleAccessor (&TcpCubiwood::alpha),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("HyStart", "Enable (true) or disable (false) hybrid slow start algorithm",
                   BooleanValue (true),
                   MakeBooleanAccessor (&TcpCubiwood::m_hystart),
                   MakeBooleanChecker ())
    .AddAttribute ("HyStartLowWindow", "Lower bound cWnd for hybrid slow start (segments)",
                   UintegerValue (16),
                   MakeUintegerAccessor (&TcpCubiwood::m_hystartLowWindow),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("HyStartDetect", "Hybrid Slow Start detection mechanisms:" \
                   "1: packet train, 2: delay, 3: both",
                   IntegerValue (3),
                   MakeIntegerAccessor (&TcpCubiwood::m_hystartDetect),
                   MakeIntegerChecker <int> (1,3))
    .AddAttribute ("HyStartMinSamples", "Number of delay samples for detecting the increase of delay",
                   UintegerValue (8),
                   MakeUintegerAccessor (&TcpCubiwood::m_hystartMinSamples),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("HyStartAckDelta", "Spacing between ack's indicating train",
                   TimeValue (MilliSeconds (2)),
                   MakeTimeAccessor (&TcpCubiwood::m_hystartAckDelta),
                   MakeTimeChecker ())
    .AddAttribute ("HyStartDelayMin", "Minimum time for hystart algorithm",
                   TimeValue (MilliSeconds (4)),
                   MakeTimeAccessor (&TcpCubiwood::m_hystartDelayMin),
                   MakeTimeChecker ())
    .AddAttribute ("HyStartDelayMax", "Maximum time for hystart algorithm",
                   TimeValue (MilliSeconds (1000)),
                   MakeTimeAccessor (&TcpCubiwood::m_hystartDelayMax),
                   MakeTimeChecker ())
    .AddAttribute ("CubiwoodDelta", "Delta Time to wait after fast recovery before adjusting param",
                   TimeValue (MilliSeconds (10)),
                   MakeTimeAccessor (&TcpCubiwood::m_cubicDelta),
                   MakeTimeChecker ())
    .AddAttribute ("CntClamp", "Counter value when no losses are detected (counter is used" \
                   " when incrementing cWnd in congestion avoidance, to avoid" \
                   " floating point arithmetic). It is the modulo of the (avoided)" \
                   " division",
                   UintegerValue (20),
                   MakeUintegerAccessor (&TcpCubiwood::m_cntClamp),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute("FilterType", "Use this to choose no filter or Tustin's approximation filter",
                  EnumValue(TcpCubiwood::TUSTIN), MakeEnumAccessor(&TcpCubiwood::m_fType),
                  MakeEnumChecker(TcpCubiwood::NONE, "None", TcpCubiwood::TUSTIN, "Tustin"))
    .AddAttribute("ProtocolType", "Use this to let the code run as Westwood or WestwoodPlus",
                  EnumValue(TcpCubiwood::WESTWOODPLUS),
                  MakeEnumAccessor(&TcpCubiwood::m_pType),
                  MakeEnumChecker(TcpCubiwood::WESTWOOD, "Westwood",TcpCubiwood::WESTWOODPLUS, "WestwoodPlus"))
    .AddAttribute ("C", "Cubiwood Scaling factor",
                   DoubleValue (0.4),
                   MakeDoubleAccessor (&TcpCubiwood::m_c),
                   MakeDoubleChecker <double> (0.0))
    .AddTraceSource("EstimatedBW", "The estimated bandwidth",
                    MakeTraceSourceAccessor(&TcpCubiwood::m_currentBW),
                    "ns3::TracedValueCallback::Double")
  ;
  return tid;
}

TcpCubiwood::TcpCubiwood ()
  : TcpCongestionOps (),
    m_cWndCnt (0),
    m_lastMaxCwnd (0),
    m_bicOriginPoint (0),
    m_bicK (0.0),
    m_delayMin (Time::Min ()),
    m_epochStart (Time::Min ()),
    m_found (false),
    m_roundStart (Time::Min ()),
    m_endSeq (0),
    m_lastAck (Time::Min ()),
    m_cubicDelta (Time::Min ()),
    m_currRtt (Time::Min ()),
    m_sampleCnt (0),
    m_currentBW (0),
    m_lastSampleBW (0),
    m_lastBW (0),
    m_ackedSegments (0),
    m_IsCount (false)
{
  NS_LOG_FUNCTION (this);
}

TcpCubiwood::TcpCubiwood (const TcpCubiwood &sock)
  : TcpCongestionOps (sock),
    m_fastConvergence (sock.m_fastConvergence),
    m_beta (sock.m_beta),
    m_hystart (sock.m_hystart),
    m_hystartDetect (sock.m_hystartDetect),
    m_hystartLowWindow (sock.m_hystartLowWindow),
    m_hystartAckDelta (sock.m_hystartAckDelta),
    m_hystartDelayMin (sock.m_hystartDelayMin),
    m_hystartDelayMax (sock.m_hystartDelayMax),
    m_hystartMinSamples (sock.m_hystartMinSamples),
    m_initialCwnd (sock.m_initialCwnd),
    m_cntClamp (sock.m_cntClamp),
    m_c (sock.m_c),
    m_cWndCnt (sock.m_cWndCnt),
    m_lastMaxCwnd (sock.m_lastMaxCwnd),
    m_bicOriginPoint (sock.m_bicOriginPoint),
    m_bicK (sock.m_bicK),
    m_delayMin (sock.m_delayMin),
    m_epochStart (sock.m_epochStart),
    m_found (sock.m_found),
    m_roundStart (sock.m_roundStart),
    m_endSeq (sock.m_endSeq),
    m_lastAck (sock.m_lastAck),
    m_cubicDelta (sock.m_cubicDelta),
    m_currRtt (sock.m_currRtt),
    m_sampleCnt (sock.m_sampleCnt),
    m_currentBW (sock.m_currentBW),
    m_lastSampleBW (sock.m_lastSampleBW),
    m_lastBW (sock.m_lastBW),
    m_pType (sock.m_pType),
    m_fType (sock.m_fType),
    m_IsCount (sock.m_IsCount)
{
  NS_LOG_FUNCTION (this);
}

std::string
TcpCubiwood::GetName () const
{
  return "TcpCubiwood";
}

void
TcpCubiwood::HystartReset (Ptr<const TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this);

  m_roundStart = m_lastAck = Simulator::Now ();
  m_endSeq = tcb->m_highTxMark;
  m_currRtt = Time::Min ();
  m_sampleCnt = 0;
}

void
TcpCubiwood::IncreaseWindow (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked);

  if (tcb->m_cWnd < tcb->m_ssThresh)
    {

      if (m_hystart && tcb->m_lastAckedSeq > m_endSeq)
        {
          HystartReset (tcb);
        }

      tcb->m_cWnd += tcb->m_segmentSize;
      segmentsAcked -= 1;

      NS_LOG_INFO ("In SlowStart, updated to cwnd " << tcb->m_cWnd <<
                   " ssthresh " << tcb->m_ssThresh);
    }

  if (tcb->m_cWnd >= tcb->m_ssThresh && segmentsAcked > 0)
    {
      m_cWndCnt += segmentsAcked;
      uint32_t cnt = Update (tcb);

      /* According to RFC 6356 even once the new cwnd is
       * calculated you must compare this to the number of ACKs received since
       * the last cwnd update. If not enough ACKs have been received then cwnd
       * cannot be updated.
       */
      if (m_cWndCnt >= cnt)
        {
          tcb->m_cWnd += tcb->m_segmentSize;
          m_cWndCnt -= cnt;
          NS_LOG_INFO ("In CongAvoid, updated to cwnd " << tcb->m_cWnd);
        }
      else
        {
          NS_LOG_INFO ("Not enough segments have been ACKed to increment cwnd."
                       "Until now " << m_cWndCnt << " cnd " << cnt);
        }
    }
}

uint32_t
TcpCubiwood::Update (Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this);
  Time t;
  uint32_t delta, bicTarget, cnt = 0;
  double offs;
  uint32_t segCwnd = tcb->GetCwndInSegments ();

  if (m_epochStart == Time::Min ())
    {
      m_epochStart = Simulator::Now ();   // record the beginning of an epoch

      if (m_lastMaxCwnd <= segCwnd)
        {
          NS_LOG_DEBUG ("lastMaxCwnd <= m_cWnd. K=0 and origin=" << segCwnd);
          m_bicK = 0.0;
          m_bicOriginPoint = segCwnd;
        }
      else
        {
          m_bicK = std::pow ((m_lastMaxCwnd - segCwnd) / m_c, 1 / 3.);
          m_bicOriginPoint = m_lastMaxCwnd;
          NS_LOG_DEBUG ("lastMaxCwnd > m_cWnd. K=" << m_bicK <<
                        " and origin=" << m_lastMaxCwnd);
        }
    }

  t = Simulator::Now () + m_delayMin - m_epochStart;

  if (t.GetSeconds () < m_bicK)       /* t - K */
    {
      offs = m_bicK - t.GetSeconds ();
      NS_LOG_DEBUG ("t=" << t.GetSeconds () << " <k: offs=" << offs);
    }
  else
    {
      offs = t.GetSeconds () - m_bicK;
      NS_LOG_DEBUG ("t=" << t.GetSeconds () << " >= k: offs=" << offs);
    }


  /* Constant value taken from Experimental Evaluation of Cubiwood Tcp, available at
   * eprints.nuim.ie/1716/1/Hamiltonpfldnet2007_cubic_final.pdf */
  delta = m_c * std::pow (offs, 3);

  NS_LOG_DEBUG ("delta: " << delta);

  if (t.GetSeconds () < m_bicK)
    {
      // below origin
      bicTarget = m_bicOriginPoint - delta;
      NS_LOG_DEBUG ("t < k: Bic Target: " << bicTarget);
    }
  else
    {
      // above origin
      bicTarget = m_bicOriginPoint + delta;
      NS_LOG_DEBUG ("t >= k: Bic Target: " << bicTarget);
    }

  // Next the window target is converted into a cnt or count value. CUBIC will
  // wait until enough new ACKs have arrived that a counter meets or exceeds
  // this cnt value. This is how the CUBIC implementation simulates growing
  // cwnd by values other than 1 segment size.
  if (bicTarget > segCwnd)
    {
      cnt = segCwnd / (bicTarget - segCwnd);
      NS_LOG_DEBUG ("target>cwnd. cnt=" << cnt);
    }
  else
    {
      cnt = 100 * segCwnd;
    }

  if (m_lastMaxCwnd == 0 && cnt > m_cntClamp)
    {
      cnt = m_cntClamp;
    }

  // The maximum rate of cwnd increase CUBIC allows is 1 packet per
  // 2 packets ACKed, meaning cwnd grows at 1.5x per RTT.
  return std::max (cnt, 2U);
}

void
TcpCubiwood::PktsAcked (Ptr<TcpSocketState> tcb, uint32_t segmentsAcked,
                     const Time &rtt)
{
  NS_LOG_FUNCTION (this << tcb << segmentsAcked << rtt);

  if (rtt.IsZero ())
    {
      NS_LOG_WARN ("RTT measured is zero!");
      return;
    }

  m_ackedSegments += segmentsAcked;

  if (m_pType == TcpCubiwood::WESTWOOD)
    {
      EstimateBW (rtt, tcb);
    }
  else if (m_pType == TcpCubiwood::WESTWOODPLUS)
    {
      if (!(rtt.IsZero () || m_IsCount))
        {
          m_IsCount = true;
          m_bwEstimateEvent.Cancel ();
          m_bwEstimateEvent = Simulator::Schedule (rtt, &TcpCubiwood::EstimateBW,
                                                   this, rtt, tcb);
        }
    }

  /* Discard delay samples right after fast recovery */
  if (m_epochStart != Time::Min ()
      && (Simulator::Now () - m_epochStart) < m_cubicDelta)
    {
      return;
    }

  /* first time call or link delay decreases */
  if (m_delayMin == Time::Min () || m_delayMin > rtt)
    {
      m_delayMin = rtt;
    }

  /* hystart triggers when cwnd is larger than some threshold */
  if (m_hystart
      && tcb->m_cWnd <= tcb->m_ssThresh
      && tcb->m_cWnd >= m_hystartLowWindow * tcb->m_segmentSize)
    {
      HystartUpdate (tcb, rtt);
    }
}

void
TcpCubiwood::HystartUpdate (Ptr<TcpSocketState> tcb, const Time& delay)
{
  NS_LOG_FUNCTION (this << delay);

  if (!(m_found & m_hystartDetect))
    {
      Time now = Simulator::Now ();

      /* first detection parameter - ack-train detection */
      if ((now - m_lastAck) <= m_hystartAckDelta)
        {
          m_lastAck = now;

          if ((now - m_roundStart) > m_delayMin)
            {
              m_found |= PACKET_TRAIN;
            }
        }

      /* obtain the minimum delay of more than sampling packets */
      if (m_sampleCnt < m_hystartMinSamples)
        {
          if (m_currRtt == Time::Min () || m_currRtt > delay)
            {
              m_currRtt = delay;
            }

          ++m_sampleCnt;
        }
      else
        {
          if (m_currRtt > m_delayMin +
              HystartDelayThresh (m_delayMin))
            {
              m_found |= DELAY;
            }
        }
      /*
       * Either one of two conditions are met,
       * we exit from slow start immediately.
       */
      if (m_found & m_hystartDetect)
        {
          NS_LOG_DEBUG ("Exit from SS, immediately :-)");
          tcb->m_ssThresh = tcb->m_cWnd;
        }
    }
}

Time
TcpCubiwood::HystartDelayThresh (const Time& t) const
{
  NS_LOG_FUNCTION (this << t);

  Time ret = t;
  if (t > m_hystartDelayMax)
    {
      ret = m_hystartDelayMax;
    }
  else if (t < m_hystartDelayMin)
    {
      ret = m_hystartDelayMin;
    }

  return ret;
}

//added
void
TcpCubiwood::EstimateBW (const Time &rtt, Ptr<TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT (!rtt.IsZero ());

  m_currentBW = m_ackedSegments * tcb->m_segmentSize / rtt.GetSeconds ();

  if (m_pType == TcpCubiwood::WESTWOODPLUS)
    {
      m_IsCount = false;
    }

  m_ackedSegments = 0;
  NS_LOG_LOGIC ("Estimated BW: " << m_currentBW);

  // Filter the BW sample

  //double alpha = 0.9;
  //double alpha = 0.93;
  NS_LOG_UNCOND("alpha value: " << alpha);

  if (m_fType == TcpCubiwood::NONE)
    {
    }
  else if (m_fType == TcpCubiwood::TUSTIN)
    {
      double sample_bwe = m_currentBW;
      m_currentBW = (alpha * m_lastBW) + ((1 - alpha) * ((sample_bwe + m_lastSampleBW) / 2));
      m_lastSampleBW = sample_bwe;
      m_lastBW = m_currentBW;
    }

  NS_LOG_LOGIC ("Estimated BW after filtering: " << m_currentBW);
}


uint32_t
TcpCubiwood::GetSsThresh (Ptr<const TcpSocketState> tcb, uint32_t bytesInFlight)
{
  NS_LOG_FUNCTION (this << tcb << bytesInFlight);

  // Without inflation and deflation, these two are the same
  uint32_t segInFlight = bytesInFlight / tcb->m_segmentSize;
  uint32_t segCwnd = tcb->GetCwndInSegments ();
  NS_LOG_DEBUG ("Loss at cWnd=" << segCwnd << " in flight=" << segInFlight);

  /* Wmax and fast convergence */
  if (segCwnd < m_lastMaxCwnd && m_fastConvergence)
    {
      //m_lastMaxCwnd = m_beta * segCwnd;
	  m_lastMaxCwnd = (1+m_beta) * segCwnd/2; //by zml
    }
  else
    {
      m_lastMaxCwnd = segCwnd;
    }

  m_epochStart = Time::Min ();    // end of epoch

  /* Formula taken from the Linux kernel */

  //uint32_t ssThresh = std::max (static_cast<uint32_t> (segInFlight * m_beta ), 2U) * tcb->m_segmentSize;
  uint32_t temp2 = std::max (2*tcb->m_segmentSize,
                   uint32_t (m_currentBW * static_cast<double> (tcb->m_minRtt.GetSeconds ())));
  uint32_t temp = std::min (segCwnd, segInFlight);
  uint32_t ssThresh = std::max (static_cast<uint32_t> (temp * m_beta ), 2U) * tcb->m_segmentSize;
  ssThresh = std::max(ssThresh, temp2);

  return ssThresh;
}

void
TcpCubiwood::CongestionStateSet (Ptr<TcpSocketState> tcb, const TcpSocketState::TcpCongState_t newState)
{
  NS_LOG_FUNCTION (this << tcb << newState);

  if (newState == TcpSocketState::CA_LOSS)
    {
      CubiwoodReset (tcb);
      HystartReset (tcb);
    }
}

void
TcpCubiwood::CubiwoodReset (Ptr<const TcpSocketState> tcb)
{
  NS_LOG_FUNCTION (this << tcb);

  m_lastMaxCwnd = 0;
  m_bicOriginPoint = 0;
  m_bicK = 0;
  m_delayMin = Time::Min ();
  m_found = false;
}

Ptr<TcpCongestionOps>
TcpCubiwood::Fork (void)
{
  NS_LOG_FUNCTION (this);
  return CopyObject<TcpCubiwood> (this);
}

}
