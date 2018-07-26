/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Authors: Nicola Baldo <nbaldo@cttc.es>
 *          Marco Miozzo <mmiozzo@cttc.es>
 *          Manuel Requena <manuel.requena@cttc.es>
 */

#include "lte-enb-rrc.h"

#include <ns3/fatal-error.h>
#include <ns3/log.h>
#include <ns3/abort.h>

#include <ns3/pointer.h>
#include <ns3/object-map.h>
#include <ns3/object-factory.h>
#include <ns3/simulator.h>

#include <ns3/lte-radio-bearer-info.h>
#include <ns3/eps-bearer-tag.h>
#include <ns3/packet.h>

#include <ns3/lte-rlc.h>
#include <ns3/lte-rlc-tm.h>
#include <ns3/lte-rlc-um.h>
#include <ns3/lte-rlc-am.h>
#include <ns3/lte-pdcp.h>




namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LteEnbRrc");

///////////////////////////////////////////
// CMAC SAP forwarder
///////////////////////////////////////////

/**
 * \brief Class for forwarding CMAC SAP User functions.
 */
class EnbRrcMemberLteEnbCmacSapUser : public LteEnbCmacSapUser
{
public:
  EnbRrcMemberLteEnbCmacSapUser (LteEnbRrc* rrc);

  virtual uint16_t AllocateTemporaryCellRnti ();
  virtual void NotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success);
  virtual void RrcConfigurationUpdateInd (UeConfig params);

private:
  LteEnbRrc* m_rrc;
};

EnbRrcMemberLteEnbCmacSapUser::EnbRrcMemberLteEnbCmacSapUser (LteEnbRrc* rrc)
  : m_rrc (rrc)
{
}

uint16_t
EnbRrcMemberLteEnbCmacSapUser::AllocateTemporaryCellRnti ()
{
  return m_rrc->DoAllocateTemporaryCellRnti ();
}

void
EnbRrcMemberLteEnbCmacSapUser::NotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success)
{
  m_rrc->DoNotifyLcConfigResult (rnti, lcid, success);
}

void
EnbRrcMemberLteEnbCmacSapUser::RrcConfigurationUpdateInd (UeConfig params)
{
  m_rrc->DoRrcConfigurationUpdateInd (params);
}



///////////////////////////////////////////
// UeManager
///////////////////////////////////////////


/// Map each of UE Manager states to its string representation.
static const std::string g_ueManagerStateName[UeManager::NUM_STATES] =
{
  "INITIAL_RANDOM_ACCESS",
  "CONNECTION_SETUP",
  "CONNECTION_REJECTED",
  "CONNECTED_NORMALLY",
  "CONNECTION_RECONFIGURATION",
  "CONNECTION_REESTABLISHMENT",
  "HANDOVER_PREPARATION",
  "HANDOVER_JOINING",
  "HANDOVER_PATH_SWITCH",
  "HANDOVER_LEAVING",
};

/**
 * \param s The UE manager state.
 * \return The string representation of the given state.
 */
static const std::string & ToString (UeManager::State s)
{
  return g_ueManagerStateName[s];
}


NS_OBJECT_ENSURE_REGISTERED (UeManager);


UeManager::UeManager ()
{
  NS_FATAL_ERROR ("this constructor is not espected to be used");
}


UeManager::UeManager (Ptr<LteEnbRrc> rrc, uint16_t rnti, State s)
  : m_lastAllocatedDrbid (0),
    m_rnti (rnti),
    m_imsi (0),
    m_lastRrcTransactionIdentifier (0),
    m_rrc (rrc),
    m_state (s),
    m_pendingRrcConnectionReconfiguration (false),
    m_sourceX2apId (0),
    m_sourceCellId (0),
    m_needPhyMacConfiguration (false)
{ 
  NS_LOG_FUNCTION (this);
}

void
UeManager::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  m_drbPdcpSapUser = new LtePdcpSpecificLtePdcpSapUser<UeManager> (this);

  m_physicalConfigDedicated.haveAntennaInfoDedicated = true;
  m_physicalConfigDedicated.antennaInfo.transmissionMode = m_rrc->m_defaultTransmissionMode;
  m_physicalConfigDedicated.haveSoundingRsUlConfigDedicated = true;
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex = m_rrc->GetNewSrsConfigurationIndex ();
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.type = LteRrcSap::SoundingRsUlConfigDedicated::SETUP;
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsBandwidth = 0;
  m_physicalConfigDedicated.havePdschConfigDedicated = true;
  m_physicalConfigDedicated.pdschConfigDedicated.pa = LteRrcSap::PdschConfigDedicated::dB0;

  m_rrc->m_cmacSapProvider->AddUe (m_rnti);
  m_rrc->m_cphySapProvider->AddUe (m_rnti);

  //A.M
  //m_hysteresisHasUpdated = false;

  // setup the eNB side of SRB0
  {
    uint8_t lcid = 0;

    Ptr<LteRlc> rlc = CreateObject<LteRlcTm> ()->GetObject<LteRlc> ();
    rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
    rlc->SetRnti (m_rnti);
    rlc->SetLcId (lcid);

    m_srb0 = CreateObject<LteSignalingRadioBearerInfo> ();
    m_srb0->m_rlc = rlc;
    m_srb0->m_srbIdentity = 0;
    // no need to store logicalChannelConfig as SRB0 is pre-configured

    LteEnbCmacSapProvider::LcInfo lcinfo;
    lcinfo.rnti = m_rnti;
    lcinfo.lcId = lcid;
    // leave the rest of lcinfo empty as CCCH (LCID 0) is pre-configured
    m_rrc->m_cmacSapProvider->AddLc (lcinfo, rlc->GetLteMacSapUser ());



  }

  // setup the eNB side of SRB1; the UE side will be set up upon RRC connection establishment
  {
    uint8_t lcid = 1;

    Ptr<LteRlc> rlc = CreateObject<LteRlcAm> ()->GetObject<LteRlc> ();
    rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
    rlc->SetRnti (m_rnti);
    rlc->SetLcId (lcid);

    Ptr<LtePdcp> pdcp = CreateObject<LtePdcp> ();
    pdcp->SetRnti (m_rnti);
    pdcp->SetLcId (lcid);
    pdcp->SetLtePdcpSapUser (m_drbPdcpSapUser);
    pdcp->SetLteRlcSapProvider (rlc->GetLteRlcSapProvider ());
    rlc->SetLteRlcSapUser (pdcp->GetLteRlcSapUser ());

    m_srb1 = CreateObject<LteSignalingRadioBearerInfo> ();
    m_srb1->m_rlc = rlc;
    m_srb1->m_pdcp = pdcp;
    m_srb1->m_srbIdentity = 1;
    m_srb1->m_logicalChannelConfig.priority = 0;
    m_srb1->m_logicalChannelConfig.prioritizedBitRateKbps = 100;
    m_srb1->m_logicalChannelConfig.bucketSizeDurationMs = 100;
    m_srb1->m_logicalChannelConfig.logicalChannelGroup = 0;

    LteEnbCmacSapProvider::LcInfo lcinfo;
    lcinfo.rnti = m_rnti;
    lcinfo.lcId = lcid;
    lcinfo.lcGroup = 0; // all SRBs always mapped to LCG 0
    lcinfo.qci = EpsBearer::GBR_CONV_VOICE; // not sure why the FF API requires a CQI even for SRBs...
    lcinfo.isGbr = true;
    lcinfo.mbrUl = 1e6;
    lcinfo.mbrDl = 1e6;
    lcinfo.gbrUl = 1e4;
    lcinfo.gbrDl = 1e4;
    m_rrc->m_cmacSapProvider->AddLc (lcinfo, rlc->GetLteMacSapUser ());
  }

  LteEnbRrcSapUser::SetupUeParameters ueParams;
  ueParams.srb0SapProvider = m_srb0->m_rlc->GetLteRlcSapProvider ();
  ueParams.srb1SapProvider = m_srb1->m_pdcp->GetLtePdcpSapProvider ();
  m_rrc->m_rrcSapUser->SetupUe (m_rnti, ueParams);

  // configure MAC (and scheduler)
  LteEnbCmacSapProvider::UeConfig req;
  req.m_rnti = m_rnti;
  req.m_transmissionMode = m_physicalConfigDedicated.antennaInfo.transmissionMode;
  m_rrc->m_cmacSapProvider->UeUpdateConfigurationReq (req);

  // configure PHY
  m_rrc->m_cphySapProvider->SetTransmissionMode (m_rnti, m_physicalConfigDedicated.antennaInfo.transmissionMode);
  m_rrc->m_cphySapProvider->SetSrsConfigurationIndex (m_rnti, m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex);

  // schedule this UeManager instance to be deleted if the UE does not give any sign of life within a reasonable time
  Time maxConnectionDelay;
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      m_connectionRequestTimeout = Simulator::Schedule (m_rrc->m_connectionRequestTimeoutDuration,
                                                        &LteEnbRrc::ConnectionRequestTimeout,
                                                        m_rrc, m_rnti);
      break;

    case HANDOVER_JOINING:
      m_handoverJoiningTimeout = Simulator::Schedule (m_rrc->m_handoverJoiningTimeoutDuration,
                                                      &LteEnbRrc::HandoverJoiningTimeout,
                                                      m_rrc, m_rnti);
      break;

    default:
      NS_FATAL_ERROR ("unexpected state " << ToString (m_state));
      break;
    }

}


UeManager::~UeManager (void)
{
}

void
UeManager::DoDispose ()
{
  delete m_drbPdcpSapUser;
  // delete eventual X2-U TEIDs
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      m_rrc->m_x2uTeidInfoMap.erase (it->second->m_gtpTeid);
    }

}

TypeId UeManager::GetTypeId (void)
{
  static TypeId  tid = TypeId ("ns3::UeManager")
    .SetParent<Object> ()
    .AddConstructor<UeManager> ()
    .AddAttribute ("DataRadioBearerMap", "List of UE DataRadioBearerInfo by DRBID.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&UeManager::m_drbMap),
                   MakeObjectMapChecker<LteDataRadioBearerInfo> ())
    .AddAttribute ("Srb0", "SignalingRadioBearerInfo for SRB0",
                   PointerValue (),
                   MakePointerAccessor (&UeManager::m_srb0),
                   MakePointerChecker<LteSignalingRadioBearerInfo> ())
    .AddAttribute ("Srb1", "SignalingRadioBearerInfo for SRB1",
                   PointerValue (),
                   MakePointerAccessor (&UeManager::m_srb1),
                   MakePointerChecker<LteSignalingRadioBearerInfo> ())
    .AddAttribute ("C-RNTI",
                   "Cell Radio Network Temporary Identifier",
                   TypeId::ATTR_GET, // read-only attribute
                   UintegerValue (0), // unused, read-only attribute
                   MakeUintegerAccessor (&UeManager::m_rnti),
                   MakeUintegerChecker<uint16_t> ())
    .AddTraceSource ("StateTransition",
                     "fired upon every UE state transition seen by the "
                     "UeManager at the eNB RRC",
                     MakeTraceSourceAccessor (&UeManager::m_stateTransitionTrace),
                     "ns3::UeManager::StateTracedCallback")
  ;
  return tid;
}

void 
UeManager::SetSource (uint16_t sourceCellId, uint16_t sourceX2apId)
{
  m_sourceX2apId = sourceX2apId;
  m_sourceCellId = sourceCellId;
}

void 
UeManager::SetImsi (uint64_t imsi)
{
  m_imsi = imsi;
}

void
UeManager::SetupDataRadioBearer (EpsBearer bearer, uint8_t bearerId, uint32_t gtpTeid, Ipv4Address transportLayerAddress)
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);

  Ptr<LteDataRadioBearerInfo> drbInfo = CreateObject<LteDataRadioBearerInfo> ();
  uint8_t drbid = AddDataRadioBearerInfo (drbInfo);
  uint8_t lcid = Drbid2Lcid (drbid); 
  uint8_t bid = Drbid2Bid (drbid);
  NS_ASSERT_MSG ( bearerId == 0 || bid == bearerId, "bearer ID mismatch (" << (uint32_t) bid << " != " << (uint32_t) bearerId << ", the assumption that ID are allocated in the same way by MME and RRC is not valid any more");
  drbInfo->m_epsBearerIdentity = bid;
  drbInfo->m_drbIdentity = drbid;
  drbInfo->m_logicalChannelIdentity = lcid;
  drbInfo->m_gtpTeid = gtpTeid;
  drbInfo->m_transportLayerAddress = transportLayerAddress;

  if (m_state == HANDOVER_JOINING)
    {
      // setup TEIDs for receiving data eventually forwarded over X2-U 
      LteEnbRrc::X2uTeidInfo x2uTeidInfo;
      x2uTeidInfo.rnti = m_rnti;
      x2uTeidInfo.drbid = drbid;
      std::pair<std::map<uint32_t, LteEnbRrc::X2uTeidInfo>::iterator, bool>
      ret = m_rrc->m_x2uTeidInfoMap.insert (std::pair<uint32_t, LteEnbRrc::X2uTeidInfo> (gtpTeid, x2uTeidInfo));
      NS_ASSERT_MSG (ret.second == true, "overwriting a pre-existing entry in m_x2uTeidInfoMap");
    }

  TypeId rlcTypeId = m_rrc->GetRlcType (bearer);

  ObjectFactory rlcObjectFactory;
  rlcObjectFactory.SetTypeId (rlcTypeId);
  Ptr<LteRlc> rlc = rlcObjectFactory.Create ()->GetObject<LteRlc> ();
  rlc->SetLteMacSapProvider (m_rrc->m_macSapProvider);
  rlc->SetRnti (m_rnti);

  drbInfo->m_rlc = rlc;

  rlc->SetLcId (lcid);

  // we need PDCP only for real RLC, i.e., RLC/UM or RLC/AM
  // if we are using RLC/SM we don't care of anything above RLC
  if (rlcTypeId != LteRlcSm::GetTypeId ())
    {
      Ptr<LtePdcp> pdcp = CreateObject<LtePdcp> ();
      pdcp->SetRnti (m_rnti);
      pdcp->SetLcId (lcid);
      pdcp->SetLtePdcpSapUser (m_drbPdcpSapUser);
      pdcp->SetLteRlcSapProvider (rlc->GetLteRlcSapProvider ());
      rlc->SetLteRlcSapUser (pdcp->GetLteRlcSapUser ());
      drbInfo->m_pdcp = pdcp;
    }

  LteEnbCmacSapProvider::LcInfo lcinfo;
  lcinfo.rnti = m_rnti;
  lcinfo.lcId = lcid;
  lcinfo.lcGroup = m_rrc->GetLogicalChannelGroup (bearer);
  lcinfo.qci = bearer.qci;
  lcinfo.isGbr = bearer.IsGbr ();
  lcinfo.mbrUl = bearer.gbrQosInfo.mbrUl;
  lcinfo.mbrDl = bearer.gbrQosInfo.mbrDl;
  lcinfo.gbrUl = bearer.gbrQosInfo.gbrUl;
  lcinfo.gbrDl = bearer.gbrQosInfo.gbrDl;
  m_rrc->m_cmacSapProvider->AddLc (lcinfo, rlc->GetLteMacSapUser ());

  if (rlcTypeId == LteRlcAm::GetTypeId ())
    {
      drbInfo->m_rlcConfig.choice =  LteRrcSap::RlcConfig::AM;
    }
  else
    {
      drbInfo->m_rlcConfig.choice =  LteRrcSap::RlcConfig::UM_BI_DIRECTIONAL;
    }

  drbInfo->m_logicalChannelIdentity = lcid;
  drbInfo->m_logicalChannelConfig.priority =  m_rrc->GetLogicalChannelPriority (bearer);
  drbInfo->m_logicalChannelConfig.logicalChannelGroup = m_rrc->GetLogicalChannelGroup (bearer);
  if (bearer.IsGbr ())
    {
      drbInfo->m_logicalChannelConfig.prioritizedBitRateKbps = bearer.gbrQosInfo.gbrUl;
    }
  else
    {
      drbInfo->m_logicalChannelConfig.prioritizedBitRateKbps = 0;
    }
  drbInfo->m_logicalChannelConfig.bucketSizeDurationMs = 1000;

  ScheduleRrcConnectionReconfiguration ();
}

void
UeManager::RecordDataRadioBearersToBeStarted ()
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      m_drbsToBeStarted.push_back (it->first);
    }
}

void
UeManager::StartDataRadioBearers ()
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti);
  for (std::list <uint8_t>::iterator drbIdIt = m_drbsToBeStarted.begin ();
       drbIdIt != m_drbsToBeStarted.end ();
       ++drbIdIt)
    {
      std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.find (*drbIdIt);
      NS_ASSERT (drbIt != m_drbMap.end ());
      drbIt->second->m_rlc->Initialize ();
      if (drbIt->second->m_pdcp)
        {
          drbIt->second->m_pdcp->Initialize ();
        }
    }
  m_drbsToBeStarted.clear ();
}


void
UeManager::ReleaseDataRadioBearer (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) m_rnti << (uint32_t) drbid);
  uint8_t lcid = Drbid2Lcid (drbid);
  std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ASSERT_MSG (it != m_drbMap.end (), "request to remove radio bearer with unknown drbid " << drbid);

  // first delete eventual X2-U TEIDs
  m_rrc->m_x2uTeidInfoMap.erase (it->second->m_gtpTeid);

  m_drbMap.erase (it);
  m_rrc->m_cmacSapProvider->ReleaseLc (m_rnti, lcid);

  LteRrcSap::RadioResourceConfigDedicated rrcd;
  rrcd.havePhysicalConfigDedicated = false;
  rrcd.drbToReleaseList.push_back (drbid);
  //populating RadioResourceConfigDedicated information element as per 3GPP TS 36.331 version 9.2.0
  rrcd.havePhysicalConfigDedicated = true;
  rrcd.physicalConfigDedicated = m_physicalConfigDedicated;
 
  //populating RRCConnectionReconfiguration message as per 3GPP TS 36.331 version 9.2.0 Release 9
  LteRrcSap::RrcConnectionReconfiguration msg;
  msg.haveMeasConfig = false;
  msg.haveMobilityControlInfo = false;
  msg.radioResourceConfigDedicated = rrcd;
  msg.haveRadioResourceConfigDedicated = true;
  //RRC Connection Reconfiguration towards UE
  m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, msg);
}

void
LteEnbRrc::DoSendReleaseDataRadioBearer (uint64_t imsi, uint16_t rnti, uint8_t bearerId)
{
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  // Bearer de-activation towards UE
  ueManager->ReleaseDataRadioBearer (bearerId);
  // Bearer de-activation indication towards epc-enb application
  m_s1SapProvider->DoSendReleaseIndication (imsi,rnti,bearerId);
}

void 
UeManager::ScheduleRrcConnectionReconfiguration ()
{

  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
    case CONNECTION_SETUP:
    case CONNECTION_RECONFIGURATION:
    case CONNECTION_REESTABLISHMENT:
    case HANDOVER_PREPARATION:
    case HANDOVER_JOINING:
    case HANDOVER_LEAVING:
      // a previous reconfiguration still ongoing, we need to wait for it to be finished
      m_pendingRrcConnectionReconfiguration = true;
      break;

    case CONNECTED_NORMALLY:
      {

        m_pendingRrcConnectionReconfiguration = false;
        LteRrcSap::RrcConnectionReconfiguration msg = BuildRrcConnectionReconfiguration ();

        m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, msg);
        RecordDataRadioBearersToBeStarted ();
        SwitchToState (CONNECTION_RECONFIGURATION);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::PrepareHandover (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  switch (m_state)
    {
    case CONNECTED_NORMALLY:
      {
        m_targetCellId = cellId;
        //A.M
        m_hysteresisHasUpdated = false;

        EpcX2SapProvider::HandoverRequestParams params;
        params.oldEnbUeX2apId = m_rnti;
        params.cause          = EpcX2SapProvider::HandoverDesirableForRadioReason;
        params.sourceCellId   = m_rrc->m_cellId;
        params.targetCellId   = cellId;
        params.mmeUeS1apId    = m_imsi;
        params.ueAggregateMaxBitRateDownlink = 200 * 1000;
        params.ueAggregateMaxBitRateUplink = 100 * 1000;
        params.bearers = GetErabList ();

        LteRrcSap::HandoverPreparationInfo hpi;
        hpi.asConfig.sourceUeIdentity = m_rnti;
        hpi.asConfig.sourceDlCarrierFreq = m_rrc->m_dlEarfcn;
        hpi.asConfig.sourceMeasConfig = m_rrc->m_ueMeasConfig;
        hpi.asConfig.sourceRadioResourceConfig = GetRadioResourceConfigForHandoverPreparationInfo ();
        hpi.asConfig.sourceMasterInformationBlock.dlBandwidth = m_rrc->m_dlBandwidth;
        hpi.asConfig.sourceMasterInformationBlock.systemFrameNumber = 0;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity = m_rrc->m_sib1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.cellIdentity = m_rrc->m_cellId;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.csgIndication = m_rrc->m_sib1.cellAccessRelatedInfo.csgIndication;
        hpi.asConfig.sourceSystemInformationBlockType1.cellAccessRelatedInfo.csgIdentity = m_rrc->m_sib1.cellAccessRelatedInfo.csgIdentity;
        LteEnbCmacSapProvider::RachConfig rc = m_rrc->m_cmacSapProvider->GetRachConfig ();
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
        hpi.asConfig.sourceSystemInformationBlockType2.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;
        hpi.asConfig.sourceSystemInformationBlockType2.freqInfo.ulCarrierFreq = m_rrc->m_ulEarfcn;
        hpi.asConfig.sourceSystemInformationBlockType2.freqInfo.ulBandwidth = m_rrc->m_ulBandwidth;
        params.rrcContext = m_rrc->m_rrcSapUser->EncodeHandoverPreparationInformation (hpi);

        NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
        NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
        NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
        NS_LOG_LOGIC ("mmeUeS1apId = " << params.mmeUeS1apId);
        NS_LOG_LOGIC ("rrcContext   = " << params.rrcContext);


        m_rrc->m_x2SapProvider->SendHandoverRequest (params);
        SwitchToState (HANDOVER_PREPARATION);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }

}

void 
UeManager::RecvHandoverRequestAck (EpcX2SapUser::HandoverRequestAckParams params)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (params.notAdmittedBearers.empty (), "not admission of some bearers upon handover is not supported");
  NS_ASSERT_MSG (params.admittedBearers.size () == m_drbMap.size (), "not enough bearers in admittedBearers");

  // note: the Handover command from the target eNB to the source eNB
  // is expected to be sent transparently to the UE; however, here we
  // decode the message and eventually reencode it. This way we can
  // support both a real RRC protocol implementation and an ideal one
  // without actual RRC protocol encoding. 

  Ptr<Packet> encodedHandoverCommand = params.rrcContext;
  LteRrcSap::RrcConnectionReconfiguration handoverCommand = m_rrc->m_rrcSapUser->DecodeHandoverCommand (encodedHandoverCommand);
  m_rrc->m_rrcSapUser->SendRrcConnectionReconfiguration (m_rnti, handoverCommand);
  SwitchToState (HANDOVER_LEAVING);
  m_handoverLeavingTimeout = Simulator::Schedule (m_rrc->m_handoverLeavingTimeoutDuration, 
                                                  &LteEnbRrc::HandoverLeavingTimeout, 
                                                  m_rrc, m_rnti);
  NS_ASSERT (handoverCommand.haveMobilityControlInfo);
  m_rrc->m_handoverStartTrace (m_imsi, m_rrc->m_cellId, m_rnti, handoverCommand.mobilityControlInfo.targetPhysCellId);

  EpcX2SapProvider::SnStatusTransferParams sst;
  sst.oldEnbUeX2apId = params.oldEnbUeX2apId;
  sst.newEnbUeX2apId = params.newEnbUeX2apId;
  sst.sourceCellId = params.sourceCellId;
  sst.targetCellId = params.targetCellId;
  for ( std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.begin ();
        drbIt != m_drbMap.end ();
        ++drbIt)
    {
      // SN status transfer is only for AM RLC
      if (0 != drbIt->second->m_rlc->GetObject<LteRlcAm> ())
        {
          LtePdcp::Status status = drbIt->second->m_pdcp->GetStatus ();
          EpcX2Sap::ErabsSubjectToStatusTransferItem i;
          i.dlPdcpSn = status.txSn;
          i.ulPdcpSn = status.rxSn;
          sst.erabsSubjectToStatusTransferList.push_back (i);
        }
    }
  m_rrc->m_x2SapProvider->SendSnStatusTransfer (sst);
}


LteRrcSap::RadioResourceConfigDedicated
UeManager::GetRadioResourceConfigForHandoverPreparationInfo ()
{
  NS_LOG_FUNCTION (this);
  return BuildRadioResourceConfigDedicated ();
}

LteRrcSap::RrcConnectionReconfiguration 
UeManager::GetRrcConnectionReconfigurationForHandover ()
{
  NS_LOG_FUNCTION (this);
  return BuildRrcConnectionReconfiguration ();
}

void
UeManager::SendData (uint8_t bid, Ptr<Packet> p)
{
  NS_LOG_FUNCTION (this << p << (uint16_t) bid);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
    case CONNECTION_SETUP:
      NS_LOG_WARN ("not connected, discarding packet");
      return;
      break;

    case CONNECTED_NORMALLY:
    case CONNECTION_RECONFIGURATION:
    case CONNECTION_REESTABLISHMENT:
    case HANDOVER_PREPARATION:
    case HANDOVER_JOINING:
    case HANDOVER_PATH_SWITCH:
      {
        NS_LOG_LOGIC ("queueing data on PDCP for transmission over the air");
        LtePdcpSapProvider::TransmitPdcpSduParameters params;
        params.pdcpSdu = p;
        params.rnti = m_rnti;
        params.lcid = Bid2Lcid (bid);
        uint8_t drbid = Bid2Drbid (bid);
        //Transmit PDCP sdu only if DRB ID found in drbMap
        std::map<uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
        if (it != m_drbMap.end ())
          {
            Ptr<LteDataRadioBearerInfo> bearerInfo = GetDataRadioBearerInfo (drbid);
            if (bearerInfo != NULL)
              {
                LtePdcpSapProvider* pdcpSapProvider = bearerInfo->m_pdcp->GetLtePdcpSapProvider ();
        pdcpSapProvider->TransmitPdcpSdu (params);
      }
          }
      }
      break;

    case HANDOVER_LEAVING:
      {
        NS_LOG_LOGIC ("forwarding data to target eNB over X2-U");
        uint8_t drbid = Bid2Drbid (bid);
        EpcX2Sap::UeDataParams params;
        params.sourceCellId = m_rrc->m_cellId;
        params.targetCellId = m_targetCellId;
        params.gtpTeid = GetDataRadioBearerInfo (drbid)->m_gtpTeid;
        params.ueData = p;
        m_rrc->m_x2SapProvider->SendUeData (params);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

std::vector<EpcX2Sap::ErabToBeSetupItem>
UeManager::GetErabList ()
{
  NS_LOG_FUNCTION (this);
  std::vector<EpcX2Sap::ErabToBeSetupItem> ret;
  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it =  m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      EpcX2Sap::ErabToBeSetupItem etbsi;
      etbsi.erabId = it->second->m_epsBearerIdentity;
      etbsi.erabLevelQosParameters = it->second->m_epsBearer;
      etbsi.dlForwarding = false;
      etbsi.transportLayerAddress = it->second->m_transportLayerAddress;
      etbsi.gtpTeid = it->second->m_gtpTeid;
      ret.push_back (etbsi);
    }
  return ret;
}

void
UeManager::SendUeContextRelease ()
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case HANDOVER_PATH_SWITCH:
      NS_LOG_INFO ("Send UE CONTEXT RELEASE from target eNB to source eNB");
      EpcX2SapProvider::UeContextReleaseParams ueCtxReleaseParams;
      ueCtxReleaseParams.oldEnbUeX2apId = m_sourceX2apId;
      ueCtxReleaseParams.newEnbUeX2apId = m_rnti;
      ueCtxReleaseParams.sourceCellId = m_sourceCellId;
      m_rrc->m_x2SapProvider->SendUeContextRelease (ueCtxReleaseParams);
      SwitchToState (CONNECTED_NORMALLY);
      m_rrc->m_handoverEndOkTrace (m_imsi, m_rrc->m_cellId, m_rnti);
     //A.M
     // m_NumHOOKPerCellCounter++;
      //m_AvgNumHOOKPerCell.insert(std::pair<double,uint16_t>(Simulator::Now().GetSeconds(),m_NumHOOKPerCellCounter))
      //A.M
      m_rrc->m_numberUePerEnb++;
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvHandoverPreparationFailure (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  switch (m_state)
    {
    case HANDOVER_PREPARATION:
      NS_ASSERT (cellId == m_targetCellId);
      NS_LOG_INFO ("target eNB sent HO preparation failure, aborting HO");
      SwitchToState (CONNECTED_NORMALLY);
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvSnStatusTransfer (EpcX2SapUser::SnStatusTransferParams params)
{
  NS_LOG_FUNCTION (this);
  for (std::vector<EpcX2Sap::ErabsSubjectToStatusTransferItem>::iterator erabIt 
         = params.erabsSubjectToStatusTransferList.begin ();
       erabIt != params.erabsSubjectToStatusTransferList.end ();
       ++erabIt)
    {
      // LtePdcp::Status status;
      // status.txSn = erabIt->dlPdcpSn;
      // status.rxSn = erabIt->ulPdcpSn;
      // uint8_t drbId = Bid2Drbid (erabIt->erabId);
      // std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator drbIt = m_drbMap.find (drbId);
      // NS_ASSERT_MSG (drbIt != m_drbMap.end (), "could not find DRBID " << (uint32_t) drbId);
      // drbIt->second->m_pdcp->SetStatus (status);
    }
}

void 
UeManager::RecvUeContextRelease (EpcX2SapUser::UeContextReleaseParams params)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_state == HANDOVER_LEAVING, "method unexpected in state " << ToString (m_state));
  m_handoverLeavingTimeout.Cancel ();
}


// methods forwarded from RRC SAP

void 
UeManager::CompleteSetupUe (LteEnbRrcSapProvider::CompleteSetupUeParameters params)
{
  NS_LOG_FUNCTION (this);
  m_srb0->m_rlc->SetLteRlcSapUser (params.srb0SapUser);
  m_srb1->m_pdcp->SetLtePdcpSapUser (params.srb1SapUser);
}

void
UeManager::RecvRrcConnectionRequest (LteRrcSap::RrcConnectionRequest msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      {
        m_connectionRequestTimeout.Cancel ();

        if (m_rrc->m_admitRrcConnectionRequest == true)
          {
            m_imsi = msg.ueIdentity;
            if (m_rrc->m_s1SapProvider != 0)
              {
                m_rrc->m_s1SapProvider->InitialUeMessage (m_imsi, m_rnti);
              }

            // send RRC CONNECTION SETUP to UE
            LteRrcSap::RrcConnectionSetup msg2;
            msg2.rrcTransactionIdentifier = GetNewRrcTransactionIdentifier ();
            msg2.radioResourceConfigDedicated = BuildRadioResourceConfigDedicated ();
            m_rrc->m_rrcSapUser->SendRrcConnectionSetup (m_rnti, msg2);

            RecordDataRadioBearersToBeStarted ();
            m_connectionSetupTimeout = Simulator::Schedule (
                m_rrc->m_connectionSetupTimeoutDuration,
                &LteEnbRrc::ConnectionSetupTimeout, m_rrc, m_rnti);
            SwitchToState (CONNECTION_SETUP);
          }
        else
          {
            NS_LOG_INFO ("rejecting connection request for RNTI " << m_rnti);

            // send RRC CONNECTION REJECT to UE
            LteRrcSap::RrcConnectionReject rejectMsg;
            rejectMsg.waitTime = 3;
            m_rrc->m_rrcSapUser->SendRrcConnectionReject (m_rnti, rejectMsg);

            m_connectionRejectedTimeout = Simulator::Schedule (
                m_rrc->m_connectionRejectedTimeoutDuration,
                &LteEnbRrc::ConnectionRejectedTimeout, m_rrc, m_rnti);
            SwitchToState (CONNECTION_REJECTED);
          }
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void
UeManager::RecvRrcConnectionSetupCompleted (LteRrcSap::RrcConnectionSetupCompleted msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTION_SETUP:
      m_connectionSetupTimeout.Cancel ();
      StartDataRadioBearers ();
      SwitchToState (CONNECTED_NORMALLY);
      m_rrc->m_connectionEstablishedTrace (m_imsi, m_rrc->m_cellId, m_rnti);
      m_rrc->m_numberUePerEnb++;
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void
UeManager::RecvRrcConnectionReconfigurationCompleted (LteRrcSap::RrcConnectionReconfigurationCompleted msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTION_RECONFIGURATION:
      StartDataRadioBearers ();
      if (m_needPhyMacConfiguration)
        {
          // configure MAC (and scheduler)
          LteEnbCmacSapProvider::UeConfig req;
          req.m_rnti = m_rnti;
          req.m_transmissionMode = m_physicalConfigDedicated.antennaInfo.transmissionMode;
          m_rrc->m_cmacSapProvider->UeUpdateConfigurationReq (req);

          // configure PHY
          m_rrc->m_cphySapProvider->SetTransmissionMode (req.m_rnti, req.m_transmissionMode);

          double paDouble = LteRrcSap::ConvertPdschConfigDedicated2Double (m_physicalConfigDedicated.pdschConfigDedicated);
          m_rrc->m_cphySapProvider->SetPa (m_rnti, paDouble);

          m_needPhyMacConfiguration = false;
        }
      SwitchToState (CONNECTED_NORMALLY);
      m_rrc->m_connectionReconfigurationTrace (m_imsi, m_rrc->m_cellId, m_rnti);
      break;

    // This case is added to NS-3 in order to handle bearer de-activation scenario for CONNECTED state UE
    case CONNECTED_NORMALLY:
      NS_LOG_INFO ("ignoring RecvRrcConnectionReconfigurationCompleted in state " << ToString (m_state));
      break;

    case HANDOVER_LEAVING:
      NS_LOG_INFO ("ignoring RecvRrcConnectionReconfigurationCompleted in state " << ToString (m_state));
      break;

    case HANDOVER_JOINING:
      {
        m_handoverJoiningTimeout.Cancel ();
        NS_LOG_INFO ("Send PATH SWITCH REQUEST to the MME");
        EpcEnbS1SapProvider::PathSwitchRequestParameters params;
        params.rnti = m_rnti;
        params.cellId = m_rrc->m_cellId;
        params.mmeUeS1Id = m_imsi;
        SwitchToState (HANDOVER_PATH_SWITCH);
        for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it =  m_drbMap.begin ();
             it != m_drbMap.end ();
             ++it)
          {
            EpcEnbS1SapProvider::BearerToBeSwitched b;
            b.epsBearerId = it->second->m_epsBearerIdentity;
            b.teid =  it->second->m_gtpTeid;
            params.bearersToBeSwitched.push_back (b);
          }
        m_rrc->m_s1SapProvider->PathSwitchRequest (params);
      }
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }
}

void 
UeManager::RecvRrcConnectionReestablishmentRequest (LteRrcSap::RrcConnectionReestablishmentRequest msg)
{
  NS_LOG_FUNCTION (this);
  switch (m_state)
    {
    case CONNECTED_NORMALLY:
      break;

    case HANDOVER_LEAVING:
      m_handoverLeavingTimeout.Cancel ();
      break;

    default:
      NS_FATAL_ERROR ("method unexpected in state " << ToString (m_state));
      break;
    }

  LteRrcSap::RrcConnectionReestablishment msg2;
  msg2.rrcTransactionIdentifier = GetNewRrcTransactionIdentifier ();
  msg2.radioResourceConfigDedicated = BuildRadioResourceConfigDedicated ();
  m_rrc->m_rrcSapUser->SendRrcConnectionReestablishment (m_rnti, msg2);
  SwitchToState (CONNECTION_REESTABLISHMENT);
}

void 
UeManager::RecvRrcConnectionReestablishmentComplete (LteRrcSap::RrcConnectionReestablishmentComplete msg)
{
  NS_LOG_FUNCTION (this);
  SwitchToState (CONNECTED_NORMALLY);
}

void 
UeManager::RecvMeasurementReport (LteRrcSap::MeasurementReport msg)
{

  uint8_t measId = msg.measResults.measId;
  NS_LOG_FUNCTION (this << (uint16_t) measId);
  NS_LOG_LOGIC ("measId " << (uint16_t) measId
                          << " haveMeasResultNeighCells " << msg.measResults.haveMeasResultNeighCells
                          << " measResultListEutra " << msg.measResults.measResultListEutra.size ());
  NS_LOG_LOGIC ("serving cellId " << m_rrc->m_cellId
                                  << " RSRP " << (uint16_t) msg.measResults.rsrpResult
                                  << " RSRQ " << (uint16_t) msg.measResults.rsrqResult);

  m_servingRSRP = msg.measResults.rsrpResult;
//  std::cout<<"m_servingRSRP = "<<m_servingRSRP<<" and rsrpResult = "<<(double)msg.measResults.rsrpResult<<std::endl;

  for (std::list <LteRrcSap::MeasResultEutra>::iterator it = msg.measResults.measResultListEutra.begin ();
       it != msg.measResults.measResultListEutra.end ();
       ++it)
    {
      NS_LOG_LOGIC ("neighbour cellId " << it->physCellId
                                        << " RSRP " << (it->haveRsrpResult ? (uint16_t) it->rsrpResult : 255)
                                        << " RSRQ " << (it->haveRsrqResult ? (uint16_t) it->rsrqResult : 255));
     if(it->haveRsrpResult)
     {
		  m_neighRSRP.push_back(it->rsrpResult);
		  m_neighRSRPId.push_back(it->physCellId);
		  m_neighRSRPMap.insert(std::pair<uint16_t, uint8_t > (it->physCellId,it->rsrpResult));
     }

    }

  if ((m_rrc->m_handoverManagementSapProvider != 0)
      && (m_rrc->m_handoverMeasIds.find (measId) != m_rrc->m_handoverMeasIds.end ()))
    {
      // this measurement was requested by the handover algorithm
      m_rrc->m_handoverManagementSapProvider->ReportUeMeas (m_rnti,
                                                            msg.measResults);
    }

  if ((m_rrc->m_anrSapProvider != 0)
      && (m_rrc->m_anrMeasIds.find (measId) != m_rrc->m_anrMeasIds.end ()))
    {
      // this measurement was requested by the ANR function
      m_rrc->m_anrSapProvider->ReportUeMeas (msg.measResults);
    }

  if ((m_rrc->m_ffrRrcSapProvider != 0)
      && (m_rrc->m_ffrMeasIds.find (measId) != m_rrc->m_ffrMeasIds.end ()))
    {
      // this measurement was requested by the FFR function
      m_rrc->m_ffrRrcSapProvider->ReportUeMeas (m_rnti, msg.measResults);
    }

  // fire a trace source
  m_rrc->m_recvMeasurementReportTrace (m_imsi, m_rrc->m_cellId, m_rnti, msg);

} // end of UeManager::RecvMeasurementReport

double
UeManager::GetServingRSRPValue ()
{
	return m_servingRSRP;
}
std::map <uint16_t, uint8_t>
UeManager::GetNeighRSRPValue()
{

/*	std::vector<uint8_t>::iterator it2 = m_neighRSRP.begin();

	for (std::vector<uint8_t>::iterator it = m_neighRSRPId.begin(); it != m_neighRSRPId.end(); ++it)
	{


		if((cellId == *it) && (it2 != m_neighRSRP.end()))
		{
			return *it2;
		}
		++it2;
	}
*/

	return m_neighRSRPMap;
}

void
UeManager::SetHysteresisValue(double newHys)
{
	std::cout<<Simulator::Now().GetSeconds()<< ": In SetHysteresisValue with Hysteresis = "<< newHys << std::endl;
	m_hysteresis = newHys;
	m_hysteresisHasUpdated = true;
}


// methods forwarded from CMAC SAP

void
UeManager::CmacUeConfigUpdateInd (LteEnbCmacSapUser::UeConfig cmacParams)
{
  NS_LOG_FUNCTION (this << m_rnti);
  // at this stage used only by the scheduler for updating txMode

  m_physicalConfigDedicated.antennaInfo.transmissionMode = cmacParams.m_transmissionMode;

  m_needPhyMacConfiguration = true;

  // reconfigure the UE RRC
  ScheduleRrcConnectionReconfiguration ();
}


// methods forwarded from PDCP SAP

void
UeManager::DoReceivePdcpSdu (LtePdcpSapUser::ReceivePdcpSduParameters params)
{
  NS_LOG_FUNCTION (this);
  if (params.lcid > 2)
    {
      // data radio bearer
      EpsBearerTag tag;
      tag.SetRnti (params.rnti);
      tag.SetBid (Lcid2Bid (params.lcid));
      params.pdcpSdu->AddPacketTag (tag);
      m_rrc->m_forwardUpCallback (params.pdcpSdu);
    }
}


uint16_t
UeManager::GetRnti (void) const
{
  return m_rnti;
}

uint64_t
UeManager::GetImsi (void) const
{
  return m_imsi;
}

uint16_t
UeManager::GetSrsConfigurationIndex (void) const
{
  return m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex;
}

void
UeManager::SetSrsConfigurationIndex (uint16_t srsConfIndex)
{
  NS_LOG_FUNCTION (this);
  m_physicalConfigDedicated.soundingRsUlConfigDedicated.srsConfigIndex = srsConfIndex;
  m_rrc->m_cphySapProvider->SetSrsConfigurationIndex (m_rnti, srsConfIndex);
  switch (m_state)
    {
    case INITIAL_RANDOM_ACCESS:
      // do nothing, srs conf index will be correctly enforced upon
      // RRC connection establishment
      break;

    default:
      ScheduleRrcConnectionReconfiguration ();
      break;
    }
}

UeManager::State
UeManager::GetState (void) const
{
  return m_state;
}

void
UeManager::SetPdschConfigDedicated (LteRrcSap::PdschConfigDedicated pdschConfigDedicated)
{
  NS_LOG_FUNCTION (this);
  m_physicalConfigDedicated.pdschConfigDedicated = pdschConfigDedicated;

  m_needPhyMacConfiguration = true;

  // reconfigure the UE RRC
  ScheduleRrcConnectionReconfiguration ();
}

uint8_t
UeManager::AddDataRadioBearerInfo (Ptr<LteDataRadioBearerInfo> drbInfo)
{
  NS_LOG_FUNCTION (this);
  const uint8_t MAX_DRB_ID = 32;
  for (int drbid = (m_lastAllocatedDrbid + 1) % MAX_DRB_ID; 
       drbid != m_lastAllocatedDrbid; 
       drbid = (drbid + 1) % MAX_DRB_ID)
    {
      if (drbid != 0) // 0 is not allowed
        {
          if (m_drbMap.find (drbid) == m_drbMap.end ())
            {
              m_drbMap.insert (std::pair<uint8_t, Ptr<LteDataRadioBearerInfo> > (drbid, drbInfo));
              drbInfo->m_drbIdentity = drbid;
              m_lastAllocatedDrbid = drbid;
              return drbid;
            }
        }
    }
  NS_FATAL_ERROR ("no more data radio bearer ids available");
  return 0;
}

Ptr<LteDataRadioBearerInfo>
UeManager::GetDataRadioBearerInfo (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) drbid);
  NS_ASSERT (0 != drbid);
  std::map<uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ABORT_IF (it == m_drbMap.end ());
  return it->second;
}


void
UeManager::RemoveDataRadioBearerInfo (uint8_t drbid)
{
  NS_LOG_FUNCTION (this << (uint32_t) drbid);
  std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.find (drbid);
  NS_ASSERT_MSG (it != m_drbMap.end (), "request to remove radio bearer with unknown drbid " << drbid);
  m_drbMap.erase (it);
}


LteRrcSap::RrcConnectionReconfiguration
UeManager::BuildRrcConnectionReconfiguration ()
{
  LteRrcSap::RrcConnectionReconfiguration msg;
  msg.rrcTransactionIdentifier 			= GetNewRrcTransactionIdentifier ();
  msg.haveRadioResourceConfigDedicated 	= true;
  msg.radioResourceConfigDedicated 		= BuildRadioResourceConfigDedicated ();
  msg.haveMobilityControlInfo 			= false;
  msg.haveMeasConfig 					= true;
  msg.measConfig 						= m_rrc->m_ueMeasConfig;
  msg.newHysteresis 					= m_hysteresis;

  if(m_hysteresisHasUpdated)
  {
	  msg.newHysteresisHasValue			= true;
  }
  else
  {
	  msg.newHysteresisHasValue			= false;
  }
  /*
  if(m_hysteresis != m_defaultHyst)
  {
	  msg.newHysteresisHasValue			= true;
	 // std::cout<<Simulator::Now().GetSeconds()<<": UE ID = "<<m_imsi<<" with newHysteresisHasValue = true"<<std::endl;
  }
  else
  {
	  msg.newHysteresisHasValue			= false;
  }
  */
  return msg;
}

LteRrcSap::RadioResourceConfigDedicated
UeManager::BuildRadioResourceConfigDedicated ()
{
  LteRrcSap::RadioResourceConfigDedicated rrcd;

  if (m_srb1 != 0)
    {
      LteRrcSap::SrbToAddMod stam;
      stam.srbIdentity = m_srb1->m_srbIdentity;
      stam.logicalChannelConfig = m_srb1->m_logicalChannelConfig;
      rrcd.srbToAddModList.push_back (stam);
    }

  for (std::map <uint8_t, Ptr<LteDataRadioBearerInfo> >::iterator it = m_drbMap.begin ();
       it != m_drbMap.end ();
       ++it)
    {
      LteRrcSap::DrbToAddMod dtam;
      dtam.epsBearerIdentity = it->second->m_epsBearerIdentity;
      dtam.drbIdentity = it->second->m_drbIdentity;
      dtam.rlcConfig = it->second->m_rlcConfig;
      dtam.logicalChannelIdentity = it->second->m_logicalChannelIdentity;
      dtam.logicalChannelConfig = it->second->m_logicalChannelConfig;
      rrcd.drbToAddModList.push_back (dtam);
    }

  rrcd.havePhysicalConfigDedicated = true;
  rrcd.physicalConfigDedicated = m_physicalConfigDedicated;
  return rrcd;
}

uint8_t 
UeManager::GetNewRrcTransactionIdentifier ()
{
  return ++m_lastRrcTransactionIdentifier;
}

uint8_t 
UeManager::Lcid2Drbid (uint8_t lcid)
{
  NS_ASSERT (lcid > 2);
  return lcid - 2;
}

uint8_t 
UeManager::Drbid2Lcid (uint8_t drbid)
{
  return drbid + 2;
}
uint8_t 
UeManager::Lcid2Bid (uint8_t lcid)
{
  NS_ASSERT (lcid > 2);
  return lcid - 2;
}

uint8_t 
UeManager::Bid2Lcid (uint8_t bid)
{
  return bid + 2;
}

uint8_t 
UeManager::Drbid2Bid (uint8_t drbid)
{
  return drbid;
}

uint8_t 
UeManager::Bid2Drbid (uint8_t bid)
{
  return bid;
}


void 
UeManager::SwitchToState (State newState)
{
  NS_LOG_FUNCTION (this << ToString (newState));
  State oldState = m_state;
  m_state = newState;
  NS_LOG_INFO (this << " IMSI " << m_imsi << " RNTI " << m_rnti << " UeManager "
                    << ToString (oldState) << " --> " << ToString (newState));
  m_stateTransitionTrace (m_imsi, m_rrc->m_cellId, m_rnti, oldState, newState);

  switch (newState)
    {
    case INITIAL_RANDOM_ACCESS:
    case HANDOVER_JOINING:
      NS_FATAL_ERROR ("cannot switch to an initial state");
      break;

    case CONNECTION_SETUP:
      break;

    case CONNECTED_NORMALLY:
      {
        if (m_pendingRrcConnectionReconfiguration == true)
          {
            ScheduleRrcConnectionReconfiguration ();
          }
      }
      break;

    case CONNECTION_RECONFIGURATION:
      break;

    case CONNECTION_REESTABLISHMENT:
      break;

    case HANDOVER_LEAVING:
      break;

    default:
      break;
    }
}



///////////////////////////////////////////
// eNB RRC methods
///////////////////////////////////////////

NS_OBJECT_ENSURE_REGISTERED (LteEnbRrc);

LteEnbRrc::LteEnbRrc ()
  : m_x2SapProvider (0),
    m_cmacSapProvider (0),
    m_handoverManagementSapProvider (0),
    m_anrSapProvider (0),
    m_ffrRrcSapProvider (0),
    m_rrcSapUser (0),
    m_macSapProvider (0),
    m_s1SapProvider (0),
    m_cphySapProvider (0),
    m_configured (false),
    m_lastAllocatedRnti (0),
    m_srsCurrentPeriodicityId (0),
    m_lastAllocatedConfigurationIndex (0),
    m_reconfigureUes (false)
{
  NS_LOG_FUNCTION (this);
  m_cmacSapUser 	= new EnbRrcMemberLteEnbCmacSapUser (this);
  m_handoverManagementSapUser = new MemberLteHandoverManagementSapUser<LteEnbRrc> (this);
  m_anrSapUser 		= new MemberLteAnrSapUser<LteEnbRrc> (this);
  m_ffrRrcSapUser 	= new MemberLteFfrRrcSapUser<LteEnbRrc> (this);
  m_rrcSapProvider 	= new MemberLteEnbRrcSapProvider<LteEnbRrc> (this);
  m_x2SapUser 		= new EpcX2SpecificEpcX2SapUser<LteEnbRrc> (this);
  m_s1SapUser 		= new MemberEpcEnbS1SapUser<LteEnbRrc> (this);
  m_cphySapUser 	= new MemberLteEnbCphySapUser<LteEnbRrc> (this);

 std::map<uint16_t, uint16_t>::iterator it= m_aliveMap.begin();

  for(int j = 1; j <= m_numberOfEnbs;j++, ++it)
  {
	   m_aliveMap.insert(std::pair<uint16_t, uint16_t>(j,1));
  }



     //Simulator::Schedule (MilliSeconds(10355), &LteEnbRrc::TriggerRLFDetection, this);
      Simulator::Schedule (MilliSeconds(200355), &LteEnbRrc::TriggerRLFDetection, this);

  	 Simulator::Schedule (MilliSeconds(20), &LteEnbRrc::ActIfPowerDown, this);




	 // Simulator::Schedule (MilliSeconds(400), &LteEnbRrc::TriggerSetPowerFromCodeForTesting, this);
  	 // Simulator::Schedule (MilliSeconds(600), &LteEnbRrc::CalAvgRSRPPerCell, this); // be carfull to just call for active eNB and not down
}


LteEnbRrc::~LteEnbRrc ()
{
  NS_LOG_FUNCTION (this);
}


void
LteEnbRrc::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_ueMap.clear ();
  delete m_cmacSapUser;
  delete m_handoverManagementSapUser;
  delete m_anrSapUser;
  delete m_ffrRrcSapUser;
  delete m_rrcSapProvider;
  delete m_x2SapUser;
  delete m_s1SapUser;
  delete m_cphySapUser;
}

TypeId
LteEnbRrc::GetTypeId (void)
{
  NS_LOG_FUNCTION ("LteEnbRrc::GetTypeId");
  static TypeId tid = TypeId ("ns3::LteEnbRrc")
    .SetParent<Object> ()
    .SetGroupName("Lte")
    .AddConstructor<LteEnbRrc> ()
    .AddAttribute ("UeMap", "List of UeManager by C-RNTI.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&LteEnbRrc::m_ueMap),
                   MakeObjectMapChecker<UeManager> ())
    .AddAttribute ("DefaultTransmissionMode",
                   "The default UEs' transmission mode (0: SISO)",
                   UintegerValue (0),  // default tx-mode
                   MakeUintegerAccessor (&LteEnbRrc::m_defaultTransmissionMode),
                   MakeUintegerChecker<uint8_t> ())
    .AddAttribute ("EpsBearerToRlcMapping", 
                   "Specify which type of RLC will be used for each type of EPS bearer. ",
                   EnumValue (RLC_SM_ALWAYS),
                   MakeEnumAccessor (&LteEnbRrc::m_epsBearerToRlcMapping),
                   MakeEnumChecker (RLC_SM_ALWAYS, "RlcSmAlways",
                                    RLC_UM_ALWAYS, "RlcUmAlways",
                                    RLC_AM_ALWAYS, "RlcAmAlways",
                                    PER_BASED,     "PacketErrorRateBased"))
    .AddAttribute ("SystemInformationPeriodicity",
                   "The interval for sending system information (Time value)",
                   TimeValue (MilliSeconds (80)),
                   MakeTimeAccessor (&LteEnbRrc::m_systemInformationPeriodicity),
                   MakeTimeChecker ())

    // SRS related attributes
    .AddAttribute ("SrsPeriodicity",
                   "The SRS periodicity in milliseconds",
                   UintegerValue (40),
                   MakeUintegerAccessor (&LteEnbRrc::SetSrsPeriodicity, 
                                         &LteEnbRrc::GetSrsPeriodicity),
                   MakeUintegerChecker<uint32_t> ())

    // Timeout related attributes
    .AddAttribute ("ConnectionRequestTimeoutDuration",
                   "After a RA attempt, if no RRC CONNECTION REQUEST is "
                   "received before this time, the UE context is destroyed. "
                   "Must account for reception of RAR and transmission of "
                   "RRC CONNECTION REQUEST over UL GRANT.",
                   TimeValue (MilliSeconds (15)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionRequestTimeoutDuration),
                   MakeTimeChecker ())

    .AddAttribute ("ConnectionSetupTimeoutDuration",
                   "After accepting connection request, if no RRC CONNECTION "
                   "SETUP COMPLETE is received before this time, the UE "
                   "context is destroyed. Must account for the UE's reception "
                   "of RRC CONNECTION SETUP and transmission of RRC CONNECTION "
                   "SETUP COMPLETE.",
                   TimeValue (MilliSeconds (150)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionSetupTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("ConnectionRejectedTimeoutDuration",
                   "Time to wait between sending a RRC CONNECTION REJECT and "
                   "destroying the UE context",
                   TimeValue (MilliSeconds (30)),
                   MakeTimeAccessor (&LteEnbRrc::m_connectionRejectedTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("HandoverJoiningTimeoutDuration",
                   "After accepting a handover request, if no RRC CONNECTION "
                   "RECONFIGURATION COMPLETE is received before this time, the "
                   "UE context is destroyed. Must account for reception of "
                   "X2 HO REQ ACK by source eNB, transmission of the Handover "
                   "Command, non-contention-based random access and reception "
                   "of the RRC CONNECTION RECONFIGURATION COMPLETE message.",
                   TimeValue (MilliSeconds (200)),
                   MakeTimeAccessor (&LteEnbRrc::m_handoverJoiningTimeoutDuration),
                   MakeTimeChecker ())
    .AddAttribute ("HandoverLeavingTimeoutDuration",
                   "After issuing a Handover Command, if neither RRC "
                   "CONNECTION RE-ESTABLISHMENT nor X2 UE Context Release has "
                   "been previously received, the UE context is destroyed.",
                   TimeValue (MilliSeconds (500)),
                   MakeTimeAccessor (&LteEnbRrc::m_handoverLeavingTimeoutDuration),
                   MakeTimeChecker ())

    // Cell selection related attribute
    .AddAttribute ("QRxLevMin",
                   "One of information transmitted within the SIB1 message, "
                   "indicating the required minimum RSRP level that any UE must "
                   "receive from this cell before it is allowed to camp to this "
                   "cell. The default value -70 corresponds to -140 dBm and is "
                   "the lowest possible value as defined by Section 6.3.4 of "
                   "3GPP TS 36.133. This restriction, however, only applies to "
                   "initial cell selection and EPC-enabled simulation.",
                   TypeId::ATTR_GET | TypeId::ATTR_CONSTRUCT,
                   IntegerValue (-70),
                   MakeIntegerAccessor (&LteEnbRrc::m_qRxLevMin),
                   MakeIntegerChecker<int8_t> (-70, -22))

	//A.M timeout for ACK should be bigger than for NACK
	.AddAttribute ("AckForFailureTokenTimeoutDuration",
				   "After a FailureDetectionToken is sent, if  ACK received back "
				   "before this timer expires then we cancel the timer and if"
				   "no ACK received then a call for ACKNotReceivedTimeout() is done",
				   TimeValue (MilliSeconds (400)),
				   MakeTimeAccessor (&LteEnbRrc::m_ackForFailureTokenTimeoutDuration),
				   MakeTimeChecker ())

	//A.M timeout for checking the m_alivemap and take decision
	.AddAttribute ("RLFDecisionDuration",
				   "After sending link failure detection tokens and when this time expires "
				   "the cell should take decision on the alive and unresponsive cells",
				   TimeValue (MilliSeconds (600)),
				   MakeTimeAccessor (&LteEnbRrc::m_RLFDecisionDuration),
				   MakeTimeChecker ())
	//A.M
	.AddAttribute ("NoForwardFailureTokenTimeoutDuration",
				   "According to the forwarding list X should forward a Token for me, start this timer"
				   "and if it expires before I receive a forwarded message from X then call"
				   "LteEnbRrc::NoForwardTokenReceivedTimeout() send NACK to Token owner.",
				   TimeValue (MilliSeconds (300)),
				   MakeTimeAccessor (&LteEnbRrc::m_NoForwardFailureTokenTimeoutDuration),
				   MakeTimeChecker ())
/**
 *
 */  //
	/*.AddAttribute ("AdmitResourceStatusRequest",
				   "Whether to admit an X2 resource status request from another eNB",
				   BooleanValue (true),
				   MakeBooleanAccessor (&LteEnbRrc::m_admitResourceStatusRequest),
				   MakeBooleanChecker ())
*/
	.AddAttribute ("AdmitResourceStatusRequest",
				   "Whether to admit an X2 resource status update from another eNB",
					BooleanValue (true),
					MakeBooleanAccessor (&LteEnbRrc::m_admitResourceStatusUpdate),
					MakeBooleanChecker ())

    // Handover related attributes
    .AddAttribute ("AdmitHandoverRequest",
                   "Whether to admit an X2 handover request from another eNB",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteEnbRrc::m_admitHandoverRequest),
                   MakeBooleanChecker ())
    .AddAttribute ("AdmitRrcConnectionRequest",
                   "Whether to admit a connection request from a UE",
                   BooleanValue (true),
                   MakeBooleanAccessor (&LteEnbRrc::m_admitRrcConnectionRequest),
                   MakeBooleanChecker ())

    // UE measurements related attributes
    .AddAttribute ("RsrpFilterCoefficient",
                   "Determines the strength of smoothing effect induced by "
                   "layer 3 filtering of RSRP in all attached UE; "
                   "if set to 0, no layer 3 filtering is applicable",
                   // i.e. the variable k in 3GPP TS 36.331 section 5.5.3.2
                   UintegerValue (4),
                   MakeUintegerAccessor (&LteEnbRrc::m_rsrpFilterCoefficient),
                   MakeUintegerChecker<uint8_t> (0))
    .AddAttribute ("RsrqFilterCoefficient",
                   "Determines the strength of smoothing effect induced by "
                   "layer 3 filtering of RSRQ in all attached UE; "
                   "if set to 0, no layer 3 filtering is applicable",
                   // i.e. the variable k in 3GPP TS 36.331 section 5.5.3.2
                   UintegerValue (4),
                   MakeUintegerAccessor (&LteEnbRrc::m_rsrqFilterCoefficient),
                   MakeUintegerChecker<uint8_t> (0))

    // Trace sources
    .AddTraceSource ("NewUeContext",
                     "Fired upon creation of a new UE context.",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_newUeContextTrace),
                     "ns3::LteEnbRrc::NewUeContextTracedCallback")
    .AddTraceSource ("ConnectionEstablished",
                     "Fired upon successful RRC connection establishment.",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_connectionEstablishedTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("ConnectionReconfiguration",
                     "trace fired upon RRC connection reconfiguration",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_connectionReconfigurationTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("HandoverStart",
                     "trace fired upon start of a handover procedure",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_handoverStartTrace),
                     "ns3::LteEnbRrc::HandoverStartTracedCallback")
    .AddTraceSource ("HandoverEndOk",
                     "trace fired upon successful termination of a handover procedure",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_handoverEndOkTrace),
                     "ns3::LteEnbRrc::ConnectionHandoverTracedCallback")
    .AddTraceSource ("RecvMeasurementReport",
                     "trace fired when measurement report is received",
                     MakeTraceSourceAccessor (&LteEnbRrc::m_recvMeasurementReportTrace),
                     "ns3::LteEnbRrc::ReceiveReportTracedCallback")

	/**.AddTraceSource ("CheckMlbCondtion_1",
					 "trace fired when Check MlB Condition_1 is received",
					 MakeTraceSourceAccessor (&LteEnbRrc::DoCheckMlbCondition_1),
					 "ns3::LteEnbRrc::DoCheckMlbCondition_1")
	 */
  ;
  return tid;
}

//A.M
void
LteEnbRrc::TriggerSetPowerFromCodeForTesting ()
{
	m_cphySapProvider->SetTxPower(43.0);
}
//A.M
void
LteEnbRrc::UpdateIntraEnbRelation(uint16_t cellId)
{
	switch(cellId)
	{
	case 1:
		m_anrSapProvider->SetNoX2(2);
		m_anrSapProvider->SetNoX2(3);
		break;
	case 2:
		m_anrSapProvider->SetNoX2(1);
		m_anrSapProvider->SetNoX2(3);
		break;
	case 3:
		m_anrSapProvider->SetNoX2(1);
		m_anrSapProvider->SetNoX2(2);
		break;
	case 4:
		m_anrSapProvider->SetNoX2(5);
		m_anrSapProvider->SetNoX2(6);
		break;
	case 5:
		m_anrSapProvider->SetNoX2(6);
		m_anrSapProvider->SetNoX2(4);
		break;
	case 6:
		m_anrSapProvider->SetNoX2(4);
		m_anrSapProvider->SetNoX2(5);
		break;
	case 7:
		m_anrSapProvider->SetNoX2(8);
		m_anrSapProvider->SetNoX2(9);
		break;
	case 8:
		m_anrSapProvider->SetNoX2(7);
		m_anrSapProvider->SetNoX2(9);
		break;
	case 9:
		m_anrSapProvider->SetNoX2(7);
		m_anrSapProvider->SetNoX2(8);
		break;
	case 10:
		m_anrSapProvider->SetNoX2(11);
		m_anrSapProvider->SetNoX2(12);
		break;
	case 11:
		m_anrSapProvider->SetNoX2(10);
		m_anrSapProvider->SetNoX2(12);
		break;
	case 12:
		m_anrSapProvider->SetNoX2(10);
		m_anrSapProvider->SetNoX2(11);
		break;
	case 13:
		m_anrSapProvider->SetNoX2(14);
		m_anrSapProvider->SetNoX2(15);
		break;
	case 14:
		m_anrSapProvider->SetNoX2(13);
		m_anrSapProvider->SetNoX2(15);
		break;
	case 15:
		m_anrSapProvider->SetNoX2(13);
		m_anrSapProvider->SetNoX2(14);
		break;
	case 16:
		m_anrSapProvider->SetNoX2(17);
		m_anrSapProvider->SetNoX2(18);
		break;
	case 17:
		m_anrSapProvider->SetNoX2(16);
		m_anrSapProvider->SetNoX2(18);
		break;
	case 18:
		m_anrSapProvider->SetNoX2(16);
		m_anrSapProvider->SetNoX2(17);
		break;
	case 19:
		m_anrSapProvider->SetNoX2(20);
		m_anrSapProvider->SetNoX2(21);
		break;
	case 20:
		m_anrSapProvider->SetNoX2(19);
		m_anrSapProvider->SetNoX2(21);
		break;
	case 21:
		m_anrSapProvider->SetNoX2(19);
		m_anrSapProvider->SetNoX2(20);
		break;
	default:
		std::cout<<"Cell Id should be between 1 and 21:"<< cellId;
		break;
	}
}
//A.M
void LteEnbRrc::CalAvgRSRPPerCell()
{
	double count = 0;
	double count2 = 0;
	double tmpRSRP = 0;
	uint16_t tmpAvgUEOverTime = 0;
	uint16_t tmpAvgRSRPOverTime = 0;
	    for (std::map<uint16_t, Ptr<UeManager> >::iterator it1 = m_ueMap.begin ();
	         it1 != m_ueMap.end ();
	         ++it1)
	    {

	    		Ptr<UeManager> ueManager = GetUeManager (it1->first);
				std::map<uint16_t, uint8_t> tmp;
				if((ueManager->m_servingRSRP*-1) >= -105) // to avoid too low RSRP (-inf)
				{
					tmpRSRP += (ueManager->m_servingRSRP*-1);
					count++;
				}
	    }
	    if(count != 0)
	    {
	    	m_UeSharePerCell.insert(std::pair<double,double>(Simulator::Now().GetSeconds(), count));
	    	m_AvgRSRP.insert(std::pair<double,double>(Simulator::Now().GetSeconds(),(tmpRSRP/count)));

	    }

	  if(Simulator::Now().GetSeconds() == 30)
	  {
		  double tmp = 0.0;
		  std::cout<<"N = 151, Hys = 5 With MLB and  no cell is down"<<std::endl;
		  std::cout<<m_cellId<<" , "<<"Time"<<" , "<<"Avg RSRP"<<" , "<<" UEPerCell(%)"<<std::endl;
		  for(std::map<double, double>::iterator it = m_AvgRSRP.begin(); it != m_AvgRSRP.end(); ++it)
		  {
			 auto it2 = m_UeSharePerCell.find(it->first);
			     if(it2 != m_UeSharePerCell.end())
			      {
			    	 tmp = it2->second;
			    	 tmpAvgUEOverTime+=tmp;
			    	 count2++;
			      }
			     tmpAvgRSRPOverTime+=it->second;
			  std::cout<<"  ,  "<<it->first<<"  ,  "<<it->second<<"  ,  "<<tmp<<std::endl;
		  }
		 // std::cout<<"  ,  "<<"  ,  "<<(uint16_t)tmpAvgRSRPOverTime/count2<<"  ,  "<<(uint16_t)tmpAvgUEOverTime/count2<<std::endl;

	  }
	  Simulator::Schedule (MilliSeconds(200), &LteEnbRrc::CalAvgRSRPPerCell, this);
}
//A.M
void LteEnbRrc::ActIfPowerDown()
{

	if(m_cphySapProvider->GetReferenceSignalPower() == 0.0)
	{
		m_PowerDown = true;
		//std::cout<<Simulator::Now().GetSeconds()<<": In cell ("<< m_cellId <<") m_PowerDown = "<<m_PowerDown<<std::endl;
		//std::cout<<Simulator::Now().GetSeconds()<<": In cell ("<< m_cellId <<") m_ueMap.empty() = "<<m_ueMap.empty()<<std::endl;
	}
	else
	{
		m_PowerDown = false;
	}


	if(m_PowerDown && !m_ueMap.empty()&&!m_RLFRemoveUeSched)
	{
		std::cout<<Simulator::Now().GetSeconds()<<": In cell ("<< m_cellId <<") ReferenceSignalPower = "<<(double)m_cphySapProvider->GetReferenceSignalPower()<<std::endl;

		m_RLFRemoveUeSched = true;
		std::cout<<"In ActIfPowerDown and the RLFRemoveUe is scheduled for cell "<<m_cellId<<std::endl;
		Simulator::Schedule (MilliSeconds(2000), &LteEnbRrc::RLFRemoveUe, this);


		//std::cout<<Simulator::Now().GetSeconds()<<": In cell ("<< m_cellId <<") Removed all UEs = "<<m_numberUePerEnb<<std::endl;
	}
//	std::cout<<Simulator::Now().GetSeconds()<<": In cell ("<< m_cellId <<") with Num of UEs = "<<m_ueMap.size()<<std::endl;



	if(Simulator::Now().GetSeconds()==4.0|| Simulator::Now().GetSeconds()==5.0||Simulator::Now().GetSeconds()==19.0||Simulator::Now().GetSeconds()==29.0)
	{
		std::cout<<Simulator::Now().GetSeconds()<<": In cell ID "<< m_cellId<< " Num_UE = "<<m_numberUePerEnb << " =="<<(int)m_ueMap.size()<<std::endl;
		for (std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.begin (); it != m_ueMap.end (); ++it)
		{
			std::cout<<"\t "<<it->second->GetImsi();
		}
		std::cout<<std::endl;
	}

	Simulator::Schedule (MilliSeconds(20), &LteEnbRrc::ActIfPowerDown, this);
}
//A.M
void LteEnbRrc::RLFRemoveUe()
{

	std::cout<<"IMSI : \t "<<std::endl;
	for (std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.begin (); it != m_ueMap.end (); ++it)
	{
		m_tmp.push_back(it->first);
		std::cout<<"\t "<<it->second->GetImsi();
	}
	std::cout<<std::endl;

	std::cout<<"RNTI : \t "<<std::endl;
	std::vector<uint16_t>::size_type sz = m_tmp.size();
	for (int i = 0; i< (int) sz; i++)
	{
		std::cout<<"\t "<<m_tmp[i];
		RemoveUe(m_tmp[i]);
	}
	m_tmp.clear();
	std::cout<<std::endl;
	m_numberUePerEnb = 0;
	m_RLFRemoveUeSched = false;
}
//A.M
void
LteEnbRrc::TriggerRLFDetection()
{

	if(m_numberUePerEnb != 0 && (m_cphySapProvider->GetReferenceSignalPower() > 0.0))
	{

	   EpcX2Sap::FailureDetectionTokenParams failParams;
	   failParams.sourceId 		= m_cellId;
	   failParams.forwardingId 	= 9999;
	   failParams.forwarded 	= 0;
	   uint16_t tmp 			= 0;
	   uint16_t tmp2 			= 0;

		 for (int j = 1 ; j <= m_numberOfEnbs; j++)
		 {
			 if(j != m_cellId)
			 {

				 if (m_anrSapProvider != 0)
					{
					  // ensure that proper neighbour relationship exists between source and target cells
					  bool noHo = m_anrSapProvider->GetNoHo (j);
					  bool noX2 = m_anrSapProvider->GetNoX2 (j);
					  NS_LOG_DEBUG (this << " cellId = " << m_cellId
										 << ", targetCellId=" << j
										 << " NRT.NoHo=" << noHo << " NRT.NoX2=" << noX2);

						  std::cout<<"Cell ID = "<<m_cellId<<" has relation to cell ID ="<<j<<", No HO ="<< (uint16_t)noHo<<" and No X2 = "<<(uint16_t)noX2<<std::endl;

					  if (noHo || noX2)
						{
						  	  NS_LOG_LOGIC (this << " handover to cell " << j
											 << " is not allowed by ANR");
						}
					  else
					    {
							 failParams.targetIds.push_back(j);
					    }
					}

			 }
		 }


		 int j = 0;

		 for (; j < (int)((failParams.targetIds.size())-1); j++)
		 {

			 tmp =  failParams.targetIds[j];
			 tmp2 = failParams.targetIds[j+1];

			 failParams.forwardingList.insert(std::pair<uint16_t,uint16_t>(tmp, tmp2));

		 }
		 std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<< m_cellId<<" failParams.targetIds.size()" <<failParams.targetIds.size()<<std::endl;
		 std::cout<<": j = "<< j<<std::endl;
		bool  dontSendRLF = false;
		 if(failParams.targetIds.size() > 1)// to avoid the case of one user 20 -- 20 error
		 {
			 tmp 	= failParams.targetIds[j];
			 tmp2 	= failParams.targetIds[0];
			 failParams.forwardingList.insert(std::pair<uint16_t,uint16_t>(tmp,tmp2));
		 }
		else
		{
		//	  failParams.forwardingList size will be zero and no forwarding will be working
				std::cout<<Simulator::Now().GetSeconds()<<" In Cell "<< m_cellId << "with empty forwardingList"<<std::endl;
			 dontSendRLF = true;
		}


		 if(!dontSendRLF)

		 {
			 ackRecv  tmpMap2;
			 nackRecv  tmpMap3;
			 //initialize first:
			 for (j = 1 ; j <= m_numberOfEnbs; j++)
			{

				/*m_ackForFailureTokenTimeoutMap.insert(std::pair<uint16_t, EventId>(9999,Simulator::Schedule (MilliSeconds (99999999),
														&LteEnbRrc::ACKNotReceivedTimeout,this,9999,9999,
														failParams.forwardingList))) ;

				 m_ackRecvMap.insert(std::pair<uint16_t,uint16_t>(9999,9999));*/

				 tmpMap2.forwardingId 		= 9999;
				 tmpMap2.forwardingForId 	= 9999;
				 tmpMap2.ownerId 			= 9999;
				 m_ackRecvMap.push_back(tmpMap2);

				 tmpMap3.forwardingId 		= 9999;
				 tmpMap3.forwardingForId 	= 9999;
				 tmpMap3.ownerId 			= 9999;
				 m_nackRecvMap.push_back(tmpMap3);

			}
			 AckForFailureTokenTimeout tmpMap;
			 //schedule
			 for (j =0 ; j < (int)failParams.targetIds.size(); j++)
			{
				/* m_ackForFailureTokenTimeoutMap.insert(std::pair<uint16_t, EventId>(std::pair<uint16_t, uint16_t>(failParams.targetIds[j],failParams.sourceId),
														Simulator::Schedule (m_ackForFailureTokenTimeoutDuration,
														&LteEnbRrc::ACKNotReceivedTimeout,
														this,failParams.sourceId,failParams.targetIds[j],failParams.forwardingList))) ;
				 */
				// m_ackForToOwnerRelTimeoutMap.insert(std::pair<uint16_t, uint16_t>(failParams.targetIds[j],failParams.sourceId));



				tmpMap.owner = failParams.sourceId;
				tmpMap.forwardId = failParams.targetIds[j];
				tmpMap.noAck = Simulator::Schedule (m_ackForFailureTokenTimeoutDuration, &LteEnbRrc::ACKNotReceivedTimeout,
																					this,failParams.targetIds[j],failParams.forwardingList,failParams.sourceId);

				m_ackForFailureTokenTimeoutMap.push_back(tmpMap);
			 }

			m_x2SapProvider->SendFailureDetectionToken(failParams);

			Simulator::Schedule (m_RLFDecisionDuration, &LteEnbRrc::RLFTakeDecision, this);

			std::cout<< " In TriggerRLFDetection\t"<<std::endl;
			for(std::map<uint16_t, uint16_t>::iterator it = failParams.forwardingList.begin(); it != failParams.forwardingList.end(); ++it)
			{
				std::cout<< it->first <<"\t"<<it->second<<std::endl;
			}
		}
		 else
		 {
				std::cout<<"\n"<<"**********************************************************************"<<std::endl;
				std::cout<<Simulator::Now().GetSeconds()<<": In TriggerRLFDetection  = "<<m_cellId<<"No Forwarding list"<<std::endl;
				std::cout<<"**********************************************************************"<<"\n"<<std::endl;
		 }
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In TriggerRLFDetection  = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}

}


uint16_t LteEnbRrc::GetCellId()
{
	return m_cellId;
}

void
LteEnbRrc::SetEpcX2SapProvider (EpcX2SapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_x2SapProvider = s;
}

EpcX2SapUser*
LteEnbRrc::GetEpcX2SapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_x2SapUser;
}

void
LteEnbRrc::SetLteEnbCmacSapProvider (LteEnbCmacSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_cmacSapProvider = s;
}

LteEnbCmacSapUser*
LteEnbRrc::GetLteEnbCmacSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_cmacSapUser;
}

void
LteEnbRrc::SetLteHandoverManagementSapProvider (LteHandoverManagementSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_handoverManagementSapProvider = s;
}

LteHandoverManagementSapUser*
LteEnbRrc::GetLteHandoverManagementSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_handoverManagementSapUser;
}

void
LteEnbRrc::SetLteAnrSapProvider (LteAnrSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_anrSapProvider = s;
}

LteAnrSapUser*
LteEnbRrc::GetLteAnrSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_anrSapUser;
}

void
LteEnbRrc::SetLteFfrRrcSapProvider (LteFfrRrcSapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_ffrRrcSapProvider = s;
}

LteFfrRrcSapUser*
LteEnbRrc::GetLteFfrRrcSapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_ffrRrcSapUser;
}

void
LteEnbRrc::SetLteEnbRrcSapUser (LteEnbRrcSapUser * s)
{
  NS_LOG_FUNCTION (this << s);
  m_rrcSapUser = s;
}

LteEnbRrcSapProvider*
LteEnbRrc::GetLteEnbRrcSapProvider ()
{
  NS_LOG_FUNCTION (this);
  return m_rrcSapProvider;
}

void
LteEnbRrc::SetLteMacSapProvider (LteMacSapProvider * s)
{
  NS_LOG_FUNCTION (this);
  m_macSapProvider = s;
}

void
LteEnbRrc::SetS1SapProvider (EpcEnbS1SapProvider * s)
{
  m_s1SapProvider = s;
}


EpcEnbS1SapUser*
LteEnbRrc::GetS1SapUser ()
{
  return m_s1SapUser;
}

void
LteEnbRrc::SetLteEnbCphySapProvider (LteEnbCphySapProvider * s)
{
  NS_LOG_FUNCTION (this << s);
  m_cphySapProvider = s;
}

LteEnbCphySapUser*
LteEnbRrc::GetLteEnbCphySapUser ()
{
  NS_LOG_FUNCTION (this);
  return m_cphySapUser;
}

bool
LteEnbRrc::HasUeManager (uint16_t rnti) const
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  std::map<uint16_t, Ptr<UeManager> >::const_iterator it = m_ueMap.find (rnti);
  return (it != m_ueMap.end ());
}

Ptr<UeManager>
LteEnbRrc::GetUeManager (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  NS_ASSERT (0 != rnti);
  std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.find (rnti);
  NS_ASSERT_MSG (it != m_ueMap.end (), "RNTI " << rnti << " not found in eNB with cellId " << m_cellId);
  return it->second;
}

uint8_t
LteEnbRrc::AddUeMeasReportConfig (LteRrcSap::ReportConfigEutra config)
{
  NS_LOG_FUNCTION (this);

  // SANITY CHECK

  NS_ASSERT_MSG (m_ueMeasConfig.measIdToAddModList.size () == m_ueMeasConfig.reportConfigToAddModList.size (),
                 "Measurement identities and reporting configuration should not have different quantity");

  if (Simulator::Now () != Seconds (0))
    {
      NS_FATAL_ERROR ("AddUeMeasReportConfig may not be called after the simulation has run");
    }

  // INPUT VALIDATION

  switch (config.triggerQuantity)
    {
    case LteRrcSap::ReportConfigEutra::RSRP:
      if ((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5)
          && (config.threshold2.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRP))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRP) does not match with the given threshold2.choice");
        }

      if (((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A1)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A2)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A4)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5))
          && (config.threshold1.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRP))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRP) does not match with the given threshold1.choice");
        }
      break;

    case LteRrcSap::ReportConfigEutra::RSRQ:
      if ((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5)
          && (config.threshold2.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRQ) does not match with the given threshold2.choice");
        }

      if (((config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A1)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A2)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A4)
           || (config.eventId == LteRrcSap::ReportConfigEutra::EVENT_A5))
          && (config.threshold1.choice != LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ))
        {
          NS_FATAL_ERROR ("The given triggerQuantity (RSRQ) does not match with the given threshold1.choice");
        }
      break;

    default:
      NS_FATAL_ERROR ("unsupported triggerQuantity");
      break;
    }

  if (config.purpose != LteRrcSap::ReportConfigEutra::REPORT_STRONGEST_CELLS)
    {
      NS_FATAL_ERROR ("Only REPORT_STRONGEST_CELLS purpose is supported");
    }

  if (config.reportQuantity != LteRrcSap::ReportConfigEutra::BOTH)
    {
      NS_LOG_WARN ("reportQuantity = BOTH will be used instead of the given reportQuantity");
    }

  uint8_t nextId = m_ueMeasConfig.reportConfigToAddModList.size () + 1;

  // create the reporting configuration
  LteRrcSap::ReportConfigToAddMod reportConfig;
  reportConfig.reportConfigId = nextId;
  reportConfig.reportConfigEutra = config;




  // create the measurement identity
  LteRrcSap::MeasIdToAddMod measId;
  measId.measId = nextId;
  measId.measObjectId = 1;
  measId.reportConfigId = nextId;

  // add both to the list of UE measurement configuration
  m_ueMeasConfig.reportConfigToAddModList.push_back (reportConfig);
  m_ueMeasConfig.measIdToAddModList.push_back (measId);


  return nextId;
}

void
LteEnbRrc::ConfigureCell (uint8_t ulBandwidth, uint8_t dlBandwidth,
                          uint16_t ulEarfcn, uint16_t dlEarfcn, uint16_t cellId)
{
  NS_LOG_FUNCTION (this << (uint16_t) ulBandwidth << (uint16_t) dlBandwidth
                        << ulEarfcn << dlEarfcn << cellId);
  NS_ASSERT (!m_configured);
  m_cmacSapProvider->ConfigureMac (ulBandwidth, dlBandwidth);
  m_cphySapProvider->SetBandwidth (ulBandwidth, dlBandwidth);
  m_cphySapProvider->SetEarfcn (ulEarfcn, dlEarfcn);
  m_dlEarfcn = dlEarfcn;
  m_ulEarfcn = ulEarfcn;
  m_dlBandwidth = dlBandwidth;
  m_ulBandwidth = ulBandwidth;
  m_cellId = cellId;
  m_cphySapProvider->SetCellId (cellId);
  m_ffrRrcSapProvider->SetCellId (cellId);
  m_ffrRrcSapProvider->SetBandwidth(ulBandwidth, dlBandwidth);

  /*
   * Initializing the list of UE measurement configuration (m_ueMeasConfig).
   * Only intra-frequency measurements are supported, so only one measurement
   * object is created.
   */

  LteRrcSap::MeasObjectToAddMod measObject;
  measObject.measObjectId = 1;
  measObject.measObjectEutra.carrierFreq = m_dlEarfcn;
  measObject.measObjectEutra.allowedMeasBandwidth = m_dlBandwidth;
  measObject.measObjectEutra.presenceAntennaPort1 = false;
  measObject.measObjectEutra.neighCellConfig = 0;
  measObject.measObjectEutra.offsetFreq = 0;
  measObject.measObjectEutra.haveCellForWhichToReportCGI = false;

  m_ueMeasConfig.measObjectToAddModList.push_back (measObject);
  m_ueMeasConfig.haveQuantityConfig = true;
  m_ueMeasConfig.quantityConfig.filterCoefficientRSRP = m_rsrpFilterCoefficient;
  m_ueMeasConfig.quantityConfig.filterCoefficientRSRQ = m_rsrqFilterCoefficient;
  m_ueMeasConfig.haveMeasGapConfig = false;
  m_ueMeasConfig.haveSmeasure = false;
  m_ueMeasConfig.haveSpeedStatePars = false;

  // Enabling MIB transmission
  LteRrcSap::MasterInformationBlock mib;
  mib.dlBandwidth = m_dlBandwidth;
  m_cphySapProvider->SetMasterInformationBlock (mib);

  // Enabling SIB1 transmission with default values
  m_sib1.cellAccessRelatedInfo.cellIdentity = cellId;
  m_sib1.cellAccessRelatedInfo.csgIndication = false;
  m_sib1.cellAccessRelatedInfo.csgIdentity = 0;
  m_sib1.cellAccessRelatedInfo.plmnIdentityInfo.plmnIdentity = 0; // not used
  m_sib1.cellSelectionInfo.qQualMin = -34; // not used, set as minimum value
  m_sib1.cellSelectionInfo.qRxLevMin = m_qRxLevMin; // set as minimum value
  m_cphySapProvider->SetSystemInformationBlockType1 (m_sib1);

  /*
   * Enabling transmission of other SIB. The first time System Information is
   * transmitted is arbitrarily assumed to be at +0.016s, and then it will be
   * regularly transmitted every 80 ms by default (set the
   * SystemInformationPeriodicity attribute to configure this).
   */
  Simulator::Schedule (MilliSeconds (16), &LteEnbRrc::SendSystemInformation, this);

  m_configured = true;

}


void
LteEnbRrc::SetCellId (uint16_t cellId)
{
  m_cellId = cellId;

  // update SIB1 too
  m_sib1.cellAccessRelatedInfo.cellIdentity = cellId;
  m_cphySapProvider->SetSystemInformationBlockType1 (m_sib1);
}

bool
LteEnbRrc::SendData (Ptr<Packet> packet)
{
  NS_LOG_FUNCTION (this << packet);

  EpsBearerTag tag;
  bool found = packet->RemovePacketTag (tag);
  NS_ASSERT_MSG (found, "no EpsBearerTag found in packet to be sent");
  Ptr<UeManager> ueManager = GetUeManager (tag.GetRnti ());
  ueManager->SendData (tag.GetBid (), packet);

  return true;
}

void
LteEnbRrc::SetForwardUpCallback (Callback <void, Ptr<Packet> > cb)
{
  m_forwardUpCallback = cb;
}

void
LteEnbRrc::ConnectionRequestTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::INITIAL_RANDOM_ACCESS,
                 "ConnectionRequestTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  RemoveUe (rnti);
}

void
LteEnbRrc::ConnectionSetupTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::CONNECTION_SETUP,
                 "ConnectionSetupTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  RemoveUe (rnti);
}

void
LteEnbRrc::ConnectionRejectedTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::CONNECTION_REJECTED,
                 "ConnectionRejectedTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  RemoveUe (rnti);
}

void
LteEnbRrc::HandoverJoiningTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::HANDOVER_JOINING,
                 "HandoverJoiningTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  RemoveUe (rnti);
}

void
LteEnbRrc::HandoverLeavingTimeout (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << rnti);
  NS_ASSERT_MSG (GetUeManager (rnti)->GetState () == UeManager::HANDOVER_LEAVING,
                 "HandoverLeavingTimeout in unexpected state " << ToString (GetUeManager (rnti)->GetState ()));
  RemoveUe (rnti);
}

void
LteEnbRrc::SendHandoverRequest (uint16_t rnti, uint16_t cellId)
{
  NS_LOG_FUNCTION (this << rnti << cellId);
  NS_LOG_LOGIC ("Request to send HANDOVER REQUEST");
  NS_ASSERT (m_configured);

  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->PrepareHandover (cellId);

}

void 
LteEnbRrc::DoCompleteSetupUe (uint16_t rnti, LteEnbRrcSapProvider::CompleteSetupUeParameters params)
{
  NS_LOG_FUNCTION (this << rnti);
  GetUeManager (rnti)->CompleteSetupUe (params);
}

void
LteEnbRrc::DoRecvRrcConnectionRequest (uint16_t rnti, LteRrcSap::RrcConnectionRequest msg)
{
  NS_LOG_FUNCTION (this << rnti);
  //A.M
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		GetUeManager (rnti)->RecvRrcConnectionRequest (msg);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvRrcConnectionRequest from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}

void
LteEnbRrc::DoRecvRrcConnectionSetupCompleted (uint16_t rnti, LteRrcSap::RrcConnectionSetupCompleted msg)
{
  NS_LOG_FUNCTION (this << rnti);
  //A.M
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		GetUeManager (rnti)->RecvRrcConnectionSetupCompleted (msg);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvRrcConnectionSetupCompleted from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}

void
LteEnbRrc::DoRecvRrcConnectionReconfigurationCompleted (uint16_t rnti, LteRrcSap::RrcConnectionReconfigurationCompleted msg)
{
  NS_LOG_FUNCTION (this << rnti);
  //A.M
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		GetUeManager (rnti)->RecvRrcConnectionReconfigurationCompleted (msg);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvRrcConnectionReconfigurationCompleted from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}

void
LteEnbRrc::DoRecvRrcConnectionReestablishmentRequest (uint16_t rnti, LteRrcSap::RrcConnectionReestablishmentRequest msg)
{
  NS_LOG_FUNCTION (this << rnti);
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		GetUeManager (rnti)->RecvRrcConnectionReestablishmentRequest (msg);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvRrcConnectionReestablishmentRequest from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}

void
LteEnbRrc::DoRecvRrcConnectionReestablishmentComplete (uint16_t rnti, LteRrcSap::RrcConnectionReestablishmentComplete msg)
{
  NS_LOG_FUNCTION (this << rnti);
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		GetUeManager (rnti)->RecvRrcConnectionReestablishmentComplete (msg);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvRrcConnectionReestablishmentComplete from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}


void
LteEnbRrc::DoRecvMeasurementReport (uint16_t rnti, LteRrcSap::MeasurementReport msg)
{
  NS_LOG_FUNCTION (this << rnti);
  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		//if(m_cellId == 5)
		//std::cout<<"\n"<<Simulator::Now().GetSeconds()<<": Cell ID= "<<m_cellId<<" with TX power = "<<(uint16_t)m_cphySapProvider->GetReferenceSignalPower()<<std::endl;

		GetUeManager (rnti)->RecvMeasurementReport (msg);

	}
	else
	{
		//std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		//std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvHandoverRequest from UE ID = "<<GetUeManager (rnti)->GetImsi()<<" to cell ID = "<<m_cellId<<" which is DOWN"<<std::endl;
		//std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}

}
/*
void
LteEnbRrc::DoRecvMlbCondition_1 (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  std::cout <<Simulator::Now()<<":CellId ="<<cellId<<"in DoRecvMlbCondition_1"<<std::endl;
 // m_handoverManagementSapProvider->s;

  EpcX2Sap::ResourceStatusRequestParams params;
  params.sourceEnbId = cellId;
  params.enb1MeasurementId = 0;
  params.enb2MeasurementId = 1;
 // params.sourceEnbId = cellId;
 // params.targetCellId = 2;
  m_x2SapProvider->SendResourceStatusRequest(params);

}*/
/*
void
LteEnbRrc::DoCheckMlbCondition_1 (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);
  std::cout <<Simulator::Now()<<":CellId ="<<cellId<<"in DoCheckMlbCondition_1"<<std::endl;

  //m_rrcSapUser->SendMlbCondition_1(cellId);

}
*/
void
LteEnbRrc::DoDataRadioBearerSetupRequest (EpcEnbS1SapUser::DataRadioBearerSetupRequestParameters request)
{
  Ptr<UeManager> ueManager = GetUeManager (request.rnti);
  ueManager->SetupDataRadioBearer (request.bearer, request.bearerId, request.gtpTeid, request.transportLayerAddress);
}

void
LteEnbRrc::DoPathSwitchRequestAcknowledge (EpcEnbS1SapUser::PathSwitchRequestAcknowledgeParameters params)
{
  Ptr<UeManager> ueManager = GetUeManager (params.rnti);
  ueManager->SendUeContextRelease ();
}

void
LteEnbRrc::DoRecvHandoverRequest (EpcX2SapUser::HandoverRequestParams req)
{

  NS_LOG_FUNCTION (this);

  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{

	  NS_LOG_LOGIC ("Recv X2 message: HANDOVER REQUEST");

	  NS_LOG_LOGIC ("oldEnbUeX2apId = " << req.oldEnbUeX2apId);
	  NS_LOG_LOGIC ("sourceCellId = " << req.sourceCellId);
	  NS_LOG_LOGIC ("targetCellId = " << req.targetCellId);
	  NS_LOG_LOGIC ("mmeUeS1apId = " << req.mmeUeS1apId);

	  NS_ASSERT (req.targetCellId == m_cellId);

	  if (m_admitHandoverRequest == false)
		{
		  NS_LOG_INFO ("rejecting handover request from cellId " << req.sourceCellId);
		  EpcX2Sap::HandoverPreparationFailureParams res;
		  res.oldEnbUeX2apId =  req.oldEnbUeX2apId;
		  res.sourceCellId = req.sourceCellId;
		  res.targetCellId = req.targetCellId;
		  res.cause = 0;
		  res.criticalityDiagnostics = 0;
		  m_x2SapProvider->SendHandoverPreparationFailure (res);
		  return;
		}

	  uint16_t rnti = AddUe (UeManager::HANDOVER_JOINING);
	  LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue anrcrv = m_cmacSapProvider->AllocateNcRaPreamble (rnti);
	  if (anrcrv.valid == false)
		{
		  std::cout<<Simulator::Now().GetSeconds()<<": In Cell ID "<<m_cellId<< ": UE ID = "<< rnti <<" Not allowed to handover"<<std::endl;
		  NS_LOG_INFO (this << " failed to allocate a preamble for non-contention based RA => cannot accept HO");
		  RemoveUe (rnti);
		  NS_FATAL_ERROR ("should trigger HO Preparation Failure, but it is not implemented");
		  return;
		}

	  Ptr<UeManager> ueManager = GetUeManager (rnti);
	  ueManager->SetSource (req.sourceCellId, req.oldEnbUeX2apId);
	  ueManager->SetImsi (req.mmeUeS1apId);

	  EpcX2SapProvider::HandoverRequestAckParams ackParams;
	  ackParams.oldEnbUeX2apId = req.oldEnbUeX2apId;
	  ackParams.newEnbUeX2apId = rnti;
	  ackParams.sourceCellId = req.sourceCellId;
	  ackParams.targetCellId = req.targetCellId;

	  for (std::vector <EpcX2Sap::ErabToBeSetupItem>::iterator it = req.bearers.begin ();
		   it != req.bearers.end ();
		   ++it)
		{
		  ueManager->SetupDataRadioBearer (it->erabLevelQosParameters, it->erabId, it->gtpTeid, it->transportLayerAddress);
		  EpcX2Sap::ErabAdmittedItem i;
		  i.erabId = it->erabId;
		  ackParams.admittedBearers.push_back (i);
		}

	  LteRrcSap::RrcConnectionReconfiguration handoverCommand = ueManager->GetRrcConnectionReconfigurationForHandover ();
	  handoverCommand.haveMobilityControlInfo = true;
	  handoverCommand.mobilityControlInfo.targetPhysCellId = m_cellId;
	  handoverCommand.mobilityControlInfo.haveCarrierFreq = true;
	  handoverCommand.mobilityControlInfo.carrierFreq.dlCarrierFreq = m_dlEarfcn;
	  handoverCommand.mobilityControlInfo.carrierFreq.ulCarrierFreq = m_ulEarfcn;
	  handoverCommand.mobilityControlInfo.haveCarrierBandwidth = true;
	  handoverCommand.mobilityControlInfo.carrierBandwidth.dlBandwidth = m_dlBandwidth;
	  handoverCommand.mobilityControlInfo.carrierBandwidth.ulBandwidth = m_ulBandwidth;
	  handoverCommand.mobilityControlInfo.newUeIdentity = rnti;
	  handoverCommand.mobilityControlInfo.haveRachConfigDedicated = true;
	  handoverCommand.mobilityControlInfo.rachConfigDedicated.raPreambleIndex = anrcrv.raPreambleId;
	  handoverCommand.mobilityControlInfo.rachConfigDedicated.raPrachMaskIndex = anrcrv.raPrachMaskIndex;

	  LteEnbCmacSapProvider::RachConfig rc = m_cmacSapProvider->GetRachConfig ();
	  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
	  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
	  handoverCommand.mobilityControlInfo.radioResourceConfigCommon.rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;

	  Ptr<Packet> encodedHandoverCommand = m_rrcSapUser->EncodeHandoverCommand (handoverCommand);

	  ackParams.rrcContext = encodedHandoverCommand;

	  NS_LOG_LOGIC ("Send X2 message: HANDOVER REQUEST ACK");

	  NS_LOG_LOGIC ("oldEnbUeX2apId = " << ackParams.oldEnbUeX2apId);
	  NS_LOG_LOGIC ("newEnbUeX2apId = " << ackParams.newEnbUeX2apId);
	  NS_LOG_LOGIC ("sourceCellId = " << ackParams.sourceCellId);
	  NS_LOG_LOGIC ("targetCellId = " << ackParams.targetCellId);

	  m_x2SapProvider->SendHandoverRequestAck (ackParams);
	}
	else
	{
		std::cout<<"\n"<<"**********************************************************************"<<std::endl;
		std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvHandoverRequest from cell ID = "<< req.sourceCellId<<" to cell ID = "<< m_cellId<<" which is DOWN"<<std::endl;
		std::cout<<"**********************************************************************"<<"\n"<<std::endl;
	}
}

void
LteEnbRrc::DoRecvHandoverRequestAck (EpcX2SapUser::HandoverRequestAckParams params)
{
  NS_LOG_FUNCTION (this);
  //A.M , I added the if  here
if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
{
  NS_LOG_LOGIC ("Recv X2 message: HANDOVER REQUEST ACK");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);

  uint16_t rnti = params.oldEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvHandoverRequestAck (params);
}
else
{
	std::cout<<"\n"<<"**********************************************************************"<<std::endl;
	std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvHandoverRequest from cell ID = "<< params.sourceCellId<<" to cell ID = "<< m_cellId<<" which is DOWN"<<std::endl;
	std::cout<<"**********************************************************************"<<"\n"<<std::endl;
}
}

void
LteEnbRrc::DoRecvHandoverPreparationFailure (EpcX2SapUser::HandoverPreparationFailureParams params)
{
  NS_LOG_FUNCTION (this);
  //A.M , I added the if  here
if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
{
  NS_LOG_LOGIC ("Recv X2 message: HANDOVER PREPARATION FAILURE");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
  NS_LOG_LOGIC ("cause = " << params.cause);
  NS_LOG_LOGIC ("criticalityDiagnostics = " << params.criticalityDiagnostics);

  uint16_t rnti = params.oldEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvHandoverPreparationFailure (params.targetCellId);
}
else
{
	std::cout<<"\n"<<"**********************************************************************"<<std::endl;
	std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvHandoverRequest from cell ID = "<< params.sourceCellId<<" to cell ID = "<< m_cellId<<" which is DOWN"<<std::endl;
	std::cout<<"**********************************************************************"<<"\n"<<std::endl;
}
}

void
LteEnbRrc::DoRecvSnStatusTransfer (EpcX2SapUser::SnStatusTransferParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: SN STATUS TRANSFER");

  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);
  NS_LOG_LOGIC ("erabsSubjectToStatusTransferList size = " << params.erabsSubjectToStatusTransferList.size ());

  uint16_t rnti = params.newEnbUeX2apId;
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->RecvSnStatusTransfer (params);
	}
}

void
LteEnbRrc::DoRecvUeContextRelease (EpcX2SapUser::UeContextReleaseParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
	  NS_LOG_FUNCTION (this);

	  NS_LOG_LOGIC ("Recv X2 message: UE CONTEXT RELEASE");

	  NS_LOG_LOGIC ("oldEnbUeX2apId = " << params.oldEnbUeX2apId);
	  NS_LOG_LOGIC ("newEnbUeX2apId = " << params.newEnbUeX2apId);

	  uint16_t rnti = params.oldEnbUeX2apId;
	  GetUeManager (rnti)->RecvUeContextRelease (params);
	  RemoveUe (rnti);
	}
  // A.M
  m_numberUePerEnb--;
}

void
LteEnbRrc::DoRecvLoadInformation (EpcX2SapUser::LoadInformationParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv X2 message: LOAD INFORMATION");

  NS_LOG_LOGIC ("Number of cellInformationItems = " << params.cellInformationList.size ());

  m_ffrRrcSapProvider->RecvLoadInformation(params);
	}
}

//A.M
void LteEnbRrc::RLFTakeDecision()
{
	std::cout<<Simulator::Now().GetSeconds()<<": In cell Id ("<< m_cellId <<") RLFTakeDecision has to be made"<<std::endl;
	std::cout<<Simulator::Now().GetMilliSeconds()<<": List of alive or unresponsive cells from the point of view of cell id ="<<m_cellId<<std::endl;
	if(m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		for(std::map<uint16_t,uint16_t>::iterator it = m_aliveMap.begin(); it != m_aliveMap.end(); ++it)
		{
				std::cout<<it->first <<" with unresponsive reports  = "<<it->second<<std::endl;

				if(it->second > 1)
				{
					if(m_numberUePerEnb != 0)
					{
						if((m_dlBandwidth/m_numberUePerEnb) >= 4) // under-loaded has 75% free
						{

							for (std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.begin (); it != m_ueMap.end (); ++it)
							{

								Ptr<UeManager> ueManager = GetUeManager (it->first);
								ueManager->SetHysteresisValue(m_KeepHysteresis);
								ueManager->ScheduleRrcConnectionReconfiguration();


							}
							//increase Tx power to 46
							m_cphySapProvider->SetTxPower(45.0); // increase power two steps
							std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<<m_cellId<< "Increases Tx power tp 43.0 dBm"<<std::endl;



						}
						else if((m_dlBandwidth/m_numberUePerEnb) >= 2) // moderate loaded has 50 % free
						{
							for (std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.begin (); it != m_ueMap.end (); ++it)
							{

								Ptr<UeManager> ueManager = GetUeManager (it->first);
								ueManager->SetHysteresisValue(m_KeepHysteresis/2);
								ueManager->ScheduleRrcConnectionReconfiguration();


							}
							//increase Tx power to 46
							m_cphySapProvider->SetTxPower(44.0); // increase power one steps
							std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<<m_cellId<< "Increases Tx power tp 43.0 dBm"<<std::endl;


						}
						else // over-loaded less than 50 %
						{
							for (std::map<uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.begin (); it != m_ueMap.end (); ++it)
							{

								Ptr<UeManager> ueManager = GetUeManager (it->first);
								//ueManager->SetHysteresisValue(m_myHysteresis);
								ueManager->SetHysteresisValue(rand()% 6);

								ueManager->ScheduleRrcConnectionReconfiguration();
								//TriggerMlbCondition_1();

							}
						}

					}
				}
		}
	}
}

//A.M
void LteEnbRrc::ACKNotReceivedTimeout(uint16_t forwardingId, std::map<uint16_t,uint16_t> forwardingList, uint16_t owner)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
	std::cout<<Simulator::Now().GetMilliSeconds()<<" : Cell Id = "<<owner<<" In ACKNotReceivedTimeout , no ACK from path "<<forwardingId<<std::endl;
	uint16_t tmp1 = 0;
	//uint16_t tmp2 = 0;
/*
	for(std::map<uint16_t,uint16_t>::iterator it= forwardingList.begin(); it != forwardingList.end(); ++it)
	{
		if(it->first == targId) // to get which user the targId cell was going to forward the Token for (it->second).
		{
			for(std::map<uint16_t,uint16_t>::iterator it2= m_ackRecvMap.begin(); it2 != m_ackRecvMap.end(); ++it2)
			{
				if(it2->second == targId) // it means that targId is alive and its corresponding forward cell is unresponsive
				{
					for(std::map<uint16_t,uint16_t>::iterator it3= m_aliveMap.begin(); it3 != m_aliveMap.end(); ++it3)
					{
						if(it3->first == it->second)
						{
							tmp1 = it3->second;
							tmp1++;
							m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1++));
						}
					}
				}
				else //it means that targId is unresponsive also and its corresponding forward cell is unresponsive
				{

					for(std::map<uint16_t,uint16_t>::iterator it3= m_aliveMap.begin(); it3 != m_aliveMap.end(); ++it3)
					{
						//this wil garantee for us to make both as unresponsive if both are down
						if(it3->first == it->second)
						{
							tmp1 = it3->second;
							tmp1++;
							m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
						}
						else if (it3->first == targId)
						{
							tmp2 = it3->second;
							tmp2++;
							m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
						}

					}
				}
			}
		}
	}
*/
 /*   auto it = forwardingList.find(targId); // to get it->second
    if(it != forwardingList.end())
    {

		for(std::map<uint16_t,uint16_t>::iterator it2= m_ackRecvMap.begin(); it2 != m_ackRecvMap.end(); ++it2)
		{
			if(it2->second == targId) // it means that targId is alive because it sends ACK to owner when it received forwarded Token and the cell to whom it forwarded the Token is unresponsive
			{
			    auto it3 = m_aliveMap.find(it->second);
			    if(it3 != m_aliveMap.end())
			    {
			    	std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it3->first<<std::endl;
			    	//found
					tmp1 = it3->second;
					std::cout<<" old value = "<<tmp1<<std::endl;
					tmp1++;
			    	//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
					m_aliveMap.at(it3->first) = tmp1;

					std::cout<<" New value = "<<m_aliveMap.find(it3->first)->second<<std::endl;
			    }
			}
			else //it means that targId is unresponsive also and its corresponding forward cell is unresponsive
			{
				//check if NACK received from it->second

				for(std::map<uint16_t,uint16_t>::iterator it5= m_nackRecvMap.begin(); it5 != m_ackRecvMap.end(); ++it5)
				{
					if(it5->second == it->second) // it means that srcId is alive because it sends NACK to owner when it didn't received forwarded Token from targId
					{
						auto it6 = m_aliveMap.find(targId);
							if (it6 != m_aliveMap.end())
							{
								tmp2 = it6->second;
								std::cout<<" old value = "<<tmp2<<std::endl;
								tmp2++;
							//	m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
								m_aliveMap.at(it6->first) = tmp2;
								std::cout<<" New value = "<<m_aliveMap.find(it6->first)->second<<std::endl;
							}

							else
							{
								auto it3 = m_aliveMap.find(it->second);
								std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it3->first<<std::endl;
								//this will guarantee for us to make both as unresponsive if both are down
								if(it3 != m_aliveMap.end())
								{
									tmp1 = it3->second;
									std::cout<<" old value = "<<tmp1<<std::endl;
									tmp1++;
									//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
									m_aliveMap.at(it3->first) = tmp1;
									std::cout<<" New value = "<<m_aliveMap.find(it3->first)->second<<std::endl;
								}
								auto it4 = m_aliveMap.find(targId);
								if (it4 != m_aliveMap.end())
								{
									tmp2 = it4->second;
									std::cout<<" old value = "<<tmp2<<std::endl;
									tmp2++;
								//	m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
									m_aliveMap.at(it4->first) = tmp2;
									std::cout<<" New value = "<<m_aliveMap.find(it4->first)->second<<std::endl;
								}
							}
					}
				}
			}
		}
    }
*/
    bool ForwardingIdUnresponsive 		= true;
    bool ForwardingForIdUnresponsive 	= true;

    uint16_t forwardingForId = 0;

    auto it1 = forwardingList.find(forwardingId); // Find the forwardingForId from the forwardingId (it->second)

if(it1 != forwardingList.end())
  {
    	forwardingForId = it1->second;


	std::vector<ackRecv>::size_type sz = m_ackRecvMap.size();
	for(int j = 0; j < (int)sz; j++)
	{
		if(m_ackRecvMap[j].forwardingForId == forwardingId && m_ackRecvMap[j].ownerId == owner)
		{
			ForwardingIdUnresponsive = false;
		}
		if(m_ackRecvMap[j].forwardingForId == forwardingForId && m_ackRecvMap[j].ownerId == owner)
		{
			ForwardingForIdUnresponsive = false;
		}
	}

	std::vector<ackRecv>::size_type sz2 = m_nackRecvMap.size();
	for(int j = 0; j < (int)sz2; j++)
	{
		if(m_nackRecvMap[j].forwardingForId == forwardingId && m_nackRecvMap[j].ownerId == owner)
		{
			ForwardingIdUnresponsive = false;
		}
		if(m_nackRecvMap[j].forwardingForId ==  forwardingForId && m_nackRecvMap[j].ownerId == owner)
		{
			ForwardingForIdUnresponsive = false;
		}
	}

	if(ForwardingIdUnresponsive)
	{
		auto it2 = m_aliveMap.find(forwardingId);
		std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for forwardingId cell  "<<forwardingId<<std::endl;
		//this will guarantee for us to make both as unresponsive if both are down
		if(it2 != m_aliveMap.end())
		{
			tmp1 = it2->second;
			std::cout<<" old value = "<<tmp1<<std::endl;
			tmp1++;
			//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
			m_aliveMap.at(it2->first) = tmp1;
			std::cout<<" New value = "<<m_aliveMap.find(it2->first)->second<<std::endl;
		}
	}
	if(ForwardingForIdUnresponsive)
	{
		auto it2 = m_aliveMap.find(forwardingForId);
		std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for  forwardingForId cell  "<<forwardingForId<<std::endl;
		//this will guarantee for us to make both as unresponsive if both are down
		if(it2 != m_aliveMap.end())
		{
			tmp1 = it2->second;
			std::cout<<" old value = "<<tmp1<<std::endl;
			tmp1++;
			//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
			m_aliveMap.at(it2->first) = tmp1;
			std::cout<<" New value = "<<m_aliveMap.find(it2->first)->second<<std::endl;
		}
	}
  }// end of if (it1 != forwardingList.end())
else
{
	// forwarding list is empty
	std::cout <<"Cell Id "<<m_cellId<<" in ACKNotReceivedTimeout with forwarding list is empty"<<std::endl;
}

/*
    bool marked = false;
    bool marked2 = false;
    //uint16_t forwardingForId = 0;
    auto it1 = forwardingList.find(forwardingId); // Find the forwardingForId from the forwardingId (it->second)
    if(it1 != forwardingList.end())
    {
    	forwardingForId = it1->second;

    	std::vector<nackRecv>::size_type sz = m_nackRecvMap.size();
    	for(int j = 0; j < (int)sz; j++)
    	{
    		if(m_nackRecvMap[j].forwardingForId == forwardingForId && m_nackRecvMap[j].ownerId == owner)
    		{
    			std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is already updated for forwardingId cell "<<forwardingId<<std::endl;
    			std::cout<<"because the forwardingForId cell "<<forwardingForId<<" already send NACK "<<std::endl;
    			m_nackRecvMap[j].forwardingId 		= 9999;
    			m_nackRecvMap[j].forwardingForId 	= 9999;
    			m_nackRecvMap[j].ownerId 			= 9999;
				marked = true;
				marked2 = true;
				break;
    		}
    	}

		if(!marked)
		{

	    	std::vector<ackRecv>::size_type sz = m_ackRecvMap.size();
	    	for(int j = 0; j < (int)sz; j++)
	    	{
	    		if((m_ackRecvMap[j].forwardingForId == forwardingId || m_nackRecvMap[j].forwardingForId == forwardingId )&& m_ackRecvMap[j].ownerId == owner)
	    		{

					//update it1->second
					auto it4 = m_aliveMap.find(forwardingForId);
					std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for forwardingForId cell C "<<forwardingForId<<std::endl;
	    			std::cout<<"because the forwardingId cell "<<forwardingId<<" already send ACK back "<<std::endl;
					if (it4 != m_aliveMap.end())
					{
						tmp2 = it4->second;
						std::cout<<" old value = "<<tmp2<<std::endl;
						tmp2++;
					//	m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
						m_aliveMap.at(it4->first) = tmp2;
						std::cout<<" New value = "<<m_aliveMap.find(it4->first)->second<<std::endl;
					}
	    			m_ackRecvMap[j].forwardingId 		= 9999;
	    			m_ackRecvMap[j].forwardingForId 	= 9999;
	    			m_ackRecvMap[j].ownerId 			= 9999;
					marked2 = true;
					break;



	    		}
	    	}
		}


		if(!marked2)
		{
			//this will guarantee for us to make both as unresponsive if both are down
			//update both targId and it1->second
			auto it5 = m_aliveMap.find(forwardingForId);
			std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for BOTH forwardingForId cell  "<<forwardingForId<<std::endl;
			//this will guarantee for us to make both as unresponsive if both are down
			if(it5 != m_aliveMap.end())
			{
				tmp1 = it5->second;
				std::cout<<" old value = "<<tmp1<<std::endl;
				tmp1++;
				//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
				m_aliveMap.at(it5->first) = tmp1;
				std::cout<<" New value = "<<m_aliveMap.find(it5->first)->second<<std::endl;
			}
			auto it6 = m_aliveMap.find(forwardingId);
			std::cout<<" and for   for forwardingId cell "<<forwardingId<<std::endl;
			if (it6 != m_aliveMap.end())
			{
				tmp2 = it6->second;
				std::cout<<" old value = "<<tmp2<<std::endl;
				tmp2++;
			//	m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
				m_aliveMap.at(it6->first) = tmp2;
				std::cout<<" New value = "<<m_aliveMap.find(it6->first)->second<<std::endl;
			}
		}
    }
*/



	/*bool markedB = false;
	bool markedC = false;
	    // to get to whom targId should be forwarding the Token (it1->second)
		for(std::map<uint16_t,uint16_t>::iterator it1= forwardingList.begin(); it1 != forwardingList.end(); ++it1)
		{
			if(it1->first == targId)
			{
				for(std::map<uint16_t,uint16_t>::iterator it2= m_nackRecvMap.begin(); it2 != m_ackRecvMap.end(); ++it2)
				{
					if(it2->second == it1->second) // it means that it1->second is alive because it sends NACK to owner srcId when it didn't received forwarded Token from targId
					{
						std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it2->first<<std::endl;
						//update  m_aliveMap for targId
						markedB = true;
					}
				}
				if(!markedB)
				{
					for(std::map<uint16_t,uint16_t>::iterator it3= m_ackRecvMap.begin(); it3 != m_ackRecvMap.end(); ++it3)
					{
						if(it3->second == targId) // it means that targId is alive because it sends ACK to owner when it received forwarded Token from someone else and the cell to whom it forwarded the Token is unresponsive
						{
							std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it3->first<<std::endl;
							//update it1->second
							marked2 = true;



						}

					}
				}
				if(!marked2)
				{

					//this will guarantee for us to make both as unresponsive if both are down
					//update both targId and it1->second
					for(std::map<uint16_t,uint16_t>::iterator it5 = m_aliveMap.begin(); it5 != m_aliveMap.end(); ++it5)
					{
						if(it5->second == it1->second)
						{

							std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it5->first<<std::endl;

							tmp1 = it5->second;
							std::cout<<" old value = "<<tmp1<<std::endl;
							tmp1++;
							//m_aliveMap.insert(std::pair<uint16_t,uint16_t>(it3->first, tmp1));
							m_aliveMap.at(it5->first) = tmp1;
							std::cout<<" New value = "<<m_aliveMap.find(it5->first)->second<<std::endl;
							break;
						}
					}
					for(std::map<uint16_t,uint16_t>::iterator it6 = m_aliveMap.begin(); it6 != m_aliveMap.end(); ++it6)
					{
						if(it6->second == targId)
						{
							std::cout<<Simulator::Now().GetMilliSeconds()<<": In cell ID "<<m_cellId<<" the m_aliveMap is updated for cell "<<it6->first<<std::endl;
							tmp2 = it6->second;
							std::cout<<" old value = "<<tmp2<<std::endl;
							tmp2++;
						//	m_aliveMap.insert(std::pair<uint16_t,uint16_t>(targId, tmp2));
							m_aliveMap.at(it6->first) = tmp2;
							std::cout<<" New value = "<<m_aliveMap.find(it6->first)->second<<std::endl;
						}
					}
				}
			}
		}//end of forwardingList loop

*/
	}//end of power>0
}
//
//A.M
void LteEnbRrc::NoForwardTokenReceivedTimeout(uint16_t forwardingId, uint16_t forwardingForId, uint16_t owner)
{
	std::cout<<Simulator::Now().GetMilliSeconds()<<" : Cell Id = "<<forwardingForId<<" In NoForwardTokenReceivedTimeout from "<<forwardingId<<" send NACK to Token owner "<<owner<<std::endl;



	  EpcX2Sap::TokenAckNACKParams paramsAckNack;
	  paramsAckNack.sourceId 		= m_cellId;
	  paramsAckNack.forwardingId 	= forwardingId;
	  paramsAckNack.tokenOwnerId 	= owner;
	 // paramsAckNack.singleNode		= 0;

	  if(!m_anrSapProvider->GetNoX2(forwardingId))
	  {
		  paramsAckNack.noX2ToForward	= 0; // Here you have to check the availability of X2 interface before you send ACK/NACK
	  }
	  else
	  {
		  paramsAckNack.noX2ToForward	= 1; // Here you have to check the availability of X2 interface before you send ACK/NACK
		  std::cout<<Simulator::Now().GetMilliSeconds()<<" :Cell Id = "<<forwardingForId<<" has no X2 with"<<forwardingId<<std::endl;
	  }
	  paramsAckNack.successStatus 	= 0; // true =1 as I am alive and the forwarding as well , so I am sending ACK
	  	  	  	  	  	  	  	  	  	 //false = 0, I am alive but NACK back to token owner,
	  m_x2SapProvider->SendTokenAckNACK(paramsAckNack);
}


void LteEnbRrc::DoRecvFailureDetectionToken (EpcX2SapUser::FailureDetectionTokenParams params)
{
	//A.M , I added the if  here 7_6_2018
	if(m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{




		   NS_ASSERT (m_configured);


		   NS_LOG_FUNCTION (this);
		   NS_LOG_LOGIC ("Recv X2 message: Failure Detection Token");
		   NS_LOG_LOGIC (" From SourceCellId 		= " << params.sourceId);

		   if(params.forwarded == 0 && !params.forwardingList.empty()) // it is not forwarded message,0 = not forwarded so forward it for next destination
		   {
			 //  std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<<m_cellId<<" : In DoRecvFailureDetectionToken from = "<<params.sourceId<<std::endl;
			  // std::cout<<"With forwarding List = "<<std::endl;
			  // for(std::map<uint16_t, uint16_t>::iterator it= params.forwardingList.begin(); it != params.forwardingList.end(); ++it)
			 //  {
			//	   std::cout<<it->first<<"\t"<<it->second<<std::endl;
			  // }



	    			  //check forwarding list for which user should forward its Token to me and check if there is X2 interface with it
	    			  NoForwardFailureTokenTimeout tmpMap;

	   			   for(std::map<uint16_t, uint16_t>::iterator it= params.forwardingList.begin(); it != params.forwardingList.end(); ++it)
	   			   {
	   				   if(it->second == m_cellId)
	   				   {

	   					/*m_NoForwardFailureTokenTimeoutMap.insert(std::pair<uint16_t,EventId>
	   							(params.sourceId, Simulator::Schedule (m_NoForwardFailureTokenTimeoutDuration,
 	 	 	 	 	 	 	 	&LteEnbRrc::NoForwardTokenReceivedTimeout,
 	 	 	 	 	 	 	 	this,it->first,it->second,params.sourceId)));*/
	   					   tmpMap.forwardId 	= it->first;
	   					   tmpMap.forwaredForId = it->second;
	   					   tmpMap.owner 		= params.sourceId;
	   					   tmpMap.noForward 	= Simulator::Schedule (m_NoForwardFailureTokenTimeoutDuration,
	 	 	 	 	 	 	 	 									&LteEnbRrc::NoForwardTokenReceivedTimeout,
																	this,it->first,it->second,params.sourceId);
	   					m_NoForwardFailureTokenTimeoutMap.push_back(tmpMap);

	   				   }
	   			   }
		    		  /*********************************************************************
		    		   *   prepare to Forward the same message to next destination         *
		    		   *********************************************************************/
				   	   	  params.forwardingId 	= m_cellId;
				   	   	  params.forwarded 		= 1;
		    			  m_x2SapProvider->SendFailureDetectionToken(params);

		   }
		  /* else if(params.forwardingList.empty())
		   {

   			   for(std::map<uint16_t, uint16_t>::iterator it= params.forwardingList.begin(); it != params.forwardingList.end(); ++it)
   			   {
   				   if(it->second == m_cellId )
   				   {

   					   std::vector<NoForwardFailureTokenTimeout>::size_type sz = m_NoForwardFailureTokenTimeoutMap.size();
   					   for(int j =0 ; j < (int)sz; j++)
   					   {
   						   if(m_NoForwardFailureTokenTimeoutMap[j].forwardId == it->first && m_NoForwardFailureTokenTimeoutMap[j].owner == params.sourceId
   								   && m_NoForwardFailureTokenTimeoutMap[j].forwaredForId == m_cellId)
   						   {
   							   m_NoForwardFailureTokenTimeoutMap[j].noForward.Cancel();
   							m_NoForwardFailureTokenTimeoutMap[j].forwardId 		= 9999;
   							m_NoForwardFailureTokenTimeoutMap[j].owner 			= 9999;
   							m_NoForwardFailureTokenTimeoutMap[j].forwaredForId 	= 9999;

   						   }
   					   }
   				   }
   			   }
			  std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<<m_cellId<<" : In DoRecv forwarded FailureDetectionToken from = "<<params.forwardingId<<std::endl;

			  EpcX2Sap::TokenAckNACKParams paramsAckNack;
			  paramsAckNack.sourceId 		= m_cellId;
			  paramsAckNack.forwardingId 	= m_cellId; // forwardingId is me as I am single Node and I don't have a forwarding List
			  paramsAckNack.tokenOwnerId 	= params.sourceId;
			  paramsAckNack.noX2ToForward	= 0; // Here you have to check the availability of X2 interface before you send ACK/NACK
			  paramsAckNack.successStatus 	= 1; // true as I am alive , so I am sending ACK back to token owner
			 // paramsAckNack.singleNode		= 1; // Single node
 			  m_x2SapProvider->SendTokenAckNACK(paramsAckNack);

		   }*/
		   else // it is forwarded message
		   {
   			   for(std::map<uint16_t, uint16_t>::iterator it= params.forwardingList.begin(); it != params.forwardingList.end(); ++it)
   			   {
   				   if(it->second == m_cellId )
   				   {
   		   			   /*for(std::map<uint16_t, EventId>::iterator it2= m_NoForwardFailureTokenTimeoutMap.begin(); it2 != m_NoForwardFailureTokenTimeoutMap.end(); ++it2)
   		   			   {
   		   				   if(it2->first == params.sourceId)this is worng if // this to be sure that I am canceling the right Token timer for the right owner
   		   				   {
   		   					   it2->second.Cancel();
   		   				   }
   		   			   }*/
   					   std::vector<NoForwardFailureTokenTimeout>::size_type sz = m_NoForwardFailureTokenTimeoutMap.size();
   					   for(int j =0 ; j < (int)sz; j++)
   					   {
   						   if(m_NoForwardFailureTokenTimeoutMap[j].forwardId == it->first && m_NoForwardFailureTokenTimeoutMap[j].owner == params.sourceId
   								   && m_NoForwardFailureTokenTimeoutMap[j].forwaredForId == m_cellId)
   						   {
   							   m_NoForwardFailureTokenTimeoutMap[j].noForward.Cancel();
   							m_NoForwardFailureTokenTimeoutMap[j].forwardId 		= 9999;
   							m_NoForwardFailureTokenTimeoutMap[j].owner 			= 9999;
   							m_NoForwardFailureTokenTimeoutMap[j].forwaredForId 	= 9999;

   						   }
   					   }
   				   }
   			   }
			  std::cout<<Simulator::Now().GetSeconds()<<": Cell Id = "<<m_cellId<<" : In DoRecv forwarded FailureDetectionToken from = "<<params.forwardingId<<std::endl;

			  EpcX2Sap::TokenAckNACKParams paramsAckNack;
			  paramsAckNack.sourceId 		= m_cellId;
			  paramsAckNack.forwardingId 	= params.forwardingId;
			  paramsAckNack.tokenOwnerId 	= params.sourceId;
			  paramsAckNack.noX2ToForward	= 0; // Here you have to check the availability of X2 interface before you send ACK/NACK
			  paramsAckNack.successStatus 	= 1; // true as I am alive , so I am sending ACK back to token owner
			  //paramsAckNack.singleNode		= 0;
 			  m_x2SapProvider->SendTokenAckNACK(paramsAckNack);
		   }
	}//end of if txPower
	else
	{
		if(params.forwarded == 0)
		{
			std::cout<<"\n"<<"**********************************************************************"<<std::endl;
			std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvFailureDetectionToken cell ID ="<<m_cellId<<" is DOWN"<<std::endl;
			std::cout<<"**********************************************************************"<<"\n"<<std::endl;
		}
		else
		{
			std::cout<<"\n"<<"**********************************************************************"<<std::endl;
			std::cout<<Simulator::Now().GetSeconds()<<": In DoRecvForwardedFailureDetectionToken from "<<params.sourceId<<" to cell ID ="<<m_cellId<<" which is DOWN and will not send ACK back"<<std::endl;
			std::cout<<"**********************************************************************"<<"\n"<<std::endl;
		}

	}
}
void LteEnbRrc::DoRecvTokenAckNACK	(EpcX2SapUser::TokenAckNACKParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		std::cout<<Simulator::Now().GetSeconds()<<": Cell Id ="<<m_cellId<<" In  DoRecvTokenAckNACK from"<<params.sourceId<<std::endl;

		// Mark the source as alive
		if(params.successStatus == 1) // then ACK
		{
			/*for (std::map<uint16_t, EventId>::iterator it =m_ackForFailureTokenTimeoutMap.begin() ; it != m_ackForFailureTokenTimeoutMap.end(); ++it)
			{
				if(params.forwardingId == it->first)
				it->second.Cancel();
			}*/
			std::vector<AckForFailureTokenTimeout>::size_type sz = m_ackForFailureTokenTimeoutMap.size();
			for(int j = 0; j < (int)sz ; j++)
			{
				if(m_ackForFailureTokenTimeoutMap[j].forwardId == params.forwardingId && m_ackForFailureTokenTimeoutMap[j].owner == params.tokenOwnerId)
				{
					m_ackForFailureTokenTimeoutMap[j].noAck.Cancel();
					//m_ackForFailureTokenTimeoutMap[j].forwardId = 9999;
					//m_ackForFailureTokenTimeoutMap[j].owner 	= 9999;
				}
			/*	else if (params.singleNode == 1 && m_ackForFailureTokenTimeoutMap[j].forwardId == params.sourceId && m_ackForFailureTokenTimeoutMap[j].owner == params.tokenOwnerId)
				{
					std::cout<<" In DoRecvTokenAckNACK cancel noACK 20-20 case"<<std::endl;
					m_ackForFailureTokenTimeoutMap[j].noAck.Cancel();
				}*/
			}
			  //m_ackRecvMap.insert(std::pair<uint16_t,uint16_t>(params.forwardingId,params.sourceId));//  params.sourceId (C) is the source of NACK

			//when I received ack then mark all the path as sent ACK
				 ackRecv  tmpMap2;
				 tmpMap2.forwardingId 		= params.forwardingId;
				 tmpMap2.forwardingForId 	= params.sourceId;
				 tmpMap2.ownerId 			= m_cellId;
				 m_ackRecvMap.push_back(tmpMap2);

				 tmpMap2.forwardingId 		= params.sourceId;
				 tmpMap2.forwardingForId 	= params.forwardingId;
				 tmpMap2.ownerId 			= m_cellId;
				 m_ackRecvMap.push_back(tmpMap2);

		}
		else//NACK
		{
			uint16_t tmp = 0;

			std::cout<<Simulator::Now().GetMilliSeconds()<<"the m_aliveMap is updated for cell B = "<<params.forwardingId<<std::endl;
			auto it = m_aliveMap.find(params.forwardingId);
			if(it != m_aliveMap.end())
			{
				 tmp = it->second;
				 tmp++;
			   //m_aliveMap.insert(std::pair<uint16_t, uint16_t>(params.forwardingId,tmp));
				m_aliveMap.at(params.forwardingId) = tmp;
			}
			/*for (std::map<uint16_t, EventId>::iterator it =m_ackForFailureTokenTimeoutMap.begin() ; it != m_ackForFailureTokenTimeoutMap.end(); ++it)
			{
				if(params.forwardingId == it->first) // it  should be connected to Token owner so we do not cancel for other owners
				it->second.Cancel();
			}*/

			std::vector<AckForFailureTokenTimeout>::size_type sz = m_ackForFailureTokenTimeoutMap.size();
			for(int j = 0; j < (int)sz ; j++)
			{
				if(m_ackForFailureTokenTimeoutMap[j].forwardId == params.forwardingId && m_ackForFailureTokenTimeoutMap[j].owner == params.tokenOwnerId)
				{
					m_ackForFailureTokenTimeoutMap[j].noAck.Cancel();
					//m_ackForFailureTokenTimeoutMap[j].forwardId = 9999;
					//m_ackForFailureTokenTimeoutMap[j].owner = 9999;
				}
			}

		   // m_nackRecvMap.insert(std::pair<uint16_t,uint16_t>(params.forwardingId,params.sourceId));//  params.sourceId (C) is the source of NACK

			nackRecv tmpMap;
			tmpMap.forwardingId = params.forwardingId;
			tmpMap.forwardingForId = params.sourceId;//  params.sourceId (C) is the source of NACK
			tmpMap.ownerId = params.tokenOwnerId;
			m_nackRecvMap.push_back(tmpMap);

		}
	}//end of if power >0
}


void
LteEnbRrc::DoRecvUeData (EpcX2SapUser::UeDataParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this);

  NS_LOG_LOGIC ("Recv UE DATA FORWARDING through X2 interface");
  NS_LOG_LOGIC ("sourceCellId = " << params.sourceCellId);
  NS_LOG_LOGIC ("targetCellId = " << params.targetCellId);
  NS_LOG_LOGIC ("gtpTeid = " << params.gtpTeid);
  NS_LOG_LOGIC ("ueData = " << params.ueData);
  NS_LOG_LOGIC ("ueData size = " << params.ueData->GetSize ());

  std::map<uint32_t, X2uTeidInfo>::iterator 
    teidInfoIt = m_x2uTeidInfoMap.find (params.gtpTeid);
  if (teidInfoIt != m_x2uTeidInfoMap.end ())
    {
      GetUeManager (teidInfoIt->second.rnti)->SendData (teidInfoIt->second.drbid, params.ueData);
    }
  else
    {
      NS_FATAL_ERROR ("X2-U data received but no X2uTeidInfo found");
    }
	}//end of if power >0
}


uint16_t 
LteEnbRrc::DoAllocateTemporaryCellRnti ()
{

  NS_LOG_FUNCTION (this);
  return AddUe (UeManager::INITIAL_RANDOM_ACCESS);

}

void
LteEnbRrc::DoRrcConfigurationUpdateInd (LteEnbCmacSapUser::UeConfig cmacParams)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  Ptr<UeManager> ueManager = GetUeManager (cmacParams.m_rnti);
  ueManager->CmacUeConfigUpdateInd (cmacParams);
	}
}

void
LteEnbRrc::DoNotifyLcConfigResult (uint16_t rnti, uint8_t lcid, bool success)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  NS_FATAL_ERROR ("not implemented");
	}
}


uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForHandover (LteRrcSap::ReportConfigEutra reportConfig)
{

  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_handoverMeasIds.insert (measId);
  return measId;

}

void
LteEnbRrc::DoTriggerHandover (uint16_t rnti, uint16_t targetCellId)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this << rnti << targetCellId);

  bool isHandoverAllowed = true;

  if (m_anrSapProvider != 0)
    {
      // ensure that proper neighbour relationship exists between source and target cells

	  bool noHo = m_anrSapProvider->GetNoHo (targetCellId);
      bool noX2 = m_anrSapProvider->GetNoX2 (targetCellId);
      NS_LOG_DEBUG (this << " cellId=" << m_cellId
                         << " targetCellId=" << targetCellId
                         << " NRT.NoHo=" << noHo << " NRT.NoX2=" << noX2);

      if (noHo || noX2)
        {
          isHandoverAllowed = false;
          NS_LOG_LOGIC (this << " handover to cell " << targetCellId
                             << " is not allowed by ANR");
        }
    }


  Ptr<UeManager> ueManager = GetUeManager (rnti);
  NS_ASSERT_MSG (ueManager != 0, "Cannot find UE context with RNTI " << rnti);

  if (ueManager->GetState () != UeManager::CONNECTED_NORMALLY)
    {
      isHandoverAllowed = false;
      NS_LOG_LOGIC (this << " handover is not allowed because the UE"
                         << " rnti=" << rnti << " is in "
                         << ToString (ueManager->GetState ()) << " state");
    }

  if (isHandoverAllowed)
    {
      // initiate handover execution
      ueManager->PrepareHandover (targetCellId);

    }
	}//end of if power >0
}

uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForAnr (LteRrcSap::ReportConfigEutra reportConfig)
{

  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_anrMeasIds.insert (measId);
  return measId;

}

uint8_t
LteEnbRrc::DoAddUeMeasReportConfigForFfr (LteRrcSap::ReportConfigEutra reportConfig)
{

  NS_LOG_FUNCTION (this);
  uint8_t measId = AddUeMeasReportConfig (reportConfig);
  m_ffrMeasIds.insert (measId);
  return measId;

}

void
LteEnbRrc::DoSetPdschConfigDedicated (uint16_t rnti, LteRrcSap::PdschConfigDedicated pdschConfigDedicated)
{
	if(m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
  NS_LOG_FUNCTION (this);
  Ptr<UeManager> ueManager = GetUeManager (rnti);
  ueManager->SetPdschConfigDedicated (pdschConfigDedicated);
	}
}

void
LteEnbRrc::DoSendLoadInformation (EpcX2Sap::LoadInformationParams params)
{
	  //A.M , I added the if  here
	if( m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
		NS_LOG_FUNCTION (this);

		m_x2SapProvider->SendLoadInformation(params);
	}
}

uint16_t
LteEnbRrc::AddUe (UeManager::State state)
{

  NS_LOG_FUNCTION (this);

  bool found = false;
  uint16_t rnti;
  for (rnti = m_lastAllocatedRnti + 1; 
       (rnti != m_lastAllocatedRnti - 1) && (!found);
       ++rnti)
    {
      if ((rnti != 0) && (m_ueMap.find (rnti) == m_ueMap.end ()))
        {
          found = true;
          break;
        }
    }

  NS_ASSERT_MSG (found, "no more RNTIs available (do you have more than 65535 UEs in a cell?)");
  m_lastAllocatedRnti = rnti;
  Ptr<UeManager> ueManager = CreateObject<UeManager> (this, rnti, state);
  m_ueMap.insert (std::pair<uint16_t, Ptr<UeManager> > (rnti, ueManager));
  ueManager->Initialize ();
  //A.M
  ueManager->m_hysteresisHasUpdated = false;
  //A.M
  if(Simulator::Now().GetSeconds() > 4.0)
  {

	  std::cout<<Simulator::Now().GetSeconds()<<": In Cell ID "<<m_cellId<< ": UE ID RNTI = ("<<rnti<<") and IMSI = ("<< ueManager->GetImsi() <<") is ADDED Newly *********************"<<std::endl;
  }
  NS_LOG_DEBUG (this << " New UE RNTI " << rnti << " cellId " << m_cellId << " srs CI " << ueManager->GetSrsConfigurationIndex ());
  m_newUeContextTrace (m_cellId, rnti);
  return rnti;


}

void
LteEnbRrc::RemoveUe (uint16_t rnti)
{
  NS_LOG_FUNCTION (this << (uint32_t) rnti);
  std::map <uint16_t, Ptr<UeManager> >::iterator it = m_ueMap.find (rnti);

  NS_ASSERT_MSG (it != m_ueMap.end (), "request to remove UE info with unknown rnti " << rnti);

  uint16_t srsCi = (*it).second->GetSrsConfigurationIndex ();
  m_ueMap.erase (it);
  m_cmacSapProvider->RemoveUe (rnti);
  m_cphySapProvider->RemoveUe (rnti);
  if (m_s1SapProvider != 0)
    {
      m_s1SapProvider->UeContextRelease (rnti);
    }
  // need to do this after UeManager has been deleted
  RemoveSrsConfigurationIndex (srsCi); 

}

TypeId
LteEnbRrc::GetRlcType (EpsBearer bearer)
{
  switch (m_epsBearerToRlcMapping)
    {
    case RLC_SM_ALWAYS:
      return LteRlcSm::GetTypeId ();
      break;

    case  RLC_UM_ALWAYS:
      return LteRlcUm::GetTypeId ();
      break;

    case RLC_AM_ALWAYS:
      return LteRlcAm::GetTypeId ();
      break;

    case PER_BASED:
      if (bearer.GetPacketErrorLossRate () > 1.0e-5)
        {
          return LteRlcUm::GetTypeId ();
        }
      else
        {
          return LteRlcAm::GetTypeId ();
        }
      break;

    default:
      return LteRlcSm::GetTypeId ();
      break;
    }
}


void
LteEnbRrc::AddX2Neighbour (uint16_t cellId)
{
  NS_LOG_FUNCTION (this << cellId);

  if (m_anrSapProvider != 0)
    {
      m_anrSapProvider->AddNeighbourRelation (cellId);
    }
}

void
LteEnbRrc::SetCsgId (uint32_t csgId, bool csgIndication)
{
  NS_LOG_FUNCTION (this << csgId << csgIndication);
  m_sib1.cellAccessRelatedInfo.csgIdentity = csgId;
  m_sib1.cellAccessRelatedInfo.csgIndication = csgIndication;
  m_cphySapProvider->SetSystemInformationBlockType1 (m_sib1);
}


/// Number of distinct SRS periodicity plus one.
static const uint8_t SRS_ENTRIES = 9;
/**
 * Sounding Reference Symbol (SRS) periodicity (TSRS) in milliseconds. Taken
 * from 3GPP TS 36.213 Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsPeriodicity[SRS_ENTRIES] = {0, 2, 5, 10, 20, 40,  80, 160, 320};
/**
 * The lower bound (inclusive) of the SRS configuration indices (ISRS) which
 * use the corresponding SRS periodicity (TSRS). Taken from 3GPP TS 36.213
 * Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsCiLow[SRS_ENTRIES] =       {0, 0, 2,  7, 17, 37,  77, 157, 317};
/**
 * The upper bound (inclusive) of the SRS configuration indices (ISRS) which
 * use the corresponding SRS periodicity (TSRS). Taken from 3GPP TS 36.213
 * Table 8.2-1. Index starts from 1.
 */
static const uint16_t g_srsCiHigh[SRS_ENTRIES] =      {0, 1, 6, 16, 36, 76, 156, 316, 636};

void 
LteEnbRrc::SetSrsPeriodicity (uint32_t p)
{

  NS_LOG_FUNCTION (this << p);
  for (uint32_t id = 1; id < SRS_ENTRIES; ++id)
    {
      if (g_srsPeriodicity[id] == p)
        {
          m_srsCurrentPeriodicityId = id;
          return;
        }
    }
  // no match found
  std::ostringstream allowedValues;
  for (uint32_t id = 1; id < SRS_ENTRIES; ++id)
    {
      allowedValues << g_srsPeriodicity[id] << " ";
    }
  NS_FATAL_ERROR ("illecit SRS periodicity value " << p << ". Allowed values: " << allowedValues.str ());

}

uint32_t 
LteEnbRrc::GetSrsPeriodicity () const
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT (m_srsCurrentPeriodicityId > 0);
  NS_ASSERT (m_srsCurrentPeriodicityId < SRS_ENTRIES);
  return g_srsPeriodicity[m_srsCurrentPeriodicityId];
}


uint16_t
LteEnbRrc::GetNewSrsConfigurationIndex ()
{

  NS_LOG_FUNCTION (this << m_ueSrsConfigurationIndexSet.size ());
  // SRS
  NS_ASSERT (m_srsCurrentPeriodicityId > 0);
  NS_ASSERT (m_srsCurrentPeriodicityId < SRS_ENTRIES);
  NS_LOG_DEBUG (this << " SRS p " << g_srsPeriodicity[m_srsCurrentPeriodicityId] << " set " << m_ueSrsConfigurationIndexSet.size ());
  if (m_ueSrsConfigurationIndexSet.size () >= g_srsPeriodicity[m_srsCurrentPeriodicityId])
    {
      NS_FATAL_ERROR ("too many UEs (" << m_ueSrsConfigurationIndexSet.size () + 1 
                                       << ") for current SRS periodicity "
                                       <<  g_srsPeriodicity[m_srsCurrentPeriodicityId]
                                       << ", consider increasing the value of ns3::LteEnbRrc::SrsPeriodicity");
    }

  if (m_ueSrsConfigurationIndexSet.empty ())
    {
      // first entry
      m_lastAllocatedConfigurationIndex = g_srsCiLow[m_srsCurrentPeriodicityId];
      m_ueSrsConfigurationIndexSet.insert (m_lastAllocatedConfigurationIndex);
    }
  else
    {
      // find a CI from the available ones
      std::set<uint16_t>::reverse_iterator rit = m_ueSrsConfigurationIndexSet.rbegin ();
      NS_ASSERT (rit != m_ueSrsConfigurationIndexSet.rend ());
      NS_LOG_DEBUG (this << " lower bound " << (*rit) << " of " << g_srsCiHigh[m_srsCurrentPeriodicityId]);
      if ((*rit) < g_srsCiHigh[m_srsCurrentPeriodicityId])
        {
          // got it from the upper bound
          m_lastAllocatedConfigurationIndex = (*rit) + 1;
          m_ueSrsConfigurationIndexSet.insert (m_lastAllocatedConfigurationIndex);
        }
      else
        {
          // look for released ones
          for (uint16_t srcCi = g_srsCiLow[m_srsCurrentPeriodicityId]; srcCi < g_srsCiHigh[m_srsCurrentPeriodicityId]; srcCi++) 
            {
              std::set<uint16_t>::iterator it = m_ueSrsConfigurationIndexSet.find (srcCi);
              if (it == m_ueSrsConfigurationIndexSet.end ())
                {
                  m_lastAllocatedConfigurationIndex = srcCi;
                  m_ueSrsConfigurationIndexSet.insert (srcCi);
                  break;
                }
            }
        } 
    }
  return m_lastAllocatedConfigurationIndex;

}


void
LteEnbRrc::RemoveSrsConfigurationIndex (uint16_t srcCi)
{
  NS_LOG_FUNCTION (this << srcCi);
  std::set<uint16_t>::iterator it = m_ueSrsConfigurationIndexSet.find (srcCi);
  NS_ASSERT_MSG (it != m_ueSrsConfigurationIndexSet.end (), "request to remove unkwown SRS CI " << srcCi);
  m_ueSrsConfigurationIndexSet.erase (it);
}

uint8_t 
LteEnbRrc::GetLogicalChannelGroup (EpsBearer bearer)
{
  if (bearer.IsGbr ())
    {
      return 1;
    }
  else
    {
      return 2;
    }
}

uint8_t 
LteEnbRrc::GetLogicalChannelPriority (EpsBearer bearer)
{
  return bearer.qci;
}

void
LteEnbRrc::SendSystemInformation ()
{
	if(m_cphySapProvider->GetReferenceSignalPower() > 0.0)
	{
	  // NS_LOG_FUNCTION (this);

	  /*
	   * For simplicity, we use the same periodicity for all SIBs. Note that in real
	   * systems the periodicy of each SIBs could be different.
	   */
	  LteRrcSap::SystemInformation si;
	  si.haveSib2 = true;
	  si.sib2.freqInfo.ulCarrierFreq = m_ulEarfcn;
	  si.sib2.freqInfo.ulBandwidth = m_ulBandwidth;
	  si.sib2.radioResourceConfigCommon.pdschConfigCommon.referenceSignalPower = m_cphySapProvider->GetReferenceSignalPower();
	  si.sib2.radioResourceConfigCommon.pdschConfigCommon.pb = 0;

	  LteEnbCmacSapProvider::RachConfig rc = m_cmacSapProvider->GetRachConfig ();
	  LteRrcSap::RachConfigCommon rachConfigCommon;
	  rachConfigCommon.preambleInfo.numberOfRaPreambles = rc.numberOfRaPreambles;
	  rachConfigCommon.raSupervisionInfo.preambleTransMax = rc.preambleTransMax;
	  rachConfigCommon.raSupervisionInfo.raResponseWindowSize = rc.raResponseWindowSize;
	  si.sib2.radioResourceConfigCommon.rachConfigCommon = rachConfigCommon;

	  m_rrcSapUser->SendSystemInformation (si);
	  Simulator::Schedule (m_systemInformationPeriodicity, &LteEnbRrc::SendSystemInformation, this);
	}
}

} // namespace ns3

