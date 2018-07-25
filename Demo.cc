/*
 * Demo.cc
 *
 *  Created on: 9.4.2018
 *      Author: tupevarj
 */


#include <ns3/lte-module.h>
#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/internet-module.h>
#include <ns3/config-store-module.h>
#include <ns3/buildings-module.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/applications-module.h>
#include <ns3/log.h>
#include <ns3/rng-seed-manager.h>

using namespace ns3;


//////////////////////////////////////////////////////////////////
// Simple element structure for CSVWriter
//////////////////////////////////////////////////////////////////
// for some number of UEs the lte-stats-calculator.cc gives error "got no matches", e.g; num = 150 give error
const int NUMBER_OF_UES = 300;

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti,
							   double rsrp)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
			<< " & RSRP = " << rsrp
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ":  connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

//A.M
void
NotifyConnectionTimeout(std::string context,
						uint64_t imsi,
						uint16_t cellId,
						uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellId
            << " with RNTI " << rnti
            << ", ConnectionTimeout "
            << std::endl;
}
//A.M
void
NotifySendResourceStatusRequest (std::string context,
                               uint16_t src,
                               uint16_t dst)
{
  std::cout << context
            << " Source Id " << src
            << ": Send a Resource Status Request to CellId " << dst
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}













/* Event name enum */
enum EventName
{
	RLF,
	OutOfSynch,
	A3RSRPEnter,
	A2RSRQLeave,
	A2RSRPLeave,
	A2RSRPEnter,
	A2RSRQEnter
};

/* Handover name enum */
enum HandoverEventName
{
	HandoverStart,
	HandoverEndOK
};

struct WritableLog
{
	virtual std::string ConvertToCSV() = 0;
protected:
	virtual ~WritableLog() {};
};

/*
 *  Output file structure for main KPIs:
 *  - time
 *  - x position
 *  - y position
 *  - imsi
 *  - cell ID
 *  - RSRP
 *  - RSRQ
 */
struct MainKpiLog : public WritableLog
{
	///////////////////////////////////
	// MEMBERS
	///////////////////////////////////

	double time;
	double x;
	double y;
	double rsrp;
	double rsrq;
	uint64_t imsi;
	uint16_t cellId;
	bool connected;

	///////////////////////////////////
	// PUBLIC METHODS
	///////////////////////////////////

	MainKpiLog(double time, double x, double y, double rsrp, double rsrq, uint64_t imsi, uint16_t cellId, bool connected)
			: time(time), x(x), y(y), rsrp(rsrp), rsrq(rsrq), imsi(imsi), cellId(cellId), connected(connected)
	{

	}

	inline
	std::string ConvertToCSV() override
	{
		std::ostringstream strs;
		strs << time << "," << x << "," << y << "," << imsi << "," << cellId << "," << rsrp << "," << rsrq << "," << connected;
		return strs.str();
	}

	static std::string GetFileName()
	{
		return "main_kpis_log";
	}
};


/*
 *  Output file structure for events:
 *  - time
 *  - x position
 *  - y position
 *  - imsi
 *  - cell ID
 *  - Handover Event (int)
 *  - RSRP or RSRQ (depends on event!)
 */
struct EventLog : public WritableLog
{
	///////////////////////////////////
	// MEMBERS
	///////////////////////////////////

	double time;
	int x;
	int y;
	double rsr;
	uint64_t imsi;
	uint16_t cellId;
	EventName e;

	///////////////////////////////////
	// PUBLIC METHODS
	///////////////////////////////////

	EventLog(double time, double x, double y, double rsr, uint64_t imsi, uint16_t cellId, EventName e)
		: time(time), x(x), y(y), rsr(rsr), imsi(imsi), cellId(cellId), e(e)
	{

	}

	inline
	std::string ConvertToCSV() override
	{
		std::ostringstream strs;
		strs << time << "," << x << "," << y << "," << imsi << "," << e << "," << cellId << "," << rsr;
		return strs.str();
	}

	static std::string GetFileName()
	{
		return "event_log";
	}
};

/*
 *  Output file structure for handovers:
 *  - time
 *  - x position
 *  - y position
 *  - imsi
 *  - cell ID
 *  - Handover Event (int)
 *  - Target Cell ID (optional)
 */
struct HandoverLog : public WritableLog
{
	///////////////////////////////////
	// MEMBERS
	///////////////////////////////////

	double time;
	int x;
	int y;
	uint64_t imsi;
	uint16_t cellId;
	uint16_t targetCellId;
	HandoverEventName e;

	///////////////////////////////////
	// PUBLIC METHODS
	///////////////////////////////////

	HandoverLog(double time, double x, double y, uint64_t imsi, uint16_t cellId, uint16_t targetCellId, HandoverEventName e)
			: time(time), x(x), y(y), imsi(imsi), cellId(cellId), targetCellId(targetCellId), e(e)
	{

	}

	inline
	std::string ConvertToCSV() override
	{
		std::ostringstream strs;
		strs << time << "," << x << "," << y << "," << imsi << "," << e << "," << cellId << "," << targetCellId;
		return strs.str();
	}

	static std::string GetFileName()
	{
		return "handover_log";
	}
};

/*
 *  Output file structure for SINR:
 *  - time
 *  - imsi
 *  - cell ID
 *  - SINR
 */
struct SinrLog : public WritableLog
{
	///////////////////////////////////
	// MEMBERS
	///////////////////////////////////

	double time;
	double sinr;
	uint64_t imsi;
	uint16_t cellId;

	///////////////////////////////////
	// PUBLIC METHODS
	///////////////////////////////////

	SinrLog(double time, double sinr, uint64_t imsi, uint16_t cellId)
			: time(time), sinr(sinr), imsi(imsi), cellId(cellId)
	{

	}

	inline
	std::string ConvertToCSV() override
	{
		std::ostringstream strs;
		strs << time << "," << imsi << "," << cellId << "," << sinr;
		return strs.str();
	}

	static std::string GetFileName()
	{
		return "sinr_log";
	}
};

/*
 *  Output file structure for throuhgput:
 *  - time
 *  - imsi
 *  - cell ID
 *  - throughput
 */
struct ThrouhgputLog : public WritableLog
{
	///////////////////////////////////
	// MEMBERS
	///////////////////////////////////

	double time;
	uint64_t throughput;
	uint64_t imsi;
	uint16_t cellId;

	///////////////////////////////////
	// PUBLIC METHODS
	///////////////////////////////////

	ThrouhgputLog(double time, uint64_t throughput, uint64_t imsi, uint16_t cellId)
				: time(time), throughput(throughput), imsi(imsi), cellId(cellId)
	{

	}

	inline
	std::string ConvertToCSV() override
	{
		std::ostringstream strs;
		strs << time << "," << imsi << "," << cellId << "," << throughput;
		return strs.str();
	}

	static std::string GetFileName()
	{
		return "throughput_log";
	}
};



//////////////////////////////////////////////////////////////////
// Simple topology helper
//////////////////////////////////////////////////////////////////

class DemoHelperII
{

public:
//EARFCN: Evolved-UTRA Absolute Radio Frequency No (LTE carrier channel NO's)
	static void
	CreateHexagonalTopology(Ptr<PointToPointEpcHelper>& epcHelper, MobilityHelper& mobility, Box& macroUeBox, Ptr <LteHelper> lteHelper, uint32_t nEnbs, uint32_t nEnbsX, double interSiteDistance,
										bool epc, double enbsTx, uint16_t enbEARFCN, uint16_t enbBandwidth)
	{

		//////////////////////////////////////////////////////////////////
		// MACRO UE BOX
		//////////////////////////////////////////////////////////////////

		double areaMarginFactor = 0.5;
 // More clarification is required about this part of the code
		if (nEnbs > 0)
		{
		    uint32_t currentSite = nEnbs -1;
		    uint32_t biRowIndex = (currentSite / (nEnbsX + nEnbsX + 1));
		    uint32_t biRowRemainder = currentSite % (nEnbsX + nEnbsX + 1);
		    uint32_t rowIndex = biRowIndex*2 + 1;
		    if (biRowRemainder >= nEnbsX)
		    {
		        ++rowIndex;
		    }
		    uint32_t nMacroEnbSitesY = rowIndex;

		    macroUeBox = Box (-areaMarginFactor*interSiteDistance, (nEnbsX + areaMarginFactor)*interSiteDistance, -areaMarginFactor*interSiteDistance,
		                      (nMacroEnbSitesY -1)*interSiteDistance*sqrt (0.75) + areaMarginFactor*interSiteDistance, 1.5, 1.5);
		  }
		else
		{
			macroUeBox = Box (0, 150, 0, 150, 1.5, 1.5);
		}


		//////////////////////////////////////////////////////////////////
		// ENBS
		//////////////////////////////////////////////////////////////////

		NodeContainer macroEnbs;
		macroEnbs.Create (3 * nEnbs);

		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");


		lteHelper->SetAttribute("PathlossModel", StringValue("ns3::FriisPropagationLossModel"));
		//lteHelper->SetAttribute("PathlossModel", StringValue("ns3::BuildingsPropagationLossModel"));

		lteHelper->SetSpectrumChannelType ("ns3::MultiModelSpectrumChannel");

		if (epc)
		{
			epcHelper = CreateObject<PointToPointEpcHelper> ();
			lteHelper->SetEpcHelper (epcHelper);
		}

		mobility.Install (macroEnbs);
		BuildingsHelper::Install (macroEnbs);
		Ptr<LteHexGridEnbTopologyHelper> lteHexGridEnbTopologyHelper = CreateObject<LteHexGridEnbTopologyHelper> ();
		lteHexGridEnbTopologyHelper->SetLteHelper (lteHelper);
		lteHexGridEnbTopologyHelper->SetAttribute ("InterSiteDistance", DoubleValue (interSiteDistance));
		lteHexGridEnbTopologyHelper->SetAttribute ("MinX", DoubleValue (interSiteDistance/2));
		lteHexGridEnbTopologyHelper->SetAttribute ("GridWidth", UintegerValue (nEnbsX));
		Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbsTx));
		lteHelper->SetEnbAntennaModelType ("ns3::ParabolicAntennaModel");
		lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (70));
		lteHelper->SetEnbAntennaModelAttribute ("MaxAttenuation",     DoubleValue (20.0));
		lteHelper->SetEnbDeviceAttribute ("DlEarfcn", UintegerValue (enbEARFCN));
		lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (enbEARFCN + 18000));
		lteHelper->SetEnbDeviceAttribute ("DlBandwidth", UintegerValue (enbBandwidth));
		lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (enbBandwidth));
		NetDeviceContainer macroEnbDevs = lteHexGridEnbTopologyHelper->SetPositionAndInstallEnbDevice (macroEnbs);

		if (epc)
		{
			lteHelper->AddX2Interface (macroEnbs);
		}

		///////////////////////////////////////////////////////////////////////////
		//	ALLOW UE MEASUREMENTS
		///////////////////////////////////////////////////////////////////////////

		LteRrcSap::ReportConfigEutra config;
		config.eventId = LteRrcSap::ReportConfigEutra::EVENT_A2;
		config.threshold1.choice = LteRrcSap::ThresholdEutra::THRESHOLD_RSRQ;
		config.threshold1.range = 26;
		config.triggerQuantity = LteRrcSap::ReportConfigEutra::RSRQ;
		config.reportInterval = LteRrcSap::ReportConfigEutra::MS240;
		config.timeToTrigger = 240;

		std::vector<uint8_t> measIdList;

		NetDeviceContainer::Iterator it;
		for (it = macroEnbDevs.Begin (); it != macroEnbDevs.End (); it++)
		{
		  Ptr<NetDevice> dev = *it;
		  Ptr<LteEnbNetDevice> enbDev = dev->GetObject<LteEnbNetDevice> ();
		  Ptr<LteEnbRrc> enbRrc = enbDev->GetRrc ();

		  uint8_t measId = enbRrc->AddUeMeasReportConfig (config);
		  measIdList.push_back (measId);
		}
	}

	static void
	CreateUsers(NodeContainer& macroUes, MobilityHelper& mobility, Box& macroUeBox, Ptr <LteHelper> lteHelper, uint32_t nUes, double uesSpeed)
	{
		macroUes.Create (nUes);

		Ptr<PositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator> ();
		mobility.SetPositionAllocator (positionAlloc);
		lteHelper->SetUeDeviceAttribute ("CsgId", UintegerValue (1));
		// (CSG) Closed Subscriber group ID, limited set of users with connectivity access to a femtocell.
		// when a femtocell is configured in CSG mode, only those users included in the femtocell
		// access control list are allowed to use the femtocell resources.

		if (uesSpeed!=0.0)
		{
			mobility.SetMobilityModel ("ns3::SteadyStateRandomWaypointMobilityModel");
		    Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinX", DoubleValue (macroUeBox.xMin));
		    Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinY", DoubleValue (macroUeBox.yMin));
		    Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxX", DoubleValue (macroUeBox.xMax));
		    Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxY", DoubleValue (macroUeBox.yMax));
		    Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::Z", DoubleValue (1.5));
			Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxSpeed", DoubleValue (uesSpeed));
			Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinSpeed", DoubleValue (uesSpeed));

			positionAlloc = CreateObject<RandomBoxPositionAllocator> ();
			mobility.SetPositionAllocator (positionAlloc);
			mobility.Install (macroUes);

			for (NodeContainer::Iterator it = macroUes.Begin (); it != macroUes.End (); ++it)
			{
				(*it)->Initialize ();
			}
		}
		else
		{
			positionAlloc = CreateObject<RandomBoxPositionAllocator> ();
			Ptr<UniformRandomVariable> xVal = CreateObject<UniformRandomVariable> ();
			xVal->SetAttribute ("Min", DoubleValue (macroUeBox.xMin));
			xVal->SetAttribute ("Max", DoubleValue (macroUeBox.xMax));
			positionAlloc->SetAttribute ("X", PointerValue (xVal));
			Ptr<UniformRandomVariable> yVal = CreateObject<UniformRandomVariable> ();
			yVal->SetAttribute ("Min", DoubleValue (macroUeBox.yMin));
			yVal->SetAttribute ("Max", DoubleValue (macroUeBox.yMax));
			positionAlloc->SetAttribute ("Y", PointerValue (yVal));
			Ptr<UniformRandomVariable> zVal = CreateObject<UniformRandomVariable> ();
			zVal->SetAttribute ("Min", DoubleValue (macroUeBox.zMin));
			zVal->SetAttribute ("Max", DoubleValue (macroUeBox.zMax));
			positionAlloc->SetAttribute ("Z", PointerValue (zVal));
			mobility.SetPositionAllocator (positionAlloc);
			mobility.Install (macroUes);
		}
		BuildingsHelper::Install (macroUes);
	}

	static void
	CreateEPC(Ptr<PointToPointEpcHelper>& epcHelper, Ptr <LteHelper> lteHelper, NodeContainer& macroUes, bool useUdp, bool epcDl, bool epcUl, uint16_t nBearersPerUe)
	{
		NetDeviceContainer macroUeDevs = lteHelper->InstallUeDevice (macroUes);

		Ipv4Address remoteHostAddr;
		NodeContainer ues;
		Ipv4StaticRoutingHelper ipv4RoutingHelper;
		Ipv4InterfaceContainer ueIpIfaces;
		Ptr<Node> remoteHost;
		NetDeviceContainer ueDevs;

		// Create a single RemoteHost
		NodeContainer remoteHostContainer;
		remoteHostContainer.Create (1);
		remoteHost = remoteHostContainer.Get (0);
		InternetStackHelper internet;
		internet.Install (remoteHostContainer);

		// Create the Internet
		PointToPointHelper p2ph;
		p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
		p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
		p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
		Ptr<Node> pgw = epcHelper->GetPgwNode ();
		NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
		Ipv4AddressHelper ipv4h;
		ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
		Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
		// in this container, interface 0 is the pgw, 1 is the remoteHost
		remoteHostAddr = internetIpIfaces.GetAddress (1);

		Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
		remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

		// for internetworking purposes, consider together home UEs and macro UEs
		ues.Add (macroUes);
		ueDevs.Add (macroUeDevs);

		// Install the IP stack on the UEs
		internet.Install (ues);
		ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

		// attachment (needs to be done after IP stack configuration)
		// using initial cell selection
		lteHelper->Attach (macroUeDevs);


		// Install and start applications on UEs and remote host
		uint16_t dlPort = 10000;
		uint16_t ulPort = 20000;

		// randomize a bit start times to avoid simulation artifacts
		// (e.g., buffer overflows due to packet transmissions happening
		// exactly at the same time)
		Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
		if (useUdp)
		  {
			startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
			startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));
		  }
		else
		  {
			// TCP needs to be started late enough so that all UEs are connected
			// otherwise TCP SYN packets will get lost
			startTimeSeconds->SetAttribute ("Min", DoubleValue (0.100));
			startTimeSeconds->SetAttribute ("Max", DoubleValue (0.110));
		  }

		for (uint32_t u = 0; u < ues.GetN (); ++u)
			  {
				Ptr<Node> ue = ues.Get (u);
				// Set the default gateway for the UE
				Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
				ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

				for (uint32_t b = 0; b < nBearersPerUe; ++b)
				  {
					++dlPort;
					++ulPort;

					ApplicationContainer clientApps;
					ApplicationContainer serverApps;

					if (useUdp)
					  {
						if (epcDl)
						  {
							UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
							clientApps.Add (dlClientHelper.Install (remoteHost));
							PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
																 InetSocketAddress (Ipv4Address::GetAny (), dlPort));
							serverApps.Add (dlPacketSinkHelper.Install (ue));
							//test.SetAttribute ("DataRate", DataRateValue (DataRate(100.0)));
						  }
						if (epcUl)
						  {
							UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
							clientApps.Add (ulClientHelper.Install (ue));
							PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
																 InetSocketAddress (Ipv4Address::GetAny (), ulPort));
							serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
						  }
					  }
					else // use TCP
					  {
						if (epcDl)
						  {
							BulkSendHelper dlClientHelper ("ns3::TcpSocketFactory",
														   InetSocketAddress (ueIpIfaces.GetAddress (u), dlPort));
							dlClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));
							clientApps.Add (dlClientHelper.Install (remoteHost));
							PacketSinkHelper dlPacketSinkHelper ("ns3::TcpSocketFactory",
																 InetSocketAddress (Ipv4Address::GetAny (), dlPort));
							serverApps.Add (dlPacketSinkHelper.Install (ue));
						  }
						if (epcUl)
						  {
							BulkSendHelper ulClientHelper ("ns3::TcpSocketFactory",
														   InetSocketAddress (remoteHostAddr, ulPort));
							ulClientHelper.SetAttribute ("MaxBytes", UintegerValue (0));
							clientApps.Add (ulClientHelper.Install (ue));
							PacketSinkHelper ulPacketSinkHelper ("ns3::TcpSocketFactory",
																 InetSocketAddress (Ipv4Address::GetAny (), ulPort));
							serverApps.Add (ulPacketSinkHelper.Install (remoteHost));
						  }
					  } // end if (useUdp)

					Ptr<EpcTft> tft = Create<EpcTft> ();
					if (epcDl)
					  {
						EpcTft::PacketFilter dlpf;
						dlpf.localPortStart = dlPort;
						dlpf.localPortEnd = dlPort;
						tft->Add (dlpf);
					  }
					if (epcUl)
					  {
						EpcTft::PacketFilter ulpf;
						ulpf.remotePortStart = ulPort;
						ulpf.remotePortEnd = ulPort;
						tft->Add (ulpf);
					  }

					if (epcDl || epcUl)
					  {
						//EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
						EpsBearer bearer (EpsBearer::GBR_CONV_VIDEO);

						GbrQosInformation qos;
						qos.gbrDl = 500000; // Downlink GBR //0.5 Mbps

						lteHelper->ActivateDedicatedEpsBearer (ueDevs.Get (u), bearer, tft);
					  }
					Time startTime = Seconds (startTimeSeconds->GetValue ());
					serverApps.Start (startTime);
					clientApps.Start (startTime);

				  } // end for b
			  }
	}
};
/* https://www.quora.com/Can-we-assign-a-GBR-guaranteed-bit-rate-on-TCP-based-services-in-Evolved-Packet-Core
 * GBR bearer type needs to be selected and defined in the PCRF policy.
    QCI=1 is not to be used as this is now reserved for voice flows using the VoLTE profile.
    QCI=2 and 4 might be ok as these was originally planned to carry video.
    QCI=3 is another choice and it was planned to be used for real time gaming.
 */

//////////////////////////////////////////////////////////////////
// Communication between ns-3 and mongo
//////////////////////////////////////////////////////////////////


class CSVWriter
{
	std::vector<EventLog> eventsLog;
	std::vector<HandoverLog> handoversLog;
	std::vector<SinrLog> sinrsLog;
	std::vector<ThrouhgputLog> throughputsLog;
	std::vector<MainKpiLog> mainKpisLog;

public:

	CSVWriter(){}

	void
	LogMainKpis(double time, double x, double y, uint64_t imsi, uint16_t cellId, double rsrp, double rsrq, bool connected)
	{
		mainKpisLog.push_back(MainKpiLog{time, x, y, rsrp, rsrq, imsi, cellId, connected});
	}

	void
	LogEvent(double time, double x, double y,uint64_t imsi, EventName e, uint16_t cellId, double rsr)
	{
		eventsLog.push_back(EventLog{time, x, y, rsr, imsi, cellId, e});
	}

	void
	LogHandover(double time, double x, double y,uint64_t imsi, HandoverEventName e, uint16_t cellId, uint16_t targetCellId)
	{
		handoversLog.push_back(HandoverLog{time, x, y, imsi, cellId, targetCellId, e});
	}

	void
	LogSinr(double time, uint64_t imsi, uint16_t cellId, double sinr)
	{
		sinrsLog.push_back(SinrLog{time, sinr, imsi, cellId});
	}

	void
	LogThroughput(double time, uint64_t imsi, uint16_t cellId, uint64_t thr)
	{
		throughputsLog.push_back(ThrouhgputLog{time, thr, imsi, cellId});
	}


	void
	FlushLogs(std::string path)
	{
		if(eventsLog.size() > 0)
		{
			std::ofstream outFile;
			outFile.open((path + EventLog::GetFileName() + ".csv").c_str(), std::ios_base::app);
			for(uint i = 0; i < eventsLog.size(); ++i)
			{
				outFile	<< eventsLog[i].ConvertToCSV() << "\n";
			}
			outFile.close();
		}
		eventsLog.clear();
		if(handoversLog.size() > 0)
		{
			std::ofstream outFile;
			outFile.open((path + HandoverLog::GetFileName() + ".csv").c_str(), std::ios_base::app);
			for(uint i = 0; i < handoversLog.size(); ++i)
			{
				outFile	<< handoversLog[i].ConvertToCSV() << "\n";
			}
			outFile.close();
		}
		handoversLog.clear();
		if(sinrsLog.size() > 0)
		{
			std::ofstream outFile;
			outFile.open((path + SinrLog::GetFileName() + ".csv").c_str(), std::ios_base::app);
			for(uint i = 0; i < sinrsLog.size(); ++i)
			{
				outFile	<< sinrsLog[i].ConvertToCSV() << "\n";
			}
			outFile.close();
		}
		sinrsLog.clear();
		if(throughputsLog.size() > 0)
		{
			std::ofstream outFile;
			outFile.open((path + ThrouhgputLog::GetFileName() + ".csv").c_str(), std::ios_base::app);
			for(uint i = 0; i < throughputsLog.size(); ++i)
			{
				outFile	<< throughputsLog[i].ConvertToCSV() << "\n";
			}
			outFile.close();
		}
		throughputsLog.clear();
		if(mainKpisLog.size() > 0)
		{
			std::ofstream outFile;
			outFile.open((path + MainKpiLog::GetFileName() + ".csv").c_str(), std::ios_base::app);
			for(uint i = 0; i < mainKpisLog.size(); ++i)
			{
				outFile	<< mainKpisLog[i].ConvertToCSV() << "\n";
			}
			outFile.close();
		}
		mainKpisLog.clear();
	}
};

CSVWriter csvWriter;

Vector3D
GetUeLocation(uint64_t imsi)
{
	// Loop through all the nodes:
	for(NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
	{
		Ptr<Node> node = *it;
		int nDevices = node->GetNDevices();

		// Loop through all devices on node:
		for(int i = 0; i < nDevices; i++)
		{
			Ptr<LteUeNetDevice> ueDevice = node->GetDevice(i)->GetObject<LteUeNetDevice>();

			// test NULL
			if(ueDevice && ueDevice->GetImsi() == imsi)
			{
				Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
				return mob->GetPosition();
			}
		}
	}
	return Vector3D(0,0,0);
}

void
KpiTestCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrp, double rsrq, uint16_t connected)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogMainKpis(Simulator::Now().GetSeconds(), location.x, location.y, imsi, cellId, rsrp, rsrq, connected);

}

void
HandoverEndOkCallback(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogHandover(Simulator::Now().GetSeconds(), (int)location.x, (int)location.y, imsi, HandoverEndOK, cellId, 0);
}


void
HandoverStartCallback(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti, uint16_t targetCellId)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogHandover(Simulator::Now().GetSeconds(), location.x, location.y, imsi, HandoverStart, cellid, targetCellId);
}

void
A2RsrqEnterCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrq)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, A2RSRQEnter, cellId, rsrq);
}

void
A2RsrpEnterCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrq)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, A2RSRPEnter, cellId, rsrq);
}

void
A2RsrpLeaveCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsr)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, A2RSRPLeave, cellId, rsr);
}

void
A2RsrqLeaveCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsr)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, A2RSRQLeave, cellId, rsr);
}

void
A3RsrpEnterCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrp)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, A3RSRPEnter, cellId, rsrp);
}

void
OutOfSynchCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrp, double thresh)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, OutOfSynch, cellId, rsrp);
}

void
RadioLinkFailureCallback(std::string context, uint64_t imsi, uint16_t cellId, double rsrp)
{
	Vector3D location = GetUeLocation(imsi);
	csvWriter.LogEvent(Simulator::Now().GetSeconds(), location.x, location.y, imsi, RLF, cellId, rsrp);
}

void
ReportUeSinr(double time, uint64_t imsi, uint16_t cellId, double sinr)
{
	csvWriter.LogSinr(time, imsi, cellId, sinr); // Converted into dBs in PhyStatsCalculator class.
}

uint16_t
GetConnectedCell(uint64_t imsi)
{
	// Loop through all the nodes:
	for(NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
	{
		Ptr<Node> node = *it;
		int nDevices = node->GetNDevices();

		// Loop through all devices on node:
		for(int i = 0; i < nDevices; i++)
		{
			Ptr<LteUeNetDevice> ueDevice = node->GetDevice(i)->GetObject<LteUeNetDevice>();

			// test NULL
			if(ueDevice && ueDevice->GetImsi() == imsi)
			{
				return ueDevice->GetRrc()->GetCellId();
			}
		}
	}
	return 0;
}
uint64_t throughputTmp = 0;
uint64_t throughputTmpCount = 0;
std::ofstream outFile ("/home/ahmasri/ns-allinone-3.26/ns-3.26/simulation_data/test.csv", std::ofstream::out);
void
WriteUeThroughPut(Ptr<RadioBearerStatsCalculator> rlcStats)
{

	for(int i = 1; i <= NUMBER_OF_UES; ++i)
	{
		uint64_t rxBytes = rlcStats->GetDlRxData(i , 4) * 8.0; // bits
		rlcStats->WriteDlResults(outFile);
		if(Simulator::Now().GetMilliSeconds() >= 3200 && rxBytes != 0.0)
		{
			throughputTmp += rxBytes;
			throughputTmpCount++;
		}
		csvWriter.LogThroughput(Simulator::Now().GetSeconds(), uint64_t(i), GetConnectedCell(i), rxBytes);
	}
	if(Simulator::Now().GetMilliSeconds() == 30000)
	{
		std::cout<<Simulator::Now().GetMilliSeconds()<<": Avg Throughput = "<< double((throughputTmp*1.0)/(throughputTmpCount*1.0))<<std::endl;
	}

	Simulator::Schedule(MilliSeconds (1000), &WriteUeThroughPut, rlcStats);
	std::cout<<"Simulation Time: "<<Simulator::Now().GetMilliSeconds()<<std::endl;
	std::cout<<"throughputTmp: "<<throughputTmp<<" and throughputTmpCount: "<<throughputTmpCount<<std::endl;
}

static ns3::GlobalValue g_training_bs("pathOut",
									"Path for output files ending slash, example:"
									"/home/simulation_data/",
									ns3::StringValue ("/home/ahmasri/ns-allinone-3.26/ns-3.26/simulation_data_1/"),
									ns3::MakeStringChecker());


//////////////////////////////////////////////////////////////////
// Main demo
//////////////////////////////////////////////////////////////////
//Configuration of LTE model parameters¶
//https://www.nsnam.org/docs/models/html/lte-user.html#configuration-of-lte-model-parameters

int
main (int argc, char *argv[])
{

	CommandLine cmd;
	cmd.Parse (argc, argv);
	ConfigStore inputConfig;
	inputConfig.ConfigureDefaults ();
	cmd.Parse (argc, argv);

	double ueSpeed = 4.16; // 15 km/h
	int numEnb 	   = 7;

	StringValue stringValue;
	GlobalValue::GetValueByName ("pathOut", stringValue);
	std::string path = stringValue.Get();

	Ptr <LteHelper> lteHelper = CreateObject<LteHelper> ();
	lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");

	//A.M
	//lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue (rand()%15)); // default
	lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue (6.0)); // default

	//lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
	//lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");

	lteHelper->SetSchedulerType ("ns3::CqaFfMacScheduler");
	lteHelper->SetSchedulerAttribute("HarqEnabled", BooleanValue(false)); //HARQ OFF, it is true by default





	Ptr<PointToPointEpcHelper> epcHelper;
	MobilityHelper mobility;
	Box macroUeBox;
	NetDeviceContainer macroUeDevs;
	NodeContainer macroUes;
	Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (160)); //Sound Reference Signal

	RngSeedManager::SetSeed(3);

	///////////////////////////////////////////////
	// CREATE TOPOLOGY
	///////////////////////////////////////////////

	//	CreateHexagonalTopology(Ptr<PointToPointEpcHelper>& epcHelper, MobilityHelper& mobility, Box& macroUeBox, Ptr <LteHelper> lteHelper, uint32_t nEnbs, uint32_t nEnbsX, double interSiteDistance,
	//bool epc, double enbsTx, uint16_t enbEARFCN, uint16_t enbBandwidth)


	// bandwidth in PRBs will be mapped to  MHz of bandwidth in LteSpectrumValueHelper::GetChannelBandwidth (uint8_t transmissionBandwidth)
	// and so bandwidth =   6 ==> 1.4 MHz, 15 ==> 3MHz , 25 ==> 5 MHz, 50 ==> 10 MHz,  75 ==> 15 MHz, 100 ==> 20 MHz
	DemoHelperII::CreateHexagonalTopology(epcHelper, mobility, macroUeBox, lteHelper, numEnb, 2, 750, true, 43.00, 100, 25);

	///////////////////////////////////////////////
	// CREATE USERS
	///////////////////////////////////////////////

	DemoHelperII::CreateUsers(macroUes, mobility, macroUeBox, lteHelper, NUMBER_OF_UES, ueSpeed); // set Speed here

	///////////////////////////////////////////////
	// CREATE INTERNET
	///////////////////////////////////////////////

	DemoHelperII::CreateEPC(epcHelper, lteHelper, macroUes, false, true, true, 1); // TCP activated

	///////////////////////////////////////////////
	// CONNECT TO DATABASE
	///////////////////////////////////////////////

//	mongo.CreateConnectionToDataBase();
//	mongo.SetDatabase("5gopt");
//	mongo.DropDatabase();
//	mongo.SetDatabase("5gopt");

	///////////////////////////////////////////////
	// SET CALLBACKS
	///////////////////////////////////////////////

	lteHelper->EnableTraces(); // Remember for RLF to disable the UL traces for the phys layer !!!!!!!!!!!!!!!!!!!!!!!!!
	//A.M
	/* I disabled the UL phystat because it cause error when cell is down
	 * /NodeList/0/DeviceList/0/LteEnbRrc/UeMap/0
		msg="Lookup /NodeList/0/DeviceList/0/LteEnbRrc/UeMap/0 got no matches", file=../src/lte/helper/lte-stats-calculator.cc, line=163
		terminate called without an active exception
	 *
	 * LteHelper::EnableTraces (void)
		{
  	  	  //EnablePhyTraces ();
	 */

	// Throuhput
	Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
	rlcStats->SetAttribute ("StartTime", TimeValue (Seconds (0)));
	Simulator::Schedule(MilliSeconds (1000), &WriteUeThroughPut, rlcStats);


	// RLF events
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/RadioLinkFailure", MakeCallback (&RadioLinkFailureCallback));
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/OutOfSynch", MakeCallback (&OutOfSynchCallback));
	// A2 events
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/A2RsrqEnter", MakeCallback (&A2RsrqEnterCallback));
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/A2RsrqLeave", MakeCallback (&A2RsrqLeaveCallback));
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/A2RsrpLeave", MakeCallback (&A2RsrpLeaveCallback));
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/A2RsrpEnter", MakeCallback (&A2RsrpEnterCallback));
	// A3 events
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/A3Rsrp", MakeCallback (&A3RsrpEnterCallback));
	// Handover events
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart", MakeCallback (&HandoverStartCallback));
	Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk", MakeCallback (&HandoverEndOkCallback));
	// Main Kpis
	Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/KpiTest",  MakeCallback(&KpiTestCallback));
	// SINR
	Ptr<PhyStatsCalculator> phyStats = lteHelper->GetPhyStatsCalculator();
	phyStats->TraceConnectWithoutContext("SinrUeTrace", MakeCallback(&ReportUeSinr));


	 // connect custom trace sinks for RRC connection establishment and handover notification

	  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
	                   MakeCallback (&NotifyConnectionEstablishedEnb));
	  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
	                   MakeCallback (&NotifyConnectionEstablishedUe));
	  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
	                  MakeCallback (&NotifyHandoverStartEnb));
	  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
	                   MakeCallback (&NotifyHandoverStartUe));

	  //A.M
	//  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
	//                   MakeCallback (&NotifyConnectionTimeout));
	  //A.M
	 // Config::Connect ("/NodeList/*/DeviceList/*/EpcX2/SendStatusRequest",
	 //                  MakeCallback (&NotifySendResourceStatusRequest));


	  Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
	                   MakeCallback (&NotifyHandoverEndOkEnb));
	  Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
	                  MakeCallback (&NotifyHandoverEndOkUe));

	///////////////////////////////////////////////
	// DEMO LOOP
	///////////////////////////////////////////////
	if(path != "")
	{
		int rounds = 0;
		while(rounds < 1)
		{
			//Simulator::Stop (Minutes (5.0));
			Simulator::Stop (Seconds (50.0));
/////////////////////////////
			//https://www.nsnam.org/docs/models/html/lte-user.html#sec-radio-environment-maps
/*
			Ptr<RadioEnvironmentMapHelper> remHelper = CreateObject<RadioEnvironmentMapHelper> ();
			remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/0"));
			remHelper->SetAttribute ("OutputFile", StringValue ("rem.out"));
			remHelper->SetAttribute ("XMin", DoubleValue (-400.0));
			remHelper->SetAttribute ("XMax", DoubleValue (400.0));
			remHelper->SetAttribute ("XRes", UintegerValue (100));
			remHelper->SetAttribute ("YMin", DoubleValue (-300.0));
			remHelper->SetAttribute ("YMax", DoubleValue (300.0));
			remHelper->SetAttribute ("YRes", UintegerValue (75));
			remHelper->SetAttribute ("Z", DoubleValue (0.0));
			remHelper->SetAttribute ("UseDataChannel", BooleanValue (true));
			remHelper->SetAttribute ("RbId", IntegerValue (10));
			remHelper->Install ();
*/

////////////////////////////

			Simulator::Run ();




			csvWriter.FlushLogs(path);
			std::cout << "Measurements are written to database.." << std::endl;
			rounds++;
		}
		std::cout << "Simulation Ended" << std::endl;
	}
	else
	{
		std::cout << "Simulation Ended With Error" << std::endl;
		std::cout << "Please set path value for output files with --pathOut argument" << std::endl;
	}
	lteHelper = 0;
	Simulator::Destroy ();
}

//MacStatsCalculator::DlScheduling
///"% time\tcellId\tIMSI\tframe\tsframe\tRNTI\tNumPRBs\tmcsTb1\tsizeTb1\tmcsTb2\tsizeTb2"
//PhyStatsCalculator::ReportCurrentCellRsrpSinr
//outFile << "% time\tcellId\tIMSI\tRNTI\trsrp\tsinr";

/* RadioBearerStatsCalculator::ShowResults
 * RadioBearerStatsCalculator::WriteUlResults
 * dlOutFile << "% start\tend\tCellId\tIMSI\tRNTI\tLCID\tnTxPDUs\tTxBytes\tnRxPDUs\tRxBytes\t";
 dlOutFile << "delay\tstdDev\tmin\tmax\t";
 dlOutFile << "PduSize\tstdDev\tmin\tmax";*/


