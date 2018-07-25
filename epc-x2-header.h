/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#ifndef EPC_X2_HEADER_H
#define EPC_X2_HEADER_H

#include "ns3/epc-x2-sap.h"
#include "ns3/header.h"

#include <vector>
#include <map>
#include <set>

namespace ns3 {


class EpcX2Header : public Header
{
public:
  EpcX2Header ();
  virtual ~EpcX2Header ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint8_t GetMessageType () const;
  void SetMessageType (uint8_t messageType);

  uint8_t GetProcedureCode () const;
  void SetProcedureCode (uint8_t procedureCode);

  void SetLengthOfIes (uint32_t lengthOfIes);
  void SetNumberOfIes (uint32_t numberOfIes);


  enum ProcedureCode_t {
    HandoverPreparation     = 0,
    LoadIndication          = 2,
    SnStatusTransfer        = 4,
    UeContextRelease        = 5,
	ResourceStatusReportingInitiation = 9,
    ResourceStatusReporting = 10,
	FailureDetectionInitiation = 11,
	ForwardingFailureDetectionInitiation = 12,
	TokenAckNACK = 13,

  };

  enum TypeOfMessage_t {
    InitiatingMessage       = 0,
    SuccessfulOutcome       = 1,
    UnsuccessfulOutcome     = 2
  };

private:
  uint8_t m_messageType;
  uint8_t m_procedureCode;

  uint32_t m_lengthOfIes;
  uint32_t m_numberOfIes;
};


class EpcX2HandoverRequestHeader : public Header
{
public:
  EpcX2HandoverRequestHeader ();
  virtual ~EpcX2HandoverRequestHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint16_t GetOldEnbUeX2apId () const;
  void SetOldEnbUeX2apId (uint16_t x2apId);

  uint16_t GetCause () const;
  void SetCause (uint16_t cause);

  uint16_t GetTargetCellId () const;
  void SetTargetCellId (uint16_t targetCellId);

  uint32_t GetMmeUeS1apId () const;
  void SetMmeUeS1apId (uint32_t mmeUeS1apId);

  std::vector <EpcX2Sap::ErabToBeSetupItem> GetBearers () const;
  void SetBearers (std::vector <EpcX2Sap::ErabToBeSetupItem> bearers);

  uint64_t GetUeAggregateMaxBitRateDownlink () const;
  void SetUeAggregateMaxBitRateDownlink (uint64_t bitRate);

  uint64_t GetUeAggregateMaxBitRateUplink () const;
  void SetUeAggregateMaxBitRateUplink (uint64_t bitRate);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_oldEnbUeX2apId;
  uint16_t          m_cause;
  uint16_t          m_targetCellId;
  uint32_t          m_mmeUeS1apId;
  uint64_t          m_ueAggregateMaxBitRateDownlink;
  uint64_t          m_ueAggregateMaxBitRateUplink;
  std::vector <EpcX2Sap::ErabToBeSetupItem> m_erabsToBeSetupList;
};


class EpcX2HandoverRequestAckHeader : public Header
{
public:
  EpcX2HandoverRequestAckHeader ();
  virtual ~EpcX2HandoverRequestAckHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint16_t GetOldEnbUeX2apId () const;
  void SetOldEnbUeX2apId (uint16_t x2apId);

  uint16_t GetNewEnbUeX2apId () const;
  void SetNewEnbUeX2apId (uint16_t x2apId);

  std::vector <EpcX2Sap::ErabAdmittedItem> GetAdmittedBearers () const;
  void SetAdmittedBearers (std::vector <EpcX2Sap::ErabAdmittedItem> bearers);

  std::vector <EpcX2Sap::ErabNotAdmittedItem> GetNotAdmittedBearers () const;
  void SetNotAdmittedBearers (std::vector <EpcX2Sap::ErabNotAdmittedItem> bearers);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_oldEnbUeX2apId;
  uint16_t          m_newEnbUeX2apId;
  std::vector <EpcX2Sap::ErabAdmittedItem>     m_erabsAdmittedList;
  std::vector <EpcX2Sap::ErabNotAdmittedItem>  m_erabsNotAdmittedList;
};


class EpcX2HandoverPreparationFailureHeader : public Header
{
public:
  EpcX2HandoverPreparationFailureHeader ();
  virtual ~EpcX2HandoverPreparationFailureHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint16_t GetOldEnbUeX2apId () const;
  void SetOldEnbUeX2apId (uint16_t x2apId);

  uint16_t GetCause () const;
  void SetCause (uint16_t cause);

  uint16_t GetCriticalityDiagnostics () const;
  void SetCriticalityDiagnostics (uint16_t criticalityDiagnostics);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_oldEnbUeX2apId;
  uint16_t          m_cause;
  uint16_t          m_criticalityDiagnostics;
};


class EpcX2SnStatusTransferHeader : public Header
{
public:
  EpcX2SnStatusTransferHeader ();
  virtual ~EpcX2SnStatusTransferHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint16_t GetOldEnbUeX2apId () const;
  void SetOldEnbUeX2apId (uint16_t x2apId);

  uint16_t GetNewEnbUeX2apId () const;
  void SetNewEnbUeX2apId (uint16_t x2apId);

  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> GetErabsSubjectToStatusTransferList () const;
  void SetErabsSubjectToStatusTransferList (std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> erabs);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_oldEnbUeX2apId;
  uint16_t          m_newEnbUeX2apId;
  std::vector <EpcX2Sap::ErabsSubjectToStatusTransferItem> m_erabsSubjectToStatusTransferList;
};


class EpcX2UeContextReleaseHeader : public Header
{
public:
  EpcX2UeContextReleaseHeader ();
  virtual ~EpcX2UeContextReleaseHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  uint16_t GetOldEnbUeX2apId () const;
  void SetOldEnbUeX2apId (uint16_t x2apId);

  uint16_t GetNewEnbUeX2apId () const;
  void SetNewEnbUeX2apId (uint16_t x2apId);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  uint16_t          m_oldEnbUeX2apId;
  uint16_t          m_newEnbUeX2apId;
};


class EpcX2LoadInformationHeader : public Header
{
public:
  EpcX2LoadInformationHeader ();
  virtual ~EpcX2LoadInformationHeader ();

  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
  virtual void Print (std::ostream &os) const;


  std::vector <EpcX2Sap::CellInformationItem> GetCellInformationList () const;
  void SetCellInformationList (std::vector <EpcX2Sap::CellInformationItem> cellInformationList);

  uint32_t GetLengthOfIes () const;
  uint32_t GetNumberOfIes () const;

private:
  uint32_t          m_numberOfIes;
  uint32_t          m_headerLength;

  std::vector <EpcX2Sap::CellInformationItem> m_cellInformationList;
};




class EpcX2FailureDetectionTokenHeader :public Header
{
public:
	EpcX2FailureDetectionTokenHeader ();
	virtual ~EpcX2FailureDetectionTokenHeader ();

	static TypeId GetTypeId (void);
	virtual TypeId GetInstanceTypeId (void) const;
	virtual uint32_t GetSerializedSize (void) const;
	virtual void Serialize (Buffer::Iterator start) const;
	virtual uint32_t Deserialize (Buffer::Iterator start);
	virtual void Print (std::ostream &os) const;

	uint16_t GetSourceId() const;
	void SetSourceId (uint16_t sourceId);

	std::vector<uint16_t> GetTargetIds() const;
	void SetTargetIds (std::vector<uint16_t> targetIds);

	std::map<uint16_t,uint16_t> GetForwardingList() const;
	void SetForwardingList (std::map<uint16_t,uint16_t> forwardingList);

	uint8_t GetForwarded() const;
	void SetForwarded (uint8_t forwarded);

	uint16_t GetForwardingId() const;
	void SetForwardingId (uint16_t forwardingId);

	bool checkTargetExists(uint16_t targetId);


	uint32_t GetLengthOfIes () ;
	uint32_t GetNumberOfIes () const;

private:
	 uint32_t          				m_numberOfIes;
	 uint32_t         				m_headerLength;

	 uint16_t 						m_sourceId; 		// original Token owner
	 std::vector <uint16_t> 		m_targetIds;
	 std::map <uint16_t, uint16_t> 	m_forwardingList;
	 uint8_t 						m_forwarded;    	// initialized as false = 0
	 uint16_t 						m_forwardingId; 	// initialized to 9999 as null

};

class EpcX2TokenAckNACKHeader :public Header
{
public:
	EpcX2TokenAckNACKHeader ();
	virtual ~EpcX2TokenAckNACKHeader ();

	static 	TypeId 		GetTypeId (void);
	virtual TypeId 		GetInstanceTypeId (void) const;
	virtual uint32_t 	GetSerializedSize (void) const;
	virtual void 		Serialize (Buffer::Iterator start) const;
	virtual uint32_t 	Deserialize (Buffer::Iterator start);
	virtual void 		Print (std::ostream &os) const;

	uint16_t GetforwardingId () const;
	void SetforwardingId (uint16_t forwardingId);

	uint16_t GetSourceId () const;
	void SetSourceId (uint16_t sourceId);

	uint16_t GetTargetId () const;
	void SetTargetId (uint16_t targetId);

	void SetACKNACK (uint8_t success);
	uint8_t GetACKNACK ();

	void SetNoX2ToForward (uint8_t noX2);
	uint8_t GetNoX2ToForward ();

	//void SetSingleNode (uint8_t noX2);
	//uint8_t GetSingleNode ();

	uint32_t GetLengthOfIes () const;
	uint32_t GetNumberOfIes () const;

private:
	  uint32_t  m_numberOfIes;
	  uint32_t  m_headerLength;

	  uint16_t 	m_forwardingId;		 // Id of the forwarding cell from which I received the forwarded Token
	  uint16_t 	m_sourceId; 		 // The ACK/NACK owner Id
	  uint16_t 	m_tokenOwnerId; 	 // original Token owner
	  uint8_t 	m_successStatus; 	 // True = 1 = ACK , False = NACK
	  uint8_t	m_noX2ToForward; 	 // True = 1,  if sent back to tokenOwnerId without forwarding (no X2 to forward)
	  //uint8_t	m_singleNode; 		 // this is true = 1 only of my list of forwarding is zero (no node to forward for so send ACK back directly)
};


} // namespace ns3

#endif // EPC_X2_HEADER_H
