/* 
NMEA0183Handlers.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#include "N2kMsg.h"
#include "NMEA2000.h"
#include "N2kMessages.h"
#include "NMEA0183Messages.h"
#include "NMEA0183Handlers.h"
#include "AIS.h"

#define pi 3.1415926535897932384626433832795
#define kmhToms 1000.0 / 3600.0
#define knToms 1852.0 / 3600.0
#define degToRad pi / 180.0
#define radToDeg 180.0 / pi
#define msTokmh 3600.0 / 1000.0
#define msTokn 3600.0 / 1852.0
#define nmTom 1.852 * 1000
#define mToFathoms 0.546806649
#define mToFeet 3.2808398950131

extern tNMEA0183 NMEA0183;
extern tNMEA0183 NMEA0183_1;
extern tNMEA0183 NMEA0183_3;
extern Stream* NMEA0183HandlersDebugStream;
extern unsigned long NMEA0183TxCounter;
extern unsigned long NMEA0183RxCounter;
extern unsigned long N2kTxCounter;
extern unsigned long N2kRxCounter;

struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg); 
  const char *NMEA0183seq;
};

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg);
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleVDM(const tNMEA0183Msg &NMEA0183Msg);
void HandleXTE(const tNMEA0183Msg &NMEA0183Msg);
void HandleZTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleXXX(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
tNMEA2000 *pNMEA2000=0;
tBoatData *pBD=0;

tNMEA0183Handler NMEA0183Handlers[]={
	{ "APA", &HandleXXX, "Autopilot Sentence - A" },
	{ "APB", &HandleXXX, "Autopilot Sentence - B" },
	{ "BOD", &HandleXXX, "Bearing Origin to Destination" },
	{ "BWC", &HandleXXX, "Bearing and Distance to Waypoint" },
	{ "GGA", &HandleGGA, "Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver" },
	{ "HDT", &HandleHDT, "Heading - True" },
	{ "MWV", &HandleXXX, "Wind Speed and Angle" },
	{ "RMB", &HandleRMB, "Recommended Minimum Navigation Information - B" },
	{ "RMC", &HandleRMC, "Recommended Minimum Navigation Information - C" },
	{ "VDM", &HandleVDM, "AIS VHF Datalink Message" },
//	{ "VDO", &HandleVDM, "AIS VHF Datalink Own-vessel Message"},
	{ "VTG", &HandleVTG, "Track made good and Ground speed" },
	{ "WPL", &HandleXXX, "Waypoint Location" },
	{ "XTE", &HandleXTE, "Cross-Track Error" },
	{ "ZTG", &HandleZTG, "UTC & Time to Destination Waypoint" },

	{0,0}
};

/*****************************************************************************
APA - Autopilot Sentence "A"
       1 2  3   4 5 6 7  8  9 10    11
       | |  |   | | | |  |  | |     |
$--APA,A,A,x.xx,L,N,A,A,xxx,M,c-- - c*hh<CR><LF>
Field Number :
1) Status
	V = LORAN - C Blink or SNR warning
	V = general warning flag or other navigation systems when a reliable fix is not available
2) Status
	V = Loran - C Cycle Lock warning flag
	A = OK or not used
3) Cross Track Error Magnitude
4) Direction to steer, L or R
5) Cross Track Units(Nautic miles or kilometers)
6) Status
	A = Arrival Circle Entered
7) Status
	A = Perpendicular passed at waypoint
8) Bearing origin to destination
9) M = Magnetic, T = True
10) Destination Waypoint ID
11) checksum

*/

/*****************************************************************************
APB - Autopilot Sentence "B"
                                        13    15
	   1 2 3   4 5 6 7 8   9 10   11  12|   14|
	   | | |   | | | | |   | |    |   | |   | |
$--APB,A,A,x.x,a,N,A,A,x.x,a,c--c,x.x,a,x.x,a*hh<CR><LF>
Field Number :
1) Status
	V = LORAN - C Blink or SNR warning
	V = general warning flag or other navigation systems when a reliable fix is not available
2) Status
	V = Loran - C Cycle Lock warning flag
	A = OK or not used
3) Cross Track Error Magnitude
4) Direction to steer, L or R
5) Cross Track Units, N = Nautical Miles
6) Status
	A = Arrival Circle Entered
7) Status
	A = Perpendicular passed at waypoint
8) Bearing origin to destination
9) M = Magnetic, T = True
10) Destination Waypoint ID
11) Bearing, present position to Destination
12) M = Magnetic, T = True
13) Heading to steer to destination waypoint
14) M = Magnetic, T = True
15) Checksum

*/

/*****************************************************************************
BOD - Bearing Origin to Destination
	   1   2 3   4 5    6    7
	   |   | |   | |    |    |
$--BOD,x.x,T,x.x,M,c--c,c--c*hh<CR><LF>
Field Number :
1) Bearing Degrees, TRUE
2) T = True
3) Bearing Degrees, Magnetic
4) M = Magnetic
5) TO Waypoint
6) FROM Waypoint
7) Checksum

*/

/*****************************************************************************
BWC - Bearing and Distance to Waypoint, Latitude, N / S, Longitude, E / W, UTC, Status
													  11
	   1         2       3 4        5 6   7 8   9 10  | 12   13
	   |         |       | |        | |   | |   | |   | |    |
$--BWC,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x.x,T,x.x,M,x.x,N,c--c*hh<CR><LF>
Field Number :
1) UTCTime
2) Waypoint Latitude
3) N = North, S = South
4) Waypoint Longitude
5) E = East, W = West
6) Bearing, True
7) T = True
8) Bearing, Magnetic
9) M = Magnetic
10) Nautical Miles
11) N = Nautical Miles
12) Waypoint ID
13) Checksum

*/

/*****************************************************************************
MWV - Wind Speed and Angle
	   1   2 3   4 5
	   |   | |   | |
$--MWV,x.x,a,x.x,a*hh<CR><LF>
Field Number :
1) Wind Angle, 0 to 360 degrees
2) Reference, R = Relative, T = True
3) Wind Speed
4) Wind Speed Units, K / M / N
5) Status, A = Data Valid
6) Checksum

*/

/*****************************************************************************
WPL - Waypoint Location
	   1       2 3        4 5    6
	   |       | |        | |    |
$--WPL,llll.ll,a,yyyyy.yy,a,c--c*hh<CR><LF>
Field Number :
1) Latitude
2) N or S(North or South)
3) Longitude
4) E or W(East or West)
5) Waypoint name
6) Checksum

*/

/*
Conversions from NMEA 0183 to NMEA 2000
NMEA 0183	NMEA 2000 PGN								Comment
---------------------------------------------------------------------------------------------------------
APA
APB			127237 Heading/Track Control
			129283 Cross Track Error
			129284 Navigation Data
BOD
BWC
DIN			127488 Engine Parameters, Rapid Update		According SeaSmart.Net protocol
			127489 Engine Parameters, Dynamic			specification v1.6.0
			127493 Transmission Parameters, Dynamic
			127505 Fluid Level
			127508 Battery Status
DPT			128267 Water Depth
DTM			129044 Datum
*GGA		129029 GNSS Position Data					ZDA or RMC are required
			126992 System Time
			129025 Position Rapid update
			129033 Local Time Offset
			129539 GNSS DOPs

GLL			129025 Position, Rapid Update				See note (7)
HDG			127250 Vessel Heading
HDM, *HDT	127250 Vessel Heading						Use variation and deviation from HDG
MDA			130311 Environmental Parameters				Relative air humidity, air and water
			130314 Actual Pressure						temperature, atmospheric pressure,
			130306 Wind Data							wind data
MOB			127233 Man Overboard Notification (MOB)
MTW			130311 Environmental Parameters
MWD			130306 Wind Data
MWV			130306 Wind Data							Theoretical wind sent as ground referenced to True North; calculated using COG/SOG
*RMB		129283 Cross Track Error					Use data from APB
			129284 Navigation Data						Sent with true bearings, use ETA from ZTG
			129285 Navigation � Route/WP information
*RMC		126992 System Time							See note (7)
			127250 Vessel Heading
			127258 Magnetic Variation
			129025 Position, Rapid Update
			129026 COG & SOG, Rapid Update
			129029 GNSS Position Data
			129033 Local Time Offset
RSA			127245 Rudder
RTE			130066 Route and WP Service �				Use data from WPL
				   Route/WP-List Attributes
			130067 Route and WP Service �
				   Route - WP Name & Position
ROT			127251 Rate of Turn
VHW			127250 Vessel Heading
			128259 Speed, Water referenced
*VDM, *VDO	129038 AIS Class A Position Report			AIS VHF messages 1, 2 and 3
			129039 AIS Class B Position Report			AIS VHF message 18
			129040 AIS Class B Extended Position Report	AIS VHF message 19
			129041 AIS Aids to Navigation (AtoN) Report	AIS VHF message 21
			129793 AIS UTC and Date Report				AIS VHF messages 4 and 11
			129794 AIS Class A Static+Voyage Rel Data	AIS VHF message 5
			129798 AIS SAR Aircraft Position Report		AIS VHF message 9
			129041 AIS Aids to Navigation (AtoN) Report	AIS VHF message 21
			129809 AIS Class B "CS" Static Data, Part A	AIS VHF message 24
			129810 AIS Class B "CS" Static Data, Part B	AIS VHF message 24
VDR			129291 Set & Drift, Rapid Update
VLW			128275 Distance Log
*VTG		129026 COG & SOG, Rapid Update
VWR			130306 Wind Data
WPL			130074 Route and WP Service �				Only waypoints not included to the route
				   WP List � WP Name & Position			(the RTE should be received during	3 seconds after WPL)
*XTE		129283 Cross Track Error
ZDA			126992 System Time
			129029 GNSS Position Data
			129033 Local Time Offset
*ZTG		129284 Navigation Data						use ETA from ZTG

Note (7): All NMEA 2000 periodic messages are sending with interval specified in the Standard.
		Except PGN 127488, 127489, 127493, 127505 and 127508, these messages are sending immediately on receiving of DIN sentence.
Note (8): Sentences with no significant data (or data marked as invalid) may not be translated to
		NMEA 2000 messages. NMEA 0183 sentences with invalid check sum are ignored.

* means NMEA 0183 currently suported

*/

// *****************************************************************************
void InitNMEA0183Handlers(tNMEA2000 *_NMEA2000, tBoatData *_BoatData) {
  pNMEA2000=_NMEA2000;
  pBD=_BoatData;
}

// *****************************************************************************
tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

double toMagnetic(double True, double Variation) {

	double magnetic = True - Variation;
	while (magnetic<0) magnetic += pi*2;
	while (magnetic >= pi*2) magnetic -= pi*2;
	return magnetic;
}

// *****************************************************************************
void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  int iHandler;
  
//  Serial.println("NMEA0183RxCounter+1");

  NMEA0183.SendMessage(NMEA0183Msg);       // Forward all received NMEA0183 messages to the NMEA0183 out stream (USB im Teensy)
//  NMEA0183_1.SendMessage(NMEA0183Msg);     // Forward all received NMEA0183 messages to the NMEA0183 out1 stream (USB an TX1/RX1)
//  NMEA0183_3.SendMessage(NMEA0183Msg);     // Forward all received NMEA0183 messages to the NMEA0183 out3 stream (RS485 an TX3/RX3)
  NMEA0183TxCounter = NMEA0183TxCounter+1;

  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  
  if (NMEA0183Handlers[iHandler].Code!=0) {
	  if (NMEA0183HandlersDebugStream != 0) {
		  NMEA0183HandlersDebugStream->print("NMEA0183 message parsed: ");
		  NMEA0183HandlersDebugStream->print(NMEA0183Handlers[iHandler].Code);
		  NMEA0183HandlersDebugStream->print(" - ");
		  NMEA0183HandlersDebugStream->println(NMEA0183Handlers[iHandler].NMEA0183seq);
	  }

/*
	  Serial.print("NMEA0183 message parsed: ");
	  Serial.print(NMEA0183Handlers[iHandler].Code);
	  Serial.print(" - ");
	  Serial.println(NMEA0183Handlers[iHandler].NMEA0183seq);
*/
	  NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
  }
  else {
	  if (NMEA0183HandlersDebugStream != 0) {
		  NMEA0183HandlersDebugStream->println("NMEA0183 message parsed, no Handler found.");
	  }
/*
	  Serial.print(  "NMEA0183 message parsed, no Handler found: ");
	  Serial.println(NMEA0183Msg.MessageCode());
	  Serial.println("**********************************************");
*/
  }
}

// NMEA0183 message Handler functions

void HandleXXX(const tNMEA0183Msg &NMEA0183Msg) {
	if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("no NMEA0183Handler available"); }
}

/*****************************************************************************
RMB - Recommended Minimum Navigation Information
															14
	   1 2   3 4    5    6       7 8        9 10  11  12  13|
	   | |   | |    |    |       | |        | |   |   |   | |
$--RMB,A,x.x,a,c--c,c--c,llll.ll,a,yyyyy.yy,a,x.x,x.x,x.x,A*hh<CR><LF>
Field Number :
1) Status, V = Navigation receiver warning
2) Cross Track error - nautical miles
3) Direction to Steer, Left or Right
4) FROM Waypoint ID
5) TO Waypoint ID
6) Destination Waypoint Latitude
7) N or S
8) Destination Waypoint Longitude
9) E or W
10) Range to destination in nautical miles
11) Bearing to destination in degrees True
12) Destination closing velocity in knots
13) Arrival Status, A = Arrival Circle Entered
14) Checksum

*/
void HandleRMB(const tNMEA0183Msg &NMEA0183Msg) {
	double XTE;
	double Latitude;
	double Longitude;
	double DTW;
	double BTW;
	double VMG;
	char arrivalAlarm;
	char originID[NMEA0183_MAX_WP_NAME_LENGTH];
	char destID[NMEA0183_MAX_WP_NAME_LENGTH];
//	int originIDi;
//	int destIDi;

//	if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->print("NMEA0183Msg="); NMEA0183HandlersDebugStream->println(NMEA0183Msg->Data[MAX_NMEA0183_MSG_LEN]); }

	if (pBD == 0) return;

	if (NMEA0183ParseRMB_nc(NMEA0183Msg, XTE, Latitude, Longitude, DTW, BTW, VMG, arrivalAlarm, *originID, *destID)) {
/*
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->print("XTE="); NMEA0183HandlersDebugStream->println(XTE);
			NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(Latitude);
			NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(Longitude);
			NMEA0183HandlersDebugStream->print("DTW="); NMEA0183HandlersDebugStream->println(DTW);
			NMEA0183HandlersDebugStream->print("BTW="); NMEA0183HandlersDebugStream->println(BTW);
			NMEA0183HandlersDebugStream->print("VMG="); NMEA0183HandlersDebugStream->println(VMG);
			NMEA0183HandlersDebugStream->print("arrivalAlarm="); NMEA0183HandlersDebugStream->println(arrivalAlarm);
			NMEA0183HandlersDebugStream->print("originID="); NMEA0183HandlersDebugStream->println(originID);
			NMEA0183HandlersDebugStream->print("destID="); NMEA0183HandlersDebugStream->println(destID);
		}
*/
/*
		if (atoi(originID) == 0) {
			originIDi = int(originID[0]) * 100 + int(originID[1]);
		}
		else {
			originIDi = atoi(originID);
		}

//		Serial1.print("originID=");
//		Serial1.print(originID);
//		Serial1.print(", ");
//		Serial1.println(originIDi);

		if (atoi(destID) == 0) {
			destIDi = int(destID[0]) * 100 + int(destID[1]);
		}
		else {
			destIDi = atoi(destID);
		}

//		Serial1.print("destID=");
//		Serial1.print(destID);
//		Serial1.print(", ");
//		Serial1.println(destIDi);
*/

/*
RMB		129283 Cross Track Error
  		129284 Navigation Data
This parameter group provides essential navigation data for following a route.  Transmissions will originate from products that can
create and manage routes using waypoints. This information is intended for navigational repeaters.
Field # Field Description
1 Sequence ID
2 Distance to Destination Waypoint
3 Course/Bearing Ref.
4 Perpendicular Crossed
5 Arrival Circle Entered
6 Calculation Type
7 ETA Time
8 ETA Date
9 Bearing, Origin To Destination Waypoint
10 Bearing, Position To Destination Waypoint
11 Origin Waypoint Number
12 Destination Waypoint Number
13 Destination Wpt Latitude
14 Destination Wpt Longitude
15 Waypoint Closing Velocity
*/
		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
			SetN2kXTE(N2kMsg, 1, N2kxtem_Autonomous, false, nmTom*XTE);       // PGN 129283
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;

			// Navigation data
			//void SetN2kPGN129284(tN2kMsg &N2kMsg, unsigned char SID, double DistanceToWaypoint, tN2kHeadingReference BearingReference,
			//	bool PerpendicularCrossed, bool ArrivalCircleEntered, tN2kDistanceCalculationType CalculationType,
			//	double ETATime, int16_t ETADate, double BearingOriginToDestinationWaypoint, double BearingPositionToDestinationWaypoint,
			//	uint8_t OriginWaypointNumber, uint8_t DestinationWaypointNumber,
			//	double DestinationLatitude, double DestinationLongitude, double WaypointClosingVelocity)
//				SetN2kPGN129284(N2kMsg, 1, DTW*nmTom, N2khr_true, false, false, N2kdct_GreatCircle, 0, 0,
//					BTW*degToRad, BTW*degToRad, 1, 2, Latitude, Longitude, 5* knToms);
//				pNMEA2000->SendMsg(N2kMsg);
//				N2kTxCounter = N2kTxCounter + 1;


			//3B 9F D1 47 4F 54 4F 20 43 55 52 53 4F 52 00 00 00 00 00 30 30 30 31 8A 72 0F 6F 54 B6 09 00 FF
			//3B9F2B474F544F20435552534F52000000000030303031 CA594F 56 2447 0800FF
			//3B9FF0474F544F20435552534F52203200383730303032 CF6F54 6C 345B 0700FF
			//3B9FB5474F544F20435552534F52203300353130303033 7954FE 50 E004 0700FF

			N2kMsg.SetPGN(130848UL);    // Raymarine proprietary PGN 130848
			N2kMsg.Priority = 7;
			N2kMsg.Destination = 255;
			N2kMsg.AddByte(0x3b);  // lower 8 Bit of Manufacturer Code, Raymarine=1851=0x073b
			N2kMsg.AddByte(0x9f);  // high 3 Bit = Industry Code, Marine Industry=4, 2 Bit Reserved, higher 3 Bit of Manufacturer Code"
			N2kMsg.AddByte(0xd1);  // Proprietary ID ??? ändert sich, zählt hoch, muss das ???

			N2kMsg.AddStr0(destID, 16);

			N2kMsg.AddStr("0001", 4);    // oder WP-Number ??

//			Serial.print("DTW="); Serial.println(DTW);
//			Serial.print("BTW="); Serial.println(BTW);

			N2kMsg.Add2ByteUInt(BTW*174.5);  // BTW for MFD Axiom Pro 9 (Waypoint) and for i70, Bearing Mode in Unit setting must be set to TRUE
			N2kMsg.Add2ByteUInt(BTW*174.5);  // 1:0.0057° => 1/0.0057=174

			N2kMsg.Add4ByteUDouble(DTW*nmTom, 0.01);   // DTW in m

//			N2kMsg.AddByte(true);
			N2kMsg.AddByte(false);      // autostep to next waypoint ???

			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;

/*
			//3B 9F 01 00 47 4F 54 4F 20 43 55 52 53 4F 52 00 00 00 00 00 FF 00 FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 0D FF FF FF FF FF FF FF FF
			//3B9F0100474F544F20435552534F520000000000FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF
			//3B9F0200474F544F20435552534F522032003837FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF
			//3B9F0300474F544F20435552534F522033003531FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF

			N2kMsg.SetPGN(130918UL);    // Raymarine proprietary PGN 130918
			N2kMsg.Priority = 7;
			N2kMsg.Destination = 255;
			N2kMsg.AddByte(0x3b);  // lower 8 Bit of Manufacturer Code, Raymarine=1851=0x073b
			N2kMsg.AddByte(0x9f);  // high 3 Bit = Industry Code, Marine Industry=4, 2 Bit Reserved, higher 3 Bit of Manufacturer Code"
			N2kMsg.AddByte(0x01);  // Proprietary ID ???
			N2kMsg.AddByte(0x00);

//			N2kMsg.AddStr("GOTO CURSOR", 16);
			N2kMsg.AddStr(destID, 16);

			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0x00);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0x0d);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);
			N2kMsg.AddByte(0xff);

			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
*/
		}

	    if (NMEA0183HandlersDebugStream != 0) {
	}
  }
  else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse RMB"); }
}

/*****************************************************************************
RMC - Recommended Minimum Navigation Information
		1         2 3       4 5        6 7   8   9    10 11 12
		|         | |       | |        | |   |   |    |   | |
 $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh<CR><LF>
 Field Number:
  1) UTC Time
  2) Status, V = Navigation receiver warning, P = Precise
  3) Latitude
  4) N or S
  5) Longitude
  6) E or W
  7) Speed over ground, knots
  8) Track made good, degrees true
  9) Date, ddmmyy
 10) Magnetic Variation, degrees
 11) E or W
 12) Checksum

*/

void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
	if (pBD==0) return;
  
	if (NMEA0183ParseRMC_nc(NMEA0183Msg, pBD->GPSTime, pBD->Latitude, pBD->Longitude, pBD->COG, pBD->SOG, pBD->DaysSince1970, pBD->Variation)) {
/*
RMC		126992 System Time
		127258 Magnetic Variation
		129025 Position, Rapid Update
		129026 COG & SOG, Rapid Update   fehlt
*/
		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;

			SetN2kMagneticVariation(N2kMsg, 1, N2kmagvar_Calc, pBD->DaysSince1970, pBD->Variation);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
			SetN2kPGN129025(N2kMsg, pBD->Latitude, pBD->Longitude);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
			SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, pBD->COG, pBD->SOG);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
		}

		if (NMEA0183HandlersDebugStream != 0) {
		}
  }
  else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
}

/*****************************************************************************
GGA - Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver.
													  11
		1         2       3 4        5 6 7  8   9  10 |  12 13  14   15
		|         |       | |        | | |  |   |   | |   | |   |    |
 $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
 Field Number:
  1) Universal Time Coordinated (UTC)
  2) Latitude
  3) N or S (North or South)
  4) Longitude
  5) E or W (East or West)
  6) GPS Quality Indicator,
	 0 - fix not available,
	 1 - GPS fix,
	 2 - Differential GPS fix
  7) Number of satellites in view, 00 - 12
  8) Horizontal Dilution of precision
  9) Antenna Altitude above/below mean-sea-level (geoid)
 10) Units of antenna altitude, meters
 11) Geoidal separation, the difference between the WGS-84 earth
	 ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level
	 below ellipsoid
 12) Units of geoidal separation, meters
 13) Age of differential GPS data, time in seconds since last SC104
	 type 1 or 9 update, null field when DGPS is not used
 14) Differential reference station ID, 0000-1023
 15) Checksum

*/

void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
	if (pBD==0) return;
  
	if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,pBD->SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {
/*
GGA		129029 GNSS Position Data
*/
		if (pNMEA2000!=0) {
			tN2kMsg N2kMsg;

			SetN2kMagneticVariation(N2kMsg,1,N2kmagvar_Calc,pBD->DaysSince1970,pBD->Variation);
			pNMEA2000->SendMsg(N2kMsg); 
			N2kTxCounter = N2kTxCounter + 1;
			SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
			        N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
				    pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge);
			pNMEA2000->SendMsg(N2kMsg); 
			N2kTxCounter = N2kTxCounter + 1;
		}
/*
		if (NMEA0183HandlersDebugStream!=0) {
			NMEA0183HandlersDebugStream->print("Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
			NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
			NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
			NMEA0183HandlersDebugStream->print("Altitude="); NMEA0183HandlersDebugStream->println(pBD->Altitude,1);
			NMEA0183HandlersDebugStream->print("GPSQualityIndicator="); NMEA0183HandlersDebugStream->println(pBD->GPSQualityIndicator);
			NMEA0183HandlersDebugStream->print("SatelliteCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
			NMEA0183HandlersDebugStream->print("HDOP="); NMEA0183HandlersDebugStream->println(pBD->HDOP);
			NMEA0183HandlersDebugStream->print("GeoidalSeparation="); NMEA0183HandlersDebugStream->println(pBD->GeoidalSeparation);
			NMEA0183HandlersDebugStream->print("DGPSAge="); NMEA0183HandlersDebugStream->println(pBD->DGPSAge);
			NMEA0183HandlersDebugStream->print("DGPSReferenceStationID="); NMEA0183HandlersDebugStream->println(pBD->DGPSReferenceStationID);
		}
*/
	} else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
}

#define PI_2 6.283185307179586476925286766559

/*****************************************************************************
HDT - Heading - True
		1   2 3
		|   | |
 $--HDT,x.x,T*hh<CR><LF>
 Field Number:
  1) Heading Degrees, true
  2) T = True
  3) Checksum

*/
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {
	if (pBD==0) return;
  
	if (NMEA0183ParseHDT_nc(NMEA0183Msg,pBD->TrueHeading)) {
/*
HDT		127250 Vessel Heading
*/
		if (pNMEA2000!=0) {
			tN2kMsg N2kMsg;
			double MHeading=pBD->TrueHeading-pBD->Variation;
			while (MHeading<0) MHeading+=PI_2;
			while (MHeading>=PI_2) MHeading-=PI_2;
			SetN2kTrueHeading(N2kMsg,1,pBD->TrueHeading);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
		}
		if (NMEA0183HandlersDebugStream!=0) {
			NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
		}
	} else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse HDT"); }
}

/*****************************************************************************
VTG - Track made good and Ground speed
		1   2 3   4 5  6 7   8 9
		|   | |   | |  | |   | |
 $--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
 Field Number:
  1) Track Degrees
  2) T = True
  3) Track Degrees
  4) M = Magnetic
  5) Speed Knots
  6) N = Knots
  7) Speed Kilometers Per Hour
  8) K = Kilometers Per Hour
  9) Checksum

*/
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
	double MagneticCOG;
	if (pBD==0) return;
  
	if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
		pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading
/*
VTG		129026 COG & SOG, Rapid Update
*/
		if (pNMEA2000!=0) { 
			tN2kMsg N2kMsg;

			SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;

			SetN2kBoatSpeed(N2kMsg,1,pBD->SOG);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
		}
		if (NMEA0183HandlersDebugStream!=0) {
			NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
		}
	} else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
}

/*****************************************************************************
XTE - Cross - Track Error, Measured
	   1 2 3   4 5  6
	   | | |   | |  |
$--XTE,A,A,x.x,a,N,*hh<CR><LF>
Field Number :
1) Status
	V = LORAN - C Blink or SNR warning
	V = general warning flag or other navigation systems when a reliable fix is not available
2) Status
	V = Loran - C Cycle Lock warning flag
	A = OK or not used
3) Cross Track Error Magnitude
4) Direction to steer, L or R
5) Cross Track Units, N = Nautical Miles
6) Checksum

*/

void HandleXTE(const tNMEA0183Msg &NMEA0183Msg) {
	double XTE;
	if (pBD == 0) return;

	if (NMEA0183ParseXTE_nc(NMEA0183Msg, XTE)) {

//XTE		129283 Cross Track Error

		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;

			SetN2kXTE(N2kMsg, 1, N2kxtem_Autonomous, false, nmTom*XTE);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
		}
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->print("XTE="); NMEA0183HandlersDebugStream->println(XTE);
		}
	}
	else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse XTE"); }
}

/*****************************************************************************
ZTG - UTC & Time to Destination Waypoint
		1         2         3    4
		|         |         |    |
 $--ZTG,hhmmss.ss,hhmmss.ss,c--c*hh<CR><LF>
 Field Number:
  1) Universal Time Coordinated (UTC)
  2) Time Remaining
  3) Destination Waypoint ID
  4) Checksum

*/
void HandleZTG(const tNMEA0183Msg &NMEA0183Msg) {
/*
	double MagneticCOG;
	if (pBD == 0) return;

	if (NMEA0183ParseZTG_nc(NMEA0183Msg, pBD->COG, MagneticCOG, pBD->SOG)) {
		pBD->Variation = pBD->COG - MagneticCOG; // Save variation for Magnetic heading
 		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
//			SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, pBD->COG, pBD->SOG);
//			pNMEA2000->SendMsg(N2kMsg);
//			SetN2kBoatSpeed(N2kMsg, 1, pBD->SOG);
//			pNMEA2000->SendMsg(N2kMsg);
		}
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
		}
	}
	else if (NMEA0183HandlersDebugStream != 0) { NMEA0183HandlersDebugStream->println("Failed to parse ZTG"); }
*/
}

void printDegrees(long min4)
{
	long intPart = min4 / 60L;
	long fracPart = intPart % 10000L;
	if (fracPart < 0)
		fracPart = -fracPart;
	char frac[6];
	sprintf(frac, "%04ld", fracPart);
	NMEA0183HandlersDebugStream->print(intPart / 10000L); NMEA0183HandlersDebugStream->print("."); NMEA0183HandlersDebugStream->println(frac);
}

double MinToDegrees(long min4)
{
	double intPart;
	intPart = min4 / 60;
	intPart = intPart / 10000;
	return (intPart);
}

/*****************************************************************************
!AIVDM/VDO - AIVDM packets are reports from other ships and AIVDO packets are reports from your own ship
		1 2 3 4 5    6 7
		| | | | |    | |
 !AIVDM,n,n,n,c,c--c,n*hh<CR><LF>
 Field Number:
  1) count of fragments in the currently accumulating message. 
     The payload size of each sentence is limited by NMEA 0183�s 82-character maximum, 
	 so it is sometimes required to split a payload over several fragment sentences.
  2) fragment number of this sentence. It will be one-based. 
     A sentence with a fragment count of 1 and a fragment number of 1 is complete in itself.
  3) sequential message ID for multi-sentence messages
  4) radio channel code. AIS uses the high side of the duplex from two VHF radio channels:
     AIS Channel A is 161.975Mhz (87B); AIS Channel B is 162.025Mhz (88B). In the wild, channel codes 1 and 2 may also be encountered;
	 the standards do not prescribe an interpretation of these but it�s obvious enough.
  5) data payload
  6) number of fill bits requires to pad the data payload to a 6 bit boundary, ranging from 0 to 5.
     Equivalently, subtracting 5 from this tells how many least significant bits of the last 6-bit nibble in the data payload should be ignored.
	 Note that this pad byte has a tricky interaction with the <[ITU-1371]> requirement for byte alignment in over-the-air AIS messages.
  7) Checksum

*/
/* samples of alle AIS-msg
!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24
!AIVDM,1,1,,A,400TcdiuiT7VDR>3nIfr6>i00000,0*78
!AIVDM,2,1,0,A,58wt8Ui`g??r21`7S=:22058<v05Htp000000015>8OA;0sk,0*7B
!AIVDM,2,2,0,A,eQ8823mDm3kP00000000000,2*5D
!AIVDM,1,1,4,B,6>jR0600V:C0>da4P106P00,2*02
!AIVDM,2,1,9,B,61c2;qLPH1m@wsm6ARhp<ji6ATHd<C8f=Bhk>34k;S8i=3To,0*2C
!AIVDM,2,2,9,B,Djhi=3Di<2pp=34k>4D,2*03
!AIVDM,1,1,1,B,8>h8nkP0Glr=<hFI0D6??wvlFR06EuOwgwl?wnSwe7wvlOw?sAwwnSGmwvh0,0*17
!AIVDM,1,1,,A,95M2oQ@41Tr4L4H@eRvQ;2h20000,0*0F
!AIVDM,1,1,,B,;8u:8CAuiT7Bm2CIM=fsDJ100000,0*51
!AIVDM,1,1,,B,>>M4fWA<59B1@E=@,0*17
!AIVDM,1,1,,A,B6CdCm0t3`tba35f@V9faHi7kP06,0*58
!AIVDM,2,1,0,B,C8u:8C@t7@TnGCKfm6Po`e6N`:Va0L2J;06HV50JV?SjBPL3,0*28
!AIVDM,2,2,0,B,11RP,0*17
!AIVDM,2,1,5,B,E1c2;q@b44ah4ah0h:2ab@70VRpU<Bgpm4:gP50HH`Th`QF5,0*79
!AIVDM,2,2,5,B,1CQ1A83PCAH0,0*62
!AIVDM,1,1,,B,H1c2;qA@PU>0U>060<h5=>0:1Dp,2*7F
!AIVDM,1,1,,B,H1c2;qDTijklmno31<<C970`43<1,0*2A
!AIVDM,1,1,,A,KCQ9r=hrFUnH7P00,0*41
!AIVDM,1,1,,B,KC5E2b@U19PFdLbMuc5=ROv62<7m,0*16
!AIVDM,1,1,,B,K5DfMB9FLsM?P00d,0*70
*/
void HandleVDM(const tNMEA0183Msg &NMEA0183Msg) {
	uint8_t pkgCnt;
	uint8_t pkgNmb;
	unsigned int seqMessageId;
	char channel; 
	unsigned int length;
	char bitstream[512] = "";
    static char bitstream_total[512] = "";
	unsigned int fillBits;

	enum AIS::Nmea0183AisMessages msgType;
	unsigned int msgNumeric;
	uint8_t part_no;
	uint8_t repeat;
	uint32_t mmsi;
//	uint32_t mothership_mmsi;
	int32_t LAT;
	int32_t LONG;
	double LAT1;
	double LONG1;
	bool accuracy;
	bool raim;
	uint8_t seconds;
	int32_t COG;
	int32_t SOG;
	int32_t HDG;
	int8_t ROT;
	double ROT1;
	uint32_t imo;
	uint8_t shiptype;
	uint8_t to_port;
	uint8_t to_starboard;
	uint16_t to_bow;
	uint16_t to_stern;
	uint8_t epfd;
	uint8_t draught;
	uint8_t navstatus;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t aidtype;
	char name[120];
	char *shipname = name;
	char dest[20];
	char *destination = dest;
	char sign[42];
	char *callsign = sign;
	char vendor[18];
	const char *vendorid = vendor;
	char name1[120];
	char *aidname = name1;
	char name2[88];
	char *aidnameext = name2;

	length = 500;

//	NMEA0183.SendMessage(NMEA0183Msg);       // Send VDM NMEA0183 message to the NMEA0183 out stream (USB im Teensy)

	if (NMEA0183ParseVDM_nc(NMEA0183Msg, pkgCnt, pkgNmb, seqMessageId, channel, length, bitstream, fillBits)) {

//#define AIS_DEBUG

#ifdef AIS_DEBUG
		Serial.print("pkgCnt="); Serial.println(pkgCnt);
		Serial.print("pkgNmb="); Serial.println(pkgNmb);
		Serial.print("seqMessageId="); Serial.println(seqMessageId);
		Serial.print("channel="); Serial.println(channel);
		Serial.print("length="); Serial.println(length);
		Serial.print("bitstream="); Serial.print(bitstream); Serial.print(" len:"); Serial.println(strlen(bitstream));
		Serial.print("fillBits="); Serial.println(fillBits);
#endif
		// Multi packets - first packet
		if (pkgCnt == 2 && pkgNmb == 1) {
			strcpy(bitstream_total, bitstream);
#ifdef AIS_DEBUG
			Serial.println("nach copy:");
			Serial.print("bitstream      ="); Serial.print(bitstream);       Serial.print(" len:"); Serial.println(strlen(bitstream));
			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
			Serial.print("Multi packets - first, pkgCnt=");	Serial.println(pkgCnt);
#endif
			return;
		}

		// Multi packets - second packet
		if (pkgCnt == 2 && pkgNmb == 2) {
#ifdef AIS_DEBUG
			Serial.println("vor tranfer:");
			Serial.print("bitstream=      "); Serial.print(bitstream);       Serial.print(" len:"); Serial.println(strlen(bitstream));
			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
#endif			
			strcat(bitstream_total, bitstream);
			strcpy(bitstream, bitstream_total);
			length = strlen(bitstream_total);

#ifdef AIS_DEBUG
			Serial.println("nach tranfer:");
			Serial.print("bitstream      ="); Serial.print(bitstream);       Serial.print(" len:"); Serial.println(strlen(bitstream));
			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
			Serial.print("Multi packets - second, pkgCnt="); Serial.println(pkgCnt);
#endif
		}

		// Single packet
		if (pkgCnt == 1 && pkgNmb == 1) {
#ifdef AIS_DEBUG
			Serial.print("bitstream=      "); Serial.print(bitstream);       Serial.print(" len:"); Serial.println(strlen(bitstream));
			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
			Serial.print("Single packet, pkgCnt=");	Serial.println(pkgCnt);
#endif
		}

		/*
		VDM/VDO
					*129038	AIS Class A Position Report								AIS VHF message 1
					*129038	AIS Class A Position Report								AIS VHF message 2
					*129038	AIS Class A Position Report								AIS VHF message 3
					*129793 AIS Base Station Report									AIS VHF message 4
					*129794	AIS Class A Static and Voyage Rel Data					AIS VHF message 5
					 129795 AIS Addressed Binary Message							AIS VHF message 6
					 129796 AIS Acknowledge											AIS VHF message 7
 					 129797 AIS Binary Broadcast Message							AIS VHF message 8
					*129798	AIS SAR Aircraft Position Report						AIS VHF message 9
					 129800 AIS UTC/Date Inquiry									AIS VHF message 10
					 129793	AIS UTC/Date Response									AIS VHF message 11
					 129801 AIS Addressed Safety Related Message					AIS VHF message 12
							AIS	Safety related acknowledgement						AIS VHF message 13
					 129802 AIS Safety Related Broadcast Message					AIS VHF message 14
					 129803 AIS Interrogation										AIS VHF message 15
					 129804 AIS Assignment Mode Command								AIS VHF message 16
					 129792 AIS DGNSS Broadcast Binary Message						AIS VHF message 17
					*129039	AIS Class B Position Report								AIS VHF message 18
					*129040	AIS Class B Extended Position Report					AIS VHF message 19
					 129805 AIS Data Link Management Message						AIS VHF message 20
					*129041	AIS Aids to Navigation (AtoN) Report					AIS VHF message 21
					 129806	AIS	Channel management									AIS VHF message 22
					 129807 AIS Class B Group Assignment							AIS VHF message 23
					*129809	AIS Class B "CS" Static Data, Part A					AIS VHF message 24
					*129810	AIS Class B "CS" Static Data, Part B					AIS VHF message 24
							AIS	Single slot binary message							AIS VHF message 25
							AIS	Multiple slot binary msg with Comm State			AIS VHF message 26
							AIS Position report for long range applications			AIS VHF message 27
		*/

		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;

			// AIS expect buf to be '\0' terminated
#ifdef AIS_DEBUG
			Serial.print("length="); Serial.println(length);
#endif
			bitstream[length] = '\0';

			AIS ais_msg(bitstream, fillBits);
//			AIS ais_msg("55Mf@6P00001MUS;7GQL4hh61L4hh6222222220t41H");

			msgType = ais_msg.get_type();
			msgNumeric = ais_msg.get_numeric_type();
			part_no = ais_msg.get_partno();
			repeat = ais_msg.get_repeat();
			mmsi = ais_msg.get_mmsi();

#ifdef AIS_DEBUG
			Serial.print("msgType="); Serial.println(msgType);
			Serial.print("msgNumeric="); Serial.println(msgNumeric);
			Serial.print("part_no="); Serial.println(part_no);
			Serial.print("repeat="); Serial.println(repeat);
			Serial.print("mmsi="); Serial.println(mmsi);
#endif
			if ((msgNumeric == 1) || (msgNumeric == 2) || (msgNumeric == 3)) {
				// Message 1,2,3 - Position Report
				// !AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*24
				// 129038 AIS Class A Position Report			AIS VHF messages 1, 2, 3

				/*	
					Parm#	Parameter			Value			Description
					01		Message ID				1	
					02		Repeat indicator		0			No repeat (default)
					03		User ID (MMSI)			265547250	
					04		Navigational status		0			Under way using engine
					05		Rate of turn ROTAIS		-2.86	
					06		SOG						13.9	
					07		Position accuracy		0			Low (> 10 m) (default)
					08		Longitude				11.8329767	
					09		Latitude				57.6603533	
					10		COG						40.4	
					11		True heading			41	
					12		Time stamp				53	
					13		Special manoeuvre indicator	0	
					14		Spare					0	
					15		RAIM-flag				0			RAIM not in use (default)

				AIS Class A Position Report 129038
				This parameter group provides data associated with the ITU-R M.1371 Messages 1, 2, and 3 Position Reports, autonomous,
				assigned, and response to interrogation, respectively. An AIS device may generate this parameter group either upon receiving a
				VHF data link message 1,2 or 3, or upon receipt of an ISO or NMEA request PGN (see ITU-R M.1371-1 for additional information).
					Field #	Field Description
					1		Message ID
					2		Repeat Indicator
					3		User ID
					4		Longitude
					5		Latitude
					6		Position Accuracy
					7		RAIM-flag
					8		Time Stamp
					9		COG
					10		SOG
					11		Communication State
					12		AIS Transceiver Information
					13		True Heading
					14		Rate of Turn
					15		Navigational Status
					16		Special Maneuver Indicator
					17		NMEA Reserved
					18		AIS Spare
					19		NMEA Reserved
					20		Sequence ID
				*/

				navstatus = ais_msg.get_navStatus();
				ROT = ais_msg.get_rot();
				ROT1 = (double)ROT / 4.733;
				ROT1 = ROT1 * ROT1;
				if (ROT < 0) {
					ROT1 = -ROT1;
				}
				SOG = ais_msg.get_SOG();
				accuracy = ais_msg.get_posAccuracy_flag();
				LONG = ais_msg.get_longitude();
				LONG1 = MinToDegrees(LONG);
				LAT = ais_msg.get_latitude();
				LAT1 = MinToDegrees(LAT);
				COG = ais_msg.get_COG();
				HDG = ais_msg.get_HDG();
				seconds = ais_msg.get_timeStamp();
				raim = ais_msg.get_raim_flag();

#ifdef AIS_DEBUG
				Serial.print("navstatus="); Serial.println(navstatus);
				Serial.print("ROT="); Serial.println(ROT1);
				Serial.print("SOG="); Serial.print(SOG / 10); Serial.print("."); Serial.println(SOG % 10);
				Serial.print("Accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("COG="); Serial.print(COG/10); Serial.print("."); Serial.println(COG % 10);
				Serial.print("HDG="); Serial.println(HDG);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("raim="); Serial.println(raim);
#endif
				SetN2kPGN129038(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, N2kaischannel_A_VDL_reception,
					HDG*degToRad, ROT1*degToRad, static_cast<tN2kAISNavStatus>(navstatus));
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
//			else if ((msgNumeric == 4) || (msgNumeric == 11)) {
			else if (msgNumeric == 4) {
				// Message 4 - Base station report
				// !AIVDM,1,1,,A,400TcdiuiT7VDR>3nIfr6>i00000,0*78
				// 129793 AIS Base Station Report				AIS VHF message 4

				// 129793 AIS UTC / Date Response				AIS VHF message 11

				/*
					Parm#	Parameter			Value	Description
					01		Message ID			4
					02		Repeat indicator	0		No repeat (default)
					03		User ID (MMSI)		000601011
					04		UTC Year			2012
					05		UTC Month			6
					06		UTC Day				8
					07		UTC Hour			7
					08		UTC Minute			38
					09		UTC Second			20
					10		Position accuracy	1		High (< 10 m; Differential Mode)
					11		Longitude			31*2.0108'E
					12		Latitude			29*52.2501'S
					13		EPFD Type			1		GPS
					14		Spare				0
					15		RAIM-flag			0		RAIM not in use (default)

					Message 11 - Mobile UTC and date response
					!AIVDM,1,1,,B,;8u:8CAuiT7Bm2CIM=fsDJ100000,0*51
					Parm#	Parameter			Value	Description
					01		Message ID			11
					02		Repeat indicator	0		No repeat (default)
					03		User ID (MMSI)		601000013
					04		UTC Year			2012
					05		UTC Month			6
					06		UTC Day				8
					07		UTC Hour			7
					08		UTC Minute			18
					09		UTC Second			53
					10		Position accuracy	0		Low (> 10 m) (default)
					11		Longitude			32*11.9718'E
					12		Latitude			29*50.2488'S
					13		EPFD Type			1		GPS
					14		Spare				0
					15		RAIM-flag			0		RAIM not in use (default)

					AIS UTC and Date Report 129793
					This parameter group provides data from ITU-R M.1371 message 4 Base Station Report providing position, time, date, and current
					slot number of a base station, and 11 UTC and date response message providing current UTC and date if available. An AIS device
					may generate this parameter group either upon receiving a VHF data link message 4 or 11, or upon receipt of an ISO or NMEA
					request PGN.
					Field#	Field Description
					1		Message ID
					2		Repeat Indicator
					3		User ID
					4		Longitude
					5		Latitude
					6		Position accuracy
					7		RAIM-flag
					8		NMEA 2000 Reserved
					9		Position time
					10		Communication State
					11		AIS Transceiver Information
					12		Position Date
					13		NMEA 2000 Reserved
					14		Type of Electronic Positioning Device
					15		Spare
				*/

				year = ais_msg.get_year();
				month = ais_msg.get_month();
				day = ais_msg.get_day();
				hour = ais_msg.get_hour();
				minute = ais_msg.get_minute();
				seconds = ais_msg.get_timeStamp();
				accuracy = ais_msg.get_posAccuracy_flag();
				LONG = ais_msg.get_longitude();
				LAT = ais_msg.get_latitude();
				LONG1 = MinToDegrees(LONG);
				LAT1 = MinToDegrees(LAT);
				epfd = ais_msg.get_epfd();
				raim = ais_msg.get_raim_flag();

#ifdef AIS_DEBUG
				Serial.print("year="); Serial.println(year);
				Serial.print("month="); Serial.println(month);
				Serial.print("day="); Serial.println(day);
				Serial.print("hour="); Serial.println(hour);
				Serial.print("minute="); Serial.println(minute);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("epfd="); Serial.println(epfd);
				Serial.print("raim="); Serial.println(raim);
#endif
				SetN2kPGN129793(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, (double)hour*3600+(double)minute*60+(double)seconds, (year-1970)*365+month*30+day, (tN2kGNSStype)epfd);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
			else if (msgNumeric == 5) {
				// !AIVDM,2,1,0,A,58wt8Ui`g??r21`7S=:22058<v05Htp000000015>8OA;0sk,0*7B
				// !AIVDM,2,2,0,A,eQ8823mDm3kP00000000000,2*5D
				// 129794 AIS Class A Static+Voyage Rel Data	AIS VHF message 5

				/*
					Parm#	Parameter				Value		Description
					01		Message ID				5
					02		Repeat indicator		0			No repeat (default)
					03		User ID (MMSI)			603916439
					04		AIS version indicator	0
					05		IMO number				439303422
					06		Call sign				ZA83R
					07		Name					ARCO AVON@@@@@@@@
					08		Type of ship & cargo	69			Passenger, No additional information
					09		Ship dimensions			A=113,B=31,C=17,D=11
					10		Type of EPFD			0			Undefined (default)
					11		ETA Month				3
					12		ETA Day					23
					13		ETA Hour				19
					14		ETA Minute				45
					15		Max. static draught		13.2
					16		Destination				HOUSTON@@@@@@@@@@@
					17		DTE (availability)		0			DTE available
					18		Spare					0

					AIS Class A Static and Voyage Related Data 129794
					This parameter group provides data associated with the ITU-R M.1371 Message 5 Ship Static and Voyage Related Data
					Message.  An AIS device may generate this parameter group either upon receiving a VHF data link message 5, or upon receipt of
					an ISO or NMEA request PGN.
					Field#  Field Description
					1		Message ID
					2		Repeat Indicator
					3		User ID
					4		IMO
					5		Call Sign
					6		Name
					7		Ship/Cargo Type
					8		Ship Length
					9		Ship Beam
					10		Position Reference Point from Starboard
					11		Position Reference Point aft of Ship's Bow
					12		Estimated Date of Arrival
					13		Estimated Time of Arrival
					14		Draft
					15		Destination
					16		AIS Version
					17		Type of Electronic Positioning Device
					18		Data Terminal Equipment (DTE)
					19		AIS Spare
					20		AIS Transceiver Information
					21		NMEA Reserved
					22		Sequence ID
				*/

				imo = ais_msg.get_imo();
				callsign = (char *)ais_msg.get_callsign();
				shipname = (char *)ais_msg.get_shipname();
				shiptype = ais_msg.get_shiptype();
				to_port = ais_msg.get_to_port();
				to_starboard = ais_msg.get_to_starboard();
				to_bow = ais_msg.get_to_bow();
				to_stern = ais_msg.get_to_stern();
				epfd = ais_msg.get_epfd();
				month = ais_msg.get_month();
				day = ais_msg.get_day();
				hour = ais_msg.get_hour();
				minute = ais_msg.get_minute();
				draught = ais_msg.get_draught();
				destination = (char *)ais_msg.get_destination();

#ifdef AIS_DEBUG
				Serial.print("imo="); Serial.println(imo);
				Serial.print("callsign="); Serial.println(callsign);
				Serial.print("shipname="); Serial.println(shipname);
				Serial.print("shiptype="); Serial.println(shiptype);
				Serial.print("to_bow="); Serial.println(to_bow);
				Serial.print("to_stern="); Serial.println(to_stern);
				Serial.print("to_port="); Serial.println(to_port);
				Serial.print("to_starboard="); Serial.println(to_starboard);
				Serial.print("epfd="); Serial.println(epfd);
				Serial.print("month="); Serial.println(month);
				Serial.print("day="); Serial.println(day);
				Serial.print("hour="); Serial.println(hour);
				Serial.print("minute="); Serial.println(minute);
				Serial.print("draught="); Serial.print(draught / 10); Serial.print("."); Serial.println(draught % 10);
				Serial.print("destination="); Serial.println(destination);
#endif
				SetN2kPGN129794(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, imo, callsign, shipname, shiptype,
					to_bow + to_stern, to_port + to_starboard, to_starboard, to_bow, (2021 - 1970) * 365 + month * 30 + day,
					(double)hour * 3600 + (double)minute * 60, (double)draught / 10, destination, 
					N2kaisv_ITU_R_M_1371_1, /*tN2kAISVersion AISversion,*/ N2kGNSSt_GPS, /*tN2kGNSStype GNSStype,*/
					N2kaisdte_Ready, /*tN2kAISDTE DTE,*/ N2kaischannel_A_VDL_reception /*tN2kAISTransceiverInformation AISTransceiverInformation*/);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
#ifdef AIRCRAFT
			else if (msgNumeric == 9) {
			// !AIVDM,1,1,,A,90009E?www<tSF0l4Q@>4?h20`GU,0*12
			// 129798 AIS SAR Aircraft Position Report		AIS VHF message 9
			/*
			Parm#  	Parameter  			Value  			Description
				01	Message ID 			9
				02	Repeat indicator 	0 				No repeat(default)
				03	User ID(MMSI) 		2388
				04	Altitude(m) 		4095
				05	SOG(knots) 			1,023.0
				06	Position accuracy 	0 				Low(> 10 m) (default)
				07	Longitude 			181*0.0000'E 	
				08	Latitude 			 91*0.0000'N 	
				09	COG 				360.0
				10	Time stamp 			63
				11	Altitude sensor 	0 				GNSS
				12	Spare 				0
				13	DTE 				1 				DTE not available(default)
				14	Spare 				0
				15	Assign mode flag 	0 				Autonomous and continuous mode(default)
				16	RAIM - flag 		0 				RAIM not in use(default)
				17	Comm.state selector flag	0		SOTDMA communication state follows
				18	Communication State 165349
			*/
			/*
			129798 AIS SAR Aircraft Position Report
				This parameter group provides data associated with the ITU - R M.1371 Message 9 SAR Aircraft Position Report Message for
				Airborne AIS units conducting Search and Rescue operations.An AIS device may generate this parameter group either upon
				receiving a VHF data link message 9, or upon receipt of an ISO or NMEA request.
				Field#	Field Description
				1		Message ID
				2		Repeat Indicator
				3		User ID
				4		Longitude
				5		Latitude
				6		Position Accuracy
				7		RAIM - Flag
				8		Time Stamp
				9		COG
				10		SOG
				11		Communication State
				12		AIS Transceiver Information
				13		Altitude
				14		Reserved for Regional Applications
				15		Data Terminal Equipment(DTE)
				16		AIS Spare
				17		NMEA Reserved
				18		Sequence ID
			*/

				Altitude(m) 		4095
				SOG = ais_msg.get_SOG();
				accuracy = ais_msg.get_posAccuracy_flag();
				LONG = ais_msg.get_longitude();
				LAT = ais_msg.get_latitude();
				LONG1 = MinToDegrees(LONG);
				LAT1 = MinToDegrees(LAT);
				COG = ais_msg.get_COG();
				seconds = ais_msg.get_timeStamp();
				11	Altitude sensor 	0 				GNSS
				12	Spare 				0
				13	DTE 				1 				DTE not available(default)
				14	Spare 				0
				15	Assign mode flag 	0 				Autonomous and continuous mode(default)				raim = ais_msg.get_raim_flag();
				raim = ais_msg.get_raim_flag();
				17	Comm.state selector flag	0		SOTDMA communication state follows
				18	Communication State 165349

#ifdef AIS_DEBUG
				Serial.print("SOG="); Serial.print(SOG / 10); Serial.print("."); Serial.println(SOG % 10);
				Serial.print("Accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("COG="); Serial.print(COG / 10); Serial.print("."); Serial.println(COG % 10);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("raim="); Serial.println(raim);
#endif
				//SetN2kPGN129798(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, imo, callsign, shipname, shiptype,
				//	to_bow + to_stern, to_port + to_starboard, to_starboard, to_bow, (2021 - 1970) * 365 + month * 30 + day,
				//	(double)hour * 3600 + (double)minute * 60, (double)draught / 10, destination,
				//	N2kaisv_ITU_R_M_1371_1, /*tN2kAISVersion AISversion,*/ N2kGNSSt_GPS, /*tN2kGNSStype GNSStype,*/
				//	N2kaisdte_Ready, /*tN2kAISDTE DTE,*/ N2kaisti_Channel_A_VDL_reception /*tN2kAISTransceiverInformation AISTransceiverInformation*/);
				//pNMEA2000->SendMsg(N2kMsg);
				//N2kTxCounter = N2kTxCounter + 1;
			}
#endif
			else if (msgNumeric == 18) {
				// !AIVDM,1,1,,A,B6CdCm0t3`tba35f@V9faHi7kP06,0*58
				// 129039 AIS Class B Position Report			AIS VHF message 18
				/*	
					Parm#	Parameter			Value		Description
					01		Message ID			18	
					02		Repeat indicator	0			No repeat (default)
					03		User ID (MMSI)		423302100	
					04		Spare				15	
					05		SOG					1.4	
					06		Position accuracy	1			Low (> 10 m) (default)
					07		Longitude			53*0.6598'E	
					08		Latitude			40*0.3170'N	
					09		COG					177	
					10		True heading		177	
					11		Time stamp			34	
					12		Spare				0	
					13		Class B unit flag	1			Class B/CS unit
					14		Class B display flag	1		Equipped with integrated display displaying Message 12 and 14
					15		Class B DSC flag	1			Equipped with DSC function (dedicated or time-shared)
					16		Class B band flag	1			Capable of operating over the whole marine band
					17		Class B Message 22 flag	1		Frequency management via Message 22
					18		Mode flag			0			Station operating in autonomous and continuous mode (default)
					19		RAIM-flag			0			RAIM not in use (default)
					20		Communication state selector flag	1	ITDMA communication state follows Communication State
					21		Sync State			3			Station is synchronized to another station based on the highest number of received stations or to another mobile station, which is directly synchronized to a base station
					22		Slot Increment		0	
					23		No. of slots		3			Consecutive slots
					24		Keep flag			0

					AIS Class B Position Report129039
					This parameter group provides data associated with the ITU-R M.1371 Message 18 Standard Class B Equipment Position
					Report.  An AIS device may generate this parameter group either upon receiving a VHF data link message 18, or upon receipt of
					an ISO or NMEA request PGN (see ITU-R M.1371-1 for additional information).
					Field#	Field Description
					1		Message ID
					2		Repeat Indicator
					3		User ID
					4		Longitude
					5		Latitude
					6		Position Accuracy
					7		RAIM-flag
					8		Time Stamp
					9		COG
					10		SOG
					11		Communication State
					12		AIS Transceiver Information
					13		True Heading
					14		Reserved for Regional Applications
					15		Reserved for Regional Applications
					16		Class B unit flag
					17		Class B Display Flag
					18		Class B DSC Flag
					19		Class B Band Flag
					20		Class B Msg 22 Flag
					21		Mode Flag
					22		Communication State Selector Flag
					23		NMEA Reserved
					24		Sequence ID
				*/	

				LAT = ais_msg.get_latitude();
				LONG = ais_msg.get_longitude();
				LAT1 = MinToDegrees(LAT);
				LONG1 = MinToDegrees(LONG);
				COG = ais_msg.get_COG();
				SOG = ais_msg.get_SOG();
				HDG = ais_msg.get_HDG();
				accuracy = ais_msg.get_posAccuracy_flag();
				raim = ais_msg.get_raim_flag();
				seconds = ais_msg.get_timeStamp();

#ifdef AIS_DEBUG
				Serial.print("SOG="); Serial.print(SOG / 10); Serial.print("."); Serial.println(SOG % 10);
				Serial.print("accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("COG="); Serial.print(COG / 10); Serial.print("."); Serial.println(COG % 10);
				Serial.print("HDG="); Serial.println(HDG);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("raim="); Serial.println(raim);
#endif
//				void SetN2kPGN129039(tN2kMsg& N2kMsg, uint8_t MessageID, tN2kAISRepeat Repeat, uint32_t UserID,
//					double Latitude, double Longitude, bool Accuracy, bool RAIM,
//					uint8_t Seconds, double COG, double SOG, tN2kAISTransceiverInformation AISTransceiverInformation,
//					double Heading, tN2kAISUnit Unit, bool Display, bool DSC, bool Band, bool Msg22, tN2kAISMode Mode,
//					bool State)
				SetN2kPGN129039(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, N2kaischannel_A_VDL_reception, (double)HDG, (tN2kAISUnit)0, false, false, false, false, /* bool Display, bool DSC, bool Band, bool Msg22*/
					N2kaismode_Autonomous, false /* bool State*/);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
			else if (msgNumeric == 19) {
				// !AIVDM,2,1,0,B,C8u:8C@t7@TnGCKfm6Po`e6N`:Va0L2J;06HV50JV?SjBPL3,0*28
				// !AIVDM,2,2,0,B,11RP,0*17
				// 129040 AIS Class B Extended Position Report	AIS VHF message 19
				/*
				Parm#	Parameter			Value		Description
					01	Message ID			19
					02	Repeat indicator	0			No repeat (default)
					03	User ID (MMSI)		601000013
					04	Spare				15
					05	SOG					2.9
					06	Position accuracy	0			Low (> 10 m) (default)
					07	Longitude			32*11.9718'E
					08	Latitude			29*50.2488'S
					09	COG					89
					10	True heading		90
					11	Time stamp			12
					12	Spare				15
					13	Name				TEST NAME CLSB MSG19
					14	Type of ship&cargo	37			Pleasure Craft
					15	Ship dimensions		A=7,B=6,C=2,D=3
					16	Type of EPFD		1			GPS
					17	RAIM-flag			0			RAIM not in use (default)
					18	DTE (availability)	1			DTE not available (default)
					19	Mode flag			0			Station operating in autonomous and continuous mode (default)
					20	Spare				0

					AIS Class B Position Report 129039
					This parameter group provides data associated with the ITU-R M.1371 Message 18 Standard Class B Equipment Position
					Report.  An AIS device may generate this parameter group either upon receiving a VHF data link message 18, or upon receipt of
					an ISO or NMEA request PGN (see ITU-R M.1371-1 for additional information).
					Field#	Field Description
					1		Message ID
					2		Repeat Indicator
					3		User ID
					4		Longitude
					5		Latitude
					6		Position Accuracy
					7		RAIM-flag
					8		Time Stamp
					9		COG
					10		SOG
					11		Communication State
					12		AIS Transceiver Information
					13		True Heading
					14		Reserved for Regional Applications
					15		Reserved for Regional Applications
					16		Class B unit flag
					17		Class B Display Flag
					18		Class B DSC Flag
					19		Class B Band Flag
					20		Class B Msg 22 Flag
					21		Mode Flag
					22		Communication State Selector Flag
					23		NMEA Reserved
					24		Sequence ID
				*/

				LAT = ais_msg.get_latitude();
				LONG = ais_msg.get_longitude();
				LAT1 = MinToDegrees(LAT);
				LONG1 = MinToDegrees(LONG);
				COG = ais_msg.get_COG();
				SOG = ais_msg.get_SOG();
				HDG = ais_msg.get_HDG();
				accuracy = ais_msg.get_posAccuracy_flag();
				raim = ais_msg.get_raim_flag();
				seconds = ais_msg.get_timeStamp();
				shipname = (char *)ais_msg.get_shipname();
				shiptype = ais_msg.get_shiptype();
				to_port = ais_msg.get_to_port();
				to_starboard = ais_msg.get_to_starboard();
				to_bow = ais_msg.get_to_bow();
				to_stern = ais_msg.get_to_stern();
				epfd = ais_msg.get_epfd();

#ifdef AIS_DEBUG
				Serial.print("SOG="); Serial.print(SOG / 10); Serial.print("."); Serial.println(SOG % 10);
				Serial.print("accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("COG="); Serial.print(COG / 10); Serial.print("."); Serial.println(COG % 10);
				Serial.print("HDG="); Serial.println(HDG);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("shipname="); Serial.println(shipname);
				Serial.print("shiptype="); Serial.println(shiptype);
				Serial.print("to_bow="); Serial.println(to_bow);
				Serial.print("to_stern="); Serial.println(to_stern);
				Serial.print("to_port="); Serial.println(to_port);
				Serial.print("to_starboard="); Serial.println(to_starboard);
				Serial.print("epfd="); Serial.println(epfd);
				Serial.print("raim="); Serial.println(raim);
#endif
				SetN2kPGN129040(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, HDG*degToRad, shiptype,
					to_bow + to_stern, to_port + to_starboard, to_starboard, to_bow, (tN2kGNSStype)epfd, shipname);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
#ifdef DATALINK
			else if (msgNumeric == 20) {
			// !AIVDM,1,1,,A,D02FhVQkTN ? b < `N00C8v000,2*48
			//
			// 	129805 AIS Data Link Management Message	                             AIS VHF message 19
			
				LAT = ais_msg.get_latitude();
				LONG = ais_msg.get_longitude();
				LAT1 = MinToDegrees(LAT);
				LONG1 = MinToDegrees(LONG);
				COG = ais_msg.get_COG();
				SOG = ais_msg.get_SOG();
				HDG = ais_msg.get_HDG();
				accuracy = ais_msg.get_posAccuracy_flag();
				raim = ais_msg.get_raim_flag();
				seconds = ais_msg.get_timeStamp();
				shipname = (char *)ais_msg.get_shipname();
				shiptype = ais_msg.get_shiptype();
				to_port = ais_msg.get_to_port();
				to_starboard = ais_msg.get_to_starboard();
				to_bow = ais_msg.get_to_bow();
				to_stern = ais_msg.get_to_stern();
				epfd = ais_msg.get_epfd();

#ifdef AIS_DEBUG
				Serial.print("SOG="); Serial.println(SOG);
				Serial.print("accuracy="); Serial.println(accuracy);
				Serial.print("LONG="); Serial.println(LONG1);
				Serial.print("LAT="); Serial.println(LAT1);
				Serial.print("COG="); Serial.println(COG);
				Serial.print("HDG="); Serial.println(HDG);
				Serial.print("seconds="); Serial.println(seconds);
				Serial.print("shipname="); Serial.println(shipname);
				Serial.print("shiptype="); Serial.println(shiptype);
				Serial.print("to_bow="); Serial.println(to_bow);
				Serial.print("to_stern="); Serial.println(to_stern);
				Serial.print("to_port="); Serial.println(to_port);
				Serial.print("to_starboard="); Serial.println(to_starboard);
				Serial.print("epfd="); Serial.println(epfd);
				Serial.print("raim="); Serial.println(raim);
#endif
				//SetN2kPGN129805(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
				//	accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, HDG*degToRad, shiptype,
				//	to_bow + to_stern, to_port + to_starboard, to_starboard, to_bow, (tN2kGNSStype)epfd, shipname);
				//pNMEA2000->SendMsg(N2kMsg);
				//N2kTxCounter = N2kTxCounter + 1;
			}
#endif
			else if (msgNumeric == 21) {
			// !AIVDM,2,1,5,B,E1c2;q@b44ah4ah0h:2ab@70VRpU<Bgpm4:gP50HH`Th`QF5,0*79
			// !AIVDM,2,2,5,B,1CQ1A83PCAH0,0*62
			// oder
			// !AIVDM,1,1,,A,ENjOsrPppg@6a9Qh1Pb0W4PP0000RRW4:fabP00000N01>,4*50
			// oder
			// !AIVDM,1,1,,A,ENjOsphrg@6a9Qh92SSTWh1PV0Q0Slm@:r;8000000N014R@@9,4*2A    (name: 5^ MRSC REGGIO CALAB, ext name: RIA@)
			//
			// 129041 AIS Aids to Navigation(AtoN) Report	AIS VHF message 21
				
			/*
			Parm#  	Parameter  			Value  			Description
				01	Message ID			21
				02	Repeat indicator	0				No repeat (default)
				03	User ID (MMSI)		112233445
				04	Nav Type			1				Reference point
				05	Name				THIS IS A TEST NAME1
				06	Position Accuracy	0				Low (> 10 m) (default)
				07	Longitude			145*10.8600'E
				08	Latitude			38*13.2100'S
				09	Dimensions			A=5,B=3,C=3,D=5
				10	EPFD Type			1				GPS
				11	UTC Time stamp		9
				12	On/Off Position Ind	1				Off position
				13	AtoN Reg. App.		00001010
				14	Raim Status			0				RAIM not in use (default)
				15	Virtual Flag		0				Real AtoN (default)
				16	Mode Indicator		1				Assigned mode
				17	Spare				0
				18	Ext. Name			EXTENDED NAME

				129041 AIS Aids to Navigation(AtoN) Report
				This PGN provides information received from an AtoN AIS station conforming to ITU-R M.1371-4
				Message 21. The AtoN station maybe mounted on an aid-to-navigation or this message may be
				transmitted by a fixed station when the functionality of an AtoN stationis integrated into the fixed station.
				This message is typically transmitted autonomously at a rate of once every three (3) min. Otherreporting
				rates are possible when the AtoN device has received an assigned mode command (Message 16) via the
				VHF data link, orby an external command such as PGN 129804 - AIS Assignment Mode Command
				Field # Field Description
				1 Message ID
				2 Repeat Indicator
				3 ID
				4 Longitude
				5 Latitude
				6 Position Accuracy
				7 RAIM Flag
				8 Time Stamp
				9 AtoN Structure Length/Diameter
				10 AtoN Structure Beam/Diameter
				11 Position Reference Point from Starboard Structure Edge/Radius
				12 Position Reference Point from True North facing Structure Edge/Radius
				13 Aid to Navigation (AtoN) Type
				14 Off Position Indicator
				15 Virtual AtoN Flag
				16 Assigned Mode Flag
				17 AIS Spare
				18 Electronic Fixing Position Fixing Device Type
				19 NMEA Reserved
				20 AtoN Status
				21 AIS Transceiver Information
				22 NMEA Reserved
				23 Aid to Navigation (AtoN) Name
			*/

			imo = ais_msg.get_imo();
			aidname = (char *)ais_msg.get_aidname();
			aidtype = ais_msg.get_aidtype();
			LAT = ais_msg.get_latitude();
			LONG = ais_msg.get_longitude();
			LAT1 = MinToDegrees(LAT);
			LONG1 = MinToDegrees(LONG);
			to_port = ais_msg.get_to_port();
			to_starboard = ais_msg.get_to_starboard();
			to_bow = ais_msg.get_to_bow();
			to_stern = ais_msg.get_to_stern();
			accuracy = ais_msg.get_posAccuracy_flag();
			raim = ais_msg.get_raim_flag();
			seconds = ais_msg.get_timeStamp();
			epfd = ais_msg.get_epfd();
			aidnameext = (char *)ais_msg.get_aidnameext();

			tN2kAISAtoNReportData data_tx;

			data_tx.MessageID = 21;
			data_tx.Repeat = N2kaisr_Final;
			data_tx.UserID = mmsi;
			strcat(aidname, aidnameext);
			data_tx.SetAtoNName(aidname);
			data_tx.AtoNType = (tN2kAISAtoNType)aidtype;
			data_tx.Longitude = LONG1;
			data_tx.Latitude = LAT1;
			data_tx.Accuracy = accuracy;
			data_tx.RAIM = raim;
			data_tx.Seconds = seconds;
			data_tx.Length = to_bow+to_stern;
			data_tx.Beam = to_port+to_starboard;
			data_tx.PositionReferenceStarboard = to_starboard;
			data_tx.PositionReferenceTrueNorth = to_bow;
			data_tx.OffPositionIndicator = true;
			data_tx.VirtualAtoNFlag = true;
			data_tx.AssignedModeFlag = true;
			data_tx.GNSSType = N2kGNSSt_Chayka;
			data_tx.AtoNStatus = 0x00;
			data_tx.AISTransceiverInformation = N2kaischannel_B_VDL_transmission;

#ifdef AIS_DEBUG
			Serial.print("aidname="); Serial.println(aidname);
			Serial.print("aidtype="); Serial.println(aidtype);
			Serial.print("mmsi="); Serial.println(mmsi);
			Serial.print("LONG="); Serial.println(LONG1);
			Serial.print("LAT="); Serial.println(LAT1);
			Serial.print("seconds="); Serial.println(seconds);
			Serial.print("to_bow="); Serial.println(to_bow);
			Serial.print("to_stern="); Serial.println(to_stern);
			Serial.print("to_port="); Serial.println(to_port);
			Serial.print("to_starboard="); Serial.println(to_starboard);
			Serial.print("epfd="); Serial.println(epfd);
			Serial.print("aidnameext="); Serial.println(aidnameext);
			Serial.print("data_tx="); Serial.println(data_tx.AtoNName);
#endif
			SetN2kAISAtoNReport(N2kMsg, data_tx);      // PGN 129041
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
		}
		else if (msgNumeric == 24)
		{
				// !AIVDM,1,1,,B,H42M;bh4hTq@E8VoT0000000003,2*3F
				// 129809 AIS Class B "CS" Static Data, Part A	AIS VHF message 24
				/*
					Parm#	Parameter			Value		Description
					01		Message ID			24
					02		Repeat indicator	0
					03		User ID (MMSI)		271010731
					04		Message part number	0			0: Part A
					05		Name				ALINTERI-9@@@@@@@@@@
				*/

				// !AIVDM,1,1,,B,H42M;blti1hhllqD31jmni00g0=0,0*52
				// 129810 AIS Class B "CS" Static Data, Part B	AIS VHF message 24
				/*
					Parm#	Parameter			Value		Description  ↓
					01		Message ID			24
					02		Repeat indicator	0
					03		User ID (MMSI)		271010731
					04		Message part number	1			1: Part B
					05		Type of ship & cargo	60		Passenger, all ships of this type
					06		Vendor ID			1A00449
					07		Call sign			TCA2561
					08		Ship dimensions		A=0,B=47,C=0,D=13
					09		Spare				0
				*/
				if (part_no == 0) {
					//
					// Part A
					//
					shipname = (char *)ais_msg.get_shipname();

#ifdef AIS_DEBUG
					Serial.println("Part A");
					Serial.print("shipname="); Serial.println(shipname);
#endif
					SetN2kPGN129809(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, shipname);
					pNMEA2000->SendMsg(N2kMsg);
					N2kTxCounter = N2kTxCounter + 1;
				}

				if (part_no == 1) {
					//
					// Part B
					//
					callsign = (char *)ais_msg.get_callsign();
					shiptype = ais_msg.get_shiptype();
					vendorid = ais_msg.get_vendorid();
					to_port = ais_msg.get_to_port();
					to_starboard = ais_msg.get_to_starboard();
					to_bow = ais_msg.get_to_bow();
					to_stern = ais_msg.get_to_stern();

#ifdef AIS_DEBUG
					Serial.println("Part B");
					Serial.print("shiptype="); Serial.println(shiptype);
					Serial.print("vendorid="); Serial.println(vendorid);
					Serial.print("callsign="); Serial.println(callsign);
					Serial.print("to_bow="); Serial.println(to_bow);
					Serial.print("to_stern="); Serial.println(to_stern);
					Serial.print("to_port="); Serial.println(to_port);
					Serial.print("to_starboard="); Serial.println(to_starboard);
#endif
					SetN2kPGN129810(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, shiptype, (char *)vendorid, callsign,
						to_bow+to_stern, to_port+to_starboard, to_starboard, to_bow, 0);
					pNMEA2000->SendMsg(N2kMsg);
					N2kTxCounter = N2kTxCounter + 1;
				}
			}
			else {
				Serial.print("not supported AIS msg, msgType="); Serial.print(msgType); Serial.print(", msgNumeric="); Serial.println(msgNumeric);
				Serial.println("*****************************************************************************************************************");
			}
		}
	}
	else
		if (NMEA0183HandlersDebugStream != 0) {
			NMEA0183HandlersDebugStream->println("Failed to parse VDM/VDO");
			NMEA0183HandlersDebugStream->print("pkgCnt="); NMEA0183HandlersDebugStream->println(pkgCnt);
			NMEA0183HandlersDebugStream->print("pkgNmb="); NMEA0183HandlersDebugStream->println(pkgNmb);
			NMEA0183HandlersDebugStream->print("seqMessageId="); NMEA0183HandlersDebugStream->println(seqMessageId);
			NMEA0183HandlersDebugStream->print("channel="); NMEA0183HandlersDebugStream->println(channel);
			NMEA0183HandlersDebugStream->print("length="); NMEA0183HandlersDebugStream->println(length);
			NMEA0183HandlersDebugStream->print("bitstream="); NMEA0183HandlersDebugStream->println(bitstream);
			NMEA0183HandlersDebugStream->print("fillBits="); NMEA0183HandlersDebugStream->println(fillBits);
		}
}
