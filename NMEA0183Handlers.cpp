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
 
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include <AIS.h>

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
  
  Serial.println("NMEA0183RxCounter+1");

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
	  Serial.print("NMEA0183 message parsed: ");
	  Serial.print(NMEA0183Handlers[iHandler].Code);
	  Serial.print(" - ");
	  Serial.println(NMEA0183Handlers[iHandler].NMEA0183seq);

	  NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
  }
  else {
	  if (NMEA0183HandlersDebugStream != 0) {
		  NMEA0183HandlersDebugStream->println("NMEA0183 message parsed, no Handler found.");
	  }
	  Serial.print("NMEA0183 message parsed, no Handler found: ");
	  Serial.println(NMEA0183Msg.MessageCode());
	  Serial.println("*******************************************************************************************************************");
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
	int originIDi;
	int destIDi;

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

/*
RMB		129283 Cross Track Error
		129284 Navigation Data
		129285 Navigation � Route / WP information
*/
		if (pNMEA2000 != 0) {
			tN2kMsg N2kMsg;
			SetN2kXTE(N2kMsg, 1, N2kxtem_Autonomous, false, nmTom*XTE);
			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;
			
			//3B 9F D1 47 4F 54 4F 20 43 55 52 53 4F 52 00 00 00 00 00 30 30 30 31 8A 72 0F 6F 54 B6 09 00 FF
			//3B9F2B474F544F20435552534F52000000000030303031 CA594F 56 2447 0800FF
			//3B9FF0474F544F20435552534F52203200383730303032 CF6F54 6C 345B 0700FF
			//3B9FB5474F544F20435552534F52203300353130303033 7954FE 50 E004 0700FF

			N2kMsg.SetPGN(130848UL);
			N2kMsg.Priority = 7;
			N2kMsg.Destination = 255;
			N2kMsg.AddByte(0x3b);  // lower 8 Bit of Manufacturer Code, Raymarine=1851=0x073b
			N2kMsg.AddByte(0x9f);  // high 3 Bit = Industry Code, Marine Industry=4, 2 Bit Reserved, higher 3 Bit of Manufacturer Code"
			N2kMsg.AddByte(0xd1);  // Proprietary ID ??? ändert sich, zählt hoch, muss das ???

			N2kMsg.AddStr0(destID, 16);

			N2kMsg.AddStr("0001", 4);    // oder WP-Number ??

			N2kMsg.AddByte(0x0);  // ???
			N2kMsg.AddByte(BTW / 1.455); // BTW for MFD Axiom Pro 9, Bearing Mode in Unit setting must be set to TRUE
			N2kMsg.AddByte(0x0);  // ???

			N2kMsg.AddByte(BTW / 1.455); // BTW for AotoPilot
			                             // 8:12°, 16:23°, 127:186°, 160:235, 192:282°, 224:329°, 240:352°

			N2kMsg.AddByte(0x0);  // ???
			N2kMsg.AddByte(0x0);  // ???

			N2kMsg.Add2ByteUDouble(DTW / 35.385, 0.01);  // DTW 1=0.35385nm

			N2kMsg.AddByte(0xff);

			pNMEA2000->SendMsg(N2kMsg);
			N2kTxCounter = N2kTxCounter + 1;

			//3B 9F 01 00 47 4F 54 4F 20 43 55 52 53 4F 52 00 00 00 00 00 FF 00 FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF FF 0D FF FF FF FF FF FF FF FF
			//3B9F0100474F544F20435552534F520000000000FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF
			//3B9F0200474F544F20435552534F522032003837FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF
			//3B9F0300474F544F20435552534F522033003531FF00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0DFFFFFFFFFFFFFFFF

			N2kMsg.SetPGN(130918UL);
			N2kMsg.Priority = 7;
			N2kMsg.Destination = 255;
			N2kMsg.AddByte(0x3b);  // lower 8 Bit of Manufacturer Code, Raymarine=1851=0x073b
			N2kMsg.AddByte(0x9f);  // high 3 Bit = Industry Code, Marine Industry=4, 2 Bit Reserved, higher 3 Bit of Manufacturer Code"
			N2kMsg.AddByte(0x01);  // Proprietary ID ???
			N2kMsg.AddByte(0x00);

//			N2kMsg.AddStr("GOTO CURSOR", 16);
			N2kMsg.AddStr(destID, 16);

/*
			N2kMsg.AddStr("GOTO CURSOR", strlen("GOTO CURSOR"));
			N2kMsg.AddByte(0x00);  // Rest Waypoint Name insgesamt 16 Stellen
			N2kMsg.AddByte(0x00);
			N2kMsg.AddByte(0x00);
			N2kMsg.AddByte(0x00);
			N2kMsg.AddByte(0x00);
*/
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

//			pNMEA2000->SendMsg(N2kMsg);
//			N2kTxCounter = N2kTxCounter + 1;

			/*
		     * PGN129284 only gives route/wp data ahead in the Active Route. So originID will always be 0 and destinationID will always be 1.
			 * Unclear why these ID's need to be set in PGN129284. On B&G Triton displays other values are ignored anyway.
			 */
     
		    bool ArrivalCircleEntered = arrivalAlarm == 'A';

		    //PerpendicularCrossed not calculated yet.
		    //Need to calculate it based on current lat/long, pND->bod.magBearing and pND->rmb.lat/long
		    bool PerpendicularCrossed = false;

		    SetN2kNavigationInfo(N2kMsg, 1, DTW*nmTom, N2khr_true, PerpendicularCrossed, ArrivalCircleEntered, N2kdct_GreatCircle,
				N2kDoubleNA, 0, /* double ETATime, int16_t ETADate N2kInt16NA */ BTW*degToRad, /* double BearingToOriginal */
				BTW*degToRad, originIDi, destIDi, /* uint8_t OriginWaypointNumber, uint8_t DestinationWaypointNumber*/ Latitude, Longitude, VMG*knToms);
//		      pNMEA2000->SendMsg(N2kMsg);
//		      N2kTxCounter = N2kTxCounter + 1;
			

			// 129285 Waypoint Data wird weder von eS97 noch von i70 empfangen und angezeigt

			char routename[120] = "Route";
			char *route = routename;

			SetN2kPGN129285(N2kMsg, 0, 0, 0, 7, 0, route);

			AppendN2kPGN129285(N2kMsg, 0, originID, Latitude, Longitude);
			AppendN2kPGN129285(N2kMsg, 1, destID, Latitude + 10.0, Longitude + 10.0);
//	  		  pNMEA2000->SendMsg(N2kMsg);
//            N2kTxCounter = N2kTxCounter + 1;
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
/*
!AIVDM,1,1,,B,177KQJ5000G?tO`K>RA1wUbN0TKH,0*5C
!AIVDM,1,1,,A,13u?etPv2;0n:dDPwUM1U1Cb069D,0*23
!AIVDM,1,1,,A,400TcdiuiT7VDR>3nIfr6>i00000,0*78
!AIVDM,2,1,0,A,58wt8Ui`g??r21`7S=:22058<v05Htp000000015>8OA;0sk,0*7B
!AIVDM,2,2,0,A,eQ8823mDm3kP00000000000,2*5D
!AIVDM,1,1,4,B,6>jR0600V:C0>da4P106P00,2*02
!AIVDM,2,1,9,B,61c2;qLPH1m@wsm6ARhp<ji6ATHd<C8f=Bhk>34k;S8i=3To,0*2C
!AIVDM,2,2,9,B,Djhi=3Di<2pp=34k>4D,2*03
!AIVDM,1,1,1,B,8>h8nkP0Glr=<hFI0D6??wvlFR06EuOwgwl?wnSwe7wvlOw?sAwwnSGmwvh0,0*17
*/
void HandleVDM(const tNMEA0183Msg &NMEA0183Msg) {
	uint8_t pkgCnt;
	uint8_t pkgNmb;
	unsigned int seqMessageId;
	char channel; 
	unsigned int length;
	char bitstream[512];
    static char bitstream_total[512];
	unsigned int fillBits;

//	uint8_t msgType;
	unsigned int msgNumeric;
	uint8_t part_no;
	uint8_t repeat;
	uint32_t mmsi;
	uint32_t mothership_mmsi;
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
	uint8_t ROT;
	uint32_t imo;
	uint8_t shiptype;
	uint8_t to_port;
	uint8_t to_starboard;
	uint16_t to_bow;
	uint16_t to_stern;
	uint8_t epfd;
	uint8_t draught;
	uint8_t navstatus;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	char name[120];
	char *shipname = name;
	char dest[20];
	char *destination = dest;
	char sign[42];
	char *callsign = sign;
	char vendor[18];
	char *vendorid = vendor;

	length = 500;

	NMEA0183.SendMessage(NMEA0183Msg);       // Send VDM NMEA0183 message to the NMEA0183 out stream (USB im Teensy)

	if (NMEA0183ParseVDM_nc(NMEA0183Msg, pkgCnt, pkgNmb, seqMessageId, channel, length, bitstream, fillBits)) {

/*
		Serial.print("pkgCnt="); Serial.println(pkgCnt);
		Serial.print("pkgNmb="); Serial.println(pkgNmb);
		Serial.print("seqMessageId="); Serial.println(seqMessageId);
		Serial.print("channel="); Serial.println(channel);
		Serial.print("length="); Serial.println(length);
		Serial.print("bitstream="); Serial.print(bitstream); Serial.print(" len:"); Serial.println(strlen(bitstream));
		Serial.print("fillBits="); Serial.println(fillBits);
*/

		// Multi packets - first
		if (pkgCnt == 2 && pkgNmb == 1) {
			strcpy(bitstream_total, bitstream);
//			Serial.println("nach copy:");
//			Serial.print("bitstream="); Serial.print(bitstream); Serial.print(" len:"); Serial.println(strlen(bitstream));
//			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
//			Serial.print("Multi packets - first, pkgCnt=");	Serial.println(pkgCnt);
			return;
		}

		// Multi packets - second
		if (pkgCnt == 2 && pkgNmb == 2) {
//			Serial.println("vor tranfer:");
//			Serial.print("bitstream="); Serial.print(bitstream); Serial.print(" len:"); Serial.println(strlen(bitstream));
//			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
			
			strcat(bitstream_total, bitstream);
			strcpy(bitstream, bitstream_total);

//			Debug
//			strcpy(bitstream, "58wt8Ui`g??r21`7S=:22058<v05Htp000000015>8OA;0skeQ8823mDm3kP00000000000");
//			!AIVDM, 1, 1, , A, H42M9J0h51 < DdT000000000, 2 * 70
//			!AIVDM, 2, 2, 9, A, 88888888882, 2 * 2F
//			!AIVDM, 1, 1, , A, 142O@Bh0001pnNJFvG;75GE < 0HFw, 0 * 16
//			!AIVDM, 1, 1, , B, 1815 ? wh1B < QpddLFuD58T6q@00Rv,0 * 27

//			Serial.println("nach tranfer:");
//			Serial.print("bitstream="); Serial.print(bitstream); Serial.print(" len:"); Serial.println(strlen(bitstream));
//			Serial.print("bitstream_total="); Serial.print(bitstream_total); Serial.print(" len:"); Serial.println(strlen(bitstream_total));
//			Serial.print("Multi packets - second, pkgCnt="); Serial.println(pkgCnt);
		}

		/*
		VDM/VDO
					*129038	AIS Class A Position Report								AIS VHF message 1
					*129038	AIS Class A Position Report								AIS VHF message 2
					*129038	AIS Class A Position Report								AIS VHF message 3
					 129793 AIS Base Station Report									AIS VHF message 4
					*129794	AIS Class A Static and Voyage Rel Data					AIS VHF message 5
					 129795 AIS Addressed Binary Message							AIS VHF message 6
					 129796 AIS Acknowledge											AIS VHF message 7
 					 129797 AIS Binary Broadcast Message							AIS VHF message 8
					 129798	AIS SAR Aircraft Position Report						AIS VHF message 9
					 129800 AIS UTC/Date Inquiry									AIS VHF message 10
					 129793	AIS UTC/Date Response									AIS VHF message 11
					 129801 AIS Addressed Safety Related Message					AIS VHF message 12
							AIS	Safety related acknowledgement						AIS VHF message 13
					 129802 AIS Safety Related Broadcast Message					AIS VHF message 14
					 129803 AIS Interrogation										AIS VHF message 15
					 129804 AIS Assignment Mode Command								AIS VHF message 16
					 129792 AIS DGNSS Broadcast Binary Message						AIS VHF message 17
					*129039	AIS Class B Position Report								AIS VHF message 18
					 129040	AIS Class B Extended Position Report					AIS VHF message 19
					 129805 AIS Data Link Management Message						AIS VHF message 20
					 129041	AIS Aids to Navigation (AtoN) Report					AIS VHF message 21
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
			bitstream[length] = '\0';

			AIS ais_msg(bitstream, fillBits);

			enum AIS::Nmea0183AisMessages msgType = ais_msg.get_type();
//			msgType = ais_msg.get_type();
			msgNumeric = ais_msg.get_numeric_type();
			part_no = ais_msg.get_partno();
			repeat = ais_msg.get_repeat();
			mmsi = ais_msg.get_mmsi();

/*
			LAT = ais_msg.get_latitude();
			LONG = ais_msg.get_longitude();
			LAT1 = MinToDegrees(LAT);
			LONG1 = MinToDegrees(LONG);
			accuracy = ais_msg.get_posAccuracy_flag();
			raim = ais_msg.get_raim_flag();
			seconds = ais_msg.get_timeStamp();
			COG = ais_msg.get_COG();
			SOG = ais_msg.get_SOG();
			HDG = ais_msg.get_HDG();
			ROT = ais_msg.get_rot();
			imo = ais_msg.get_imo();
			to_port = ais_msg.get_to_port();
			to_starboard = ais_msg.get_to_starboard();
			to_bow = ais_msg.get_to_bow();
			to_stern = ais_msg.get_to_stern();
			shiptype = ais_msg.get_shiptype();
			epfd = ais_msg.get_epfd();
			draught = ais_msg.get_draught();
			navstatus = ais_msg.get_navStatus();
			month = ais_msg.get_month();
			day = ais_msg.get_day();
			hour = ais_msg.get_minute();
			minute = ais_msg.get_minute();
			shipname = ais_msg.get_shipname();
			destination = ais_msg.get_destination();
			callsign = ais_msg.get_callsign();
			vendorid = ais_msg.get_vendorid();
			mothership_mmsi = ais_msg.get_mothership_mmsi();

			Serial.print("msgType="); Serial.println(msgType);
			Serial.print("msgNumeric="); Serial.println(msgNumeric);
			Serial.print("repeat="); Serial.println(repeat);
			Serial.print("mmsi="); Serial.println(mmsi);
			Serial.print("LAT="); printDegrees(LAT);
			Serial.print("LONG="); printDegrees(LONG);
			Serial.print("accuracy="); Serial.println(accuracy);
			Serial.print("seconds="); Serial.println(seconds);
			Serial.print("COG="); Serial.print(COG/10); Serial.print("."); Serial.println(COG % 10);
			Serial.print("SOG="); Serial.print(SOG/10); Serial.print("."); Serial.println(SOG % 10);
			Serial.print("HDG="); Serial.println(HDG);
			Serial.print("ROT="); Serial.println(ROT);
			Serial.print("imo="); Serial.println(imo);
			Serial.print("to_port="); Serial.println(to_port);
			Serial.print("to_starboard="); Serial.println(to_starboard);
			Serial.print("to_bow="); Serial.println(to_bow);
			Serial.print("to_stern="); Serial.println(to_stern);
			Serial.print("shiptype="); Serial.println(shiptype);
			Serial.print("epfd="); Serial.println(epfd);
			Serial.print("draught="); Serial.print(draught/10); Serial.print("."); Serial.println(draught % 10);
			Serial.print("navstatus="); Serial.println(navstatus);
			Serial.print("shipname="); Serial.println(shipname);
			Serial.print("destination="); Serial.println(destination);
			Serial.print("month="); Serial.println(month);
			Serial.print("day="); Serial.println(day);
			Serial.print("hour="); Serial.println(hour);
			Serial.print("minute="); Serial.println(minute);
			Serial.print("callsign="); Serial.println(callsign);
			Serial.print("vendorid="); Serial.println(vendorid);
*/

			if ((msgNumeric == 1) || (msgNumeric == 2) || (msgNumeric == 3)) {
				// 129038 AIS Class A Position Report			AIS VHF messages 1, 2, 3
				Serial.print("msgType="); Serial.println(msgType);
				Serial.print("msgNumeric="); Serial.println(msgNumeric);
				Serial.print("part_no="); Serial.println(part_no);
				Serial.print("mmsi="); Serial.println(mmsi);

				LAT = ais_msg.get_latitude();
				LONG = ais_msg.get_longitude();
				LAT1 = MinToDegrees(LAT);
				LONG1 = MinToDegrees(LONG);
				COG = ais_msg.get_COG();
				SOG = ais_msg.get_SOG();
				HDG = ais_msg.get_HDG();
				ROT = ais_msg.get_rot();
				navstatus = ais_msg.get_navStatus();
				accuracy = ais_msg.get_posAccuracy_flag();
				raim = ais_msg.get_raim_flag();
				seconds = ais_msg.get_timeStamp();

//				Serial.print("LAT="); printDegrees(LAT);
//				Serial.print("LONG="); printDegrees(LONG);
				Serial.print("LAT="); Serial.println(LAT);
				Serial.print("LONG="); Serial.println(LONG);
				Serial.print("COG="); Serial.println(COG);
				Serial.print("SOG="); Serial.println(SOG);
				Serial.print("HDG="); Serial.println(HDG);
				Serial.print("ROT="); Serial.println(ROT);
				Serial.print("navstatus="); Serial.println(navstatus);

				SetN2kPGN129038(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, HDG*degToRad / 10, ROT*degToRad / 10, static_cast<tN2kAISNavStatus>(navstatus));
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
			else if ((msgNumeric == 4) || (msgNumeric == 11)) {
				// 129793 AIS Base Station Report				AIS VHF message 4
				// 129793 AIS UTC / Date Response				AIS VHF message 11
/*
Message 11 - Mobile UTC and date response
!AIVDM,1,1,,B,;8u:8CAuiT7Bm2CIM=fsDJ100000,0*51
Parm# Parameter			Value		Description
01	Message ID			11
02	Repeat indicator	 0			No repeat (default)
03	User ID (MMSI)		601000013
04	UTC Year			2012
05	UTC Month			6
06	UTC Day				8
07	UTC Hour			7
08	UTC Minute			18
09	UTC Second			53
10	Position accuracy	0			Low (> 10 m) (default)
11	Longitude			32*11.9718'E
12	Latitude			29*50.2488'S
13	EPFD Type			1			GPS
14	Spare				0
15	RAIM-flag			0			RAIM not in use (default)

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
				Serial.print("msgType="); Serial.println(msgType);
				Serial.print("msgNumeric="); Serial.println(msgNumeric);
				//SetN2kPGN129793(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, imo, callsign, shipname, shiptype,
				//pNMEA2000->SendMsg(N2kMsg);
				//N2kTxCounter = N2kTxCounter + 1;
			}
			else if (msgNumeric == 5) {
				// 129794 AIS Class A Static+Voyage Rel Data	AIS VHF message 5
				Serial.print("msgType="); Serial.println(msgType);
				Serial.print("msgNumeric="); Serial.println(msgNumeric);
				Serial.print("mmsi="); Serial.println(mmsi);

				imo = ais_msg.get_imo();
				to_port = ais_msg.get_to_port();
				to_starboard = ais_msg.get_to_starboard();
				to_bow = ais_msg.get_to_bow();
				to_stern = ais_msg.get_to_stern();
				shiptype = ais_msg.get_shiptype();
				draught = ais_msg.get_draught();
				epfd = ais_msg.get_epfd();
				month = ais_msg.get_month();
				day = ais_msg.get_day();
				hour = ais_msg.get_minute();
				minute = ais_msg.get_minute();
				shipname = (char *)ais_msg.get_shipname();
				destination = (char *)ais_msg.get_destination();
				callsign = (char *)ais_msg.get_callsign();

				Serial.print("imo="); Serial.println(imo);
				Serial.print("to_port="); Serial.println(to_port);
				Serial.print("to_starboard="); Serial.println(to_starboard);
				Serial.print("to_bow="); Serial.println(to_bow);
				Serial.print("to_stern="); Serial.println(to_stern);
				Serial.print("shipname="); Serial.println(shipname);
				Serial.print("destination="); Serial.println(destination);
				Serial.print("callsign="); Serial.println(callsign);
				Serial.print("shiptype="); Serial.println(shiptype);
				Serial.print("epfd="); Serial.println(epfd);
				Serial.print("draught="); Serial.print(draught / 10); Serial.print("."); Serial.println(draught % 10);
				Serial.print("month="); Serial.println(month);
				Serial.print("day="); Serial.println(day);
				Serial.print("hour="); Serial.println(hour);
				Serial.print("minute="); Serial.println(minute);

				SetN2kPGN129794(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, imo, callsign, shipname, shiptype,
					to_bow+to_stern, to_port+to_starboard, to_starboard, to_bow, 0, /*uint16_t ETAdate,*/ 0, /*double ETAtime,*/
					(double)draught / 10, destination, N2kaisv_ITU_R_M_1371_1, /*tN2kAISVersion AISversion,*/ N2kGNSSt_GPS, /*tN2kGNSStype GNSStype,*/
					N2kaisdte_Ready, /*tN2kAISDTE DTE,*/ N2kaisti_Channel_A_VDL_reception /*tN2kAISTranceiverInfo AISinfo*/);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
			else if (msgNumeric == 18) {
				// 129039 AIS Class B Position Report			AIS VHF message 18
				Serial.print("msgType="); Serial.println(msgType);
				Serial.print("msgNumeric="); Serial.println(msgNumeric);
				Serial.print("mmsi="); Serial.println(mmsi);

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

				Serial.print("LAT="); Serial.println(LAT);
				Serial.print("LONG="); Serial.println(LONG);
				Serial.print("COG="); Serial.println(COG);
				Serial.print("SOG="); Serial.println(SOG);
				Serial.print("HDG="); Serial.println(HDG);

				SetN2kPGN129039(N2kMsg, seqMessageId, static_cast<tN2kAISRepeat>(repeat), mmsi, LAT1, LONG1,
					accuracy, raim, seconds, COG*degToRad / 10, SOG*knToms / 10, HDG, (tN2kAISUnit)0, /* tN2kAISUnit Unit*/ false, false, false, false, /* bool Display, bool DSC, bool Band, bool Msg22*/
					N2kaismode_Autonomous, /*tN2kAISMode Mode*/ false /* bool State*/);
				pNMEA2000->SendMsg(N2kMsg);
				N2kTxCounter = N2kTxCounter + 1;
			}
			else if (msgNumeric == 24) {
				// 129809 AIS Class B "CS" Static Data, Part A	AIS VHF message 24
				// !AIVDM,1,1,,B,H42M;bh4hTq@E8VoT0000000003,2*3F
				// 129810 AIS Class B "CS" Static Data, Part B	AIS VHF message 24
				// !AIVDM,1,1,,B,H42M;blti1hhllqD31jmni00g0=0,0*52
				Serial.print("msgType="); Serial.println(msgType);
				Serial.print("msgNumeric="); Serial.println(msgNumeric);
				Serial.print("part_no="); Serial.println(part_no);
				Serial.print("mmsi="); Serial.println(mmsi);

				if (part_no == 0) {
					// Part A
					shipname = (char *)ais_msg.get_shipname();

					Serial.print("shipname="); Serial.println(shipname);

					SetN2kPGN129809(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, shipname);
					pNMEA2000->SendMsg(N2kMsg);
					N2kTxCounter = N2kTxCounter + 1;
				}

				if (part_no == 1) {
					// Part B

					callsign = (char *)ais_msg.get_callsign();
					shiptype = ais_msg.get_shiptype();
					vendorid = (char *)ais_msg.get_vendorid();
					to_port = ais_msg.get_to_port();
					to_starboard = ais_msg.get_to_starboard();
					to_bow = ais_msg.get_to_bow();
					to_stern = ais_msg.get_to_stern();
					mothership_mmsi = ais_msg.get_mothership_mmsi();

					Serial.print("callsign="); Serial.println(callsign);
					Serial.print("shiptype="); Serial.println(shiptype);
					Serial.print("vendorid="); Serial.println(vendorid);
					Serial.print("to_port="); Serial.println(to_port);
					Serial.print("to_starboard="); Serial.println(to_starboard);
					Serial.print("to_bow="); Serial.println(to_bow);
					Serial.print("to_stern="); Serial.println(to_stern);
					Serial.print("mothership_mmsi="); Serial.println(mothership_mmsi);

					SetN2kPGN129810(N2kMsg, msgType, static_cast<tN2kAISRepeat>(repeat), mmsi, shiptype, vendorid, callsign,
						to_bow+ to_stern, to_port+to_starboard, to_starboard, to_bow, mothership_mmsi);
					pNMEA2000->SendMsg(N2kMsg);
					N2kTxCounter = N2kTxCounter + 1;
				}
			}
			else {
				Serial.print("no N2K msg, msgType="); Serial.print(msgType); Serial.print(", msgNumeric="); Serial.println(msgNumeric);
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
