/*
N2kDataToNMEA0183.cpp

Copyright (c) 2015-2018 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "N2kDataToNMEA0183.h"
#include "N2kMessages.h"
#include "NMEA0183Messages.h"

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

extern Stream* N2kHandlersDebugStream;
extern unsigned long NMEA0183TxCounter;
extern unsigned long NMEA0183RxCounter;
extern unsigned long N2kTxCounter;
extern unsigned long N2kRxCounter;

extern tNMEA0183 NMEA0183;
extern tNMEA0183 NMEA0183_1;
extern tNMEA0183 NMEA0183_3;

extern tNMEA2000 &NMEA2000;

tNMEA0183Msg NMEA0183MsgDBT;
tNMEA0183Msg NMEA0183MsgVHW;

/**
 * @brief Convert Double to ddmm.
 */
double DoubleToddmm(double val) {
	if (val != NMEA0183DoubleNA) {
		double intpart;
		val = modf(val, &intpart);
		val = intpart * 100 + val * 60;
	}

	return val;
}

struct tN2KDebugHandler {
	const unsigned long PGN;
	const char* N2KMsg;
};

tN2KDebugHandler N2KDebugMsgs[] = {
	{  60928UL, "ISO Address Claim" },
	{  65359UL, "Pilot Heading (Raymarine)" },
	{  65379UL, "Pilot Mode (Raymarine)" },
	{  65384UL, "Heartbeat (Raymarine)" },
	{ 126720UL, "Proprietary Message" },
	{ 126993UL, "Heartbeat" },
	{ 126992UL, "System Time" },
	{ 127233UL, "Man Overboard Notification(MOB)" },
	{ 127237UL, "Heading/Track Control" },
	{ 127245UL, "Rudder" },
	{ 127250UL, "Vessel Heading" },
	{ 127251UL, "Rate of Turn" },
	{ 127257UL, "Vessel Attitude" },
	{ 127258UL, "Magnetic Variation" },
	{ 127488UL, "Engine Parameters, Rapid Update" },
	{ 127489UL, "Engine Parameters, Dynamic" },
	{ 127493UL, "Transmission Parameters, Dynamic" },
	{ 127505UL, "Fluid Level" },
	{ 127508UL, "Battery Status" },
	{ 128259UL, "Speed, Water referenced" },
	{ 128267UL, "Water Depth" },
	{ 128275UL, "Distance Log" },
	{ 129025UL, "Position, Rapid Update" },
	{ 129026UL, "COG & SOG, Rapid Update" },
	{ 129029UL, "GNSS Position Data" },
	{ 129033UL, "Local Time Offset" },
	{ 129038UL, "AIS Class A Position Report" },
	{ 129039UL, "AIS Class B Position Report" },
	{ 129040UL, "AIS Class B Extended Position Report" },
	{ 129041UL, "AIS Aids to Navigation(AtoN) Report" },
	{ 129044UL, "Datum" },
	{ 129283UL, "Cross Track Error" },
	{ 129284UL, "Navigation Data" },
	{ 129285UL, "Navigation — Route/WP information" },
	{ 129291UL, "Set & Drift, Rapid Update" },
	{ 129539UL, "GNSS DOPs" },
	{ 129540UL, "GNSS Sats in View" },
	{ 129793UL, "AIS UTC and Date Report" },
	{ 129794UL, "AIS Class A Static and Voyage Related Data" },
	{ 129798UL, "AIS SAR Aircraft Position Report" },
	{ 129809UL, "AIS Class B ""CS"" Static Data Report, Part A" },
	{ 129810UL, "AIS Class B ""CS"" Static Data Report, Part B" },
	{ 130066UL, "Route and WP Service — Route/WP, List Attributes" },
	{ 130067UL, "Route and WP Service — Route, WP Name&Position" },
	{ 130074UL, "Route and WP Service — WP List, WP Name&Position" },
	{ 130306UL, "Wind Data" },
	{ 130310UL, "Environmental Parameters" },
	{ 130311UL, "Environmental Parameters" },
	{ 130312UL, "Temperature" },
	{ 130313UL, "Humidity" },
	{ 130314UL, "Actual Pressure" },
	{ 130316UL, "Temperature, Extended Range" },
	{ 130323UL, "Meteorological Station Data" },
	{ 130577UL, "Direction Data" },
	{ 130880UL, "Proprietary Message" },
	{ 130916UL, "Proprietary Message" },
	{ 130945UL, "Proprietary Message" },
	{ 0,0 }
};

void tN2kDataToNMEA0183::HandleMsg(const tN2kMsg &N2kMsg) {
	int i;
	
//	Serial.println("N2kRxCounter+1");
	N2kRxCounter = N2kRxCounter + 1;
	
	if (N2kHandlersDebugStream != 0) {
		N2kHandlersDebugStream->print("N2k message parsed: ");
		N2kHandlersDebugStream->print(N2kMsg.PGN);

		// Find debug msg
		for (i = 0; (N2KDebugMsgs[i].PGN != 0 && N2KDebugMsgs[i].PGN != N2kMsg.PGN); i++);

		if (N2KDebugMsgs[i].PGN != 0) {
			N2kHandlersDebugStream->print(" - ");
			N2kHandlersDebugStream->println(N2KDebugMsgs[i].N2KMsg);
		}
    }
//#define debug
#ifdef debug
	Serial.print("N2k message parsed: ");
	Serial.print(N2kMsg.PGN);

	// Find debug msg
	for (i = 0; (N2KDebugMsgs[i].PGN != 0 && N2KDebugMsgs[i].PGN != N2kMsg.PGN); i++);

	if (N2KDebugMsgs[i].PGN != 0) {
		Serial.print(" - ");
		Serial.println(N2KDebugMsgs[i].N2KMsg);
	}
#endif

	switch (N2kMsg.PGN) {
		case 127250UL: HandleHeading(N2kMsg); break;   // -> HDG
	    case 127258UL: HandleVariation(N2kMsg); break; // store variation
		case 128259UL: HandleBoatSpeed(N2kMsg); break; // -> VHW
	    case 128267UL: HandleDepth(N2kMsg); break;     // -> DBT
		case 129025UL: HandlePosition(N2kMsg); break;  // -> GLL
	    case 129026UL: HandleCOGSOG(N2kMsg); break;    // -> VTG
		case 129029UL: HandleGNSS(N2kMsg); break;      // -> GGA, GLL
		case 129283UL: HandleXTE(N2kMsg); break;       // -> XTE
		case 130306UL: HandleWindSpeed(N2kMsg); break; // -> MWV, MWD
		case 126996UL: HandleSysTime(N2kMsg); break;   // -> ZDA
		case 128275UL: HandleLog(N2kMsg); break;       // -> VLW
		case 127245UL: HandleRudder(N2kMsg); break;    // -> RSA
		case 130311UL: HandleEnv(N2kMsg); break;       // -> XDR
		case  60928UL: break;                          // ISO Address Claim
		case 126993UL: break;                          // Heartbeat

#ifdef debug
		default:
			Serial.print("NMEA2000 message parsed, no Handler found: ");
			Serial.print(N2kMsg.PGN);
			// Find debug msg
			for (i = 0; (N2KDebugMsgs[i].PGN != 0 && N2KDebugMsgs[i].PGN != N2kMsg.PGN); i++);

			if (N2KDebugMsgs[i].PGN != 0) {
				Serial.print(" - ");
				Serial.print(N2KDebugMsgs[i].N2KMsg);
			}
			Serial.println("");
#endif
		}
}

/*
Conversions Between NMEA 2000 and NMEA 0183

NMEA 2000 PGN											NMEA 0183						Comment
----------------------------------------------------------------------------------------------------------------------------
126992 System Time										ZDA, GLL						See also PGN 129033
127233 Man Overboard Notification (MOB)					MOB
127237 Heading/Track Control							APB								Use PGN 129284, 129283 if possible
*127245 Rudder											RSA								Two rudders supported
*127250 Vessel Heading									HDG, HDM, HDT					See note (4) => HDG
127251 Rate of Turn										ROT
*127258 Magnetic Variation — See note (4)
127488 Engine Parameters, Rapid Update					RPM, XDR, DIN, PGN				See note (6)
127489 Engine Parameters, Dynamic						XDR, DIN						See note (6)
127493 Transmission Parameters, Dynamic					DIN, PGN						See note (6)
127505 Fluid Level										DIN, PGN						See note (6)
127508 Battery Status									DIN, PGN						See note (6)
*128259 Speed, Water referenced							VHW								Also may be used in RMC, VTG => VHW
*128267 Water Depth										DBT, DBS, DPT					DBS, DPT are off in factory settings => DBT
*128275 Distance Log									VLW
*129025 Position, Rapid Update							GLL								Also use PGN 126992 or 129029
*129026 COG & SOG, Rapid Update							VTG								Also used in RMC => VTG
*129029 GNSS Position Data								GGA, GLL, RMC, ZDA				See also PGN 129033  => GGA, RMC
129033 Local Time Offset								—								Time offset is used in ZDA
129044 Datum											DTM
129283 Cross Track Error								XTE
129284 Navigation Data									RMB								Use 129283, 129029 if possible
129285 Navigation — Route/WP information				—								Waypoint names from this message are used in RMB and APB sentences
129291 Set & Drift, Rapid Update						VDR
129539 GNSS DOPs										GSA								PGN 129540 is also required
129540 GNSS Sats in View								GSV
130066 Route and WP Service — Route/WP, List Attributes	RTE								Use waypoints from 130067
130067 Route and WP Service — Route, WP Name & Position WPL
130074 Route and WP Service — WP List, WP Name&Position WPL
*130306 Wind Data										MWD, MWV						See note (3). Also used in MDA. => MWV
130310 Environmental Parameters							XDR, MTW, MDA					See note (1), (5)
*130311 Environmental Parameters						XDR, MTW, MDA					See notes (1), (2), (5)
130312 Temperature										XDR, MTW, MDA					See notes (1), (2), (5)
130313 Humidity											XDR, MDA						See notes (1), (2), (5)
130314 Actual Pressure									XDR, MDA						See notes (1), (2), (5)
130316 Temperature, Extended Range						XDR, MTW, MDA					See notes (1), (2), (5)
129038 AIS Class A Position Report						VDM, VDO						AIS VHF messages 1, 2 and 3
129039 AIS Class B Position Report						VDM, VDO						AIS VHF message 18
129040 AIS Class B Extended Position Report				VDM, VDO						AIS VHF message 19
129041 AIS Aids to Navigation (AtoN) Report				VDM, VDO						AIS VHF message 21
129793 AIS UTC and Date Report							VDM, VDO						AIS VHF messages 4 and 11
129794 AIS Class A Static and Voyage Related Data		VDM, VDO						AIS VHF message 5
129798 AIS SAR Aircraft Position Report					VDM, VDO						AIS VHF message 9
129809 AIS Class B "CS" Static Data Report, Part A		VDM, VDO						AIS VHF message 24
129810 AIS Class B "CS" Static Data Report, Part B		VDM, VDO						AIS VHF message 24

Note (1): Air, dew point, inside (saloon), water and exhaust gas temperature, inside and outside humidity, barometric pressure are supported.
Note (2): Only messages with data instance 0 are converted.
Note (3): Devices with factory settings perform conversion from true to apparent wind. The MWV sentence is sent twice (one for apparent wind and one for true). See VI.11 for details.
Note (4): Magnetic variation is used in RMC, HDT, HDG, VDR, VHW, VTG. Priority of variation PGNs: 127250, 127258, 65311.
Note (5): MDA is sent only when air, dew point or water temperature, or barometric pressure or outside humidity are available. Also contains wind speed and direction.
Note (6): DIN and PGN are wrap NMEA 2000 messages according SeaSmart (v1.6.0) and MiniPlex (v2.0) specifications. Engine revolutions, boost pressure, coolant temperature, hours, fuel rate, 
          alternator voltage are also transmitted in XDR sentence. DIN, PGN and XDR sentences are off in the factory settings (see VI.3)
		
* PGN currently suported

*/

#define UpdatePeriod 1000

//*****************************************************************************
void tN2kDataToNMEA0183::Update() {

	static unsigned long Updated = millis();

	if (Updated + UpdatePeriod<millis()) {
		Updated = millis();
		if (N2kHandlersDebugStream != 0) { N2kHandlersDebugStream->println("N2k message Update(), SendRMC() and Send DBT"); }

		//SendRMC();                    // -> RMC

		//SendMessage(NMEA0183MsgDBT);
		NMEA0183TxCounter = NMEA0183TxCounter + 1;
		//		Serial1.println("DBT message sent.");

		//SendMessage(NMEA0183MsgVHW);
		NMEA0183TxCounter = NMEA0183TxCounter + 1;
		//		Serial1.println("VHW message sent.");
	}

/*
	if ( LastHeadingTime+2000 < millis() ) {
		Heading = N2kDoubleNA;
	}
	if ( LastCOGSOGTime+2000<millis() ) {
		COG=N2kDoubleNA;
		SOG=N2kDoubleNA;
	}
	if ( LastPositionTime+4000<millis() ) {
		Latitude=N2kDoubleNA;
		Longitude=N2kDoubleNA;
	}
*/
}

//*****************************************************************************
void tN2kDataToNMEA0183::SendMessage(const tNMEA0183Msg &NMEA0183Msg) {
	if (pNMEA0183 != 0) {
		pNMEA0183->SendMessage(NMEA0183Msg);
	}
	if (SendNMEA0183MessageCallback != 0) {
		SendNMEA0183MessageCallback(NMEA0183Msg);
	}
}

/*****************************************************************************
ZDA - Time & Date - UTC, day, month, year and local time zone
       1         2  3  4    5  6  7
       |         |  |  |    |  |  |
$--ZDA,hhmmss.ss,xx,xx,xxxx,xx,xx*hh<CR><LF>
Field Number:
1) Local zone minutes description, same sign as local hours
2) Local zone description, 00 to +- 13 hours
3) Year
4) Month, 01 to 12
5) Day, 01 to 31
6) Universal Time Coordinated (UTC)
7) Checksum

*/

void tN2kDataToNMEA0183::HandleSysTime(const tN2kMsg &N2kMsg) {
/*
	unsigned char SID;
	tN2kHeadingReference ref;
	double Deviation;
	double _Variation;
	tNMEA0183Msg NMEA0183Msg;

	if ( ParseN2kHeading(N2kMsg, SID, Heading, Deviation, _Variation, ref) ) {
		if ( ref==N2khr_magnetic ) {
			if ( !N2kIsNA(_Variation) ) Variation=_Variation; // Update Variation
			if ( !N2kIsNA(Heading) && !N2kIsNA(Variation) ) Heading-=Variation;
		}
		LastHeadingTime=millis();
		if ( NMEA0183SetHDG(NMEA0183Msg,Heading, NMEA0183DoubleNA, Variation) )
		{
			SendMessage(NMEA0183Msg);
			NMEA0183TxCounter = NMEA0183TxCounter+1;
		}
	}
*/
}

/*****************************************************************************
HDG - Heading - Deviation & Variation
       1   2   3 4   5 6
       |   |   | |   | |
$--HDG,x.x,x.x,a,x.x,a*hh<CR><LF>
Field Number:
1) Magnetic Sensor heading in degrees
2) Magnetic Deviation, degrees
3) Magnetic Deviation direction, E = Easterly, W = Westerly
4) Magnetic Variation degrees
5) Magnetic Variation direction, E = Easterly, W = Westerly
6) Checksum


HDT - Heading - True
       1   2 3
       |   | |
$--HDT,x.x,T*hh<CR><LF>
Field Number:
1) Heading Degrees, true
2) T = True
3) Checksum

*/

void tN2kDataToNMEA0183::HandleHeading(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kHeadingReference ref;
	double Deviation;
	double _Variation;
	tNMEA0183Msg NMEA0183Msg;

#define HDTUpdatePeriod 5000

	static unsigned long HDTUpdated = millis();

//	Serial1.print("N2K Heading message parsed, source: ");
//	Serial1.println(N2kMsg.Source);
	if (N2kMsg.Source == 38) {
		if (ParseN2kHeading(N2kMsg, SID, Heading, Deviation, _Variation, ref)) {
			if (ref == N2khr_magnetic) {
				if (!N2kIsNA(_Variation)) Variation = _Variation; // Update Variation
				if (!N2kIsNA(Heading) && !N2kIsNA(Variation)) Heading -= Variation;
			}
			LastHeadingTime = millis();
			if (NMEA0183SetHDG(NMEA0183Msg, Heading, NMEA0183DoubleNA, Variation)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
			if (NMEA0183SetHDT(NMEA0183Msg, Heading)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
				if (HDTUpdated + HDTUpdatePeriod < millis()) {
					HDTUpdated = millis();
//					NMEA0183_1.SendMessage(NMEA0183Msg);     // to the NMEA0183 out1 stream (USB an TX1/RX1)
				}
			}
		}
	}
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleVariation(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kMagneticVariation Source;

	ParseN2kMagneticVariation(N2kMsg,SID,Source,DaysSince1970,Variation);
}

/*****************************************************************************
VHW - Water speed and heading
        1   2 3   4 5   6 7   8 9 
        |   | |   | |   | |   | | 
 $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
 Field Number:  
  1) Degress True 
  2) T = True 
  3) Degrees Magnetic 
  4) M = Magnetic 
  5) Knots (speed of vessel relative to the water) 
  6) N = Knots 
  7) Kilometers (speed of vessel relative to the water) 
  8) K = Kilometers 
  9) Checksum

*/

void tN2kDataToNMEA0183::HandleBoatSpeed(const tN2kMsg &N2kMsg) {
//	VHW	128259 Speed, Water referenced

	unsigned char SID;
	double WaterReferenced;
	double GroundReferenced;
	tN2kSpeedWaterReferenceType SWRT;

	tN2kMsg N2kMsg128259;

	if ( ParseN2kBoatSpeed(N2kMsg,SID,WaterReferenced,GroundReferenced,SWRT) ) {
		double MagneticHeading=( !N2kIsNA(Heading) && !N2kIsNA(Variation)?Heading+Variation: NMEA0183DoubleNA);

		// if boat speed fails, due to no turning the paddle wheel, take SOG instad
		if (WaterReferenced < 0.5) { WaterReferenced = SOG; }

		if ( NMEA0183SetVHW(NMEA0183MsgVHW,Heading,MagneticHeading,WaterReferenced) ) {
			SendMessage(NMEA0183MsgVHW);
			NMEA0183TxCounter = NMEA0183TxCounter + 1;

			SetN2kPGN128259(N2kMsg128259, SID, WaterReferenced, GroundReferenced, SWRT);
			NMEA2000.SendMsg(N2kMsg128259);
			N2kTxCounter = N2kTxCounter + 1;
		}
	}
}

/*****************************************************************************
DBT - Depth below transducer
        1   2 3   4 5   6 7 
        |   | |   | |   | | 
 $--DBT,x.x,f,x.x,M,x.x,F*hh<CR><LF>
 Field Number:  
  1) Depth, feet 
  2) f = feet 
  3) Depth, meters 
  4) M = meters 
  5) Depth, Fathoms 
  6) F = Fathoms 
  7) Checksum

*/

void tN2kDataToNMEA0183::HandleDepth(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double DepthBelowTransducer;
	double Offset;
	double Range;

	if ( ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset,Range) ) {
		if ( NMEA0183SetDBT(NMEA0183MsgDBT,DepthBelowTransducer) ) {
			SendMessage(NMEA0183MsgDBT);
			NMEA0183TxCounter = NMEA0183TxCounter + 1;
		}
	}
}

/*****************************************************************************

*/
void tN2kDataToNMEA0183::HandleXTE(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kXTEMode XTEMode;
	bool NavigationTerminated;
	double XTE;

	if (ParseN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE)) {
		tN2kMsg N2kMsgXTE;
		SetN2kXTE(N2kMsgXTE, 1, N2kxtem_Autonomous, false, XTE);
		NMEA2000.SendMsg(N2kMsgXTE);
		N2kTxCounter = N2kTxCounter + 1;
	}
}

/*****************************************************************************
MWV - Wind Speed and Angle

        1   2 3   4 5 
        |   | |   | | 
 $--MWV,x.x,a,x.x,a*hh<CR><LF>

 Field Number:  
  1) Wind Angle, 0 to 360 degrees 
  2) Reference, R = Relative, T = True 
  3) Wind Speed 
  4) Wind Speed Units, K/M/N 
  5) Status, A = Data Valid 
  6) Checksum


MWD - Wind Direction and Speed
		1   2 3   4 5   6 7   8
		|   | |   | |   | |   |
 $--MWD,x.x,a,x.x,a,x.x,a,x.x,a*hh<CR><LF>

 Field Number:
  1) Wind Direction, 0.0 to 359.9 degrees
  2) Reference, R = Relative, T = True
  3) Wind Direction, 0.0 to 359.9 degrees
  4) Reference, M = Magnetic, T = True
  5) Wind Speed, knots, to the nearest 0.1 knot
  6) Wind Speed Units, K/M/N
  7) Wind speed, meters/second, to the nearest 0.1 m/s.
  8) Wind Speed Units, M = Meters/second
  9) Checksum

*/
  
void tN2kDataToNMEA0183::HandleWindSpeed(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	double WindSpeed;
	double WindAngle;
	double WindAngleDeg;
	tN2kWindReference WindReference;

	tN2kMsg N2kMsg130306;

	if ( ParseN2kWindSpeed(N2kMsg,SID,WindSpeed,WindAngle,WindReference) ) {
		tNMEA0183Msg NMEA0183Msg;


		WindAngleDeg = RadToDeg(WindAngle);
		WindAngleDeg = WindAngleDeg - 20;
		if ( WindAngleDeg > 360 ) {
			WindAngleDeg = WindAngleDeg - 360;
		}
		if (WindAngleDeg < 0) {
			WindAngleDeg = WindAngleDeg + 360;
		}

		SetN2kPGN130306(N2kMsg130306, SID, WindSpeed, DegToRad(WindAngleDeg), WindReference);
		NMEA2000.SendMsg(N2kMsg130306);
		N2kTxCounter = N2kTxCounter + 1;

		if ( WindReference==N2kWind_Apprent ) {
			if ( NMEA0183SetMWV(NMEA0183Msg,RadToDeg(WindAngle),NMEA0183Wind_Apparent,WindSpeed) ) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}

		if ( WindReference==N2kWind_True_North ) {
			if ( NMEA0183SetMWV(NMEA0183Msg,RadToDeg(WindAngle),NMEA0183Wind_True,WindSpeed) ) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
			if (NMEA0183SetMWD(NMEA0183Msg, RadToDeg(WindAngle), WindSpeed)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}
	}
}

/*****************************************************************************
GLL - Geographic Position - Latitude/Longitude
	   1       2 3        4 5         6 7
	   |       | |        | |         | |
$--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
 Field Number:
  1) Latitude
  2) N or S (North or South)
  3) Longitude
  4) E or W (East or West)
  5) Universal Time Coordinated (UTC)
  6) Status A - Data Valid, V - Data Invalid , P - Precise
  7) Checksum

*/

void tN2kDataToNMEA0183::HandlePosition(const tN2kMsg &N2kMsg) {
	tNMEA0183Msg NMEA0183Msg;

	if (N2kMsg.Source == 38) {
		if (ParseN2kPGN129025(N2kMsg, Latitude, Longitude)) {
			LastPositionTime = millis();
			if (N2kHandlersDebugStream != 0) {
				N2kHandlersDebugStream->print("Pos from 129025: ");
				N2kHandlersDebugStream->print(Latitude, 8);
				N2kHandlersDebugStream->print(" ");
				N2kHandlersDebugStream->println(Longitude, 8);
			}

//			Latitude = 3723.40002;
//			Longitude = 2700.00003;

			if (NMEA0183SetGLL(NMEA0183Msg, NMEA0183DoubleNA, Latitude, Longitude, 0)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}

/*
			Serial1.println("SendGLL");
			Serial1.print("Pos from 129025: ");
			Serial1.print(Latitude, 4);
			Serial1.print(" ");
			Serial1.println(Longitude, 4);
*/
		}
	}
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

//*****************************************************************************
void tN2kDataToNMEA0183::HandleCOGSOG(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kHeadingReference HeadingReference;
	tNMEA0183Msg NMEA0183Msg;

	if (N2kMsg.Source == 38) {
		if (ParseN2kCOGSOGRapid(N2kMsg, SID, HeadingReference, COG, SOG)) {
			LastCOGSOGTime = millis();
			double MCOG = (!N2kIsNA(COG) && !N2kIsNA(Variation) ? COG - Variation : NMEA0183DoubleNA);
			if (HeadingReference == N2khr_magnetic) {
				MCOG = COG;
				if (!N2kIsNA(Variation)) {
					COG -= Variation;
				}
			}
			if (NMEA0183SetVTG(NMEA0183Msg, COG, MCOG, SOG)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}
	}
}

/*****************************************************************************
GGA - Global Positioning System Fix Data, Time, Position and fix related data fora GPS receiver.
		1         2       3 4        5 6 7  8   9  10 11 12 13  14   15
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

******************************************************************************
GLL - Geographic Position - Latitude/Longitude
	   1       2 3        4 5         6 7
	   |       | |        | |         | |
$--GLL,llll.ll,a,yyyyy.yy,a,hhmmss.ss,A*hh<CR><LF>
 Field Number:
  1) Latitude
  2) N or S (North or South)
  3) Longitude
  4) E or W (East or West)
  5) Universal Time Coordinated (UTC)
  6) Status A - Data Valid, V - Data Invalid , P - Precise
  7) Checksum

*/

//*****************************************************************************
void tN2kDataToNMEA0183::HandleGNSS(const tN2kMsg &N2kMsg) {
	unsigned char SID;
	tN2kGNSStype GNSStype;
	tN2kGNSSmethod GNSSmethod;
	unsigned char nSatellites;
	double HDOP;
	double PDOP;
	double GeoidalSeparation;
	unsigned char nReferenceStations;
	tN2kGNSStype ReferenceStationType;
	uint16_t ReferenceStationID;
	double AgeOfCorrection;
	int GPSQualityIndicator = 0;
	tNMEA0183Msg NMEA0183Msg;

	if (N2kMsg.Source == 38) {
		if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight, Latitude, Longitude, Altitude, GNSStype, GNSSmethod,
			nSatellites, HDOP, PDOP, GeoidalSeparation, nReferenceStations, ReferenceStationType, ReferenceStationID, AgeOfCorrection)) {
			LastPositionTime = millis();

//			Latitude = 12.00;
//			Longitude = 56.00;

			if (N2kHandlersDebugStream != 0) {
				N2kHandlersDebugStream->print("Pos from 129029: ");
				N2kHandlersDebugStream->print(Latitude, 8);
				N2kHandlersDebugStream->print(" ");
				N2kHandlersDebugStream->println(Longitude, 8);
			}

			if (NMEA0183SetGGA(NMEA0183Msg, SecondsSinceMidnight, Latitude, Longitude, GPSQualityIndicator, nSatellites, HDOP, Altitude, GeoidalSeparation, AgeOfCorrection, ReferenceStationID)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
			if (NMEA0183SetGLL(NMEA0183Msg, SecondsSinceMidnight, Latitude, Longitude, 0)) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}

			Serial1.println("SendGGA/GLL");
			Serial1.print("Pos from 129029: ");
			Serial1.print(Latitude, 4);
			Serial1.print(" ");
			Serial1.println(Longitude, 4);
		}
	}
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

//*****************************************************************************
void tN2kDataToNMEA0183::SendRMC() {
//    if ( NextRMCSend<=millis() && !N2kIsNA(Latitude) && ( SecondsSinceMidnight > 0 ) ) {
		tNMEA0183Msg NMEA0183Msg;
		if ( NMEA0183SetRMC(NMEA0183Msg,SecondsSinceMidnight,Latitude,Longitude,COG,SOG,DaysSince1970,Variation) ) {
			SendMessage(NMEA0183Msg);
			Serial1.println("RMC message sent.");
			NMEA0183TxCounter = NMEA0183TxCounter + 1;
		}
		SetNextRMCSend();
//    }
}

void tN2kDataToNMEA0183::HandleLog(const tN2kMsg &N2kMsg) {

	if (ParseN2kDistanceLog(N2kMsg, DaysSince1970, SecondsSinceMidnight, Log, TripLog)) {

		tNMEA0183Msg NMEA0183Msg;

		if (!NMEA0183Msg.Init("VLW", "GP")) return;
		if (!NMEA0183Msg.AddDoubleField(Log / 1852.0)) return;
		if (!NMEA0183Msg.AddStrField("N")) return;
		if (!NMEA0183Msg.AddDoubleField(TripLog / 1852.0)) return;
		if (!NMEA0183Msg.AddStrField("N")) return;

		SendMessage(NMEA0183Msg);
	}
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleRudder(const tN2kMsg &N2kMsg) {

	double RudderPosition;
	unsigned char Instance;
	tN2kRudderDirectionOrder RudderDirectionOrder;
	double AngleOrder;

	if (ParseN2kRudder(N2kMsg, RudderPosition, Instance, RudderDirectionOrder, AngleOrder)) {

		tNMEA0183Msg NMEA0183Msg;

		if (!NMEA0183Msg.Init("RSA", "GP")) return;
		if (!NMEA0183Msg.AddDoubleField(RudderPosition*radToDeg)) return;
		if (!NMEA0183Msg.AddStrField("A")) return;
		if (!NMEA0183Msg.AddDoubleField(0.0)) return;
		if (!NMEA0183Msg.AddStrField("A")) return;

		SendMessage(NMEA0183Msg);
	}
}

//*****************************************************************************
void tN2kDataToNMEA0183::HandleEnv(const tN2kMsg &N2kMsg) {

	unsigned char SID;
	tN2kTempSource TempSource;
	double Temperature;
	tN2kHumiditySource HumiditySource;
	double Humidity;
	double AtmosphericPressure;

	if (ParseN2kEnvironmentalParameters(N2kMsg, SID, TempSource, Temperature, HumiditySource, Humidity, AtmosphericPressure)) {
		tNMEA0183Msg NMEA0183Msg;
/*
		Serial.println("HandleEnv");
		Serial.println(TempSource);
		Serial.println(AtmosphericPressure/100000.0);
*/
		if (TempSource == N2kts_SeaTemperature) {
//			if (NMEA0183SetXDR(NMEA0183Msg, 1, Temperature, "SEA")) {
			if (NMEA0183SetMTW(NMEA0183Msg, Temperature)) {
					SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}
		if (TempSource == N2kts_OutsideTemperature) {
			if (NMEA0183SetXDR(NMEA0183Msg, 1, Temperature, "AIR")) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}
		if (AtmosphericPressure != NMEA0183DoubleNA) {
			if (NMEA0183SetXDR(NMEA0183Msg, 2, AtmosphericPressure, "Barometer")) {
				SendMessage(NMEA0183Msg);
				NMEA0183TxCounter = NMEA0183TxCounter + 1;
			}
		}

	}
}
