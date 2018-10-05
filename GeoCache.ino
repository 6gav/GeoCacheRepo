/******************************************************************************

GeoCache Hunt Project (GeoCache.ino)

This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.  Be sure you compile in "RELEASE" mode.

Complete the team information below before submitting code for grading.

Team Number: ?

Team Members: ?

NOTES: 
You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.

The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).  
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.

*******************************************************************************

Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By 
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and 
manufactured the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline

Following are the results calculated from above GPS GPRMC message (current
location) to the provided GEOLAT0/GEOLON0 tree (target location).  Your 
results should be nearly identical, if not exactly the same.

degMin2DecDeg() LAT_2307.1256_N = 23.118757 decimal degrees
degMin2DecDeg() LON_12016.4438_E = 120.274055 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999652 degrees
Relative target bearing to the tree = 217.519650 degrees

Note that the above calculated results display 6 decimial places to the right.
This is accomplished setting the second String class parameter to 6, as in the 
following example:

String sstr = String(float, 6);

******************************************************************************/

/*
Configuration settings.

These defines makes it easy for you to enable/disable certain 
code during the development and debugging cycle of this project.
There may not be sufficient room in the PROGRAM or DATA memory to
enable all these libraries at the same time.  You must have NEO_ON, 
GPS_ON and SDC_ON during the actual GeoCache Flag Hunt on Finals Day.
*/
#define NEO_ON 0		// NeoPixel Shield (0=OFF, 1=ON)
#define LOG_ON 1		// Serial Terminal Logging (0=OFF, 1=ON)
#define SDC_ON 0		// Secure Digital Card (0=OFF, 1=ON)
#define GPS_ON 0		// 0 = simulated GPS message, 1 = actual GPS message

// define pin usage
#define SWITCH_BUTTON 0
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive

#define GPS_BUFSIZ	96	// max size of GPS char buffer

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);
#endif

#if SDC_ON
#include <SD.h>
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.

On GeoCache day, you will be given waypoints in Decimal Degrees format for 4x
flags located on Full Sail campus.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#endif

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *cind, char *ccor)

Input:
	cind = char string pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
	ccor = char string pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate

Return:
	Decimal degrees coordinate.
	
**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	float degrees = 0.0;
	
	// add code here
	
	return(degrees);
}

/************************************************** 
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
	flat1, flon1 = GPS latitude and longitude coordinate in decimal degrees
	flat2, flon2 = Target latitude and longitude coordinate in decimal degrees

Return:
	distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
uint32_t eRadius = 6371;
float radLat1;
float radLat2;
float deltaLat;
float deltaLon;
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	radLat1 = flat1 * DEG_TO_RAD;
	radLat2 = flat2 * DEG_TO_RAD;
	deltaLat = (flat2 - flat1) * DEG_TO_RAD;
	deltaLon = (flon2 - flon1) * DEG_TO_RAD;

	float sinDeltaLat = sin(deltaLat / 2);
	float sinDeltaLon = sin(deltaLon / 2);

	float tempA = sinDeltaLat + cos(radLat1) * cos(radLat2) * sin(sinDeltaLon);

	float tempC = 2 * atan2(sqrt(tempA), sqrt(1-tempA));

	return (eRadius * tempC);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
	flat1, flon1 = course over ground latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
	angle in decimal degrees from magnetic north (NOTE: arc tangent returns range of -pi/2 to +pi/2)
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	
	// add code here
	float y = sin(flon2 - flon1) * cos(flat2);
	float x = cos(flat1) * sin(flat2) - sin(flat1) * cos(flat2) * cos(flon2 - flon1);

	
	return((atan2(y, x) * RAD_TO_DEG));
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
/*
Sets target number, heading and distance on NeoPixel Display

NOTE: Target number, bearing and distance parameters used
by this function do not need to be passed in, since these
parameters are in global data space.

*/
void setNeoPixel(void)
{
	// add code here
}

#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message.

char* getGpsMessage(void)

Side affects:
Message is placed in local static char buffer.

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/
char* getGpsMessage(void)
{
	bool rv = false;
	static uint8_t x = 0;
	static char cstr[GPS_BUFSIZ];

	// get nmea string
	while (gps.peek() != -1)
	{
		// reset or bound cstr
		if (x == 0) memset(cstr, 0, sizeof(cstr));
		else if (x >= (GPS_BUFSIZ - 1)) x = 0;
		
		// read next char
		cstr[x] = gps.read();

		// toss invalid or undesired message
		if ((x >= 3) && (cstr[0] != '$') && (cstr[3] != 'R'))
		{
			x = 0;
			break;
		}

		// if end of message received (sequence is \r\n)
		if (cstr[x] == '\n')
		{
			// nul terminate char buffer (before \r\n)
			cstr[x - 1] = 0;

			// if checksum not found
			if (cstr[x - 4] != '*')
			{
				x = 0;
				break;
			}

			// convert hex checksum to binary
			uint8_t isum = strtol(&cstr[x - 3], NULL, 16);

			// reverse checksum
			for (uint8_t y = 1; y < (x - 4); y++) isum ^= cstr[y];

			// if invalid checksum
			if (isum != 0)
			{
				x = 0;
				break;
			}

			// else valid message
			rv = true;
			x = 0;
			break;
		}

		// increment buffer position
		else x++;

		// software serial must breath, else miss incoming characters
		delay(1);
	}

	if (rv) return(cstr);
	else return(nullptr);
}

#else
	
/*
Get simulated GPS message (same as message described at top of this *.ino file)

char* getGpsMessage(void)

Side affects:
Message is place in local static char buffer

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/
char* getGpsMessage(void)
{
	static char cstr[GPS_BUFSIZ];
	static uint32_t timestamp = 0;

	uint32_t timenow = millis();

	// provide message every second
	if (timestamp >= timenow) return(nullptr);

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	timestamp = timenow + 1000;

	return(cstr);
}
#endif

void setup(void)
{
	pinMode(SWITCH_BUTTON, INPUT_PULLUP);

	#if LOG_ON
	// init serial interface
	Serial.begin(115200);
	#endif	

	#if NEO_ON
	// init NeoPixel Shield
	#endif	

	#if SDC_ON
	/*
	Initialize the SecureDigitalCard and open a numbered sequenced file
	name "MyMapN.txt" for storing your coordinates, where N is the 
	sequential number of the file.  The filename can not be more than 8
	chars in length (excluding the ".txt").  Do not close this file, 
	but remains open.  The file automatcially closes when the program
	is reloaded or the board is reset.
	*/
	#endif

	#if GPS_ON
	// enable GPS sending GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
	#endif		

	// set target button pinmode
}

void loop(void)
{
	// get GPS message
	char *cstr = getGpsMessage();

	// if valid message delivered (happens once a second)
	if (cstr)
	{
#if LOG_ON
		// print the GPRMC message
		Serial.println(cstr);	
#endif



		// check button for incrementing target index
		if (digitalRead(SWITCH_BUTTON)) {
			target++;
		}

		

		// parse required latitude, longitude and course over ground message parameters
		/*
		$GPRMC,         // GPRMC Message
		064951.000,     // utc time hhmmss.sss
		A,              // status A=data valid or V=data not valid						
		2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm)		3
		N,              // N/S Indicator N=north or S=south								4
		12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm)		5
		E,              // E/W Indicator E=east or W=west								6
		0.03,           // Speed over ground knots
		165.48,         // Course over ground (decimal degrees format ddd.dd)			8
		260406,         // date ddmmyy
		3.05,           // Magnetic variation (decimal degrees format ddd.dd)
		W,              // E=east or W=west												11
		A               // Mode A=Autonomous D=differential E=Estimated
		* 2C            // checksum
		/ r / n         // return and newline
		*/
		char* longitude;
		char* latitude;



		char* tempStr = strtok(cstr, ",");
		for (uint32_t i = 0; tempStr != NULL; i++) {
			
			switch (i)
			{
			case 3:
				latitude = tempStr;
				break;
			case 5:
				longitude = tempStr;
				break;
			default:
			
				break;
			}
			tempStr = strtok(NULL, ",");

		}

		Serial.println(longitude);
		Serial.println(latitude);

		// convert latitude and longitude degrees minutes to decimal degrees
		degMin2DecDeg(latitude, );

		// calculate destination distance
				
		// calculate destination heading
		
		// calculate relative bearing
		
		#if SDC_ON
		// write required data to SecureDigital then execute flush()
		#endif
		
		#if NEO_ON
			// set NeoPixel target display information
			setNeoPixel(target, heading, distance);
		#endif			
	}
}