
// LIBRARIES
#include <Wire.h> // For I2C
#include <EEPROM.h>
#include <glcd.h>
#include <glcd_Config.h>
#include "bitmaps/allBitmaps.h"

#include "fonts/Arial14.h"
#include "fonts/SystemFont5x7.h"
#include "fonts/Arial_bold_14.h"
#include "verdana28.h"

// CONSTANT DEFINITION
/******************* Code Customization Section *********************/
#define MAX_BUF_SIZE 127
enum { DPLL_BW_NONE, DPLL_BW_LOWEST, DPLL_BW_LOW, DPLL_BW_MED_LOW, DPLL_BW_MED, DPLL_BW_MED_HIGH, DPLL_BW_HIGH, DPLL_BW_HIGHEST};
#define DPLL_BW_NONE_VAL			0x81
#define DPLL_BW_LOWEST_VAL			0x85
#define DPLL_BW_LOW_VAL				0x89
#define DPLL_BW_MED_LOW_VAL		0x8D
#define DPLL_BW_MED_VAL				0x91
#define DPLL_BW_MED_HIGH_VAL		0x95
#define DPLL_BW_HIGH_VAL			0x99
#define DPLL_BW_HIGHEST_VAL		0x9D

#define GO_IDLE					"idle mixer"
#define GO_NOIDLE				"noidle"
#define BUT_SEL                  0
#define BUT_CCW                  1
#define BUT_CW                   2
#define SERIAL_DELAY			250
#define MAX_INPUT_BUF	256
#define USB_READ_TIMEOUT	2000 // 2 seconds
#define SCREEN_SPLASH -1
#define SCREEN_VOLUME 0
#define SCREEN_INPUTS 1
#define SCREEN_PREFS 2
#define SCREEN_IDLE 3
#define SCREEN_DAC_WAIT 4
#define SCREEN_MPD_WAIT 5

#define USE100MHZ
#define ICHO 6

#define DEFAULTATTNU 0x8C           //-50 dB this is 50x2=100 or 0x64. Sabre32 is 0 to -127dB in .5dB steps
#define MAXATTNU 0xC6               //-99dB this is 99X2=198 or 0xC6 -Can go higher but LCD shows 2 digits
#define MAX_ATTN MAXATTNU
#define MINATTNU 0x00               //-0 dB -Maximum volume is -0 dB
#define DIM 0x8C                    //-70 dB this is 70x2=140 or 0x8C. Dim volume
#define RAMP 10                     // Additional msec delay per 1 db for ramping up to volume after dim

#define VOLUPPIN 4                  // Button to increase  volume or RotEnc A terminal
#define VOLDOWNPIN 2                // Button to decrease volume or RotEnc B terminal
#define SELECTPIN 5                 // Switch to select function
#define REMOTEPIN 3                 // Pin for IR receiver (remote control)

#define INTERVAL_SAMPLE 500           // Time interval in milliseconds for refreshing the sample rate
#define INTERVAL_BOUNCE 2           // Time in milliseconds to debounce the rotary encoder
#define INTERVAL_SWITCHBOUNCE 250   // Time in milliseconds to debounce switch
#define INTERVAL_SELECT 30           // Time in sec to exit select mode when no activity

#define B 0xFF                      // The character for a completely filled box
#define A 0x20                      // The character for blank

#define I2C_BUF_ADDR 0x48
#define I2C_BUF_ADDR_2 0x49

// The following is used for parameter selection and parameter storage
// This is the order of selection when clicking the select swtich
#define VOL 0   // Volume
#define FIR 3   // FIR filter selection
#define IIR 4   // IIR filter selection
#define QTZ 6   // Quantizer selection
#define INP 1   // Input selection
#define INF 2   // Input format selection
#define DPL 5   // DPLL bandwidth setting
#define NCH 7   // Notch selection
#define PLM 8   // DPLL mode
#define OSF 9   // Oversampling
#define SRF 10  // Sample Rage format: "nominal" or "exact"

// Order of settings in the array for each input (for those saved in array)
#define FORMATVAL 0
#define FIRVAL 1
#define IIRVAL 2
#define DPLLVAL 3
#define QUANVAL 4
#define NOTCHVAL 5
#define PLMVAL 6

/* Total number of parameters to keep track for each input. The 7 listed above plus the input
	choice. This makes 8.
	OSF is not tracked  per input as it always starts with oversampling ON
	VOL is also not tracked per input as it always starts with the default volume
	SRF is also not trackec per input as it applies to all inputs
	Thus there are 11 parameters, 8 which are saved per input, and 3 are across all inputs
 */
#define MAXPARAM 8

// Number of valid choices for each parameter
#define FORMATCHO 10    // SPDIF, I2S/DSD, LJ, RJ
#define FIRCHO 2        // Fast, slow
#define IIRCHO 4        // PCM, 50k, 60k, 70k
#define DPLLCHO 8       // Lowest to Best 8 values
#define QUANCHO 6       // 6bT, 7bP, 7bT, 8bP, 8bT, 9bP
#define NOTCHCHO 6      // /4 to /64
#define PLMCHO 4        // Normal x1 (NOR), Multiplier x128 (MUL), Bypass (OFF), INV (Invert phase)
// (There is one more: ICHO which is already defined above)
// Choices for OSF and SRF are two, but we use a toggle variable. Vol selection is seprate

// VARIABLE DECLARATION

// Register variables: used for registers that have lots of options. They are initialized here
// with valid values, but will eventually be overwritten by the values in EEPROM
byte reg14 = 0xF9;  // Default value for register 14. We use a variable for reg 14 because it controls
						  // several parameters: IIR, FIR, differential whereas the other registers typically
						  // controls a single parameter.
byte reg25 = 0;     // 0= allow all settings
byte reg17L = 0x1C; // Auto SPDIF, 8-channel mode, other defaults
						  // reg17L is used for stereo and for left channel if set for MONO
#ifdef DUALMONO
byte reg17R = 0x9D; // Auto SPDIF, MONO RIGHT CHANNEL, other defaults
#endif DUALMONO

byte reg10 = 0xCE;                  // jitter ON, dacs unmute, other defaults (0xFE also I2S)

unsigned long displayMillis = 0;    // Stores last recorded time for display interval
unsigned long debounceMillis = 0;   // Stores last recorded time for switch debounce interval
unsigned long selectMillis = 0;     // Stores last recorded time for being in select mode
unsigned long sr = 0;               // Holds the value for the incoming sample rate

byte input = 0;                     // The current input to the DAC
byte currAttn = DEFAULTATTNU;      // Variable to hold the current attenuation value

byte select;                        // To record current select position (FIL, VOL, DPLL, etc)

boolean selectMode;                 // To indicate whether in select mode or not
boolean SPDIFValid;                 // To indicate whether SPDIF valid data
boolean spdifIn;                    // To indicate whether in I2S/DSD or SPDIF input format mode
boolean bypassOSF = false;          // false=no bypass; This is the default condition at startup
boolean SRExact = true;             // Display exact sample rate value; false = display nominal value
boolean primed = false;             // To indicate if dpll has been conditioned (once at startup)
boolean dimmed = false;             // To indicate dim (mute) or not
byte pulse = 0;                     // Used by the "heartbeat" display
byte Status;                        // To hold value of Sabre32 status register


// The following variables for the remote control feature
int duration;           // Duration of the IR pulse
int mask;               // Used to decode the remote signals
int c1;                 // Byte 1 of the 32-bit remote command code
int c2;                 // Byte 2 of the 32-bit remote command code
int c3;                 // Byte 3 of the 32-bit remote command code
int c4;                 // Byte 4 of the 32-bit remote command code
int IRkey;              // The unique code (Byte 3) of the remote key
int previousIRkey;      // The previous code (used for repeat)

// The array holds the parameters for each input
// (The current input selection is recorded after the array)
byte settings[ICHO][MAXPARAM];  // Array to hold parameter values

// Rotary encoder interrupt service routine
static boolean rotating = false;

int currentScreenType = SCREEN_SPLASH;
int volumeSyncRequired=false;		// this indicates that the DAC has updated its volume and must sync it to MPD.
char buffer[MAX_BUF_SIZE+1]="";
char tmp[MAX_BUF_SIZE+1]="";
int connectedToMPD=false;
byte currentVolume=0; // 0-100 percent

/* Optionally change the name of the inputs. Keep 6 characters
	Use blanks if necessary                                          */

char no0[] = "PC-BST";
char no1[] = "PC-LOW";
char no2[] = "SPDIF1";
char no3[] = "SPDIF2";
char no4[] = "SACD-1";
char no5[] = "AMNERO";



// The following 2 routine sets up the registers in the DAC at startup

void startDac1(){               // Set registers not requiring display info
	mute();
	volume_set_DAC(currentVolume);  // Reg 0 to Reg 7 Set volume registers with startup volume level
	writeSabreReg(0x0D, 0x00);  // DAC in-phase
	writeSabreReg(0x13, 0x00);  // DACB anti-phase
	writeSabreReg(0x25, 0x00);  // Use built in filter for stage 1 and stage 2
	writeSabreReg(0x0E, reg14); // Reg 14: BuffII input map, trueDiff, normal IIR and Fast rolloff
										 // Reg 14: except BuffII input map setting, the others will be
										 // redefined.

	//The code below may provide some mitigation to the white noise during silence
  #ifdef USE80MHZ
	writeSabreReg(0x10, 0x08);  // Reg 16: Turn automute loopback -only to mitigate 352.8KHz noise
	writeSabreReg(0x09, 0x10);  // Reg 9: Set automute time 4x less than default (value is in denom.)
  #endif USE80MHZ

  #ifdef DUALMONO                   // DAC registers default to stereo. Set to MONO L/R for dual MONO
	bitSet(reg17L, 0);              // Set for MONO left channel. Right ch variable is already set for MONO
	writeSabreLeftReg(0x11, reg17L);
	writeSabreRightReg(0x11, reg17R);
  #endif DUALMONO

  #ifdef TPAPHASE
	/* The outputs on each side of each MONO board will be in opposite phase. In order to do this
		the phase of the odd dacs are out of phase with the even dacs. Further, buffalo is configured
		such that (In dual mono mode) the channel on the DAC which is opposite of the selected channel
		carries the same analog signal but in anti-phase (odd dacs are the left channel;
		even dacs are the right channel)
		See http://hifiduino.wordpress.com/sabre32/ for further explaination
	 */
	writeSabreLeftReg(0x0D, 0x22);  // MONO LEFT DACx: odd dacs=in-phase; even dacs=anti-phase
	// writeSabreLeftReg(0x13,0x00);  // MONO LEFT DACBx: all dacs anti-phase with respect to DACx
	writeSabreRightReg(0x0D, 0x11); // MONO RIGHT DACx: odd dacs=anti-phase; even dacs=in-phase
	// writeSabreRightReg(0x13,0x00); // MONO RIGHT DACBx: all dacs anti-phase with respect to DACx
  #endif TPAPHASE
}

void startDac2(){
	readSettings();
	initForInput(input);
	unmute();
}

void unmute() {
	bitClear(reg10, 0);                 // Clear bit zero of reg 10: unmute DACs
	writeSabreReg(0x0A, reg10);         // Unmute DACs
}

void mute() {
	bitSet(reg10, 0);               // Set bit zero of reg 10: unmute DACs
	writeSabreReg(0x0A, reg10);     // mute DACs
}

/*
 * Read a single line from the USB serial (which is really just a bridge to MPD).
 * Returns a single \n-terminated line of text on success.
 * Returns NULL on timeout or error.
 * If error, it tears down the USB -> MPD connection.
 */
char *usb_readln() {
	static char buf[MAX_INPUT_BUF];
	char *p;
	unsigned long ms=millis(), timeElapsed;

	// if we're not connected to MPD just return NULL
	if(!connectedToMPD)
		return NULL;

	memset(buf, 0, sizeof(buf));
	p=buf;
	do {
		if(Serial.available() > 0) {
			*p=Serial.read();
			if(*p=='\n')
				break;
			p++;
		}
		timeElapsed=millis() - ms;
	} while(p<buf+(MAX_INPUT_BUF-1) && timeElapsed < USB_READ_TIMEOUT); // timeout after 2 seconds
	*(++p)=0;

	if(timeElapsed >= USB_READ_TIMEOUT) {
		disconnectMPD(); // if we timeout then assume MPD went away. Forcibly disconnect to remain sane.
		return NULL;
	} else {
		return buf;
	}
}

void usb_writeln(const char *s) {
	if(!connectedToMPD)
		return;
	Serial.println(s);
	Serial.flush();
}

void xyfputs(int x, int y, uint8_t *font, const char *str) {
	GLCD.SelectFont(font);
	GLCD.CursorTo(x, y);
	GLCD.Puts(str);
}

void xyputs(int x, int y, uint8_t *font, const char *str) {
	GLCD.CursorTo(0, 0);
	GLCD.Puts(str);
}

void show_screen(int screenType) {
	char buf[16];
	// Only clear the screen the first time a new screen is selected. Faster.
	if(screenType != currentScreenType)
		GLCD.ClearScreen();
	
	switch(screenType) {
		case SCREEN_MPD_WAIT:
			xyfputs(0, 0, Verdana28, "Init MPD...");
			break;
		case SCREEN_DAC_WAIT:
			xyfputs(0, 0, Verdana28, "Init DAC...");
			break;
		case SCREEN_SPLASH:
			xyfputs(0, 0, Verdana28, "Booting... ");
			break;
		case SCREEN_PREFS:
			break;
		case SCREEN_INPUTS:
			break;
		case SCREEN_VOLUME:
			sprintf(buf, "%3d%%  ", currentVolume);
			if(currentScreenType!=SCREEN_VOLUME) // optimize: only write this once.
				xyfputs(0, 0, Verdana28, "Vol. ");
			xyfputs(5, 0, Verdana28, buf);
			break;
		case SCREEN_IDLE:
			sprintf(buf, "%3d%%  ", currentVolume);
			if(currentScreenType!=SCREEN_IDLE) // optimize: only write this once.
				xyfputs(0, 0, Verdana28, "Vol. ");
			xyfputs(5, 0, Verdana28, buf);
			break;
		default:
			break;
	}
	currentScreenType=screenType;
}

void disconnectMPD() {
	Serial.end();
	connectedToMPD=false;
}

int startMPD() { // talk to the USB <-> TCP/IP daemon
	int mpdVol=-1;
	char *ptr;

	if(connectedToMPD==true)
		return 0;

	Serial.begin(115200);
	connectedToMPD=true;

	// read banner and any gunk
	delay(250);
	usb_writeln("nonsense");
	delay(250);
	while(Serial.available() > 0)
		Serial.read();
	
	// get the current volume, etc
	usb_writeln("status");
	do {
		if((ptr=usb_readln())==NULL)
			return -1; // timeout talking to MPD

		if(strstr(ptr, "volume: ")) {
			ptr+=8; // skip past "volume: "
			mpdVol=atoi(ptr);
			if(mpdVol <0 || mpdVol > 100)  { // sanity check
				mpdVol=-1;
			}
		}
	} while(!strstr(ptr, "OK")); 
	
	if(mpdVol != -1)
		volume_set_DAC((mpdVol<currentVolume)?mpdVol:currentVolume); // set vol to the lower of the two volume controls: DAC or MPD.
	else
		return -2; // return -2 if we don't get the OK from mpd daemon

	// subscribe to the remote feed
	usb_writeln("subscribe remote");

	// check mpd's response for errors
	if((strstr(usb_readln(),"OK"))==NULL)
		return -3; // return -3 if we can't subscribe to remote events on mpd

	// Otherwise, we got the OK from mpd. We're subscribed.
	// Now we go idle.
	usb_writeln(GO_IDLE);
	
	// success
	return 0;
}


/************************ MAIN PROGRAM ************************************************************/






void setup() {
	Wire.begin();               // Join the I2C bus as a master
	GLCD.Init(NON_INVERTED);    // initialise the library, white-on-black style

	show_screen(SCREEN_DAC_WAIT);
	startDac1();       // mute the DAC and make it sane
	
	show_screen(SCREEN_MPD_WAIT);
	startMPD();

	// Attach Interrupts
	attachInterrupt(0, rotEncoder, CHANGE);  // ISR for rotary encoder on pin 2

	// Set up the pin modes
	pinMode(VOLUPPIN, INPUT);       // Button switch or Encoder pin for volume up
	digitalWrite(VOLUPPIN, HIGH);
	pinMode(VOLDOWNPIN, INPUT);     // Button switch or Encoder pin for volume down
	digitalWrite(VOLDOWNPIN, HIGH);
	pinMode(SELECTPIN, INPUT);      // Button switch or pin for encoder built-in switch
	digitalWrite(SELECTPIN, HIGH);  // Enable pull-up resistor
	//pinMode(REMOTEPIN, INPUT);      // Pin for IR sensor
	//digitalWrite(REMOTEPIN, HIGH);  // Enable pull-up resistor

	readSettings();
	initForInput(input);
	volume_set_DAC(currentVolume);
	unmute();

	show_screen(SCREEN_IDLE);
} // End setup()




// -------------------------------------loop section------------------------------------------




void loop() {
	int button, mpdVol;
	char *ptr;

	button=0xff;
	mpdVol=-1;

	// Print the sample rate, input and lock (once every "INTERVAL_SAMPLE" time)
	if ((millis() - displayMillis > INTERVAL_SAMPLE) && !selectMode) {
		displayMillis = millis();                   // Saving last time we display sample rate		
		Status = readRegister(27);                  // Read status register
		if (Status & B00000100) SPDIFValid = true;  // Determine if valid spdif data
		else SPDIFValid = false;                    // SR calculation depends on I2S or SPDIf

		show_screen(SCREEN_IDLE);

		// once every INTERVAL attempt to connect to MPD if we're not already connected.
		if(!connectedToMPD)
			startMPD();

		// sync our volume to MPD if required
		if(connectedToMPD && volumeSyncRequired) {
			volume_set_MPD();
		}
	}


	// USB MPD interface
	// Check for data on the USB port
	if(connectedToMPD && Serial.available() > 0) {		
		// read any data from MPD
		while(connectedToMPD && Serial.available() > 0) {
			if((ptr=usb_readln())==NULL) // read the next line of incoming data from MPD
				break;

			if(strstr(ptr, "changed: mixer")) { 
				if((ptr=usb_readln())==NULL) // Expect OK from MPD. Abort on timeout.
					break;	

				usb_writeln("status"); // Issue "status" command to MPD...	
				
				// loop through the server response until "OK" or timeout
				do {
					if((ptr=usb_readln())==NULL) // get status line
						break;

					if(strstr(ptr, "volume: ")) {
						ptr+=8; // skip past "volume: "
						mpdVol=atoi(ptr);
						if(mpdVol < 0 || mpdVol > 100) // sanity check
							mpdVol=-1;
					}
				} while(ptr && !strstr(ptr, "OK"));

				// if mpdVol was changed, update the volume level in the Buffalo DAC
				if(mpdVol != -1) {
					volume_set_DAC(mpdVol);
					mpdVol=-1; // reset the flag so that we only do this once
				}
			}// changed: mixer
		}
		
		usb_writeln(GO_IDLE);
	}
	

	// Check the push-button
	if((digitalRead(SELECTPIN)==LOW)&&((millis()-debounceMillis)>INTERVAL_SWITCHBOUNCE) || (button==BUT_SEL)) {
		GLCD.ClearScreen();
		initForInput(input);
		selectMode=true;          // Now we are in select mode
		debounceMillis=millis();  // Start debounce timer
		selectMillis=millis();    // Start being-in-select-mode timer

		select++;  // Move to next select option
		// First read the current setting for input format since it is not normally displaye
		GLCD.CursorTo(8,0);
		switch(settings[input][FORMATVAL]%FORMATCHO) {
			case 0:
				GLCD.Puts("SPd");
			break;
				case 1:
				GLCD.Puts("Sp3");
				break;
			case 2:
				GLCD.Puts("Sp7");
				break;
			case 3:
				GLCD.Puts("Sp8");
				break;
			case 4:
				GLCD.Puts("I2S");
				break;
			case 5:
				GLCD.Puts("LJF");
				break;
			case 6:
				GLCD.Puts("R32");
				break;
			case 7:
				GLCD.Puts("R24");
				break;
			case 8:
				GLCD.Puts("R20");
				break;
			case 9:
				GLCD.Puts("R16");
				break;
		}

		switch(select%(MAXPARAM+3)){
			case VOL:
				GLCD.CursorTo(13,1);
				GLCD.Puts("*");
				break;
			case INP:
				GLCD.CursorTo(0,0);
				GLCD.Puts("*");
				break;
			case INF:
				GLCD.CursorTo(7,0);
				GLCD.Puts("*");
				break;
			case FIR:
				GLCD.CursorTo(0,1);
				GLCD.Puts("*");
				break;
			case IIR:
				GLCD.CursorTo(7,1);
				GLCD.Puts("*");
				break;
			case DPL:
				GLCD.CursorTo(0,2);
				GLCD.Puts("*");
				break;
			case QTZ:
				GLCD.CursorTo(0,3);
				GLCD.Puts("*");
				break;
			case NCH:
				GLCD.CursorTo(7,3);
				GLCD.Puts("*");
				break;
			case PLM:
				GLCD.CursorTo(7,2);
				GLCD.Puts("*");
				break;
			case OSF:
				GLCD.CursorTo(11,1);
				GLCD.Puts("*");
				break;
			case SRF:
				GLCD.CursorTo(11,0);
				GLCD.Puts("*");
				break;
		}  // End of switch
	}
 

	// For the rotary encoder. 
	// The control depends whether in display mode or select mode
	if(rotating) {
		delay(INTERVAL_BOUNCE);  // debounce by waiting INTERVAL_BOUNCE time
		switch(select%(MAXPARAM+3)) {
			case VOL:  // Volume adjustment
				dimmed=false;                          // Disengage dim
				if(button==BUT_CCW || (digitalRead(4) == digitalRead(2))) {
					volume_down();
				} else {                                   // If not CCW, then it is CW
					volume_up();
				}
				break;
			case INP:  // Input selection
				if((digitalRead(4) == digitalRead(2))||(button==BUT_CCW)) input--;  // CCW
				else input++;
				input%=ICHO;
				initForInput(input);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case INF:  // Input format selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][FORMATVAL]--;  // CCW
				else settings[input][FORMATVAL]++;
				initForInputFormat(settings[input][FORMATVAL]%FORMATCHO);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case FIR:  // FIR filter selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][FIRVAL]--;  // CCW
				else settings[input][FIRVAL]++;
				initFirFilter(settings[input][FIRVAL]%FIRCHO);
				selectMillis=millis();  // Update being-in-select-mode timer
				break;
			case IIR:  // IIR filter selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][IIRVAL]--;  // CCW
				else settings[input][IIRVAL]++;
				initIirFilter(settings[input][IIRVAL]%IIRCHO);
				selectMillis=millis();  // Update being-in-select-mode timer
				break;
			case QTZ:  // Quantizer selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][QUANVAL]--;  // CCW
				else settings[input][QUANVAL]++;
				initQuantizer(settings[input][QUANVAL]%QUANCHO);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case DPL:  // DPLL setting: separate selection for SPDIF and I2S
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][DPLLVAL]--;  // CCW
				else settings[input][DPLLVAL]++;
				initDPLL(settings[input][DPLLVAL]%DPLLCHO);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case NCH:  // Notch selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][NOTCHVAL]--;  // CCW
				else settings[input][NOTCHVAL]++;
				initNotch(settings[input][NOTCHVAL]%NOTCHCHO);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case PLM:  // DPLL mode selection
				if (digitalRead(4) == digitalRead(2)||(button==BUT_CCW)) settings[input][PLMVAL]--;  // CCW
				else settings[input][PLMVAL]++;
				initDPLLMode(settings[input][PLMVAL]%PLMCHO);
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case OSF:  //OSF Bypass -Since this is a toggle, we just call the function
				initBypassOSF();
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
			case SRF:  //Sampler Rate format -Since this is a toggle, we just call the function
				initSRFormat();
				selectMillis=millis();  // Reset being-in-select-mode timer
				break;
		}  // End of (rotary encoder) switch
		
		rotating=false; // Reset the flag
	}  // End of if(rotating)
	 
	// When the being-in-select mode timer expires, we revert to volume/display mode
	if (selectMode && millis() - selectMillis > INTERVAL_SELECT * 1000) {
		selectMode = false; // No longer in select mode
		select = VOL;       // Back to volume mode
		writeSettings();    // Write new settings into EEPROM, including current input
	}

	/*
	The following code is for the remote control. It can handle all the codes generated by the Apple
	remote control except code "0" has been designated (by me) for "repeat" and code "255" designated
	(by me)as an error code. The current Apple remote does not generate these codes so it is safe to
	designate them for other things.

	In addition, NEW: the code ignores the repeat code by default, except if designated for repeat. In
	this case the keys for up and down volume have been designated for repeat.

	If you are using the apple remote that has the different set of codes, you must now change it in
	two places for the "up" and the "down" keys
	*/
	/*
		while(digitalRead(REMOTEPIN)==LOW){
		// Serial.println("hi");
		if((IRkey=getIRkey())==255){
		// Do nothing
		}
		else {
		if(IRkey==0) {            // Repeat code
		  if(previousIRkey==5||previousIRkey==6) {
			 IRkey=previousIRkey;  // Repeat code, only for specified keys as indicated. Add additional
										  // keys to the "or" comparison in the if() statement if you want them
										  // to repeat
		  }
		  else {
			 // Do nothing. No repeat for the rest of the keys
		  }
		}
		else {                    // Not a repeat code, it is a new command
		  previousIRkey=IRkey;    // Remember the key in case we want to use the repeat code
		}
		}

		switch(IRkey){
		// case 0 and 255 are "valid" cases from the code, but we do nothing in this switch statement
		case 5:  // 5 is the up key, we will use for volume up
		if (currAttn>MINATTNU)                // Check if not already at minimum attenuation
		{
		  if(dimmed) {
			 rampUp();                          // Disengage dim
			 dimmed=false;
		  }

		  currAttn-=2;                      // Decrease attenuation 1 dB (increase volume 1 db)
		  setSabreAttenuation(currAttn);         // Write value into registers
		  printTwoNumber(13,currAttn/2);    // Divide by 2 to print in whole dBs

		}
		break;
		case 6:  // 6 is the down key, we will use for volume down
		if (currAttn<MAXATTNU)              // Check if not already at maximum attenuation
		{
		  if(dimmed) {
			 rampUp();                        // Disengage dim
			 dimmed=false;
		  }

		  currAttn+=2;                      // Increase 1 dB attenuation (decrease volume 1 db)
		  setSabreAttenuation(currAttn);         // Write value into registers
		  printTwoNumber(13,currAttn/2);    // Divide by 2 to print in whole dBs

		}
		break;
		case 2:  // 4 is the center key. This is the soft mute -dim feature
		if(dimmed){                          // Back to previous volume level 1 db at a time
		  rampUp();                          // Disengage dim
		  dimmed=false;
		}
		else {
		  if(DIM>=currAttn) {               // only DIM if current attenuation is lower
			 setSabreAttenuation(DIM);             // Dim volume
			 printTwoNumber(13,DIM/2);
			 dimmed=true;
		  }
		}
		break;
		}
		} // End of remote control code
	 */
	/*
		To debounce the switch, we detect the first instance of the "click" and then ignore
		the switch: first during the switch "bouncing" time and then while the switch remains
		pressed because one cannot lift the finger so fast.
		We want to register a single "click" per push of the switch
	 */
 
	//GLCD.CursorTo(3,7);
	//GLCD.Printf("select %d",select%(MAXPARAM+3));

}  // End of loop()



void volume_up() {
	if(dimmed) {
		rampUp();                          // Disengage dim
		dimmed=false;
	}
	currentVolume+=2; // 1dB
	if(currentVolume>100)
		currentVolume=100;
	volumeSyncRequired=true;	// remember to update MPD when we get a chance
	volume_set_DAC(currentVolume);
}

void volume_down() {
	currentVolume-=2; // 1dB
	if(currentVolume < 0 || currentVolume > 100) // meh, integer sign laziness
		currentVolume=0;
	volumeSyncRequired=true;	// remember to update MPD when we get a chance
	volume_set_DAC(currentVolume);
}

void volume_set_DAC(int level) {
	currentVolume=level;
	currAttn=(int)(((float)(MAX_ATTN/100)) * ((float)(100-currentVolume)));	
	setSabreAttenuation(currAttn);
}

/*
 * Sets the volume level in MPD to that currently set in the DAC.
 * Used to sync DAC -> MPD volume after the user adjusts volume via the rotary encoder.
 */
void volume_set_MPD() {
	if(!connectedToMPD || !volumeSyncRequired)		// make sure we're connected to MPD and we're due to sync
		return;

	usb_writeln(GO_NOIDLE);	// drop out of idle mode
	if(usb_readln()==NULL)	// wait for OK
		return;
	
	sprintf(tmp,"setvol %d", currentVolume);
	usb_writeln(tmp);		// write new volume level to MPD server
	
	if(usb_readln()==NULL)	// wait for OK
		return;
	
	usb_writeln(GO_IDLE);	// return to idling
	volumeSyncRequired=false; // success! no need to sync again until next time.
}



/*
	The following function returns the code from the Apple Aluminum remote control. The Apple remote is
	based on the NEC infrared remote protocol. Of the 32 bits (4 bytes) coded in the protocol, only the
	third byte corresponds to the keys. The function also handles errors due to noise (returns 255) and
	the repeat code (returs zero)

	The Apple remote returns the following codes:

	Up key:     238 135 011 089
	Down key:   238 135 013 089
	Left key:   238 135 008 089
	Right key:  238 135 007 089
	Center key: 238 135 093 089 followed by 238 135 004 089 (See blog for why there are two codes)
	Menu key:   238 135 002 089
	Play key:   238 135 094 089 followed by 238 135 004 089

	(update) The value of the third byte varies from remote to remote. It turned out that the 8th bit
	is not part of the command, so if we only read the 7 most significant bits, the value is the same
	for all the remotes, including the white platic remote.

	The value for the third byte when we discard the least significant bit is:

	Up key:     238 135 005 089
	Down key:   238 135 006 089
	Left key:   238 135 004 089
	Right key:  238 135 003 089
	Center key: 238 135 046 089 followed by 238 135 002 089 (See blog for why there are two codes)
	Menu key:   238 135 001 089
	Play key:   238 135 047 089 followed by 238 135 002 089

	More info here: http://hifiduino.wordpress.com/apple-aluminum-remote/
 */

int getIRkey() {

	c1 = 0;
	c2 = 0;
	c3 = 0;
	c4 = 0;
	duration = 1;
	while ((duration = pulseIn(REMOTEPIN, HIGH, 15000)) < 2000 && duration != 0) {
		// Wait for start pulse
	}
	if (duration == 0)                                  // This is an error no start or end of pulse
		return 255;                                     // Use 255 as Error

	else if (duration < 3000)                           // This is the repeat
		return 0;                                       // Use zero as the repeat code

	else if (duration < 5000) {                         // This is the command get the 4 byte
		mask = 1;
		for (int i = 0; i < 8; i++) {                   // get 8 bits
			if (pulseIn(REMOTEPIN, HIGH, 3000) > 1000)  // If "1" pulse
				c1 |= mask;                             // Put the "1" in position
			mask <<= 1;                                 // shift mask to next bit
		}
		mask = 1;
		for (int i = 0; i < 8; i++) {                   // get 8 bits
			if (pulseIn(REMOTEPIN, HIGH, 3000) > 1000)  // If "1" pulse
				c2 |= mask;                             // Put the "1" in position
			mask <<= 1;                                 // shift mask to next bit
		}
		mask = 1;
		for (int i = 0; i < 8; i++) {                   // get 8 bits
			if (pulseIn(REMOTEPIN, HIGH, 3000) > 1000)  // If "1" pulse
				c3 |= mask;                             // Put the "1" in position
			mask <<= 1;                                 // shift mask to next bit                                         // Now discard least significant bit
		}
		mask = 1;
		for (int i = 0; i < 8; i++) {                   // get 8 bits
			if (pulseIn(REMOTEPIN, HIGH, 3000) > 1000)  // If "1" pulse
				c4 |= mask;                             // Put the "1" in position
			mask <<= 1;                                 // shift mask to next bit
		}
		//Serial.println(c1, HEX); //For debugging
		//Serial.println(c2, HEX); //For debugging
		//Serial.println(c3, HEX); //For debugging
		//Serial.println(c4, HEX); //For debugging

		c3 >>= 1;                                    // Discard the least significant bit
		return c3;
	}
}

// Routines for reading and writing from EEPROM
void writeSettings() {
	for (byte i = 0; i < ICHO; i++) {
		for (byte j = 0; j < MAXPARAM; j++)
			if (EEPROM.read((i * MAXPARAM) + j) != settings[i][j])  // Check an see if there are any changes
				EEPROM.write((i * MAXPARAM) + j, settings[i][j]);   // Write the changes in eeprom
	}
	EEPROM.write(ICHO * MAXPARAM, input);                           // Write the current input in a variable location after the array
	EEPROM.write(ICHO * MAXPARAM + 1, SRExact);                     // Write the value of SRExact
}

void readSettings() {
	for (byte i = 0; i < ICHO; i++)
		for (byte j = 0; j < MAXPARAM; j++)
			settings[i][j] = EEPROM.read((i * MAXPARAM) + j);
	input=EEPROM.read(ICHO*MAXPARAM);      // Read the last saved input setting
	SRExact=EEPROM.read(ICHO*MAXPARAM+1);  // Read the last saved setting for SRExact
}


/*
	ROTARY ENCODER

	The rotary encoder is connected to digital pin 2 (A) and digital pin 4 (B). pin 2 (and pin 3) is an
	interrupt line in Arduino. It does not matter which terminal is connected to which pin. The third
	terminal is connected to GND. At each cycle, the rotary encoder will pull the pins LOW or
	HIGH and the transition is detected by the interrupt line. The interrupt service routine is
	specified below.

	Debounce
	In this version, the code implements debouncing by adding a few millisecond delay that pauses
	the code from evaluating the state of the pins. Typically, all the switching noise –the “bouncing”
	is generated in the first few milliseconds after the switch event. The code is optimized by calling
	the delay only if there is any activity in the rotary encoder. Activity in the rotary encoder is
	detected with an interrupt. The interrupt service routine is minimalist in design and determines if
	there is any motion in the rotary encoder.

	In addition, in order to minimize spurious interrupts –those generated by the switching noise,
	hardware debounce can be optionally implemented in the rotary encoder. Although the code will ignore
	all the interrupts generated during the debounce delay time, interrupts are still generated and it is
	still a good idea to minimize those spurious interrupts. There is just a slight change in the software
	if H/W debouncing is implemented, namely the pull up resistors in the pins are to be disabled.

	Further, the rotary encoder has an on-off switch, and the debouncing of the switch is also done in s/w
	(again the same h/w debouncing can be implemented, but it is optional). It is not based on interrupts
	and because pushing the switch manually will generate not only noise during the switching, but the
	switch can remain pressed for 100 milliseconds or more because one cannot lift the finger that fast.
	In this implementation and with my way of pusshing down the switch, 200 msec is an appropriate value.

	INTERRUPT SERVICE ROUTINE FOR ROTARY ENCODER

	The interrupt service routine has been designed to be minimalist in nature. The code just sets a
	flag indicating that some activity has been detected in the rotary encoder.
 */

void rotEncoder(){
	static int toggle=0;

	toggle=1-toggle; // my encoder is shit and generates 2 interrupts per click. ignore every 2nd one.
	if(toggle)
		rotating = true; // If motion is detected in the rotary encoder, set the flag to true
}

/*
	READING THE SAMPLE RATE

	The sample rate can be calculated by reading the DPLL 32-bit register. For SPDIF DPLL value
	is divided by (2^32/Crystal-Frequency). In Buffalo II (original), the Crystal frequency is
	80,000,000 Hz. In Arduino (and other small microprocessors) it is NOT advisable to do floating point
	math because "it is very slow"; therefore integer math will be used to calculate the sample rate.

	The value of 2^32/80,000,000 is 53.687091 (which is a floating point number). If we use the
	integer part (53 or 54) we get the following results for a 44.1K sample rate signal:  divided by 53
	the result is 44.677K; divided by 54, the result is 43.849K. Clearly there are large errors from
	being confined to integer math. The actual result, if we use floating point math and use all the
	significant digits is 44,105 Hz (the 5 Hz deviation from ideal 44100 Hz is within the specification
	of SPDIF and the tolerances of the crystals and clocks involved)

	In order to increase the accuracy of the integer calculation, we can use more of the significant
	digits of the divisor. I did some evaluation of the DPLL register values for sample rates ranging
	from 44.1K to 192K and noticed that I could multiply the value of the DPLL number by up to
	400 without overflowing the 32-bits. Therefore, since we have 32 bit number to work with, we
	can multiply the DPLL number by  up to 400 and then divide by 400X53.687091=21475. If we do this,
	we obtain 44.105K which is the same as the exact value.

	I used a spreadsheet to calculate the multipliers for SPDIF and I2S and for both 80Mhz and 100Mhz

	SPDIF 80Mhz:  x80, %4295
	SPDIF 100Mhz: x20, %859
	I2S 80Mhz:     x1, %3436
	I2S 100Mhz:    x4, %10995 (higher multiplier will overflow the 32bit value for 384KHz SR)
					 x5, %13744 (More accurate but only works up to 192KHz SR)

	For I2S input format the dpll number is divided by (2^32*64/Crystal-Frequency) Note the 64 factor.
	The value of this is 3435.97 which rounds off nicely to 3436 (which is only 0.0008% error). The
	resultant value for the sample rate is the same wheter in spdif or I2S mode.
 */

// Sample rate reading routines

volatile unsigned long DPLLNum; // Variable to hold DPLL value

byte readRegister(byte regAddr) {
	Wire.beginTransmission(I2C_BUF_ADDR);   // Hard coded the Sabre/Buffalo device  address
	Wire.write(regAddr);            // Queues the address of the register
	Wire.endTransmission();         // Sends the address of the register
	Wire.requestFrom(I2C_BUF_ADDR, 1);      // Hard coded to Buffalo, request one byte from address
											  // specified with Wire.write()/wire.endTransmission()
	//while(!Wire.available()) {  // Wait for byte to be available on the bus
	if (Wire.available())           // Wire.available indicates if data is available
		return Wire.read();         // Wire.read() reads the data on the wire
	else
		return 0;                   // In no data in the wire, then return 0 to indicate error
}

unsigned long sampleRate() {
	DPLLNum = 0;
	// Reading the 4 registers of DPLL one byte at a time and stuffing into a single 32-bit number
	DPLLNum |= readRegister(31);
	DPLLNum <<= 8;
	DPLLNum |= readRegister(30);
	DPLLNum <<= 8;
	DPLLNum |= readRegister(29);
	DPLLNum <<= 8;
	DPLLNum |= readRegister(28);

	// The following calculation supports only the 100MHz oscillator in Buf3
	if (SPDIFValid) {	
		DPLLNum *= 20;      // Calculate SR for SPDIF -100MHz part
		DPLLNum /= 859;     // Calculate SR for SDPIF -100MHz part
	} else {                // Different calculation for SPDIF and I2S
		DPLLNum *= 4;       // Calculate SR for I2S -100MHz part
		DPLLNum /= 10995;   // Calculate SR for I2S -100MHz part
	}
	if (DPLLNum < 90000) // Adjusting because in integer operation, the residual is truncated
		DPLLNum += 1;
	else {
		if (DPLLNum < 190000)
			DPLLNum += 2;
		else {
			if (DPLLNum < 350000)
				DPLLNum += 3;
			else
				DPLLNum += 4;
		}
	}
	if (bypassOSF)   // When OSF is bypassed, the magnitude of DPLL is reduced by a factor of 64
		DPLLNum *= 64;

	return DPLLNum;
}

/*
	CONTROLLING THE DIGITAL ATTENUATION (VOLUME) -and other registers IN THE DAC

	The device address of Sabre DAC Datasheet specifies the address as 0x90 which is an 8-bit value.
	The wire library in Arduino uses 7-bit device addresses and the 8th R/W bit is added automatically
	depending on whether you use the write call [beginTransmission()] or the read call [requestFrom()].
	Therefore, you will use the 7 most significant bits of the 8-bit address.
	In our example, 0x90 becomes 0x48 as follows:
	0x90: 10010000 (we eliminate the rightmost bit to get I2C address)
	0x48: 1001000
	When using dual-mono configuration, the other device can be set to addres 0x92
	0x92: 10010010 (we eliminate the rightmost bit to get I2C address)
	0x49: 1001001
 */

void writeSabreReg(byte regAddr, byte regVal){
	Wire.beginTransmission(I2C_BUF_ADDR);   //Hard coded to the the Sabre/Buffalo device address
	Wire.write(regAddr);            // Specifying the address of register
	Wire.write(regVal);             // Writing the value into the register
	Wire.endTransmission();

  #ifdef DUALMONO
	Wire.beginTransmission(I2C_BUF_ADDR_2);   //Hard coded to the the other Sabre/Buffalo device address
	Wire.write(regAddr);            // Specifying the address of register
	Wire.write(regVal);             // Writing the value into the register
	Wire.endTransmission();
  #endif DUALMONO
}


//This is the old volume routine. May work better in dual mono because it incrementally works
// on each left and right dac, rather that all of left dac and then all of right dac

void setSabreAttenuation(byte regVal){
	writeSabreReg(0, regVal);   // set up volume in DAC1
	writeSabreReg(1, regVal);   // set up volume in DAC2
	writeSabreReg(2, regVal);   // set up volume in DAC3
	writeSabreReg(3, regVal);   // set up volume in DAC4
	writeSabreReg(4, regVal);   // set up volume in DAC5
	writeSabreReg(5, regVal);   // set up volume in DAC6
	writeSabreReg(6, regVal);   // set up volume in DAC7
	writeSabreReg(7, regVal);   // set up volume in DAC8
}

void rampUp(){
	byte i = (DIM - currAttn);

	for (byte dimval = DIM; dimval > currAttn; dimval--) {
		setSabreAttenuation(dimval);
		printTwoNumber(13, dimval / 2);
		delay((RAMP) * (1 + (10 / i * i)));
		i--;
	}
}

// Because of register 17 sets MONO/8-channel, different values are written into different chips
void writeSabreLeftReg(byte regAddr, byte regVal){
	Wire.beginTransmission(I2C_BUF_ADDR);   // Hard coded to the the Sabre/Buffalo device address for stereo
											  // or mono left. For stereo same as writeSabreReg()
	Wire.write(regAddr);            // Specifying the address of register
	Wire.write(regVal);             // Writing the value into the register
	Wire.endTransmission();
}

#ifdef DUALMONO
void writeSabreRightReg(byte regAddr, byte regVal){
	Wire.beginTransmission(I2C_BUF_ADDR_2);   //Hard coded to the the Sabre/Buffalo device address
	Wire.write(regAddr);            // Specifying the address of register
	Wire.write(regVal);             // Writing the value into the register
	Wire.endTransmission();
}
#endif DUALMONO


void initDPLL(byte value){
	GLCD.CursorTo(1, 2);
	GLCD.Puts("PL ");
	//GLCD.Printf("%c",0xA5);
	switch (value) {
	case DPLL_BW_NONE:
		bitSet(reg25, 1);           // Reg 25: set bit 1 for "use best dpll"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for best dpll
		GLCD.Puts("BST");
		break;
	case DPLL_BW_LOWEST:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x85);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("LST");
		break;
	case DPLL_BW_LOW:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x89);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("LOW");
		break;
	case DPLL_BW_MED_LOW:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x8D);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("L-M");
		break;
	case DPLL_BW_MED:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x91);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("MED");
		break;
	case DPLL_BW_MED_HIGH:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x95);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("M-H");
		break;
	case DPLL_BW_HIGH:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x99);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("HIG");
		break;
	case DPLL_BW_HIGHEST:
		bitClear(reg25, 1);         // Reg 25: Clear bit 1 for "use all settings"
		writeSabreReg(0x19, reg25); // Write value into reg 25 for all settings
		writeSabreReg(0x0B, 0x9D);  // Reg 11: Set corresponding DPLL bandwidth
		GLCD.Puts("HST");
		break;
	}
}

void initDPLLMode(byte value){ // Set the DPLL Mode
	GLCD.CursorTo(8, 2);
	switch (value) {
	case 0:
		bitSet(reg10, 2);       // Set bit 2 of reg 10: jitter reduction ON
		writeSabreReg(0x0A, reg10);
		bitSet(reg25, 0);       // Set bit 0 of reg 25 for x128 DPLL bandwidth
		writeSabreReg(0x19, reg25);
		bitClear(reg17L, 1);    // Reg 17: Clear bit 1 to NOT invert DPLL phase
		writeSabreLeftReg(0x11, reg17L);
	#ifdef DUALMONO
		bitClear(reg17R, 1);   // Reg 17: Clear bit 1 to NOT invert DPLL phase -dual mono
		writeSabreRightReg(0x11, reg17R);
	#endif DUALMONO
		GLCD.Puts("MUL");
		break;
	case 1:
		bitSet(reg10, 2);       // Set bit 2 of reg 10: jitter reduction ON
		writeSabreReg(0x0A, reg10);
		bitClear(reg25, 0);     // Clear bit 0 of reg 25 for x1 DPLL bandwidth
		writeSabreReg(0x19, reg25);
		bitClear(reg17L, 1);    // Reg 17: Clear bit 1 to NOT invert DPLL phase
		writeSabreLeftReg(0x11, reg17L);
	#ifdef DUALMONO
		bitClear(reg17R, 1);   // Reg 17: Clear bit 1 to NOT invert DPLL phase -dual mono
		writeSabreRightReg(0x11, reg17R);
	#endif DUALMONO
		GLCD.Puts("NOR");
		break;
	case 2:
		bitSet(reg10, 2);       // Set bit 2 of reg 10: jitter reduction ON
		writeSabreReg(0x0A, reg10);
		bitClear(reg25, 0);     // Clear bit 0 of reg 25 for x1 DPLL bandwidth
		writeSabreReg(0x19, reg25);
		bitSet(reg17L, 1);      // Reg 17: Set bit 1 to invert DPLL phase
		writeSabreLeftReg(0x11, reg17L);
	#ifdef DUALMONO
		bitSet(reg17R, 1);     // Reg 17: Set bit 1 to invert DPLL phase -dual mono
		writeSabreRightReg(0x11, reg17R);
	#endif DUALMONO
		GLCD.Puts("INV");
		break;
	case 3:
		bitClear(reg10, 3);    // Clear bit 2 of reg 10: jitter reduction bypass
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("OFF");
		break;
	}
}

void initBypassOSF(){                // This is just a toggle function
	if (bypassOSF == true) {                // This is to use oversampling (bypass off)
		bypassOSF = false;
		bitClear(reg17L, 6);                // Reg 17: clear bypass oversampling bit in register
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: bypass OSF off
	#ifdef DUALMONO
		bitClear(reg17R, 6);                // Reg 17: clear bypass oversampling bit in register -dual mono
		writeSabreRightReg(0x11, reg17R);   // Reg 17: bypass OSF off
	#endif DUALMONO
		GLCD.CursorTo(12, 1);
		GLCD.Printf("%c", 0x5E);            // Indicate oversampling is ON (^)
	}else  {
		bypassOSF = true;                   // This is to bypass oversampling
		bitSet(reg17L, 6);                  // Reg 17: set bypass oversampling bit in register
		bitSet(reg17L, 5);                  // Reg 17: set Jitter lock bit, normal operation
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: bypass OSF on, force relock
		delay(50);
		bitClear(reg17L, 5);                // Reg 17: clear relock jitter for normal operation
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Jitter eliminator Normal operation
	#ifdef DUALMONO
		bitSet(reg17R, 6);                  // Reg 17: set bypass oversampling bit in register -dual mono
		bitSet(reg17R, 5);                  // Reg 17: set Jitter lock bit, normal operation -dual mono
		writeSabreRightReg(0x11, reg17R);   // Reg 17: bypass OSF on, force relock
		delay(50);
		bitClear(reg17L, 5);                // Reg 17: clear relock jitter for normal operation
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Jitter eliminator Normal operation
	#endif DUALMONO
		GLCD.CursorTo(12, 1);
		GLCD.Printf("%c", 0xA5);            // Indicate no oversampling (.)
	}
}

void initSRFormat(){             // This is a toggle function for selecting SR display format
	if (SRExact == true) {              // Currently set to display exact sample rate
		SRExact = false;                // Set to Nominal
		GLCD.CursorTo(1, 6);
		GLCD.Puts(" NOMINAL");          // Indicate NOMINAL mode
	}else  {
		SRExact = true;                 // Set to display exact sample rate
		GLCD.CursorTo(1, 6);
		GLCD.Puts("  EXACT ");          // Indicate EXACT mode
	}
}

void initFirFilter(byte value){
	GLCD.CursorTo(1, 1);
	GLCD.Puts("Fi");
	GLCD.Printf("%c", 0xA5);
	GLCD.CursorTo(4, 1);
	switch (value) {
	case 0:
		bitSet(reg14, 0);      // Set bit 0 of reg 14 for sharp fir
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("SHR");
		break;
	case 1:
		bitClear(reg14, 0);    // Clear bit 0 of reg 14 for slow fir
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("SLW");
		break;
	}
}

void initIirFilter(byte value){
	GLCD.CursorTo(1, 5);
	GLCD.Puts("IR");
	GLCD.CursorTo(4, 5);
	switch (value) {
	case 0:                         // | | | | | |0|0| | IIR Bandwidth: Normal (for PCM)
		bitClear(reg14, 1);         // Clear bit 1
		bitClear(reg14, 2);         // Clear bit 2
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("PCM");
		break;
	case 1:                         // | | | | | |0|1| | IIR Bandwidth: 50k (for DSD) (D)
		bitSet(reg14, 1);           // Set bit 1
		bitClear(reg14, 2);         // Clear bit 2
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("50K");
		break;
	case 2:                         // | | | | | |1|0| | IIR Bandwidth: 60k (for DSD)
		bitSet(reg14, 2);           // Set bit 2
		bitClear(reg14, 1);         // Clear bit 1
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("60K");
		break;
	case 3:                         // | | | | | |1|1| | IIR Bandwidth: 70k (for DSD)
		bitSet(reg14, 1);           // Set bit 1
		bitSet(reg14, 2);           // Set bit 2
		writeSabreReg(0x0E, reg14);
		GLCD.Puts("70K");
		break;
	}
}

void initQuantizer(byte value){
	GLCD.CursorTo(1, 3);
	GLCD.Puts("Qz");
	GLCD.Printf("%c", 0xA5);
	GLCD.CursorTo(4, 3);
	switch (value) {
	case 0:                         // 6-bit true diff
		bitSet(reg14, 3);           // True differential
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0x00);  // 6-bit quantizer
		GLCD.Puts("6bT");
		break;
	case 1:                         // 7-bit pseudo diff
		bitClear(reg14, 3);         // Pseudo diff
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0x55);  // 7-bit quantizer
		GLCD.Puts("7bP");
		break;
	case 2:                         // 7-it true diff
		bitSet(reg14, 3);           // True differential
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0x55);  // 7-bit quantizer
		GLCD.Puts("7bT");
		break;
	case 3:                         // 8-bit pseudo diff
		bitClear(reg14, 3);         // Pseudo diff
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0xAA);  // 8-bit quantizer
		GLCD.Puts("8bP");
		break;
	case 4:                         // 8-bit true diff
		bitSet(reg14, 3);           // True differential
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0xAA);  // 8-bit quantizer
		GLCD.Puts("8bT");
		break;
	case 5:                         // 9-bit pseudo diff
		bitClear(reg14, 3);         // Pseudo diff
		writeSabreReg(0x0E, reg14);
		writeSabreReg(0x0F, 0xFF);  // 9-bit quantizer
		GLCD.Puts("9bP");
		break;
	}
}

void initNotch(byte value){
	GLCD.CursorTo(8, 3);
	switch (value) {
	case 0:
		writeSabreReg(0x0C, 0x20); // No notch delay
		GLCD.Puts("NON");
		break;
	case 1:
		writeSabreReg(0x0C, 0x21); // notch delay=mclk/4
		GLCD.Puts("/04");
		break;
	case 2:
		writeSabreReg(0x0C, 0x23); // notch delay=mclk/8
		GLCD.Puts("/08");
		break;
	case 3:
		writeSabreReg(0x0C, 0x27); // notch delay=mclk/16
		GLCD.Puts("/16");
		break;
	case 4:
		writeSabreReg(0x0C, 0x2F); // notch delay=mclk/32
		GLCD.Puts("/32");
		break;
	case 5:
		writeSabreReg(0x0C, 0x3F); // notch delay=mclk/64
		GLCD.Puts("/64");
		break;
	}
}
void initForInputFormat(byte value){
	// This register also controls mono-8channel operation, thus more code...
	GLCD.CursorTo(8, 0);
	switch (value) {
	case 0:                                 // Enable SPDIF for DATA 1 pin
		writeSabreReg(0x08, 0xE8);          // Reg 8: Enable SPDIF input format
		bitSet(reg17L, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: write value into register
		#ifdef DUALMONO
		bitSet(reg17R, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: write value into register
		#endif DUALMONO
		writeSabreReg(0x12, 0x01);          // Set SPDIF to input #1 (only input valid for BII is 1 and 5)
		spdifIn = true;                     // Indicates input format is spdif.
		GLCD.Puts("SPd");
		break;
	case 1:                                 // Enable SPDIF for DATA 3 pin (For BIII use)
		writeSabreReg(0x08, 0xE8);          // Reg 8: Enable SPDIF input format
		bitSet(reg17L, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: write value into register
		#ifdef DUALMONO
		bitSet(reg17R, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: write value into register
		#endif DUALMONO
		writeSabreReg(0x12, 0x02);          // Set SPDIF to input #3
		spdifIn = true;                     // Indicates input format is spdif.
		GLCD.Puts("Sp3");
		break;
	case 2:                                 // Enable SPDIF for DATA 7 pin (For BIII use)
		writeSabreReg(0x08, 0xE8);          // Reg 8: Enable SPDIF input format
		bitSet(reg17L, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: write value into register
		#ifdef DUALMONO
		bitSet(reg17R, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: write value into register
		#endif DUALMONO
		writeSabreReg(0x12, 0x40);          // Set SPDIF to input #7
		spdifIn = true;                     // Indicates input format is spdif.
		GLCD.Puts("Sp7");
		break;
	case 3:                                 // Enable SPDIF for DATA 7 pin (For BIII use)
		writeSabreReg(0x08, 0xE8);          // Reg 8: Enable SPDIF input format
		bitSet(reg17L, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: write value into register
	#ifdef DUALMONO
		bitSet(reg17R, 3);                  // Reg 17: auto spdif detection ON -Set bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: write value into register
	#endif DUALMONO
		writeSabreReg(0x12, 0x80);          // Set SPDIF to input #7
		spdifIn = true;                     // Indicates input format is spdif.
		GLCD.Puts("Sp8");
		break;

	case 4:                                 // I2S 32 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitClear(reg10, 4);                 // Setting to I2S (2 bits required)
		bitClear(reg10, 5);
		bitSet(reg10, 6);                   // Setting to 32 bit (2 bits required)
		bitSet(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("I2S");
		break;
	case 5:                                 // LJ 32 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitSet(reg10, 4);                   // Set to LJ
		bitClear(reg10, 5);
		bitSet(reg10, 6);                   // Set to 32 bit
		bitSet(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("LJF");
		break;
	case 6:                                 // RJ 32 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitSet(reg10, 5);                   // Set to right justified format
		bitClear(reg10, 4);
		bitSet(reg10, 6);                   // Set to 32 bit
		bitSet(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("R32");
		break;
	case 7:                                 // LRJ 24 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitSet(reg10, 5);                   // Set to right justified format
		bitClear(reg10, 4);
		bitClear(reg10, 6);                 // Set to 24 bits mode
		bitClear(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("R24");
		break;
	case 8:                                 // LRJ 24 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitSet(reg10, 5);                   // Set to right justified format
		bitClear(reg10, 4);
		bitSet(reg10, 6);                   // Set to 20 bits mode
		bitClear(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("R20");
		break;
	case 9:                                 // LRJ 16 bit
		bitClear(reg17L, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreLeftReg(0x11, reg17L);    // Reg 17: Auto spdif detection OFF
	#ifdef DUALMONO
		bitClear(reg17R, 3);                // Reg 17: manual SPDIF -Clear bit 3
		writeSabreRightReg(0x11, reg17R);   // Reg 17: Auto spdif detection OFF
	#endif DUALMONO
		writeSabreReg(0x08, 0x68);          // Reg 8: Enable I2S/DSD input format
		spdifIn = false;                    // Set variable to indicate input format is I2S/DSD mode
		bitSet(reg10, 5);                   // Set to right justified mode
		bitClear(reg10, 4);
		bitClear(reg10, 6);                 // Set to 16 bit mode
		bitSet(reg10, 7);
		writeSabreReg(0x0A, reg10);
		GLCD.Puts("R16");
		break;
	default:
		break;

		GLCD.Printf("*", 127);             // Print Arrow to indicate this is input seletion and not signal
	}
}

void initForInput(byte value) {
	initForInputFormat(settings[value][FORMATVAL] % FORMATCHO); // Setup input format value
	initFirFilter(settings[value][FIRVAL] % FIRCHO);         // Setup FIR filter value
	initIirFilter(settings[value][IIRVAL] % IIRCHO);         // Setup IIR filter value
	initDPLL(settings[value][DPLLVAL] % DPLLCHO);            // Setup the DPLL value
	initQuantizer(settings[value][QUANVAL] % QUANCHO);       // Setup quantizer value
	initNotch(settings[value][NOTCHVAL] % NOTCHCHO);         // Setup notch delay value
	initDPLLMode(settings[value][PLMVAL] % PLMCHO);          // Setup dpll mode value
}



void printTwoNumber(byte value, byte number){
	GLCD.CursorTo(value, 1);
	GLCD.Printf("%02d", number);
}

