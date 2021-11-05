// Marble Maze 


#include <Adafruit_SleepyDog.h>


// Radio Includes and variables ----------------------------------

//#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define RF69_FREQ 915.0

// Where to send packets to!
//#define DEST_ADDRESS   1
// change addresses for each client board, any number :)
#define MY_ADDRESS     80

// Feather M0 w/Radio
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define RELAY 		  12
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

int16_t packetnum = 0;  // packet counter, we increment per xmission

uint32_t localCounter = 0;

struct EvenDataPacket{
  uint32_t counter;
  float batteryVoltage;
  uint8_t cubeID;
  uint8_t side;
} eventData;

// Dont put this on the stack:
uint8_t eventBuffer[sizeof(eventData)];
uint8_t from;
uint8_t len = sizeof(eventData);


void selectRadio() {
  digitalWrite(LED,HIGH);
//  digitalWrite(WIZ_CS, HIGH);
  //delay(100);
  digitalWrite(RFM69_CS, LOW);
  //delay(100);
}



//RF communication
void sendEventData()
{  
	rf69.send((uint8_t*)&eventData, sizeof(eventData));
	rf69.waitPacketSent();
}



#include <Adafruit_NeoPixel.h>

// Motor Shield includes and variables for controlling relays for actuators
#include <Wire.h>


#include <JrkG2.h>

JrkG2I2C jrk1(11);
JrkG2I2C jrk2(12);
JrkG2I2C jrk3(13);
JrkG2I2C jrk4(14);

// Driver full down: 1448, stop: 2048, full up: 2648
#define ONE_DOWNSLOW 		  1900
#define ONE_DOWNFAST 		  1500*0.9
#define ONE_UPSLOW			  2300*1.1
#define ONE_UPFAST 			  2648

#define TWO_DOWNSLOW 		  1840*1.0
#define TWO_DOWNFAST 		  1500*1.05
#define TWO_UPSLOW			  2300*1.0
#define TWO_UPFAST 			  2648*0.97

#define THREE_DOWNSLOW 		1860*1.0
#define THREE_DOWNFAST 		1500*1.03
#define THREE_UPSLOW		  2300*1
#define THREE_UPFAST 		  2575*1

#define FOUR_DOWNSLOW 		1700*1.0
#define FOUR_DOWNFAST 		1500*1.15
#define FOUR_UPSLOW			  2300*1.0
#define FOUR_UPFAST 		  2560*1.0


#define STOP 			2048

#define FASTDURATION	2400
#define SLOWDURATION	700
#define STOPDURATION	1400



#define PIXELPIN 11
#define NUMPIXELS 68

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB + NEO_KHZ800);


// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce2
#include <Bounce2.h>

#define SERIALDEBUG 0 // Make everything wait to start until Serial connection established

#define MAXDIRECTIONS	34 // Only use upper half for now to reduce LED play

#define EMPTY	0
#define RED 	1
#define BLUE 	2
#define GREEN	3
#define PURPLE	4

#define LEDPIN				13
#define REDBUTTONPIN		16
#define BLUEBUTTONPIN		17
#define GREENBUTTONPIN		14
#define PURPLEBUTTONPIN		15
#define DELETEBUTTONPIN		18
#define GOBUTTONPIN			19
//#define DOORCLOSESWITCH		6   // Not yet implemented, maybe not debounced
//#define BOXMOTIONSENSE		5	// Not yet implemented, maybe not debounced


int nextDirection = 0;
int prevDirection = 0;

int directions[MAXDIRECTIONS];
int queueHead = 0;
int queueTail = 0;

#define NUM_BUTTONS 6
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {REDBUTTONPIN,BLUEBUTTONPIN,GREENBUTTONPIN,PURPLEBUTTONPIN,DELETEBUTTONPIN,GOBUTTONPIN};
int ledState = LOW;
Bounce * buttons = new Bounce[NUM_BUTTONS];

int pixelAddress[68] = {34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

long lastPressMillis = 0;
#define INACTIVETIMEOUT 60000

void sendGoEvent()
{
  eventData.side = queueTail;
  eventData.cubeID = 80;
  eventData.batteryVoltage = 0;
  eventData.counter++;
  Serial.print("About to send transmission number: ");
  Serial.println(eventData.counter);      
  sendEventData();
  //eventData.side = 0;
}


void setup() {  

	Serial.begin(9600);
	Wire.begin();
	
	#if (SERIALDEBUG)
    	// Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
    	while(!Serial);
	#endif
	restAll();



  // set the printer of the queue.
//  directionsQueue.setPrinter (Serial);

	pinMode(PIXELPIN, OUTPUT);
	strip.begin(); // This initializes the NeoPixel library.


	for(int i=0; i<MAXDIRECTIONS; i++)
	{
		directions[i] = EMPTY;
	}

	pinMode(LEDPIN, OUTPUT);  // use the p13 LED as debugging
	pinMode(REDBUTTONPIN, INPUT_PULLUP);
	pinMode(BLUEBUTTONPIN, INPUT_PULLUP);
	pinMode(GREENBUTTONPIN, INPUT_PULLUP);
	pinMode(PURPLEBUTTONPIN, INPUT_PULLUP);
	pinMode(DELETEBUTTONPIN, INPUT_PULLUP);
	pinMode(GOBUTTONPIN, INPUT_PULLUP);
	
	//pinMode(REDYELLOWSERVO, OUTPUT);
	//pinMode(BLUEORANGESERVO, OUTPUT);
	
	for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
    
  }

  // Setup the LED :
  digitalWrite(LEDPIN, ledState);
  
	push(RED);
	//updateLEDs();
	delay(500);
	popHead();
	//updateLEDs();
	queueHead = 0;
	queueTail = 0;



	//--Radio Setup--//
	selectRadio();
	//pinMode(LED, OUTPUT);     
	pinMode(RFM69_RST, OUTPUT);
	digitalWrite(RFM69_RST, LOW);

	Serial.println("Feather Addressed RFM69 TX Test!");
	Serial.println();

	// manual reset
	digitalWrite(RFM69_RST, HIGH);
	delay(50);
	digitalWrite(RFM69_RST, LOW);
	delay(50);
  
	if (!rf69_manager.init()) {
		Serial.println("RFM69 radio init failed");
		while (1);
	}
	Serial.println("RFM69 radio init OK!");
	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
	// No encryption
	if (!rf69.setFrequency(RF69_FREQ)) {
		Serial.println("setFrequency failed");
	}

	// If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
	// ishighpowermodule flag set like this:
	rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

	// The encryption key has to be the same as the one in the server
	uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
					0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
	rf69.setEncryptionKey(key);  

	eventData.cubeID = 80; // 80 is the Marble Maze
	eventData.side = 0;
	eventData.batteryVoltage = 0;
	eventData.counter = 0;

  // CALIBRATION LOOP
  /*for(int cal=0; cal<4; cal++) {
    upAll();
    delay(3000);
    restAll();
    delay(3000);
  }*/

	Serial.println("Setup Complete.");
}



void loop() {

	readButtons();

	long currentMillis = millis();
	if (((currentMillis - lastPressMillis) > INACTIVETIMEOUT) && (queueHead != queueTail)) {
		emptyQueue();
	}

}

void readButtons()
{
	bool needToToggleLed = false;


  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    // If it fell, flag the need to toggle the LED
    if ( buttons[i].fell() ) {
      needToToggleLed = true;
      switch (i) {
      	case 0: 
      		Serial.println("Red Button Pressed");
      		push(RED);
      		printDirections();
      		updateLEDs();
      		break;
      	case 1: 
      		Serial.println("Blue Button Pressed");
      		push(BLUE);
      		printDirections();
      		updateLEDs();
      		//tiltBlue();
      		break;
      	case 2: 
      		Serial.println("GREEN Button Pressed");
      		push(GREEN);
      		printDirections();
      		updateLEDs();
      		//tiltGreen();
      		break;
      	case 3: 
      		Serial.println("PURPLE Button Pressed");
      		push(PURPLE);
      		printDirections();
      		updateLEDs();
      		//tiltPurple();
      		break;
      	case 4: 
      		Serial.println("Delete Button Pressed");
      		emptyQueue();
      		//Serial.print(popTail()); Serial.print(" removed.");
      		updateLEDs();
      		printDirections();
      		break;
      	case 5: 
      		Serial.println("Go Button Pressed");
      		sendGoEvent();
      		printDirections();
      		while (queueHead != queueTail) {
				nextDirection = popHead();
				//Serial.print(nextDirection);
				if (nextDirection != prevDirection) {
					switch (nextDirection) {
						case EMPTY:
							Serial.println("ERROR: zero direction!");
							break;
						case GREEN:
							tiltGreen();
							break;
						case RED:
							tiltRed();
							break;
						case BLUE:
							tiltBlue();
							break;
						case PURPLE:
							tiltPurple();
							break;
						default:
							Serial.println("ERROR, unknown direction!");
							break;
					}
				} else {
					Serial.print("Duplicate, ignoring");
				}
				printDirections();
				updateLEDs();
				//restAll();
				prevDirection = nextDirection;
				if (digitalRead(DELETEBUTTONPIN) == LOW) {
					emptyQueue();
					restAll();
				}
			}
      		restAll();
      		updateLEDs();
      		queueHead = 0;
      		queueTail = 0;
      		prevDirection = 0;
      		break;
      	case 6: 
      		Serial.println("Door Button Pressed");
      		break;
      	case 7: 
      		Serial.println("Motion Button Pressed");
      		break;
		default:
			Serial.println("ERROR: Unknown Button Pressed");
			break;      	
      }
    }
  }

  // if a LED toggle has been flagged :
  if ( needToToggleLed ) {
    // Toggle LED state :
    ledState = !ledState;
    digitalWrite(LEDPIN, ledState);
    lastPressMillis = millis();
  }

}


void emptyQueue() {
	Serial.println("Emptying Queue");
	while (queueHead != queueTail) {
		nextDirection = popHead();
		prevDirection = nextDirection;
	}
	queueHead = 0;
	queueTail = 0;
	prevDirection = 0;
}


void tiltRed() {
	Serial.println("Tilting Red");
	Serial.println("FAST");
	jrk1.setTarget(ONE_UPFAST);
	jrk2.setTarget(TWO_DOWNFAST);
	jrk3.setTarget(THREE_DOWNFAST);
	jrk4.setTarget(FOUR_UPFAST);
	checkButtonsForDelay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_UPSLOW);
	jrk2.setTarget(TWO_DOWNSLOW);
	jrk3.setTarget(THREE_DOWNSLOW);
	jrk4.setTarget(FOUR_UPSLOW);
	checkButtonsForDelay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	checkButtonsForDelay(STOPDURATION);
}

void tiltPurple() {
	Serial.println("Tilting Purple/Purple");
	Serial.println("FAST");
	jrk1.setTarget(ONE_UPFAST);
	jrk2.setTarget(TWO_UPFAST);
	jrk3.setTarget(THREE_DOWNFAST);
	jrk4.setTarget(FOUR_DOWNFAST);
	checkButtonsForDelay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_UPSLOW);
	jrk2.setTarget(TWO_UPSLOW);
	jrk3.setTarget(THREE_DOWNSLOW);
	jrk4.setTarget(FOUR_DOWNSLOW);
	checkButtonsForDelay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	checkButtonsForDelay(STOPDURATION);
}

void tiltGreen() {
	Serial.println("Tilting Green/Yellow");
	Serial.println("FAST");
	jrk1.setTarget(ONE_DOWNFAST);
	jrk2.setTarget(TWO_UPFAST);
	jrk3.setTarget(THREE_UPFAST);
	jrk4.setTarget(FOUR_DOWNFAST);
	checkButtonsForDelay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_DOWNSLOW);
	jrk2.setTarget(TWO_UPSLOW);
	jrk3.setTarget(THREE_UPSLOW);
	jrk4.setTarget(FOUR_DOWNSLOW);
	checkButtonsForDelay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	checkButtonsForDelay(STOPDURATION);
}

void tiltBlue() {
	Serial.println("Tilting Blue");
	Serial.println("FAST");
	jrk1.setTarget(ONE_DOWNFAST);
	jrk2.setTarget(TWO_DOWNFAST);
	jrk3.setTarget(THREE_UPFAST);
	jrk4.setTarget(FOUR_UPFAST);
	checkButtonsForDelay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_DOWNSLOW);
	jrk2.setTarget(TWO_DOWNSLOW);
	jrk3.setTarget(THREE_UPSLOW);
	jrk4.setTarget(FOUR_UPSLOW);
	checkButtonsForDelay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	checkButtonsForDelay(STOPDURATION);
}

void restAll() {
	Serial.println("All down ==================");
	Serial.println("FAST");
	jrk1.setTarget(ONE_DOWNFAST);
	jrk2.setTarget(TWO_DOWNFAST);
	jrk3.setTarget(THREE_DOWNFAST);
	jrk4.setTarget(FOUR_DOWNFAST);
	delay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_DOWNSLOW);
	jrk2.setTarget(TWO_DOWNSLOW);
	jrk3.setTarget(THREE_DOWNSLOW);
	jrk4.setTarget(FOUR_DOWNSLOW);
	delay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	delay(STOPDURATION);
}

void upAll() {
	Serial.println("All Up ==================");
	Serial.println("FAST");
	jrk1.setTarget(ONE_UPFAST);
	jrk2.setTarget(TWO_UPFAST);
	jrk3.setTarget(THREE_UPFAST);
	jrk4.setTarget(FOUR_UPFAST);
	delay(FASTDURATION);
	Serial.println("SLOW");
	jrk1.setTarget(ONE_UPSLOW);
	jrk2.setTarget(TWO_UPSLOW);
	jrk3.setTarget(THREE_UPSLOW);
	jrk4.setTarget(FOUR_UPSLOW);
	delay(SLOWDURATION);
	Serial.println("STOP");
	jrk1.setTarget(STOP);
	jrk2.setTarget(STOP);	
	jrk3.setTarget(STOP);
	jrk4.setTarget(STOP);
	delay(STOPDURATION);
}



void printDirections()
{
/*	int i;
	Serial.print("[");
	for(i=queueHead; i<queueTail; i++)
	{
		switch (directions[i]) {
			case GREEN:
				Serial.print("GREEN");
				break;
			case RED:
				Serial.print("Red");
				break;
			case BLUE:
				Serial.print("Blue");
				break;
			case PURPLE:
				Serial.print("PURPLE");
				break;
			default:
				Serial.print("Error_printing_direction!");
		}
		if (i != (queueTail - 1)) Serial.print(", ");
	}
	Serial.println("]");
*/
}

void push(int dir)
{
	if (queueTail < MAXDIRECTIONS) {
		updateLED(dir,queueTail);
		directions[queueTail] = dir;
		queueTail++;
	} else {
		Serial.println("Can't push, queue full.");
	}
}

int popTail()
{
	int tmpDir = 0;
	if(queueTail > 0){
	
		updateLED(EMPTY,(queueTail - 1));
		tmpDir = directions[queueTail - 1];
		directions[queueTail - 1] = 0;
	
		queueTail--;
	} else {
		Serial.println("Can't pop, queue empty.");
	}
	return tmpDir;
}

int popHead()
{
	updateLED(EMPTY,queueHead);
	int tmpDir = directions[queueHead];
	directions[queueHead] = 0;
	queueHead++;
	return tmpDir;
}

void updateLEDs() {
	//strip.show();
}

void updateLED(int color, int position) {
    switch(color)
    {
		case EMPTY:
			strip.setPixelColor(pixelAddress[position], strip.Color(0, 0, 0));
			break;
		case GREEN:
			strip.setPixelColor(pixelAddress[position], strip.Color(0, 40, 0));
			break;
		case RED:
			strip.setPixelColor(pixelAddress[position], strip.Color(40, 0, 0));
			break;
		case BLUE:
			strip.setPixelColor(pixelAddress[position], strip.Color(15, 0, 20));
			break;
		case PURPLE:
			strip.setPixelColor(pixelAddress[position], strip.Color(0, 0, 40));
			break;
		default:
			Serial.print("Error changing color!");
	}
    strip.show();
}

void checkButtonsForDelay(int d) {
	long startCBMillis = millis();
	long currentCBMillis = millis();
	while ((currentCBMillis - startCBMillis) < d) {
		currentCBMillis = millis();
		if (	(digitalRead(REDBUTTONPIN) == LOW) ||
				(digitalRead(BLUEBUTTONPIN) == LOW) ||
				(digitalRead(PURPLEBUTTONPIN) == LOW) ||
				(digitalRead(GREENBUTTONPIN) == LOW) ||
				(digitalRead(DELETEBUTTONPIN) == LOW)
			) {
				emptyQueue();
			}
		delay(5);
	}
	return;
}
