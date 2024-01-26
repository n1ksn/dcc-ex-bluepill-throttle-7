/*
  myBluePillThrottle7.ino

  Simple DCC-EX tethered throttle using STM32 Blue Pill,
  5x4 keypad, and SSD1306 I2C OLED display.  It is primarily
  for operating one loco at a time and does not include
  consisting or other advanced features.

  This version has added code to handle an optional rotary 
  encoder for speed changes in addition to the keypad UP and
  DOWN keys.  If an encoder is not used, comment out the 
  statement #define ENCODER below before compiling and
  uploading to the throttle.

  This version has several modifications and refactoring
  of the code to improve switching between locos and
  handling of automatic speed changes when going into and
  out of the Brake/Pause sub-mode.  The emergency brake now
  stops only the current loco being operated.

  Loco speed and direction are read from CS broadcasts
  only to update the display in case a throttle command
  is corrupted for some reason or if a second throttle
  is being used to control the same loco.

  The action of the "Brake" mode emulates the Pause button
  in Engine Driver.  The Brake key is the ESC key in Run mode.

  Decoder functions Fn 0 to Fn 9 are activated by the digit
  keys in Run mode.  The functions 10 to 19 are activated by
  the digit keys in Fn+10 mode, and functions 20 to 27 are
  activated similarly in Fn+20 mode.  In Fn+20 mode the 8
  digit exchanges the direction keys and the 9 digit does
  an emergency stop for the current loco.

  Andrew Palm, 1/26/2024

  Operation:  There are four modes, Run, Fn+10, Fn+20, and Addr, as follows:

  Run mode:  Primary mode of operation with keypad assignments:

    F1 - Enter Fn+10 mode
    F2 - Enter Fn+20 mode
    # - Toggle track power
    * - Enter Addr mode
    UP - Increase speed (inactive while braked)
    DOWN - Decrease speed (inactive while braked)
    Esc - Toggle brake (pause)
    Ent - Stop, set speed to zero
    LEFT - Go Forward direction
    RIGHT - Go Reverse direction
    0 to 9 - Toggle decoder function 0 to 9

  Fn+10 mode:  Toggle decoder functions 10 to 19

    0 to 9 - Toggle function 10 + digit
    Esc - Exit Fn mode without entering digit

  Fn+20 mode:  Toggle decoder functions 20 to 28

    0 to 7 - Toggle function 20 + digit
    Esc - Exit Fn mode without entering digit
    8 - Exchange directions of LEFT and RIGHT keys
    9 - Emergency stop (current loco only)

  Addr mode:  Enter loco address (short or long)

    0 to 9 - Used to enter address into display
    Ent - Finished entering address, set address
    Esc - Exit Addr mode without setting address

*/
#include <Adafruit_SSD1306.h>
#include <Keypad.h>
#include <DCCEXProtocol.h>

//--------------------------------------------------------------------
// Hardware connections
//
//   PA9  - UART TX  - Serial connection to MEGA command station RX2
//   PA10 - UART RX  - Serial connection to MEGA command station TX2
//
//   PB6  - I2C SCL  - I2C clock for 128x32 OLED display
//   PB7  - I2c SDA  - I2C data  for 128x32 OLED display
//
// Note:  The above pins are defaults with STM32duino Core generic STM32F1xx
//
// The Blue Pill board is powered by 3.3v from a small 5v to 3.3v
// regulator board whose 5v side is connected to the MEGA board.
//
// 5 row 4 column keypad conncections and pin definitions
#define KP_COL1 PA0
#define KP_COL2 PA1
#define KP_COL3 PA2
#define KP_COL4 PA3
#define KP_ROW5 PA4
#define KP_ROW4 PA5
#define KP_ROW3 PA6
#define KP_ROW2 PA7
#define KP_ROW1 PA8

// Comment out the line below if a rotary encoder is not used
#define ENCODER

#ifdef ENCODER
// Rotary encoder channel inputs
#define chA PB12      // Denoted CLK on some encoders
#define chB PB13      // Denoted DT on some encoders
#endif

//--------------------------------------------------------------------
// Global variables for throttle operation
#define SPEED_MAX 127

int loco;         // Decoder address
int oldLoco;
int speed;
int oldSpeed;
int speedStep = 1;

enum Direction dir;   // Defined in DCC-EX protocol code

enum dirKey {
  Normal = 0,
  Inverted = 1
};
enum dirKey dirKeyMode;   // Indicates LEFT is Fwd (Normal)
                          // or LEFT key is Rev (Inverted)

bool powerOnFlag;

bool brakedFlag;          // Flags when in Brake (Pause) mode
bool decrToStop;          // Flags when speed is decreasing
                          // after entry into Brake mode
bool incrToOldSpeed;      // Flags when speed is increasing
                          // after exit from Brake mode
int brakeTimeIvl = 300;   // Time (milliseconds) between speed
                          // steps entering and leaving braked state
int brakeTime;            // Time for next speed change for brake
                          // set or release

enum opMode {
  Run = 0,  // Normal throttle operation mode
  Fn1 = 1,   // Decoder function +10 mode
  Fn2 = 2,   // Decoder function +20 mode
  Addr = 3  // Set loco address mode
};
enum opMode mode;

#ifdef ENCODER
// For rotary encoder
volatile unsigned int state;
volatile bool cwFlag;
volatile bool ccwFlag;
#endif

// Declarations for DCC-EX library
DCCEXProtocol dccexProtocol;
Loco* myLoco = nullptr;

//--------------------------------------------------------------------
// Definitions for display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// Contrary to specs, I had to use 0x3C screen address below for both
// 128x64 and 128x32 displays.
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//--------------------------------------------------------------------
// Definitions for keypad
const byte ROWS = 5;  // 5 rows
const byte COLS = 4;  // 4 columns
/*
// Names on keys not needed, but here for reference
char* keyName[] = {
            "F1",  "F2", "#", "*",
            "1",  "2", "3", "UP",
            "4",  "5", "6", "DOWN",
            "7",  "8", "9", "ESC",
            "LEFT",  "0", "RIGHT", "ENTER"
      };
*/
char keyID[] = {
        'A',  'B', '#', '*',
        '1',  '2', '3', 'C',
        '4',  '5', '6', 'D',
        '7',  '8', '9', 'E',
        'F',  '0', 'G', 'H'
      };

char keys[ROWS][COLS] = {
  {keyID[0], keyID[1], keyID[2], keyID[3]},
  {keyID[4], keyID[5], keyID[6], keyID[7]},
  {keyID[8], keyID[9], keyID[10], keyID[11]},
  {keyID[12], keyID[13], keyID[14], keyID[15]},
  {keyID[16], keyID[17], keyID[18], keyID[19]}
};

byte rowPins[ROWS] = {KP_ROW1, KP_ROW2, KP_ROW3, KP_ROW4, KP_ROW5};
byte colPins[COLS] = {KP_COL1, KP_COL2, KP_COL3, KP_COL4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//--------------------------------------------------------------------
// Declare functions
void InitDisplay(void);
void SetDisplayToText(void);
void AcquireLoco();
void UpdateDisplay(void);
void Enc_ISR(void);
void IncrSpeed(void);
void DecrSpeed(void);

class MyDelegate : public DCCEXProtocolDelegate {
  // Define callback functions for DCC-EX library
  public:
    void receivedTrackPower(TrackPower state) {
      // Update track power status from CS broadcast
      if (state == PowerOn)
        powerOnFlag = true;
      else
        powerOnFlag = false;
      UpdateDisplay();
    }

    void receivedLocoUpdate(Loco* someLoco) {
      // Update loco speed and direction from CS broadcast
      // Note:  When changes are made to the loco's speed or direction
      //        from the keypad, the display is updated immediately.
      //        This callback function will change the displayed values
      //        only if there is a problem that prevents the sent
      //        command from being executed by the loco or if another
      //        throttle is also being used to control the same loco.
      if (someLoco == myLoco) {
        dir = myLoco->getDirection();
        speed = myLoco->getSpeed();
        if (mode == Run) UpdateDisplay();
      }
    }
};

MyDelegate myDelegate;

//--------------------------------------------------------------------
void setup() {

  Serial.begin(115200);   // Initialize UART connection to DCC-EX CW
  Wire.begin();           // Initialize I2C for display
  dccexProtocol.connect(&Serial);   // Pass UART connection to protocol
                                    // library

  // Pass callback functions to protocol libary
  dccexProtocol.setDelegate(&myDelegate);

  // Set startup display with defaults
  InitDisplay();
  SetDisplayToText();
  delay(500);

  #ifdef ENCODER
  // Rotary encoder
  pinMode(chA, INPUT_PULLUP);
  pinMode(chB, INPUT_PULLUP);

  // Set up encoder input pin interrupts
  attachInterrupt(digitalPinToInterrupt(chA), Enc_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(chB), Enc_ISR, CHANGE);

  // Initialize encoder state machine in ISR
  state = (digitalRead(chA) << 1 | digitalRead(chB));   // Init state
  cwFlag = false;
  ccwFlag = false;
  #endif

  // Initialize throttle and power flag
  powerOnFlag = false;
  mode = Run;
  brakedFlag = false;
  incrToOldSpeed = false;
  decrToStop = false;

  // Create initial loco object
  loco = 3;
  // Create loco object
  speed = 0;
  dir = Forward;
  AcquireLoco();
  UpdateDisplay();


}

void loop() {

  char key;   // Character returned from keypad
  int keyNum; // Number of function requested

  // Monitor and parse broadcasts from CS, execute
  // callback functions if applicable
  dccexProtocol.check();

  // This section handles the "automated" speed changes
  // when entering or exiting the "Brake" sub-mode (which
  // emulates the Engine Driver Pause key).
  // This code will run no matter what the throttle mode is.

  // Slow to a stop if Brake/Pause state was toggled on
  if (decrToStop) {
    if (millis() >= brakeTime) {
      // Decrement speed
      if (speed > speedStep) {
        speed = speed - speedStep;
      } else {
        speed = 0;
      }
      if (speed == 0) {
        decrToStop = false;
      } else {
        brakeTime = millis() + brakeTimeIvl;
      }
      dccexProtocol.setThrottle(myLoco, speed, dir);
      if (mode == Run) UpdateDisplay();
    }
  }

  // Increase speed to old speed if Brake/Pause state was
  // toggled off
  if (incrToOldSpeed) {
    if (millis() >= brakeTime) {
      // Increment speed
      if (speed <= oldSpeed - speedStep) {
        speed = speed + speedStep;
      } else {
        speed = oldSpeed;
      }
      if (speed == oldSpeed) {
        incrToOldSpeed = false;
        brakedFlag = false;
      } else {
        brakeTime = millis() + brakeTimeIvl;
      }
      dccexProtocol.setThrottle(myLoco, speed, dir);
      if (mode == Run) UpdateDisplay();
    }
  }

  switch (mode) {

    case Run:   // Normal throttle operations

      #ifdef ENCODER
      // Act on any rotary encoder operation
      if (cwFlag) {
        cwFlag = false;   // Reset flag
        IncrSpeed();
      }
      if (ccwFlag) {
        ccwFlag = false;
        DecrSpeed();
      }
      #endif

      // Poll keypad and respond to a key press
      key = keypad.getKey();

      if (key >= '0' && key <= '9') {
        // Activate decoder functions 0 to 9
        keyNum = key - '0';
        if (dccexProtocol.isFunctionOn(myLoco, keyNum))
          dccexProtocol.functionOff(myLoco, keyNum);
        else
          dccexProtocol.functionOn(myLoco, keyNum);
        UpdateDisplay();
      } else {
        // Process all other keys (non-digits)
        switch (key) {
          case 'A':   // F1 key pressed, go into Fn+10 mode
            mode = Fn1;
            UpdateDisplay();
            break;
          case 'B':   // F2 key pressed, go into Fn+20 mode
            mode = Fn2;
            UpdateDisplay();
            break;
          case '*':   // * key pressed, go into address entry mode
            mode = Addr;
            oldLoco = loco;   // Save old address in case of error
            loco = 0;         // Prepare for new address
            UpdateDisplay();
            break;
          case 'C':   // UP key pressed, increment speed
            IncrSpeed();
            break;
          case 'D':   // DOWN key pressed, decrement speed
            DecrSpeed();
            break;
          case 'E':   // Brake (pause) key, ESC key
            if (brakedFlag) {   // Exit braked state (toggle off)
              incrToOldSpeed = true;  // Set flag to resume speed
              brakedFlag = false;
            } else {            // Enter braked state (toggle on)
              brakedFlag = true;
              oldSpeed = speed; // Save current speed
              decrToStop = true;  // Set flag to reduce speed to zero
            }
            UpdateDisplay();
            brakeTime = millis() + brakeTimeIvl;
            break;
          case 'F':   // LEFT key pressed, set direction
            if (dirKeyMode == Normal) dir = Forward;
            else dir = Reverse;
            dccexProtocol.setThrottle(myLoco, speed, dir);
            UpdateDisplay();
            break;
          case 'G':   // RIGHT key pressed, set direction
            if (dirKeyMode == Normal) dir = Reverse;
            else dir = Forward;
            dccexProtocol.setThrottle(myLoco, speed, dir);
            UpdateDisplay();
            break;
          case 'H':   // ENTER key pressed, normal stop
            speed = 0;
            brakedFlag = false;
            incrToOldSpeed = false;
            decrToStop = false;
            dccexProtocol.setThrottle(myLoco, speed, dir);
            UpdateDisplay();
            break;
          case '#':   // # key pressed, toggle track power
            if(!powerOnFlag) {
              dccexProtocol.powerOn();
              powerOnFlag = true;
            } else {
              dccexProtocol.powerOff();
              powerOnFlag = false;
            }
            UpdateDisplay();
            break;
          default:
            break;
        }
      }
      break;    // End Run

    case Fn1:    // Decoder functions 10 to 19
      key = keypad.getKey();  // Get function number (single digit)
      if (key >= '0' && key <= '9') {
        keyNum = key - '0' + 10;
        if (dccexProtocol.isFunctionOn(myLoco, keyNum))
          dccexProtocol.functionOff(myLoco, keyNum);
        else
          dccexProtocol.functionOn(myLoco, keyNum);
        mode = Run;
        UpdateDisplay();
      } else {    // Abort function call with ESC key
        if (key == 'E') {
          mode = Run;
          UpdateDisplay();
        }
      }

      break;    // End Fn1

    case Fn2:    // Decoder functions 20 to 27
                 // Enter 8 to interchange direction key actions
                 // Enter 9 for emergency stop
      key = keypad.getKey();  // Get function number (single digit)
      if (key >= '0' && key <= '7') {
        keyNum = key - '0' + 20;
        if (dccexProtocol.isFunctionOn(myLoco, keyNum))
          dccexProtocol.functionOff(myLoco, keyNum);
        else
          dccexProtocol.functionOn(myLoco, keyNum);
        mode = Run;
        UpdateDisplay();
      } else if (key == 'E') {    // Abort function call with ESC key
        mode = Run;
        UpdateDisplay();
      } else if (key == '8') {
        // Exchange directions of LEFT and RIGHT keys
        if (dirKeyMode == Normal) dirKeyMode = Inverted;
        else dirKeyMode = Normal;
        mode = Run;
        UpdateDisplay();
      } else {
        if (key == '9') {
          // Emergency for current loco
          dccexProtocol.setThrottle(myLoco, -1, dir);
          dccexProtocol.setThrottle(myLoco, 0, dir);
          mode = Run;
          UpdateDisplay();
        }
      }
      break;    // End Fn2

    case Addr:    // Set loco address
      key = keypad.getKey();  // Get digit for address
      if (key >= '0' && key <= '9') {
          loco = 10*loco + (key - '0');   // Build up address
          UpdateDisplay();
      } else {
        switch (key) {
          case 'E':   // Escape out of Addr mode, ESC key
            loco = oldLoco;   // Restore original address
            mode = Run;
            UpdateDisplay();
            break;
          case 'H':   // Enter new address if within address range, ENTER key
            if (loco < 1 || loco > 10293) {
              loco = oldLoco;
            } else {
              // Wait for any Brake/Pause speed changes to finish
              while (decrToStop || incrToOldSpeed) {};
              // Destroy previous loco object and create new loco object
              // with entered address.  Acquires loco with that address.
              delete myLoco;
              AcquireLoco();
            }
            mode = Run;
            UpdateDisplay();
            break;
          default:
            break;
        }
      }
      break;    // End Addr

    default:
      break;

  }

}

//--------------------------------------------------------------------
void AcquireLoco(void) {
  // Create new loco object
  myLoco = new Loco(loco, LocoSource::LocoSourceEntry);
  // Acquire loco by getting it's status (incl. loco already running)
  dccexProtocol.requestLocoUpdate(loco);
  // Reset throttle (these variables aren't saved switching between locos)
  dirKeyMode = Normal;
  brakedFlag = false;
  decrToStop = false;
  incrToOldSpeed = false;
}
//--------------------------------------------------------------------
void IncrSpeed(void) {
  // Increment speed
  if (!brakedFlag) {
    // Stop any speed change due to brake release
    incrToOldSpeed = false;
    // Increase speed if not braked
    if (speed < SPEED_MAX - speedStep) {
      speed = speed + speedStep;
    } else {
      speed = SPEED_MAX;
    }
    dccexProtocol.setThrottle(myLoco, speed, dir);
    UpdateDisplay();
  }
}

void DecrSpeed(void) {
  // Decrement speed
  if (!brakedFlag) {
    // Stop any speed change due to brake release
    incrToOldSpeed = false;
    // Decrease speed if not braked
    if (speed > speedStep) {
      speed = speed - speedStep;
    } else {
      speed = 0;
    }
    dccexProtocol.setThrottle(myLoco, speed, dir);
    UpdateDisplay();
  }
}

//--------------------------------------------------------------------
#ifdef ENCODER
void Enc_ISR() {
  // Interrupt service routine for rotary encoder
  //
  // Reads values on encoder input pins chA and chB and updates state of
  // state machine based on them.  An interrupt is triggered if there is a
  // change on either chA or chB.  If a CW or CCW movement is detected
  // based on a final transition from ENC_CW_FINAL to R_START or, resp.,
  // ENC_CCW_FINAL to R_START, a flag is set for use by the main loop.
  //
  // CW state sequence:
  //   R_START -> ENC_CW_BEGIN -> ENC_CW_NEXT -> ENC_CW_FINAL -> R_START
  //
  // CCW state sequence:
  //   R_START -> ENC_CCW_BEGIN -> ENC_CCW_NEXT -> ENC_CCW_FINAL -> R_START
  //

  // Bit flags that CW or CCW step has been detected
  #define DIR_NONE 0x0      // No complete step yet
  #define DIR_CW 0x10       // CW step
  #define DIR_CCW 0x20      // CCW step

  // State machine state codes, bit1 = chA, bit0 = chB
  // CCW states have bit2 = 1, CW states bit2 = 0
  #define R_START     0x3
  #define R_CW_BEGIN  0x1
  #define R_CW_NEXT   0x0
  #define R_CW_FINAL  0x2
  #define R_CCW_BEGIN 0x6
  #define R_CCW_NEXT  0x4
  #define R_CCW_FINAL 0x5

  // State transition table, row = current state, col = next state
  // Note the bit flags OR-ed with R_START at final sequence transisions
  const unsigned char ttable[8][4] = {
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},            // R_CW_NEXT
    {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},            // R_CW_BEGIN
    {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},   // R_CW_FINAL
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},            // R_START
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},            // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},  // R_CCW_FINAL
    {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},            // R_CCW_BEGIN
    {R_START,    R_START,     R_START,     R_START}             // iLLEGAL
  };

  // Get state of input pins from encoder
  unsigned char pinstate = (digitalRead(chA) << 1 | digitalRead(chB));

  // Determine new state from the pins and state table
  state = ttable[state & 0x07][pinstate];

  // Change count if a CW/CCW step has completed
  if (state & DIR_CW) cwFlag = true;
  if (state & DIR_CCW) ccwFlag = true;

}
#endif

//--------------------------------------------------------------------
void InitDisplay(void) {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void SetDisplayToText(void) {
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
}

void UpdateDisplay(void){

  display.clearDisplay();
  display.setCursor(0, 0);     // Start at top-left corner

  // Line 1
  display.print("Pwr: ");
  if (powerOnFlag)
    display.println("On");
  else
    display.println("Off");
  // Line 2
  display.print("Mode: ");
  switch (mode) {
    case Run:
      display.println("Run");
      break;
    case Fn1:
      display.println("Fn10");
      break;
    case Fn2:
      display.println("Fn20");
      break;
    case Addr:
      display.println("Addr");
      break;
    default:
      display.println("?");
      break;
  }

  switch(mode) {

    case Run:
      // Normal throttle operations
      // Line 3
      display.print("Loco: ");
      display.println(loco);
      // Line 4
      display.print("S/D: ");
      display.print(speed);
      display.print("/");
      if (brakedFlag) {
        display.println("B");
      } else if (dir == Forward) {
        display.println("F");
      } else {
        display.println("R");
      }
      break;

    case Fn1:
    case Fn2:
      // Enter digit 0-9 for decoder function
      // Line 3
      display.println("Enter 0-9");
      // Line 4
      display.println("Esc=quit");
      break;

    case Addr:
      // Enter decoder address
      // Line 3
      display.print("Addr:");
      display.println(loco);
      // Line 4
      display.println("Esc=quit");
      break;

    default:
      display.println("?");
      break;
  }

  display.display();

}
