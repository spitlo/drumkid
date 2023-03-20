/*  DrumKid firmware V1.2
 *  This code should work for DrumKid boards from V6 onwards.
 *  It might work with earlier versions with a bit of tweaking :) 
 */

// Include debouncing library to make buttons work reliably
#include "src/Bounce2/src/Bounce2.h"

// Include Mozzi library files for generating audio (custom version to allow reverse playback)
#include "src/MozziDK/src/MozziGuts.h"
#include "src/MozziDK/src/Sample.h"
#include "src/MozziDK/src/mozzi_rand.h"
#include "src/MozziDK/src/Oscil.h"
#include "src/MozziDK/src/tables/saw256_int8.h"

// Include drum beat pattern definition from separate file
#include "beats.h"

// Include sample data (generated via Python script)
#include "sample0.h"
#include "sample1.h"
#include "sample2.h"
#include "sample3.h"
#include "sample4.h"

// Declare samples as Mozzi sample objects
Sample<sample0_NUM_CELLS, AUDIO_RATE> sample0(sample0_DATA);
Sample<sample1_NUM_CELLS, AUDIO_RATE> sample1(sample1_DATA);
Sample<sample2_NUM_CELLS, AUDIO_RATE> sample2(sample2_DATA);
Sample<sample3_NUM_CELLS, AUDIO_RATE> sample3(sample3_DATA);
Sample<sample4_NUM_CELLS, AUDIO_RATE> sample4(sample4_DATA);

// Include EEPROM library for saving data
#include <EEPROM.h>

// declare buttons as Bounce objects
Bounce buttons[6];

// Debugging (https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824)
#define DEBUG 1  // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

// Mozzi control rate, measured in Hz, must be power of 2
// Try to keep this value as high as possible (256 is good) but reduce if having performance issues
#define CONTROL_RATE 128

// Define various values
#define NUM_KNOBS 4
#define NUM_LEDS 5
#define NUM_BUTTONS 6
#define NUM_SAMPLES 5
#define NUM_PARAM_GROUPS 1
#define SAVED_STATE_SLOT_BYTES 24
#define NUM_TAP_TIMES 8
#define MAX_BEATS_PER_BAR 8
#define MIN_SWING 0.5
#define MAX_SWING 0.75
#define TIME_SIGNATURE 64

// Define pin numbers
// Keep pin 9 for audio, but others can be changed to suit your breadboard layout
const byte ledPins[5] = { 2, 3, 11, 12, 13 };
const byte buttonPins[6] = { 4, 5, 6, 7, 8, 10 };
const byte analogPins[4] = { A0, A1, A2, A3 };

// Various other global variables
float nextPulseTime = 0.0;                                          // the time, in milliseconds, of the next pulse
float msPerPulse = 20.8333;                                         // time for one "pulse" (there are 24 pulses per quarter note, as defined in MIDI spec)
byte pulseNum = 0;                                                  // 0 to 23 (24ppqn, pulses per quarter note)
unsigned int stepNum = 0;                                           // 0 to 95 (max one bar of 8 beats, aka 96 pulses)
unsigned int numSteps;                                              // number of steps used - dependent on time signature
unsigned int specialOffset = 0;                                     // number of steps to offset pattern in random time signature mode
bool beatPlaying = false;                                           // true when beat is playing, false when not
byte noteDown = B00000000;                                          // keeps track of whether a MIDI note is currently down or up
bool syncReceived = false;                                          // has a sync/clock signal been received? (IMPROVE THIS LATER)
byte midiNotes[NUM_SAMPLES];                                        // MIDI note numbers
const byte defaultMidiNotes[NUM_SAMPLES] = { 36, 42, 38, 37, 43 };  // default MIDI note numbers
byte midiNoteCommands[NUM_SAMPLES];                                 // MIDI note on commands
byte sampleVolumes[NUM_SAMPLES] = { 0, 0, 0, 0, 0 };                // current sample volumes
byte storedValues[NUM_PARAM_GROUPS * NUM_KNOBS];                    // analog knob values, one for each parameter (so the value can be remembered even after switching groups)
byte firstLoop = true;                                              // allows special actions on first loop
byte secondLoop = false;                                            // allows special actions on second loop
byte controlSet = 0;                      // current active set of parameters
byte knobLocked = B11111111;              // record whether a knob's value is currently "locked" (i.e. won't alter effect until moved by a threshold amount)
byte analogValues[NUM_KNOBS];             // current analog (knob) values
byte initValues[NUM_KNOBS];               // initial parameter values
byte specialLedDisplayNum;                // binary number to display on LEDs (when needed)
unsigned long specialLedDisplayTime = 0;  // the last time the LEDs were told to display a number

// save/load variables
bool readyToChooseSaveSlot = false;
bool readyToChooseLoadSlot = false;
bool readyToChooseBank = false;
bool newStateLoaded = false;
byte activeBank = 0;
byte activeMidiSettingsNum = 0;

// Tempo variables
float tapTimes[NUM_TAP_TIMES];
byte nextTapIndex = 0;

// Parameter variables, each with a range of 0-255 unless otherwise noted
byte paramRandom;
int paramMidpoint;  // -255 to 255
int paramRange;     // 0 to 510
byte paramBeat;           // 0 to 23 (24 beats in total)
byte paramTimeSignature;  // 1 to 13 (3/4, 4/4, 5/4, 6/4, 7/4)
float paramTempo;
byte paramSwing = 0;  // 0 to 2 (0=straight, 1=approx quintuplet swing, 2=triplet swing)

// special global variables needed for certain parameters
// byte crushCompensation;
byte previousBeat;
byte previousTimeSignature;

#define MIN_TEMPO 40
#define MAX_TEMPO 295

#if MIN_TEMPO + 255 != MAX_TEMPO
#error "Tempo must have a range of exactly 255 otherwise I'll have to write more code"
#endif

// define which knob controls which parameter
// e.g. 0 is group 1 knob 1, 6 is group 2 knob 3
#define SWING 0
#define TEMPO 1
#define RANDOM 2

// define root notes
float rootNotes[13] = {
  138.5913155f,
  146.832384f,
  155.5634919f,
  164.8137785f,
  174.6141157f,
  184.9972114f,
  195.997718f,
  207.6523488f,
  220.0f,
  233.0818808f,
  246.9416506f,
  261.6255653f,
  277.182631f,
};

void setup() {
  byte i;

  checkEepromScheme();  // write defaults to memory if not previously done
  initialiseSettings(false);

  for (i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);  // set LED pins as outputs
  }

  randSeed((long)analogRead(4) * analogRead(5));  // A4 and A5 should be floating, use to seed random numbers
  startMozzi(CONTROL_RATE);

  // Initialise all buttons using Bounce2 library
  for (i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].interval(25);
    buttons[i].attach(buttonPins[i], INPUT_PULLUP);
  }

  // Set starting values for each parameter
  resetSessionToDefaults();

  Serial.begin(31250);    // begin serial communication for MIDI input/output
  D_SerialBegin(115200);  // And for debugging

  flashLeds();  // do a brief LED light show to show the unit is working
}

byte buttonsPressed = 0;
byte buttonGroup = 0;
byte lastButtonsPressed = 0;
byte menuState = 0;

void updateControl() {
  byte i;
  bool controlSetChanged = false;
  newStateLoaded = false;
  byte prevControlSet = controlSet;

  for (i = 0; i < 6; i++) {
    buttons[i].update();
  }
  for (i = 0; i < 6; i++) {
    bitWrite(buttonsPressed, i, !buttons[i].read());
    buttonGroup = buttonsPressed | buttonGroup;
  }

  // Start/stop
  if (buttons[0].fell()) {
    if (!beatPlaying) {
      startBeat();
    } else {
      stopBeat();
    }
  }

  // Tap tempo
  if (buttons[5].fell()) {
    tapTimes[nextTapIndex] = (float)millis();

    float firstTime;
    float timeTally = 0.0;
    byte validTimes = 0;
    bool keepChecking = true;
    for (i = 0; i < NUM_TAP_TIMES - 1 && keepChecking; i++) {
      byte thisTapIndex = (nextTapIndex + NUM_TAP_TIMES - i) % NUM_TAP_TIMES;
      byte lastTapIndex = (nextTapIndex + NUM_TAP_TIMES - i - 1) % NUM_TAP_TIMES;

      float thisTime = tapTimes[thisTapIndex] - tapTimes[lastTapIndex];
      if (i == 0) {
        firstTime = thisTime;
      }
      float timeCompare = firstTime / thisTime;
      if (tapTimes[lastTapIndex] > 0.0 && timeCompare > 0.8 && timeCompare < 1.2) {
        timeTally += thisTime;
        validTimes++;
      } else {
        keepChecking = false;
      }
    }
    if (validTimes >= 2) {
      float newPulseLength = (timeTally / validTimes) / 24;
      paramTempo = 2500.0 / newPulseLength;
      storedValues[TEMPO] = tempoToByte(paramTempo);
    }
    nextTapIndex = (nextTapIndex + 1) % NUM_TAP_TIMES;

    // Investigate this
    if (controlSet == TEMPO / NUM_KNOBS) {
      byte tempoKnobNum = TEMPO % NUM_KNOBS;
      bitWrite(knobLocked, tempoKnobNum, true);
      initValues[tempoKnobNum] = analogValues[tempoKnobNum];
    }
  }

  controlSet = 0;

  lastButtonsPressed = buttonsPressed;
  controlSetChanged = (prevControlSet != controlSet);

  msPerPulse = 2500.0 / paramTempo;

  // Perform pulse actions inside loop - stop beat if everything gets super overloaded
  byte numLoops = 0;
  while (!syncReceived && beatPlaying && millis() >= nextPulseTime) {
    if (numLoops < 5) {
      doPulseActions();
    }
    nextPulseTime = nextPulseTime + msPerPulse;
    if (numLoops >= 100) {
      beatPlaying = false;
    }
    numLoops++;
  }

  for (i = 0; i < NUM_KNOBS; i++) {
    if (firstLoop) {
      byte dummyReading = mozziAnalogRead(analogPins[i]);  // need to read pin once because first reading is not accurate
    } else {
      analogValues[i] = mozziAnalogRead(analogPins[i]) >> 2;
    }
  }

  if (controlSetChanged || secondLoop || newStateLoaded) {
    for (i = 0; i < NUM_KNOBS; i++) {
      bitWrite(knobLocked, i, true);
      initValues[i] = analogValues[i];
    }
  } else {
    for (i = 0; i < NUM_KNOBS; i++) {
      if (bitRead(knobLocked, i)) {
        int diff = initValues[i] - analogValues[i];
        if (diff < -5 || diff > 5) {
          bitWrite(knobLocked, i, false);
        }
      }
      if (!bitRead(knobLocked, i)) {
        storedValues[NUM_KNOBS * controlSet + i] = analogValues[i];
      }
    }
  }

  updateParameters();

  if (firstLoop) {
    firstLoop = false;
    secondLoop = true;
  } else {
    secondLoop = false;
  }

  if (!beatPlaying) {
    updateLeds();
  }
}

void doPulseActions() {
  Serial.write(0xF8);  // MIDI clock
  cancelMidiNotes();
  if (pulseNum % 24 == 0) {
    if (stepNum == 0) {
      numSteps = paramTimeSignature * 24;  // 24 pulses per beat
      specialOffset = 0;
    }
  }
  playPulseHits();
  updateLeds();
  incrementPulse();
  incrementStep();
}

void setDefaults() {
  paramRandom = storedValues[RANDOM];
  paramRange = 300;
  paramMidpoint = 0;

  // Pitch alteration (could do this in a more efficient way to reduce RAM usage)
  float newKickFreq = (float)sample0_SAMPLERATE / (float)sample0_NUM_CELLS;
  float newHatFreq = (float)sample1_SAMPLERATE / (float)sample1_NUM_CELLS;
  float newSnareFreq = (float)sample2_SAMPLERATE / (float)sample2_NUM_CELLS;
  float newRimFreq = (float)sample3_SAMPLERATE / (float)sample3_NUM_CELLS;
  float newTomFreq = (float)sample4_SAMPLERATE / (float)sample4_NUM_CELLS;
  sample0.setFreq(newKickFreq);
  sample1.setFreq(newHatFreq);
  sample2.setFreq(newSnareFreq);
  sample3.setFreq(newRimFreq);
  sample4.setFreq(newTomFreq);

  sample0.setStart(0);
  sample1.setStart(0);
  sample2.setStart(0);
  sample3.setStart(0);
  sample4.setStart(0);
  sample0.setEnd(sample0_NUM_CELLS);
  sample1.setEnd(sample1_NUM_CELLS);
  sample2.setEnd(sample2_NUM_CELLS);
  sample3.setEnd(sample3_NUM_CELLS);
  sample4.setEnd(sample4_NUM_CELLS);

  paramBeat = 1;

  paramTempo = byteToTempo(storedValues[TEMPO]);

  paramTimeSignature = map(TIME_SIGNATURE, 0, 256, 1, MAX_BEATS_PER_BAR + 2);

  paramSwing = storedValues[SWING]; // Used to be divided by 86;  // gives range of 0 to 2
}

void updateParameters() {
  paramTempo = byteToTempo(storedValues[TEMPO]);
  paramSwing = storedValues[SWING];
  paramRandom = storedValues[RANDOM];
}

void startBeat() {
  beatPlaying = true;
  pulseNum = 0;
  stepNum = 0;
  nextPulseTime = millis();
  Serial.write(0xFA);  // MIDI clock start
}

void stopBeat() {
  beatPlaying = false;
  Serial.write(0xFC);  // MIDI clock stop
  syncReceived = false;
}

void playPulseHits() {
  for (byte i = 0; i < 5; i++) {
    calculateNote(i);
  }
}

void calculateNote(byte sampleNum) {
  long thisVelocity = 0;
  // first, check for notes which are defined in the beat
  unsigned int effectiveStepNum = stepNum;
  // D_println(paramSwing); // 0-255
  // D_println(stepNum); // 0-95
  // if (paramSwing == 1) {
  //   if (stepNum % 12 == 6) effectiveStepNum = stepNum - 1;  // arbitrary...
  //   if (stepNum % 12 == 7) effectiveStepNum = stepNum - 1;
  // } else if (paramSwing == 2) {
  //   if (stepNum % 12 == 6) effectiveStepNum = stepNum - 1;  // arbitrary...
  //   if (stepNum % 12 == 8) effectiveStepNum = stepNum - 2;
  // }
  // effectiveStepNum = (specialOffset + effectiveStepNum) % 192;
  // Roger Linn swing. 50% is completely straight on the sixteenth
  // 100% would be (almost) on the next sixteenth.
  float normalizedSwing = normalize(paramSwing, 0, 255, MIN_SWING, MAX_SWING);
  // D_println(normalizedSwing);
  int swingAmount = (int)normalize(normalizedSwing, MIN_SWING, MAX_SWING, 0, 23);
  // This function hits on every pulse, then checks backwards (accounting for swing)
  // if there is a note for this sample, in this beat, on the step this pulse represents.
  // So if
  // D_println(swingAmount);
  if (effectiveStepNum % 6 == 0) {
    // beats only defined down to 16th notes not 32nd, hence %2 (CHANGE COMMENT)
    byte beatByte = pgm_read_byte(&beats[paramBeat][sampleNum][effectiveStepNum / 48]);  // 48...? was 16
    if (bitRead(beatByte, 7 - ((effectiveStepNum / 6) % 8))) {
      thisVelocity = 255;
    }
  }

  byte yesNoRand = rand(0, 256);
  long randomVelocity = 0;
  if (yesNoRand < paramRandom) {
    int lowerBound = paramMidpoint - paramRange / 2;
    int upperBound = paramMidpoint + paramRange / 2;
    randomVelocity = rand(lowerBound, upperBound);
    randomVelocity = constrain(randomVelocity, -255, 255);
  }
  thisVelocity += randomVelocity;
  thisVelocity = constrain(thisVelocity, 0, 255);
  triggerNote(sampleNum, thisVelocity);
}

void triggerNote(byte sampleNum, byte velocity) {
  if (velocity > 8) {  // don't bother with very quiet notes
    switch (sampleNum) {
      case 0:
        sample0.start();
        break;
      case 1:
        sample1.start();
        break;
      case 2:
        sample2.start();
        break;
      case 3:
        sample3.start();
        break;
      case 4:
        sample4.start();
        break;
    }
    sampleVolumes[sampleNum] = velocity;
    bitWrite(noteDown, sampleNum, true);
    playMidiNote(sampleNum, velocity);
  }
}

void playMidiNote(byte sampleNum, byte velocity) {
  Serial.write(midiNoteCommands[sampleNum]);  // note down command
  Serial.write(midiNotes[sampleNum]);         // note number
  Serial.write(velocity >> 1);                // velocity (scaled down to MIDI standard, 0-127)
}

void incrementPulse() {
  pulseNum++;
  if (pulseNum == 24) {
    pulseNum = 0;
  }
}

void incrementStep() {
  stepNum++;
  if (stepNum == numSteps) {
    stepNum = 0;
  }
}

void cancelMidiNotes() {
  byte i;
  for (i = 0; i < NUM_SAMPLES; i++) {
    if (bitRead(noteDown, i)) {
      Serial.write(midiNoteCommands[i]);  // note down command (zero velocity is equivalent to note up)
      Serial.write(midiNotes[i]);         // note number
      Serial.write(0x00);                 // zero velocity
      bitWrite(noteDown, i, false);
    }
  }
}

const byte atten = 9;
int updateAudio() {
  char asig = (
    (sampleVolumes[0] * sample0.next()) >> atten) +
    ((sampleVolumes[1] * sample1.next()) >> atten) +
    ((sampleVolumes[2] * sample2.next()) >> atten) +
    ((sampleVolumes[3] * sample3.next()) >> atten) +
    ((sampleVolumes[4] * sample4.next()) >> atten);
  // asig = (asig >> paramCrush) << crushCompensation;
  return asig;
}

byte thisMidiByte;
byte midiBytes[3];
byte currentMidiByte = 0;
void loop() {
  while (Serial.available()) {
    thisMidiByte = Serial.read();
    if (bitRead(thisMidiByte, 7)) {
      // status byte
      midiBytes[0] = thisMidiByte;
      if (thisMidiByte == 0xFA) {
        // clock start signal received
        startBeat();
      } else if (thisMidiByte == 0xFB) {
        // clock continue signal received
        startBeat();
      } else if (thisMidiByte == 0xFC) {
        // clock stop signal received
        stopBeat();
      } else if (thisMidiByte == 0xF8) {
        // clock signal received
        syncReceived = true;
        if (beatPlaying) {
          doPulseActions();
        }
      }
      currentMidiByte = 1;
    } else {
      // data byte
      midiBytes[currentMidiByte] = thisMidiByte;
      if ((midiBytes[0] >> 4) == 0xB && currentMidiByte == 2) {
        // CC message (any channel)
        if (midiBytes[1] >= 16 && midiBytes[1] < 32) {
          byte paramNum = midiBytes[1] - 16;
          byte paramSet = paramNum / 4;
          byte paramSetNum = paramNum % 4;
          storedValues[paramNum] = midiBytes[2] * 2;
          if (controlSet == paramSet) {
            bitWrite(knobLocked, paramSetNum, true);
            initValues[paramSetNum] = analogValues[paramSetNum];
          }
          updateParameters();
        }
      }
      currentMidiByte++;
    }
  }
  audioHook();  // main Mozzi function, calls updateAudio and updateControl
}

void flashLeds() {
  byte i, randByte;
  for (i = 0; i < 30; i++) {
    randByte = rand(0, 256);
    displayLedNum(randByte);
    while (millis() < (i + 1) * 25) {
      // do nothing here
      // this is a silly hack so that the delay() function doesn't get compiled
      // this is what happens when your program takes up 99%+ of the storage space
    }
  }
  displayLedNum(0);
}

void updateLeds() {
  if (millis() < specialLedDisplayTime + 750UL && specialLedDisplayTime > 0) {
    // show desired binary number for certain parameters where visual feedback is helpful
    displayLedNum(specialLedDisplayNum);
  } else if (beatPlaying && menuState == 0) {
    //if(stepNum==0||(paramTimeSignature==4&&stepNum==96)||(paramTimeSignature==6&&stepNum==72)) displayLedNum(B00000011);
    if (stepNum == 0) displayLedNum(B00000011);
    else if (stepNum % 24 == 0) displayLedNum(B00000010);
    else if (stepNum % 24 == 3) displayLedNum(B00000000);
  } else {
    displayLedNum(B00000000);
  }
}

void displayLedNum(byte displayNum) {
  byte i;
  for (i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], bitRead(displayNum, i));
  }
}

void specialLedDisplay(byte displayNum, bool isBinary) {
  if (!firstLoop && !secondLoop) {
    if (isBinary) {
      specialLedDisplayNum = displayNum;
    } else {
      specialLedDisplayNum = B00000000;
      bitWrite(specialLedDisplayNum, displayNum % NUM_LEDS, HIGH);
    }
    specialLedDisplayTime = millis();
  }
}

float byteToTempo(byte tempoByte) {
  float tempoFloat;
  if (tempoByte <= 192) {
    tempoFloat = 10.0 + tempoByte;
  } else {
    tempoFloat = 202.0 + 12.66667 * (tempoByte - 192.0);
  }
  return tempoFloat;
}

byte tempoToByte(float tempoFloat) {
  byte tempoByte;
  if (tempoFloat <= 202.0) {
    tempoByte = ((byte)tempoFloat) - 10;
  } else {
    tempoByte = (byte)((tempoFloat - 202.0) / 12.66667) + 192;
  }
  return tempoByte;
}

float normalize(float num, float fromMin, float fromMax, float toMin, float toMax) {
  return toMin + (num - fromMin) / (fromMax - fromMin) * (toMax - toMin);
}

void chooseBank(byte newBank) {
  activeBank = newBank;
  readyToChooseBank = false;
}

void resetSessionToDefaults() {
  // Set starting values for each parameter
  storedValues[RANDOM] = 128;
  // storedValues[ZOOM] = 150;
  // storedValues[RANGE] = 0;
  // storedValues[MIDPOINT] = 128;

  // storedValues[PITCH] = 160;
  // storedValues[CRUSH] = 255;
  // storedValues[CROP] = 255;
  // storedValues[DROP] = 128;

  // storedValues[BEAT] = 0;
  // TIME_SIGNATURE = 64;  // equates to 4/4
  storedValues[SWING] = 128;
  storedValues[TEMPO] = 72;  // actually equates to 120BPM

  newStateLoaded = true;
  setDefaults();
}

void loadParams(byte slotNum) {
  readyToChooseLoadSlot = false;
  newStateLoaded = true;
  for (byte i = 0; i < NUM_PARAM_GROUPS * NUM_KNOBS; i++) {
    if (!(i == TEMPO && beatPlaying)) {
      byte thisValue = EEPROM.read((slotNum + 6 * activeBank) * SAVED_STATE_SLOT_BYTES + i);
      storedValues[i] = thisValue;
    }
  }
  updateParameters();
}

void saveParams(byte slotNum) {
  // 1024 possible byte locations in EEPROM
  // currently need 16 bytes per saved state (4 groups of 4 params, 1 byte per param)
  // 36 possible save slots (6 banks of 6 slots)
  // need to leave space for some possible extra saved state data in future
  // allot 24 bytes per saved state (vaguely round number in binary), taking up total of 864 bytes
  readyToChooseSaveSlot = false;
  for (byte i = 0; i < NUM_PARAM_GROUPS * NUM_KNOBS; i++) {
    EEPROM.write((slotNum + 6 * activeBank) * SAVED_STATE_SLOT_BYTES + i, storedValues[i]);
  }
}

// prior to firmware version 1.1, no way of knowing whether older firmware was previously installed
// from V1.1 onwards, new special address beyond the 864 bytes reserved for storing sessions
// Slot 864-865: if these bytes have values 119 and 206 respectively, we are using the new memory system (1 in 65,025 chance of these being randomly correct)
// Slot 866: current version (0 = V1.1, 1 = V1.2)
// Slots 867-871: reserved for other version stuff if needed
// Slots 872-879: MIDI note numbers for channels 1 through 5 (and another 3 reserved)
// Slots 880-887: MIDI channel numbers for channels 1 through 5 (and another 3 reserved)
// Slots 888+: free for future features

void checkEepromScheme() {
  byte i;
  byte check1 = EEPROM.read(864);
  byte check2 = EEPROM.read(865);
  if (check1 == 119 && check2 == 206) {
    // all good, already using the new EEPROM scheme

    // update firmware version in memory if needed
    if (EEPROM.read(866) != 1) EEPROM.write(866, 1);
  } else {
    // prepare EEPROM for new scheme
    EEPROM.write(864, 119);
    EEPROM.write(865, 206);
    EEPROM.write(866, 1);
    // write default MIDI note/channel settings
    for (i = 0; i < NUM_SAMPLES; i++) {
      EEPROM.write(872 + i, defaultMidiNotes[i]);
      EEPROM.write(880 + i, 9);  // channel 10 (actually 9 because zero-indexed) is default channel to use
    }
  }
}

void initialiseSettings(bool isReset) {
  // must run checkEepromScheme() BEFORE this function
  byte i;
  for (i = 0; i < NUM_SAMPLES; i++) {
    if (isReset) {
      EEPROM.write(872 + i, defaultMidiNotes[i]);
      EEPROM.write(880 + i, 9);
    }
    midiNotes[i] = EEPROM.read(872 + i);
    midiNoteCommands[i] = 0x90 + EEPROM.read(880 + i);
  }
}
