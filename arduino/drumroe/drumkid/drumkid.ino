/*  DrumKid firmware V1.2
 *  This code should work for DrumKid boards from V6 onwards.
 *  It might work with earlier versions with a bit of tweaking :)
 */

// Include debouncing library to make buttons work reliably
#include "src/Bounce2/src/Bounce2.h"

// Include Mozzi library files for generating audio (custom version to allow reverse playback)
#include "src/MozziDK/src/LowPassFilter.h"
#include "src/MozziDK/src/MozziGuts.h"
#include "src/MozziDK/src/Oscil.h"
#include "src/MozziDK/src/Sample.h"
#include "src/MozziDK/src/mozzi_rand.h"
// #include "src/MozziDK/src/tables/saw256_int8.h"
#include "src/MozziDK/src/tables/cos256_int8.h"

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

// Filter
Oscil<COS256_NUM_CELLS, CONTROL_RATE> kFilterMod(COS256_DATA);
LowPassFilter rf;

// Include EEPROM library for saving data
#include <EEPROM.h>

// declare buttons as Bounce objects
Bounce buttons[6];

// Debugging (https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824)
#define DEBUG 1 // SET TO 0 OUT TO REMOVE TRACES

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
#define CONTROL_RATE 256

// Define various values
#define NUM_KNOBS 4
#define NUM_LEDS 5
#define NUM_BUTTONS 6
#define NUM_SAMPLES 5
#define NUM_PARAM_GROUPS 1
#define SAVED_STATE_SLOT_BYTES 24
#define NUM_TAP_TIMES 8
#define MIN_SWING 0.25
#define MAX_SWING 0.75
#define MIN_TEMPO 40
#define MAX_TEMPO 295

#if MIN_TEMPO + 255 != MAX_TEMPO
#error "Tempo must have a range of exactly 255 otherwise I'll have to write more code"
#endif

// Define which knob controls which parameter
// E.g. 0 is group 1 knob 1, 6 is group 2 knob 3
#define SWING 0
#define TEMPO 1
#define RANDOM 2
#define SLOP 3

// Define pin numbers
// Keep pin 9 for audio, but others can be changed to suit your breadboard layout
const byte ledPins[5] = {2, 3, 11, 12, 13};
const byte buttonPins[6] = {4, 5, 6, 7, 8, 10};
const byte analogPins[4] = {A0, A1, A2, A3};

// Various other global variables
float swingIncrement = 0.082; // MAX_SWING - MIN_SWING / 11;
float nextPulseTime = 0.0;    // the time, in milliseconds, of the next pulse
float msPerPulse =
    20.8333; // time for one "pulse" (there are 24 pulses per quarter note, as defined in MIDI spec)
byte pulseNum = 0;           // 0 to 23 (24ppqn, pulses per quarter note)
unsigned int stepNum = 0;    // 0 to 95 (max one bar of 8 beats, aka 96 pulses)
unsigned int numSteps;       // number of steps used - dependent on time signature
bool beatPlaying = false;    // true when beat is playing, false when not
byte noteDown = B00000000;   // keeps track of whether a MIDI note is currently down or up
bool syncReceived = false;   // has a sync/clock signal been received? (IMPROVE THIS LATER)
byte midiNotes[NUM_SAMPLES]; // MIDI note numbers
const byte defaultMidiNotes[NUM_SAMPLES] = {36, 42, 38, 37, 43}; // default MIDI note numbers
byte midiNoteCommands[NUM_SAMPLES];                              // MIDI note on commands
byte sampleVolumes[NUM_SAMPLES] = {0, 0, 0, 0, 0};               // current sample volumes
byte storedValues[NUM_PARAM_GROUPS *
                  NUM_KNOBS]; // analog knob values, one for each parameter (so the value can be
                              // remembered even after switching groups)
byte firstLoop = true;        // allows special actions on first loop
byte secondLoop = false;      // allows special actions on second loop
byte controlSet = 0;          // current active set of parameters
byte knobLocked = B11111111;  // record whether a knob's value is currently "locked" (i.e. won't
                              // alter effect until moved by a threshold amount)
byte analogValues[NUM_KNOBS]; // current analog (knob) values
byte initValues[NUM_KNOBS];   // initial parameter values
byte specialLedDisplayNum;    // binary number to display on LEDs (when needed)
unsigned long specialLedDisplayTime = 0; // the last time the LEDs were told to display a number

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
int paramMidpoint;       // -255 to 255
int paramRange;          // 0 to 510
byte paramTimeSignature; // 12 to 16
byte paramCrush;
byte paramFilter;
byte crushCompensation;
byte paramSwing; // 0 to 1 (50%=straight, 66%=triplet swing etc)
byte paramTempo;
byte paramRandom;
byte paramSlop;
bool doSlop = false;

void setup() {
  byte i;

  checkEepromScheme(); // write defaults to memory if not previously done
  initialiseSettings(false);

  for (i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT); // set LED pins as outputs
  }

  randSeed((long)analogRead(4) *
           analogRead(5)); // A4 and A5 should be floating, use to seed random numbers
  startMozzi(CONTROL_RATE);

  kFilterMod.setFreq(1.1f);

  // Initialise all buttons using Bounce2 library
  for (i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].interval(25);
    buttons[i].attach(buttonPins[i], INPUT_PULLUP);
  }

  // Set starting values for each parameter
  resetSessionToDefaults();

  Serial.begin(31250);   // begin serial communication for MIDI input/output
  D_SerialBegin(115200); // And for debugging

  flashLeds(); // do a brief LED light show to show the unit is working
}

byte buttonsPressed = 0;
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
      byte dummyReading = mozziAnalogRead(
          analogPins[i]); // need to read pin once because first reading is not accurate
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
    storedValues[NUM_KNOBS * controlSet + i] = analogValues[i];
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
  Serial.write(0xF8); // MIDI clock
  cancelMidiNotes();
  if (pulseNum % 24 == 0) {
    if (stepNum == 0) {
      numSteps = paramTimeSignature * 24; // 24 pulses per beat
    }
  }
  playPulseHits();
  updateLeds();
  incrementPulse();
  incrementStep();
}

void setDefaults() {
  paramRange = 300;
  paramMidpoint = 0;

  paramTimeSignature = 4; // For now... could be 3 as well, right? Or 3, 3.25, 3.5, 3.75 and 4?

  paramTempo = 72;
  paramSwing = 128;
  paramRandom = 128;
  paramSlop = 128;
}

void updateParameters() {
  paramTempo = byteToTempo(storedValues[TEMPO]);
  paramSwing = storedValues[SWING];
  paramRandom = storedValues[RANDOM];
  paramSlop = storedValues[SLOP];

  float normalizedSlop = normalize(paramSlop, 0, 255, 0.8, -0.8);

  doSlop = paramSlop < 124 || paramSlop > 132;

  // Pitch alteration (could do this in a more efficient way to reduce RAM usage)
  float newKickFreq = (float)sample0_SAMPLERATE / (float)sample0_NUM_CELLS;
  float newHatFreq = (float)sample1_SAMPLERATE / (float)sample1_NUM_CELLS;
  float newSnareFreq = (float)sample2_SAMPLERATE / (float)sample2_NUM_CELLS;
  float newRimFreq = (float)sample3_SAMPLERATE / (float)sample3_NUM_CELLS;
  float newTomFreq = (float)sample4_SAMPLERATE / (float)sample4_NUM_CELLS;
  sample0.setFreq(doSlop ? newKickFreq - normalizedSlop : newKickFreq);
  sample1.setFreq(doSlop ? newHatFreq - normalizedSlop : newHatFreq);
  sample2.setFreq(doSlop ? newSnareFreq - normalizedSlop : newSnareFreq);
  sample3.setFreq(doSlop ? newRimFreq - normalizedSlop : newRimFreq);
  sample4.setFreq(doSlop ? newTomFreq - normalizedSlop : newTomFreq);

  sample0.setStart(doSlop ? 0 + (normalizedSlop / 3) : 0);
  sample1.setStart(doSlop ? 0 + (normalizedSlop / 3) : 0);
  sample2.setStart(doSlop ? 0 + (normalizedSlop / 3) : 0);
  sample3.setStart(doSlop ? 0 + (normalizedSlop / 3) : 0);
  sample4.setStart(doSlop ? 0 + (normalizedSlop / 3) : 0);
  sample0.setEnd(sample0_NUM_CELLS);
  sample1.setEnd(sample1_NUM_CELLS);
  sample2.setEnd(sample2_NUM_CELLS);
  sample3.setEnd(sample3_NUM_CELLS);
  sample4.setEnd(sample4_NUM_CELLS);

  if (doSlop) {
    // Bit crush! high value = clean (8 bits), low value = dirty (1 bit?)
    paramCrush = (paramSlop >> 7);
    crushCompensation = paramCrush;
    if (paramCrush >= 6) {
      crushCompensation--;
    }
    if (paramCrush >= 7) {
      crushCompensation--;
    }

    // Filter
    paramFilter = 64 - (paramSlop >> 1);
    D_println(paramFilter);
    byte cutoff_freq = paramFilter + kFilterMod.next() / 2;
    rf.setCutoffFreq(cutoff_freq);
  }
}

void startBeat() {
  beatPlaying = true;
  pulseNum = 0;
  stepNum = 0;
  nextPulseTime = millis();
  Serial.write(0xFA); // MIDI clock start
}

void stopBeat() {
  beatPlaying = false;
  Serial.write(0xFC); // MIDI clock stop
  syncReceived = false;
}

void playPulseHits() {
  for (byte i = 0; i < 5; i++) {
    calculateNote(i);
  }
}

bool readNote(int sample, int sixteenth) {
  // This function will later read the 16x5 button matrix through
  // a cluster of MCP23017-E/SP I/O expanders.
  // For now, just read the multidimensional array in beats.h
  return beat[sample][sixteenth] == 1;
}

void calculateNote(byte sampleNum) {
  // Default to not playing audibly
  long thisVelocity = 0;
  // Normalize swing (0-255) to a number ranging from MIN_SWING to MAX_SWING
  // (Swing is described as 0-1 where 0.5 is of course 50%)
  float normalizedSwing = normalize(paramSwing, 0, 255, MIN_SWING, MAX_SWING);
  // Normalize randomicity to a number between -128 and 128. Negative numbers
  // mean we subtract existing notes randomly. Positive numbers mean we add
  // non-existing notes randomly.
  float normalizedRandomicity = normalize(paramRandom, 0, 255, -128, 128);
  // This function hits on every pulse, then checks backwards (accounting for swing)
  // if there is a note for this sample, in this beat, on the step this pulse represents.
  if (stepNum % 12 == 0) {
    if (readNote(sampleNum, stepNum / 6)) {
      thisVelocity = 255;
    }
    // If we’re in subtractive randomicity, throw the dice on whether
    // to play note or note.
    if (normalizedRandomicity < 0) {
      int yesNoRand = rand(-128, 0);
      if (yesNoRand > normalizedRandomicity) {
        thisVelocity = 0;
      }
    }
  } else {
    // Calculate what actual sixteenth this pulse represents based on
    // what percentage of swing we’re working in.
    unsigned int sixteenth = 0;
    if (normalizedSwing < swingIncrement * 1) {
      if (stepNum % 12 == 1) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 2) {
      if (stepNum % 12 == 2) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 3) {
      if (stepNum % 12 == 3) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 4) {
      if (stepNum % 12 == 4) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 5) {
      if (stepNum % 12 == 5) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing == 0.50) {
      if (stepNum % 12 == 6) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 7) {
      if (stepNum % 12 == 7) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 8) {
      if (stepNum % 12 == 8) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 9) {
      if (stepNum % 12 == 9) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing < swingIncrement * 10) {
      if (stepNum % 12 == 10) {
        sixteenth = stepNum / (stepNum % 12);
      }
    } else if (normalizedSwing >= swingIncrement * 11) {
      if (stepNum % 12 == 11) {
        sixteenth = stepNum / (stepNum % 12);
      }
    }

    if (sixteenth > 0) {
      if (readNote(sampleNum, sixteenth - 1)) {
        if (normalizedSwing == 0.50) {
          thisVelocity = 255;
        } else {
          // Swing notes get a little velocity flair
          thisVelocity = rand(240, 255);
        }
        // If we’re in subtractive randomicity, throw the dice on whether
        // to play note or note.
        if (normalizedRandomicity < 0) {
          int yesNoRand = rand(-128, 0);
          if (yesNoRand > normalizedRandomicity) {
            thisVelocity = 0;
          }
        }
      } else {
        // Only do this if we’re in additive randomicity
        if (normalizedRandomicity > 0) {
          byte yesNoRand = rand(0, 128);
          long randomVelocity = 0;
          if (yesNoRand < paramRandom) {
            int lowerBound = paramMidpoint - paramRange / 2;
            int upperBound = paramMidpoint + paramRange / 2;
            randomVelocity = rand(lowerBound, upperBound);
            randomVelocity = constrain(randomVelocity, -255, 255);
          }
          thisVelocity += randomVelocity;
        }
      }
    }
  }

  thisVelocity = constrain(thisVelocity, 0, 255);
  triggerNote(sampleNum, thisVelocity);
}

void triggerNote(byte sampleNum, byte velocity) {
  if (velocity > 8) { // Don't bother with very quiet notes
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
  Serial.write(midiNoteCommands[sampleNum]); // Note down command
  Serial.write(midiNotes[sampleNum]);        // Note number
  Serial.write(velocity >> 1);               // Velocity (scaled down to MIDI standard, 0-127)
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
      Serial.write(
          midiNoteCommands[i]);   // Note down command (zero velocity is equivalent to note up)
      Serial.write(midiNotes[i]); // Note number
      Serial.write(0x00);         // Zero velocity
      bitWrite(noteDown, i, false);
    }
  }
}

const byte atten = 9;
int updateAudio() {
  char asig = ((sampleVolumes[0] * sample0.next()) >> atten) +
              ((sampleVolumes[1] * sample1.next()) >> atten) +
              ((sampleVolumes[2] * sample2.next()) >> atten) +
              ((sampleVolumes[3] * sample3.next()) >> atten) +
              ((sampleVolumes[4] * sample4.next()) >> atten);
  asig = (asig >> paramCrush) << crushCompensation;
  return doSlop ? rf.next(asig) : asig;
}

byte thisMidiByte;
byte midiBytes[3];
byte currentMidiByte = 0;
void loop() {
  while (Serial.available()) {
    thisMidiByte = Serial.read();
    if (bitRead(thisMidiByte, 7)) {
      // Status byte
      midiBytes[0] = thisMidiByte;
      if (thisMidiByte == 0xFA) {
        // Clock start signal received
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
  audioHook(); // main Mozzi function, calls updateAudio and updateControl
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
    // if(stepNum==0||(paramTimeSignature==4&&stepNum==96)||(paramTimeSignature==6&&stepNum==72))
    // displayLedNum(B00000011);
    if (stepNum == 0)
      displayLedNum(B00000011);
    else if (stepNum % 24 == 0)
      displayLedNum(B00000010);
    else if (stepNum % 24 == 3)
      displayLedNum(B00000000);
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
  storedValues[SWING] = 128;
  storedValues[TEMPO] = 72;

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
// Slot 864-865: if these bytes have values 119 and 206 respectively, we are using the new memory
// system (1 in 65,025 chance of these being randomly correct) Slot 866: current version (0 = V1.1,
// 1 = V1.2) Slots 867-871: reserved for other version stuff if needed Slots 872-879: MIDI note
// numbers for channels 1 through 5 (and another 3 reserved) Slots 880-887: MIDI channel numbers for
// channels 1 through 5 (and another 3 reserved) Slots 888+: free for future features

void checkEepromScheme() {
  byte i;
  byte check1 = EEPROM.read(864);
  byte check2 = EEPROM.read(865);
  if (check1 == 119 && check2 == 206) {
    // all good, already using the new EEPROM scheme

    // update firmware version in memory if needed
    if (EEPROM.read(866) != 1)
      EEPROM.write(866, 1);
  } else {
    // prepare EEPROM for new scheme
    EEPROM.write(864, 119);
    EEPROM.write(865, 206);
    EEPROM.write(866, 1);
    // write default MIDI note/channel settings
    for (i = 0; i < NUM_SAMPLES; i++) {
      EEPROM.write(872 + i, defaultMidiNotes[i]);
      EEPROM.write(880 + i,
                   9); // channel 10 (actually 9 because zero-indexed) is default channel to use
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
