#include <Arduino.h>    // Include Arduino core library
#include <notes.h>      // Include the note definitions
#include "stm8s.h"      // Include STM8S standard definitions
#include "stm8s_awu.h"  // Include AWU definitions

// Define constants if not defined
#ifndef CLK_PCKENR1_AWU
#define CLK_PCKENR1_AWU ((uint8_t)0x20) // Bit 5
#endif

// Board pinout definitions
#define powerPin 1        // PA2 - Main power detection pin (HIGH when power is connected)
#define buttonPin 3       // PA3 - Onboard button pin (LOW when pressed)
#define whiteLedPin 5     // PC3 - Large white LED pin
#define redLedPin 8       // PC6 - Red LED pin (onboard LED)
#define buzzerPin 13      // PD4 - Buzzer control pin
#define signalPin 7       // PC5 - Signal pin (from flight controller)

// Constants
const unsigned long disarmDelay = 10000;               // In milliseconds, time to apply power to disarm
const unsigned long armDisarmPressDuration = 5000;     // In milliseconds, duration to hold button for arm/disarm
const unsigned long alarmInterval = 30000;             // In milliseconds, interval between alarm beeps (30 seconds)
const unsigned long softAlarmDuration = 120000;        // In milliseconds, duration of soft alarms before loud alarms (2 minutes)
const unsigned long redLedFlashInterval = 500;         // In milliseconds, interval for red LED flashing
const unsigned long whiteLedFlashDuration = 200;       // In milliseconds, duration for white LED flash
const unsigned long loudAlarmDuration = 500;           // In milliseconds, duration for loud alarm buzzer sound
const int loudAlarmFrequency = 2000;                   // In Hz, frequency for loud alarm buzzer sound

// Variables
bool armed = false;                           // Armed status
bool powerOutage = false;                     // Power outage detection
bool buttonPressed = false;                   // Button pressed flag
unsigned long buttonPressStartTime = 0;       // Timestamp when button was pressed
unsigned long lastAlarmMillis = 0;            // Timestamp of the last alarm
unsigned long powerCutMillis = 0;             // Timestamp when power was cut
unsigned long lastRedLedToggleTime = 0;       // Timestamp for red LED flashing

// Function prototypes
void arm();
void disarm();
void play_soft_alarm();
void play_alarm();
void play_signal();
void play_rearming();
void play_disarming();
void buzz(int targetPin, long frequency, long length);
void enter_active_halt_mode();
void configure_AWU();
void initialize_hardware();
void handle_power_state();
void handle_button_press();
void handle_alarm();
void handle_signal_buzzer();

// Interrupt Service Routine for Active-Halt Wake-Up via AWU
void AWU_IRQHandler(void) __interrupt(1) {
    // Clear the AWU wake-up flag
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUF);
}

void NonHandledInterrupt(void) __interrupt(0) {
    // In case of unexpected interrupts, do nothing
}

// Setup function
void setup() {
    initialize_hardware();

    // Enable interrupts
    enableInterrupts();

    // Initial power state handling
    int mainPowerState = digitalRead(powerPin); // Read main power status (HIGH when connected)
    if (mainPowerState == HIGH) {
        arm();
    } else {
        disarm();
    }
}

// Main loop
void loop() {
    // Since we're using Active-Halt Mode, the MCU wakes up periodically via AWU
    // and resumes execution from here.

    handle_power_state();

    // Handle button presses
    handle_button_press();

    // Handle alarms if armed
    if (armed) {
        handle_alarm();
    }

    // Handle signal buzzer if active
    handle_signal_buzzer();

    // Enter low power mode again
    enter_active_halt_mode();
}

// Function to initialize hardware settings
void initialize_hardware() {
    // Initialize pin modes
    pinMode(buttonPin, INPUT_PULLUP);        // Button pin as input with pull-up resistor
    pinMode(powerPin, INPUT);                // Power pin as input
    pinMode(signalPin, INPUT);               // Signal pin as input
    pinMode(redLedPin, OUTPUT);              // Red LED pin as output
    pinMode(whiteLedPin, OUTPUT);            // Large white LED pin as output
    pinMode(buzzerPin, OUTPUT);              // Buzzer pin as output

    // Set initial states
    digitalWrite(redLedPin, LOW);            // Turn off red LED
    digitalWrite(whiteLedPin, LOW);          // Turn off white LED
    digitalWrite(buzzerPin, LOW);            // Ensure buzzer is off

    // Configure AWU for periodic wake-up
    configure_AWU();
}

// Function to handle power state changes
void handle_power_state() {
    int mainPowerState = digitalRead(powerPin); // Read main power status (HIGH when connected)

    if (mainPowerState == HIGH) {
        if (powerOutage) {
            powerOutage = false;
            digitalWrite(redLedPin, HIGH); // Update red LED based on armed status
        }
    } else {
        if (armed && !powerOutage) {
            powerCutMillis = millis();                // Record power cut time
            powerOutage = true;
            lastAlarmMillis = powerCutMillis;         // Initialize last alarm timestamp
            digitalWrite(redLedPin, HIGH);            // Ensure red LED is on
        }
    }
}

// Function to handle button presses
void handle_button_press() {
    bool isButtonPressed = (digitalRead(buttonPin) == LOW);
    unsigned long currentTime = millis();

    if (isButtonPressed) {
        if (!buttonPressed) {
            // Button was just pressed
            buttonPressed = true;
            buttonPressStartTime = currentTime;
        } else {
            // Button is being held
            if ((currentTime - buttonPressStartTime) >= armDisarmPressDuration) {
                // Button has been held long enough to arm/disarm
                if (armed) {
                    disarm();
                } else {
                    arm();
                }
                buttonPressed = false; // Reset the flag to prevent repeated triggers
            }
        }
    } else {
        // Button is not pressed
        buttonPressed = false;
    }
}

// Function to handle alarms when power is cut
void handle_alarm() {
    if (powerOutage) {
        unsigned long currentMillis = millis();
        unsigned long powerOutageDuration = currentMillis - powerCutMillis; // In milliseconds

        if (powerOutageDuration <= softAlarmDuration) {
            // Within soft alarm duration, play soft beep
            if ((currentMillis - lastAlarmMillis) >= alarmInterval) {
                play_soft_alarm();
                lastAlarmMillis = currentMillis;
            }
        } else {
            // After soft alarm duration, play loud alarm
            if ((currentMillis - lastAlarmMillis) >= alarmInterval) {
                play_alarm();
                lastAlarmMillis = currentMillis;
            }
        }
    }
}

// Function to handle the signal buzzer
void handle_signal_buzzer() {
    if (digitalRead(signalPin) == LOW) {
        play_signal();
    }
}

// Function to arm the buzzer
void arm() {
    armed = true;
    powerOutage = false;
    digitalWrite(redLedPin, HIGH);            // Turn on red LED to indicate armed status
    play_rearming();                          // Play arming sound
}

// Function to disarm the buzzer
void disarm() {
    armed = false;
    powerOutage = false;
    digitalWrite(redLedPin, LOW);             // Turn off red LED to indicate disarmed status
    play_disarming();                         // Play disarming sound
}

// Function to generate a tone on the buzzer
void buzz(int targetPin, long frequency, long length) {
    // targetPin: the pin connected to the buzzer
    // frequency: the frequency of the tone in Hz
    // length: the duration of the tone in milliseconds

    if (frequency == 0) {
        digitalWrite(targetPin, LOW);
        return;
    }

    long delayValue = 1000000 / frequency / 2;    // Calculate delay for half period in microseconds
    long numCycles = frequency * length / 1000;   // Calculate number of cycles for the tone

    for (long i = 0; i < numCycles; i++) {       // For each cycle
        digitalWrite(targetPin, HIGH);             // Turn buzzer on
        delayMicroseconds(delayValue);             // Wait for half period
        digitalWrite(targetPin, LOW);              // Turn buzzer off
        delayMicroseconds(delayValue);             // Wait for half period
    }
    digitalWrite(targetPin, LOW);                // Ensure buzzer is off after tone
}

// Function to play arming sound sequence
void play_rearming() {
    int melody[] = { NOTE_A4, NOTE_A5, NOTE_C5, NOTE_C6 }; // Notes for arming sound (higher notes)
    int tempo[] = { 12, 12, 12, 12 };                      // Tempo for each note
    int size = sizeof(melody) / sizeof(int);               // Calculate the number of notes

    for (int thisNote = 0; thisNote < size; thisNote++) {
        int noteDuration = 1000 / tempo[thisNote];           // Calculate note duration
        buzz(buzzerPin, melody[thisNote], noteDuration);     // Play the note
        int pauseBetweenNotes = noteDuration * 1.30;         // Calculate pause duration
        delay(pauseBetweenNotes);                            // Wait before next note
        buzz(buzzerPin, 0, noteDuration);                    // Ensure buzzer is off
    }
}

// Function to play disarming sound sequence
void play_disarming() {
    int melody[] = { NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4 }; // Notes for disarming sound
    int tempo[] = { 12, 12, 12, 12 };                      // Tempo for each note
    int size = sizeof(melody) / sizeof(int);               // Calculate the number of notes

    for (int thisNote = 0; thisNote < size; thisNote++) {
        int noteDuration = 1000 / tempo[thisNote];           // Calculate note duration
        buzz(buzzerPin, melody[thisNote], noteDuration);     // Play the note
        int pauseBetweenNotes = noteDuration * 1.30;         // Calculate pause duration
        delay(pauseBetweenNotes);                            // Wait before next note
        buzz(buzzerPin, 0, noteDuration);                    // Ensure buzzer is off
    }
}

// Function to play alarm sound sequence (loud alarm)
void play_alarm() {
    // Play loud alarm tone
    buzz(buzzerPin, loudAlarmFrequency, loudAlarmDuration); // Play the loud alarm tone

    // Briefly flash the white LED
    digitalWrite(whiteLedPin, HIGH);
    delay(whiteLedFlashDuration);                          // Short flash duration
    digitalWrite(whiteLedPin, LOW);
}

// Function to play soft alarm sound
void play_soft_alarm() {
    int melody[] = { NOTE_C6 };   // Note for soft alarm
    int tempo[] = { 16 };         // Tempo for soft alarm
    int size = sizeof(melody) / sizeof(int);       // Calculate the number of notes

    for (int thisNote = 0; thisNote < size; thisNote++) {
        int noteDuration = 1000 / tempo[thisNote];       // Calculate note duration
        buzz(buzzerPin, melody[thisNote], noteDuration); // Play the note
        int pauseBetweenNotes = noteDuration * 1.30;     // Calculate pause duration
        delay(pauseBetweenNotes);                        // Wait before next note
        buzz(buzzerPin, 0, noteDuration);                // Ensure buzzer is off
    }
}

// Function to play signal sound sequence
void play_signal() {
    int melody[] = { NOTE_DS8 };    // Note for signal
    int tempo[] = { 16 };           // Tempo for signal
    int size = sizeof(melody) / sizeof(int);          // Calculate the number of notes

    for (int thisNote = 0; thisNote < size; thisNote++) {
        int noteDuration = 1000 / tempo[thisNote];           // Calculate note duration
        buzz(buzzerPin, melody[thisNote], noteDuration);     // Play the note
        int pauseBetweenNotes = noteDuration * 1.30;         // Calculate pause duration
        delay(pauseBetweenNotes);                            // Wait before next note
        buzz(buzzerPin, 0, noteDuration);                    // Ensure buzzer is off
    }
}

// Function to configure AWU for periodic wake-up
void configure_AWU() {
    // Enable the AWU clock
    CLK->PCKENR1 |= CLK_PCKENR1_AWU; // Enable clock for AWU (Bit 5)

    // Disable AWU before configuration
    AWU->CSR = 0;

    // Set the AWU timebase to 2 seconds
    AWU->TBR = AWU_TIMEBASE_2S;

    // Enable AWU
    AWU->CSR |= AWU_CSR_AWUEN;
}

// Function to enter Active-Halt Mode
void enter_active_halt_mode() {
    // Clear the AWU wake-up flag
    AWU->CSR &= (uint8_t)(~AWU_CSR_AWUF);

    // Enter Active-Halt Mode
    __asm__("halt");
}
