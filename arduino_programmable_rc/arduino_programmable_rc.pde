////////////////////////////////////////////////////////////////////////////////////////////////
//    Copyright (C) 2012 Jon Bennett
//
//    This file is part of Programmable RC Car Controller.
//
//    http://www.jbprojects.net/articles/programmable-rc/
//
//    Programmable RC Car Controller is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    Programmable RC Car Controller is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with Programmable RC Car Controller.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////////////////////

#include <Time.h>

#define DEBUG (1)

// Pin Functions
#define FORWARD_PIN   (9)
#define BACKWARD_PIN (10)
#define LEFT_PIN     (11)
#define RIGHT_PIN    (12)

// Bits to indicate FORWARD, BACKWARD, LEFT, and RIGHT
#define FORWARD_BIT  (1) // b'0001' (binary)
#define BACKWARD_BIT (2) // b'0010'
#define LEFT_BIT     (4) // b'0100'
#define RIGHT_BIT    (8) // b'1000'

// Number of commands the record buffer can hold.
// Button up and button down each counts as one command.
#define CMDBUFFERSIZE (128)

// Get current time in milliseconds on an unsigned long,
// allowing 1000 hours long recording sessions.
// Save space by using centiseconds, i.e. 0,01 s or 10 ms, or similar,
// on an unsigned int, allowing for about 10 minutes recording time.
#define CURRENTTIME (millis())
#define time_type unsigned long

// Each command is 4 bytes in size
struct Command
{
    byte id;
    byte data1;
    byte data2;
    byte checksum;
};

// Pair a command with its time offset for recording
struct TimedCmd
{
    time_type csTimeOffset;
    Command cmd;
};

// List of commands
enum COMMAND_IDS
{
    INVALID_CMD = 0,
    DRIVE = 10,         // b'0000 1010'
    RECORD = 18,        // b'0001 0100'
    REPLAY = 96,        // b'0110 0000'
    STOP = 129          // b'1000 0001'
};

// Current state of recorder, initially DRIVE
byte recorderState = DRIVE;

// Command buffer for recording. This easily blows the Arduino's SRAM limit.
// Currently, size(TimedCmd)*CMDBUFFERSIZE = (4+4)*128 = 1 KB is used.
TimedCmd cmdBuffer[CMDBUFFERSIZE];
int cmdBufferIndex = 0;
int replayIndex = 0;
time_type csStartTime = 0;

void setup()
{
    // Setup Pin I/O Functions
    pinMode(FORWARD_PIN, OUTPUT);
    pinMode(BACKWARD_PIN, OUTPUT);
    pinMode(LEFT_PIN, OUTPUT);
    pinMode(RIGHT_PIN, OUTPUT);
    
    // Initialize Serial
    Serial.begin(9600);
}

// If DEBUG is enabled, this function writes a string to the serial port
void dbg_print(const char * s)
{
#if DEBUG
    Serial.println(s);
#endif
}

// Serial write a string for debugging the recording buffer.
void dbg_print_recordreplay_action(const char * action, int index)
{
#if DEBUG
#define RRDBG_BUFFER_SIZE (70)
    char buffer[RRDBG_BUFFER_SIZE];
    snprintf(buffer, RRDBG_BUFFER_SIZE, "%s at index [%u] at time [%u] command [%u].", action, index, cmdBuffer[index].csTimeOffset, cmdBuffer[index].cmd.data1);
    dbg_print(buffer);
#endif
}

// Decodes a command struct, does some error checking, and controls the Arduino pins
void driveCar(struct Command &newCmd)
{
    // If forward and backward are both enabled, error, remove the backward bit set
    if ((newCmd.data1 & FORWARD_BIT) && (newCmd.data1 & BACKWARD_BIT)) {
        newCmd.data1 -= BACKWARD_BIT;
    }
    
    // If left and right are both enabled, error, remove the right bit set
    if ((newCmd.data1 & LEFT_BIT) && (newCmd.data1 & RIGHT_BIT)) {
        newCmd.data1 -= RIGHT_BIT;
    }
    
    // Drive forward if enabled
    if (newCmd.data1 & FORWARD_BIT) {
        // Note: newCmd.data2 is the speed, a PWM value specified in range 0 - 255, 255 = MAX
        analogWrite(FORWARD_PIN, newCmd.data2);
    } else {
        analogWrite(FORWARD_PIN, 0);
    }
    
    // Drive backward if enabled
    if (newCmd.data1 & BACKWARD_BIT) {
        analogWrite(BACKWARD_PIN, newCmd.data2);
    } else {
        analogWrite(BACKWARD_PIN, 0);
    }
    
    // Drive left if enabled
    if (newCmd.data1 & LEFT_BIT) {
        digitalWrite(LEFT_PIN, HIGH);
    } else {
        digitalWrite(LEFT_PIN, LOW);
    }
    
    // Drive right if enabled
    if (newCmd.data1 & RIGHT_BIT) {
        digitalWrite(RIGHT_PIN, HIGH);
    } else {
        digitalWrite(RIGHT_PIN, LOW);
    }
}

// Adds a command with the current time to the command buffer,
// if in recording mode. Does not record commands, if buffer is full.
void recordCommand(struct Command &cmd)
{
    if (recorderState == RECORD && cmdBufferIndex < CMDBUFFERSIZE) {
        cmdBuffer[cmdBufferIndex].csTimeOffset = CURRENTTIME - csStartTime;
        cmdBuffer[cmdBufferIndex].cmd = cmd;
        dbg_print_recordreplay_action("Recorded", cmdBufferIndex);
        cmdBufferIndex++;
    }
}

// Initialize a record session
void resetRecord()
{
    dbg_print("Resetting record states...");
    recorderState = RECORD;
    cmdBufferIndex = 0;
    csStartTime = CURRENTTIME;
}

// Initialize a replay session
void resetReplay()
{
    dbg_print("Resetting replay states...");
    recorderState = REPLAY;
    replayIndex = 0;
    csStartTime = CURRENTTIME;
}

void processCommand(struct Command &newCmd)
{
    switch (newCmd.id)
    {
        case DRIVE:
            dbg_print("Drive...");
            recordCommand(newCmd);
            driveCar(newCmd);
            break;
        case RECORD:
            dbg_print("Start record...");
            if (recorderState != RECORD) {
                resetRecord();
            }
            break;
        case REPLAY:
            dbg_print("Start replay...");
            if (recorderState != REPLAY) {
                resetReplay();
            }
            break;
        case STOP:
            dbg_print("Stop record/replay...");
            // If recording, record the last command again.
            // Saves the last time gap for replay,
            // i.e. the time from the last issued command to the recording stop.
            if (recorderState == RECORD && cmdBufferIndex > 0) {
                recordCommand(cmdBuffer[cmdBufferIndex-1].cmd);
            }
            // If stopping replay, issue a stop command, so as not to let the car drive endlessly
            if (recorderState == REPLAY) {
                Command cmd;
                cmd.id = DRIVE;
                cmd.data1 = 0;
                cmd.data2 = 0;
                cmd.checksum = 0;
                driveCar(cmd);
            }
            recorderState = DRIVE;
            break;
        default:
            // Unknown Command, do nothing
            dbg_print("Invalid cmd received...");
            break;
    }
}

// Main control loop
// Receives data from the serial port and sends it to be processed
void loop()
{
    if (recorderState == REPLAY) {
        // Restart replay if end of record reached
        if (replayIndex >= cmdBufferIndex) {
            resetReplay();
        }
        // If time of next recorded command reached, replay it.
        if (CURRENTTIME - csStartTime >= cmdBuffer[replayIndex].csTimeOffset) {
            dbg_print_recordreplay_action("Replaying", replayIndex);
            processCommand(cmdBuffer[replayIndex].cmd);
            replayIndex++;
        }
    }

    Command incomingCmd;
    if (Serial.available() >= (signed) sizeof(Command)) {
        // read the incoming data:
        Command * mem = &incomingCmd;
        unsigned char * p = (unsigned char *)mem;
        for (unsigned int i = 0; i < sizeof(Command); i++) {
            unsigned int data = Serial.read();
            p[i] = data;
        }
        
        // Verify checksum
        byte received_sum = incomingCmd.id + incomingCmd.data1 + incomingCmd.data2;
        if (incomingCmd.id != INVALID_CMD && received_sum == incomingCmd.checksum) {
            processCommand(incomingCmd);
            dbg_print("Good Cmd - checksum matched");
        } else {
            //Checksum didn't match, don't process the command
            dbg_print("Bad Cmd - invalid cmd or checksum didn't match");
        }
    }
}
