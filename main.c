
#include "main.h"
#include "driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <msp430.h>
#include "InitFunctions.h"
#include <string.h>
/**
 * main.c
 */

/* PinMap
 * P1.0 - C2
 * P8.0 - C1
 * P2.5 - C3
 *
 * P2.7 - SensorInteruptPin (Might be needed for LCD)
 *
 * P1.3 - XA
 * P1.4 - XB
 * P1.5 - YA
 * P1.6 - YB
 *
 * P1.7 - Motor Enable
 *
 * P5.0 - Sensor In
 * P5.2 - SensorSel0
 * P5.3 - SensorSel1
 *
 * P8.2 KeyPadSel0
 * P8.3 KeyPadSel1
 */

#define BAD_MOTOR_WIRING

#define Keypad_Col1 GPIO_PORT_P8, GPIO_PIN0
#define Keypad_Col2 GPIO_PORT_P1, GPIO_PIN0
#define Keypad_Col3 GPIO_PORT_P2, GPIO_PIN5

#define KeypadSel0 GPIO_PORT_P8, GPIO_PIN2
#define KeypadSel1 GPIO_PORT_P8, GPIO_PIN3

#define SensorInterrupt GPIO_PORT_P2, GPIO_PIN7

#define SensorInput GPIO_PORT_P5, GPIO_PIN0
#define SensorSel0 GPIO_PORT_P5, GPIO_PIN2
#define SensorSel1 GPIO_PORT_P5, GPIO_PIN3

#define MotorEnable GPIO_PORT_P1, GPIO_PIN7

#ifndef BAD_MOTOR_WIRING
#define MotorXA GPIO_PORT_P1, GPIO_PIN3
#define MotorXB GPIO_PORT_P1, GPIO_PIN4
#define MotorYA GPIO_PORT_P1, GPIO_PIN5
#define MotorYB GPIO_PORT_P1, GPIO_PIN6
#else
#define MotorXB GPIO_PORT_P1, GPIO_PIN3
#define MotorXA GPIO_PORT_P1, GPIO_PIN4
#define MotorYB GPIO_PORT_P1, GPIO_PIN5
#define MotorYA GPIO_PORT_P1, GPIO_PIN6
#endif
#define setPinHigh(x) GPIO_setOutputHighOnPin(x)
#define setPinLow(x) GPIO_setOutputLowOnPin(x)
#define getPinValue(x) GPIO_getInputPinValue(x)

#define StartButton GPIO_PORT_P1, GPIO_PIN2

#define STEPS_PER_CYCLE 512


#define TOP_SENSOR 0
#define BOT_SENSOR 1
#define LEFT_SENSOR 2
#define RIGHT_SENSOR 3

#define MOTOR_DELAY_CYCLES 200

int currentXPos;
int currentYPos;

int targetX[5];
int targetY[5];

int X_state;
int Y_state;

volatile int TopBlocked;
volatile int BotBlocked;
volatile int LeftBlocked;
volatile int RightBlocked;
char digits[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
//We only allow targets 0-9

void displaySixChars(char *txt)
{
    showChar(txt[0], pos1);
    showChar(txt[1], pos2);
    showChar(txt[2], pos3);
    showChar(txt[3], pos4);
    showChar(txt[4], pos5);
    showChar(txt[5], pos6);
}
inline void motorDelay()
{
    volatile int cycles = MOTOR_DELAY_CYCLES;
    do cycles--;
    while(cycles != 0);
}

inline void Delay(uint32_t c)
{
    volatile uint32_t cycles = c;
    do cycles--;
    while(cycles != 0);
}


// AB - BC - CD - DA
// AB - !AB  !A!B A!B
//Toggle A, B, A, B
void stepX_FWD_Y_FWD(int mvX, int mvY) //4 Steps
{
    int canX = (ReadLimitSensor(RIGHT_SENSOR) == 0) ? mvX : 0;
    int canY = (ReadLimitSensor(TOP_SENSOR) == 0) ? mvY : 0;

    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
}
void stepX_FWD_Y_BWD() //4 Steps
{
    int canX = (ReadLimitSensor(RIGHT_SENSOR) == 0) ? 1 : 0;
    int canY = (ReadLimitSensor(BOT_SENSOR) == 0) ? 1 : 0;

    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY) GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY)  GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
}
void stepX_BWD_Y_FWD() //4 Steps
{
    int canX = (ReadLimitSensor(LEFT_SENSOR) == 0) ? 1 : 0;
    int canY = (ReadLimitSensor(TOP_SENSOR) == 0) ? 1 : 0;

    if(canX)  GPIO_toggleOutputOnPin(MotorXB);
    if(canY)  GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX)  GPIO_toggleOutputOnPin(MotorXA);
    if(canY)  GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX)  GPIO_toggleOutputOnPin(MotorXB);
    if(canY)  GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX)  GPIO_toggleOutputOnPin(MotorXA);
    if(canY)  GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
}
void stepX_BWD_Y_BWD(int mvX, int mvY) //4 Steps
{
    int canX = (ReadLimitSensor(LEFT_SENSOR) == 0) ? mvX : 0;
    int canY = (ReadLimitSensor(BOT_SENSOR) == 0) ? mvY : 0;

    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXB);
    if(canY) GPIO_toggleOutputOnPin(MotorYB);
    motorDelay();
    if(canX) GPIO_toggleOutputOnPin(MotorXA);
    if(canY) GPIO_toggleOutputOnPin(MotorYA);
    motorDelay();
}


void moveMotors(int Xtarget, int Ytarget)
{
    int X = Xtarget - currentXPos; //Find Deltas that we need to move
    int Y = Ytarget - currentYPos;
    int i;
    showChar(digits[currentXPos], pos2);
    showChar(digits[currentYPos], pos6);
    while(X || Y) //If X or Y needs to move
    {
        //Each iteration is 4 steps
        //Each Completion of the loop is 1 full rotation
        for(i = STEPS_PER_CYCLE; i != 0; i--)
        {
            if(X > 0 && Y > 0)
            {
                stepX_FWD_Y_FWD(1,1);
            }
            else if(X < 0 && Y > 0)
            {
                stepX_BWD_Y_FWD();
            }
            else if(X > 0 && Y < 0)
            {
                stepX_FWD_Y_BWD();
            }
            else if(X < 0 && Y < 0)
            {
                stepX_BWD_Y_BWD(1, 1);
            }
            else if(X == 0 && Y > 0)
            {
                stepX_FWD_Y_FWD(0,1);
            }
            else if(X == 0 && Y < 0)
            {
                stepX_BWD_Y_BWD(0, 1);
            }
            else if(X > 0 && Y == 0)
            {
                stepX_FWD_Y_FWD(1,0);
            }
            else if(X < 0 && Y == 0)
            {
                stepX_BWD_Y_BWD(1, 0);
            }
            else if(X == 0 && Y == 0)
            {
                break;
            }

        }
        X += (X == 0) ? 0 : (X > 0) ? -1 : 1;
        Y += (Y == 0) ? 0 : (Y > 0) ? -1 : 1;
        showChar(digits[Xtarget - X], pos2);
        showChar(digits[Ytarget - Y], pos6);
    }
    currentXPos = Xtarget;
    currentYPos = Ytarget;
}


void enableKeypadRow(int row)
{
    switch(row)
    {
    case 1:
        setPinLow(KeypadSel0);
        setPinLow(KeypadSel1);
        break;
    case 2:
        setPinHigh(KeypadSel0);
        setPinLow(KeypadSel1);
        break;
    case 3:
        setPinLow(KeypadSel0);
        setPinHigh(KeypadSel1);
        break;
    case 4:
        setPinHigh(KeypadSel0);
        setPinHigh(KeypadSel1);
    }
}

int getKeypadColumn()
{
    if(getPinValue(Keypad_Col1) > 0)
      return 1;
    else if(getPinValue(Keypad_Col2) > 0)
       return 2;
    else if(getPinValue(Keypad_Col3) > 0)
        return 3;
    return 0;
}

int getKeyInput()
{
    int key = 0;
    while(1)
    {
        enableKeypadRow(1);
        Delay(300);
        key = getKeypadColumn();
        if(key) break;
        enableKeypadRow(2);
        Delay(300);
        key = getKeypadColumn();
        key += (key) ? 3 : 0;
        if(key) break;
        enableKeypadRow(3);
        Delay(300);
        key = getKeypadColumn();
        key += (key) ? 6 : 0;
        if(key) break;
        enableKeypadRow(4);
        Delay(300);
        key = getKeypadColumn();
        key += (key) ? 9 : 0;
        if(key) break;
    }
    return (key == 11) ? 0 : key;
}
int ReadLimitSensor(int sensor)
{
    switch(sensor)
    {
    case TOP_SENSOR: //10
        setPinHigh(SensorSel1);
        setPinLow(SensorSel0);
        break;
    case BOT_SENSOR: //11
        setPinHigh(SensorSel1);
        setPinHigh(SensorSel0);
        break;
    case LEFT_SENSOR: //00
        setPinLow(SensorSel1);
        setPinLow(SensorSel0);
        break;
    case RIGHT_SENSOR: //01
        setPinLow(SensorSel1);
        setPinHigh(SensorSel0);
    }
    return (getPinValue(SensorInput));
}

int populateCoordinateArray()
{
    int key;
    int numCor = 0;
    volatile int i = 20000;
    SelectKeys:
    memset(targetX, 0, 5);
    memset(targetY, 0, 5);
    numCor = 0;
    displaySixChars("SEL X1");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetX[0] = key;
    do i--;
    while(i != 0);
    i = 20000;


    displaySixChars("SEL Y1");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetY[0] = key;
    numCor++;
    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL X2");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetX[1] = key;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL Y2");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetY[1] = key;
    numCor++;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL X3");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetX[2] = key;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL Y3");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetY[2] = key;
    numCor++;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL X4");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetX[3] = key;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL Y4");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetY[3] = key;
    numCor++;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL X5");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetX[4] = key;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("SEL Y5");
    key = getKeyInput();
    if(key == 10) goto SelectKeys;
    if(key == 12) goto StartMotors;
    targetY[4] = key;
    numCor++;

    do i--;
    while(i != 0);
    i = 20000;

    displaySixChars("HIT GO");
    while(getKeyInput() != 12) {}
    StartMotors:
    displaySixChars("X   Y ");
   return numCor;
}

void InitAllPins()
{
    GPIO_setAsInputPinWithPullDownResistor(Keypad_Col2); // C2
    GPIO_setAsInputPinWithPullDownResistor(Keypad_Col1); // C1
    GPIO_setAsInputPinWithPullDownResistor(Keypad_Col3); // C3

    GPIO_setAsOutputPin(KeypadSel0); //KeyPadSel0
    GPIO_setAsOutputPin(KeypadSel1); //KeyPadSel1

    GPIO_setAsOutputPin(StartButton);

    GPIO_setAsInputPinWithPullDownResistor(SensorInterrupt); //Sensor Interrupt

    GPIO_setAsOutputPin(MotorXA); // XA
    GPIO_setAsOutputPin(MotorXB); // XB
    GPIO_setAsOutputPin(MotorYA); // YA
    GPIO_setAsOutputPin(MotorYB); // YB

    GPIO_setAsOutputPin(MotorEnable); // Motor Enable

    GPIO_setAsInputPinWithPullDownResistor(SensorInput); //Sensor Input
    GPIO_setAsOutputPin(SensorSel0); //SensorSel0
    GPIO_setAsOutputPin(SensorSel1); //SensorSel1
}


void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	MCUInit();
	InitAllPins();
	unsigned int i;
	currentXPos = 0;
	currentYPos = 0;
	int numCor;
	while(1)
	{
        numCor = populateCoordinateArray();
        for(i = 0; i < numCor; i++)
        {
            moveMotors(targetX[i], targetY[i]);
            showChar('C',  pos3);
            showChar(digits[i+1], pos4);
            Delay(150000);
            showChar(' ',  pos3);
            showChar(' ', pos4);

        }
	}
}
