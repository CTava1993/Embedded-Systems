/*
 * ======================================================================================
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *======================================================================================
 *
 * ======================================================================================
 *       Christian Tavares || CS 350 Emerging Sys Arch & Tech 2024 || 12/14/2024
 * ======================================================================================
 * This is a modified version of the initial gpiointerrupt program I wrote to do morse
 * code. There are some straggling variables and artifacts here and there that you can
 * notice around the code. The new program is all part of a project that is meant to be
 * a coded thermostat. The program includes a task scheduler, as per the requirements.
 * The program will loop in periods of 100ms, checking for button presses every 200ms
 * while updating the LED and heat to on/off every 500ms.
 * ======================================================================================
 *
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART2_write(uart, &output, x);

// UART Global Variables
char  output[64];
int  bytesToSend;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t  txBuffer[1];
uint8_t  rxBuffer[2];
I2C_Transaction  i2cTransaction;

// Driver Handles - Global variables
UART2_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

// Button States
enum BUTTON_STATES {BUTTON_INIT, BUTTON1, BUTTON2} BUTTON_STATE;

// Display Variables
volatile unsigned char TimerFlag = 0;

int16_t temperature = 0; // Initial temp set to 0
unsigned char i = 0; // Initialize and set it to 0
unsigned char state; // Our variable for declaring OK and SOS states
int setTemp = 21; // Sets the initial  temp to 21 degrees Celsius (changes using buttons)
unsigned char heat = 0; // Displays 0 or 1 ( 0 = Heat off / 1 = Heat on )
int seconds = 0; // Displays how long the system has been running

// Arrays with state information. Each integer is in intervals of 500ms
// (ie: 1,0 is 500ms of red light followed by both lights off for 500ms)
// I could have written this neater, but it worked in the moment.
//
// Trust the process.
int SOS_array[] = {1,0,1,0,1,0,0,0,2,2,2,0,2,2,2,0,2,2,2,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0};
int OK_array[] = {2,2,2,0,2,2,2,0,2,2,2,0,0,0,2,2,2,0,1,0,2,2,2,0,0,0,0,0,0,0};

// This function does all of the processing for the LEDs by reading our arrays.
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
}

// This function has all of our desired parameters upon initializing our timer.
void initTimer(void)
{
    Timer_Handle timer0;
    Timer_Params params;
    Timer_init();
    Timer_Params_init(&params);
    params.period = 500000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

void initUART(void)
{
    UART2_Params uartParams;

    // Init the driver
    // UART2_init();

    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {

    /* UART_open() failed */
    while (1);
    }
}

// Make sure you call initUART() before calling this function.
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {

        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

void Read_Button() {
    switch (BUTTON_STATE) {
        case BUTTON_INIT: // If no button was pressed, setPoint does not change
            break;
        case BUTTON1: // Increase setTemp when button 1 is pressed, reset
            setTemp += 1;
            BUTTON_STATE = BUTTON_INIT;
            break;
        case BUTTON2: // Decrease setTemp when button 2 is pressed, reset
            setTemp -= 1;
            BUTTON_STATE = BUTTON_INIT;
            break;
    }
}

// Temperature and LED States
enum TEMP_STATES {TEMP_INIT, TEMP_READ} TEMP_STATE;
enum LED_STATES {LED_INIT, LED_ON, LED_OFF} LED_STATE;

void Read_Temp() {
    switch (TEMP_STATE) {
        case TEMP_INIT: // Initial temp state
            TEMP_STATE = TEMP_READ;
            break;
        case TEMP_READ: // Set temp, then turn LED on or off
            temperature = readTempSensor();
            if (temperature < setTemp) { // LED on
                LED_STATE = LED_ON;
            }
            if (temperature >= setTemp) { // LED off
                LED_STATE = LED_OFF;
            }
            break;
    }
}

void Read_LED() {
    switch (LED_STATE) {
        case LED_INIT: // Initial LED state
            break;
        case LED_ON: // Heat on
            heat = 1;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;
        case LED_OFF: // Heat off
            heat = 0;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    BUTTON_STATE = BUTTON1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    BUTTON_STATE = BUTTON2;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // Initialize time to display output right away
    unsigned long buttonTime = 200;
    unsigned long tempTime = 500;
    unsigned long updateTime= 1000;

    const unsigned long timerIncrement = 100; // Timers increment every 100ms

    /* Call driver init functions */
    GPIO_init();
    initTimer();
    initUART();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // Initialize state machines
    BUTTON_STATE = BUTTON_INIT;
    TEMP_STATE = TEMP_INIT;
    LED_STATE = LED_INIT;

    while(1){
        if (buttonTime >= 200) {
            Read_Button();
            buttonTime = 0; // Clear timer
        }
        if (tempTime >= 500) {
            Read_Temp();
            Read_LED();
            tempTime = 0; // Clear timer
        }
        if (updateTime >= 1000) { // Update display every 1000ms
            if (temperature == 0){
                DISPLAY(snprintf(output, 64, "<Retrieving Data>\n\r", temperature, setTemp, heat, seconds))
            }
            else {
                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setTemp, heat, seconds))
            }
            seconds += 1;
            updateTime = 0; // Clear timer
        }

        while(!TimerFlag){}
        TimerFlag = 0;

        // Increment timers every 100ms
        updateTime += timerIncrement;
        tempTime += timerIncrement;
        buttonTime += timerIncrement;
    }
}
