#include <stdio.h>
#include <msp430.h>
#include <string.h>
#include "main.h"
#include "src/DynamentComms.h"
#include "src/ModbusComms.h"
#include "src/comms.h"
#include "src/uart.h"
#include "src/studiolib.h"
#include "stdbool.h"

/**
 * main.c
 * Considering 1MHz
 */
#define POLL_COUNT              4
volatile int pollCounter = POLL_COUNT;  // Define pollCounter
volatile bool readyToPoll = true;
#define WATCHDOG_INTERVAL_MS    1000    // Define the watchdog timer interval in milliseconds (e.g., 1000 ms)

void clockConfigure(void)
{
    CSCTL0_H = CSKEY_H;               // Unlock CS registers
    CSCTL1 = DCOFSEL_0;               // DCO at 1 MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;  // No dividers
    CSCTL0_H = 0;                      // Lock CS registers
}

void Timer_Init(void)
{
    TA0CCR0 = 65535;             // 1 second (assuming 1MHz / 16)
    TA0CCTL0 = CCIE;             // Enable CCR0 interrupt
    TA0CTL = TASSEL_2 | MC_1 | TACLR | ID_3; // SMCLK, Up mode, /8 divider
}

void Timer1_A_Setup(void)
{
    TA1CCTL0 = CCIE;         // Enable Timer1_A interrupt
    TA1CCR0 = 10000;          // 1 ms interval (1 MHz / 1000 = 1ms)
    TA1CTL = TASSEL_2 | MC_1 | ID_0 | TACLR;  // SMCLK (1 MHz), Up mode, No divider, Clear timer
}


// Timer flag (set in timer ISR)
volatile bool timerFlag = false;

//--------------------------------------------------------------------------
// Function Prototypes
//--------------------------------------------------------------------------

void ReadingReceived(int status, float value);
void DualReadingReceived(int status, float reading1, float reading2);
void Watchdog(void);


// These communications functions are assumed to be defined elsewhere:

extern void InitialiseDynamentComms(void);
extern void DynamentCommsHandler(void);
extern void ModbusCommsHandler(void);
extern void RequestLiveData2(void (*cb)(int, float, float));


void initialise_comms(void)
{
    Timer_Init();
    rxHead = rxTail = 0;

}
void active_all()
{
    clockConfigure();
    uart_init();
    initGLED();

    //Timer_Init();
    Timer1_A_Setup();
    DEBUG_STRING("Main Code Running!r\n");

}


//--------------------------------------------------------------------------
// Watchdog Function: Pump/update the watchdog timer
void Watchdog_Init(void)
{
    // Stop the watchdog timer initially to configure it
    WDTCTL = WDTPW + WDTHOLD; // Disable Watchdog Timer

    // Configure the watchdog timer: watchdog reset after a timeout
    WDTCTL = WDTPW + WDTSSEL_2 + WDTTMSEL + WDTIS_4; // ACLK, interval = 1000 ms (1 second)

    // Enable Watchdog interrupt (optional)
    // WDTCTL |= WDTIE;  // Uncomment if you want an interrupt instead of a reset
}

// Function to refresh (clear) the watchdog timer to prevent reset
void Watchdog_Refresh(void)

{
    // Clear the Watchdog Timer to prevent a reset
    WDTCTL = WDTPW + WDTCNTCL; // Clear the Watchdog Timer
}

// Watchdog interrupt service routine (optional, if interrupt mode is enabled)
// This function will be triggered when the watchdog timer expires
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
    // Handle the Watchdog timeout event here if using interrupt mode
    // For example, you could reset some peripherals or log an error
    __no_operation(); // Placeholder for actual ISR logic
}
//--------------------------------------------------------------------------
// RequestGasReading: Determines which protocol to use to poll the sensor
//--------------------------------------------------------------------------
void RequestGasReading(void)
{

    DEBUG_STRING("RequestGasReading called\n");


    if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
    {
        DEBUG_STRING("Using Dynament_Protocol\n");
        // Request dual reading; callback function DualReadingReceived will be called
        RequestLiveData2(DualReadingReceived);
        DEBUG_STRING("RequstedLiveData2 passed\n");
    }
    else
    {
        DEBUG_STRING("Using Modbus Protocol\n");
        // Request a measurand reading; callback ReadingReceived will be called
        ReadMeasurand(GAS_READING_MEASURAND, ReadingReceived);
    }
}

//--------------------------------------------------------------------------
// Callback: ReadingReceived - called when a single gas reading is received
//--------------------------------------------------------------------------
void ReadingReceived(int status, float value)
{
    if (status == READ_RESPONSE_VALUE_VALID)
    {
        // Process the valid gas reading (value) here
        DEBUG_STRING("ReadingReceived: Valid reading: ");
        printInt(value);
        DEBUG_STRING("r\n");
    }
    else
    {
        DEBUG_STRING("ReadingReceived: Invalid reading\n");
        printInt(status);
        DEBUG_STRING("\n");
    }
    readyToPoll = true;  // Now we can send a new request

}

//--------------------------------------------------------------------------
// Callback: DualReadingReceived - called when two gas readings are received
//--------------------------------------------------------------------------
void DualReadingReceived(int status, float reading1, float reading2)
{
    DEBUG_STRING("DualReadingReceived DRR\n");
    if (status == READ_RESPONSE_VALUE_VALID)
    {
        DEBUG_STRING("Callback Executed:  Status =");
        printInt(status);

        DEBUG_STRING("Gas1 = ");
        UART_sendFloat(reading1);
        DEBUG_STRING("Gas2 =");
        UART_sendFloat(reading2);
        DEBUG_STRING("\n");
    }
    else
    {
        //printf("DualReadingReceived: Invalid reading, status=%d\n", status);
        DEBUG_STRING("Invalid reading, status =");
        char buffer[10];
        snprintf(buffer,sizeof(buffer),"%d", status);
        DEBUG_STRING(buffer);
        DEBUG_STRING("\n");
    }
    readyToPoll = true;  // Now we can send a new request

}


// Timer ISR - Handles LED toggle & sensor polling
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    P6OUT ^= BIT0;  // Toggle LED to indicate ISR is running
    static int pollTicks = 0;
    pollTicks++;
    if (pollTicks >= 8) // Every 2 seconds
    {
        pollTicks = 0;

        if (readyToPoll)
        {
            RequestGasReading();
        }
    }
}


void main(void)
{

    WDTCTL = WDTPW | WDTHOLD;         // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5;
    active_all();

    __bis_SR_register(GIE); // Enable global interrupts


    // Initialize communications
    initialise_comms();
    if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
    {
        InitialiseDynamentComms();
    }

    // Main loop
    for(;;)
    {


        //DEBUG_STRING("System Running!\r\n");

        if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
        {
            DynamentCommsHandler();
        }
        else
        {
            ModbusCommsHandler();
        }


    }

}
