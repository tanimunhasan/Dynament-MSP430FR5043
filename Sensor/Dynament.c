
#include <msp430.h>
#include <stdio.h>   // For debug printing (adjust as needed for your system)
#include "DynamentComms.h"
#include "stdbool.h"
#include "stdint.h"
#include "Hal/hal.h"
//-----------------------------------------------------------------------------

// Global variables for packet reception and state
bool receivingPacket = false;
uint8_t D_rxBuffer[DYNAMENT_MAX_PACKET_SIZE]; // Need to replace DYNAMENT_PACKET_SIZE to  (?)
unsigned int Drx_Count = 0; // Need to replace snrRxCount
bool DLEReceived = false;
bool waitingForCommand = false;
uint8_t command = 0;
bool EOFReceived = false;
uint16_t rcvCsum = 0;
uint16_t calcCsum = 0;
bool packetComplete = false;
bool csumError = false;
int csumByteReceived = 0;
bool packetNAKed = false;
uint8_t errorCode = 0;
bool packetACKed = false;

// Global protocol-level timeout counters
volatile int dynamentInterByteTimeout = 0;
volatile int dynamentResponseTimeoutCounter = 0;
volatile bool dynamentFrameComplete = false;
volatile bool readyToPoll = true;

// State machine variables
enum
{
    CommsModeIdle,
    CommsModeRequestingLiveDataSimple,
    CommsModeRequestingLiveData2,
};

int currentMode = CommsModeIdle;
GetFloat_CallBack_t readGasCallBack = 0;                // callback pointer to return values to if set to point to a function
GetDualFloat_CallBack_t readDualGasCallBack = 0;        // callback pointer to return dual values to if set to point to a function

uint16_t UpdateChecksum(uint16_t currentCRC, uint8_t newByte);
uint16_t UpdateCRCTab(uint16_t index);

void Reset(void);
bool SendDynamentPacket(uint8_t cmd, uint8_t variableID, uint8_t dlen, uint8_t *dataPtr);
void ProcessReceivedPacket(void);
void ReadLiveDataResponse(uint8_t *dataPtr, int len);
void ReadLiveData2Response(uint8_t *dataPtr, int len);
void ACKReceived(void);
void NAKReceived(void);
void PacketChecksumError(void);
void PacketTimedOut(void);
void PacketSent(void);
//------------------------------------------------------------------------------

// Public functions
//------------------------------------------------------------------------------
void ReadingReceived(int status, float value);
void DualReadingReceived(int status, float reading1, float reading2);
void initialise_comms(void);


//------------------------------------------------------------------------------

void DynamentCommsHandler(void)
{


    while(snr_is_rx_ready()) // Need to replace with snr_uart_Rx_Peek_Byte()
    {

        uint8_t ch = snr_Read();
        CharReceived(ch);
    }


    if (dynamentFrameComplete)
    {
//        DEBUG_String("Frame timeout triggered\n");
        dynamentFrameComplete = false;

        if (!packetComplete)
        {

            PacketTimedOut();  //// Frame ended but was incomplete
            readyToPoll = true;   // Allow sending next request [Only after the current packet times out or completes (good or bad), you set readyToPoll = true, signaling it's safe to start a new communication cycle.]
        }
    }

}


//------------------------------------------------------------------------------

// Private functions
//------------------------------------------------------------------------------
uint16_t UpdateChecksum(uint16_t currentCRC, uint8_t newByte)
{
    uint16_t retVal;
    if(CSUM_TYPE == CSUM_CRC)
    {
        retVal = (uint16_t)((currentCRC << 8) ^ UpdateCRCTab((uint16_t)((currentCRC >> 8) ^ newByte)));
    }
    else
    {
        retVal = (uint16_t) (currentCRC + (uint16_t) newByte);
    }
    return retVal;
}

uint16_t UpdateCRCTab(uint16_t index)
{
    uint16_t uiIndex;
    uint16_t uiCrcValue = 0, uiTempCrcValue;
    uiCrcValue = 0;
    uiTempCrcValue = (uint16_t)(((unsigned int)index) << 8);
    for(uiIndex = 0; uiIndex < 8; uiIndex++)
    {
        if (((uiCrcValue ^ uiTempCrcValue) & CRC_VALUE_MASK) > 0)
        {
            uiCrcValue = (uint16_t)(uint16_t)((uiCrcValue << 1) ^ (uint16_t)CRC_POLYNOMIAL);
        }
        else
        {
            uiCrcValue = (uint16_t)(unsigned int)(uiCrcValue << 1);
        }
        uiTempCrcValue = (uint16_t)(unsigned int)(uiTempCrcValue << 1);
    }
    return uiCrcValue;
}

/* Function: Reset all teh variables associated with a packet being received - typically called
           when a packet has timed out or been aborted */

void Reset(void)
{
    receivingPacket = false;
    Drx_Count = 0;  // Replace snrRxCount to  ( ? )
    DLEReceived = false;
    waitingForCommand = false;
    command = 0;
    EOFReceived = false;
    rcvCsum = 0;
    calcCsum = 0;
    packetComplete = false;
    csumError = false;
    csumByteReceived = 0;
    packetNAKed = false;
    packetACKed = false;
    errorCode = 0;
}
void InitialiseDynamentComms(void)
{

    Reset();                // set all the packet reception variables to their default to await an incoming packet
}

void DynamentTick(void)
{
    if (dynamentInterByteTimeout > 0)
    {
        if (--dynamentInterByteTimeout == 0)
        {
            dynamentFrameComplete = true;
        }
    }

    if (dynamentResponseTimeoutCounter > 0)
    {
        if (--dynamentResponseTimeoutCounter == 0)
        {
            PacketTimedOut();
        }
    }
}

void PacketSent(void)
{
    Reset();
    dynamentResponseTimeoutCounter = DYNAMENT_RESPONSE_TIMEOUT;
}

// Function Called when a single uint8_t of data is received in order to process it as part of an ongoing packet frame being received
void CharReceived(uint8_t chr)
{

    if (Drx_Count >= DYNAMENT_MAX_PACKET_SIZE)  // buffer full so reset - this is a bad packet
    {

         Reset();
    }


    if (chr == DLE && !EOFReceived)
    {
        if (!receivingPacket)
        {
            receivingPacket = true;
            Drx_Count = 0;
            D_rxBuffer[Drx_Count++] = chr;
            calcCsum = UpdateChecksum(calcCsum, chr);
            DLEReceived |= true;
        }
        else if (DLEReceived) // last char was a DLE so this is a uint8_t stuffed DLE character in the data (unless the checksum is being received whicvh is not uint8_t stuffed
        {

            D_rxBuffer[Drx_Count++] = chr;   // Replace D_rxBuffer [snrRxCount] to (?)
            calcCsum = UpdateChecksum(calcCsum, chr);
        }
        else  // add characters to array as ber normal  but set DLE flag
        {
            DLEReceived = true;
            D_rxBuffer[Drx_Count++] = chr;
            calcCsum = UpdateChecksum(calcCsum, chr);
        }

    }
    else if (chr == EOF && DLEReceived && !EOFReceived)  // this is the end of frame
    {
        D_rxBuffer[Drx_Count++] = chr;
        calcCsum = UpdateChecksum(calcCsum, chr);
        EOFReceived = true;
        csumByteReceived = 0;
        rcvCsum = 0;
        //DEBUG_String("EXPECTING 2 CHECKSUM BYTES NEXT\n");

    }
    else if (EOFReceived)  // receiving the checksum
    {
        D_rxBuffer[Drx_Count++] = chr;
        ++csumByteReceived;
        if (csumByteReceived >= 2)  // end of checksum receiving and end of packet
        {
            rcvCsum = (uint16_t)((D_rxBuffer[Drx_Count - 2] * 0x100) + D_rxBuffer[Drx_Count - 1]);
            packetComplete = true;
            if (rcvCsum != calcCsum)
            {
                 csumError = true;
                 /*
                 DEBUG_String("Checksum Mismatch\n");
                 DEBUG_Int(rcvCsum);
                 DEBUG_String(" vs ");
                 DEBUG_Int(calcCsum);
                 DEBUG_String("\n");
                 */
            }
            else
            {
                //DEBUG_String("Checksum OK\n");
            }
        }

    }
    /* Test should continue from here*/
    else if (packetNAKed)  // the packet has returned an error - this is the last uint8_t which is an error code
    {
        errorCode = chr;
        packetComplete = true;
        D_rxBuffer[Drx_Count++] = chr;
    }
    else if (receivingPacket) // normal character - added to the buffer if a packet reception is in progress
    {
        if (DLEReceived && Drx_Count == 1)
        {
            command = chr;   // the second uint8_t in the packet (if a DLE was the first) is the command character
            if (command == NAK)
            {
                packetNAKed = true;
            }
            if (command == ACK)
            {
                packetACKed = true;
                packetComplete = true;
            }
        }
        D_rxBuffer[Drx_Count++] = chr;
        calcCsum = UpdateChecksum(calcCsum, chr);
    }

    if (chr != DLE) DLEReceived = false;

    dynamentInterByteTimeout = DYNAMENT_FRAME_TIMEOUT;

    if (packetComplete)  // a complete packet has actually been received  -act on it according to the various flags
    {
        if (packetACKed)
            ACKReceived();
        else if (packetNAKed)
            NAKReceived();
        else if (csumError)
            PacketChecksumError();
        else  // complete data packet received intact without error
        {
            ProcessReceivedPacket();
        }

        Reset();
    }
}
/*
 * Function: Create a packet of data in the Dynament protocol
   Inputs : cmd - the type of packet (read or write variable typically)
            variableID - the id of the variable to be read or written
            dlen - the number of bytes of data within the packet
            dataPtr - pointer to array of bytes of the packet data
   Outputs: Returns true if packet sent else returns false
   */



bool SendDynamentPacket(uint8_t cmd, uint8_t variableID, uint8_t dlen, uint8_t *dataPtr)
{
    unsigned int x;

    uint8_t txBuf[DYNAMENT_MAX_PACKET_SIZE];
    unsigned short txBufPtr = 0;
    uint16_t csum = 0;

    txBuf[txBufPtr++] = DLE;
    csum = UpdateChecksum(csum, DLE);
    txBuf[txBufPtr++] = cmd;
    csum = UpdateChecksum(csum, (uint16_t)cmd);

    if(cmd == READ_VAR)
    {
        txBuf[txBufPtr++] = variableID;
        csum = UpdateChecksum(csum, (uint16_t)variableID);
    }
    else if(cmd == WRITE_REQUEST)
    {
        txBuf[txBufPtr++] = WRITE_PASSWORD_1;
        csum = UpdateChecksum(csum, (uint16_t)WRITE_PASSWORD_1);

        txBuf[txBufPtr++] = WRITE_PASSWORD_2;
        csum = UpdateChecksum(csum, (uint16_t)WRITE_PASSWORD_2);

        txBuf[txBufPtr++] = variableID;
        csum = UpdateChecksum(csum, (uint16_t)variableID);
    }

    if(dlen > 0)
    {
        if(dlen == DLE)
        {
            txBuf[txBufPtr++] = DLE;
            csum = UpdateChecksum(csum, (uint16_t)DLE);
        }
        txBuf[txBufPtr++] = dlen;
        csum = UpdateChecksum(csum, (uint16_t)dlen);
        for(x = 0; x < dlen; x++)
        {
            if(dataPtr[x] == DLE)
            {
                txBuf[txBufPtr++] = DLE;
                csum = UpdateChecksum(csum,(uint16_t) DLE);
            }
            txBuf[txBufPtr++] = dataPtr[x];
            csum = UpdateChecksum(csum, (uint16_t)dataPtr[x]);
        }
    }
    txBuf[txBufPtr++] = DLE;
    csum = UpdateChecksum(csum, (uint16_t)DLE);

    txBuf[txBufPtr++] = EOF;
    csum = UpdateChecksum(csum, (uint16_t)EOF);

    // Append checksum (two bytes)
    txBuf[txBufPtr++] = (uint8_t)((csum >> 8) & 0x00ff);
    txBuf[txBufPtr++] = (uint8_t)(csum & 0x00ff);

    if(dlen > (DYNAMENT_MAX_PACKET_SIZE)) // // if there is too much data for a full length packet, then abort
        return false;

    unsigned int dx;
    // Transmit packet to serial port
    for (dx = 0; dx < txBufPtr; dx++)
        snr_Write(txBuf[dx]);
    PacketSent();

    /*
    unsigned int idx;
    DEBUG_String ("TX: ");
    for (idx=0; idx < txBufPtr; idx++)
        DEBUG_Hex(txBuf[idx]);
    DEBUG_String(" <Tx End>\n");
    */


    return true;

}

void ProcessReceivedPacket(void)
{
    //DEBUG_String("Entered ProcessReceivedPacket()\n");
    uint8_t cmd = D_rxBuffer[1]; /* replace D_rxBuffer to ( ? ) */
    uint8_t len = D_rxBuffer[2]; /* replace D_rxBuffer to ( ? ) */
    unsigned int x;

    if(cmd == DAT)      // data packet response to a read variable received
    {
        uint8_t rcvData[200];
        for(x = 0; x < len && (x + 3) < snrRxCount; x++) // replace snrRxCount to (?)
        {
            rcvData[x] = D_rxBuffer[x + 3];
        }

        switch(currentMode)
        {
            case CommsModeRequestingLiveDataSimple: // waiting for a response to a read live data simple packet
                ReadLiveDataResponse(rcvData, len);
                break;
            case CommsModeRequestingLiveData2:      // waiting for a response to a read live data 2 packet
                ReadLiveData2Response(rcvData, len);
                break;

        }
    }
    else
    {
        // do nothing..
    }
}

// Function: Called when a data response to a read simple live data packet has been received
// Inputs : dataPtr - pointer to the data contents of the packet
//          len - the number of bytes of data within the packet

void ReadLiveDataResponse(uint8_t *dataPtr, int len)
{
    float gasValue = 0;

    // get the status values from the message
    uint16_t statusVal1 = (uint16_t)((dataPtr[3] * 0x100) + dataPtr[2]);

    // get the float value from the two registers that cover the actual reading
    uint32_t intVal = (uint32_t)((dataPtr[7] * 0x1000000) +
                                  (dataPtr[6] * 0x10000) +
                                  (dataPtr[5] * 0x100) +
                                  dataPtr[4]);
    float *fPtr = (float *)&intVal;
    gasValue = *fPtr;

    /*
    DEBUG_String("Parsed Gas Value: ");
    DEBUG_Int(gasValue);
    DEBUG_String("\n");

    DEBUG_String("Parsed StatusVal1: ");
    DEBUG_Int(statusVal1);
    DEBUG_String("\n");
     */
    // call the callback routine to rerpot the received gas reading to the calling layer
    if(currentMode == CommsModeRequestingLiveDataSimple && readGasCallBack != 0)
    {
        if(gasValue < -1 || statusVal1 > 0)
            // any gas reading less than -1 is likely to be an error signal (e.g. sensor warming up or in fault) or any
            // status flags set is likely to be an error too
            readGasCallBack(READ_RESPONSE_VALUE_INVALID, gasValue);
        else
            readGasCallBack(READ_RESPONSE_VALUE_VALID, gasValue);
    }
    currentMode = CommsModeIdle;
    readGasCallBack = 0;
}


// Function: Called when a data response to a read live data 2 packet has been received
// Inputs : dataPtr - pointer to the data contents of the packet
//          len - the number of bytes of data within the packet

void ReadLiveData2Response(uint8_t *dataPtr, int len)
{
    float gasValue1 = 0;
    float gasValue2 = 0;



    // Get the status values from the message
    uint16_t statusVal1 = (uint16_t)((dataPtr[3] * 0x100) + dataPtr[2]);
    uint16_t statusVal2 = (uint16_t)((dataPtr[41] * 0x100) + dataPtr[40]);

    // Get the float value for gas reading 1
    uint32_t intVal = (uint32_t)((dataPtr[7] * 0x1000000) +
                                  (dataPtr[6] * 0x10000) +
                                  (dataPtr[5] * 0x100) +
                                  dataPtr[4]);
    float *fPtr = (float *)&intVal;
    gasValue1 = *fPtr;

    // Get the float value for gas reading 2
    intVal = (uint32_t)((dataPtr[15] * 0x1000000) +
                        (dataPtr[14] * 0x10000) +
                        (dataPtr[13] * 0x100) +
                        dataPtr[12]);
    fPtr = (float *)&intVal;
    gasValue2 = *fPtr;


    // Call the callback routine to report the received gas reading
    if(currentMode == CommsModeRequestingLiveData2 && readDualGasCallBack != 0)
    {

        if(statusVal1 > 0 || statusVal2 > 0)    // Any status flags set indicate an invalid value
        {
            readDualGasCallBack(READ_RESPONSE_VALUE_INVALID, gasValue1, gasValue2);

        }
        else
        {
            readDualGasCallBack(READ_RESPONSE_VALUE_VALID, gasValue1, gasValue2);
        }
    }
    else
    {

    }

    currentMode = CommsModeIdle;
    readDualGasCallBack = 0;

}

/* Function: Called when an ACK paclet is received to a previously transmitted messages e.g. a Write Variable message
              - the data to be written can now be sent */
void ACKReceived(void)
{
    // currently not implemented - but data for a write variable would be transmitted from here
}

// Function: Called when a NAK packet to a previously transmitted message is received
void NAKReceived(void)
{
    // call the callback routone with a status PACKET_IINVALID
    if(currentMode == CommsModeRequestingLiveDataSimple && readGasCallBack != 0)
    {
        //printf("PacketTimedOut called, resetting callback.\n");
        readGasCallBack(READ_RESPONSE_INVALID_REGISTER, 0);
    }
    currentMode = CommsModeIdle;
    readGasCallBack = 0;
}

// Function: Called when a received packet has detecged a checksum error
void PacketChecksumError(void)
{
    PacketTimedOut(); // treat the packet as having received no response
}

//Function: Called when no response has been received to a previously transmitted packet
void PacketTimedOut(void)
{
    if(currentMode == CommsModeRequestingLiveDataSimple && readGasCallBack != 0)
    {
        readGasCallBack(READ_RESPONSE_TIMED_OUT, 0);
    }
    currentMode = CommsModeIdle;
    readGasCallBack = 0;

    Reset();
}


// Function: Send the command to read a live data packet to obtain the current gas reading
//Inputs: cb - pointer to callback routine to be called when a result has been determined
void RequestLiveDataSimple(GetFloat_CallBack_t cb)
{

    SendDynamentPacket(READ_VAR, LIVE_DATA_SIMPLE, 0, 0); //LIVE_DATA_SIMPLE
    readGasCallBack = cb;
    currentMode = CommsModeRequestingLiveDataSimple;

}

// Function: Send the command to read a live data 2 packet to obtain the current gas reading and status
//Inputs: cb - pointer to callback routine to be called when a result has been determined
void RequestLiveData2(GetDualFloat_CallBack_t cb)
{

    SendDynamentPacket(READ_VAR, LIVE_DATA_2, 0,0);
    readDualGasCallBack = cb;
    currentMode = CommsModeRequestingLiveData2;

}


void initialise_comms(void)
{

}


//--------------------------------------------------------------------------
// Callback: ReadingReceived - called when a single gas reading is received
//--------------------------------------------------------------------------
void ReadingReceived(int status, float value)
{
    if (status == READ_RESPONSE_VALUE_VALID)
    {
        // Process the valid gas reading (value) here
        //DEBUG_String("ReadingReceived: Valid reading: ");
        //DEBUG_Int(value);
        //DEBUG_String("r\n");
    }
    else
    {
        //DEBUG_String("ReadingReceived: Invalid reading\n");
        //DEBUG_Int(status);
        //DEBUG_String("\n");
    }

}


void RequestGasReading(void)
{

    if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
    {
       // DEBUG_String("Using Dynament_Protocol\n");
        // Request dual reading; callback function DualReadingReceived will be called
        RequestLiveData2(DualReadingReceived);
        //DEBUG_String("RequstedLiveData2 passed\n");
    }
    else
    {
        //DEBUG_String("Using Modbus Protocol\n");
        // Request a measurand reading; callback ReadingReceived will be called
        //ReadMeasurand(GAS_READING_MEASURAND, ReadingReceived);
    }
}


//--------------------------------------------------------------------------
// Callback: DualReadingReceived - called when two gas readings are received
//--------------------------------------------------------------------------
void DualReadingReceived(int status, float reading1, float reading2)
{
   // DEBUG_String("DualReadingReceived DRR\n");
    if (status == READ_RESPONSE_VALUE_VALID)
    {
        /*

        DEBUG_String("Callback Executed:  Status =");
        DEBUG_Int(status);
        DEBUG_String("\n");

        DEBUG_String("Gas1 = ");
        DEBUG_Float(reading1);

        DEBUG_String("   Gas2 =");
        DEBUG_Float(reading2);
        DEBUG_String("\n");
        */

    }
    else
    {

       /*
        DEBUG_String("Invalid reading, status =");
        P4OUT ^= BIT6;
        char buffer[10];
        snprintf(buffer,sizeof(buffer),"%d", status);
        DEBUG_String(buffer);
        DEBUG_String("\n");
        */
    }


}

void DynamentSensorApplication(void)
{
    // Step 1: Init UART, buffers, etc.
    initialise_comms();

    // Step 2: Protocol-specific init
    if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
    {
        InitialiseDynamentComms();
    }

    // Step 3: Main loop
    for (;;)
    {
        if (COMMS_PROTOCOL == DYNAMENT_PROTOCOL)
        {
            DynamentCommsHandler();
        }
//        else
//        {
//            ModbusCommsHandler();
//        }
    }
}


/*** end of file ***/
