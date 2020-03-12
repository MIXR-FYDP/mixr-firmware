/*******************************************************************************
*
*               Audio Framework
*               ---------------
*
********************************************************************************
*     Platform.c
********************************************************************************
*
*     Description:  AudioWeaver Main Platform processing
*
*     Copyright:    (c) 2018 DSP Concepts Inc., All rights reserved
*                   3235 Kifer Road
*                   Santa Clara, CA 95054
*
*******************************************************************************/
#include "Platform.h"
#include "STM32F769_PassThru_with_ControlIO_ControlInterface.h"

/** This awe instance */
AWEInstance g_AWEInstance;

/** Flash file system instance */
AWEFlashFSInstance g_AWEFlashFSInstance;

/** The only input pin for this core. */
static IOPinDescriptor s_InputPin = { 0 };

/** The only output pin for this core. */
static IOPinDescriptor s_OutputPin = { 0 };

volatile BOOL g_bPacketReceived = FALSE;
volatile BOOL g_bDeferredProcessingRequired = FALSE;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
  UINT32 g_MasterHeap[MASTER_HEAP_SIZE] @ ".DTCMRAM_Section";
#else
	#if defined( __GNUC__ )
		__attribute__((__section__(".dtcmram")))
  	  	__ALIGN_BEGIN UINT32 g_MasterHeap[MASTER_HEAP_SIZE] __ALIGN_END;
	#else
		#if defined ( __CC_ARM ) /*!< Keil Compiler */
			__attribute__((__section__(".data_DTCM")))
			__ALIGN_BEGIN UINT32 g_MasterHeap[MASTER_HEAP_SIZE];
		#endif
	#endif
#endif

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
    UINT32 g_FastbHeap[FASTB_HEAP_SIZE];
#else
    __ALIGN_BEGIN UINT32 g_FastbHeap[FASTB_HEAP_SIZE] __ALIGN_END;
#endif

#if defined( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
	__no_init UINT32 g_SlowHeap[SLOW_HEAP_SIZE] @ ".SDRAM_Section";
#else
	#if defined( __GNUC__ )
		__attribute__((__section__(".sdram")))
		__ALIGN_BEGIN UINT32 g_SlowHeap[SLOW_HEAP_SIZE] __ALIGN_END;
	#else
		#if defined ( __CC_ARM ) /*!< Keil Compiler */
			__attribute__((__section__(".data_SDRAM"), zero_init))
			__ALIGN_BEGIN UINT32 g_SlowHeap[SLOW_HEAP_SIZE];
		#endif
	#endif
#endif

static UINT32 nPrevCycles = 0;
static volatile UINT32 g_nElapsedMilliSeconds = 0;
extern __IO UINT32 uwTick;

/* ----------------------------------------------------------------------
** Module table
** ------------------------------------------------------------------- */

/* Array of pointers to module descriptors. This is initialized at compile time.
Each item is the address of a module descriptor that we need linked in. The
linker magic is such that only those modules referenced here will be in the
final program. */
const void * g_module_descriptor_table[] =
{
    //The suitably cast pointers to the module descriptors go here.
    (void *)LISTOFCLASSOBJECTS
};

UINT32 g_module_descriptor_table_size = sizeof(g_module_descriptor_table) / sizeof(g_module_descriptor_table[0]);


//-----------------------------------------------------------------------------
// METHOD:  AWEInstanceInit
// PURPOSE: Initialize AWE Instance with target details
//-----------------------------------------------------------------------------
void AWEInstanceInit()
{
    memset(&g_AWEInstance, 0, sizeof(AWEInstance) );

    g_AWEInstance.pInputPin = &s_InputPin;
    g_AWEInstance.pOutputPin = &s_OutputPin;

    awe_initPin(&s_InputPin, INPUT_CHANNEL_COUNT, NULL);
    awe_initPin(&s_OutputPin, OUTPUT_CHANNEL_COUNT, NULL);

#if HAS_FLASH_FILESYSTEM

    // Setup the flash file system */
    memset(&g_AWEFlashFSInstance, 0, sizeof(AWEFlashFSInstance) );

    g_AWEFlashFSInstance.cbInit = &usrInitFlashFileSystem;
    g_AWEFlashFSInstance.cbEraseSector = &usrEraseFlashMemorySector;
    g_AWEFlashFSInstance.cbFlashWrite = &usrWriteFlashMemory;
    g_AWEFlashFSInstance.cbFlashRead = &usrReadFlashMemory;

    g_AWEFlashFSInstance.flashSizeInBytes = FLASH_MEMORY_SIZE_IN_BYTES;
    g_AWEFlashFSInstance.flashErasableBlockSizeInBytes = ERASEABLE_BLOCKSIZE;
    g_AWEFlashFSInstance.flashStartOffsetInBytes = FILE_SYSTEM_START_OFFSET;
    g_AWEFlashFSInstance.flashEraseTimeInMs = ( (MX25L512_BULK_ERASE_MAX_TIME / 10000) * 2) + 20;

    awe_initFlashFS(&g_AWEInstance, &g_AWEFlashFSInstance);

    g_AWEInstance.pFlashFileSystem = &g_AWEFlashFSInstance;

#endif

    // User version word
    g_AWEInstance.userVersion = USER_VER;

    g_AWEInstance.instanceId = CORE_ID;
    g_AWEInstance.coreSpeed = CORE_SPEED;
    g_AWEInstance.profileSpeed = CORE_SPEED;
    g_AWEInstance.pName = "ST32F769";
    g_AWEInstance.numThreads = NUM_AUDIO_THREADS;
    g_AWEInstance.pModuleDescriptorTable = g_module_descriptor_table;
    g_AWEInstance.numModules = g_module_descriptor_table_size;
    g_AWEInstance.sampleRate = AUDIO_SAMPLE_RATE;
    g_AWEInstance.fundamentalBlockSize = AUDIO_BLOCK_SIZE;

    // Define the heap sizes
    g_AWEInstance.fastHeapASize = MASTER_HEAP_SIZE;
    g_AWEInstance.fastHeapBSize = FASTB_HEAP_SIZE;
    g_AWEInstance.slowHeapSize  = SLOW_HEAP_SIZE;

    // Point to the heaps on this target
    g_AWEInstance.pFastHeapA = g_MasterHeap;
    g_AWEInstance.pFastHeapB = g_FastbHeap;
    g_AWEInstance.pSlowHeap  = g_SlowHeap;

    g_AWEInstance.pPacketBuffer = g_packet_buffer;
    g_AWEInstance.packetBufferSize = MAX_COMMAND_BUFFER_LEN;

    g_AWEInstance.pReplyBuffer = g_packet_buffer;

    // Initialize AWE signal processing instance
    awe_init(&g_AWEInstance);

}   // End AWEInstanceInit


//-----------------------------------------------------------------------------
// METHOD:  targetInit
// PURPOSE: Initialize AWE
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
void targetInit(void)
{
    // Setup processor clocks, signal routing, timers, etc.
    CoreInit();

    // Setup board peripherals (CODECs, external memory, etc.)
    BoardInit();

    // Initialize the target info
    AWEInstanceInit();

    // Setup audio DMA, interrupt priorities, etc.
    AudioInit();

    // Setup communication channel for monitoring and control
    USBMsgInit();

}	// End targetInit


//-----------------------------------------------------------------------------
// METHOD:  HAL_IncTick
// PURPOSE: User callback for millisecond update
//-----------------------------------------------------------------------------
void HAL_IncTick(void)
{
    uwTick++;
    g_nElapsedMilliSeconds++;
}


//-----------------------------------------------------------------------------
// METHOD:  aweuser_getCycleCount
// PURPOSE: Returns the current value in the counter
//-----------------------------------------------------------------------------
UINT32 aweuser_getCycleCount(void)
{
	UINT32 nCycles, nCycles1, nCycles2, nElapsedCycles, nElapsedMilliSecs;

    // This value is 400,000
    UINT32 nReloadValue = SysTick->LOAD + 1;

    DISABLE_INTERRUPTS();

    // Current COUNTDOWN value (215,999  - 0)
    nCycles1 = SysTick->VAL;

    //nElapsedMilliSecs = HAL_GetTick();
    nElapsedMilliSecs = g_nElapsedMilliSeconds;

    // Current COUNTDOWN value (215,999  - 0)
	nCycles2 = SysTick->VAL;

    ENABLE_INTERRUPTS();

    if (nCycles2 <= nCycles1)
    {
        nCycles = nCycles2;
    }
    else
    {
        nElapsedMilliSecs++;
        nCycles = nCycles1;
    }

    nElapsedCycles = (nElapsedMilliSecs * nReloadValue) + (nReloadValue - nCycles);

    // Additional correction needed
    if (nPrevCycles > nElapsedCycles)
    {
        g_nElapsedMilliSeconds++;

        nElapsedCycles = (g_nElapsedMilliSeconds * nReloadValue) + (nReloadValue - nCycles);
    }

    nPrevCycles = nElapsedCycles;

	return nElapsedCycles;

}   // End aweuser_getCycleCount


//-----------------------------------------------------------------------------
// METHOD:  aweuser_getElapsedCycles
// PURPOSE: Returns the cycle count between start time and end time
//-----------------------------------------------------------------------------
UINT32 aweuser_getElapsedCycles(UINT32 nStartTime, UINT32 nEndTime)
{
    UINT32 nElapsedTime;

	if (nEndTime > nStartTime)
	{
		nElapsedTime = nEndTime - nStartTime;
	}
	else
	{
		// Wrap around occurred
        nStartTime = (UINT32)0xFFFFFFFF - nStartTime;
		nElapsedTime = nStartTime + nEndTime + 1;
    }

	return nElapsedTime;

}   // End aweuser_getElapsedCycles


#if (HAS_FLASH_FILESYSTEM == 1)

//-----------------------------------------------------------------------------
// METHOD:  usrInitFlashFileSystem
// PURPOSE: This method is here
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
BOOL usrInitFlashFileSystem(void)
{
    BOOL bSuccess;
    UINT32 Buffer[4];

    // Prime the flash file system by issuing a read
    // The system returns invalid data on the first read
    bSuccess = usrReadFlashMemory(0, (UINT32 *)Buffer, 4);

    return bSuccess;

}   // End usrInitFlashFileSystem


//-----------------------------------------------------------------------------
// METHOD:  usrReadFlashMemory
// PURPOSE: Read from flash memory device
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
BOOL usrReadFlashMemory(UINT32 nFlashAddress, UINT32 * pBuffer, UINT32 nDWordsToRead)
{
    UINT8 nStatus;
    UINT32 nBytesToRead = nDWordsToRead * 4;
    UINT32 nEndingAddress = nFlashAddress + nBytesToRead;

 	// Flash address must be on a 4 byte boundary since writing DWords
	if ( (nFlashAddress & 0x00000003) != 0)
	{
		return FAILURE;
	}

    if (nEndingAddress >= FILE_SYSTEM_START_OFFSET + FLASH_MEMORY_SIZE_IN_BYTES)
    {
        return FAILURE;
    }

    if (BSP_QSPI_Read((uint8_t *)pBuffer, nFlashAddress, nBytesToRead) != QSPI_OK)
    {
        //printf("usrReadFlashMemory failed\n");
        return FAILURE;
    }

    do
    {
        nStatus = BSP_QSPI_GetStatus();
    }
    while (nStatus == QSPI_BUSY);

    if (nStatus != QSPI_OK)
    {
        //printf("Error: usrReadFlashMemory failed - nStatus = %d\n", nStatus);
    }

	return SUCCESS;

}	// End usrReadFlashMemory


//-----------------------------------------------------------------------------
// METHOD:  usrWriteFlashMemory
// PURPOSE: Write to flash memory device
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
BOOL usrWriteFlashMemory(UINT32 nFlashAddress, UINT32 * pBuffer, UINT32 nDWordsToWrite)
{
	UINT32 n;
    UINT32 nValueRead, nValueWritten;
    UINT32 nAddr;
    UINT32 nBytesToWrite = nDWordsToWrite * 4;

    UINT32 nEndingAddress = nFlashAddress + nBytesToWrite;

	// Flash address must be on a 4 byte boundary since writing DWords
	if ( (nFlashAddress & 0x00000003) != 0)
	{
        //printf("usrWriteFlashMemory failed\n");
		return FAILURE;
	}

    if (nEndingAddress >= FILE_SYSTEM_START_OFFSET + FLASH_MEMORY_SIZE_IN_BYTES)
    {
        //printf("usrWriteFlashMemory failed\n");
        return FAILURE;
    }

    if (BSP_QSPI_Write((uint8_t *)pBuffer, nFlashAddress, nBytesToWrite) != QSPI_OK)
    {
        //printf("usrWriteFlashMemory BSP_QSPI_Write failed\n");
        return FAILURE;
    }

    n = 0;

    for (nAddr = nFlashAddress; nAddr < nEndingAddress; nAddr += 4)
    {
        BSP_QSPI_Read((uint8_t *)&nValueRead, nAddr, 4);

        nValueWritten = pBuffer[n++];

        if (nValueWritten != nValueRead)
        {
            //printf("ERROR: usrWriteFlashMemory read bytes did not match bytes written\n");
            //printf("Address: 0x%.8X Wrote 0x%X and read 0x%X\n", nAddr, nValueWritten, nValueRead);
            return FAILURE;
        }
    }

	return SUCCESS;

}	// End usrWriteFlashMemory


//-----------------------------------------------------------------------------
// METHOD:  usrEraseFlashMemorySector
// PURPOSE: Erase Flash Memory used for Flash File System
//-----------------------------------------------------------------------------
AWE_OPTIMIZE_FOR_SPACE
AWE_FW_SLOW_CODE
BOOL usrEraseFlashMemorySector(UINT32 nStartingAddress, UINT32 nNumberOfSectors)
{
    UINT8 nStatus;
    UINT32 n;
    UINT32 nSectorAddress;

    if (nStartingAddress == 0 && nNumberOfSectors == 16384)
    {
        //printf("Start full erase\n");

        // This operation takes almost a full minute
        nStatus = BSP_QSPI_Erase_Chip();

        //printf("*** Full erase complete ***\n");

        if (nStatus != QSPI_OK)
        {
            return FAILURE;
        }

        return SUCCESS;

    }

    nSectorAddress = nStartingAddress & 0x1000;

    for (n = 0; n < nNumberOfSectors; n++)
    {
        BSP_QSPI_Erase_Block(nSectorAddress);

        do
        {
            nStatus = BSP_QSPI_GetStatus();
        }
        while (nStatus == QSPI_BUSY);

        if (nStatus != QSPI_OK)
        {
            //printf("Error: BSP_QSPI_Erase_Block failed for block %d, nStatus = %d\n", n+1, nStatus);
        }

        nSectorAddress += 0x1000;

        HAL_Delay(100);
    }

	return SUCCESS;

}	// End usrEraseFlashMemorySector


//-----------------------------------------------------------------------------
// METHOD:  AWEIdleLoop
// PURPOSE: AWE Idle loop processing
//-----------------------------------------------------------------------------
void AWEIdleLoop(void)
{
    BOOL bMoreProcessingRequired = FALSE;

    while(TRUE)
    {
        // Reset the pump count to show that the idle loop got serviced
        g_nPumpCount = 0;

       	if (g_bPacketReceived)
        {
            g_bPacketReceived = FALSE;

            awe_packetProcess(&g_AWEInstance);

            USBSendReply(&g_AWEInstance);
        }

       // Process any local controls
        if (awe_audioIsStarted(&g_AWEInstance) )
        {
            UINT32 classID;
            INT32 nButtonState, nLED1State;

            if (g_bDeferredProcessingRequired || bMoreProcessingRequired)
            {
                g_bDeferredProcessingRequired = FALSE;
                bMoreProcessingRequired = awe_deferredSetCall(&g_AWEInstance);
            }

            // Does the current AWE model have a SinkInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&g_AWEInstance, AWE_SinkInt1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class SinkInt
                if (classID == AWE_SinkInt1_classID)
                {
                    // SinkInt module (value is an array)
                    awe_ctrlGetValue(&g_AWEInstance, AWE_SinkInt1_value_HANDLE, (void *)&nLED1State, 0, 1);

                    // Set the current state of LED1
                    awe_pltGPIOSetPin(1, nLED1State);
                }
            }

            // Does the current AWE model have a DCSourceInt module with this control object ID?
            if (awe_ctrlGetModuleClass(&g_AWEInstance, AWE_DC1_value_HANDLE, &classID) == OBJECT_FOUND)
            {
                // Check that module assigned this object ID is of module class DCSourceInt
                if (classID == AWE_DC1_classID)
                {
                     // Get the current state of the blue user button
                    awe_pltGPIOGetPin(1, (UINT32 *)&nButtonState);

                    // DCSourceInt module (value is a scalar)
                    awe_ctrlSetValue(&g_AWEInstance, AWE_DC1_value_HANDLE, (void *)&nButtonState, 0, 1);
                }
            }
        }

        if (g_bBlinkLED2ForBoardAlive)
        {
            static INT32 nLoopCount = 0;

            // Blink LED2 to show board is alive
            nLoopCount++;

            if (nLoopCount == 1000000)
            {
                // Indicate board running by toggling LED2
                BSP_LED_Toggle(LED2);

                nLoopCount = 0;
            }
        }

    }   // End while

}   // End AWEIdleLoop

#endif
