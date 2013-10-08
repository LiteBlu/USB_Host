/******************************************************************************
    USB Memory File Interface and Data Logger

This application is described by AN1145 "Using a USB Flash Drive on an
Embedded Host".  The application integrates the Microchip USB Embedded Host
stack with the Microchip Memory Disk Drive library to create, manipulate, and
display files on a generic USB Flash Drive.

******************************************************************************/

/******************************************************************************

* FileName:        USB Data Logger.c
* Dependencies:    USB Host library; project requires File System library
* Processor:       PIC24F
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½CompanyÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½) for its PICmicroÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ Microcontroller is intended and
supplied to you, the CompanyÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½s customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½AS ISÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¯ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¿ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â½ CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

*******************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "GenericTypeDefs.h"
#include "HardwareProfile.h"
#include "MDD File System/FSIO.h"
#include "usb_config.h"
#include "USB/usb.h"
#define USE_AND_OR
#include <i2c.h>
#if defined( __PIC32MX__ )
    #include "plib.h"
#endif

// *****************************************************************************
// *****************************************************************************
// Build Configuration
// *****************************************************************************
// *****************************************************************************

// When this label is defined, the project is built such that only USB support
// code is compiled.  No application-specific code is built.  The project in
// this form will not run.  It is used to determine the resources required by
// the USB support code.
//#define MINIMUM_BUILD

// When this label is defined, the SUSPEND and RESUME commands are supported.
//#define IMPLEMENT_SUSPEND_AND_RESUME

// When this label is defined, the soft (MSD) and hard (device) resets are
// supported.
#define IMPLEMENT_RESET

// When this label is defined, the WHO command is supported.
#define IMPLEMENT_WHO

// *****************************************************************************
// *****************************************************************************
// Constants
// *****************************************************************************
// *****************************************************************************

#if defined( __C30__)
    #define ADC_READING_POTENTIOMETER   ADC1BUF1     
    #define ADC_READING_TEMPERATURE     ADC1BUF0
    #define ADC_READING_VBUS            ADC1BUF2
    #if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ256GB210__) || defined(__PIC24FJ256DA210__)
        #define SCAN_MASK                   0x0130      // Mask for AN4, AN5, and AN8
    #elif defined(__PIC24FJ64GB004__) || defined(__PIC24FJ64GB002__)
        //#define SCAN_MASK                   0x01C0      // Mask for AN6, AN7, and AN8
    #else
        #error "Device not supported.  Please add support."
    #endif
#elif defined( __PIC32MX__ )
    #define ADC_READING_POTENTIOMETER   ADC1BUF0
    #define ADC_READING_TEMPERATURE     ADC1BUF1
    #define ADC_READING_VBUS            ADC1BUF2
    #define SCAN_MASK               0x0114      // Mask for AN2, AN5 and AN8
#else
    #error Unsupported Processor
#endif
#define COPY_BUFFER_SIZE                MEDIA_SECTOR_SIZE //64   // Increased to improve copy efficiency
#define MAX_ALLOWED_CURRENT             500         // Maximum power we can supply in mA
#define MAX_BUFFERED_COMMANDS           8           // Must be a power of 2
#define MAX_COMMAND_LENGTH              50
#define MAX_LOG_BUFFER_SIZE             MEDIA_SECTOR_SIZE
#define NUM_LOG_BUFFERS                 2
#define VERSION                         "v1.00"

// RTCC Format Correlation Constants
// The 16-bit and 32-bit architectures use two different formats for the RTCC.
// These constants can be used to access the byte in a four-byte date or time
// array.  Note: No constant is given for the weekday, because the weekday is
// stored in a different byte.  This is done to flag the user to take special
// care when writing code that utilizes the weekday.

#if defined( __C30__ )
    #define DEFAULT_YEARS               0x0013
    #define DEFAULT_MONTH_DAY           0x0822
    #define DEFAULT_WEEKDAY_HOURS       0x0304
    #define DEFAULT_MINUTES_SECONDS     0x2100

    #define INDEX_HOURS                 2
    #define INDEX_MINUTES               1
    #define INDEX_SECONDS               0
    #define INDEX_YEAR                  2
    #define INDEX_MONTH                 1
    #define INDEX_DAY                   0


#elif defined (__PIC32MX__ )
    #define PIC32MX_TIMER2_INTERRUPT    0x00000100
    #define PIC32MX_TIMER3_INTERRUPT    0x00001000

    #define DEFAULT_DATE                0x07081503
    #define DEFAULT_TIME                0x04210000

    #define INDEX_HOURS                 3
    #define INDEX_MINUTES               2
    #define INDEX_SECONDS               1
    #define INDEX_YEAR                  3
    #define INDEX_MONTH                 2
    #define INDEX_DAY                   1
#else
    #error Cannot set up RTCC constants
#endif

// We are taking Timer 3 for an acquisition timer.

// NOTE - The datasheet doesn't state this, but the timer does get reset to 0
// after a period register match.  So we don't have to worry about resetting
// the timer manually.

#define STOP_TIMER_IN_IDLE_MODE             0x2000
#define TIMER_SOURCE_INTERNAL               0x0000
#define TIMER_ON                            0x8000
#define GATED_TIME_DISABLED                 0x0000
#define TIMER_16BIT_MODE                    0x0000

#if defined( __C30__ )
    #define TIMER_PRESCALER_1               0x0000
    #define TIMER_PRESCALER_8               0x0010
    #define TIMER_PRESCALER_64              0x0020
    #define TIMER_PRESCALER_256             0x0030
    #define TIMER_INTERRUPT_PRIORITY        0x0002
#elif defined( __PIC32MX__ )
    #define TIMER_PRESCALER_1               0x0000
    #define TIMER_PRESCALER_2               0x0010
    #define TIMER_PRESCALER_4               0x0020
    #define TIMER_PRESCALER_8               0x0030
    #define TIMER_PRESCALER_16              0x0040
    #define TIMER_PRESCALER_32              0x0050
    #define TIMER_PRESCALER_64              0x0060
    #define TIMER_PRESCALER_256             0x0070
#else
    #error No timer constants
#endif

// *****************************************************************************
// *****************************************************************************
// Configuration Bits
// *****************************************************************************
// *****************************************************************************
#if defined(__C30__)
    #define PLL_96MHZ_OFF   0xFFFF
    #define PLL_96MHZ_ON    0xF7FF

    // Configuration Bit settings  for an Explorer 16 with USB PICtail Plus
    //      Primary Oscillator:             HS
    //      Internal USB 3.3v Regulator:    Disabled
    //      IOLOCK:                         Set Once
    //      Primary Oscillator Output:      Digital I/O
    //      Clock Switching and Monitor:    Both disabled
    //      Oscillator:                     Primary with PLL
    //      USB 96MHz PLL Prescale:         Divide by 2
    //      Internal/External Switch Over:  Enabled
    //      WDT Postscaler:                 1:32768
    //      WDT Prescaler:                  1:128
    //      WDT Window:                     Non-window Mode
    //      Comm Channel:                   EMUC2/EMUD2
    //      Clip on Emulation Mode:         Reset into Operation Mode
    //      Write Protect:                  Disabled
    //      Code Protect:                   Disabled
    //      JTAG Port Enable:               Disabled

    #if defined(__PIC24FJ256GB110__)
        _CONFIG2(FNOSC_PRIPLL & POSCMOD_HS & PLL_96MHZ_ON & PLLDIV_DIV2) // Primary HS OSC with PLL, USBPLL /2
        _CONFIG1(JTAGEN_OFF & FWDTEN_OFF & ICS_PGx2)   // JTAG off, watchdog timer off
    #elif defined(__PIC24FJ64GB004__) || defined(__PIC24FJ64GB002__)
        _CONFIG1(WDTPS_PS1 & FWPSA_PR32 & WINDIS_OFF & FWDTEN_OFF & ICS_PGx1 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & I2C1SEL_PRI & IOL1WAY_OFF & OSCIOFNC_ON & FCKSM_CSECME & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_ON)
        _CONFIG3(WPFP_WPFP0 & SOSCSEL_IO & WUTSEL_LEG & WPDIS_WPDIS & WPCFG_WPCFGDIS & WPEND_WPENDMEM)
        _CONFIG4(DSWDTPS_DSWDTPS3 & DSWDTOSC_LPRC & RTCOSC_LPRC & DSBOREN_OFF & DSWDTEN_OFF)
    #elif defined(__PIC24FJ256GB106__)
        _CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
        _CONFIG2( 0xF7FF & IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_HS & FNOSC_PRIPLL & PLLDIV_DIV3 & IOL1WAY_ON)
    #elif defined(__PIC24FJ256DA210__) || defined(__PIC24FJ256GB210__)
        _CONFIG1(FWDTEN_OFF & ICS_PGx2 & GWRP_OFF & GCP_OFF & JTAGEN_OFF)
        _CONFIG2(POSCMOD_HS & IOL1WAY_ON & OSCIOFNC_ON & FCKSM_CSDCMD & FNOSC_PRIPLL & PLL96MHZ_ON & PLLDIV_DIV2 & IESO_OFF)
    #endif
#elif defined (__PIC32MX__)
    // Original
    //    #pragma config PLLMUL = MUL_16, PLLIDIV = DIV_2, FWDTEN = OFF
    //    #pragma config POSCMOD = HS, FNOSC = PRIPLL
    // Define the device configuration
    //  System Clock = 64 MHz,  Peripherial Bus = 8 MHz
    //  Primary Osc w/PLL (XT+,HS+,EC+PLL)
    //  Input Divider    2x Divider
    //  Multiplier      16x Multiplier
    //  WDT disabled
    //  Other options are don't care
    #if defined(USE_USB_PLL)
        #ifdef USB_A0_SILICON_WORK_AROUND
            #pragma config UPLLEN   = OFF       // USB PLL Enabled (A0 bit inverted)
        #else
            #pragma config UPLLEN   = ON        // USB PLL Enabled
        #endif
    #else
        #ifdef USB_A0_SILICON_WORK_AROUND
            #pragma config UPLLEN   = ON       // USB PLL Disabled (A0 bit inverted)
        #else
            #pragma config UPLLEN   = OFF        // USB PLL Disabled
        #endif
    #endif
    #pragma config UPLLIDIV = DIV_2     // USB PLL Input Divider
    #if defined(RUN_AT_48MHZ) || defined(RUN_AT_24MHZ)
        #pragma config FPLLMUL   = MUL_24 //15    // PLL Multiplier
    #elif defined(RUN_AT_60MHZ)
        #pragma config FPLLMUL   = MUL_15    // PLL Multiplier
    #endif
    #pragma config FPLLIDIV = DIV_2     // PLL Input Divider
    #pragma config FPLLODIV = DIV_1     // PLL Output Divider
    #pragma config FPBDIV   = DIV_1     // PB Clock Divisor
    #pragma config FWDTEN   = OFF       // Watchdog Timer
    #pragma config WDTPS    = PS1       // Watchdog Timer Postscale
    #pragma config FCKSM    = CSDCMD    // Clock Switching & Fail Safe Clock Monitor
    #pragma config OSCIOFNC = OFF       // CLKO Enable
    #pragma config POSCMOD  = HS        // Primary Oscillator
    #pragma config IESO     = OFF       // Internal/External Switch-over
    #pragma config FSOSCEN  = ON        // Secondary Oscillator Enable (KLO was off)
    #pragma config FNOSC    = PRIPLL    // Oscillator Selection
    #pragma config CP       = OFF       // Code Protect
    #pragma config BWP      = OFF       // Boot Flash Write Protect
    #pragma config PWP      = OFF       // Program Flash Write Protect
    #pragma config ICESEL   = ICS_PGx2  // ICE/ICD Comm Channel Select
    #pragma config DEBUG    = OFF       // Background Debugger Enable

#else
    #error Cannot set up configuration bits.
#endif
// *****************************************************************************
// *****************************************************************************
// Data Structures
// *****************************************************************************
// *****************************************************************************

typedef struct _COMMAND
{
    char        buffer[MAX_COMMAND_LENGTH];
    BYTE        index;
    BYTE        command;
    BYTE        escFirstChar;
    struct
    {
        BYTE    reading             : 1;
        BYTE    escNeedFirstChar    : 1;
        BYTE    escNeedSecondChar   : 1;
    };
} COMMAND;


typedef enum _COMMANDS
{
    COMMAND_NO_COMMAND,
    COMMAND_APPEND,
    COMMAND_BAUD,
    COMMAND_CD,
    COMMAND_COPY,
    COMMAND_DATE,
    COMMAND_DEL,
    COMMAND_DIR,
    COMMAND_HELP,
    COMMAND_LOG,
    COMMAND_MD,
    COMMAND_MODE,
    COMMAND_READ,
    COMMAND_RD,
    COMMAND_REN,
#ifdef IMPLEMENT_RESET
    COMMAND_RESET,
#endif
#ifdef IMPLEMENT_SUSPEND_AND_RESUME
    COMMAND_RESUME,
    COMMAND_SUSPEND,
#endif
    COMMAND_TIME,
    COMMAND_TYPE,
    COMMAND_WRITE,
#ifdef IMPLEMENT_WHO
    COMMAND_WHO,
#endif
    COMMAND_UNKNOWN
} COMMANDS;


typedef struct _LOG_DATA
{
    BYTE        buffer[MAX_LOG_BUFFER_SIZE];
    WORD        index;
    BOOL        bufferFull;
} LOG_DATA;


typedef struct _LOGGER_STATUS
{
    #if defined( __PIC32MX__ )
        // PIC32 does not have atomic bit-field set/clear instructions, so 
        // flags that are modified in both an ISR and main-line code must be
        // separate variables or we risk inadvertantly changing other flags
        // in the same byte.
        BOOL                mediaPresent;
        union
        {
            BYTE    value;
            struct
            {    
                BYTE        mediaPresent            : 1;
                BYTE        cannotInitialize        : 1;
                BYTE        overcurrentStateUSB     : 1;
                BYTE        readingPotentiometer    : 1;
                BYTE        readingTemperature      : 1;
            };
        };
    #elif defined(__C30__)
        union
        {
            BYTE    value;
            struct
            {    
                BYTE        mediaPresent            : 1;
                BYTE        cannotInitialize        : 1;
                BYTE        overcurrentStateUSB     : 1;
                BYTE        readingPotentiometer    : 1;
                BYTE        readingTemperature      : 1;
            };
        };
    #endif
} LOGGER_STATUS;


typedef struct _OLD_COMMANDS
{
    char        lines[MAX_BUFFERED_COMMANDS][MAX_COMMAND_LENGTH];
    BYTE        oldest;
    BYTE        newest;
    BYTE        showing;
} OLD_COMMANDS;


#if defined( __C30__ )

    // PIC24 RTCC Structure
    typedef union
    {
        struct
        {
            unsigned char   mday;       // BCD codification for day of the month, 01-31
            unsigned char   mon;        // BCD codification for month, 01-12
            unsigned char   year;       // BCD codification for years, 00-99
            unsigned char   reserved;   // reserved for future use. should be 0
        };                              // field access
        unsigned char       b[4];       // byte access
        unsigned short      w[2];       // 16 bits access
        unsigned long       l;          // 32 bits access
    } PIC24_RTCC_DATE;

    // PIC24 RTCC Structure
    typedef union
    {
        struct
        {
            unsigned char   sec;        // BCD codification for seconds, 00-59
            unsigned char   min;        // BCD codification for minutes, 00-59
            unsigned char   hour;       // BCD codification for hours, 00-24
            unsigned char   weekday;    // BCD codification for day of the week, 00-06
        };                              // field access
        unsigned char       b[4];       // byte access
        unsigned short      w[2];       // 16 bits access
        unsigned long       l;          // 32 bits access
    } PIC24_RTCC_TIME;

#endif


typedef struct 
{
    WORD        vid;
    WORD        pid;
} THUMB_DRIVE_ID;

    
typedef struct _VOLUME_INFO
{
    char        label[12];
    BYTE        valid;
} VOLUME_INFO;


// *****************************************************************************
// *****************************************************************************
// Internal Function Prototypes
// *****************************************************************************
// *****************************************************************************

void    DisableUSBInterrupts( void );
void    EnableUSBInterrupts( void );
BYTE    GetCommand( void );
DWORD   GetCurrentTick( void );
void    GetOneWord( char *buffer );
DWORD   GetUserDate( void );
DWORD   GetUserTime( void );
void    InitializeAnalogMonitor( void );
void    InitializeClock( void );
void    InitializeCommand( void );
void    MonitorMedia( void );
void    MonitorUser( void );
void    MonitorVBUS( void );
void    PrintFileInformation( SearchRec searchRecord );
void    RedoCommandPrompt( void );
void    ReplaceCommandLine( void );
void    WriteOneBuffer( FSFILE *fptr, BYTE *data, WORD size );
void    InitI2C(void);
void    WriteByteEEP(BYTE address, BYTE data);
BYTE    ReadByteEEP(BYTE address);

#if defined( __C30__ )
    DWORD   PIC24RTCCGetTime( void );
    DWORD   PIC24RTCCGetDate( void );
    void    PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds );
    void    PIC24RTCCSetDate( WORD xx_year, WORD month_day );
    void    UnlockRTCC( void );
#endif

// Define Text-IO routines as appropriate for the specific platform.
#if 0   // To Do:  Replace these definitions as needed
    #define InitTextIO()
    #define PutChar(c)
    #define PutHex(h)
    #define PrintString(s)
#else
    #define InitTextIO(b)   UART2Init(b)
    #define PutChar(c)      LED_COMMS = 1;\
                            UART2PutChar(c);\
                            LED_COMMS = 0
    #define PutHex(h)       LED_COMMS = 1;\
                            UART2PutHex(h);\
                            LED_COMMS = 0
    #define PrintString(s)  LED_COMMS = 1; \
                            UART2PrintString(s); \
                            LED_COMMS = 0
#endif

// *****************************************************************************
// *****************************************************************************
// Macros
// *****************************************************************************
// *****************************************************************************

#define IsNum(c)            ((('0' <= c) && (c <= '9')) ? TRUE : FALSE)
#define UpperCase(c)        (('a'<= c) && (c <= 'z') ? c - 0x20 : c)
#define SkipWhiteSpace()    { while (commandInfo.buffer[commandInfo.index] == ' ') commandInfo.index++; }

// Let compile time pre-processor calculate the PR1 (period)
#define FOSC                72E6
#define PB_DIV              8
#define PRESCALE            256
#define TOGGLES_PER_SEC     1
#define T3_TICK             (FOSC/PB_DIV/PRESCALE/TOGGLES_PER_SEC)


// *****************************************************************************
// *****************************************************************************
// Global Variables
// *****************************************************************************
// *****************************************************************************

#ifndef MINIMUM_BUILD

    OLD_COMMANDS        commandBuffer;
    COMMAND             commandInfo;
    volatile BYTE       logBufferReading;
    BYTE                logBufferWriting;
    LOGGER_STATUS       loggerStatus;
    volatile DWORD      currentTick;
    volatile LOG_DATA   logData[NUM_LOG_BUFFERS];
    VOLUME_INFO         volume;

    #if defined( __C30__ )
        PIC24_RTCC_DATE currentDate;
        PIC24_RTCC_TIME currentTime;
        PIC24_RTCC_TIME previousTime;
    #elif defined( __PIC32MX__ )
        rtccDate        currentDate;
        rtccTime        currentTime;
        rtccTime        previousTime;
    #else
        #error No time structure defined
    #endif

    // These thumb drives appear to have electrical issues that makes their
    // operation at full speed marginal.
    THUMB_DRIVE_ID      problemThumbDrives[] = { {0x0930, 0x6545} };
    #define PROBLEM_THUMB_DRIVE_COUNT   (sizeof(problemThumbDrives) / sizeof(THUMB_DRIVE_ID))
    
#endif

//******************************************************************************
//******************************************************************************
//******************************************************************************
// Main
//******************************************************************************
//******************************************************************************
//******************************************************************************
#define LED_COMMS           LATAbits.LATA4
#define LED_MEDIA           LATBbits.LATB5
#define OUT_MEDIA           LATBbits.LATB7
        BYTE                termMode;

int main (void)
{
    #ifndef MINIMUM_BUILD

        FSFILE              *filePointer1;
        FSFILE              *filePointer2;
        char                oneChar;
        char                param1[MAX_COMMAND_LENGTH];
        char                param2[MAX_COMMAND_LENGTH];
        BOOL                readingPot;
        BOOL                resetting = FALSE;
        BYTE                returnCode;
        SearchRec           searchRecord;
        int                 value;


        #if defined( __PIC24FJ256GB110__ ) || defined(__PIC24FJ256GB210__)
            // PPS - Configure U2RX - put on pin 49 (RP10)
            RPINR19bits.U2RXR = 10;

            // PPS - Configure U2TX - put on pin 50 (RP17)
            RPOR8bits.RP17R = 5;
            
            OSCCON = 0x3302;    // Enable secondary oscillator
            CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

            TRISD = 0x00C0;

       #elif defined(__PIC24FJ64GB004__) || defined(__PIC24FJ64GB002__)
            OSCCON = 0x3302;    // Enable secondary oscillator
            CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

        	//On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
        	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
        	//This allows the device to power up at a lower initial operating frequency, which can be
        	//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
        	//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
        	//power up the PLL.
            {
                unsigned int pll_startup_counter = 6000;
                CLKDIVbits.PLLEN = 1;
                while(pll_startup_counter--);
            }
        
            //Device switches over automatically to PLL output after PLL is locked and ready.
        
            // PPS - Configure U2RX - put on RA0/pin 2 (RP5)
            RPINR19bits.U2RXR = 5;
            TRISAbits.TRISA0 = 1;
    
            // PPS - Configure U2TX - put on RA1/pin 3 (RP6)
            RPOR3bits.RP6R = 5;

            TRISAbits.TRISA1 = 0;
            TRISAbits.TRISA4 = 0;
            TRISBbits.TRISB5 = 0;
            TRISBbits.TRISB7 = 0;
            TRISBbits.TRISB3 = 0;

            CNPU1bits.CN6PUE = 1;
            CNPU1bits.CN7PUE = 1;

            AD1PCFG = 0xFFFF;   // Set analog pins to digital.

       #elif defined(__PIC24FJ256DA210__)
            OSCCON = 0x3302;    // Enable secondary oscillator
            CLKDIV = 0x0000;    // Set PLL prescaler (1:1)

        	//On the PIC24FJ256DA210 Family of USB microcontrollers, the PLL will not power up and be enabled
        	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
        	//This allows the device to power up at a lower initial operating frequency, which can be
        	//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
        	//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
        	//power up the PLL.
            {
                unsigned int pll_startup_counter = 6000;
                CLKDIVbits.PLLEN = 1;
                while(pll_startup_counter--);
            }
        
            //Device switches over automatically to PLL output after PLL is locked and ready.
        
            // PPS - Configure U2RX - put on RC14/pin 74 (RPI37)
            RPINR19bits.U2RXR = 37;
    
            // PPS - Configure U2TX - put on RF3/pin 51 (RP16)
            RPOR8bits.RP16R = 5;

            TRISFbits.TRISF3 = 0;

        #elif defined(__PIC32MX__)
        
            #if defined(RUN_AT_60MHZ)
                // Use OSCCON default
            #else
                OSCCONCLR = 0x38000000; //PLLODIV
                #if defined(RUN_AT_48MHZ)
                    OSCCONSET = 0x08000000; //PLLODIV /2
                #elif defined(RUN_AT_24MHZ)
                    OSCCONSET = 0x10000000; //PLLODIV /4
                #else
                    #error Cannot set OSCCON
                #endif
            #endif

            value = SYSTEMConfigWaitStatesAndPB( GetSystemClock() );

            // Enable the cache for the best performance
            CheKseg0CacheOn();

            INTEnableSystemMultiVectoredInt();

            value = OSCCON;
            while (!(value & 0x00000020))
            {
                value = OSCCON;    // Wait for PLL lock to stabilize
            }

            INTEnableInterrupts();

            AD1PCFG = 0xFFFF;   // Set analog pins to digital.
            TRISF = 0x00;

        #else
            #error Cannot initialize
        #endif

        //termMode = 1;
        InitI2C();
        BYTE baudrate;
        //WriteByteEEP(0x00, 0xFF);
        if (ReadByteEEP(0x00) != 0x01)
        {
            WriteByteEEP(0x00, 0x01);
            WriteByteEEP(0x01, 0x01);
            WriteByteEEP(0x02, 0x02);
        }
        //else
        //{
        // [2400|4800|9600|14400|19200|28800|38400|57600|115200]
        termMode = ReadByteEEP(0x01);
        baudrate = ReadByteEEP(0x02);
        switch (baudrate)
        {
            case 0:
                InitTextIO(BAUD2400);
            break;

            case 1:
                InitTextIO(BAUD4800);
            break;

            case 2:
                InitTextIO(BAUD9600);
            break;

            case 3:
                InitTextIO(BAUD14400);
            break;

            case 4:
                InitTextIO(BAUD19200);
            break;

            case 5:
                InitTextIO(BAUD28800);
            break;

            case 6:
                InitTextIO(BAUD38400);
            break;

            case 7:
                InitTextIO(BAUD57600);
            break;

            case 8:
                InitTextIO(BAUD115200);
            break;
        }
       // }

        //InitTextIO(BAUD9600);
        if (termMode)
        {
            PrintString( "\r\n\r\n***** LiteBlu Explorer " VERSION " *****\r\n\r\n" );
        }

        // Initialize USB layers
        USBInitialize( 0 );


        // Initialize the RTCC
        #if defined( __C30__)
            // Turn on the secondary oscillator
            __asm__ ("MOV #OSCCON,w1");
            __asm__ ("MOV.b #0x02, w0");
            __asm__ ("MOV #0x46, w2");
            __asm__ ("MOV #0x57, w3");
            __asm__ ("MOV.b w2, [w1]");
            __asm__ ("MOV.b w3, [w1]");
            __asm__ ("MOV.b w0, [w1]");

            PIC24RTCCSetDate( DEFAULT_YEARS, DEFAULT_MONTH_DAY );
            PIC24RTCCSetTime( DEFAULT_WEEKDAY_HOURS, DEFAULT_MINUTES_SECONDS );

            RCFGCAL = 0x8000;
        #elif defined( __PIC32MX__)
            RtccInit();
            RtccSetDate( DEFAULT_DATE );
            RtccSetTime( DEFAULT_TIME );
            RtccEnable();
            mRtccWrEnable();
            while (RtccEnable() != RTCC_CLK_ON); // Make sure the RTCC is counting.
                //PutChar('~');
            mRtccWrDisable();
        #else
            #error No Real Time clock
        #endif


        //InitI2C();
        // Not initialised yet Save defaults


        WriteByteEEP(0x01, 0x02);
        //BYTE temp = ReadByteEEP(0x01);
        // Turn on the timer and the A/D converter to monitor inputs.
        //InitializeAnalogMonitor();
        InitializeClock();

        commandBuffer.newest    = MAX_BUFFERED_COMMANDS;
        commandBuffer.oldest    = MAX_BUFFERED_COMMANDS;
        commandBuffer.showing   = MAX_BUFFERED_COMMANDS;
        loggerStatus.value      = 0;
        volume.valid            = FALSE;
        InitializeCommand();

        while (1)
        {
            MonitorMedia();
            MonitorUser();

            if (GetCommand())
            {
                SkipWhiteSpace();
                switch ( commandInfo.command )
                {
                    case COMMAND_NO_COMMAND:
                        break;

                    case COMMAND_APPEND:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        if ((param1[0] == 0))
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "One parameters required\r\n" );
                        }
                        else
                        {
                            if ((filePointer2 = FSfopen( param1, "a" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error creating file\r\n" );
                                break;
                            }

                            oneChar = 0;
                            do
                            {
                                MonitorMedia();
                                if (U2STAbits.URXDA)
                                {
                                    oneChar = UART2GetChar();
                                    if (oneChar != 26)
                                    {
                                        if (termMode)
                                        {
                                            PutChar(oneChar);
                                        }
                                        FSfwrite( (BYTE*)&oneChar, 1, 1, filePointer2);
                                        if (oneChar == 0x0D)
                                        {
                                            oneChar = 0x0A;
                                            if (termMode)
                                            {
                                                PutChar(oneChar);
                                            }
                                            FSfwrite( (BYTE*)&oneChar, 1, 1, filePointer2);
                                        }
                                    }
                                }

                            } while (loggerStatus.mediaPresent &&(oneChar != 26));

                            FSfclose( filePointer2 );
                        }
                        break;

                    case COMMAND_BAUD:
                        GetOneWord( param1 );
                        if (!strcmp( "2400", param1 ))
                        {
                            WriteByteEEP(0x02, 0x00);
                            PrintString( "Baudrate changed to 2400\r\n" );
                            InitTextIO(BAUD2400);
                        }
                        else if (!strcmp( "4800", param1 ))
                        {
                            WriteByteEEP(0x02, 0x01);
                            PrintString( "Baudrate changed to 4800\r\n" );
                            InitTextIO(BAUD4800);
                        }
                        else if (!strcmp( "9600", param1 ))
                        {
                            WriteByteEEP(0x02, 0x02);
                            PrintString( "Baudrate changed to 9600\r\n" );
                            InitTextIO(BAUD9600);
                        }
                        else if (!strcmp( "14400", param1 ))
                        {
                            WriteByteEEP(0x02, 0x03);
                            PrintString( "Baudrate changed to 14400\r\n" );
                            InitTextIO(BAUD14400);
                        }
                        else if (!strcmp( "19200", param1 ))
                        {
                            WriteByteEEP(0x02, 0x04);
                            PrintString( "Baudrate changed to 19200\r\n" );
                            InitTextIO(BAUD19200);
                        }
                        else if (!strcmp( "28800", param1 ))
                        {
                            WriteByteEEP(0x02, 0x05);
                            PrintString( "Baudrate changed to 28800\r\n" );
                            InitTextIO(BAUD28800);
                        }
                        else if (!strcmp( "38400", param1 ))
                        {
                            WriteByteEEP(0x02, 0x06);
                            PrintString( "Baudrate changed to 38400\r\n" );
                            InitTextIO(BAUD38400);
                        }
                        else if (!strcmp( "57600", param1 ))
                        {
                            WriteByteEEP(0x02, 0x07);
                            PrintString( "Baudrate changed to 57600\r\n" );
                            InitTextIO(BAUD57600);
                        }
                        else if (!strcmp( "115200", param1 ))
                        {
                            WriteByteEEP(0x02, 0x08);
                            PrintString( "Baudrate changed to 115200\r\n" );
                            InitTextIO(BAUD115200);
                        }
                        else
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        break;
                    break;

                    case COMMAND_CD:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FSchdir( &(commandInfo.buffer[commandInfo.index]) ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error performing command\r\n" );
                            }
                        }
                        break;

                    case COMMAND_COPY:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        GetOneWord( param2 );
                        if ((param1[0] == 0) || (param2[0] == 0))
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Two parameters required\r\n" );
                        }
                        else
                        {
                            if (FindFirst( param1,
                                    ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "File not found\r\n" );
                            }
                            else if ((filePointer1 = FSfopen( param1, "r" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error opening file\r\n" );
                            }
                            else
                            {
                                if ((filePointer2 = FSfopen( param2, "w" )) == NULL)
                                {
                                    if (!termMode)
                                    {
                                        PrintString( "E:=" );
                                    }
                                    PrintString( "Error creating file\r\n" );
                                }
                                else
                                {
                                    WORD    charsRead;
                                    // We will use one of the log buffers to save stack space

                                    while (!FSfeof( filePointer1 ))
                                    {
                                        charsRead = FSfread( (BYTE *)logData[0].buffer, 1, COPY_BUFFER_SIZE, filePointer1 );
                                        if (charsRead)
                                        {
                                            if (FSfwrite( (BYTE *)logData[0].buffer, 1, charsRead, filePointer2) != charsRead)
                                            {
                                                if (!termMode)
                                                {
                                                    PrintString( "E:=" );
                                                }
                                                PrintString( "Error writing file\r\n" );
                                                break;
                                            }
                                        }
                                    }
                                    FSfclose( filePointer2 );
                                }
                                FSfclose( filePointer1 );
                            }
                        }
                        break;

                    case COMMAND_DATE:
                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (termMode)
                            {
                                PrintString( "Current date: " );
                            }

                            #if defined( __C30__)
                                currentDate.l = PIC24RTCCGetDate();
                            #elif defined( __PIC32MX__ )
                                currentDate.l   = RtccGetDate();
                            #else
                                #error Need DATE implementation.
                            #endif

                            PutChar( '2' );
                            PutChar( '0' );
                            PutHex( currentDate.year );
                            PutChar( '-' );
                            PutHex( currentDate.mon );
                            PutChar( '-' );
                            PutHex( currentDate.mday );
                            PrintString( "\r\n" );
                        }
                        else
                        {
                            // Set the current date.
                            DWORD_VAL   date;

                            date.Val = GetUserDate();

                            if (date.Val)
                            {
                                #if defined( __C30__)
                                    PIC24RTCCSetDate( date.w[1], date.w[0] );
                                #elif defined( __PIC32MX__ )
                                    RtccSetDate( date.Val );
                                    RtccEnable();
                                #else
                                    #error Need DATE implementation.
                                #endif
                            }
                            else
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Invalid date specified\r\n" );
                            }
                        }
                        break;

                    case COMMAND_DEL:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FindFirst( &(commandInfo.buffer[commandInfo.index]),
                                    ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "File not found\r\n" );
                            }
                            else
                            {
                                if (FSremove( &(commandInfo.buffer[commandInfo.index] )))
                                {
                                    if (!termMode)
                                    {
                                        PrintString( "E:=" );
                                    }
                                    PrintString( "Error performing command\r\n" );
                                }
                            }
                        }
                        break;

                    case COMMAND_DIR:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            strcpy( param1, "*.*" );
                        }
                        else
                        {
                            strcpy( param1, &(commandInfo.buffer[commandInfo.index]) );
                        }
                        if (!FindFirst( param1, ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
                        {
                            PrintFileInformation( searchRecord );
                            while (!FindNext( &searchRecord ))
                            {
                                PrintFileInformation( searchRecord );
                            }
                        }
                        else
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No files found\r\n" );
                        }
                        break;

                    case COMMAND_HELP:
                        PrintString( "\r\nLiteBlu Flash Drive Explorer " );
                        PrintString( VERSION );
                        PrintString( "\r\n\r\nAvailable commands:\r\n" );
                        PrintString( "    APPEND <file>        - Append input to end of existing <file>.\r\n");
                        PrintString( "                           Terminate data entry with Control-Z\r\n" );
                        PrintString( "    BAUD <value>         - Set Serial Port Baud Rate\r\n");
                        PrintString( "                           [2400|4800|9600|14400|19200|38400|57600|115200]\r\n");
                        PrintString( "    CD <name>            - Change Directory\r\n" );
                        PrintString( "    COPY <file1> <file2> - Copy <file1> to <file2>\r\n" );
                        PrintString( "    DATE [yyyy-mm-dd]    - Display Or Set The Date\r\n" );
                        PrintString( "    DEL <file>           - Delete File, Current Directory Only\r\n" );
                        PrintString( "    DIR [name]           - Display Directory\r\n" );
                        PrintString( "    HELP or ?            - Display Help\r\n" );
                        PrintString( "    LS [name]            - List Directory. Same as DIR\r\n");
                        PrintString( "    MD <name>            - Make Directory\r\n" );
                        PrintString( "    MODE <TERM|MCU>      - Set the mode. Terminal or Microcontroller\r\n");
                        PrintString( "    RD <name>            - Remove directory\r\n" );
                        PrintString( "    READ <file> <num>    - Read line <num> from <file>, <num> starts at 0\r\n");
                        PrintString( "    REN <file1> <file2>  - Rename [file1] to [file2]\r\n" );
#ifdef IMPLEMENT_RESET
                        PrintString( "    RESET <HARD|SOFT>    - Reset Media. Either Hard reset or Soft reset\r\n" );
#endif
#ifdef IMPLEMENT_SUSPEND_AND_RESUME
                        PrintString( "    RESUME               - resume a suspended USB device\r\n" );
                        PrintString( "    SUSPEND              - suspend attached USB device\r\n" );
#endif
                        PrintString( "    TIME [hh:mm:ss]      - Display or set the time (24 hr format)\r\n" );
                        PrintString( "    TYPE <file>          - Print out contents of file, current directory only\r\n\r\n" );
                        PrintString( "    WRITE <file>         - Create <file> from input.\r\n");
                        PrintString( "                           Terminate data entry with Control-Z\r\n");
#ifdef IMPLEMENT_WHO
                        PrintString( "    WHO                  - Display the VID and PID of the attached device\r\n\r\n" );
#endif
                        break;

                    case COMMAND_MD:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FSmkdir( &(commandInfo.buffer[commandInfo.index])))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error performing command\r\n" );
                            }
                        }
                        break;

                    case COMMAND_MODE:
                        GetOneWord( param1 );
                        if (!strcmp( "TERM", param1 ))
                        {
                            WriteByteEEP(0x01, 0x01);
                            termMode = 1;
                        }
                        else if (!strcmp( "MCU", param1 ))
                        {
                            WriteByteEEP(0x01, 0x00);
                            termMode = 0;
                        }
                        else
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        break;

                    case COMMAND_RD:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FSrmdir( &(commandInfo.buffer[commandInfo.index]), TRUE ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error performing command\r\n" );
                            }
                        }
                        break;

                    case COMMAND_READ:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        GetOneWord( param2 );
                        if (param1 == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FindFirst( param1,
                                    ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "File not found\r\n" );
                            }
                            else if ((filePointer1 = FSfopen( param1, "r" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error opening file\r\n" );
                            }
                            else if (param2[0] != 0)
                            {
                                WORD    charsRead;
                                WORD    i;
                                int    lineCounter = 0;
                                int    userLine;



                                userLine = atoi(param2);
                                while (!FSfeof( filePointer1 ))
                                {
                                    // Read one sector at a time.
                                    charsRead = FSfread( (BYTE *)logData[0].buffer, 1, MEDIA_SECTOR_SIZE, filePointer1 );
                                    if (charsRead > 0)
                                    {
                                        for (i=0; i<charsRead; i++)
                                        {
                                            if ((logData[0].buffer[i] == 0x0D))
                                            {
                                                continue;
                                            }
                                            else if ((logData[0].buffer[i] == 0x0A) && (logData[0].buffer[i-1] == 0x0D) )
                                            {
                                                lineCounter++;
                                                continue;
                                            }
                                            if (lineCounter == userLine)
                                            {
                                                PutChar( logData[0].buffer[i] );
                                            }

                                        }
                                    }
                                    else
                                    {
                                        if (termMode)
                                        {
                                            PrintString( "\r\n - Error reading file\r\n" );
                                        }
                                        else
                                        {
                                            PrintString( "E:=Error reading file\r\n");
                                        }
                                        break;
                                    }
                                }
                                PrintString( "\r\n" );
                                FSfclose( filePointer1 );
                            }
                            else
                            {
                                WORD    charsRead;
                                WORD    i;
                                int     lineCounter = 0;
                                char    lineCountString[6];

                                while (!FSfeof( filePointer1 ))
                                {
                                    // Read one sector at a time.
                                    charsRead = FSfread( (BYTE *)logData[0].buffer, 1, MEDIA_SECTOR_SIZE, filePointer1 );
                                    if (charsRead > 0)
                                    {
                                        for (i=0; i<charsRead; i++)
                                        {
                                            if ((logData[0].buffer[i] == 0x0A) && (logData[0].buffer[i-1] == 0x0D) )
                                            {
                                                lineCounter++;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (termMode)
                                        {
                                            PrintString( "\r\n - Error reading file\r\n" );
                                        }
                                        else
                                        {
                                            PrintString( "E:=Error reading file\r\n");
                                        }
                                        break;
                                    }
                                }
                                itoa(lineCountString, lineCounter + 1, 10);
                                PrintString( lineCountString );
                                PrintString( "\r\n" );
                                FSfclose( filePointer1 );
                            }
                        }
                        break;

                    case COMMAND_REN:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        GetOneWord( param2 );
                        if ((param1[0] == 0) || (param2[0] == 0))
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Two parameters required\r\n" );
                        }
                        else
                        {
                            if ((filePointer1 = FSfopen( param1, "r" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Cannot find file\r\n" );
                            }
                            else
                            {
                                if (FSrename( param2, filePointer1 ))
                                {
                                    if (!termMode)
                                    {
                                        PrintString( "E:=" );
                                    }
                                    PrintString( "Error performing command\r\n" );
                                }
                                FSfclose( filePointer1 );
                            }
                        }
                        break;

                    #if defined(IMPLEMENT_RESET)
                    case COMMAND_RESET:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        if (!strcmp( "SOFT", param1 ))
                        {
                            if (USBHostMSDSCSIMediaReset())
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error performing command\r\n" );
                            }
                        }
                        else if (!strcmp( "HARD", param1 ))
                        {
                            if (USBHostResetDevice( USB_SINGLE_DEVICE_ADDRESS ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error performing command\r\n" );
                            }
                        }
                        else
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        break;
                    #endif

                    #ifdef IMPLEMENT_SUSPEND_AND_RESUME
                    case COMMAND_RESUME:
                        if (commandInfo.buffer[commandInfo.index] != 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Extra parameter\r\n" );
                        }
//                        We cannot do the mediaPresent check.  If the device
//                        is suspended, this test will fail.
//                        else if (!loggerStatus.mediaPresent)
//                        {
//                            PrintString( "No media present.\r\n" );
//                        }
                        else
                        {
                            switch( USBHostResumeDevice( USB_SINGLE_DEVICE_ADDRESS ))
                            {
                                case USB_SUCCESS:
                                    PrintString( "Device resumed\r\n" );
                                    break;

                                case USB_UNKNOWN_DEVICE:
                                    PrintString( "Device not attached\r\n" );
                                    break;

                                case USB_ILLEGAL_REQUEST:
                                    PrintString( "Device cannot be resumed\r\n" );
                                    break;
                            }
                        }
                        break;

                    case COMMAND_SUSPEND:
                        if (commandInfo.buffer[commandInfo.index] != 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Extra parameter\r\n" );
                        }
                        else if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                        }
                        else
                        {
                            switch( USBHostSuspendDevice( USB_SINGLE_DEVICE_ADDRESS ))
                            {
                                case USB_SUCCESS:
                                    PrintString( "Device suspended\r\n" );
                                    break;

                                case USB_UNKNOWN_DEVICE:
                                    PrintString( "Device not attached\r\n" );
                                    break;

                                case USB_ILLEGAL_REQUEST:
                                    PrintString( "Device cannot be suspended\r\n" );
                                    break;
                            }
                        }
                        break;
                    #endif

                    case COMMAND_TIME:
                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            // Display the current time.
                            if (termMode)
                            {
                                PrintString( "Current time: " );
                            }

                            #if defined( __C30__)
                                currentTime.l = PIC24RTCCGetTime();
                            #elif defined( __PIC32MX__ )
                                currentTime.l = RtccGetTime();
                            #else
                                #error Need TIME implementation.
                            #endif

                            PutHex( currentTime.hour );
                            PutChar( ':' );
                            PutHex( currentTime.min );
                            PutChar( ':' );
                            PutHex( currentTime.sec );
                            PrintString( "\r\n" );
                        }
                        else
                        {
                            // Set the current date.
                            DWORD_VAL   time;

                            time.Val = GetUserTime();

                            if (time.Val)
                            {
                                #if defined( __C30__)
                                    PIC24RTCCSetTime( time.w[1], time.w[0] );
                                #elif defined( __PIC32MX__ )
                                    RtccSetTime( time.Val );
                                    RtccEnable();
                                #else
                                    #error Need TIME implementation.
                                #endif
                            }
                            else
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Invalid time specified\r\n" );
                            }
                        }
                        break;

                    case COMMAND_TYPE:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        if (commandInfo.buffer[commandInfo.index] == 0)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "Parameter required\r\n" );
                        }
                        else
                        {
                            if (FindFirst( &(commandInfo.buffer[commandInfo.index]),
                                    ATTR_DIRECTORY | ATTR_ARCHIVE | ATTR_READ_ONLY | ATTR_HIDDEN, &searchRecord ))
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "File not found\r\n" );
                            }
                            else if ((filePointer1 = FSfopen( &(commandInfo.buffer[commandInfo.index]), "r" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error opening file\r\n" );
                            }
                            else
                            {
                                WORD    charsRead;
                                WORD    i;

                                while (!FSfeof( filePointer1 ))
                                {
                                    // Read one sector at a time.
                                    charsRead = FSfread( (BYTE *)logData[0].buffer, 1, MEDIA_SECTOR_SIZE, filePointer1 );
                                    if (charsRead > 0)
                                    {
                                        for (i=0; i<charsRead; i++)
                                        {
                                            PutChar( logData[0].buffer[i] );
                                        }
                                    }
                                    else
                                    {
                                        if (termMode)
                                        {
                                            PrintString( "\r\n - Error reading file\r\n" );
                                        }
                                        else
                                        {
                                            PrintString( "E:=Error reading file\r\n");
                                        }
                                        break;
                                    }
                                }
                                PrintString( "\r\n" );
                                FSfclose( filePointer1 );
                            }
                        }
                        break;

                    #ifdef IMPLEMENT_WHO
                    case COMMAND_WHO:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );//UART2PrintString( "No media present.\r\n" );
                            break;
                        }

                        {
                            USB_DEVICE_DESCRIPTOR   *pDescriptor;

                            pDescriptor = (USB_DEVICE_DESCRIPTOR *)USBHostGetDeviceDescriptor( 1 );
                            PrintString( "VID: 0x" );
                            PutHex( pDescriptor->idVendor >> 8 );
                            PutHex( pDescriptor->idVendor );
                            PrintString( " PID: 0x" );
                            PutHex( pDescriptor->idProduct >> 8 );
                            PutHex( pDescriptor->idProduct );
                            PrintString( "\r\n" );
                        }
                        break;
                    #endif

                    case COMMAND_WRITE:
                        if (!loggerStatus.mediaPresent)
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "No media present\r\n" );
                            break;
                        }

                        GetOneWord( param1 );
                        if ((param1[0] == 0))
                        {
                            if (!termMode)
                            {
                                PrintString( "E:=" );
                            }
                            PrintString( "One parameters required\r\n" );
                        }
                        else
                        {
                            if ((filePointer2 = FSfopen( param1, "w" )) == NULL)
                            {
                                if (!termMode)
                                {
                                    PrintString( "E:=" );
                                }
                                PrintString( "Error creating file\r\n" );
                                break;
                            }

                            oneChar = 0;
                            do
                            {
                                MonitorMedia();
                                if (U2STAbits.URXDA)
                                {
                                    oneChar = UART2GetChar();
                                    if (oneChar != 26)
                                    {
                                        if (termMode)
                                        {
                                            PutChar(oneChar);
                                        }
                                        FSfwrite( (BYTE*)&oneChar, 1, 1, filePointer2);
                                        if (oneChar == 0x0D)
                                        {
                                            oneChar = 0x0A;
                                            if (termMode)
                                            {
                                                PutChar(oneChar);
                                            }
                                            FSfwrite( (BYTE*)&oneChar, 1, 1, filePointer2);
                                        }
                                    }
                                }

                            } while (loggerStatus.mediaPresent &&(oneChar != 26));

                            FSfclose( filePointer2 );
                        }
                        break;

                    default:
                        if (!termMode)
                        {
                            PrintString( "E:=" );
                        }
                        PrintString( "Unsupported command\r\n" );
                        break;
                }
                InitializeCommand();
            }
        }
    #endif
}


//******************************************************************************
//******************************************************************************
// USB Support Functions
//******************************************************************************
//******************************************************************************

/****************************************************************************
  Function:
    BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event,
                void *data, DWORD size )

  Summary:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.

  Description:
    This is the application event handler.  It is called when the stack has
    an event that needs to be handled by the application layer rather than
    by the client driver.  If the application is able to handle the event, it
    returns TRUE.  Otherwise, it returns FALSE.

  Precondition:
    None

  Parameters:
    BYTE address    - Address of device where event occurred
    USB_EVENT event - Identifies the event that occured
    void *data      - Pointer to event-specific data
    DWORD size      - Size of the event-specific data

  Return Values:
    TRUE    - The event was handled
    FALSE   - The event was not handled

  Remarks:
    The application may also implement an event handling routine if it
    requires knowledge of events.  To do so, it must implement a routine that
    matches this function signature and define the USB_HOST_APP_EVENT_HANDLER
    macro as the name of that function.
  ***************************************************************************/

BOOL USB_ApplicationEventHandler( BYTE address, USB_EVENT event, void *data, DWORD size )
{
    #ifndef MINIMUM_BUILD
        switch( event )
        {
            case EVENT_VBUS_REQUEST_POWER:
                // The data pointer points to a byte that represents the amount of power
                // requested in mA, divided by two.  If the device wants too much power,
                // we reject it.
                if (((USB_VBUS_POWER_EVENT_DATA*)data)->current <= (MAX_ALLOWED_CURRENT / 2))
                {
                    return TRUE;
                }
                else
                {
                    PrintString( "\r\n***** USB Error - device requires too much current *****\r\n" );
                    RedoCommandPrompt();
                }
                break;

            case EVENT_VBUS_RELEASE_POWER:
                // Turn off Vbus power.
                // The PIC24F with the Explorer 16 cannot turn off Vbus through software.
                return TRUE;
                break;

            case EVENT_HUB_ATTACH:
                PrintString( "\r\n***** USB Error - hubs are not supported *****\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

            case EVENT_UNSUPPORTED_DEVICE:
                PrintString( "\r\n***** USB Error - device is not supported *****\r\n" );
                PrintString( "   If the device is a thumb drive, it probably uses a\r\n" );
                PrintString( "   transport protocol other than SCSI, such as SFF-8070i.\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

            case EVENT_CANNOT_ENUMERATE:
                PrintString( "\r\n***** USB Error - cannot enumerate device *****\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

            case EVENT_CLIENT_INIT_ERROR:
                PrintString( "\r\n***** USB Error - client driver initialization error *****\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

            case EVENT_OUT_OF_MEMORY:
                PrintString( "\r\n***** USB Error - out of heap memory *****\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

            case EVENT_UNSPECIFIED_ERROR:   // This should never be generated.
                PrintString( "\r\n***** USB Error - unspecified *****\r\n" );
                RedoCommandPrompt();
                return TRUE;
                break;

        }
    #endif

    return FALSE;
}



//******************************************************************************
//******************************************************************************
// Internal Functions
//******************************************************************************
//******************************************************************************

#ifndef MINIMUM_BUILD


/****************************************************************************
  Function:
    void EraseCommandLine( void )

  Description:
    This function erases the current command line.  It works by sending a
    backspace-space-backspace combination for each character on the line.

  Precondition:
    commandInfo.index must be valid.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void EraseCommandLine( void )
{
    BYTE    i;
    BYTE    lineLength;

    lineLength = commandInfo.index;
    for (i=0; i<lineLength; i++)
    {
        PutChar( 0x08 );
        PutChar( ' ' );
        PutChar( 0x08 );
    }
    commandInfo.index = 0;
}


/****************************************************************************
  Function:
    BYTE GetCommand( void )

  Description:
    This function returns whether or not the user has finished entering a
    command.  If so, then the command entered by the user is determined and
    placed in commandInfo.command.  The command line index
    (commandInfo.index) is set to the first non-space character after the
    command.

  Precondition:
    commandInfo.reading must be valid.

  Parameters:
    None

  Return Values:
    TRUE    - The user has entered a command.  The command is in
                  commandInfo.command.
    FALSE   - The user has not finished entering a command.

  Remarks:
    None
  ***************************************************************************/

BYTE GetCommand( void )
{
    char    firstWord[MAX_COMMAND_LENGTH];

    if (commandInfo.reading)
    {
        return FALSE;
    }
    else
    {
        commandInfo.index = 0;

        commandInfo.index = 0;
        GetOneWord( firstWord );
        SkipWhiteSpace();

        if (firstWord[0] == 0)
        {
            commandInfo.command = COMMAND_NO_COMMAND;
            return TRUE;
        }

        if (!strncmp( firstWord, "APPEND", 6 ) && (strlen(firstWord) == 6))
        {
            commandInfo.command = COMMAND_APPEND;
            return TRUE;
        }
        if (!strncmp( firstWord, "BAUD", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_BAUD;
            return TRUE;
        }
        if (!strncmp( firstWord, "CD", 2 ) && (strlen(firstWord) == 2))
        {
            commandInfo.command = COMMAND_CD;
            return TRUE;
        }
        if (!strncmp( firstWord, "COPY", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_COPY;
            return TRUE;
        }
        if (!strncmp( firstWord, "DATE", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_DATE;
            return TRUE;
        }
        if (!strncmp( firstWord, "DEL", 3 ) && (strlen(firstWord) == 3))
        {
            commandInfo.command = COMMAND_DEL;
            return TRUE;
        }
        if ((!strncmp( firstWord, "DIR", 3 ) && (strlen(firstWord) == 3)) ||
            (!strncmp( firstWord, "LS", 2 ) && (strlen(firstWord) == 2)))
        {
            commandInfo.command = COMMAND_DIR;
            return TRUE;
        }
        if ((!strncmp( firstWord, "HELP", 4 )  && (strlen(firstWord) == 4)) ||
            (!strncmp( firstWord, "?", 1 ) && (strlen(firstWord) == 1)))
        {
            commandInfo.command = COMMAND_HELP;
            return TRUE;
        }
        if (!strncmp( firstWord, "LOG", 3 ) && (strlen(firstWord) == 3))
        {
            commandInfo.command = COMMAND_LOG;
            return TRUE;
        }
        if (!strncmp( firstWord, "MD", 2 ) && (strlen(firstWord) == 2))
        {
            commandInfo.command = COMMAND_MD;
            return TRUE;
        }
        if (!strncmp( firstWord, "MODE", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_MODE;
            return TRUE;
        }
        if (!strncmp( firstWord, "READ", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_READ;
            return TRUE;
        }
        if (!strncmp( firstWord, "RD", 2 ) && (strlen(firstWord) == 2))
        {
            commandInfo.command = COMMAND_RD;
            return TRUE;
        }
        if (!strncmp( firstWord, "REN", 3 ) && (strlen(firstWord) == 3))
        {
            commandInfo.command = COMMAND_REN;
            return TRUE;
        }
        #ifdef IMPLEMENT_RESET
        if (!strncmp( firstWord, "RESET", 5 ) && (strlen(firstWord) == 5))
        {
            commandInfo.command = COMMAND_RESET;
            return TRUE;
        }
        #endif
        #ifdef IMPLEMENT_SUSPEND_AND_RESUME
        if (!strncmp( firstWord, "RESUME", 6 ) && (strlen(firstWord) == 6))
        {
            commandInfo.command = COMMAND_RESUME;
            return TRUE;
        }
        if (!strncmp( firstWord, "SUSPEND", 7 ) && (strlen(firstWord) == 7))
        {
            commandInfo.command = COMMAND_SUSPEND;
            return TRUE;
        }
        #endif
        if (!strncmp( firstWord, "TIME", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_TIME;
            return TRUE;
        }
        if (!strncmp( firstWord, "TYPE", 4 ) && (strlen(firstWord) == 4))
        {
            commandInfo.command = COMMAND_TYPE;
            return TRUE;
        }
        if (!strncmp( firstWord, "WRITE", 5 ) && (strlen(firstWord) == 5))
        {
            commandInfo.command = COMMAND_WRITE;
            return TRUE;
        }
        #ifdef IMPLEMENT_WHO
        if (!strncmp( firstWord, "WHO", 3 ) && (strlen(firstWord) == 3))
        {
            commandInfo.command = COMMAND_WHO;
            return TRUE;
        }
        #endif

        commandInfo.command = COMMAND_UNKNOWN;
        return TRUE;
    }
}


/****************************************************************************
  Function:
    DWORD GetCurrentTick( void )

  Description:
    This function gets the current tick count in terms of milliseconds.
    Note that the timer is only accurate to MILLISECONDS_PER_TICK ms.

  Precondition:
    None

  Parameters:
    None

  Returns:
    DWORD - Number of milliseconds since the timer was turned on.

  Remarks:

  ***************************************************************************/
DWORD GetCurrentTick( void )
{
    return (currentTick * MILLISECONDS_PER_TICK);
}


/****************************************************************************
  Function:
    void GetOneWord( char *buffer )

  Description:
    This function copies the next word in the command line to the specified
    buffer.  Word deliniation is marked by a space character.  The returned
    word is null terminated.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid

  Parameters:
    *buffer - Pointer to where the word is to be stored.

  Returns:
    None

  Remarks:

  ***************************************************************************/
void GetOneWord( char *buffer )
{
    SkipWhiteSpace();

    while ((commandInfo.buffer[commandInfo.index] != 0) &&
           (commandInfo.buffer[commandInfo.index] != ' '))
    {
        *buffer++ = commandInfo.buffer[commandInfo.index++];
    }
    *buffer = 0;
}


/****************************************************************************
  Function:
    DWORD GetUserDate( void )

  Description:
    This function extracts a user entered date from the command line and
    places it in a DWORD that matches the format required for the RTCC.
    The required format is:
                        YYYY-MM-DD
    where YY is between 2000 and 2099.  If the date is not in a valid format,
    0 is returned.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid, and
    commandInfo.index points to the first character of the date

  Parameters:


  Returns:
    If the project is built for a PIC24F, this function returns a DWORD in
    the format <00><YY><MM><DD>.  If it is built for a PIC32MX, this function
    returns a DWORD in the format <YY><MM><DD><00>.

  Remarks:
    Range checks are not comprehensive.  The day is not qualified based on
    how many days are in the specified month.

    The values from the RTCC are assumed to be in BCD format.

    The two architectures have different formats for the date.  The index
    values are set above accordingly.
  ***************************************************************************/

DWORD GetUserDate( void )
{
    DWORD_VAL   date;

    date.Val = 0;

    // Get the year.
    if (commandInfo.buffer[commandInfo.index++] != '2')                     return 0;
    if (commandInfo.buffer[commandInfo.index++] != '0')                     return 0;

    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_YEAR] =  (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_YEAR] |= (commandInfo.buffer[commandInfo.index++] - '0');

    if (commandInfo.buffer[commandInfo.index++] != '-')                     return 0;

    // Get the month.
    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_MONTH] =  (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_MONTH] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (!((0x01 <= date.v[INDEX_MONTH]) && (date.v[INDEX_MONTH] <= 0x12)))  return 0;

    if (commandInfo.buffer[commandInfo.index++] != '-')                     return 0;

    // Get the day.
    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_DAY] =  (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] ))                    return 0;
    date.v[INDEX_DAY] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (!((0x01 <= date.v[INDEX_DAY]) && (date.v[INDEX_DAY] <= 0x31)))      return 0;

    return date.Val;
}


/****************************************************************************
  Function:
    DWORD GetUserTime( void )

  Description:
    This function extracts a user entered time from the command line and
    places it in a DWORD that matches the format required for the RTCC.  The
    required format is:
                        HH:MM:SS
    in 24-hour format.  If the time is not in a valid format, 0 is returned.

  Precondition:
    commandInfo.buffer and commandInfo.index are valid;
    commandInfo.index points to the first character of the time

  Parameters:
    None

  Return Values:
    If the project is built for a PIC24F, this function returns a DWORD in
    the format <00><HH><MM><SS>.  If it is built for a PIC32MX, this function
    returns a DWORD in the format <HH><MM><SS><00>.

  Remarks:
    The values from the RTCC are assumed to be in BCD format.

    The two architectures have different formats for the date. The index
    values are set above accordingly.
  ***************************************************************************/

DWORD GetUserTime( void )
{
    DWORD_VAL   time;

    time.Val = 0;

    // Get the hours.
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_HOURS] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_HOURS] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_HOURS] > 0x23)                      return 0;

    if (commandInfo.buffer[commandInfo.index++] != ':')  return 0;

    // Get the minutes.
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_MINUTES] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_MINUTES] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_MINUTES] > 0x59)                    return 0;

    if (commandInfo.buffer[commandInfo.index++] != ':')  return 0;

    // Get the seconds.
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_SECONDS] |= (commandInfo.buffer[commandInfo.index++] - '0') << 4;
    if (!IsNum( commandInfo.buffer[commandInfo.index] )) return 0;
    time.v[INDEX_SECONDS] |= (commandInfo.buffer[commandInfo.index++] - '0');
    if (time.v[INDEX_SECONDS] > 0x59)                    return 0;

    return time.Val;
}


/****************************************************************************
  Function:
     void InitializeClock( void )

  Description:
    This function initializes the tick timer.  It configures and starts the
    timer, and enables the timer interrupt.  We are using Timer 3 so we can
    also trigger an A/D conversion when the timer rolls over.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void InitializeClock( void )
{
    currentTick = 0;

    #if defined( __C30__ )
        IPC2bits.T3IP = TIMER_INTERRUPT_PRIORITY;
        IFS0bits.T3IF = 0;

        TMR3 = 0;
        PR3 = TIMER_PERIOD;
        T3CON = TIMER_ON | STOP_TIMER_IN_IDLE_MODE | TIMER_SOURCE_INTERNAL |
                GATED_TIME_DISABLED | TIMER_16BIT_MODE | TIMER_PRESCALER;

        IEC0bits.T3IE = 1;
    #elif defined( __PIC32MX__ )
        //OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_8, TIMER_PERIOD);
        OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, TIMER_PERIOD);

        // set up the timer interrupt with a priority of 2
        ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_2);
    #else
        #error Cannot initialize timer.
    #endif

    return;
}


/****************************************************************************
  Function:
    void InitializeCommand( void )

  Description:
    This function prints a command prompt and initializes the command line
    information.  If available, the command prompt format is:
                    [Volume label]:[Current directory]>

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void InitializeCommand( void )
{
    char            buffer[50];

    if (loggerStatus.mediaPresent)
    {
        buffer[0] = 0;
        if (volume.valid)
        {
            if (termMode)
            {
                PrintString( volume.label );
                PutChar( ':' );
            }
        }

        FSgetcwd( buffer, 50 );
        if (termMode)
        {
            PrintString( buffer );
        }
    }
    if (loggerStatus.overcurrentStateUSB)
    {
        if (termMode)
        {
            PrintString( "\r\n***** USB Error - overcurrent detected *****\r\n" );
        }
        else
        {
            PrintString("E:=USB Error - overcurrent detected\r\n");
        }
    }

    if (termMode)
    {
        PrintString( "> " );
    }
    else
    {
        PrintString( ">" );
    }

    commandInfo.command     = COMMAND_NO_COMMAND;
    commandInfo.index       = 0;
    commandInfo.reading     = TRUE;

    memset( commandInfo.buffer, 0x00, MAX_COMMAND_LENGTH );
}



/****************************************************************************
  Function:
     void InitializeAnalogMonitor( void )

  Description:
    This function initializes the monitoring of the three analog inputs.
    Since we need to monitor Vbus while doing other operations, we will
    simply scan all required input channels: the temperature sensor (AN4),
    the potentiometer (AN5), and Vbus (AN8).

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

/*void InitializeAnalogMonitor( void )
{
    // Set up the A/D converter
    #if defined( __C30__ )
        #if defined(__PIC24FJ256GB110__) || defined(__PIC24FJ64GB004__)
	        AD1PCFGL    &= ~SCAN_MASK; // Disable digital input on AN4, AN5, AN8
	    #elif defined(__PIC24FJ256GB210__) || defined(__PIC24FJ256DA210__)
	        ANSB        &= ~SCAN_MASK; // Disable digital input on AN4, AN5, AN8
	    #endif
	    AD1CSSL     = SCAN_MASK;    // Scan AN4, AN5, AN8
	    AD1CON3     = 0x1F05;       // 31 Tad auto-sample, Tad = 5*Tcy
	    AD1CON2     = 0x040C;       // Scan inputs, 3 conversions per interrupt, MUX A only
	    AD1CON1     = 0x8046;       // Turn on, start sampling, convert on Timer 3
    #elif defined( __PIC32MX__ )
	    AD1PCFG     &= ~SCAN_MASK;  // Disable digital input on AN4, AN5, AN8
	    AD1CSSL     = SCAN_MASK;    // Scan AN4, AN5, AN8
	    AD1CON3     = 0x1F05;       // 31 Tad auto-sample, Tad = 5*Tcy
	    AD1CON2     = 0x0408;       // Scan inputs, 3 conversions per interrupt, MUX A only
	    AD1CON1     = 0x8046;       // Turn on, start sampling, convert on Timer 3
    #endif

    // Enable A/D interrupts
    
    #if defined( __C30__ )
        IEC0bits.AD1IE = 1;
    #elif defined( __PIC32MX__ )
        IEC1SET = 0x00000002;
        ConfigIntADC10(ADC_INT_ON | ADC_INT_PRI_2);
    #else
        #error Cannot turn on ADC interrupt
    #endif

    return;
}*/


/****************************************************************************
  Function:
    void MonitorMedia( void )

  Description:
    This function calls the background tasks necessary to support USB Host
    operation.  Upon initial insertion of the media, it initializes the file
    system support and reads the volume label.  Upon removal of the media,
    the volume label is marked invalid.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void MonitorMedia( void )
{
    BYTE            mediaPresentNow;
    BYTE            mountTries;
    SearchRec       searchRecord;

    USBTasks();

    if (loggerStatus.overcurrentStateUSB)
    {
        // An overcurrent situation exists, and the USB has been shut down.
        if (loggerStatus.mediaPresent)
        {
            if (termMode)
            {
                PrintString( "\r\n***** USB Error - overcurrent detected *****\r\n" );
            }
            LED_MEDIA = 0;
            OUT_MEDIA = 0;
            loggerStatus.mediaPresent = FALSE;
        }
    }
    else
    {
        mediaPresentNow = USBHostMSDSCSIMediaDetect();
        if (!mediaPresentNow)
        {
            LED_MEDIA = 0;
            OUT_MEDIA = 0;
            loggerStatus.cannotInitialize = FALSE;
        }
        if ((mediaPresentNow != loggerStatus.mediaPresent) && !loggerStatus.cannotInitialize)
        {
            if (mediaPresentNow)
            {
                mountTries = 10;
                while(!FSInit() && --mountTries);
                if (!mountTries)
                {
                    PrintString( "\r\nAPP: Could not initialize media.  Media format must be \r\n" );
                    PrintString( "     FAT or FAT32, and sector size must be 0x" );
                    UART2PutHexWord( MEDIA_SECTOR_SIZE );
                    PrintString( ".\r\n" );
                    RedoCommandPrompt();
                    LED_MEDIA = 0;
                    OUT_MEDIA = 0;
                    loggerStatus.mediaPresent       = FALSE;
                    loggerStatus.cannotInitialize   = TRUE;
                }
                else
                {
                    int                     i;
                    USB_DEVICE_DESCRIPTOR   *pDescriptor;
                    
                    pDescriptor = (USB_DEVICE_DESCRIPTOR *)USBHostGetDeviceDescriptor( 1 );
                    i = 0;
                    while ((i<PROBLEM_THUMB_DRIVE_COUNT) && 
                           (pDescriptor->idVendor != problemThumbDrives[i].vid) && (pDescriptor->idProduct != problemThumbDrives[i].pid))
                    {
                        i++;
                    }
                    
                    if (i < PROBLEM_THUMB_DRIVE_COUNT)
                    {    
                        PrintString( "\r\nThis thumb drive is not supported. Please replace with a different model.\r\n" );
                        LED_MEDIA = 0;
                        OUT_MEDIA = 0;
                        loggerStatus.mediaPresent       = FALSE;
                        loggerStatus.cannotInitialize   = TRUE;
                    }
                    else
                    {
                        LED_MEDIA = 1;
                        OUT_MEDIA = 1;
                        loggerStatus.mediaPresent = TRUE;
    
                        // Find the volume label.  We need to do this here while we are at the
                        // root directory.
                        if (!FindFirst( "*.*", ATTR_VOLUME, &searchRecord ))
                        {
                            strcpy( volume.label, searchRecord.filename );
                            volume.valid = TRUE;
                        }
                    }
                                        
                    RedoCommandPrompt();
                }
            }
            else
            {
                LED_MEDIA = 0;
                OUT_MEDIA = 0;
                loggerStatus.mediaPresent   = FALSE;
                volume.valid                = FALSE;

                RedoCommandPrompt();
            }
        }
    }
}



/****************************************************************************
  Function:
    void MonitorUser( void )

  Description:
    This routine monitors command line input from the user.  The
    UART is polled rather than using interrupts.  If the previous
    command line is still being processed, the characters are
    ignored.  All text input is converted to upper case for
    simplification.  When the user presses the Enter or Return key,
    the command line is considered terminated.  Escape sequences are
    captured and checked.  If the up-arrow is pressed, the previous
    buffered command is displayed, until the oldest command is
    reached.  If the down-arrow is pressed, the next buffered
    command is displayed, until the newest command is displayed.
    If the down-arrow is pressed one additional time, the command
    line is erased.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    Currently, blank entries are added to the old command buffer.
  ***************************************************************************/

void MonitorUser( void )
{
    char    oneChar;

    if (U2STAbits.URXDA)
    {
        LED_COMMS = 1;
        oneChar = U2RXREG;  

        // If we are currently processing a command, throw the character away.
        if (commandInfo.reading)
        {
            if (commandInfo.escNeedSecondChar)
            {
                if (commandInfo.escFirstChar == 0x5B)
                {
                    if (oneChar == 0x41)        // Up arrow
                    {
                        if (commandBuffer.showing != commandBuffer.oldest)
                        {
                            if (commandBuffer.showing == MAX_BUFFERED_COMMANDS)
                            {
                                commandBuffer.showing = commandBuffer.newest;
                            }
                            else
                            {
                                commandBuffer.showing = (commandBuffer.showing - 1) & (MAX_BUFFERED_COMMANDS-1);
                            }
                        }
                        ReplaceCommandLine();
                    }
                    else if (oneChar == 0x42)   // Down arrow
                    {
                        if (commandBuffer.showing != MAX_BUFFERED_COMMANDS)
                        {
                            if (commandBuffer.showing != commandBuffer.newest)
                            {
                                commandBuffer.showing = (commandBuffer.showing + 1) & (MAX_BUFFERED_COMMANDS-1);
                                ReplaceCommandLine();
                            }
                            else
                            {
                                EraseCommandLine();
                                commandBuffer.showing = MAX_BUFFERED_COMMANDS;
                            }
                        }
                        else
                        {
                            EraseCommandLine();
                        }
                    }
                }
                commandInfo.escNeedSecondChar   = FALSE;
            }
            else if (commandInfo.escNeedFirstChar)
            {
                commandInfo.escFirstChar        = oneChar;
                commandInfo.escNeedFirstChar    = FALSE;
                commandInfo.escNeedSecondChar   = TRUE;
            }
            else
            {
                if (oneChar == 0x1B)    // ESC - an escape sequence
                {
                    commandInfo.escNeedFirstChar = TRUE;
                }
                else if (oneChar == 0x08 || oneChar == 0x7F)    // Backspace
                {
                    if (commandInfo.index > 0)
                    {
                        commandInfo.index --;
                        if (termMode)
                        {
                            PutChar( 0x08 );
                            PutChar( ' ' );
                            PutChar( 0x08 );
                        }
                    }
                }
                else if ((oneChar == 0x0D) || (oneChar == 0x0A))
                {
                    if (termMode)
                    {
                        PrintString( "\r\n" );
                    }
                    commandInfo.buffer[commandInfo.index]   = 0; // Null terminate the input command
                    commandInfo.reading                     = FALSE;
                    commandInfo.escNeedFirstChar            = FALSE;
                    commandInfo.escNeedSecondChar           = FALSE;

                    // Copy the new command into the command buffer
                    commandBuffer.showing = MAX_BUFFERED_COMMANDS;
                    if (commandBuffer.oldest == MAX_BUFFERED_COMMANDS)
                    {
                        commandBuffer.oldest = 0;
                        commandBuffer.newest = 0;
                    }
                    else
                    {
                        commandBuffer.newest = (commandBuffer.newest + 1) & (MAX_BUFFERED_COMMANDS-1);
                        if (commandBuffer.newest == commandBuffer.oldest)
                        {
                            commandBuffer.oldest = (commandBuffer.oldest + 1) & (MAX_BUFFERED_COMMANDS-1);
                        }
                    }
                    strcpy( &(commandBuffer.lines[commandBuffer.newest][0]), commandInfo.buffer );
                }
                else if ((0x20 <= oneChar) && (oneChar <= 0x7E))
                {
                    oneChar = UpperCase( oneChar ); // To make later processing simpler
                    if (commandInfo.index < MAX_COMMAND_LENGTH)
                    {
                        commandInfo.buffer[commandInfo.index++] = oneChar;
                    }
                    if (termMode)
                    {
                        PutChar( oneChar );    // Echo the character
                    }
                }
            }
        }
        LED_COMMS = 0;
    }
}


/****************************************************************************
  Function:
    void MonitorVBUS( void )

  Description:
    This routine monitors VBUS to check for overcurrent conditions.  The
    Explorer 16 with the PIC24FJ256GA110 PIM and USB PICtail Plus has an
    analog input dedicated to monitoring Vbus.  The voltage on Vbus is
    divided by two and fed into RB8/AN8.  The valid range for Vbus is
    4.4V - 5.25V.  If we are in an overcurrent condition, Vbus will be
    lower than 4.4V.  Full range on the A/D is 0x3FF for 3.3V.  So any
    value lower than 0xnnn on AN8 is overcurrent.  Otherwise, the current
    level is fine.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    TRUE    - An overcurrent situation exists. USB shut down.
    FALSE   - Normal USB operation.

  Remarks:
    Since Vbus monitoring is application-specific, it is the application's
    responsibility.

    If we get an overcurrent, we must shut down immediately to avoid brownout
    or blackout.   If we get the overcurrent while writing to a flash drive,
    the flash drive could be corrupted, because we cannot properly close the
    file.
  ***************************************************************************/

#define OVERCURRENT_RESET_VOLTAGE   4800ul        // In millivolts
#define OVERCURRENT_TRIP_VOLTAGE    4750ul        // In millivolts
#define SYSTEM_VOLTAGE              3300ul        // In millivolts
#define OVERCURRENT_RESET_READING   (1023ul * (OVERCURRENT_RESET_VOLTAGE/2) / SYSTEM_VOLTAGE)
#define OVERCURRENT_TRIP_READING    (1023ul * (OVERCURRENT_TRIP_VOLTAGE/2) / SYSTEM_VOLTAGE)

void MonitorVBUS( void )
{

    #if defined(__PIC32MX__)    // No VBus Overcurrent notification on Rev 1 of the PIC32 USB PIM.
        #warning  "VBus overcurrent notification not supported on PIC32 USB PIM Rev 1"
    #else

        UINT32 vBusValue = ADC_READING_VBUS;

        if (vBusValue < OVERCURRENT_TRIP_READING)
    {
        // We have an overcurrent condition.  Shut down the bus.
        if (loggerStatus.mediaPresent)
        {
            USBHostVbusEvent( EVENT_VBUS_OVERCURRENT, USB_ROOT_HUB, 0 );
        }
        if (!loggerStatus.overcurrentStateUSB)
        {
            if (termMode)
            {
                PrintString( "\r\n***** USB Error - overcurrent detected *****\r\n" );
            }
            RedoCommandPrompt();
            loggerStatus.overcurrentStateUSB    = TRUE;
        }
    }
    else if (ADC_READING_VBUS > OVERCURRENT_RESET_READING)
    {
        if (loggerStatus.overcurrentStateUSB)
        {
            USBHostVbusEvent( EVENT_VBUS_POWER_AVAILABLE, USB_ROOT_HUB, 0 );
            if (termMode)
            {
                PrintString( "\r\n***** USB overcurrent condition removed *****\r\n" );
            }
            RedoCommandPrompt();
        }
        loggerStatus.overcurrentStateUSB    = FALSE;
    }
    #endif  // defined(PIC32_USB_REV1_PIM)
}


/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetDate( void )

  Description:
    This routine reads the date from the RTCC module.

  Precondition:
    The RTCC module has been initialized.


  Parameters:
    None

  Returns:
    DWORD in the format <xx><YY><MM><DD>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the date in the
    PIC32MX format.
  ***************************************************************************/

#if defined( __C30__ )
DWORD   PIC24RTCCGetDate( void )
{
    DWORD_VAL   date1;
    DWORD_VAL   date2;

    do
    {
        while (RCFGCALbits.RTCSYNC);

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 1;
        date1.w[1] = RTCVAL;
        date1.w[0] = RTCVAL;

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 1;
        date2.w[1] = RTCVAL;
        date2.w[0] = RTCVAL;

    } while (date1.Val != date2.Val);

    return date1.Val;
}
#endif


/****************************************************************************
  Function:
    DWORD   PIC24RTCCGetTime( void )

  Description:
    This routine reads the time from the RTCC module.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    None

  Returns:
    DWORD in the format <xx><HH><MM><SS>

  Remarks:
    To catch roll-over, we do two reads.  If the readings match, we return
    that value.  If the two do not match, we read again until we get two
    matching values.

    For the PIC32MX, we use library routines, which return the time in the
    PIC32MX format.
  ***************************************************************************/

#if defined( __C30__ )
DWORD   PIC24RTCCGetTime( void )
{
    DWORD_VAL   time1;
    DWORD_VAL   time2;

    do
    {
        while (RCFGCALbits.RTCSYNC);

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 0;
        time1.w[1] = RTCVAL;
        time1.w[0] = RTCVAL;

        RCFGCALbits.RTCPTR0 = 1;
        RCFGCALbits.RTCPTR1 = 0;
        time2.w[1] = RTCVAL;
        time2.w[0] = RTCVAL;

    } while (time1.Val != time2.Val);

    return time1.Val;
}
#endif


/****************************************************************************
  Function:
    void PIC24RTCCSetDate( WORD xx_year, WORD month_day )

  Description:
    This routine sets the RTCC date to the specified value.


  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD xx_year    - BCD year in the lower byte
    WORD month_day  - BCD month in the upper byte, BCD day in the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

#if defined( __C30__ )
void PIC24RTCCSetDate( WORD xx_year, WORD month_day )
{
    UnlockRTCC();
    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 1;
    RTCVAL = xx_year;
    RTCVAL = month_day;
}
#endif


/****************************************************************************
  Function:
    void PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds )

  Description:
    This routine sets the RTCC time to the specified value.

  Precondition:
    The RTCC module has been initialized.

  Parameters:
    WORD weekDay_hours      - BCD weekday in the upper byte, BCD hours in the
                                lower byte
    WORD minutes_seconds    - BCD minutes in the upper byte, BCD seconds in
                                the lower byte

  Returns:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

#if defined( __C30__ )
void PIC24RTCCSetTime( WORD weekDay_hours, WORD minutes_seconds )
{
    UnlockRTCC();
    RCFGCALbits.RTCPTR0 = 1;
    RCFGCALbits.RTCPTR1 = 0;
    RTCVAL = weekDay_hours;
    RTCVAL = minutes_seconds;
}
#endif


/****************************************************************************
  Function:
    void PrintFileInformation( SearchRec searchRecord )

  Description:
    This function prints the file information that is contained in
    searchRecord.  Information printed is:
                <date> <time> [<DIR>] [<size>] <name>

  Precondition:
    None

  Parameters:
    SearchRec searchRecord  - File information

  Returns:
    None

  Remarks:
    The timestamp is assumed to be in the following format:
            Date format:    Bits 15-9:  Year (0 = 1980, 127 = 2107)
                            Bits 8-5:   Month (1 = January, 12 = December)
                            Bits 4-0:   Day (1 - 31)

            Time format:    Bits 15-11: Hours (0-23)
                            Bits 10-5:  Minutes (0-59)
                            Bits 4-0:   Seconds/2 (0-29)
  ***************************************************************************/

void PrintFileInformation( SearchRec searchRecord )
{
    char        buffer[20];

    // Display the file's date/time stamp.
    sprintf( buffer, "%04d-%02d-%02d ", ((((DWORD_VAL)(searchRecord.timestamp)).w[1] & 0xFE00) >> 9) + 1980,
                                         (((DWORD_VAL)(searchRecord.timestamp)).w[1] & 0x01E0) >> 5,
                                          ((DWORD_VAL)(searchRecord.timestamp)).w[1] & 0x001F );
    PrintString( buffer );
    sprintf( buffer, "%02d:%02d:%02d ",  (((DWORD_VAL)(searchRecord.timestamp)).w[0] & 0xF800) >> 11,
                                         (((DWORD_VAL)(searchRecord.timestamp)).w[0] & 0x07E0) >> 5,
                                         (((DWORD_VAL)(searchRecord.timestamp)).w[0] & 0x001F) << 1 );
    PrintString( buffer );

    // Display the file size.  If the file is actually a directory, display an indication.
    if (searchRecord.attributes & ATTR_DIRECTORY)
    {
        PrintString( "<DIR>           " );
    }
    else
    {
        sprintf( buffer, "     %10ld ", ((DWORD_VAL)(searchRecord.filesize)).Val );
        PrintString( buffer );
    }

    // Display the file name.
    PrintString( searchRecord.filename );
    PrintString( "\r\n" );
}


/****************************************************************************
  Function:
    void RedoCommandPrompt( void )

  Description:
    This function is called when the user either removes or inserts media.
    We want to let the user know right away that something has changed, so
    we change the command prompt immediately.  If the user is entering a
    command, we rebuild his command.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void RedoCommandPrompt( void )
{
    int i;

    PrintString( "\r\n" );
    if (volume.valid)
    {
        PrintString( volume.label );
        //UART2PutChar( ':' );
        PrintString( ":\\" );
    }
    else
    {
        PrintString( "<no label>:\\" );
    }
    PrintString( "> " );

    if (commandInfo.reading)
    {
        for (i = 0; i < commandInfo.index; i++)
        {
            PutChar( commandInfo.buffer[i] );
        }
    }
}


/****************************************************************************
  Function:
    void ReplaceCommandLine( void )

  Description:
    This function is called when the user presses the arrow keys to scroll
    through previous commands.  The function erases the current command line
    and replaces it with the previous command indicated by
    commandBuffer.showing.

  Precondition:
    The buffer of old commands is valid.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/

void ReplaceCommandLine( void )
{
    BYTE    i;
    BYTE    lineLength;
    char    oneChar;

    EraseCommandLine();

    lineLength = strlen( commandBuffer.lines[commandBuffer.showing] );
    for (i=0; i<lineLength; i++)
    {
        oneChar = commandBuffer.lines[commandBuffer.showing][i];
        PutChar( oneChar );
        commandInfo.buffer[commandInfo.index++] = oneChar;
    }
}


/****************************************************************************
  Function:
    void UnlockRTCC( void )

  Description:
    This function unlocks the RTCC so we can write a value to it.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    For the PIC32MX, we use library routines.
  ***************************************************************************/

#if !defined(TEST_PIC24)
    #define RTCC_INTERRUPT_REGISTER IEC5
    #define RTCC_INTERRUPT_VALUE    0x0040
#else
    #define RTCC_INTERRUPT_REGISTER IEC3
    #define RTCC_INTERRUPT_VALUE    0x2000
#endif

#if defined( __C30__ )
void UnlockRTCC( void )
{
    BOOL interruptsWereOn;

    interruptsWereOn = FALSE;
    if ((RTCC_INTERRUPT_REGISTER & RTCC_INTERRUPT_VALUE) == RTCC_INTERRUPT_VALUE)
    {
        interruptsWereOn = TRUE;
        RTCC_INTERRUPT_REGISTER &= ~RTCC_INTERRUPT_VALUE;
    }

    // Unlock the RTCC module
    __asm__ ("mov #NVMKEY,W0");
    __asm__ ("mov #0x55,W1");
    __asm__ ("mov #0xAA,W2");
    __asm__ ("mov W1,[W0]");
    __asm__ ("nop");
    __asm__ ("mov W2,[W0]");
    __asm__ ("bset RCFGCAL,#13");
    __asm__ ("nop");
    __asm__ ("nop");

    if (interruptsWereOn)
    {
        RTCC_INTERRUPT_REGISTER |= RTCC_INTERRUPT_VALUE;
    }
}
#endif



/****************************************************************************
  Function:
    void WriteOneBuffer( FSFILE *fptr, BYTE *data, WORD size )

  Description:
    This function writes one log buffer to the indicated file.  It then
    advances the pointer to the current buffer to write.

  Precondition:
    None

  Parameters:
    FSFILE *fptr    - Pointer to an open file
    BYTE *data      - Pointer to data to write
    WORD size       - How many bytes of data to write

  Return Values:
    None

  Remarks:
    None
  ***************************************************************************/

void WriteOneBuffer( FSFILE *fptr, BYTE *data, WORD size )
{
    if (FSfwrite( data, 1, size, fptr) != size)
    {
        PrintString( "! Error writing log file\r\n" );
    }
    logData[logBufferWriting].bufferFull    = FALSE;
    logData[logBufferWriting].index         = 0;

    logBufferWriting++;
    if (logBufferWriting >= NUM_LOG_BUFFERS)
    {
        logBufferWriting = 0;
    }
}

void InitI2C()
{
    UINT config1 = 0;
    UINT config2 = 0;

    CloseI2C2();    //Disbale I2C2 module if enabled previously
    ConfigIntI2C2(MI2C_INT_OFF);  //Disable I2C interrupt

    config1 = (I2C_ON | I2C_7BIT_ADD);
    config2 = 157;
    OpenI2C2(config1, config2);   //configure I2C2
}

void WriteByteEEP(BYTE address, BYTE data)
{
    int i;

    IdleI2C2();     // Wait for Idle
    StartI2C2();
    while(I2C2CONbits.SEN );  //Wait till Start sequence is completed
    for(i=0;i<1000;i++);
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag

    MasterWriteI2C2(0xA0);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    MasterWriteI2C2(address);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    MasterWriteI2C2(data);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    IdleI2C2();              //wait for the I2C to be Idle
    StopI2C2();              //Terminate communication protocol with stop signal
    while(I2C2CONbits.PEN);  //Wait till stop sequence is completed
    IdleI2C2();
    
    // Poll until EEPROM Becomes ready
    do
    {
        StartI2C2();
        while(I2C2CONbits.SEN );  //Wait till Start sequence is completed
        for(i=0;i<1000;i++);
        MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag

        MasterWriteI2C2(0xA0);       //Write Slave address and set master for transmission
        while(I2C2STATbits.TBF);     //Wait till address is transmitted
        while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
        MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    }while(I2C2STATbits.ACKSTAT);
}

BYTE ReadByteEEP(BYTE address)
{
    int i;
    BYTE returnVal;

    InitI2C();
    IdleI2C2();     // Wait for Idle
    StartI2C2();
    while(I2C2CONbits.SEN );  //Wait till Start sequence is completed
    for(i=0;i<1000;i++);
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag

    MasterWriteI2C2(0xA0);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    MasterWriteI2C2(address);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    IdleI2C2();     // Wait for Idle
    RestartI2C2();
    while(I2C2CONbits.RSEN );  //Wait till Start sequence is completed
    for(i=0;i<1000;i++);
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag

    MasterWriteI2C2(0xA1);       //Write Slave address and set master for transmission
    while(I2C2STATbits.TBF);     //Wait till address is transmitted
    while(!IFS3bits.MI2C2IF);    //Wait for ninth clock cycle
    MI2C2_Clear_Intr_Status_Bit; //Clear interrupt flag
    while(I2C2STATbits.ACKSTAT);

    returnVal = MasterReadI2C2();       //read

    IdleI2C2();              //wait for the I2C to be Idle
    StopI2C2();              //Terminate communication protocol with stop signal
    while(I2C2CONbits.PEN);  //Wait till stop sequence is completed
    IdleI2C2();

    return returnVal;
}


/****************************************************************************
  Function:
    void __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)

  Description:
    Timer ISR, used to update the tick count.  Since we are using Timer 3, we
    can also trigger an A/D conversion off of the timer.

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    None
  ***************************************************************************/

#if defined( __C30__ )
void __attribute__((__interrupt__, auto_psv)) _T3Interrupt( void )
#elif defined( __PIC32MX__ )
void __ISR(_TIMER_3_VECTOR, ipl2) _T3Interrupt(void)
#else
#error Cannot prototype timer interrupt
#endif
{
    #if defined( __C30__ )
    if (IFS0bits.T3IF)
    #elif defined( __PIC32MX__ )
    if (IFS0 & PIC32MX_TIMER3_INTERRUPT)
    #else
    #error Cannot check timer interrupt
    #endif
    {
        // Clear the interrupt flag
        #if defined( __C30__ )
            IFS0bits.T3IF   = 0;
        #elif defined( __PIC32MX__ )
            mT3ClearIntFlag();
        #else
            #error Cannot clear timer interrupt.
        #endif

        // Count the timer tick
        currentTick++;
    }
}

/****************************************************************************
  Function:
    void __attribute__((__interrupt__, auto_psv)) _ADCInterrupt(void)

  Description:
    ADC ISR, used to read the temperature

  Precondition:
    None

  Parameters:
    None

  Return Values:
    None

  Remarks:
    The ADC interrupt bits are in different registers in the two architectures.

    The conversion itself is triggered by Timer 3.
  ***************************************************************************/

#if defined( __C30__ )
void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt( void )
#elif defined( __PIC32MX__ )
#pragma interrupt _ADC1Interrupt ipl2 vector 27
void _ADC1Interrupt( void )
#else
#error Cannot prototype ADC interrupt
#endif
{
    #if defined( __C30__ )
    if (IFS0bits.AD1IF)
    #elif defined( __PIC32MX__ )
    if (IFS1bits.AD1IF)
    #else
    #error Cannot check ADC interrupt flag
    #endif
    {
        // Clear the interrupt flag
        #if defined( __C30__ )
            IFS0bits.AD1IF = 0;
        #elif defined( __PIC32MX__ )
            #if defined(__32MX795F512L__)
                //Must read all channels before clearing the interrupt
                //  These values will stick around until the next interrupt but
                //  before we can clear the interrupt we need to touch all of
                //  requested sample registers
                {
                    unsigned int bit_bucket;

                    bit_bucket = ADC1BUF0;
                    bit_bucket = ADC1BUF1;
                    bit_bucket = ADC1BUF2;
                }
            #endif

            IFS1CLR = 0x00000002;
        #else
            #error Cannot clear ADC interrupt
        #endif

        if (loggerStatus.readingTemperature)
        {
            // Store the temperature reading
            sprintf( (char *)&(logData[logBufferReading].buffer[logData[logBufferReading].index]),
                        "%010ld,%05d\r\n", GetCurrentTick(), ADC_READING_TEMPERATURE );
            logData[logBufferReading].index += 18;

            if ((logData[logBufferReading].index + 18) > MAX_LOG_BUFFER_SIZE)
            {
				// We don't check for buffers wrapping.  A truly robust application
				// should probably add something here to do that.  For demonstration
				// purposes, we are running fast enough that we don't see a problem.
                logData[logBufferReading++].bufferFull = TRUE;
                if (logBufferReading >= NUM_LOG_BUFFERS)
                {
                    logBufferReading = 0;
                }
            }
        }

        MonitorVBUS();
    }
}


    #if defined( __C30__ )
/*******************************************************************************
Function:       void __attribute__((__interrupt__, auto_psv)) _DefaultInterrupt(void)

This is just here to catch any interrupts that we see during debugging.

*******************************************************************************/

void __attribute__((interrupt, auto_psv)) _DefaultInterrupt(void)
{
  PrintString( "!!! Default interrupt handler !!!\r\n" );
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}

void __attribute__((interrupt, auto_psv)) _OscillatorFail(void)
{
  PrintString( "!!! Oscillator Fail interrupt handler !!!\r\n" );
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
void __attribute__((interrupt, auto_psv)) _AddressError(void)
{
  PrintString( "!!! Address Error interrupt handler !!!\r\n" );
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
void __attribute__((interrupt, auto_psv)) _StackError(void)
{
  PrintString( "!!! Stack Error interrupt handler !!!\r\n" );
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
void __attribute__((interrupt, auto_psv)) _MathError(void)
{
  PrintString( "!!! Math Error interrupt handler !!!\r\n" );
  while (1)
  {
      Nop();
      Nop();
      Nop();
  }
}
    #endif

#endif


