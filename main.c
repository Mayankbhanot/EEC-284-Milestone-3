// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

#include "hw_uart.h"
#include "udma.h"
#include "gpio.h"
#include "hw_apps_rcm.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"
#include "udma_if.h"
#include "gpio_if.h"
#include "timer_if.h"


#define APPLICATION_VERSION     "1.1.1"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************
#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     2

#define MASTER_MSG       "This is CC3200 SPI Master Application\n\r"
#define SLAVE_MSG        "This is CC3200 SPI Slave Application\n\r"

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char g_ucTxBuff[TR_BUFF_SIZE];
static unsigned char g_ucRxBuff[TR_BUFF_SIZE];
static unsigned char ucTxBuffNdx;
static unsigned char ucRxBuffNdx;

volatile static tBoolean bRxDone;
volatile static tBoolean bTimerDone;
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulRefBase;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;
volatile float delay_up = 1000/48;
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

#define q   6       /* for 2^3 points */
#define N   (1<<q)      /* N-point FFT, iFFT */

typedef float real;
typedef struct{real Re; real Im;} complex;

#ifndef PI
# define PI 3.14159265358979323846264338327950288
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//*****************************************************************************
//
//! SPI Slave Interrupt handler
//!
//! This function is invoked when SPI slave has its receive register full or
//! transmit register empty.
//!
//! \return None.
//
//*****************************************************************************
static void SlaveIntHandler()
{
    unsigned long ulRecvData;
    unsigned long ulStatus;

    ulStatus = MAP_SPIIntStatus(GSPI_BASE,true);

    MAP_SPIIntClear(GSPI_BASE,SPI_INT_RX_FULL|SPI_INT_TX_EMPTY);

    if(ulStatus & SPI_INT_TX_EMPTY)
    {
        MAP_SPIDataPut(GSPI_BASE,g_ucTxBuff[ucTxBuffNdx%TR_BUFF_SIZE]);
        ucTxBuffNdx++;
    }

    if(ulStatus & SPI_INT_RX_FULL)
    {
        MAP_SPIDataGetNonBlocking(GSPI_BASE,&ulRecvData);
        g_ucTxBuff[ucRxBuffNdx%TR_BUFF_SIZE] = ulRecvData;
        Report("%c",ulRecvData);
        ucRxBuffNdx++;
    }
}

//*****************************************************************************
//
//! The interrupt handler for the timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int n=0;
void
TimerBaseIntHandler(void)
{
    // Clear the timer interrupt.
    Timer_IF_InterruptClear(g_ulBase);
    // Raise timer flag
    bTimerDone = true;
}

//*****************************************************************************
//
//! The interrupt handler for the second timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void
TimerRefIntHandler(void)
{
    if(n==0)
        {
            GPIOPinWrite(GPIOA1_BASE, 0x1,0x1);     // pin 63 yellow
            GPIOPinWrite(GPIOA2_BASE, 0x40,0);      // pin 15 orange
            GPIOPinWrite(GPIOA3_BASE, 0x80,0x80);   // pin 45 brown
            GPIOPinWrite(GPIOA3_BASE, 0x10,0);      // pin 18 black
        }
        else if(n==1)
        {
            GPIOPinWrite(GPIOA1_BASE, 0x1,0x1);     // pin 63 yellow
                        GPIOPinWrite(GPIOA2_BASE, 0x40,0);      // pin 15 orange
                        GPIOPinWrite(GPIOA3_BASE, 0x80,0);      // pin 45 brown
                        GPIOPinWrite(GPIOA3_BASE, 0x10,0x10);
        }
        else if(n==2)
        {
            GPIOPinWrite(GPIOA1_BASE, 0x1,0);       // pin 63 yellow
                        GPIOPinWrite(GPIOA2_BASE, 0x40,0x40);   // pin 15 orange
                        GPIOPinWrite(GPIOA3_BASE, 0x80,0);      // pin 45 brown
                        GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);  // pin 18 black
        }
        else if(n==3)
        {
            GPIOPinWrite(GPIOA1_BASE, 0x1,0);       // pin 63 yellow
                        GPIOPinWrite(GPIOA2_BASE, 0x40,0x40);   // pin 15 orange
                        GPIOPinWrite(GPIOA3_BASE, 0x80,0x80);   // pin 45 brown
                        GPIOPinWrite(GPIOA3_BASE, 0x10,0);      // pin 18 black
        }
        if(n==4) n=0;
        else n++;
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulRefBase);
    Timer_IF_Start(g_ulRefBase, TIMER_A, delay_up);
    //g_ulRefTimerInts ++;
    //GPIO_IF_LedToggle(MCU_RED_LED_GPIO);
}
//*****************************************************************************
//
//! delay_function is used to generate the delay between steps of the
//! stepper motor
//!
//! \param  float freq - Frequency value selected in the range [1,10]
//!
//! \return None
//
//*****************************************************************************

void delay_function(float freq)
{
    float delay;
    //Calculate delay in ms for each step
    delay = 1000/(freq);
    // Turn on the timers feeding values in mSec
    Timer_IF_Start(g_ulBase, TIMER_A, delay);
    //Wait for timer flag
    while(!bTimerDone);
    //Reset timer flag
    bTimerDone = false;
}

void fft( complex *v, int n, complex *tmp )
{
  if(n>1) {         /* otherwise, do nothing and return */
    int k,m;    complex z, w, *vo, *ve;
    ve = tmp; vo = tmp+n/2;
    for(k=0; k<n/2; k++) {
      ve[k] = v[2*k];
      vo[k] = v[2*k+1];
    }
    fft( ve, n/2, v );      /* FFT on even-indexed elements of v[] */
    fft( vo, n/2, v );      /* FFT on odd-indexed elements of v[] */
    for(m=0; m<n/2; m++) {
      w.Re = cos(2*PI*m/(double)n);
      w.Im = -sin(2*PI*m/(double)n);
      z.Re = w.Re*vo[m].Re - w.Im*vo[m].Im; /* Re(w*vo[m]) */
      z.Im = w.Re*vo[m].Im + w.Im*vo[m].Re; /* Im(w*vo[m]) */
      v[  m  ].Re = ve[m].Re + z.Re;
      v[  m  ].Im = ve[m].Im + z.Im;
      v[m+n/2].Re = ve[m].Re - z.Re;
      v[m+n/2].Im = ve[m].Im - z.Im;
    }
  }
  return;
}
///

//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************
void MasterMain()
{
    complex v[N], scratch[N];
    int k;
    long adc_value;
    long data[1024];
    int i;

    //
    // Set Tx buffer index
    //
    ucTxBuffNdx = 0;
    ucRxBuffNdx = 0;

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    while(1)
    {
        for(i=0;i<N;i++)
        {
            //
            // Send the string to slave. Chip Select(CS) needs to be
            // asserted at start of transfer and deasserted at the end.
            //
            MAP_SPITransfer(GSPI_BASE,0,g_ucRxBuff,50,
                    SPI_CS_ENABLE|SPI_CS_DISABLE);

            adc_value = ((int)(g_ucRxBuff[0])<<8 | g_ucRxBuff[1] );
            adc_value >>=3;
            adc_value = adc_value & 0x3FF;

            data[i]=adc_value;
            delay_function(30);
        }

        for(k=0; k<N; k++) {
            v[k].Re = data[k];
            v[k].Im = 0;
          }
          
        fft( v, N, scratch );
        
        // Detect First Heart Rate
        float max_val;
        max_val=v[1].Re;
        int max_i=1;
        for(i=6;i<22;i++)
        {
            if(v[i].Re>max_val)
                {
                    max_val=v[i].Re;
                    max_i=i+1;
                }
        }
		
		// Detect Second Heart Rate
        float max_val1;
        max_val1=v[1].Re;
        int max_i1=1;
        for(i=6;i<22;i++)
        {
            if(v[i].Re>max_val1 && max_i1!=max_i)
                {
                    max_val1=v[i].Re;
                    max_i1=i+1;
                }
        }

        float freq,freq1;
        freq = (max_i)*0.09375;
        freq1 = (max_i1)*0.09375;
        
        printf("\nHeart Rate 1: %0.2f BPM, Heart Rate 2: %0.2f BPM\n",freq*60, freq1*60);
        
    }

}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    bTimerDone = false;
    g_ulBase = TIMERA0_BASE;
    //
    // Base address for second timer
    //
    g_ulRefBase = TIMERA1_BASE;
    // Configuring the timer to one shot count down mode
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_Init(PRCM_TIMERA1, g_ulRefBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    // Setup the interrupts for the timer timeout.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);
    Timer_IF_IntSetup(g_ulRefBase, TIMER_A, TimerRefIntHandler);

    Timer_IF_Start(g_ulRefBase, TIMER_A, delay_up);

    GPIOPinWrite(GPIOA1_BASE, 0x1,0);           // pin 63 yellow
    GPIOPinWrite(GPIOA2_BASE, 0x40,0);          // pin 15 orange
    GPIOPinWrite(GPIOA3_BASE, 0x80,0);          // pin 45 brown
    GPIOPinWrite(GPIOA3_BASE, 0x10,0);          // pin 18 black

#if MASTER_MODE

    MasterMain();

#else

    SlaveMain();

#endif

    while(1)
    {

    }

}

