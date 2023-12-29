/**
 * \copyright Copyright (C) 2019-2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "hal.h"


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************

static void initGPIO(void);
static void initSPI(void);
static void initTIMER(void);
void        TIMER0IntHandler(void);



//****************************************************************************
//
// Function Definitions
//
//****************************************************************************
//*****************************************************************************
//
//! \brief  Initializes MCU peripherals for interfacing with the ADC.
//!
//! \fn void InitADC(void)
//!
//! \return None.
//
//*****************************************************************************
void initAdcPeripherals(void)
{
    // Initialize GPIOs pins used by ADC
    initGPIO();

    // Initialize SPI peripheral used by ADC
    initSPI();

    // Initialize timer peripheral used by ADC
    initTIMER();
}


//*****************************************************************************
//
//! \brief  Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn static void InitGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
static void initGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...

    // Enable the clock to the GPIO Port M and wait for it to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)));

    // Configure the GPIO for 'nCS' as output and set high
    MAP_GPIOPinTypeGPIOOutput(nCS_PORT, nCS_PIN);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);
}


//*****************************************************************************
//
//! \brief  Configures the MCU's SPI peripheral, for interfacing with the ADC.
//!
//! \fn static void InitSPI(void)
//!
//! \return None.
//
//*****************************************************************************
static void initSPI(void)
{
    /* --- INSERT YOUR CODE HERE ---
     * NOTE: The ADS7038 by default operates in SPI mode 0 (CPOL = 0, CPHA = 0).
     */

    // Enable the clock to SSI3 module
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
    {
    }

    // Enable clocks to GPIO Port Q
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }

    // Configure the SSI3 peripheral pins PQ0, PQ2 & PQ3. For nCS, the default FSS pin PQ1
    // is not used, instead PM7 is used.
    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

    // Increasing the drive strength to support higher-frequencies on SCLK
    MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    // Configure: SPI MODE 1, 5 MHz SCLK, 8-bits per frame
    MAP_SSIConfigSetExpClk(SSI_BASE_ADDR, SYSTEM_CLOCK_HZ, SSI_FRF_MOTO_MODE_0,   \
                           SSI_MODE_MASTER, 5000000, 8);

    // Enable the SSI3 module
    MAP_SSIEnable(SSI_BASE_ADDR);

    // Remove residual data from the SSI3 FIFO (if any).
    // NOTE: The SSIDataGetNonBlocking() function returns "true" when data
    // is returned, and "false" when no there is no more data to read.
    uint32_t junk;
    while (MAP_SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));
}


//*****************************************************************************
//
//! Enables the MCU's TIMER to control ADC conversions.
//!
//! \fn static void initTIMER(void)
//!
//! \return None.
//
//*****************************************************************************
static void initTIMER(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    // Enable timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    // Enable interrupts
    IntMasterEnable();
    IntRegister(INT_TIMER0A, TIMER0IntHandler);
    IntEnable(INT_TIMER0A);
}


//*****************************************************************************
//
//! \brief  Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArray(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param dataTx[] byte array of SPI data to send on MOSI.
//!
//! \param dataRx[] byte array of SPI data captured on MISO.
//!
//! \param byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArray(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     */

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    setCS(LOW);

    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    int i;
    for (i = 0; i < byteLength; i++)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    // Set the nCS pin HIGH
    setCS(HIGH);
}


//*****************************************************************************
//
//! \brief  Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArray(). If it is called
//! directly, then the /CS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    /*  --- INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    //
    // Remove any residual or old data from the receive FIFO
    //
    uint32_t junk;
    while (SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));

    //
    // SSI TX & RX
    //
    uint8_t dataRx;
    MAP_SSIDataPut(SSI_BASE_ADDR, (uint32_t) dataTx);
    MAP_SSIDataGet(SSI_BASE_ADDR, (uint32_t *) &dataRx);

    return dataRx;
}


//*****************************************************************************
//
//! Initializes timer to interrupt at specified frequency
//!
//! \fn void startTimer(uint32_t timerFreqHz)
//! \param timerFreqHz interrupt frequency in units of Hz (or SPS)
//!
//! \return None.
//
//*****************************************************************************
void startTimer(uint32_t timerFreqHz)
{
    // Configure full-width periodic timer and count time
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SYSTEM_CLOCK_HZ / timerFreqHz);

    // Enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timer
    TimerEnable(TIMER0_BASE, TIMER_A);
}


//*****************************************************************************
//
//! Disables the timer
//!
//! \fn void stopTimer(void)
//!
//! \return None.
//
//*****************************************************************************
void stopTimer(void)
{
    // Disable and clear timer interrupts
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    TimerDisable(TIMER0_BASE, TIMER_A);
}


//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void TIMER0IntHandler(void)
{
    // Clear the timer interrupt
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Array to store ADC conversion results
    uint8_t data[4] = {0};

    // Start conversion
    setCS(HIGH);

    // Wait for conversion to complete
    // IMPORTANT: This delay will need to be modified if averaging is enabled!
    delay_us(3);

    // Read data
    readData(data);
}


//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief  Reads that current state of the /CS GPIO pin.
//!
//! \fn bool getCS(void)
//!
//! \return boolean ('true' if /CS is high, 'false' if /CS is low).
//
//*****************************************************************************
bool getCS(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    return (bool) GPIOPinRead(nCS_PORT, nCS_PIN);
}


//*****************************************************************************
//
//! \brief  Controls the state of the /CS GPIO pin.
//!
//! \fn void setCS(const bool state)
//!
//! \param state boolean indicating which state to set the /CS pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setCS(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    // t(QUIET) delay
    if (state) { SysCtlDelay(2); }

    uint8_t value = (uint8_t) (state ? nCS_PIN : 0);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, value);

    // delay
    if (!state) { SysCtlDelay(2); }
}


//****************************************************************************
//
// Timing functions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief  Provides a timing delay with 'ms' resolution.
//!
//! \fn void delay_ms(const uint32_t delay_time_ms)
//!
//! \param delay_time_ms is the number of milliseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    MAP_SysCtlDelay( delay_time_ms * SYSTEM_CLOCK_HZ / (cycles_per_loop * 1000u) );
}


//*****************************************************************************
//
//! \brief  Provides a timing delay with 'us' resolution.
//!
//! \fn void delay_us(const uint32_t delay_time_us)
//!
//! \param delay_time_us is the number of microseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_us(const uint32_t delay_time_us)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    MAP_SysCtlDelay( delay_time_us * SYSTEM_CLOCK_HZ / (cycles_per_loop * 1000u) );
}
