#include "F28x_Project.h"     // Device headerfile
#include "driverlib.h"         // TI provided driver library
#include "F2837xD_I2C.h"
#include <stdio.h>             // Standard I/O library for printf

#define SLAVE_ADDR 0x18        // BQ76940EVM I2C Address

#define SYS_CTRL1 0x04         // System Control 1 Register Address
#define SYS_STAT 0x00          // System Status Register Address

// Function prototypes
void initI2CA(void);
int confirmI2CConnection(void);
void I2C_WriteRegister(Uint16 regAddr, Uint16 data);
Uint16 I2C_ReadRegister(Uint16 regAddr);
void delay(Uint16 count);

void main(void) {
    InitSysCtrl();
    InitGpio();

    // Configure GPIO pins for I2C A (GPIO32 - SDA, GPIO33 - SCL)
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(32, GPIO_INPUT, GPIO_OPENDRAIN);
    GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(33, GPIO_INPUT, GPIO_OPENDRAIN);

    initI2CA();

    // Infinite loop to confirm the I2C connection and verify the SYS_STAT register
    while (1) {
        if (confirmI2CConnection()) {
            Uint16 sysStat = I2C_ReadRegister(SYS_STAT);
            if (sysStat == 0x00) {  // Expected default value
                ESTOP0;  // Stop here if connection and read succeeded
            }
        }
    }
}

void initI2CA(void) {
    I2caRegs.I2CMDR.all = 0;           // Reset I2C module
    I2caRegs.I2CPSC.all = 9;           // I2C Clock Prescaler
    I2caRegs.I2CCLKL = 10;             // Clock low-time divider
    I2caRegs.I2CCLKH = 5;              // Clock high-time divider
    I2caRegs.I2CSAR.all = SLAVE_ADDR;  // Set Slave Address
    I2caRegs.I2CMDR.all = 0x0020;      // Take I2C out of reset, master mode
    I2caRegs.I2CFFTX.all = 0x6000;     // Enable FIFO mode
    I2caRegs.I2CFFRX.all = 0x2040;     // Enable RX FIFO, clear RX FIFO
}

int confirmI2CConnection(void) {
    I2caRegs.I2CSAR.all = SLAVE_ADDR;  // Set slave address
    I2caRegs.I2CMDR.all = 0x6620;      // Send START and address

    delay(20000);                      // Small delay to give device time to respond

    if (I2caRegs.I2CSTR.bit.NACK == 1) { // Check if NACK received
        I2caRegs.I2CSTR.bit.NACK = 1;    // Clear NACK bit
        return 0;                        // No acknowledgment received
    }

    I2caRegs.I2CMDR.all = 0x6C20;        // Send STOP condition
    return 1;                            // Acknowledgment received
}

Uint16 I2C_ReadRegister(Uint16 regAddr) {
    // Step 1: Write the register address to the device
    I2caRegs.I2CSAR.all = SLAVE_ADDR;
    I2caRegs.I2CDXR.all = regAddr;
    I2caRegs.I2CMDR.all = 0x6620;  // Master-Transmitter, send START condition

    delay(20000);  // Wait for transmission to complete

    // Check for any NACK
    if (I2caRegs.I2CSTR.bit.NACK == 1) {
        I2caRegs.I2CSTR.bit.NACK = 1;
        return 0xFFFF;  // Indicate error in reading
    }

    // Step 2: Send repeated START to switch to receive mode
    I2caRegs.I2CMDR.all = 0x6C20;  // Master-Receiver, send START, prepare to read

    delay(20000);  // Wait for data to be received

    Uint16 data = I2caRegs.I2CDRR.all;  // Read received data
    I2caRegs.I2CMDR.all = 0x6C20;       // Send STOP condition

    return data;
}

void delay(Uint16 count) {
    while (count--);
}
