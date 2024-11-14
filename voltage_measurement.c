#include "F28x_Project.h"     // Device headerfile
#include "driverlib.h"         // TI provided driver library
#include "F2837xD_I2C.h"
#include <stdio.h>             // Standard I/O library for printf

#define BQ76940_I2C_ADDRESS 0x18        // BQ76940EVM I2C Address

#define CELL_VOLTAGE_START_ADDR 0x04    // Starting address for cell voltages in BQ76940
#define BQ76940_SYS_CTRL1 0x00          // System Control 1 register
#define BQ76940_SYS_CTRL2 0x01          // System Control 2 register
#define BQ76940_PROTECT1 0x06           // Protection 1 register (OV)
#define BQ76940_PROTECT2 0x07           // Protection 2 register (UV)
#define BQ76940_CELLBAL1 0x01           // Cell Balancing register

// Function prototypes
void initI2CA(void);
void initBQ76940(void);
int confirmI2CConnection(void);
Uint16 readBQ76940CellVoltage(Uint16 cellIndex);
void I2C_writeRegister(Uint16 regAddr, Uint16 data);
void I2C_waitForStopCondition(void);
Uint16 I2C_ReadRegister(Uint16 regAddr);
void delay(Uint16 count);
void delay2(Uint16 count);

void main(void) {
    InitSysCtrl();
    InitGpio();

    // Configure GPIO pins for I2C A (GPIO32 - SDA, GPIO33 - SCL)
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(32, GPIO_INPUT, GPIO_OPENDRAIN);
    GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(33, GPIO_INPUT, GPIO_OPENDRAIN);

    initI2CA();

    while (1) {
        if (confirmI2CConnection()) {
            Uint16 sysStat = I2C_ReadRegister(BQ76940_SYS_CTRL1);
            if (sysStat == 0x00) {  // Expected default value
                break;
            }
        }
    }

    Uint16 cellVoltage;
    initBQ76940();  // Set up BQ76940 for cell voltage reading

    while(1) {
        cellVoltage = readBQ76940CellVoltage(0);  // Read the voltage for cell 0
        // Add a delay to avoid continuous polling
        delay2(30000);
    }
}

void initI2CA(void) {
    I2caRegs.I2CMDR.all = 0;           // Reset I2C module
    I2caRegs.I2CPSC.all = 9;           // I2C Clock Prescaler
    I2caRegs.I2CCLKL = 10;             // Clock low-time divider
    I2caRegs.I2CCLKH = 5;              // Clock high-time divider
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;  // Set Slave Address
    I2caRegs.I2CMDR.all = 0x0020;      // Take I2C out of reset, master mode
    I2caRegs.I2CFFTX.all = 0x6000;     // Enable FIFO mode
    I2caRegs.I2CFFRX.all = 0x2040;     // Enable RX FIFO, clear RX FIFO
}

void initBQ76940(void) {
    I2C_writeRegister(BQ76940_SYS_CTRL1, 0x01);  // Enable ADC
    I2C_writeRegister(BQ76940_SYS_CTRL2, 0x1D);  // Enable voltage sensing, temperature sensing, etc.
    I2C_writeRegister(BQ76940_PROTECT1, 0xC0);   // Enable OV protection
    I2C_writeRegister(BQ76940_PROTECT2, 0xC0);   // Enable UV protection
    I2C_writeRegister(BQ76940_CELLBAL1, 0x00);    // Disable cell balancing for now
}

int confirmI2CConnection(void) {
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CMDR.all = 0x6620;      // Send START and address

    delay(20000);

    if (I2caRegs.I2CSTR.bit.NACK == 1) {
        I2caRegs.I2CSTR.bit.NACK = 1;  // Clear NACK
        return 0;                      // No acknowledgment received
    }

    I2caRegs.I2CMDR.all = 0x6C20;      // Send STOP condition
    return 1;                          // Acknowledgment received
}

Uint16 I2C_ReadRegister(Uint16 regAddr) {
    // Write the register address
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CDXR.all = regAddr;
    I2caRegs.I2CMDR.all = 0x6620;

    delay(20000);

    if (I2caRegs.I2CSTR.bit.NACK == 1) {
        I2caRegs.I2CSTR.bit.NACK = 1;
        return 0xFFFF;
    }

    // Read the register data
    I2caRegs.I2CMDR.all = 0x6C20;

    delay(20000);

    Uint16 data = I2caRegs.I2CDRR.all;
    I2caRegs.I2CMDR.all = 0x6C20;

    return data;
}

Uint16 readBQ76940CellVoltage(Uint16 cellIndex) {
    Uint16 msb, lsb, voltage;
    Uint16 registerAddress = CELL_VOLTAGE_START_ADDR + (cellIndex * 2);

    // Read MSB
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CDXR.all = regcisterAddress;
    I2caRegs.I2CMDR.all = 0x6620;

    delay(20000);

    msb = I2caRegs.I2CDRR.all;

    // Read LSB
    I2caRegs.I2CDXR.all = registerAddress + 1;
    I2caRegs.I2CMDR.all = 0x6C20;

    delay2(20000);

    lsb = I2caRegs.I2CDRR.all;

    voltage = (msb << 8) | lsb;  // Combine MSB and LSB
    return voltage;
}

void I2C_writeRegister(Uint16 registerAddress, Uint16 data) {
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CDXR.all = registerAddress;
    I2caRegs.I2CMDR.all = 0x6620;

    delay2(20000);

    I2caRegs.I2CDXR.all = data;
    I2caRegs.I2CMDR.all = 0x6620;

    delay2(20000);
    I2C_waitForStopCondition();
}

void I2C_waitForStopCondition(void) {
    while (I2caRegs.I2CSTR.bit.SCD == 0);
}

void delay(Uint16 count) {
    while (count--);
}

void delay2(Uint16 count) {
    while(count--);
}
