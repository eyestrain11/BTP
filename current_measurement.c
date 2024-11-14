#include "F28x_Project.h"     // Device headerfile
#include "driverlib.h"         // TI provided driver library
#include "F2837xD_I2C.h"
#include <stdio.h>             // Standard I/O library for printf

#define BQ76940_I2C_ADDRESS 0x18       // BQ76940EVM I2C Address

#define CURRENT_MEAS_START_ADDR 0x0C   // Register address for current measurement in BQ76940

// Function prototypes
int16_t readBQ76940Current(void);

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

    initBQ76940();  // Initialize BQ76940

    while(1) {
        int16_t cellCurrent = readBQ76940Current();  // Read the current measurement
        printf("Cell Current: %d mA\n", cellCurrent);  // Display current (adjust scaling as needed)
        
        // Add a delay to avoid continuous polling
        delay2(30000);
    }
}

int16_t readBQ76940Current(void) {
    Uint16 msb, lsb;
    int16_t raw_current;

    // Set I2C target address to BQ76940
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;

    // Send the register address for the MSB of the current measurement
    I2caRegs.I2CDXR.all = CURRENT_MEAS_START_ADDR;
    I2caRegs.I2CMDR.all = 0x6620; // Initiates a write to specify the register address

    // Wait for the operation to complete
    delay(20000);

    // Trigger a read for the MSB
    I2caRegs.I2CMDR.all = 0x6C20; // Initiates a read to get the MSB
    delay(20000);

    // Read the MSB of the current measurement
    msb = I2caRegs.I2CDRR.all;

    // Send the register address for the LSB of the current measurement
    I2caRegs.I2CDXR.all = CURRENT_MEAS_START_ADDR + 1;
    I2caRegs.I2CMDR.all = 0x6620; // Initiates a write to specify the next register address
    delay(20000);

    // Trigger a read for the LSB
    I2caRegs.I2CMDR.all = 0x6C20; // Initiates a read to get the LSB
    delay(20000);

    // Read the LSB of the current measurement
    lsb = I2caRegs.I2CDRR.all;

    // Combine MSB and LSB to form a signed 16-bit integer (BQ76940 outputs a signed value)
    raw_current = (int16_t)((msb << 8) | lsb);

    // Convert the raw ADC value to current in mA based on the 1 mΩ sense resistor
    float current_mA = raw_current * 8.44; // Each LSB represents 8.44 mA with a 1 mΩ resistor

    return (int16_t)current_mA;
}