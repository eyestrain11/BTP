#include "F28x_Project.h"     // Device headerfile
#include "driverlib.h"         // TI provided driver library
#include "F2837xD_I2C.h"
#include <stdio.h>             // Standard I/O library for printf

#define BQ76940_I2C_ADDRESS 0x18
#define NUM_CELLS 10                       // Example: 10 cells in the battery pack
#define BALANCING_THRESHOLD 50             // Balance if difference exceeds 50 mV
#define BALANCE_STOP_THRESHOLD 20          // Stop if difference below 20 mV
#define CELL_VOLTAGE_START_ADDR 0x04       // Starting address for cell voltages in BQ76940
#define BQ76940_CELLBAL1 0x01              // Cell Balancing register

// Function prototypes
Uint16 readBQ76940CellVoltage(Uint16 cellIndex);
void balanceCells(Uint16 *cellVoltages);
void I2C_writeRegister(Uint16 regAddr, Uint16 data);

void main(void) {
    InitSysCtrl();
    InitGpio();

    // Configure GPIO pins for I2C A (GPIO32 - SDA, GPIO33 - SCL)
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(32, GPIO_INPUT, GPIO_OPENDRAIN);
    GPIO_SetupPinMux(33, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(33, GPIO_INPUT, GPIO_OPENDRAIN);

    initI2CA();

    Uint16 cellVoltages[NUM_CELLS];
    initBQ76940();  // Set up BQ76940 for cell voltage reading

    while(1) {
        // Read all cell voltages
        for (Uint16 i = 0; i < NUM_CELLS; i++) {
            cellVoltages[i] = readBQ76940CellVoltage(i);
        }

        // Perform cell balancing
        balanceCells(cellVoltages);

        // Add delay to avoid excessive polling
        delay2(50000);
    }
}

// Function to read cell voltage
Uint16 readBQ76940CellVoltage(Uint16 cellIndex) {
    Uint16 msb, lsb, voltage;
    Uint16 registerAddress = CELL_VOLTAGE_START_ADDR + (cellIndex * 2);

    // Read MSB
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CDXR.all = registerAddress;
    I2caRegs.I2CMDR.all = 0x6620;
    delay(20000);
    msb = I2caRegs.I2CDRR.all;

    // Read LSB
    I2caRegs.I2CDXR.all = registerAddress + 1;
    I2caRegs.I2CMDR.all = 0x6C20;
    delay2(20000);
    lsb = I2caRegs.I2CDRR.all;

    voltage = (msb << 8) | lsb;
    return voltage;
}

// Cell balancing function
void balanceCells(Uint16 *cellVoltages) {
    Uint16 minVoltage = cellVoltages[0];
    Uint16 maxVoltage = cellVoltages[0];
    Uint16 balanceFlags = 0;

    // Find minimum and maximum cell voltages
    for (Uint16 i = 1; i < NUM_CELLS; i++) {
        if (cellVoltages[i] < minVoltage) minVoltage = cellVoltages[i];
        if (cellVoltages[i] > maxVoltage) maxVoltage = cellVoltages[i];
    }

    // Calculate voltage difference
    Uint16 voltageDifference = maxVoltage - minVoltage;

    // If the difference exceeds the threshold, initiate balancing
    if (voltageDifference > BALANCING_THRESHOLD) {
        for (Uint16 i = 0; i < NUM_CELLS; i++) {
            // Balance cells with voltage significantly above minimum voltage
            if (cellVoltages[i] > minVoltage + BALANCING_THRESHOLD) {
                balanceFlags |= (1 << i);  // Set bit to enable balancing for this cell
            }
        }
    }

    // Write the balance flags to the BQ76940 CELLBAL register
    I2C_writeRegister(BQ76940_CELLBAL1, balanceFlags);

    // Stop balancing once cells are within the stop threshold
    if (voltageDifference < BALANCE_STOP_THRESHOLD) {
        balanceFlags = 0;
        I2C_writeRegister(BQ76940_CELLBAL1, balanceFlags);
    }
}

// I2C write function for the CELLBAL register
void I2C_writeRegister(Uint16 registerAddress, Uint16 data) {
    I2caRegs.I2CSAR.all = BQ76940_I2C_ADDRESS;
    I2caRegs.I2CDXR.all = registerAddress;
    I2caRegs.I2CMDR.all = 0x6620;
    delay2(20000);
    I2caRegs.I2CDXR.all = data;
    I2caRegs.I2CMDR.all = 0x6C20;
    delay2(20000);
}