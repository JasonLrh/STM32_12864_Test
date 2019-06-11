#include "I2Cdev.h"



#ifndef BUFFER_LENGTH
// band-aid fix for platforms without Wire-defined BUFFER_LENGTH (removed from some official implementations)
#define BUFFER_LENGTH 32
#endif

/** Default constructor.
 */
I2Cdev::I2Cdev() {
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read multiple bits from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-15)
 * @param length Number of bits to read (not more than 16)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
 */
int8_t I2Cdev::readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout) {
    // 1101011001101001 read byte
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    //    010           masked
    //           -> 010 shifted
    uint8_t count;
    uint16_t w;
    if ((count = readWord(devAddr, regAddr, &w, timeout)) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        w &= mask;
        w >>= (bitStart - length + 1);
        *data = w;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout) {
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout) {
    return readWords(devAddr, regAddr, 1, data, timeout);
}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        printf("I2C (0x");
        printf(devAddr, HEX);
        printf(") reading ");
        printf(length, DEC);
        printf(" bytes from 0x");
        printf(regAddr, HEX);
        printf("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = HAL_GetTick();

    #if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE)

        #if (ARDUINO < 100)
            // Arduino v00xx (before v1.0), Wire library

            // I2C/TWI subsystem uses internal buffer that breaks with large data requests
            // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
            // smaller chunks instead of all at once
            for (uint8_t k = 0; k < length; k += min((int)length, BUFFER_LENGTH)) {
                HAL_I2C_Mem_Read(&I2C_Dev_Port,(uint16_t)devAddr,(uint16_t)regAddr,I2C_MEMADD_SIZE_8BIT,data,length,I2CDEV_DEFAULT_READ_TIMEOUT);
                for (;HAL_I2C_GetState(&I2C_Dev_Port) != HAL_I2C_STATE_READY;) {
                    
                    #ifdef I2CDEV_SERIAL_DEBUG
                        printf(data[count], HEX);
                        if (count + 1 < length) printf(" ");
                    #endif
                }


            }

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        // Fastwire library
        // no loop required for fastwire
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, data, length);
        if (status == 0) {
            count = length; // success
        } else {
            count = -1; // error
        }

    #endif
    // check for timeout
    if (timeout > 0 && HAL_GetTick() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        printf(". Done (");
        printf(count, DEC);
        printf(" read).\n");
    #endif

    return count;
}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of words read (-1 indicates failure)
 */
int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout) {
    #ifdef I2CDEV_SERIAL_DEBUG
        printf("I2C (0x");
        printf(devAddr, HEX);
        printf(") reading ");
        printf(length, DEC);
        printf(" words from 0x");
        printf(regAddr, HEX);
        printf("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = HAL_GetTick();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE || I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_SBWIRE

        #if (ARDUINO < 100)
            // Arduino v00xx (before v1.0), Wire library

            // I2C/TWI subsystem uses internal buffer that breaks with large data requests
            // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
            // smaller chunks instead of all at once
            for (uint8_t k = 0; k < length * 2; k += min(length * 2, BUFFER_LENGTH)) {
                HAL_I2C_Mem_Read(&I2C_Dev_Port,(uint16_t)devAddr,(uint16_t)regAddr,I2C_MEMADD_SIZE_8BIT,data,length,I2CDEV_DEFAULT_READ_TIMEOUT);
                for (;HAL_I2C_GetState(&I2C_Dev_Port) != HAL_I2C_STATE_READY;) {
                    
                    #ifdef I2CDEV_SERIAL_DEBUG
                        printf(data[count], HEX);
                        if (count + 1 < length) printf(" ");
                    #endif
                }
            }
        
        #endif

    #elif (I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE)

        // Fastwire library
        // no loop required for fastwire
        uint16_t intermediate[(uint8_t)length];
        uint8_t status = Fastwire::readBuf(devAddr << 1, regAddr, (uint8_t *)intermediate, (uint8_t)(length * 2));
        if (status == 0) {
            count = length; // success
            for (uint8_t i = 0; i < length; i++) {
                data[i] = (intermediate[2*i] << 8) | intermediate[2*i + 1];
            }
        } else {
            count = -1; // error
        }

    #endif

    if (timeout > 0 && HAL_GetTick() - t1 >= timeout && count < length) count = -1; // timeout

    #ifdef I2CDEV_SERIAL_DEBUG
        printf(". Done (");
        printf(count, DEC);
        printf(" read).\n");
    #endif
    
    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return writeByte(devAddr, regAddr, b);
    } else {
        return false;
    }
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) {
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask word
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;
    if (readWord(devAddr, regAddr, &w) != 0) {
        uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        return writeWord(devAddr, regAddr, w);
    } else {
        return false;
    }
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data) {
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        printf("I2C (0x");
        printf(devAddr, HEX);
        printf(") writing ");
        printf(length, DEC);
        printf(" bytes to 0x");
        printf(regAddr, HEX);
        printf("...");
    #endif
    HAL_I2C_Mem_Write(&I2C_Dev_Port,(uint16_t)devAddr,(uint16_t)regAddr,I2C_MEMADD_SIZE_8BIT,data,length,I2CDEV_DEFAULT_READ_TIMEOUT);

    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            printf(data[i], HEX);
            if (i + 1 < length) printf(" ");
        #endif

    }

    return 0;
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data) {
    #ifdef I2CDEV_SERIAL_DEBUG
        printf("I2C (0x");
        printf(devAddr, HEX);
        printf(") writing ");
        printf(length, DEC);
        printf(" words to 0x");
        printf(regAddr, HEX);
        printf("...");
    #endif
    HAL_I2C_Mem_Write(&I2C_Dev_Port,(uint16_t)devAddr,(uint16_t)regAddr,I2C_MEMADD_SIZE_8BIT,data,length,I2CDEV_DEFAULT_READ_TIMEOUT);

    return 0;
}

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;

