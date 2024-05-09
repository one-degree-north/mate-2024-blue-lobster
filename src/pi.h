//
// Created by Sidharth Maheshwari on 17/4/24.
//

#include <string>

#ifndef MATE_PI_H
#define MATE_PI_H


class Pi {
public:
    explicit Pi(const std::string& server_address);
    ~Pi();

    // GPIO Methods
    int SetGPIOMode(int gpio, int mode) const;

    // Shell Methods
    int Shell(const std::string& scriptName, const std::string& arguments) const;

    // PWM Methods
    int SetServoPulseWidth(int gpio, int pulse_width) const;

    // I2C Methods
    int OpenI2C(int bus, uint8_t address) const;
    int CloseI2C(int handle) const;

    int WriteI2CByte(int handle, uint8_t byte) const;
    int ReadI2CByte(int handle) const;

    int WriteI2CByteData(int handle, uint8_t reg, uint8_t byte) const;
    int ReadI2CByteData(int handle, uint8_t reg) const;

    int WriteI2CWordData(int handle, uint8_t reg, uint16_t word) const;
    int ReadI2CWordData(int handle, uint8_t reg) const;

    int WriteI2CBlockData(int handle, uint8_t reg, char* data, int length) const;
    int ReadI2CBlockData(int handle, uint8_t reg, char* data, int length) const;

    int ZipI2C(int handle, char *data, int length) const;

    enum I2C_ZIP_COMMAND {
        END = 0x00,
        ESCAPE = 0x01,
        ON = 0x02,
        OFF = 0x03,
        ADDRESS = 0x04,
        FLAGS = 0x05,
        READ = 0x06,
        WRITE = 0x07
    };
private:
    int pi_handle_;
};


#endif // MATE_PI_H
