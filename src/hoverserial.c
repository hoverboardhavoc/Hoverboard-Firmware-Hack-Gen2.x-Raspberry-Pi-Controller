#include "hoverserial.h"
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h> // Required for tcdrain
#include <unistd.h>

const uint16_t START_FRAME = 0xABCD; // Start frame definition for reliable serial communication

uint16_t calcCRC(uint8_t *ptr, int count)
{
    uint16_t crc;
    uint8_t i;
    crc = 0;
    while (--count >= 0)
    {
        crc = crc ^ (uint16_t)*ptr++ << 8;
        i = 8;
        do
        {
            if (crc & 0x8000)
            {
                crc = crc << 1 ^ 0x1021;
            }
            else
            {
                crc = crc << 1;
            }
        } while (--i);
    }
    return (crc);
}

int initializeUart(const char *device, speed_t baud)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Unable to open UART");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // Set baud rate
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    // Set 8 data bits, no parity, 1 stop bit
    options.c_cflag &= ~PARENB; // Disable parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;  // Clear data bits setting
    options.c_cflag |= CS8;     // 8 data bits

    // Set raw input/output mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST;                          // Raw output

    // Apply the settings
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);
    return fd;
}

/**
 * Send a command frame to the hoverboard motor controller.
 * @param fd UART file descriptor
 * @param iSteer Steering value
 * @param iSpeed Speed value
 * @param wStateMaster Master state
 * @param wStateSlave Slave state
 */
void hoverSend(int fd, int16_t iSteer, int16_t iSpeed, uint8_t wStateMaster /*=32*/, uint8_t wStateSlave /*=0*/)
{
    SerialServer2Hover oData = {0};
    oData.cStart = '/'; // 0xABCD;
    oData.iSpeed = iSpeed;
    oData.iSteer = iSteer;
    oData.wStateMaster = wStateMaster;
    oData.wStateSlave = wStateSlave;
    oData.checksum = calcCRC((uint8_t *)&oData, sizeof(SerialServer2Hover) - 2); // first bytes except crc

    write(fd, (uint8_t *)&oData, sizeof(SerialServer2Hover));

    // Ensure all data is transmitted
    if (tcdrain(fd) == -1)
    {
        perror("tcdrain failed");
    }
}

void hexDump(const void *data, size_t size)
{
    const uint8_t *byteData = (const uint8_t *)data;
    for (size_t i = 0; i < size; i++)
    {
        printf("%02X ", byteData[i]); // Print each byte in hexadecimal
        if ((i + 1) % 16 == 0)
        { // Add a newline every 16 bytes
            printf("\n");
        }
    }
    if (size % 16 != 0)
    {
        printf("\n"); // Final newline if the last line isn't complete
    }
}

/**
 * Attempt to read and parse one SerialHover2Server frame from a Unix fd.
 * @param fd         file descriptor to read from
 * @param outFrame   pointer to caller-allocated struct to fill on success
 * @return           true if a valid frame was read & checksum matched
 */
bool hoverReceive(int fd, SerialHover2Server *outFrame)
{
    int avail = 0;
    if (ioctl(fd, FIONREAD, &avail) < 0)
    {
        perror("ioctl FIONREAD");
        return false;
    }

    // Do we have room for a full frame?
    int too_much = avail - (int)sizeof(SerialHover2Server) + 1;
    int8_t bFirst = 1;

    while (too_much >= bFirst)
    {
        uint8_t c;
        if (read(fd, &c, 1) != 1)
        {
            // read error or EOF
            return false;
        }
        too_much--;

        if (bFirst)
        {
            // waiting for START_FRAME low byte
            if (c == (uint8_t)(START_FRAME & 0xFF))
            {
                bFirst = 0;
            }
        }
        else
        {
            // waiting for START_FRAME high byte
            if (c == (uint8_t)((START_FRAME >> 8) & 0xFF))
            {
                SerialHover2Server tmp;
                tmp.cStart = START_FRAME;

                // read remainder of struct
                uint8_t *p = (uint8_t *)&tmp + 2;
                size_t to_read = sizeof(SerialHover2Server) - 2;
                if (read(fd, p, to_read) != (ssize_t)to_read)
                {
                    return false;
                }

                // verify CRC over all bytes except the checksum field
                uint16_t cs = calcCRC((uint8_t *)&tmp, sizeof(tmp) - 2);

                if (cs == tmp.checksum)
                {
                    memcpy(outFrame, &tmp, sizeof(tmp));
                    return true;
                }
                return false;
            }
            // if this byte looks like the low-byte again, stay in state=0, else reset
            // first_ok = (c == (uint8_t)(START_FRAME & 0xFF)) ? 0 : 1;
            if (c != (uint8_t)(START_FRAME & 0xFF))
            {
                bFirst = 1;
            }
        }
    }

    return false;
}

/*void debugOut(uint8_t aBuffer[], size_t iSize)
{
  for (size_t i=0; i<iSize; i++)
  {
    uint8_t c = aBuffer[i];
    printf((c < 16) ? " 0%X" : " %X", c);
  }
  printf("\n");
}*/