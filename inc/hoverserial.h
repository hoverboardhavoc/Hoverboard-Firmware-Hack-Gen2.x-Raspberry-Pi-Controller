#ifndef HOVERSERIAL_H
#define HOVERSERIAL_H

#include <stdint.h>
#include <stdbool.h>
#include <termios.h> // For speed_t
#include <sys/ioctl.h> // For ioctl and FIONREAD

#include "util.h"

/**
 * Struct representing feedback data from the hoverboard controller.
 */
typedef struct __attribute__((packed, aligned(1))) {
  uint16_t cStart; // Start frame marker
  int16_t iSpeedL;   // 100* km/h
  int16_t iSpeedR;   // 100* km/h
  uint16_t iVolt;    // 100* V
  int16_t iAmpL;     // 100* A
  int16_t iAmpR;     // 100* A
  int32_t iOdomL;    // hall steps
  int32_t iOdomR;    // hall steps
  uint16_t checksum;
} SerialHover2Server;

/**
 * Struct representing a command frame to send to the hoverboard controller.
 */
typedef struct __attribute__((packed, aligned(1))) {
  uint8_t cStart; // Start frame marker
  int16_t iSpeed; // Speed command
  int16_t iSteer; // Steering command
  uint8_t wStateMaster; // Master state flags
  uint8_t wStateSlave;  // Slave state flags
  uint16_t checksum;
} SerialServer2Hover;

/**
 * Initialize UART with the given device and baud rate.
 * @param device Path to UART device
 * @param baud Baud rate constant (e.g., B19200)
 * @return File descriptor on success, -1 on error
 */
int initializeUart(const char *device, speed_t baud);

/**
 * Attempt to receive a feedback frame from the hoverboard controller.
 * @param fd UART file descriptor
 * @param Feedback Pointer to struct to fill with received data
 * @return true if a valid frame was received, false otherwise
 */
bool hoverReceive(int fd, SerialHover2Server* Feedback);

/**
 * Send a command frame to the hoverboard controller.
 * @param fd UART file descriptor
 * @param iSteer Steering value
 * @param iSpeed Speed value
 * @param wStateMaster Master state
 * @param wStateSlave Slave state
 */
void hoverSend(int fd, int16_t iSteer, int16_t iSpeed, uint8_t wStateMaster, uint8_t wStateSlave);


#endif // HOVERSERIAL_H