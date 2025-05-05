#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdbool.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include "hoverserial.h"

#define DEFAULT_BAUD B19200
#define DEFAULT_JOYSTICK 0

void print_usage(const char *prog)
{
    printf("Usage: %s -d <device> -a <axis> -m <button> [-b baud] [-j joystick] [-l]\n", prog);
    printf("  -d <device>    UART device (required)\n");
    printf("  -a <axis>      Joystick axis for speed (required)\n");
    printf("  -m <button>    Joystick button for arming (required)\n");
    printf("  -b <baud>      Baud rate (default: B19200)\n");
    printf("  -j <index>     Joystick index (default: %d)\n", DEFAULT_JOYSTICK);
    printf("  -l             List available joysticks and exit\n");
}

void handleEvents(int axis_speed, int button_arm, int fd)
{
    SDL_Event e;
    int running = 1;
    int16_t iSpeed = 0;
    Uint8 latestButtonDown = 0;
    Uint8 latestButtonUp = 0;
    Uint8 latestAxis = 0;
    bool armed = false;

    while (running)
    {
        while (SDL_PollEvent(&e))
        {
            switch (e.type)
            {
            case SDL_QUIT:
                running = 0;
                break;
            case SDL_JOYAXISMOTION:
                latestAxis = e.jaxis.axis;
                if (e.jaxis.axis == axis_speed)
                {
                    // Map axis value to iSpeed (-1000 to 1000, negative is full power)
                    iSpeed = interpolate(e.jaxis.value, SDL_JOYSTICK_AXIS_MAX, SDL_JOYSTICK_AXIS_MIN, -1000, 1000);
                }
                break;
            case SDL_JOYBUTTONDOWN:
                if (e.jbutton.button == button_arm)
                {
                    armed = true;
                }
                latestButtonDown = e.jbutton.button;
                break;
            case SDL_JOYBUTTONUP:
                if (e.jbutton.button == button_arm)
                {
                    armed = false;
                }
                latestButtonUp = e.jbutton.button;
                break;
            }
        }
        if (armed)
        {
            hoverSend(fd, 0, iSpeed, 32, 0);
        }
        SerialHover2Server oData = {0};
        if (hoverReceive(fd, &oData))
        {
            printf("\r\033[KiOdomL: %d\tiOdomR: %d\tiSpeedL: %d\tiSpeedR: %d\tiAmpL: %d\tiAmpR: %d\tiVolt: %d   \n",
                   oData.iOdomL, oData.iOdomR, oData.iSpeedL, oData.iSpeedR, oData.iAmpL, oData.iAmpR, oData.iVolt);
        }
        else
        {
            printf("\n");
        }
        printf("\r\033[Kspeed: %d\tarmed: %d\tlatestButtonDown: %d\tlatestButtonUp: %d\tlatestAxis: %d\n", iSpeed, armed, latestButtonDown, latestButtonUp, latestAxis);
        printf("\033[2A");
        fflush(stdout);
        SDL_Delay(10); // avoid maxing out CPU
    }
}

/**
 * Main entry point: Parses options, initializes UART and SDL, reads joystick events,
 * and sends speed commands to the hoverboard controller.
 */
int main(int argc, char *argv[])
{
    const char *device = NULL;
    speed_t baud = DEFAULT_BAUD;
    int axis_speed = -1;
    int button_arm = -1;
    int joystick_index = DEFAULT_JOYSTICK;
    int list_joysticks = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:b:a:m:j:lh")) != -1)
    {
        switch (opt)
        {
        case 'd':
            device = optarg;
            break;
        case 'b':
            if (strcmp(optarg, "9600") == 0)
                baud = B9600;
            else if (strcmp(optarg, "19200") == 0)
                baud = B19200;
            else if (strcmp(optarg, "38400") == 0)
                baud = B38400;
            else if (strcmp(optarg, "57600") == 0)
                baud = B57600;
            else if (strcmp(optarg, "115200") == 0)
                baud = B115200;
            else
            {
                printf("Unsupported baud rate: %s\n", optarg);
                print_usage(argv[0]);
                return 1;
            }
            break;
        case 'a':
            axis_speed = atoi(optarg);
            break;
        case 'm':
            button_arm = atoi(optarg);
            break;
        case 'j':
            joystick_index = atoi(optarg);
            break;
        case 'l':
            list_joysticks = 1;
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    if (!device)
    {
        printf("UART device (-d) is required.\n");
        print_usage(argv[0]);
        return 1;
    }
    if (axis_speed < 0)
    {
        printf("Joystick axis (-a) is required.\n");
        print_usage(argv[0]);
        return 1;
    }
    if (button_arm < 0)
    {
        printf("Armed button (-m) is required.\n");
        print_usage(argv[0]);
        return 1;
    }

    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_EVENTS) < 0)
    {
        printf("SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    int num_joysticks = SDL_NumJoysticks();
    if (list_joysticks)
    {
        printf("Available joysticks:\n");
        for (int i = 0; i < num_joysticks; ++i)
        {
            printf("  [%d] %s\n", i, SDL_JoystickNameForIndex(i));
        }
        SDL_Quit();
        return 0;
    }

    if (num_joysticks < 1)
    {
        printf("No joysticks found.\n");
        SDL_Quit();
        return 1;
    }
    if (joystick_index < 0 || joystick_index >= num_joysticks)
    {
        printf("Invalid joystick index: %d\n", joystick_index);
        SDL_Quit();
        return 1;
    }

    SDL_Joystick *joystick = SDL_JoystickOpen(joystick_index);
    if (joystick)
    {
        printf("Opened joystick: %s\n", SDL_JoystickName(joystick));
    }
    else
    {
        printf("Could not open joystick: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    int fd = initializeUart(device, baud);
    if (fd < 0)
    {
        printf("Failed to initialize UART. Exiting.\n");
        SDL_JoystickClose(joystick);
        SDL_Quit();
        return 1;
    }

    handleEvents(axis_speed, button_arm, fd);
    SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}
