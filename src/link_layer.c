// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define BUF_SIZE 256
#define FLAG 0x7E
#define A_T 0x03 // Endereço de Comando (T -> R)
#define A_R 0x01 // Endereço de Resposta (R -> T)
#define C_SET 0x03 // Campo de Controlo para SET
#define C_UA 0x07  // Campo de Controlo para UA
#define BCC_SET (A_T ^ C_SET) // BCC para SET: 0x03 ^ 0x03 = 0x00
#define BCC_UA (A_R ^ C_UA)  // BCC para UA: 0x01 ^ 0x07 = 0x06
#define ESC 0x7D
#define STUFF_7E 0x5E  // 0x7E -> {0x7D, 0x5E}
#define STUFF_7D 0x5D  // 0x7D -> {0x7D, 0x5D}
#define C_I(ns) ((ns) ? 0x40 : 0x00)
#define C_RR(nr) (0x05 | ((nr) << 7))
#define C_REJ(ns) (0x01 | ((ns) << 7))

int nretransmissions;
int timeout;
static int ns = 0; //sequence number

typedef enum{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_RCV,
    STOPP,
} State;


volatile int timeout_flag = FALSE;

void alarm_handler(int signo)
{
    timeout_flag = TRUE;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    nretransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;

        //error handling
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate)< 0)
    {
        perror("openSerialPort");
        exit(-1);
    }

    printf("Serial port %s opened\n", connectionParameters.serialPort);

    const unsigned char SET_FRAME[BUF_SIZE] = {FLAG, A_T, C_SET, BCC_SET, FLAG};
    const unsigned char UA_FRAME[BUF_SIZE]  = {FLAG, A_R, C_UA, BCC_UA, FLAG};

    switch(connectionParameters.role) {
        case LlTx: {
            (void) signal(SIGALRM, alarm_handler);
            State state = START;
            while(nretransmissions != 0) {
                
                int bytesWritten = writeBytesSerialPort(SET_FRAME, BUF_SIZE);
                //error handling
                if (bytesWritten != BUF_SIZE) {
                    perror("writeBytesSerialPort SET");
                    closeSerialPort();
                    return -1;
                }
                
                alarm(timeout);
                timeout_flag = FALSE;


                printf("Transmitter: Sent SET frame (Try %d/%d). Frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
                    nretransmissions, connectionParameters.nRetransmissions, 
                    SET_FRAME[0], SET_FRAME[1], SET_FRAME[2], SET_FRAME[3], SET_FRAME[4]);
                    
                state = START;
                unsigned char byte;

                while(state != STOPP && timeout_flag == FALSE) {
                    int bytesRead = readByteSerialPort(&byte);
                    //error handling
                    if(bytesRead == -1) {
                        perror("readByteSerialPort");
                        break;
                    }

                    switch(state){

                        case START:
                                if(byte == FLAG)  state = FLAG_RCV;
                                printf("Start\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case FLAG_RCV:
                                if(byte == A_R)   state = A_RCV;
                                else if(byte != FLAG) state = START;
                                printf("Flag\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case A_RCV:
                                if(byte == FLAG) state = FLAG_RCV;
                                else if(byte == C_UA) state = C_RCV;
                                else state = START;
                                printf("A\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case C_RCV:
                                if(byte == BCC_UA) state = BCC_RCV;
                                else if(byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                printf("C\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;

                        case BCC_RCV:
                                if(byte == FLAG) state = STOPP;
                                else state = START;
                                printf("BCC\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;

                        default:
                                break;
                    }
                }
                
                alarm(0);

                if (state == STOPP) {
                    printf("Transmitter: UA received successfully. Connection established.\n");
                    closeSerialPort();
                    return 0;
                }

                nretransmissions--;
                printf("Transmitter: Timeout or invalid frame. Retrying... %d tries left.\n", nretransmissions);
            }
            printf("Transmitter: Max retransmissions reached. Connection failed.\n");
            closeSerialPort();
            return -1;
        }
        case LlRx: {
            State state = START;
            unsigned char byte;
            
            while(state != STOPP) {
                int bytesRead = readByteSerialPort(&byte);

                //error handling
                if(bytesRead == -1) {
                    perror("readByteSerialPort");
                    closeSerialPort();
                    return -1;  
                }

                switch(state){

                    case START:
                            if(byte == FLAG)  state = FLAG_RCV;
                            printf("Start\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case FLAG_RCV:
                            if(byte == A_T)   state = A_RCV;
                            else if(byte != FLAG) state = START;
                            printf("Flag\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case A_RCV:
                            if(byte == C_SET) state = C_RCV;
                            else if(byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            printf("A\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case C_RCV:
                            if(byte == BCC_SET) state = BCC_RCV;
                            else if(byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            printf("C\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;

                    case BCC_RCV:
                            if(byte == FLAG) state = STOPP;
                            else state = START;
                            printf("BCC\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;

                    default:
                            break;

                }
            }

            if(state == STOPP) {
                int bytesWritten = writeBytesSerialPort(UA_FRAME, BUF_SIZE);
                
                //error handling
                if(bytesWritten != BUF_SIZE) {
                    perror("writeBytesSerialPort UA");
                    closeSerialPort();
                    return -1;  
                }

                printf("Receiver: SET received successfully. Sent UA frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
                UA_FRAME[0], UA_FRAME[1], UA_FRAME[2], UA_FRAME[3], UA_FRAME[4]);
                return 0;
            }
            closeSerialPort();
            return -1;
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // Check preconditions
    if (buf == NULL || bufSize <= 0 || bufSize > BUF_SIZE) return -1;

    // Calculate BCC2
    unsigned char bcc2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        bcc2 ^= buf[i];
    }

    // Prepare data with BCC2 (before stuffing)
    unsigned char data[bufSize + 1];
    memcpy(data, buf, bufSize); //copy a sequence of bytes from buf to data, (bufSize = number of bytes being copied)
    data[bufSize] = bcc2;
    int dataSize = bufSize + 1;

    // Perform byte stuffing
    unsigned char stuffed[2 * dataSize];  // Worst-case size
    int stuffedSize = 0;
    for (int i = 0; i < dataSize; i++) {
        if (data[i] == FLAG) {
            stuffed[stuffedSize++] = ESC;
            stuffed[stuffedSize++] = STUFF_7E;
        } else if (data[i] == ESC) {
            stuffed[stuffedSize++] = ESC;
            stuffed[stuffedSize++] = STUFF_7D;
        } else {
            stuffed[stuffedSize++] = data[i];
        }
    }

    // Build I frame
    unsigned char C = C_I(ns); // 0x00 or 0x40
    unsigned char bcc1 = A_T ^ C;
    int frameSize = stuffedSize + 5; // FLAG | A | C | BCC1 | stuffed_data | FLAG
    unsigned char *frame = (unsigned char *)malloc(frameSize * sizeof(unsigned char));
    if (frame == NULL) return -1;
    frame[0] = FLAG;
    frame[1] = A_T;
    frame[2] = C;
    frame[3] = bcc1;
    memcpy(frame + 4, stuffed, stuffedSize);
    frame[frameSize - 1] = FLAG;

    // Prepare for retransmissions
    int currentTry = 0;
    int nr = (ns + 1) % 2; //next expected sequence number
    unsigned char expected_rr_C = C_RR(nr); // 0x05 | (nr << 7)
    unsigned char rej_C = C_REJ(ns);        // 0x01 | (ns << 7)
    State state = START;
    
    do {
        // Send frame
        int bytesWritten = writeBytesSerialPort(frame, frameSize);
        if (bytesWritten != frameSize) {
            perror("writeBytesSerialPort I frame");
            return -1;
        }

        // Set alarm
        alarm(timeout);
        timeout_flag = FALSE;

        // Wait for response
        unsigned char byte, received_A = 0, received_C = 0, received_BCC1 = 0;
        while (state != STOPP && !timeout_flag) {
            int bytesRead = readByteSerialPort(&byte);
            if (bytesRead <= 0) continue; // No data or error, loop

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    printf("Start - Byte: 0x%02X\n", byte);
                    break;
                case FLAG_RCV:
                    if (byte == A_R) { state = A_RCV; received_A = byte; }
                    else if (byte != FLAG) state = START;
                    printf("Flag - Byte: 0x%02X\n", byte);
                    break;
                case A_RCV:
                    if (byte == expected_rr_C || byte == rej_C) {
                        state = C_RCV; received_C = byte;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    printf("A - Byte: 0x%02X\n", byte);
                    break;
                case C_RCV:
                    if (byte == (received_A ^ received_C)) {
                        state = BCC_RCV; received_BCC1 = byte;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    printf("C - Byte: 0x%02X\n", byte);
                    break;
                case BCC_RCV:
                    if (byte == FLAG) state = STOPP;
                    else state = START;
                    printf("BCC - Byte: 0x%02X\n", byte);
                    break;
            }
        }
        alarm(0); // Cancel alarm

        if (state == STOPP) {
            if (received_BCC1 == (received_A ^ received_C)) {
                if (received_C == expected_rr_C) {
                    printf("Received RR (positive ack)\n");
                    ns = 1 - ns; // Toggle sequence number
                    return bufSize; // Success
                } else if (received_C == rej_C) {
                    printf("Received REJ (negative ack). Retransmitting...\n");
                }
            } else {
                printf("Invalid BCC1 in response. Retransmitting...\n");
            }
        } else {
            printf("Timeout. Retransmitting...\n");
        }
        currentTry++;
    } while (currentTry < nretransmissions);

    printf("Max retransmissions reached. Failed.\n");
    free(frame);
    return -1;
}
////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
