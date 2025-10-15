// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <unistd.h>


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

typedef enum{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_RCV,
    STOPP,
} State;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int nretransmissions = connectionParameters.nRetransmissions;
    int timeout = connectionParameters.timeout;

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
            State state = START;
            while(nretransmissions != 0) {

                int bytesWritten = writeBytesSerialPort(SET_FRAME, BUF_SIZE);
                if (bytesWritten != BUF_SIZE) {
                    perror("writeBytesSerialPort SET");
                    closeSerialPort();
                    return -1;
                }
                printf("Transmitter: Sent SET frame (Try %d/%d). Frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
                    nretransmissions, connectionParameters.nRetransmissions, 
                    SET_FRAME[0], SET_FRAME[1], SET_FRAME[2], SET_FRAME[3], SET_FRAME[4]);
                    
                state = START;
                unsigned char byte;

                while(state != STOPP) {
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

                if (state == STOPP) {
                    printf("Transmitter: UA received successfully. Connection established.\n");
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
    // TODO: Implement this function

    return 0;
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
