// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define BUF_SIZE 256

typedef enum{
    START,
    FLAG,
    A,
    C,
    BCC,
    STOPP,
} State;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int nretransmissions = connectionParameters.nRetransmissions;

    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate)< 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    
    State state = START;

    printf("Serial port %s opened\n", connectionParameters.serialPort);
    switch(connectionParameters.role) {
        case LlTx: {
            while(nretransmissions != 0 && state != STOPP) {

                //supervision e alarmes
                int nBytesBuf = 0;
                unsigned char byte;
                int bytes = readByteSerialPort(&byte);

                while(state != STOPP) {
                    switch(state){

                        case START:
                                nBytesBuf++;
                                if(byte == 0x7E)  state = FLAG;
                                printf("Start\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case FLAG:
                                nBytesBuf++;
                                if(byte == 0x07)   state = A;
                                else if(byte != 0x7E) state = START;
                                printf("Flag\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case A:
                                nBytesBuf++;
                                if(byte == 0x7E) state = FLAG;
                                else if(byte == 0x07) state = C;
                                else state = START;
                                printf("A\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case C:
                                nBytesBuf++;
                                if(byte == 0) state = BCC;
                                else if(byte == 0x7E) state = FLAG;
                                else state = START;
                                printf("C\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;

                        case BCC:
                                nBytesBuf++;
                                if(byte == 0x7E) state = STOPP;
                                else state = START;
                                printf("BCC\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;

                        default:
                                break;
                    }
                }
                    nretransmissions--;
            }
            unsigned char buf[BUF_SIZE] = {0};    
            buf[0]=0x7E;
            buf[1]=0x03;
            buf[2]=0x03;
            buf[3]=buf[1] ^ buf[2];
            buf[4]=0x7E;
            if (state != STOPP) return -1;
            break; 
        }
        case LlRx: {
            while(state != STOPP) {

                int nBytesBuf = 0;
                unsigned char byte;
                int bytes = readByteSerialPort(&byte);

                switch(state){

                    case START:
                            nBytesBuf++;
                            if(byte == 0x7E)  state = FLAG;
                            printf("Start\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case FLAG:
                            nBytesBuf++;
                            if(byte == 0x03)   state = A;
                            else if(byte != 0x7E) state = START;
                            printf("Flag\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case A:
                            nBytesBuf++;
                            if(byte == 0x7E) state = FLAG;
                            else if(byte == 0x03) state = C;
                            else state = START;
                            printf("A\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;


                    case C:
                            nBytesBuf++;
                            if(byte == 0) state = BCC;
                            else if(byte == 0x7E) state = FLAG;
                            else state = START;
                            printf("C\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;

                    case BCC:
                            nBytesBuf++;
                            if(byte == 0x7E) state = STOPP;
                            else state = START;
                            printf("BCC\n");
                            printf("Byte received: 0x%02X\n", byte);
                            break;

                    default:
                            break;

                }
            }
            unsigned char buf[BUF_SIZE] = {0};
            buf[0]=0x7E;
            buf[1]=0x01;
            buf[2]=0x07;
            buf[3]=0x06;
            buf[4]=0x7E;
        }
        int bytes = writeBytesSerialPort(buf,5);
        break;
    }
    return 0;
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
