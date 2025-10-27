// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>

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
#define C_DISC 0x0B 
#define C_I(ns) ((ns) ? 0x40 : 0x00)
#define C_RR(nr) (0x05 | ((nr) << 7))
#define C_REJ(ns) (0x01 | ((ns) << 7))

#define TRUE 1
#define FALSE 0


int nretransmissions;
int timeout;
LinkLayerRole role;

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

static void send_RR(int nr) {
    unsigned char frame[5] = {FLAG, A_R, C_RR(nr), A_R ^ C_RR(nr), FLAG};
    writeBytesSerialPort(frame, 5);
}

static void send_REJ(int ns) {
    unsigned char frame[5] = {FLAG, A_R, C_REJ(ns), A_R ^ C_REJ(ns), FLAG};
    writeBytesSerialPort(frame, 5);
}

static int destuff(const unsigned char *stuffed, int stuffed_len,
                        unsigned char *destuffed, int *destuffed_len)
{
    if (stuffed == NULL || destuffed == NULL || destuffed_len == NULL || stuffed_len < 1)
        return -1;

    int i = 0, j = 0;
    while (i < stuffed_len) {
        if (stuffed[i] == ESC) {
            ++i;
            if (i >= stuffed_len) return -1;
            if (stuffed[i] == STUFF_7E) {
                destuffed[j++] = FLAG;
            } else if (stuffed[i] == STUFF_7D) {
                destuffed[j++] = ESC;
            } else {
                return -1;  // Sequência inválida
            }
        } else {
            destuffed[j++] = stuffed[i];
        }
        ++i;
    }
    *destuffed_len = j;
    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    nretransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    int tries = 0;
    role = connectionParameters.role;


    //error handling
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) return -1;

    printf("Serial port %s opened\n", connectionParameters.serialPort);

    const unsigned char SET_FRAME[5] = {FLAG, A_T, C_SET, BCC_SET, FLAG};
    const unsigned char UA_FRAME[5]  = {FLAG, A_R, C_UA, BCC_UA, FLAG};

    switch(role) {
        case LlTx: {
            (void) signal(SIGALRM, alarm_handler);
            State state = START;
            while(tries < nretransmissions) {
                
                int bytesWritten = writeBytesSerialPort(SET_FRAME, 5);
                //error handling
                //if (bytesWritten == -1) return -1;
                if (bytesWritten != 5) return -1;
                
                alarm(timeout);
                timeout_flag = FALSE;


                printf("Transmitter: Sent SET frame (Try %d/%d). Frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
                    tries + 1, connectionParameters.nRetransmissions, 
                    SET_FRAME[0], SET_FRAME[1], SET_FRAME[2], SET_FRAME[3], SET_FRAME[4]);
                    
                state = START;
                unsigned char byte;

                while(state != STOPP && timeout_flag == FALSE) {
                    int bytesRead = readByteSerialPort(&byte);
                    if(bytesRead == 1) {
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
                }
                
                alarm(0);
                if (state == STOPP) {
                    printf("Transmitter: UA received successfully. Connection established.\n");
                    return 0;
                }

                tries++;
                printf("Transmitter: Timeout or invalid frame. Retrying... %d tries left.\n", nretransmissions);
            }
            printf("Transmitter: Max retransmissions reached. Connection failed.\n");
            return -1;
        }
        case LlRx: {
            State state = START;
            unsigned char byte;
            
            while(state != STOPP) {
                int bytesRead = readByteSerialPort(&byte);
                // if (bytesRead < 0) return -1; // Error
                // if (bytesRead == 0) continue; // No data
                if (bytesRead == 1) {
                    switch(state){

                        case START:
                                if(byte == FLAG)  state = FLAG_RCV;
                                // printf("Start\n");
                                // printf("Byte received: 0x%02X\n", byte);
                                break;


                        case FLAG_RCV:
                                if(byte == A_T)   state = A_RCV;
                                else if(byte != FLAG) state = START;
                                // printf("Flag\n");
                                // printf("Byte received: 0x%02X\n", byte);
                                break;


                        case A_RCV:
                                if(byte == C_SET) state = C_RCV;
                                else if(byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                // printf("A\n");
                                // printf("Byte received: 0x%02X\n", byte);
                                break;


                        case C_RCV:
                                if(byte == BCC_SET) state = BCC_RCV;
                                else if(byte == FLAG) state = FLAG_RCV;
                                else state = START;
                                // printf("C\n");
                                // printf("Byte received: 0x%02X\n", byte);
                                break;

                        case BCC_RCV:
                                if(byte == FLAG) state = STOPP;
                                else state = START;
                                // printf("BCC\n");
                                // printf("Byte received: 0x%02X\n", byte);
                                break;

                        default:
                                break;

                    }
                }
            }

            int bytesWritten = writeBytesSerialPort(UA_FRAME, 5);
            
            //error handling
            if(bytesWritten != 5) {
                perror("writeBytesSerialPort UA");
                return -1;  
            }

            printf("Receiver: SET received successfully. Sent UA frame: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
            UA_FRAME[0], UA_FRAME[1], UA_FRAME[2], UA_FRAME[3], UA_FRAME[4]);
            return 0;
        }
    }
    return -1;
}

////////////////////////////////////////////////
// 
//LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    static int ns = 0; //sequence number

    if (role != LlTx) {
        printf("llread called in wrong role (not Tx)\n");
        return -1;
    }

    // Check preconditions
    if (buf == NULL || bufSize <= 0 || bufSize > BUF_SIZE) return -1;

    // Calculate BCC2
    unsigned char bcc2 = 0;
    for (int i = 0; i < bufSize; i++) {
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
    unsigned char frame[frameSize];
    frame[0] = FLAG;
    frame[1] = A_T;
    frame[2] = C;
    frame[3] = bcc1;
    memcpy(frame + 4, stuffed, stuffedSize);
    frame[frameSize - 1] = FLAG;

    // Prepare for retransmissions
    unsigned char expected_rr = C_RR(1 - ns); // 0x05 | (nr << 7)
    unsigned char expected_rej = C_REJ(ns);        // 0x01 | (ns << 7)
   
    int tries = 0;
    while (tries < nretransmissions){
        // Send frame
        int bytesWritten = writeBytesSerialPort(frame, frameSize);
        if (bytesWritten != frameSize) {
            perror("writeBytesSerialPort I frame");
            return -1;
        }

        // Set alarm
        alarm(timeout);
        timeout_flag = FALSE;

        State state = START;
        // Wait for response
        unsigned char byte, received_A = 0, received_C = 0;
        while (state != STOPP && !timeout_flag) {
            int bytesRead = readByteSerialPort(&byte);
            if (bytesRead <= 0) { continue; } 

            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == A_R) { state = A_RCV; received_A = byte; }
                    else if (byte != FLAG) state = START;
                    break;
                case A_RCV:
                    if (byte == expected_rr || byte == expected_rej) {
                        state = C_RCV; received_C = byte;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case C_RCV:
                    if (byte == (received_A ^ received_C)) {
                        state = BCC_RCV;
                    } else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC_RCV:
                    if (byte == FLAG) state = STOPP;
                    else state = START;
                    break;
            }
        }
        alarm(0); // Cancel alarm

        if (state == STOPP) {
                if (received_C == expected_rr) {
                    printf("Received RR (positive ack)\n");
                    ns = 1 - ns; // Toggle sequence number
                    return bufSize; // Success
                } else if (received_C == expected_rej) {
                    continue;
                }
        }
        tries++;
    }
    printf("llwrite: Max retransmissions reached.\n");
    return -1;
}

////////////////////////////////////////////////
// LLREAD
/////////////////////////////////i

int llread(unsigned char *packet)
{
    if (role != LlRx || packet == NULL) return -1;

    static int expected_ns = 0;               // sequência esperada
    State state = START;
    unsigned char stuffed[2 * BUF_SIZE];
    int stuffed_len = 0;
    unsigned char c_field = 0;
    int ns_received = -1;

    while (1) {
        unsigned char byte;
        int bytesRead = readByteSerialPort(&byte);
        if (bytesRead <= 0) continue; //no data

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    stuffed_len = 0;
                }
                break;

            case FLAG_RCV:
                if (byte == A_T) {
                    state = A_RCV;
                } else if (byte != FLAG) {
                    state = START;
                }
                break;

            case A_RCV:
                if (byte == C_I(0) || byte == C_I(1)) {
                    c_field = byte;
                    ns_received = (byte == C_I(1)) ? 1 : 0;
                    state = C_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case C_RCV:
                if (byte == (A_T ^ c_field)) {
                    state = BCC_RCV;
                } else if (byte == FLAG) {
                    state = FLAG_RCV;
                } else {
                    state = START;
                }
                break;

            case BCC_RCV:
                if (byte == FLAG) {
                    // Fim do I-frame recebido
                    if (stuffed_len < 1) {  // Pelo menos BCC2
                        state = START;
                        continue;
                    }

                    // 1. Destuff completo (inclui BCC2)
                    unsigned char destuffed[2 * BUF_SIZE];
                    int destuffed_len = 0;

                    // O campo de dados + BCC2 deve ter pelo menos 1 byte (BCC2).
                    // Se destuffed_len == 0 → não tem nem o BCC2 → frame incompleto ou corrompido.
                    // frame invalido:corrompido ou incompleto
                    if (destuff(stuffed, stuffed_len, destuffed, &destuffed_len) != 0 || destuffed_len < 1) {
                        send_REJ(ns_received);
                        state = START;
                        continue;
                    }

                    // 2. Separar BCC2 (último byte) e dados
                    unsigned char bcc2_recv = destuffed[destuffed_len - 1];
                    int data_len = destuffed_len - 1;
                    // 3. Calcular BCC2
                    unsigned char bcc2_calc = 0;
                    for (int i = 0; i < data_len; i++) {
                        bcc2_calc ^= destuffed[i];
                    }
                    


                    // 4. Determinar se é novo ou duplicado
                    bool is_new_frame = (ns_received == expected_ns);
                    bool bcc2_error = (bcc2_calc != bcc2_recv);

                    // 5. Resposta conforme especificação (pág. 22)
                    if (bcc2_error) {
                        if (is_new_frame) {
                            send_REJ(ns_received);           // Erro em novo → REJ
                        } else {
                            send_RR(expected_ns);            // Erro em duplicado → RR
                        }
                        state = START;
                        continue;
                    }

                    // BCC2 correto
                    if (!is_new_frame) {
                        send_RR(expected_ns);                // Duplicado bom → RR
                        state = START;
                        continue;
                    }

                    // Sucesso: novo frame, BCC2 OK
                    memcpy(packet, destuffed, data_len);
                    send_RR(1 - expected_ns);
                    expected_ns = 1 - expected_ns;
                    return data_len;                         // Entrega pacote à camada superior

                } else {
                    // Acumula dados stuffed
                    if (stuffed_len >= (int)sizeof(stuffed) - 1) { //evitar overflows
                        send_REJ(ns_received);
                        state = START;
                    }
                    stuffed[stuffed_len++] = byte;
                }
                break;

            default:
                state = START;
                break;
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    (void) signal(SIGALRM, alarm_handler);
    State state = START;
    unsigned char byte;
    int tries = 0;
    
    switch(role) {
        case LlTx:
            const unsigned char DISC_T[5] = {FLAG, A_T, C_DISC, A_T ^ C_DISC, FLAG};
            const unsigned char UA_T[5] = {FLAG, A_T, C_UA, A_T ^ C_UA, FLAG};
           
            
            while (tries < nretransmissions) {

                int bytesWritten = writeBytesSerialPort(DISC_T, 5);
                //error handling
                if (bytesWritten != 5) {
                    perror("writeBytesSerialPort SET");
                    closeSerialPort();
                    return -1;
                }
                
                //DISC enviado

                alarm(timeout);
                state = START;
                timeout_flag = FALSE;
                
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
                                else if(byte == C_DISC) state = C_RCV;
                                else state = START;
                                printf("A\n");
                                printf("Byte received: 0x%02X\n", byte);
                                break;


                        case C_RCV:
                                if(byte == A_R ^ C_DISC) state = BCC_RCV;
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

                if(state == STOPP) {
                    //DISC recebido
                    int bytesWritten = writeBytesSerialPort(UA_T, 5);
                    //error handling
                    if(bytesWritten != 5) {
                        perror("writeBytesSerialPort UA_T");
                        closeSerialPort();
                        return -1;  
                    }
                    //UA final enviado
                    printf("Receiver: final UA sent: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
                        UA_T[0], UA_T[1], UA_T[2], UA_T[3], UA_T[4]);
                    closeSerialPort();
                    return 0;
                }
                tries++;
            }
            printf("Transmitter: Máximo de retransmissões atingido. A forçar fecho.\n");
            closeSerialPort();
            return -1;
        
        case LlRx:
            const unsigned char DISC_R[5] = {FLAG, A_R, C_DISC, A_R ^ C_DISC, FLAG};
            state = START;

            while(state != STOPP) {

                int bytesread = readByteSerialPort(&byte);
                //error handling
                if (bytesread == -1) {
                    perror("readByteSerialPort");
                    break;
                }

                switch (state) {
                    case START:
                        if (byte == FLAG) state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (byte == A_T) state = A_RCV;
                        else if (byte != FLAG) state = START;
                        break;
                    case A_RCV:
                        if (byte == C_DISC) state = C_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case C_RCV:
                        if (byte == (A_T ^ C_DISC)) state = BCC_RCV;
                        else if (byte == FLAG) state = FLAG_RCV;
                        else state = START;
                        break;
                    case BCC_RCV:
                        if (byte == FLAG) state = STOPP;
                        else state = START;
                        break;
                    default:
                        break;
                }
            }
            //receiver recebeu DISC

            state = START;
           

            while(tries < nretransmissions) {

                int bytesWritten = writeBytesSerialPort(DISC_R, 5);
                //error handling
                if (bytesWritten == -1) {
                    perror("llclose(Rx): writeBytesSerialPort(DISC_RX)");
                    closeSerialPort();
                    return -1;
                }

                //receiver enviou DISC e aguarda UA final
                alarm(timeout);
                timeout_flag = FALSE;
                state = START;

                while(state != STOPP && timeout_flag == FALSE) {

                    int bytesRead = readByteSerialPort(&byte);
                    //error handling
                    if(bytesRead == -1) {
                        perror("readByteSerialPort");
                        break;
                    }

                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == A_T) state = A_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case A_RCV:
                            if (byte == C_UA) state = C_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case C_RCV:
                            if (byte == (A_T ^ C_UA)) state = BCC_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC_RCV:
                            if (byte == FLAG) state = STOPP;
                            else state = START;
                            break;
                        default:
                            break;
                    }
                }
                alarm(0);

                if(state == STOPP) {
                    //Rx recebeu UA final do Tx
                    closeSerialPort();
                    return 0;
                }

                //retry por timeout
                tries++;
            }

            //tentativas esgotadas
            printf("Rx: Max retransmissões. Forçar fecho.\n");
            closeSerialPort();
            return -1;
    }
    return -1;
}

