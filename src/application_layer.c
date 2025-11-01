// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>    
#include <stdlib.h>    
#include <sys/stat.h>  

#define C_START 1
#define C_DATA 2
#define C_END 3
#define T_FILE_SIZE 0
#define T_FILE_NAME 1

void parseControlPacket(unsigned char *packet, int size, long *fileSize, char **filename)
{
    *filename = NULL;
    *fileSize = 0;    
    int pos = 1; 
    
    while (pos < size)
    {
        unsigned char T = packet[pos++]; 
        if (pos >= size) break; 
        unsigned char L = packet[pos++]; 
        if (pos + L > size) break; 

        if (T == T_FILE_SIZE)
        {
            if (L == sizeof(long)) {
                memcpy(fileSize, &packet[pos], L);
            } else {
                 fprintf(stderr, "Warning: Received file size with unexpected length L=%d\n", L);
                 if (L <= sizeof(long)) {
                     memcpy(fileSize, &packet[pos], L < sizeof(long) ? L : sizeof(long));
                 }
            }
        }
        else if (T == T_FILE_NAME)
        {
            if (*filename != NULL) {
                free(*filename);
                *filename = NULL;
            }
            *filename = (char *)malloc(L + 1);
            if (*filename != NULL) {
                memcpy(*filename, &packet[pos], L);
                (*filename)[L] = '\0';
            } else {
                 fprintf(stderr, "Error: Failed to allocate memory for filename in parseControlPacket\n");
            }
        } else {
             fprintf(stderr, "Warning: Unknown TLV Type T=%d in control packet\n", T);
        }
        pos += L; 
    }
}

unsigned char *buildControlPacket(int controlField, const char *filename, long fileSize, int *packetSize){
    *packetSize = 5 + strlen(filename) + sizeof(long);
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) return NULL;
    int index = 0;
    packet[index++] = (unsigned char)controlField; 
    packet[index++] = T_FILE_SIZE; 
    packet[index++] = sizeof(long); 
    memcpy(&packet[index], &fileSize, sizeof(long)); 
    index += sizeof(long);

    int filenameValueSize = strlen(filename);
    packet[index++] = T_FILE_NAME; 
    packet[index++] = (unsigned char)filenameValueSize; 
    memcpy(&packet[index], filename, filenameValueSize); 
    index += filenameValueSize;

    return packet;
}

unsigned char *buildDataPacket(const unsigned char *data, int dataSize, int *packetSize) {
    *packetSize = 1 + 2 + dataSize; 
    
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) return NULL;

    int index = 0;
    packet[index++] = C_DATA; 
 
    packet[index++] = (unsigned char)(dataSize / 256); // L2
    packet[index++] = (unsigned char)(dataSize % 256); // L1

    memcpy(&packet[index], data, dataSize);
    return packet;
}


void handleTransmission(const char *filename)
{
    FILE *file = fopen(filename, "rb"); 
    if (file == NULL) {
        perror("Application(Tx): Failed to open file");
        llclose(); 
        exit(-1);
    }

    struct stat st;
    if (stat(filename, &st) != 0) {
        perror("Application(Tx): Failed to get file stats");
        fclose(file);
        llclose();
        exit(-1);
    }
    long fileSize = st.st_size;
    printf("Application(Tx): Opened file '%s' (%ld bytes).\n", filename, fileSize);

    int startPacketSize;
    unsigned char *startPacket = buildControlPacket(C_START, filename, fileSize, &startPacketSize);
    if (startPacket == NULL) {
        fprintf(stderr, "Application(Tx): Malloc failed for START packet.\n");
        fclose(file);
        llclose();
        exit(-1);
    }
    printf("Application(Tx): Sending START packet (%d bytes)...\n", startPacketSize);
    if (llwrite(startPacket, startPacketSize) < 0) {
        fprintf(stderr, "Application(Tx): llwrite(START) failed. Exiting.\n");
        free(startPacket);
        fclose(file);
        llclose();
        exit(-1);
    }
    free(startPacket); 

    printf("Application(Tx): Sending DATA packets...\n");
    int dataChunkSize = MAX_PAYLOAD_SIZE - 3; 
    unsigned char *dataChunk = (unsigned char *)malloc(dataChunkSize);
     if (dataChunk == NULL) {
        fprintf(stderr, "Application(Tx): Malloc failed for data chunk.\n");
        fclose(file);
        llclose();
        exit(-1);
    }
    int bytesRead;
    while ((bytesRead = fread(dataChunk, 1, dataChunkSize, file)) > 0)
    {
        int dataPacketSize;
        unsigned char *dataPacket = buildDataPacket(dataChunk, bytesRead, &dataPacketSize);
        if (dataPacket == NULL) {
            fprintf(stderr, "Application(Tx): Malloc failed for DATA packet.\n");
            break; 
        }
        printf("."); 
        fflush(stdout); 
        if (llwrite(dataPacket, dataPacketSize) < 0) {
            fprintf(stderr, "\nApplication(Tx): llwrite(DATA) failed. Exiting.\n");
            free(dataPacket);
            free(dataChunk);
            fclose(file);
            return;
        }
        free(dataPacket); 
    }
    free(dataChunk); 
    printf("\nApplication(Tx): Finished sending DATA packets.\n");

    int endPacketSize;
    unsigned char *endPacket = buildControlPacket(C_END, filename, fileSize, &endPacketSize);
     if (endPacket == NULL) {
         fprintf(stderr, "Application(Tx): Malloc failed for END packet.\n");
         fclose(file);
         llclose();
         exit(-1);
     }
    printf("Application(Tx): Sending END packet (%d bytes)...\n", endPacketSize);
    if (llwrite(endPacket, endPacketSize) < 0) {
        fprintf(stderr, "Application(Tx): llwrite(END) failed.\n"); 
        free(endPacket);
        fclose(file);
        return;
    }
    free(endPacket); 
    fclose(file);
}

void handleReception(const char *filename) 
{
    FILE *newFile = fopen(filename, "wb"); 
    if (!newFile) {
        perror("Application(Rx): Error opening output file");
        llclose();
        exit(-1);
    }

    unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
    if (!packet) {
        fprintf(stderr, "Application(Rx): Malloc failed for packet buffer.\n");
        fclose(newFile);
        llclose();
        exit(-1);
    }

    long receivedFileSize = 0; 
    char *receivedFilename = NULL; 

    printf("Application(Rx): Waiting for packets...\n");
    while (true) { 
        
        int packetSize = llread(packet); 
        
        if (packetSize < 0) { 
            fprintf(stderr, "\nApplication(Rx): llread failed. Stopping.\n");
            break;
        }
        
        if (packetSize == 0) { 
             fprintf(stderr, "\nApplication(Rx): llread returned 0 bytes. Stopping.\n");
             break;
        }

 
        if (packet[0] == C_START) {
            printf("\nApplication(Rx): START packet received.\n");
           
            if (receivedFilename) free(receivedFilename); 
            parseControlPacket(packet, packetSize, &receivedFileSize, &receivedFilename);
            if (receivedFilename) {
                 printf("   - Receiving file (info): %s (%ld bytes)\n", receivedFilename, receivedFileSize);
            }
        } else if (packet[0] == C_DATA) {
            int dataSize = (packet[1] << 8) | packet[2]; 
            if (dataSize != packetSize - 3) {
                 fprintf(stderr, "\nWarning: Data packet size mismatch (L2L1=%d vs packetSize=%d)\n", dataSize, packetSize);
                 dataSize = (packetSize - 3 < dataSize) ? (packetSize - 3) : dataSize;
                 if (dataSize < 0) dataSize = 0; 
            }
            fwrite(packet + 3, sizeof(unsigned char), dataSize, newFile);
            printf("."); 
            fflush(stdout);
        } else if (packet[0] == C_END) {
            printf("\nApplication(Rx): END packet received. Finishing.\n");
            break;
        } else {
             fprintf(stderr, "\nApplication(Rx): Received unknown packet type (C=0x%02X). Discarding.\n", packet[0]);
        }
    }

    if (receivedFilename) free(receivedFilename);
    free(packet);
    fclose(newFile);
    printf("Application(Rx): File reception finished.\n");
}


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (strcmp(role, "tx") == 0) {
        connectionParameters.role = LlTx;
    } else if (strcmp(role, "rx") == 0) {
        connectionParameters.role = LlRx;
    } else {
        printf("Application: Invalid role ('tx' or 'rx').\n");
        exit(-1);
    }

    printf("Application: Calling llopen()...\n");
    if (llopen(connectionParameters) < 0) {
        fprintf(stderr, "Application: llopen() failed. Exiting.\n");
        exit(-1);
    }
    printf("Application: llopen() successful.\n");

    if (connectionParameters.role == LlTx) {
        handleTransmission(filename);
    } else {
        handleReception(filename); 
    }

    printf("Application: Calling llclose()...\n");
    if (llclose() < 0) { 
        fprintf(stderr, "Application: llclose() failed.\n");
    } else {
        printf("Application: llclose() successful.\n");
    }
    printf("Application: Finished.\n");
}
