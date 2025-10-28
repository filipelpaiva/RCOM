// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>


// #define PACKET_DATA 2
// #define PACKET_START 1
// #define PACKET_END 3
// #define TYPE_FILESIZE 0
// #define TYPE_FILENAME 1
// #define MAX_PAYLOAD_SIZE 256

#define C_START 1
#define C_DATA 2
#define C_END 3
#define T_FILE_SIZE 0
#define T_FILE_NAME 1

unsigned char *buildControlPacket(int controlField, const char *filename, long fileSize, int *packetSize){
    *packetSize = 5 + strlen(filename) + sizeof(long);
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) return NULL;
    // 1. Calcular o tamanho necessário: C (1 byte) + (Tipo1 + Tamanho1 + Valor1) + (Tipo2 + Tamanho2 + Valor2)
    int index = 0;
    packet[index++] = (unsigned char)controlField; // 1 para START, 3 para END
    packet[index++] = T_FILE_SIZE; //T(type) 
    packet[index++] = sizeof(long); 
    memcpy(&packet[index], &fileSize, sizeof(long)); //V(value) //quantos bytes o long ocupa
    index += sizeof(long);

    int filenameValueSize = strlen(filename);
    packet[index++] = T_FILE_NAME; // T (Type)
    packet[index++] = (unsigned char)filenameValueSize; // L (Length)
    memcpy(&packet[index], filename, filenameValueSize); // V (Value - a string do nome)
    index += filenameValueSize;

    return packet;
}

unsigned char *buildDataPacket(const unsigned char *data, int dataSize, int *packetSize) {
    // Estrutura do Pacote de Dados: C (1) + L2 (1) + L1 (1) + Dados (K)
    // K = dataSize
    *packetSize = 1 + 2 + dataSize; // C (1 byte) + L2 L1 (2 bytes) + Dados (K bytes)
    
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) return NULL;

    int index = 0;
    packet[index++] = C_DATA; 

    // 2. Campo de Tamanho (L2 L1) - 2 octetos para o tamanho dos dados 
    // O tamanho K (dataSize) é K = 256*L2 + L1
    // L2 é o byte mais significativo, L1 o byte menos significativo
    packet[index++] = (unsigned char)(dataSize / 256); // L2
    packet[index++] = (unsigned char)(dataSize % 256); // L1

    // 3. Campo de Dados (P1...PK)
    memcpy(&packet[index], data, dataSize);
    return packet;
}


// void applicationLayer(const char *serialPort, const char *role, int baudRate,
//                       int nTries, int timeout, const char *filename)
// {

// }
