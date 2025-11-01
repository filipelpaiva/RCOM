// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>    // Para strlen, memcpy, strcpy, strcmp
#include <stdlib.h>    // Para malloc, free, exit
#include <sys/stat.h>  // Para struct stat, stat()


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

void parseControlPacket(unsigned char *packet, int size, long *fileSize, char **filename)
{
    *filename = NULL; // Inicializar para evitar lixo
    *fileSize = 0;    // Inicializar
    int pos = 1; // Posição 0 é o 'C'
    
    while (pos < size)
    {
        unsigned char T = packet[pos++]; // Type
        // Verificar se ainda há espaço para L
        if (pos >= size) break; 
        unsigned char L = packet[pos++]; // Length
        // Verificar se o valor V cabe no pacote
        if (pos + L > size) break; 

        if (T == T_FILE_SIZE)
        {
            // Garantir que L corresponde ao que esperamos (sizeof(long))
            if (L == sizeof(long)) {
                memcpy(fileSize, &packet[pos], L);
            } else {
                 fprintf(stderr, "Warning: Received file size with unexpected length L=%d\n", L);
                 // Tentar ler mesmo assim, mas pode dar erro se L for > sizeof(long)
                 if (L <= sizeof(long)) {
                     // Segurança extra: copiar no máximo sizeof(long) bytes
                     memcpy(fileSize, &packet[pos], L < sizeof(long) ? L : sizeof(long));
                     // Se L < sizeof(long), os bytes restantes de fileSize podem ficar com lixo,
                     // mas é melhor que ler fora do pacote. Idealmente, L==sizeof(long).
                 }
            }
        }
        else if (T == T_FILE_NAME)
        {
            // Libertar filename anterior se já tiver sido alocado neste pacote (improvável, mas seguro)
            if (*filename != NULL) {
                free(*filename);
                *filename = NULL;
            }
            *filename = (char *)malloc(L + 1);
            if (*filename != NULL) {
                memcpy(*filename, &packet[pos], L);
                (*filename)[L] = '\0'; // Terminar a string
            } else {
                 fprintf(stderr, "Error: Failed to allocate memory for filename in parseControlPacket\n");
                 // Continuar a processar o resto do pacote, se houver
            }
        } else {
             fprintf(stderr, "Warning: Unknown TLV Type T=%d in control packet\n", T);
        }
        pos += L; // Avançar para o próximo TLV
    }
}

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

    // 1. Enviar Pacote START (usando a sua função)
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
    free(startPacket); // Libertar memória após envio

    // 2. Enviar Pacotes DATA (lendo em chunks - MELHORIA vs Solução 2)
    printf("Application(Tx): Sending DATA packets...\n");
    // MAX_PAYLOAD_SIZE é o tamanho MÁXIMO do PACOTE.
    // O chunk lido do ficheiro deve ser mais pequeno para caber C+L2+L1.
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
        // Usar a sua função para criar o pacote
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
        free(dataPacket); // Libertar memória após envio
    }
    free(dataChunk); // Libertar buffer de leitura
    printf("\nApplication(Tx): Finished sending DATA packets.\n");

    // 3. Enviar Pacote END (usando a sua função)
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
        fprintf(stderr, "Application(Tx): llwrite(END) failed.\n"); // Não precisa de Exiting aqui
        free(endPacket);
        fclose(file);
        return;
    }
    free(endPacket); // Libertar memória após envio
    fclose(file);
}

/**
 * @brief Lógica principal do Recetor (Rx). Adaptada da Solução 2.
 */
void handleReception(const char *filename) // filename aqui é o nome do ficheiro *de SAÍDA*
{
    // Abrir ficheiro de saída logo no início
    FILE *newFile = fopen(filename, "wb"); 
    if (!newFile) {
        perror("Application(Rx): Error opening output file");
        llclose();
        exit(-1);
    }

    // Alocar buffer UMA VEZ fora do loop (MELHORIA vs Solução 2)
    unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
    if (!packet) {
        fprintf(stderr, "Application(Rx): Malloc failed for packet buffer.\n");
        fclose(newFile);
        llclose();
        exit(-1);
    }

    long receivedFileSize = 0; // Para referência
    char *receivedFilename = NULL; // Para referência

    printf("Application(Rx): Waiting for packets...\n");
    while (true) { // Loop infinito até receber END ou erro
        
        int packetSize = llread(packet); // Chama llread para preencher o buffer 'packet'
        
        if (packetSize < 0) { // Erro em llread
            fprintf(stderr, "\nApplication(Rx): llread failed. Stopping.\n");
            break;
        }
        
        if (packetSize == 0) { // Pode acontecer se llread tiver um bug ou timeout interno não tratado
             fprintf(stderr, "\nApplication(Rx): llread returned 0 bytes. Stopping.\n");
             break;
        }

        // Analisar o pacote recebido
        if (packet[0] == C_START) {
            printf("\nApplication(Rx): START packet received.\n");
            // Usar parseControlPacket para extrair info
            if (receivedFilename) free(receivedFilename); // Libertar anterior, se houver
            parseControlPacket(packet, packetSize, &receivedFileSize, &receivedFilename);
            if (receivedFilename) {
                 printf("   - Receiving file (info): %s (%ld bytes)\n", receivedFilename, receivedFileSize);
            }
        } else if (packet[0] == C_DATA) {
            // Calcular tamanho dos dados (K = L2*256 + L1)
            int dataSize = (packet[1] << 8) | packet[2]; // Mais eficiente
            // Verificar se o tamanho reportado faz sentido com o tamanho recebido
            if (dataSize != packetSize - 3) {
                 fprintf(stderr, "\nWarning: Data packet size mismatch (L2L1=%d vs packetSize=%d)\n", dataSize, packetSize);
                 // Tentar usar o menor dos dois para evitar ler lixo ou escrever a menos
                 dataSize = (packetSize - 3 < dataSize) ? (packetSize - 3) : dataSize;
                 if (dataSize < 0) dataSize = 0; // Segurança extra
            }
            // Escrever os dados (a partir de packet[3])
            fwrite(packet + 3, sizeof(unsigned char), dataSize, newFile);
            printf("."); 
            fflush(stdout);
        } else if (packet[0] == C_END) {
            printf("\nApplication(Rx): END packet received. Finishing.\n");
            break; // Sair do loop while(true)
        } else {
             fprintf(stderr, "\nApplication(Rx): Received unknown packet type (C=0x%02X). Discarding.\n", packet[0]);
        }
    }

    // Libertar memória e fechar ficheiro
    if (receivedFilename) free(receivedFilename);
    free(packet);
    fclose(newFile);
    printf("Application(Rx): File reception finished.\n");
}


/**
 * @brief Função principal da Camada de Aplicação.
 */
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
        // No Rx, o 'filename' da linha de comando é o NOME DO FICHEIRO DE SAÍDA
        handleReception(filename); 
    }

    printf("Application: Calling llclose()...\n");
    // Usar llclose() sem argumento, conforme link_layer.h
    if (llclose() < 0) { 
        fprintf(stderr, "Application: llclose() failed.\n");
    } else {
        printf("Application: llclose() successful.\n");
    }
    printf("Application: Finished.\n");
}
