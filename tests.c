// // Cria START/END packets (nome e tamanho).
// static int build_control_packet(unsigned char *packet, int control, const char *filename, long file_size) {
//     int index = 0;
//     packet[index++] = control;

//     // Tamanho do ficheiro
//     packet[index++] = TYPE_FILESIZE; //T (Type)
//     packet[index++] = (unsigned char)sizeof(long); //L (Length)
//     memcpy(&packet[index], &file_size, sizeof(long)); // V (Value - o próprio long)
//         O memcpy copia fileSizeValueSize bytes (o valor de fileSize) para o pacote, começando na posição atual de index.
//         O memcpy é uma função de biblioteca que não altera a variável index. Ela apenas escreve na memória a partir do endereço que lhe foi dado
//     index += sizeof(long); //ajuste no indice

//     // Nome do ficheiro
//     packet[index++] = TYPE_FILENAME;
//     int name_len = strlen(filename);
//     packet[index++] = name_len;
//     memcpy(&packet[index], filename, name_len);
//     index += name_len;

//     return index;
// }

// // Cria DATA packets (número de sequência, tamanho, dados).
// static int build_data_packet(unsigned char *packet, unsigned char seq_num, const unsigned char *data, int size) {
//     packet[0] = PACKET_DATA;
//     packet[1] = seq_num;
//     packet[2] = size / 256;  // L2
//     packet[3] = size % 256;  // L1
//     memcpy(&packet[4], data, size);
//     return size + 4;
// }

// // ------------------ Transmitter ------------------

// static void transmitter(const char *serialPort, int baudRate, int nTries, int timeout, const char *filename) {
//     LinkLayer link;
//     memset(&link, 0, sizeof(link));
//     strcpy(link.serialPort, serialPort);
//     link.role = LlTx;
//     link.baudRate = baudRate;
//     link.nRetransmissions = nTries;
//     link.timeout = timeout;

//     if (llopen(link) < 0) {
//         printf("ERROR: llopen failed\n");
//         return;
//     }

//     // Abrir ficheiro
//     FILE *file = fopen(filename, "rb");
//     if (!file) {
//         perror("fopen");
//         llclose();
//         return;
//     }

//     // Obter tamanho do ficheiro
//     fseek(file, 0, SEEK_END);
//     long file_size = ftell(file);
//     fseek(file, 0, SEEK_SET);

//     printf("Sending file '%s' (%ld bytes)\n", filename, file_size);

//     // Enviar START
//     unsigned char ctrl_packet[512];
//     int ctrl_size = build_control_packet(ctrl_packet, PACKET_START, filename, file_size);
//     if (llwrite(ctrl_packet, ctrl_size) < 0) {
//         printf("ERROR: Failed to send START packet\n");
//         fclose(file);
//         llclose();
//         return;
//     }

//     // Enviar DATA
//     unsigned char data_buf[MAX_PAYLOAD_SIZE];
//     unsigned char packet_buf[MAX_PAYLOAD_SIZE + 4];
//     int bytes_read;
//     unsigned char seq = 0;
//     long total_sent = 0;

//     while ((bytes_read = fread(data_buf, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
//         int packet_size = build_data_packet(packet_buf, seq, data_buf, bytes_read);
//         if (llwrite(packet_buf, packet_size) < 0) {
//             printf("ERROR: Failed to send data packet (seq %d)\n", seq);
//             break;
//         }
//         total_sent += bytes_read;
//         printf("Sent packet %d (%d bytes) [%.1f%%]\n", seq, bytes_read, 100.0 * total_sent / file_size);
//         seq = (seq + 1) % 256;
//     }

//     // Enviar END
//     ctrl_size = build_control_packet(ctrl_packet, PACKET_END, filename, file_size);
//     llwrite(ctrl_packet, ctrl_size);

//     fclose(file);
//     llclose();

//     printf("File transmission completed.\n");
// }

// // ------------------ Receiver ------------------

// static void receiver(const char *serialPort, int baudRate, int nTries, int timeout, const char *filename) {
//     LinkLayer link;
//     memset(&link, 0, sizeof(link));
//     strcpy(link.serialPort, serialPort);
//     link.role = LlRx;
//     link.baudRate = baudRate;
//     link.nRetransmissions = nTries;
//     link.timeout = timeout;

//     if (llopen(link) < 0) {
//         printf("ERROR: llopen failed\n");
//         return;
//     }

//     unsigned char packet[65536];
//     int size = llread(packet);
//     if (size <= 0 || packet[0] != PACKET_START) {
//         printf("ERROR: Invalid START packet\n");
//         llclose();
//         return;
//     }

//     // Extrair informação do START
//     long file_size = 0;
//     char recv_filename[256] = {0};

//     for (int i = 1; i < size;) {
//         unsigned char T = packet[i++];
//         unsigned char L = packet[i++];
//         if (T == TYPE_FILESIZE) {
//             memcpy(&file_size, &packet[i], L);
//         } else if (T == TYPE_FILENAME) {
//             memcpy(recv_filename, &packet[i], L);
//             recv_filename[L] = '\0';
//         }
//         i += L;
//     }

//     printf("Receiving file '%s' (%ld bytes)\n", recv_filename, file_size);
//     FILE *out = fopen(recv_filename, "wb");
//     if (!out) {
//         perror("fopen");
//         llclose();
//         return;
//     }

//     long total_received = 0;
//     bool end_received = false;

//     while (!end_received) {
//         int pkt_size = llread(packet);
//         if (pkt_size <= 0) continue;

//         if (packet[0] == PACKET_DATA) {
//             int data_len = packet[2] * 256 + packet[3];
//             fwrite(&packet[4], 1, data_len, out);
//             total_received += data_len;
//             printf("Received data packet (%d bytes) [%.1f%%]\n", data_len, 100.0 * total_received / file_size);
//         } else if (packet[0] == PACKET_END) {
//             end_received = true;
//         }
//     }

//     fclose(out);
//     llclose();
//     printf("File reception completed.\n");
// }


// void applicationLayer(const char *serialPort, const char *role, int baudRate,
//                       int nTries, int timeout, const char *filename)
// {
//     if (strcmp(role, "tx") == 0)
//         transmitter(serialPort, baudRate, nTries, timeout, filename);
//     else if (strcmp(role, "rx") == 0)
//         receiver(serialPort, baudRate, nTries, timeout, filename);
//     else
//         printf("Invalid role: %s\n", role);

// }
