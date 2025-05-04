#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <arpa/inet.h>
#include <stdbool.h>

#include "zmodlib/Zmod/zmod.h"
#include "zmodlib/ZmodDAC1411/zmoddac1411.h"

#define PORT 8081
#define BUFFER_SIZE 8192
#define MAX_SAMPLES 16383  // (1<<14) - 1, maximum buffer size

#define TRANSFER_LEN    0x400
#define IIC_BASE_ADDR   0xE0005000
#define ZMOD_IRQ        61
#define DAC_BASE_ADDR       0x43C10000
#define DAC_DMA_BASE_ADDR   0x40410000
#define DAC_FLASH_ADDR      0x31
#define DAC_DMA_IRQ         63

// Global ZMOD DAC object to be shared across functions
ZMODDAC1411* g_dacZmod = NULL;

/*
 * Generate DAC waveforms from complex data and output to both channels
 * @param realData - Array of real part data
 * @param imagData - Array of imaginary part data
 * @param numSamples - Number of samples
 * @param frequencyDivider - Output frequency divider
 * @param gain - Channel gain
 */
void dacGenerateFromComplex(float* realData, float* imagData, int numSamples, 
                           uint8_t frequencyDivider, uint8_t gain)
{
    if (!g_dacZmod) {
        std::cerr << "DAC not initialized!" << std::endl;
        return;
    }
    
    // Ensure sample count doesn't exceed maximum buffer size
    if (numSamples > MAX_SAMPLES) {
        std::cout << "Truncating samples from " << numSamples << " to " << MAX_SAMPLES << std::endl;
        numSamples = MAX_SAMPLES;
    }
    
    size_t length = numSamples;
    uint32_t *buf = g_dacZmod->allocChannelsBuffer(length);
    if (!buf) {
        std::cerr << "Failed to allocate DMA buffer!" << std::endl;
        return;
    }
    
    // Set frequency divider for both channels
    g_dacZmod->setOutputSampleFrequencyDivider(frequencyDivider);
    
    // Set gain for both channels
    g_dacZmod->setGain(0, gain); // Channel 1 (real)
    g_dacZmod->setGain(1, gain); // Channel 2 (imaginary)
    
    // Prepare data for both channels
    // Channel 1 (0) for real part, Channel 2 (1) for imaginary part
    for (int i = 0; i < numSamples; i++) {
        // Process real part for Channel 1
        int16_t realRaw = g_dacZmod->getSignedRawFromVolt(realData[i], gain);
        
        // Process imaginary part for Channel 2
        int16_t imagRaw = g_dacZmod->getSignedRawFromVolt(imagData[i], gain);
        
        // Combine both channels' data into a single 32-bit word
        // Each channel gets its own position in the 32-bit value
        uint32_t realValBuf = g_dacZmod->arrangeChannelData(0, realRaw);  // Channel 1
        uint32_t imagValBuf = g_dacZmod->arrangeChannelData(1, imagRaw);  // Channel 2
        
        // Combine both channel values
        buf[i] = realValBuf | imagValBuf;
        
        // Print some debug info for the first and last few samples
        if (i < 5 || i > numSamples - 5) {
            std::cout << "Sample[" << i << "]: real=" << realData[i] 
                      << ", imag=" << imagData[i]
                      << ", real_raw=" << realRaw 
                      << ", imag_raw=" << imagRaw << std::endl;
        }
    }
    
    // Send data to DAC and start output
    g_dacZmod->setData(buf, length);
    g_dacZmod->start();
    
    // Free the buffer after transmission is set up
    g_dacZmod->freeChannelsBuffer(buf, length);
}

// Helper function to receive exactly the specified number of bytes
bool receiveExactBytes(int socket, void* buffer, size_t length) {
    size_t totalReceived = 0;
    char* buf = (char*)buffer;
    
    while (totalReceived < length) {
        ssize_t received = recv(socket, buf + totalReceived, length - totalReceived, 0);
        if (received <= 0) {
            return false;
        }
        totalReceived += received;
    }
    return true;
}

int main() {
    std::cout << "ZmodDAC1411 TCP Server\n";

    // Initialize DAC
    g_dacZmod = new ZMODDAC1411(DAC_BASE_ADDR, DAC_DMA_BASE_ADDR, IIC_BASE_ADDR, DAC_FLASH_ADDR, DAC_DMA_IRQ);
    if (!g_dacZmod) {
        std::cerr << "Failed to initialize DAC!" << std::endl;
        return 1;
    }

    // Create TCP server
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];
    
    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        delete g_dacZmod;
        return 1;
    }

    // Define server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    // Bind socket to port
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        delete g_dacZmod;
        return 1;
    }

    // Listen for connections
    if (listen(server_fd, 1) < 0) {
        perror("Listen failed");
        delete g_dacZmod;
        return 1;
    }

    printf("Server listening on port %d...\n", PORT);

    // Accept client connection
    if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len)) < 0) {
        perror("Accept failed");
        delete g_dacZmod;
        return 1;
    }

    printf("Client connected. Awaiting commands...\n");

    bool transmission = false;
    
    while (1) {
        memset(buffer, 0, BUFFER_SIZE);
        ssize_t bytes_read = recv(client_fd, buffer, BUFFER_SIZE, 0);
        
        if (bytes_read > 0) {
            printf("Received command: %s\n", buffer);
            
            if (strcmp(buffer, "transmit") == 0) {
                transmission = true;
                
                // Reply to client
                snprintf(buffer, BUFFER_SIZE, "Ready for data");
                if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                    perror("Send failed");
                    break;
                }
                
                printf("Waiting for data length...\n");
                
                // First receive the data length as int32
                int32_t dataLength = 0;
                if (!receiveExactBytes(client_fd, &dataLength, sizeof(int32_t))) {
                    perror("Failed to receive data length");
                    break;
                }
                
                printf("Received data length: %d samples\n", dataLength);
                
                // Limit sample count to maximum buffer size
                if (dataLength > MAX_SAMPLES) {
                    printf("Limiting samples from %d to %d\n", dataLength, MAX_SAMPLES);
                    dataLength = MAX_SAMPLES;
                }
                
                // Allocate buffers for real and imaginary parts
                float* imagPart = new float[dataLength];
                float* realPart = new float[dataLength];
                
                // Receive imaginary part
                printf("Receiving imaginary part...\n");
                if (!receiveExactBytes(client_fd, imagPart, dataLength * sizeof(float))) {
                    perror("Failed to receive imaginary data");
                    delete[] imagPart;
                    delete[] realPart;
                    break;
                }
                
                // Receive real part
                printf("Receiving real part...\n");
                if (!receiveExactBytes(client_fd, realPart, dataLength * sizeof(float))) {
                    perror("Failed to receive real data");
                    delete[] imagPart;
                    delete[] realPart;
                    break;
                }
                
                // Print the first few samples for debugging
                for (int i = 0; i < 5 && i < dataLength; i++) {
                    printf("Sample[%d]: real = %f, imaginary = %f\n", 
                           i, realPart[i], imagPart[i]);
                }
                
                // Generate DAC waveform
                // Using frequency divider 2 and high gain
                dacGenerateFromComplex(realPart, imagPart, dataLength, 2, 1);
                
                // Free the allocated memory
                delete[] realPart;
                delete[] imagPart;
                
                // Confirm to client
                snprintf(buffer, BUFFER_SIZE, "Transmission started with %d samples", dataLength);
                if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                    perror("Send confirmation failed");
                }
                
            } else if (strcmp(buffer, "stop") == 0) {
                if (!transmission) {
                    snprintf(buffer, BUFFER_SIZE, "Error: Not transmitting");
                    if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                        perror("Send failed");
                        break;
                    }
                } else {
                    // Stop DAC output
                    g_dacZmod->stop();
                    
                    snprintf(buffer, BUFFER_SIZE, "Transmission stopped");
                    if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                        perror("Send failed");
                        break;
                    }
                    
                    printf("Transmission stopped\n");
                    transmission = false;
                }
            } else {
                snprintf(buffer, BUFFER_SIZE, "Unknown command");
                if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                    perror("Send failed");
                    break;
                }
                printf("Unknown command: %s\n", buffer);
            }
            
        } else if (bytes_read == 0) {
            printf("Client disconnected.\n");
            break;
        } else {
            perror("Receive failed");
            break;
        }
    }

    // Clean up resources
    close(client_fd);
    close(server_fd);
    
    // Stop DAC and release
    if (g_dacZmod) {
        g_dacZmod->stop();
        delete g_dacZmod;
    }
    
    return 0;
}
