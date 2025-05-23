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

#define PORT 8080
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
        if (i < 10 || i > numSamples - 10) {
            std::cout << "Sample[" << i << "]: real=" << realData[i] 
                      << ", imag=" << imagData[i]
                      << ", real_raw=" << realRaw 
                      << ", imag_raw=" << imagRaw << std::endl;
        }
    }
    
    // Send data to DAC and start output
    g_dacZmod->setData(buf, length);
    g_dacZmod->start();
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
    float realPart[BUFFER_SIZE];
    float imaginaryPart[BUFFER_SIZE];
    
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
                
                printf("Waiting for complex data...\n");
                
                // Receive imaginary part first (matches MATLAB client order)
                memset(imaginaryPart, 0, BUFFER_SIZE);
                ssize_t imaginary_read = recv(client_fd, imaginaryPart, BUFFER_SIZE, 0);
                
                int num_imaginary = imaginary_read / sizeof(float);
                printf("Received %d imaginary samples\n", num_imaginary);
                
                // Then receive real part
                memset(realPart, 0, BUFFER_SIZE);
                ssize_t real_read = recv(client_fd, realPart, BUFFER_SIZE, 0);
                
                int num_real = real_read / sizeof(float);
                printf("Received %d real samples\n", num_real);
                
                // Ensure both parts have the same number of samples
                int num_samples = (num_real < num_imaginary) ? num_real : num_imaginary;
                
                if (num_samples > 0) {
                    // Print the first few samples for debugging
                    for (int i = 0; i < 5 && i < num_samples; i++) {
                        printf("Sample[%d]: real = %f, imaginary = %f\n", 
                               i, realPart[i], imaginaryPart[i]);
                    }
                    
                    // Generate DAC waveform on both channels
                    // Channel 1 (0) for real part, Channel 2 (1) for imaginary part
                    // Using frequency divider 2 and high gain
                    dacGenerateFromComplex(realPart, imaginaryPart, num_samples, 2, 1);
                    
                    // Confirm to client
                    snprintf(buffer, BUFFER_SIZE, "Transmission started with %d samples", num_samples);
                    if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                        perror("Send confirmation failed");
                    }
                } else {
                    snprintf(buffer, BUFFER_SIZE, "Error: No valid data received");
                    if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                        perror("Send error failed");
                    }
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
