#include <arpa/inet.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <signal.h>
#include <sys/time.h>
// 添加netinet/tcp.h以支持TCP_NODELAY
#include <netinet/tcp.h>

#include "zmodlib/Zmod/zmod.h"
#include "zmodlib/ZmodADC1410/zmodadc1410.h"

#define PORT 8080
#define BUFFER_SIZE 8192
#define TRANSFER_LEN 0x400  // ADC buffer size (1024 samples)
#define IIC_BASE_ADDR 0xE0005000
#define ZMOD_IRQ 61

#define ADC_BASE_ADDR 0x43C00000
#define ADC_DMA_BASE_ADDR 0x40400000
#define ADC_FLASH_ADDR 0x30
#define ADC_DMA_IRQ 62

// Global flag for handling termination
volatile bool running = true;

// Signal handler for graceful termination
void sig_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        printf("Received terminate signal, shutting down...\n");
        running = false;
    }
}

// Function to get current time in milliseconds
uint64_t getCurrentTimeMs() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
}

// Function to acquire ADC data and format as string
std::string acquireADCData(ZMODADC1410 &adcZmod, uint32_t *buffer, uint8_t channel, uint8_t gain, size_t length) {
    std::stringstream dataStream;
    
    // Use immediate acquisition for higher speed
    adcZmod.acquireImmediatePolling(buffer, length);
    
    // Format data for transmission
    char val_formatted[15];
    char time_formatted[15];
    int16_t valCh;
    float val;
    
    // Current timestamp in ms
    uint64_t currentTime = getCurrentTimeMs();
    
    // Add timestamp to first line
    dataStream << "TIME:" << currentTime << "\n";
    
    // Format data as CSV: Voltage,Time
    for (size_t i = 0; i < length; i++) {
        valCh = adcZmod.signedChannelData(channel, buffer[i]);
        val = adcZmod.getVoltFromSignedRaw(valCh, gain);
        
        // Format time with higher precision
        if (i < 100) {
            adcZmod.formatValue(time_formatted, i*10, "ns");
        } else {
            adcZmod.formatValue(time_formatted, (float)(i)/100.0, "us");
        }
        
        // Format voltage (in mV)
        adcZmod.formatValue(val_formatted, 1000.0*val, "mV");
        
        // Add to data stream
        dataStream << val_formatted << "," << time_formatted << "\n";
    }
    
    return dataStream.str();
}

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];
    
    // Setup signal handler
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    
    // Initialize ZMOD ADC
    std::cout << "Initializing ZmodADC1410..." << std::endl;
    ZMODADC1410 adcZmod(ADC_BASE_ADDR, ADC_DMA_BASE_ADDR, IIC_BASE_ADDR, ADC_FLASH_ADDR,
                        ZMOD_IRQ, ADC_DMA_IRQ);
    
    // Set ADC gain for channel 0 (CH1)
    adcZmod.setGain(0, 0); // 0 for LOW gain
    
    // Pre-allocate a fixed DMA buffer
    size_t bufferLength = TRANSFER_LEN;
    uint32_t *adcBuffer = adcZmod.allocChannelsBuffer(bufferLength);
    
    if (!adcBuffer) {
        std::cerr << "Failed to allocate DMA buffer" << std::endl;
        return 1;
    }
    
    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        adcZmod.freeChannelsBuffer(adcBuffer, bufferLength);
        return 1;
    }
    
    // Enable socket reuse
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
        adcZmod.freeChannelsBuffer(adcBuffer, bufferLength);
        return 1;
    }
    
    // 尝试设置 TCP_NODELAY，但如果失败则继续
    // 某些嵌入式系统可能不支持此选项
    #ifdef TCP_NODELAY
    if (setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(TCP_NODELAY) failed");
        // Non-fatal, continue
    }
    #endif
    
    // Define server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);
    
    // Bind socket to port
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        adcZmod.freeChannelsBuffer(adcBuffer, bufferLength);
        return 1;
    }
    
    // Listen for connections
    if (listen(server_fd, 1) < 0) {
        perror("Listen failed");
        adcZmod.freeChannelsBuffer(adcBuffer, bufferLength);
        return 1;
    }
    
    printf("Server listening on port %d...\n", PORT);
    
    while (running) {
        // Accept client connection
        if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len)) < 0) {
            perror("Accept failed");
            continue;
        }
        
        printf("Client connected from %s:%d\n", 
               inet_ntoa(client_addr.sin_addr), 
               ntohs(client_addr.sin_port));
        
        // 尝试为客户端套接字设置 TCP_NODELAY
        #ifdef TCP_NODELAY
        if (setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
            perror("Client socket setsockopt(TCP_NODELAY) failed");
            // Non-fatal, continue
        }
        #endif
        
        bool streaming = false;
        
        while (running) {
            // Check for client commands
            fd_set readfds;
            struct timeval tv;
            FD_ZERO(&readfds);
            FD_SET(client_fd, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 0;
            
            if (select(client_fd + 1, &readfds, NULL, NULL, &tv) > 0) {
                // Command available
                memset(buffer, 0, BUFFER_SIZE);
                ssize_t bytes_read = recv(client_fd, buffer, BUFFER_SIZE, 0);
                
                if (bytes_read > 0) {
                    printf("Received command: %s\n", buffer);
                    
                    if (strcmp(buffer, "stream") == 0) {
                        // Start streaming mode
                        streaming = true;
                        printf("Starting streaming mode...\n");
                        
                        // Send acknowledgment
                        snprintf(buffer, BUFFER_SIZE, "Streaming mode started");
                        if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                            perror("Send acknowledgment failed");
                            break;
                        }
                    }
                    else if (strcmp(buffer, "stop") == 0) {
                        // Stop streaming mode
                        streaming = false;
                        printf("Streaming mode stopped\n");
                        
                        // Send acknowledgment
                        snprintf(buffer, BUFFER_SIZE, "Streaming mode stopped");
                        if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                            perror("Send acknowledgment failed");
                            break;
                        }
                    }
                    else {
                        // Unknown command
                        snprintf(buffer, BUFFER_SIZE, "Unknown command");
                        if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                            perror("Send failed");
                            break;
                        }
                    }
                }
                else if (bytes_read == 0) {
                    // Client disconnected
                    printf("Client disconnected\n");
                    break;
                }
                else {
                    // Error receiving data
                    perror("Receive failed");
                    break;
                }
            }
            
            // Streaming mode: continuously send data
            if (streaming) {
                // Get ADC data
                std::string adcData = acquireADCData(adcZmod, adcBuffer, 0, 0, bufferLength);
                
                // Send data length first
                snprintf(buffer, BUFFER_SIZE, "%zu", adcData.length());
                if (send(client_fd, buffer, strlen(buffer), 0) < 0) {
                    perror("Send data length failed");
                    break;
                }
                
                // Wait for acknowledgment
                memset(buffer, 0, BUFFER_SIZE);
                
                fd_set ackfds;
                struct timeval ack_tv;
                FD_ZERO(&ackfds);
                FD_SET(client_fd, &ackfds);
                ack_tv.tv_sec = 0;
                ack_tv.tv_usec = 100000; // 100ms timeout
                
                if (select(client_fd + 1, &ackfds, NULL, NULL, &ack_tv) > 0) {
                    if (recv(client_fd, buffer, BUFFER_SIZE, 0) <= 0) {
                        perror("Receive acknowledgment failed");
                        break;
                    }
                }
                else {
                    // Timeout waiting for acknowledgment
                    // Skip this iteration and try again
                    usleep(5000); // 5ms delay
                    continue;
                }
                
                // Send actual data
                if (send(client_fd, adcData.c_str(), adcData.length(), 0) < 0) {
                    perror("Send data failed");
                    break;
                }
                
                // Small delay to prevent overwhelming the network
                usleep(5000); // 5ms delay
            }
            else {
                // Not streaming, sleep to prevent busy loop
                usleep(100000); // 100ms delay
            }
        }
        
        close(client_fd);
        printf("Connection closed. Waiting for new connections...\n");
    }
    
    // Clean up
    adcZmod.freeChannelsBuffer(adcBuffer, bufferLength);
    
    close(server_fd);
    printf("Server shutdown complete\n");
    return 0;
}
