#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <fstream>
#include <iostream>
#include <vector>

// Include ZMOD library
#include "zmodlib/Zmod/zmod.h"
#include "zmodlib/ZmodDAC1411/zmoddac1411.h"
#include "zmodlib/ZmodADC1410/zmodadc1410.h"

// Configuration constants
#define SERVER_PORT 8080
#define BUFFER_SIZE 8192
#define MAX_SAMPLES 16384  // Maximum number of samples

// Hardware clock configuration
#define BASE_CLOCK 100000000  // 100 MHz base clock
#define DAC_SAMPLING_RATE 100000000  // 100 MHz DAC sampling rate
#define ADC_SAMPLING_RATE 125000000  // 125 MHz ADC sampling rate - assumed to be default

// DAC configuration
#define DAC_BASE_ADDR 0x43C10000
#define DAC_DMA_BASE_ADDR 0x40410000
#define IIC_BASE_ADDR 0xE0005000
#define DAC_FLASH_ADDR 0x31
#define DAC_DMA_IRQ 63

// ADC configuration
#define ADC_BASE_ADDR 0x43C00000
#define ADC_DMA_BASE_ADDR 0x40400000
#define ADC_FLASH_ADDR 0x30
#define ADC_DMA_IRQ 62
#define ZMOD_IRQ 61

// DAC register offsets
#define DAC_EN_REG_OFFSET 0x0  // DAC Enable register offset

// Fixed gain settings
#define DAC_GAIN 0  // 0 dB gain (LOW)
#define ADC_GAIN 0  // 0 dB gain (LOW)

// ADC scaling factor to match DAC amplitude
#define ADC_SCALING_FACTOR 0.4  // Reduce ADC values to 40% to match DAC amplitude

// Global variables
volatile bool running = true;
ZMODDAC1411* g_dacZmod = NULL;
ZMODADC1410* g_adcZmod = NULL;

// Add global variable to track last used DAC sample count
volatile int g_lastDacSampleCount = 1300;  // Default value

// File save paths
const char* DAC_CSV_FILE_PATH = "dac_signal_data.csv";
const char* ADC_DETAILED_CSV_FILE_PATH = "adc_detailed_data.csv";

// Signal handler function
void sig_handler(int signo) {
    if (signo == SIGINT || signo == SIGTERM) {
        printf("Received termination signal, shutting down...\n");
        running = false;
    }
}

// Helper function: Receive exact number of bytes
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

// Save received signal data to CSV file
bool saveSignalToCSV(float* realData, float* imagData, int numSamples, const char* filePath) {
    std::ofstream outFile(filePath);
    if (!outFile.is_open()) {
        std::cerr << "Error: Cannot open file for writing: " << filePath << std::endl;
        return false;
    }
    
    // Write CSV header
    outFile << "Index,Real,Imaginary" << std::endl;
    
    // Write all samples
    for (int i = 0; i < numSamples; i++) {
        outFile << i << "," << realData[i] << "," << imagData[i] << std::endl;
    }
    
    outFile.close();
    std::cout << "Signal data saved to " << filePath << std::endl;
    return true;
}

// Generate DAC signal from complex data
void dacGenerateFromComplex(float* realData, float* imagData, int numSamples) {
    if (!g_dacZmod) {
        std::cerr << "DAC not initialized!" << std::endl;
        return;
    }
    
    // Ensure sample count doesn't exceed maximum buffer size
    if (numSamples > MAX_SAMPLES) {
        std::cout << "Truncating samples from " << numSamples << " to " << MAX_SAMPLES << std::endl;
        numSamples = MAX_SAMPLES;
    }
    
    // Update global sample count tracker
    g_lastDacSampleCount = numSamples;
    std::cout << "Updated g_lastDacSampleCount to " << g_lastDacSampleCount << std::endl;
    
    // Allocate DAC channel buffer
    size_t length = numSamples;
    uint32_t *buf = g_dacZmod->allocChannelsBuffer(length);
    if (!buf) {
        std::cerr << "Failed to allocate DMA buffer!" << std::endl;
        return;
    }
    
    // Set frequency divider and gain for 100 MHz DAC sampling rate
    uint8_t frequencyDivider = 1;  // No division - use full 100 MHz clock
    uint8_t gain = DAC_GAIN;       // Fixed gain setting
    
    g_dacZmod->setOutputSampleFrequencyDivider(frequencyDivider);
    g_dacZmod->setGain(0, gain);  // Channel 0 (real part)
    g_dacZmod->setGain(1, gain);  // Channel 1 (imaginary part)
    
    std::cout << "DAC configuration: base clock=" << BASE_CLOCK/1000000 
              << "MHz, frequencyDivider=" << (int)frequencyDivider 
              << ", actual sampling rate=" << DAC_SAMPLING_RATE/1000000 
              << "MHz, gain=" << (int)gain << std::endl;
    
    // Prepare data for both channels
    for (int i = 0; i < numSamples; i++) {
        // Process real part (Channel 0)
        int16_t realRaw = g_dacZmod->getSignedRawFromVolt(realData[i], gain);
        
        // Process imaginary part (Channel 1)
        int16_t imagRaw = g_dacZmod->getSignedRawFromVolt(imagData[i], gain);
        
        // Combine both channels' data into a single 32-bit word
        uint32_t realValBuf = g_dacZmod->arrangeChannelData(0, realRaw);
        uint32_t imagValBuf = g_dacZmod->arrangeChannelData(1, imagRaw);
        
        // Combine both channel values
        buf[i] = realValBuf | imagValBuf;
        
        // Print debug info for first/last few samples only
        if (i < 5 || i > numSamples - 5) {
            std::cout << "DAC Sample[" << i << "]: real=" << realData[i] 
                      << ", imag=" << imagData[i]
                      << ", real_raw=" << realRaw 
                      << ", imag_raw=" << imagRaw << std::endl;
        }
    }
    
    // Send data to DAC and start output
    g_dacZmod->setData(buf, length);
    g_dacZmod->start();
    
    // Print DAC status after start
    uint32_t dacEnValue = g_dacZmod->readReg(DAC_EN_REG_OFFSET);
    std::cout << "DAC_EN register value after start: " << (dacEnValue & 0x1) << std::endl;
    
    // Free the buffer after transmission is set up
    g_dacZmod->freeChannelsBuffer(buf, length);
    
    // Save data to CSV file for analysis
    saveSignalToCSV(realData, imagData, numSamples, DAC_CSV_FILE_PATH);
}


bool handleReceiveCommand(int client_fd) {
       if (!g_adcZmod) {
           std::cerr << "ADC not initialized!" << std::endl;
           return false;
       }
       
       // Send acknowledgment to client
       const char* reply = "Starting receiving";
       if (send(client_fd, reply, strlen(reply), 0) < 0) {
           perror("Send acknowledgment failed");
           return false;
       }
       
       // Use the same sample count as the last DAC operation
       int numSamples = g_lastDacSampleCount;
       
       // Print detailed debug information
       std::cout << "Using sample count from last DAC operation: " << numSamples << std::endl;
       
       // Send the sample count as a formatted string instead of binary
       char sampleCountStr[32];
       sprintf(sampleCountStr, "SAMPLES=%d", numSamples);
       std::cout << "Sending sample count as string: " << sampleCountStr << std::endl;
       
       if (send(client_fd, sampleCountStr, strlen(sampleCountStr), 0) < 0) {
           perror("Failed to send sample count string");
           return false;
       }
       
       // Wait to ensure client has processed previous message
       usleep(100000); // 100ms
       
       // Create buffer for ADC data
       size_t adcBufferLength = numSamples;
       printf("Attempting to allocate DMA buffer, size: %zu\n", adcBufferLength * sizeof(uint32_t));
    
    uint32_t *adcBuffer = g_adcZmod->allocChannelsBuffer(adcBufferLength);
    
    if (!adcBuffer) {
        std::cerr << "Failed to allocate ADC DMA buffer" << std::endl;
        return false;
    }
    
    // Acquire data from ADC - ADC uses default 125MHz sampling rate
    std::cout << "Acquiring data from ADC at " << ADC_SAMPLING_RATE/1000000 << "MHz..." << std::endl;
    g_adcZmod->acquireImmediatePolling(adcBuffer, adcBufferLength);
    
    // Process data from both channels using proper API methods
    float* realPart = new float[numSamples];
    float* imagPart = new float[numSamples];
    float* rawRealVolt = new float[numSamples];
    float* rawImagVolt = new float[numSamples];
    
    char val_formatted[15];
    std::ofstream detailedFile(ADC_DETAILED_CSV_FILE_PATH);
    detailedFile << "Index,Channel0_Raw,Channel0_Volt,Channel1_Raw,Channel1_Volt,Scaled_Real,Scaled_Imag" << std::endl;
    
    for (int i = 0; i < numSamples; i++) {
        // Extract real data (channel 0) - properly following official demo
        int16_t valCh0 = g_adcZmod->signedChannelData(0, adcBuffer[i]);
        rawRealVolt[i] = g_adcZmod->getVoltFromSignedRaw(valCh0, ADC_GAIN);
        
        // Extract imaginary data (channel 1) - properly following official demo
        int16_t valCh1 = g_adcZmod->signedChannelData(1, adcBuffer[i]);
        rawImagVolt[i] = g_adcZmod->getVoltFromSignedRaw(valCh1, ADC_GAIN);
        
        // Apply scaling factor to match DAC amplitude
        realPart[i] = rawRealVolt[i] * ADC_SCALING_FACTOR;
        imagPart[i] = rawImagVolt[i] * ADC_SCALING_FACTOR;
        
        // Write detailed data to file
        detailedFile << i << "," << valCh0 << "," << rawRealVolt[i] << "," 
                    << valCh1 << "," << rawImagVolt[i] << "," 
                    << realPart[i] << "," << imagPart[i] << std::endl;
        
        // Print debug info for first/last few samples only
        if (i < 5 || i > numSamples - 5) {
            g_adcZmod->formatValue(val_formatted, 1000.0*rawRealVolt[i], "mV");
            std::cout << "ADC Sample[" << i << "]: ch0_raw=" << valCh0 
                      << ", ch0_volt=" << rawRealVolt[i] 
                      << " (" << val_formatted << "), ";
            
            g_adcZmod->formatValue(val_formatted, 1000.0*rawImagVolt[i], "mV");
            std::cout << "ch1_raw=" << valCh1
                      << ", ch1_volt=" << rawImagVolt[i]
                      << " (" << val_formatted << ")";
            
            std::cout << ", scaled_real=" << realPart[i]
                      << ", scaled_imag=" << imagPart[i] << std::endl;
        }
    }
    
    detailedFile.close();
    std::cout << "Detailed ADC data saved to " << ADC_DETAILED_CSV_FILE_PATH << std::endl;
    
    // Free the ADC buffer
    g_adcZmod->freeChannelsBuffer(adcBuffer, adcBufferLength);
    
    // Send real part data to MATLAB
    std::cout << "Sending real part to MATLAB..." << std::endl;
    if (send(client_fd, realPart, numSamples * sizeof(float), 0) < 0) {
        perror("Send real data failed");
        delete[] realPart;
        delete[] imagPart;
        delete[] rawRealVolt;
        delete[] rawImagVolt;
        return false;
    }
    
    // Wait a bit to ensure data is received
    usleep(100000); // 100ms
    
    // Send imaginary part data to MATLAB
    std::cout << "Sending imaginary part to MATLAB..." << std::endl;
    if (send(client_fd, imagPart, numSamples * sizeof(float), 0) < 0) {
        perror("Send imaginary data failed");
        delete[] realPart;
        delete[] imagPart;
        delete[] rawRealVolt;
        delete[] rawImagVolt;
        return false;
    }
    
    // Clean up
    delete[] realPart;
    delete[] imagPart;
    delete[] rawRealVolt;
    delete[] rawImagVolt;
    
    std::cout << "ADC data transmission complete" << std::endl;
    return true;
}

int main() {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);
    char buffer[BUFFER_SIZE];
    
    // Setup signal handler
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    
    // Initialize ZMOD DAC
    std::cout << "Initializing Zmod hardware..." << std::endl;
    g_dacZmod = new ZMODDAC1411(DAC_BASE_ADDR, DAC_DMA_BASE_ADDR, IIC_BASE_ADDR, DAC_FLASH_ADDR, DAC_DMA_IRQ);
    if (!g_dacZmod) {
        std::cerr << "Failed to initialize DAC!" << std::endl;
        return 1;
    }
    
    // Initialize ZMOD ADC
    g_adcZmod = new ZMODADC1410(ADC_BASE_ADDR, ADC_DMA_BASE_ADDR, IIC_BASE_ADDR, ADC_FLASH_ADDR,
                              ZMOD_IRQ, ADC_DMA_IRQ);
    if (!g_adcZmod) {
        std::cerr << "Failed to initialize ADC!" << std::endl;
        delete g_dacZmod;
        return 1;
    }
    
    // Set ADC gain for channel 0 (CH1) and channel 1 (CH2)
    g_adcZmod->setGain(0, ADC_GAIN); // Fixed gain setting
    g_adcZmod->setGain(1, ADC_GAIN); // Fixed gain setting
    
    std::cout << "Zmod hardware initialized successfully" << std::endl;
    std::cout << "System base clock: " << BASE_CLOCK/1000000 << " MHz" << std::endl;
    std::cout << "DAC configuration: sampling rate=" << DAC_SAMPLING_RATE/1000000 
              << " MHz, gain=" << (int)DAC_GAIN << std::endl;
    std::cout << "ADC configuration: sampling rate=" << ADC_SAMPLING_RATE/1000000 
              << " MHz, gain=" << (int)ADC_GAIN 
              << ", scaling factor=" << ADC_SCALING_FACTOR << std::endl;
    std::cout << "Initial sample count set to: " << g_lastDacSampleCount << std::endl;
    
    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation failed");
        delete g_dacZmod;
        delete g_adcZmod;
        return 1;
    }
    
    // Enable socket reuse
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR) failed");
        delete g_dacZmod;
        delete g_adcZmod;
        return 1;
    }
    
    // Set TCP_NODELAY if supported
    #ifdef TCP_NODELAY
    if (setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
        perror("setsockopt(TCP_NODELAY) failed");
        // Non-fatal, continue
    }
    #endif
    
    // Define server address
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(SERVER_PORT);
    
    // Bind socket to port
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("Bind failed");
        delete g_dacZmod;
        delete g_adcZmod;
        return 1;
    }
    
    // Listen for connections
    if (listen(server_fd, 1) < 0) {
        perror("Listen failed");
        delete g_dacZmod;
        delete g_adcZmod;
        return 1;
    }
    
    printf("Server listening on port %d...\n", SERVER_PORT);
    
    while (running) {
        // Accept client connection
        if ((client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len)) < 0) {
            perror("Accept failed");
            continue;
        }
        
        printf("Client connected from %s:%d\n", 
               inet_ntoa(client_addr.sin_addr), 
               ntohs(client_addr.sin_port));
        
        // Set TCP_NODELAY for client socket
        #ifdef TCP_NODELAY
        if (setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) < 0) {
            perror("Client socket setsockopt(TCP_NODELAY) failed");
        }
        #endif
        
        bool dac_transmitting = false;
        
        while (running) {
            memset(buffer, 0, BUFFER_SIZE);
            ssize_t bytes_read = recv(client_fd, buffer, BUFFER_SIZE, 0);
            
            if (bytes_read <= 0) {
                if (bytes_read == 0) {
                    printf("Client disconnected\n");
                } else {
                    perror("Receive failed");
                }
                break;
            }
            
            printf("Received command: %s\n", buffer);
            
            if (strcmp(buffer, "transmit") == 0) {
                // MATLAB->DAC: Start transmission mode
                dac_transmitting = true;
                
                // Reply to client
                const char* reply = "Ready for data";
                if (send(client_fd, reply, strlen(reply), 0) < 0) {
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
                
                // Generate DAC waveform
                dacGenerateFromComplex(realPart, imagPart, dataLength);
                
                // Free the allocated memory
                delete[] realPart;
                delete[] imagPart;
                
                // Confirm to client
                char confirm_buf[100];
                snprintf(confirm_buf, sizeof(confirm_buf), "Transmission started with %d samples", dataLength);
                if (send(client_fd, confirm_buf, strlen(confirm_buf), 0) < 0) {
                    perror("Send confirmation failed");
                    break;
                }
            }
            else if (strcmp(buffer, "receive") == 0) {  // Fixed to match the corrected MATLAB function
                // Handle receive command - ADC->MATLAB
                printf("Handling receive command from MATLAB\n");
                if (!handleReceiveCommand(client_fd)) {
                    perror("Receive operation failed");
                }
            }
            else if (strcmp(buffer, "stop") == 0) {
                if (dac_transmitting) {
                    // Stop DAC output
                    g_dacZmod->stop();
                    dac_transmitting = false;
                    
                    const char* reply = "Transmission stopped";
                    if (send(client_fd, reply, strlen(reply), 0) < 0) {
                        perror("Send failed");
                        break;
                    }
                    
                    printf("Transmission stopped\n");
                } else {
                    const char* reply = "Nothing to stop";
                    if (send(client_fd, reply, strlen(reply), 0) < 0) {
                        perror("Send failed");
                        break;
                    }
                }
            }
            else if (strcmp(buffer, "exit") == 0) {
                // Clean exit
                const char* reply = "Goodbye";
                send(client_fd, reply, strlen(reply), 0);
                printf("Client requested exit\n");
                break;
            }
            else {
                // Unknown command
                const char* reply = "Unknown command";
                if (send(client_fd, reply, strlen(reply), 0) < 0) {
                    perror("Send failed");
                    break;
                }
            }
        }
        
        close(client_fd);
        printf("Connection closed. Waiting for new connections...\n");
    }
    
    // Clean up hardware
    delete g_dacZmod;
    delete g_adcZmod;
    
    close(server_fd);
    printf("Server shutdown complete\n");
    return 0;
}
