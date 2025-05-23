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
#include <algorithm>
#include <complex>
#include <cmath>
#include <random>

// Include ZMOD library
#include "zmodlib/Zmod/zmod.h"
#include "zmodlib/ZmodDAC1411/zmoddac1411.h"
#include "zmodlib/ZmodADC1410/zmodadc1410.h"

// Configuration constants
#define SERVER_PORT 8080
#define BUFFER_SIZE 8192
#define MAX_SAMPLES 3000  // Maximum number of samples per buffer

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
#define ADC_GAIN 0  // 

// ADC scaling factor to match DAC amplitude
#define ADC_SCALING_FACTOR 0.325

// Global variables
volatile bool running = true;
ZMODDAC1411* g_dacZmod = NULL;
ZMODADC1410* g_adcZmod = NULL;

// Add global variable to track last used DAC sample count
volatile int g_lastDacSampleCount = 65536;  // Default value

// File save paths
const char* DAC_CSV_FILE_PATH = "dac_signal_data.csv";
const char* ADC_DETAILED_CSV_FILE_PATH = "adc_detailed_data.csv";

// Global variables to store filtered pilot sequences
std::vector<std::complex<float>> g_filteredStartPilot;
std::vector<std::complex<float>> g_filteredEndPilot;


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


void resetZmodHardware() {
    std::cout << "Resetting ZMOD hardware state..." << std::endl;
    
    // 1. 停止并重置DAC
    if (g_dacZmod) {
        std::cout << "  Stopping DAC..." << std::endl;
        g_dacZmod->stop();
        usleep(50000); // 50ms延迟
        
        // 发送零信号清空DAC输出
        size_t zeroLength = 1024;
        uint32_t *zeroBuf = g_dacZmod->allocChannelsBuffer(zeroLength);
        if (zeroBuf) {
            memset(zeroBuf, 0, zeroLength * sizeof(uint32_t));
            g_dacZmod->setData(zeroBuf, zeroLength);
            g_dacZmod->start();
            usleep(10000); // 让零信号输出一段时间
            g_dacZmod->stop();
            g_dacZmod->freeChannelsBuffer(zeroBuf, zeroLength);
        }
        
        // 重新设置DAC参数
        g_dacZmod->setOutputSampleFrequencyDivider(0);
        g_dacZmod->setGain(0, DAC_GAIN);
        g_dacZmod->setGain(1, DAC_GAIN);
        std::cout << "  DAC reset complete" << std::endl;
    }
    
    // 2. 重置ADC
    if (g_adcZmod) {
        std::cout << "  Resetting ADC..." << std::endl;
        
        // 重新设置ADC增益
        g_adcZmod->setGain(0, ADC_GAIN);
        g_adcZmod->setGain(1, ADC_GAIN);
        
        // 执行一次小的采集操作来清空ADC缓存
        size_t clearSize = 100;
        uint32_t *clearBuf = g_adcZmod->allocChannelsBuffer(clearSize);
        if (clearBuf) {
            g_adcZmod->acquireImmediatePolling(clearBuf, clearSize);
            g_adcZmod->freeChannelsBuffer(clearBuf, clearSize);
        }
        std::cout << "  ADC reset complete" << std::endl;
    }
    
    std::cout << "Hardware reset complete" << std::endl;
}





// Function to receive filtered pilots from MATLAB
bool receiveFilteredPilots(int client_fd) {

    // Send acknowledgment
    const char* reply = "Ready for filtered pilots";
    if (send(client_fd, reply, strlen(reply), 0) < 0) {
        perror("Send acknowledgment failed");
        return false;
    }
    
    // Receive start pilot length
    int32_t startPilotLength;
    if (!receiveExactBytes(client_fd, &startPilotLength, sizeof(int32_t))) {
        perror("Failed to receive start pilot length");
        return false;
    }
    
    std::cout << "Receiving filtered start pilot, length: " << startPilotLength << " samples" << std::endl;
    
    // Allocate memory for start pilot
    float* startPilotReal = new float[startPilotLength];
    float* startPilotImag = new float[startPilotLength];
    
    // Receive real part of start pilot
    if (!receiveExactBytes(client_fd, startPilotReal, startPilotLength * sizeof(float))) {
        perror("Failed to receive start pilot real part");
        delete[] startPilotReal;
        delete[] startPilotImag;
        return false;
    }
    
    // Receive imaginary part of start pilot
    if (!receiveExactBytes(client_fd, startPilotImag, startPilotLength * sizeof(float))) {
        perror("Failed to receive start pilot imaginary part");
        delete[] startPilotReal;
        delete[] startPilotImag;
        return false;
    }
    
    // Store start pilot in global vector
    g_filteredStartPilot.resize(startPilotLength);
    for (int i = 0; i < startPilotLength; i++) {
        g_filteredStartPilot[i] = std::complex<float>(startPilotReal[i], startPilotImag[i]);
    }
    
    // Free temporary arrays
    delete[] startPilotReal;
    delete[] startPilotImag;
    
    // Receive end pilot length
    int32_t endPilotLength;
    if (!receiveExactBytes(client_fd, &endPilotLength, sizeof(int32_t))) {
        perror("Failed to receive end pilot length");
        return false;
    }
    
    std::cout << "Receiving filtered end pilot, length: " << endPilotLength << " samples" << std::endl;
    
    // Allocate memory for end pilot
    float* endPilotReal = new float[endPilotLength];
    float* endPilotImag = new float[endPilotLength];
    
    // Receive real part of end pilot
    if (!receiveExactBytes(client_fd, endPilotReal, endPilotLength * sizeof(float))) {
        perror("Failed to receive end pilot real part");
        delete[] endPilotReal;
        delete[] endPilotImag;
        return false;
    }
    
    // Receive imaginary part of end pilot
    if (!receiveExactBytes(client_fd, endPilotImag, endPilotLength * sizeof(float))) {
        perror("Failed to receive end pilot imaginary part");
        delete[] endPilotReal;
        delete[] endPilotImag;
        return false;
    }
    
    // Store end pilot in global vector
    g_filteredEndPilot.resize(endPilotLength);
    for (int i = 0; i < endPilotLength; i++) {
        g_filteredEndPilot[i] = std::complex<float>(endPilotReal[i], endPilotImag[i]);
    }
    
    // Free temporary arrays
    delete[] endPilotReal;
    delete[] endPilotImag;
    
    // Save pilots to files for debugging
    std::ofstream startPilotFile("filtered_start_pilot.csv");
    startPilotFile << "Index,Real,Imag,Magnitude\n";
    for (size_t i = 0; i < g_filteredStartPilot.size(); i++) {
        float magnitude = std::abs(g_filteredStartPilot[i]);
        startPilotFile << i << "," 
                      << g_filteredStartPilot[i].real() << "," 
                      << g_filteredStartPilot[i].imag() << "," 
                      << magnitude << "\n";
    }
    startPilotFile.close();
    
    std::ofstream endPilotFile("filtered_end_pilot.csv");
    endPilotFile << "Index,Real,Imag,Magnitude\n";
    for (size_t i = 0; i < g_filteredEndPilot.size(); i++) {
        float magnitude = std::abs(g_filteredEndPilot[i]);
        endPilotFile << i << "," 
                    << g_filteredEndPilot[i].real() << "," 
                    << g_filteredEndPilot[i].imag() << "," 
                    << magnitude << "\n";
    }
    endPilotFile.close();
    
    // Calculate pilot energies for later use
    float startPilotEnergy = 0.0f;
    float endPilotEnergy = 0.0f;
    
    for (const auto& sample : g_filteredStartPilot) {
        startPilotEnergy += std::norm(sample);
    }
    
    for (const auto& sample : g_filteredEndPilot) {
        endPilotEnergy += std::norm(sample);
    }
    
    std::cout << "Filtered start pilot energy: " << startPilotEnergy << std::endl;
    std::cout << "Filtered end pilot energy: " << endPilotEnergy << std::endl;
    
    // Send acknowledgment
    const char* ack = "Pilots received successfully";
    if (send(client_fd, ack, strlen(ack), 0) < 0) {
        perror("Send final acknowledgment failed");
        return false;
    }
    
    std::cout << "Filtered pilots received and stored successfully" << std::endl;
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
    
    
    g_dacZmod->setOutputSampleFrequencyDivider(0);
    g_dacZmod->setGain(0, DAC_GAIN);  // Channel 0 (real part)
    g_dacZmod->setGain(1, DAC_GAIN);  // Channel 1 (imaginary part)

    
    // Prepare data for both channels
    for (int i = 0; i < numSamples; i++) {
        // Process real part (Channel 0)
        int16_t realRaw = g_dacZmod->getSignedRawFromVolt(realData[i], DAC_GAIN);
        
        // Process imaginary part (Channel 1)
        int16_t imagRaw = g_dacZmod->getSignedRawFromVolt(imagData[i], DAC_GAIN);
        
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
    
    float transmissionTimeMs = (float)numSamples / 100000.0f; // 100MHz采样率
    int waitTimeUs = (int)(transmissionTimeMs * 1000) * 2; 
    usleep(waitTimeUs);

    // Free the buffer after transmission is set up
    g_dacZmod->freeChannelsBuffer(buf, length);
    
    // Save data to CSV file for analysis
    saveSignalToCSV(realData, imagData, numSamples, DAC_CSV_FILE_PATH);
    usleep(500000);
}


bool handleReceiveCommand(int client_fd) {
    if (!g_adcZmod) {
        std::cerr << "ADC not initialized!" << std::endl;
        return false;
    }
    
    // Check if we have the filtered pilots
    if (g_filteredStartPilot.empty() || g_filteredEndPilot.empty()) {
        std::cerr << "Filtered pilots not received yet! Run transmit first." << std::endl;
        const char* error_msg = "Error: Filtered pilots not available";
        send(client_fd, error_msg, strlen(error_msg), 0);
        return false;
    }
    
    // Define constants for data acquisition
    const int samplesPerSecond = 100000000; // 100MHz
    const int maxSamplesToCollect = samplesPerSecond; // Up to 1 second
    size_t batchSize = 500000; // Number of samples per batch
    
    // Get pilot lengths from stored filtered pilots
    const int startPilotLength = g_filteredStartPilot.size();
    const int endPilotLength = g_filteredEndPilot.size();
    
    std::cout << "\n===== FILTERED PILOT INFORMATION =====\n";
    std::cout << "Start Pilot Length: " << startPilotLength << " samples\n";
    std::cout << "End Pilot Length: " << endPilotLength << " samples\n";
    
    // Calculate energies and find maximum values of the pilot sequences
    float startPilotEnergy = 0.0f;
    float endPilotEnergy = 0.0f;
    float startPilotMeanMag = 0.0f;
    float endPilotMeanMag = 0.0f;
    float startPilotMaxMag = 0.0f;
    float endPilotMaxMag = 0.0f;
    
    for (const auto& sample : g_filteredStartPilot) {
        float mag = std::abs(sample);
        startPilotEnergy += std::norm(sample);
        startPilotMeanMag += mag;
        startPilotMaxMag = std::max(startPilotMaxMag, mag);
    }
    startPilotMeanMag /= startPilotLength;
    
    for (const auto& sample : g_filteredEndPilot) {
        float mag = std::abs(sample);
        endPilotEnergy += std::norm(sample);
        endPilotMeanMag += mag;
        endPilotMaxMag = std::max(endPilotMaxMag, mag);
    }
    endPilotMeanMag /= endPilotLength;
    
    std::cout << "Start Pilot: Energy=" << startPilotEnergy 
              << ", Mean Mag=" << startPilotMeanMag 
              << ", Max Mag=" << startPilotMaxMag << std::endl;
    std::cout << "End Pilot: Energy=" << endPilotEnergy 
              << ", Mean Mag=" << endPilotMeanMag 
              << ", Max Mag=" << endPilotMaxMag << std::endl;
    
    // Calculate expected signal amplitudes
    float expectedSignalMag = (startPilotMeanMag + endPilotMeanMag) / 2.0f;
    std::cout << "Expected Signal Magnitude: " << expectedSignalMag << std::endl;
    
    // Thresholds
    const float startPilotThreshold = 0.5f;  // Increased threshold for better reliability
    const float endPilotThreshold = 0.5f;
    std::cout << "Detection Thresholds: Start=" << startPilotThreshold 
              << ", End=" << endPilotThreshold << std::endl;
    std::cout << "===================================\n\n";
    
    // Create vector for received samples
    std::vector<std::complex<float>> receivedSamples;
    receivedSamples.reserve(maxSamplesToCollect);
    
    // Timer and detection variables
    time_t startTime = time(NULL);
    bool pilotFound = false;
    bool endPilotFound = false;
    int pilotPosition = -1;
    int endPilotPosition = -1;
    int totalSamplesCollected = 0;
    const int minExpectedDataLength = 300; // Minimum data length between pilots
    
    // Create debug logs
    std::ofstream corrLog("pilot_correlation.csv");
    corrLog << "Position,StartPilotCorr,EndPilotCorr,SignalMagnitude,StartCorrUnorm,EndCorrUnorm\n";
    
    std::ofstream sampleTrace("sample_trace.csv");
    sampleTrace << "Index,Real,Imag,Magnitude,Phase\n";
    
    // Histogram for signal magnitudes
    const int magnitudeBins = 20;
    std::vector<int> magnitudeHistogram(magnitudeBins, 0);
    const float maxMagnitudeForHistogram = 1.0f;
    
    // Global maximum correlation tracking
    float globalMaxStartCorr = 0.0f;
    int globalMaxStartPos = -1;
    float globalMaxEndCorr = 0.0f;
    int globalMaxEndPos = -1;
    
    // Batch processing loop
    int batchNumber = 0;

    while (totalSamplesCollected < maxSamplesToCollect) {
    batchNumber++;
    std::cout << "\n===== PROCESSING BATCH #" << batchNumber << " =====\n";
    
    // 时间限制检查
    time_t currentTime = time(NULL);
    if (difftime(currentTime, startTime) > 1.0) {
        std::cout << "Time limit reached. Stopping collection." << std::endl;
        break;
    }
    
    // 如果pilots都已找到且数据有效，退出
    if (pilotFound && endPilotFound && 
        endPilotPosition > pilotPosition) {
        std::cout << "Valid signal detected. Stopping collection." << std::endl;
        break;
    }
    
    // 分配ADC缓冲区
    uint32_t *adcBuffer = g_adcZmod->allocChannelsBuffer(batchSize);
    if (!adcBuffer) {
        std::cerr << "Failed to allocate ADC buffer!" << std::endl;
        corrLog.close();
        sampleTrace.close();
        return false;
    }
    
    // 获取数据
    g_adcZmod->acquireImmediatePolling(adcBuffer, batchSize);
    
    // 处理样本
    int batchStartIndex = totalSamplesCollected;
    
    // 本批次统计
    float maxMagnitude = 0.0f;
    float avgMagnitude = 0.0f;
    
    for (int i = 0; i < batchSize; i++) {
        int16_t realRaw = g_adcZmod->signedChannelData(0, adcBuffer[i]);
        int16_t imagRaw = g_adcZmod->signedChannelData(1, adcBuffer[i]);
        
        float realVolt = g_adcZmod->getVoltFromSignedRaw(realRaw, ADC_GAIN) * ADC_SCALING_FACTOR;
        float imagVolt = g_adcZmod->getVoltFromSignedRaw(imagRaw, ADC_GAIN) * ADC_SCALING_FACTOR;
        
        float magnitude = std::sqrt(realVolt*realVolt + imagVolt*imagVolt);
        float phase = std::atan2(imagVolt, realVolt) * 180.0f / M_PI;
        
        maxMagnitude = std::max(maxMagnitude, magnitude);
        avgMagnitude += magnitude;
        
        // 更新直方图
        int binIndex = std::min(magnitudeBins - 1, static_cast<int>(magnitude * magnitudeBins / maxMagnitudeForHistogram));
        if (binIndex >= 0) {
            magnitudeHistogram[binIndex]++;
        }
        
        // 存储样本
        receivedSamples.push_back(std::complex<float>(realVolt, imagVolt));
        
        // 记录样本
        sampleTrace << (totalSamplesCollected + i) << "," 
                    << realVolt << "," << imagVolt << "," 
                    << magnitude << "," << phase << "\n";
    }
    
    // 更新样本计数并释放缓冲区
    totalSamplesCollected += batchSize;
    g_adcZmod->freeChannelsBuffer(adcBuffer, batchSize);
    

    //下面开始的相关性检测应该就算没问题了
    // 执行相关性检测
    if (receivedSamples.size() >= std::max(startPilotLength, endPilotLength)) {
        // 搜索范围
        int searchStart = std::max(0, batchStartIndex - std::max(startPilotLength, endPilotLength));
        int searchEnd = totalSamplesCollected - std::max(startPilotLength, endPilotLength);
        
        std::cout << "Searching in range [" << searchStart << ", " << searchEnd << "]\n";
        
        // 追踪最后有效的检测位置
        static int last_valid_start_pos = -1;
        static float last_valid_start_corr = 0.0f;
        static int last_valid_end_pos = -1;
        static float last_valid_end_corr = 0.0f;
        
        // 相关性计算步长
        int stepSize = 1;
        
        // 对每个位置计算相关性
        for (int pos = searchStart; pos < searchEnd; pos += stepSize) {
            if ((pos - searchStart) % 10000 == 0 && pos > searchStart) {
                std::cout << "Processed " << (pos - searchStart) << " positions..." << std::endl;
            }
            
            // 确保有足够的样本进行相关性计算
            if (pos + std::max(startPilotLength, endPilotLength) > receivedSamples.size()) {
                continue;
            }
            
            // 计算归一化因子
            float signalEnergyStart = 0.0f;
            float signalEnergyEnd = 0.0f;
            
            for (int i = 0; i < startPilotLength && pos + i < receivedSamples.size(); i++) {
                signalEnergyStart += std::norm(receivedSamples[pos + i]);
            }
            
            if (startPilotLength != endPilotLength) {
                for (int i = 0; i < endPilotLength && pos + i < receivedSamples.size(); i++) {
                    signalEnergyEnd += std::norm(receivedSamples[pos + i]);
                }
            } else {
                signalEnergyEnd = signalEnergyStart;
            }
            
            // Start pilot相关性计算
            std::complex<float> startCorr(0.0f, 0.0f);
            if (!pilotFound) {
                for (int i = 0; i < startPilotLength && pos + i < receivedSamples.size(); i++) {
                    startCorr += receivedSamples[pos + i] * std::conj(g_filteredStartPilot[i]);
                }
            }
            
            // End pilot相关性计算
            std::complex<float> endCorr(0.0f, 0.0f);
            if (pilotFound) {
                for (int i = 0; i < endPilotLength && pos + i < receivedSamples.size(); i++) {
                    endCorr += receivedSamples[pos + i] * std::conj(g_filteredEndPilot[i]);
                }
            }
            
            // 归一化相关性
            float startCorrNorm = 0.0f;
            float endCorrNorm = 0.0f;
            
            if (signalEnergyStart > 0 && !pilotFound) {
                startCorrNorm = std::abs(startCorr) / std::sqrt(signalEnergyStart * startPilotEnergy);
                if (startCorrNorm > 1.0f) startCorrNorm = 1.0f;
            }
            
            if (signalEnergyEnd > 0 && pilotFound) {
                endCorrNorm = std::abs(endCorr) / std::sqrt(signalEnergyEnd * endPilotEnergy);
                if (endCorrNorm > 1.0f) endCorrNorm = 1.0f;
            }
            
            // Start pilot
            if (!pilotFound && startCorrNorm > startPilotThreshold) {
                pilotFound = true;
                pilotPosition = pos;
                std::cout << "*** START PILOT DETECTED at position " << pilotPosition 
                        << " with correlation " << startCorrNorm << " ***\n";
                
                // 打印调试信息
                std::cout << "Start pilot signal samples:\n";
                for (int i = 0; i < 10 && pilotPosition + i < receivedSamples.size(); i++) {
                    std::complex<float> sample = receivedSamples[pilotPosition + i];
                    std::cout << "[" << i << "]: " << sample.real() << " + " << sample.imag() << "j\n";
                }
            }

            // End pilot
            if (pilotFound && !endPilotFound && pos > pilotPosition && 
                endCorrNorm > endPilotThreshold) {
                
                endPilotFound = true;
                endPilotPosition = pos;
                std::cout << "*** END PILOT DETECTED at position " << endPilotPosition 
                        << " with correlation " << endCorrNorm << " ***\n";
                
                // 打印调试信息
                std::cout << "End pilot signal samples:\n";
                for (int i = 0; i < 10 && endPilotPosition + i < receivedSamples.size(); i++) {
                    std::complex<float> sample = receivedSamples[endPilotPosition + i];
                    std::cout << "[" << i << "]: " << sample.real() << " + " << sample.imag() << "j\n";
                }
                
                // 检查位置关系
                if (endPilotPosition <= pilotPosition) {
                    endPilotFound = false;
                    endPilotPosition = -1;
                    std::cout << "End pilot detected before start pilot - continuing search\n";
                }
            }


            // 记录相关性数据
            float signalMeanMag = 0.0f;
            int sampleCount = std::min(100, (int)(receivedSamples.size() - pos));
            for (int i = 0; i < sampleCount; i++) {
                signalMeanMag += std::abs(receivedSamples[pos + i]);
            }
            signalMeanMag /= sampleCount;
            
            corrLog << pos << "," << startCorrNorm << "," << endCorrNorm << "," 
                    << signalMeanMag << "," << std::abs(startCorr) << "," << std::abs(endCorr) << "\n";
        }
    }
}


    // Print magnitude histogram
    std::cout << "\n===== SIGNAL MAGNITUDE HISTOGRAM =====\n";
    int totalSamples = 0;
    for (int count : magnitudeHistogram) {
        totalSamples += count;
    }
    
    for (int i = 0; i < magnitudeHistogram.size(); i++) {
        float binStart = i * maxMagnitudeForHistogram / magnitudeBins;
        float binEnd = (i + 1) * maxMagnitudeForHistogram / magnitudeBins;
        float percentage = (totalSamples > 0) ? 100.0f * magnitudeHistogram[i] / totalSamples : 0.0f;
        
        std::cout << "[" << binStart << "-" << binEnd << "]: " 
                  << magnitudeHistogram[i] << " samples (" << percentage << "%)" << std::endl;
    }
    std::cout << "=====================================\n";
    
    // Close logs
    corrLog.close();
    sampleTrace.close();
    
    // Determine what data to send to MATLAB
    int dataStart = 0;
    int dataLength = receivedSamples.size();
    
    std::cout << "\n===== PREPARING DATA FOR TRANSMISSION =====\n";
    
    if (pilotFound && endPilotFound && 
        endPilotPosition > pilotPosition && 
        (endPilotPosition - pilotPosition) > minExpectedDataLength) {
        // Extract data including both pilots
        dataStart = pilotPosition;
        dataLength = (endPilotPosition + endPilotLength) - pilotPosition;
        //dataLength = endPilotPosition - pilotPosition;
        std::cout << "Sending data with both pilots.\n";
        std::cout << "  Start position: " << dataStart << "\n";
        std::cout << "  End position: " << (dataStart + dataLength - 1) << "\n";
        std::cout << "  Data length: " << dataLength << " samples\n";
    }

    else if (pilotFound) {
        // Only start pilot found - send from start pilot to end of buffer
        dataStart = pilotPosition;
        dataLength = receivedSamples.size() - pilotPosition;
        
        std::cout << "Only start pilot found. Sending from start pilot to end.\n";
        std::cout << "  Start position: " << dataStart << "\n";
        std::cout << "  Data length: " << dataLength << " samples\n";
    }
    else {
        // No pilots found - send all collected data
        std::cout << "No pilots found. Sending all collected data.\n";
        std::cout << "  Data length: " << dataLength << " samples\n";
    }
    
    // Safety checks
    if (dataStart < 0) {
        std::cout << "WARNING: Negative data start index. Resetting to 0.\n";
        dataStart = 0;
    }
    
    if (dataStart + dataLength > receivedSamples.size()) {
        std::cout << "WARNING: Data extends beyond buffer. Truncating.\n";
        dataLength = receivedSamples.size() - dataStart;
    }
    
    // Send the sample count to MATLAB
    char sampleCountStr[32];
    sprintf(sampleCountStr, "SAMPLES=%d", dataLength);
    std::cout << "Sending sample count: " << sampleCountStr << std::endl;
    
    if (send(client_fd, sampleCountStr, strlen(sampleCountStr), 0) < 0) {
        perror("Failed to send sample count string");
        return false;
    }
    
    usleep(500000);  // 10ms delay
    
    // Prepare arrays for transmission
    float* realPart = new float[dataLength];
    float* imagPart = new float[dataLength];
    
    // Fill arrays and analyze signal
    float avgMagnitude = 0.0f;
    float maxMagnitude = 0.0f;
    float minMagnitude = std::numeric_limits<float>::max();
    
    for (int i = 0; i < dataLength; i++) {
        realPart[i] = receivedSamples[dataStart + i].real();
        imagPart[i] = receivedSamples[dataStart + i].imag();
        
        float magnitude = std::sqrt(realPart[i]*realPart[i] + imagPart[i]*imagPart[i]);
        avgMagnitude += magnitude;
        maxMagnitude = std::max(maxMagnitude, magnitude);
        minMagnitude = std::min(minMagnitude, magnitude);
    }
    
    if (dataLength > 0) {
        avgMagnitude /= dataLength;
    }
    
    std::cout << "Extracted data statistics:\n";
    std::cout << "  Min magnitude: " << minMagnitude << "\n";
    std::cout << "  Avg magnitude: " << avgMagnitude << "\n";
    std::cout << "  Max magnitude: " << maxMagnitude << "\n";
    
    // Save extracted data to CSV for analysis
    std::ofstream dataFile("extracted_data.csv");
    dataFile << "Index,Real,Imag,Magnitude,Phase\n";
    
    for (int i = 0; i < dataLength; i++) {
        float magnitude = std::sqrt(realPart[i]*realPart[i] + imagPart[i]*imagPart[i]);
        float phase = std::atan2(imagPart[i], realPart[i]) * 180.0f / M_PI;
        dataFile << i << "," << realPart[i] << "," << imagPart[i] << ","
                << magnitude << "," << phase << "\n";
    }
    dataFile.close();
    
    // Send real part to MATLAB
    std::cout << "Sending real part data (" << dataLength << " samples) to MATLAB..." << std::endl;
    if (send(client_fd, realPart, dataLength * sizeof(float), 0) < 0) {
        perror("Send real data failed");
        delete[] realPart;
        delete[] imagPart;
        return false;
    }
    
    usleep(100000);  // 100ms delay
    
    // Send imaginary part to MATLAB
    std::cout << "Sending imaginary part data (" << dataLength << " samples) to MATLAB..." << std::endl;
    if (send(client_fd, imagPart, dataLength * sizeof(float), 0) < 0) {
        perror("Send imaginary data failed");
        delete[] realPart;
        delete[] imagPart;
        return false;
    }
    
    // Clean up
    delete[] realPart;
    delete[] imagPart;
    
    std::cout << "\nADC data transmission complete - sent " << dataLength << " samples" << std::endl;
    
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
        
        resetZmodHardware();

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
            
            // New command for filtered pilots
            if (strcmp(buffer, "filtered_pilots") == 0) {
                if (!receiveFilteredPilots(client_fd)) {
                    perror("Failed to receive filtered pilots");
                }
            }
            else if (strcmp(buffer, "transmit") == 0) {
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
            else if (strcmp(buffer, "receive") == 0) {
                // Handle receive command - ADC->MATLAB
                printf("Handling receive command from MATLAB\n");
                if (!handleReceiveCommand(client_fd)) {
                    perror("Receive operation failed");
                }
            }
            else if (strcmp(buffer, "stop") == 0) {
                if (dac_transmitting) {
                    dac_transmitting = false;
                    size_t zeroLength = 1024; 
                    uint32_t *zeroBuf = g_dacZmod->allocChannelsBuffer(zeroLength);
                    
                    if (zeroBuf) {
                        memset(zeroBuf, 0, zeroLength * sizeof(uint32_t));
                        g_dacZmod->setData(zeroBuf, zeroLength);
                        g_dacZmod->start(); 
                        usleep(10000); 
                        g_dacZmod->stop(); 
                        usleep(10000); 
                        g_dacZmod->freeChannelsBuffer(zeroBuf, zeroLength);
                    }
                    
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






