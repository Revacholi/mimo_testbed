%%
clear all; close all;
% SERVER DETAILS
serverIP = '192.168.1.10'; % Change to the server's IP address
%serverIP = "localhost";
serverPort = 8080;    % Change to the server's port

% Create a TCP/IP client
client = tcpclient(serverIP, serverPort);
%client.InputBufferSize = 8192;

flush(client);

rolloff = 0.25; % RRC roll-off factor
span = 25; % RRC filter transient lenght
Rsamp_dac = 100e6; % DAC sample rate (100 MHz)
Rsamp_adc = 105e6; % ADC sample rate 
Rsym = 10e6; % symbol rate

% Generate random binary data
M = 16; % modulation order (M-QAM)
k = log2(M); % number of bits per symbol
numSymbols = 230; % number of symbols
numBits = numSymbols*k; % number of bits
bitsIn = randi([0 1], numBits, 1);

% Create RRC filter with the correct samples per symbol for DAC
samplesPerSymbol_dac = Rsamp_dac/Rsym; % Should be 10
rrc_filt = rcosdesign(rolloff, span, samplesPerSymbol_dac, 'sqrt');

% Start transmission
txSignal = transmit_start(client, bitsIn);
pause(3);
% Recieve signal
rxSignal = receive(client);
pause(3);
% Stop transmission
transmit_stop(client);

% 
% Downsample
rxSymbols = rxSignal((span*Rsamp_adc/Rsym)+1:Rsamp_adc/Rsym:(numSymbols+span)*Rsamp_adc/Rsym);
scatterplot(rxSymbols/0.4);
% QAM Demodulation
dataSymbolsOut = qamdemod(rxSymbols, M, 'gray', UnitAveragePower=true);
% convert decimal values back to binary
dataOutMatrix = de2bi(dataSymbolsOut, k, 'left-msb');
% reshape binary matrix to a vector
dataOut = dataOutMatrix(:);
% calculate the number of bit errors
numErrors = sum(bitsIn ~= dataOut);
disp(['Number of bit errors: ' num2str(numErrors)])
disp(['Bit error rate: ' num2str(numErrors / numBits)])

% Calculate delay
delay = (length(rrc_filt)-1)/2;
% Trim filtered rxSignal
rxSignal_trimmed = rxSignal(delay+1:end-delay);

% Plot total
figure('Name','Total'), subplot(1,2,1)
pwelch(txSignal,[],[],[],'centered',Rsamp_dac)
hold on
pwelch(rxSignal,[],[],[],'centered',Rsamp_adc)
legend("Transmitted", "Recieved");
subplot(1,2,2)
plot((0:length(txSignal)-1)/100, real(txSignal));
hold on
plot((0:length(txSignal)-1)/100, imag(txSignal));
plot((0:length(rxSignal)-1)/100, real(rxSignal));
plot((0:length(rxSignal)-1)/100, imag(rxSignal));
legend("Re(TX)", "Im(TX)", "Re(RX)", "Im(RX)");
title("IQ Data")
grid on
xlabel('Time (us)')
ylabel('Amplitude');

% %% Shutdown
% write(client, "exit");
% response = read(client, client.NumBytesAvailable, 'uint8');
% disp(char(response));