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
Rsamp_adc = 125e6; % ADC sample rate (125 MHz)
Rsym = 10e6; % symbol rate
freq = 1e6; % 1 MHz sine wave

% Generate random binary data
M = 16; % modulation order (M-QAM)
k = log2(M); % number of bits per symbol
numSymbols = 300; % number of symbols
numBits = numSymbols*k; % number of bits
bitsIn = randi([0 1], numBits, 1);

% Create RRC filter with the correct samples per symbol for DAC
samplesPerSymbol_dac = Rsamp_dac/Rsym; % Should be 10
rrc_filt = rcosdesign(rolloff, span, samplesPerSymbol_dac, 'sqrt');

% Start transmission
txSignal = transmit_start(client, bitsIn);
pause(3);
% Stop transmission
transmit_stop(client);
% Recieve signal
rxSignal = receive(client);

% Calculate delay
delay = (length(rrc_filt)-1)/2;
% Trim filtered rxSignal
rxSignal_trimmed = rxSignal(delay+1:end-delay);

% Plot total
figure('Name','Combined TX & RX'), subplot(1,2,1)
pwelch(txSignal,[],[],[],'centered',Rsamp_dac)
hold on
pwelch(rxSignal,[],[],[],'centered',Rsamp_adc)
legend("Transmitted (100 MHz)", "Received (125 MHz)");
title("Power Spectral Density");
xlabel('Frequency (MHz)');
ylabel('Power/Frequency (dB/Hz)');

subplot(1,2,2)
% Calculate time duration for 5 periods (5 microseconds)
timeDuration = 5; % us

% Create time axes using the correct sampling rates
t_tx = (0:(Rsamp_dac*timeDuration*1e-6))/Rsamp_dac * 1e6; % Time in microseconds for TX
t_rx = (0:(Rsamp_adc*timeDuration*1e-6))/Rsamp_adc * 1e6; % Time in microseconds for RX

% Get samples for plotting - limit to available samples and time duration
tx_samples = min(length(txSignal), length(t_tx));
rx_samples = min(length(rxSignal), length(t_rx));

% Plot TX and RX signals with their respective time scales
plot(t_tx(1:tx_samples), real(txSignal(1:tx_samples)), 'b');
hold on
plot(t_tx(1:tx_samples), imag(txSignal(1:tx_samples)), 'g');
plot(t_rx(1:rx_samples), real(rxSignal(1:rx_samples)), 'r');
plot(t_rx(1:rx_samples), imag(rxSignal(1:rx_samples)), 'm');
legend("Re(TX) 100MHz", "Im(TX) 100MHz", "Re(RX) 125MHz", "Im(RX) 125MHz");
title("IQ Data (First 5 Periods)");
grid on
xlabel('Time (us)');
ylabel('Amplitude');

% Add period markers
for i = 1:5
    xline(i/freq*1e6, '--', ['Period ' num2str(i)]);
end

% %% Shutdown
% write(client, "exit");
% response = read(client, client.NumBytesAvailable, 'uint8');
% disp(char(response));