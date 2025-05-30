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
span = 20; % RRC filter transient lenght
Rsym = 5e6; % symbol rate
fs_rx = 105e6;
fs_tx = 100e6;

sps_rx = fs_rx / Rsym;
sps_tx = fs_tx / Rsym;

% Generate random binary data
M = 16; % modulation order (M-QAM)
k = log2(M); % number of bits per symbol
numSymbols = 100; % number of symbols
numBits = numSymbols*k; % number of bits
data_bits = randi([0 1], numBits, 1);
bitsIn = data_bits;

dataIn = reshape(bitsIn, [], k);
% Convert binary values to decimal values (integers)
decIn = bi2de(dataIn, 'left-msb');
% QAM Modulation
symbols = qammod(decIn, M, 'gray', UnitAveragePower=true);

rrc_rx = rcosdesign(rolloff, span, sps_rx,'sqrt');
rrc_tx = rcosdesign(rolloff, span, sps_tx,'sqrt');


% Start transmission
[txSignal, tx_preamble_waveform, tx_payload_waveform, tx_endpreamble_waveform] = transmit_start(client, bitsIn);
preamble_resampled = resample(tx_preamble_waveform, fs_rx, fs_tx);
matched_preamble = conv(preamble_resampled, rrc_rx, 'same');
payload_resampled = resample(tx_payload_waveform, fs_rx, fs_tx);
matched_payload = conv(payload_resampled, rrc_rx, 'same');
preamble_endresampled = resample(tx_endpreamble_waveform, fs_rx, fs_tx);
matched_endpreamble = conv(preamble_endresampled, rrc_rx, 'same');

pause(3);
% Recieve signal
rxSignal = receive(client);
pause(3);
% Stop transmission
transmit_stop(client);


delay_samples = 0;  % for example, delay by 500 samples
rxSignal = rxSignal(:);
rxSignal = [zeros(delay_samples, 1); rxSignal];

[corr, lags] = xcorr(rxSignal, matched_preamble);
[~, peak_idx] = max(abs(corr));
frame_start = lags(peak_idx);
preamble_len_samples = 63 * sps_rx;
payload_start = frame_start + preamble_len_samples;

aligned = rxSignal(payload_start+1:payload_start+numSymbols*sps_tx);
aligned_downsampled = aligned(1:sps_tx:end);
scatterplot(aligned_downsampled);

% QAM Demodulation
dataSymbolsOut = qamdemod(aligned_downsampled, M, 'gray', UnitAveragePower=true);
% convert decimal values back to binary
dataOutMatrix = de2bi(dataSymbolsOut, k, 'left-msb');
% reshape binary matrix to a vector
dataOut = dataOutMatrix(:);
% calculate the number of bit errors
numErrors = sum(data_bits ~= dataOut);
% numErrors = sum(bitsIn ~= dataOut);
disp(['Number of bit errors: ' num2str(numErrors)])
disp(['Bit error rate: ' num2str(numErrors / numBits)])

% Plot the correlation magnitude
figure;
plot(lags, abs(corr));
hold on;
xline(lags(peak_idx), 'r--', 'LineWidth', 2);
title('Cross-correlation of recieved complex waveform with the matched preamble');
xlabel('Lag');
ylabel('Correlation Magnitude');
legend('Cross-Correlation', 'Detected Frame Start');
grid on;

% Resample tx signal
txSignal = resample(txSignal, sps_rx,sps_tx);
% Normalize tx
txSignal_norm = txSignal / max(abs(txSignal(preamble_len_samples+1:1:end)));
% Normalize aligned
aligned_norm = aligned / max(abs(aligned));
% Plot total
figure('Name','Total'), subplot(1,2,1)
pwelch(txSignal,[],[],[],'centered',40e6)
hold on
pwelch(rxSignal,[],[],[],'centered',40e6)
legend("Transmitted", "Recieved");
subplot(1,2,2)
plot((1:length(txSignal)+frame_start), [zeros(frame_start, 1); real(txSignal_norm)]);
hold on
plot((1:length(txSignal)+frame_start), [zeros(frame_start, 1); imag(txSignal_norm)]);
plot((1:length(aligned)+payload_start), [zeros(payload_start, 1); real(aligned_norm)]);
plot((1:length(aligned)+payload_start), [zeros(payload_start, 1); imag(aligned_norm)]);

xline(payload_start, 'r--', 'LineWidth', 2);

legend("Re(TX)", "Im(TX)", "Re(RX)", "Im(RX)", "Detected Payload Start");
title("IQ Data")
grid on
xlabel('Time (us)')
% %% Shutdown
% write(client, "exit");
% response = read(client, client.NumBytesAvailable, 'uint8');
% disp(char(response));


%%

close all; clear; clc;
load("C:\Users\mail2\Desktop\20.mat");
delay_samples = 0;  % for example, delay by 500 samples
rxSignal = rxSignal(:);
rxSignal = [zeros(delay_samples, 1); rxSignal];

[corr, lags] = xcorr(rxSignal, matched_preamble);
[~, peak_idx] = max(abs(corr));
frame_start = lags(peak_idx);
preamble_len_samples = 63 * sps_rx;
payload_start = frame_start + preamble_len_samples;
aligned = rxSignal(payload_start+1:payload_start+numSymbols*sps_rx);
aligned_downsampled = aligned(1:sps_rx:end);
scatterplot(aligned_downsampled);

% QAM Demodulation
dataSymbolsOut = qamdemod(aligned_downsampled, M, 'gray', UnitAveragePower=true);
% convert decimal values back to binary
dataOutMatrix = de2bi(dataSymbolsOut, k, 'left-msb');
% reshape binary matrix to a vector
dataOut = dataOutMatrix(:);
% calculate the number of bit errors
numErrors = sum(data_bits ~= dataOut);
% numErrors = sum(bitsIn ~= dataOut);
disp(['Number of bit errors: ' num2str(numErrors)])
disp(['Bit error rate: ' num2str(numErrors / numBits)])

% Plot the correlation magnitude
figure;
plot(lags, abs(corr));
hold on;
xline(lags(peak_idx), 'r--', 'LineWidth', 2);
title('Cross-correlation of recieved complex waveform with the matched preamble');
xlabel('Lag');
ylabel('Correlation Magnitude');
legend('Cross-Correlation', 'Detected Frame Start');
grid on;

% Resample tx signal
txSignal = resample(txSignal, sps_rx,sps_tx);
% Normalize tx
txSignal_norm = txSignal / max(abs(txSignal(preamble_len_samples+1:1:end)));
% Normalize aligned
aligned_norm = aligned / max(abs(aligned));
% Plot total
figure('Name','Total'), subplot(1,2,1)
pwelch(txSignal,[],[],[],'centered',40e6)
hold on
pwelch(rxSignal,[],[],[],'centered',40e6)
legend("Transmitted", "Recieved");
subplot(1,2,2)
plot((1:length(txSignal)+frame_start), [zeros(frame_start, 1); real(txSignal_norm)]);
hold on
plot((1:length(txSignal)+frame_start), [zeros(frame_start, 1); imag(txSignal_norm)]);
plot((1:length(aligned)+payload_start), [zeros(payload_start, 1); real(aligned_norm)]);
plot((1:length(aligned)+payload_start), [zeros(payload_start, 1); imag(aligned_norm)]);

xline(payload_start, 'r--', 'LineWidth', 2);

legend("Re(TX)", "Im(TX)", "Re(RX)", "Im(RX)", "Detected Payload Start");
title("IQ Data")
grid on
xlabel('Time (us)')