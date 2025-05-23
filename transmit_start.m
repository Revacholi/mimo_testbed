function [txSignal] = transmit_start(client, bitsIn)
% PARAMETERS
M = 16; % Modulation order
rolloff = 0.25; % RRC roll-off factor
span = 8; % RRC filter transient length
Rsamp = 100e6; % DAC sample rate (100 MHz)
Rsym = 10e6; % symbol rate
filter = 'yes'; % opt filter 'yes' or 'no'
plotting = 'yes'; % opt plot 'yes' or 'no'

% Generate pilot sequence for synchronization
% Using Zadoff-Chu sequence which has good correlation properties
pilotLength = 63; % Length of pilot sequence (prime number works well)
pilotRoot = 7; % Root index for Zadoff-Chu sequence (coprime with length)
pilotSeq = generateZadoffChu(pilotRoot, pilotLength);

% Scale pilot sequence to match QAM symbols power
pilotSeq = pilotSeq * sqrt(1/mean(abs(pilotSeq).^2));

% Scale up pilot sequence power to make it more detectable
pilotAmplification = 2; % Boost pilot signal power
pilotSeq = pilotSeq * pilotAmplification;

% 
endPilotRoot = 13; 
endPilotSeq = generateZadoffChu(endPilotRoot, pilotLength);
endPilotSeq = endPilotSeq * sqrt(1/mean(abs(endPilotSeq).^2));
endPilotSeq = endPilotSeq * pilotAmplification; 

% Modulate signal
k = log2(M);
% Reshape data into k-bit symbols for QAM modulation
dataIn = reshape(bitsIn, [], k);
% Convert binary values to decimal values (integers)
decIn = bi2de(dataIn, 'left-msb');
% QAM Modulation
symbols = qammod(decIn, M, 'gray', UnitAveragePower=true);

symbols = [pilotSeq; symbols; endPilotSeq];
disp(['Prepended pilot sequence of length ', num2str(pilotLength), ' to ', ...
      num2str(length(symbols)-2*pilotLength), ' data symbols, and appended end pilot sequence of length ', ...
      num2str(pilotLength)]);

save('pilot_sequence.mat', 'pilotSeq', 'endPilotSeq', 'pilotAmplification');

% Create RRC filtered versions of pilots for TCP transmission to C++
% (This is the added part for filtered pilots)
samplesPerSymbol = Rsamp/Rsym;  % 10 samples per symbol

% Create RRC filter
rrc_filt = rcosdesign(rolloff, span, samplesPerSymbol, 'sqrt');

% Up-sample and filter pilot sequences
pilotSeqUp = upsample(pilotSeq, samplesPerSymbol);
filteredPilot = conv(rrc_filt, pilotSeqUp);

endPilotSeqUp = upsample(endPilotSeq, samplesPerSymbol);
filteredEndPilot = conv(rrc_filt, endPilotSeqUp);

% Send the filtered pilot sequences to C++ via TCP before transmitting the data
% First send a command indicating we're sending filtered pilots
write(client, "filtered_pilots");



% Send the number of samples in filtered start pilot
write(client, int32(length(filteredPilot)));

% Send filtered start pilot (real part then imaginary part)
write(client, single(real(filteredPilot)));
write(client, single(imag(filteredPilot)));

% Send the number of samples in filtered end pilot
write(client, int32(length(filteredEndPilot)));

% Send filtered end pilot (real part then imaginary part)
write(client, single(real(filteredEndPilot)));
write(client, single(imag(filteredEndPilot)));

% Wait for acknowledgment
while (client.NumBytesAvailable == 0)
    pause(0.1);
end
response = read(client, client.NumBytesAvailable, 'uint8');
disp(['Server response after pilots: ', char(response)]);

% Now continue with normal signal transmission
switch filter
    case 'yes'
        % Create RRC filter with exactly 10 samples per symbol (100MHz/10MHz)
        rrc_filt = rcosdesign(rolloff, span, Rsamp/Rsym,'sqrt');
        % up-sample
        symbolsUp = upsample(symbols, Rsamp/Rsym);
        % pulse shaping
        txSignal = conv(rrc_filt,symbolsUp);
    case 'no'
        txSignal = symbols;
end

switch plotting
    case 'yes'
        figure('Name','Transmitter');
        subplot(1,2,1)
        pwelch(txSignal,[],[],[],'centered',Rsamp)
        subplot(1,2,2)
        hold off;
        delete(findobj(gcf, 'Type', 'Legend'));
        h1 = plot((0:length(txSignal)-1)/100, real(txSignal),"b");
        hold on
        h2 = plot((0:length(txSignal)-1)/100, imag(txSignal),"g");
        lgd = legend([h1 h2], {'In-phase', 'Quadrature'}, 'AutoUpdate', 'off');
        title("IQ Data with Start/End Pilot Sequences")
        grid on
        xlabel('Time (us)')
        
        % Highlight pilot sequence areas
        if strcmp(filter, 'yes')
            pilotSamplesPerSymbol = Rsamp/Rsym;
            startPilotEndSample = pilotLength * pilotSamplesPerSymbol + (span-1)*pilotSamplesPerSymbol/2;
            xline(startPilotEndSample/100, '--r', 'Start Pilot End');
            
            dataSymbolCount = length(symbols) - 2*pilotLength;
            endPilotStartSample = (pilotLength + dataSymbolCount) * pilotSamplesPerSymbol + (span-1)*pilotSamplesPerSymbol/2;
            xline(endPilotStartSample/100, '--g', 'End Pilot Start');
            
            % Plot the filtered pilots
            figure('Name', 'Filtered Pilots');
            subplot(2,1,1);
            plot(real(filteredPilot), 'b'); 
            hold on;
            plot(imag(filteredPilot), 'g');
            title('Filtered Start Pilot');
            legend('Real', 'Imaginary');
            
            subplot(2,1,2);
            plot(real(filteredEndPilot), 'b');
            hold on;
            plot(imag(filteredEndPilot), 'g');
            title('Filtered End Pilot');
            legend('Real', 'Imaginary');
        else
            xline(pilotLength, '--r', 'Start Pilot End');
            xline(length(symbols) - pilotLength, '--g', 'End Pilot Start');
        end

        hold off;
    case 'no'
end

% Now start the normal transmission process
write(client, "transmit");

while (client.NumBytesAvailable == 0)
    pause(0.1);
end
response = read(client, client.NumBytesAvailable, 'uint8');
disp(char(response));

write(client, int32(length(txSignal)));

write(client, single(imag(txSignal)));
write(client, single(real(txSignal)));

end

% Function to generate Zadoff-Chu sequence
function seq = generateZadoffChu(u, length)
    % u: root index (relatively prime to length)
    % length: sequence length (prime number is ideal)
    
    % Ensure u and length are coprime
    if gcd(u, length) ~= 1
        %warning('Root index u and length should be coprime for optimal sequence. Using default u=1.');
        u = 1;
    end
    
    % Generate sequence
    n = (0:length-1)';
    if mod(length, 2) == 0
        % Even length
        seq = exp(-1j * pi * u * n .* (n+1) / length);
    else
        % Odd length
        seq = exp(-1j * pi * u * n.^2 / length);
    end
end