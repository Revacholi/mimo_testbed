function [txSignal,tx_preamble_waveform, tx_payload_waveform, tx_endpreamble_waveform] = transmit_start(client, bitsIn)
% PARAMETERS
M = 16; % Modulation order
rolloff = 0.25; % RRC roll-off factor
span = 20; % RRC filter transient length
Rsamp = 100e6; % DAC sample rate (100 MHz)
Rsym = 5e6; % symbol rate
filter = 'yes'; % opt filter 'yes' or 'no'
plotting = 'yes'; % opt plot 'yes' or 'no'
sps = Rsamp/ Rsym;
% Modulate signal
k = log2(M);


% Reshape data into k-bit symbols for QAM modulation
dataIn = reshape(bitsIn, [], k);
% Convert binary values to decimal values (integers)
decIn = bi2de(dataIn, 'left-msb');

% QAM Modulation
symbols = qammod(decIn, M, 'gray', UnitAveragePower=true);
% Load in pilot sequence
load("pilot_sequence.mat");
preamble_sym = pilotSeq;
endpreamble_sym = endPilotSeq;

switch filter
    case 'yes'
        % Create RRC filter
        rrc_filt = rcosdesign(rolloff, span, Rsamp/Rsym,"sqrt");
        filter_delay = span * sps / 2;

        % Separate preamble and payload upsampling
        preamble_upsampled = upsample(preamble_sym, sps);
        payload_upsampled = upsample(symbols, sps);
        endpreamble_upsampled = upsample(endpreamble_sym, sps);

        % Apply pulse shaping
        tx_preamble_waveform = conv(preamble_upsampled, rrc_filt);  % Full waveform
        tx_payload_waveform = conv(payload_upsampled, rrc_filt);
        tx_endpreamble_waveform = conv(endpreamble_upsampled, rrc_filt);

        % Remove filter delay (optional: keep for alignment purposes)
        tx_preamble_waveform = tx_preamble_waveform(filter_delay+1 : end-filter_delay);
        tx_payload_waveform = tx_payload_waveform(filter_delay+1 : end-filter_delay);
        tx_endpreamble_waveform = tx_endpreamble_waveform(filter_delay+1 : end-filter_delay);

        % Concatenate full signal
        txSignal = [tx_preamble_waveform; tx_payload_waveform; tx_endpreamble_waveform];


        % Send the filtered pilot sequences to C++ via TCP before transmitting the data
        % First send a command indicating we're sending filtered pilots
        write(client, "filtered_pilots");
        

        % Send the number of samples in filtered start pilot
        write(client, int32(length(tx_preamble_waveform)));
        
        % Send filtered start pilot (real part then imaginary part)
        write(client, single(real(tx_preamble_waveform)));
        write(client, single(imag(tx_preamble_waveform)));
        
        % Send the number of samples in filtered end pilot
        write(client, int32(length(tx_endpreamble_waveform)));
        
        % Send filtered end pilot (real part then imaginary part)
        write(client, single(real(tx_endpreamble_waveform)));
        write(client, single(imag(tx_endpreamble_waveform)));
        
        % Wait for acknowledgment
        while (client.NumBytesAvailable == 0)
        pause(0.1);
        end
        response = read(client, client.NumBytesAvailable, 'uint8');
        disp(['Server response after pilots: ', char(response)]);

    case 'no'
        txSignal = symbols;
end



switch plotting
    case 'yes'
        figure('Name','Transmitter'), subplot(1,2,1)
        pwelch(txSignal,[],[],[],'centered',40e6)
        subplot(1,2,2)
        plot((0:length(txSignal)-1)/40, real(txSignal),"b");
        hold on
        plot((0:length(txSignal)-1)/40, imag(txSignal),"g");
        legend("In-phase", "Quadrature");
        title("IQ Data")
        grid on
        xlabel('Time (us)')
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

