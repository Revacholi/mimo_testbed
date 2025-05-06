function [txSignal] = transmit_start(client, bitsIn)
% PARAMETERS
M = 16; % Modulation order
rolloff = 0.25; % RRC roll-off factor
span = 25; % RRC filter transient lenght
Rsamp = 100e6; % DAC sample rate (100 MHz)
Rsym = 10e6; % symbol rate
filter = 'yes'; % opt filter 'yes' or 'no'
plotting = 'yes'; % opt plot 'yes' or 'no'

% Generate sinusoidal test signal
numSamples = 300; % number of symbols
freq = 1e6; % 1 MHz sine wave
phase = pi/4; % 45 degree phase shift

% Create sine wave symbols (real + imaginary components)
t = (0:numSamples-1)';
symbols = 0.5 * cos(2*pi*freq*t/Rsym + phase) + 1i * 0.5 * sin(2*pi*freq*t/Rsym + phase);

% % Original QAM modulation code (commented out)
% k = log2(M);
% % Reshape data into k-bit symbols for QAM modulation
% dataIn = reshape(bitsIn, [], k);
% % Convert binary values to decimal values (integers)
% decIn = bi2de(dataIn, 'left-msb');
% % QAM Modulation
% symbols = qammod(decIn, M, 'gray', UnitAveragePower=true);

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
        figure('Name','Transmitter'), subplot(1,2,1)
        pwelch(txSignal,[],[],[],'centered',Rsamp)
        subplot(1,2,2)
        
        % Calculate the period length in samples
        samplesPerPeriod = Rsamp/freq;
        % Display exactly 5 periods
        plotSamples = min(length(txSignal), ceil(5*samplesPerPeriod));
        
        % Plot only the first 5 periods
        t_plot = (0:plotSamples-1)/Rsamp * 1e6; % Time in microseconds
        plot(t_plot, real(txSignal(1:plotSamples)),"b");
        hold on
        plot(t_plot, imag(txSignal(1:plotSamples)),"g");
        legend("In-phase", "Quadrature");
        title("IQ Data (First 5 Periods) - DAC 100MHz");
        grid on
        xlabel('Time (us)');
        ylabel('Amplitude');
        % Add period markers
        for i = 1:5
            xline(i/freq*1e6, '--', ['Period ' num2str(i)]);
        end
    case 'no'
end

write(client, "transmit");

while (client.NumBytesAvailable == 0)
    pause(0.1);
end
response = read(client, client.NumBytesAvailable, 'uint8');
disp(char(response));

write(client, int32(length(txSignal)));

write(client, single(imag(txSignal)));
write(client, single(real(txSignal)));

%single(txSignal)
%response = read(client, client.NumBytesAvailable, 'uint8');

%write(client, double(txSignal));

end