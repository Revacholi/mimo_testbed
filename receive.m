function [rxSignal] = receive(client)
    % Parameters
    rolloff = 0.25; % RRC roll-off factor
    span = 25; % RRC filter transient lenght
    Rsamp = 125e6; % ADC sample rate (125 MHz)
    Rsym = 10e6; % symbol rate
    SNR = 20;
    plotting = 'yes';
    noise = false;
    filter = true;
    freq = 1e6; % 1 MHz sine wave (same as transmitter)
    
    % Signal the server to recieve
    write(client, "receive");
    
    % Wait for initial response
    while (client.NumBytesAvailable == 0)
        pause(0.1);
    end
    response = char(read(client, client.NumBytesAvailable, 'uint8'));
    disp(['Server response: ' response]);
    
    % Wait for sample count string
    while (client.NumBytesAvailable == 0)
        pause(0.1);
    end
    
    % Read sample count string "SAMPLES=xxxx"
    samplesStr = char(read(client, client.NumBytesAvailable, 'uint8'));
    disp(['Sample string: ' samplesStr]);
    
    % Extract the number from the string
    parts = split(samplesStr, '=');
    if length(parts) == 2
        numFloats = str2double(parts{2});
        disp(['Expecting ', num2str(numFloats), ' floats.']);
    else
        error('Invalid sample count format received from server');
    end
    
    % Increase the input buffer size to accommodate more samples
    if client.InputBufferSize < numFloats * 4
        client.InputBufferSize = numFloats * 4 * 2; % Double it to be safe
        disp(['Increased buffer size to ', num2str(client.InputBufferSize), ' bytes']);
    end
    
    % Wait for real data
    bytesNeeded = numFloats * 4;  % 4 bytes per float
    while (client.NumBytesAvailable < bytesNeeded)
        pause(0.1);
        disp(['Waiting for real data: ' num2str(client.NumBytesAvailable) '/' num2str(bytesNeeded) ' bytes available']);
    end
    
    real_data = fread(client, numFloats, 'single'); % Read real part
    disp(['Real part samples received: ' num2str(length(real_data))]);
    
    % Wait for imaginary data
    while (client.NumBytesAvailable < bytesNeeded)
        pause(0.1);
        disp(['Waiting for imaginary data: ' num2str(client.NumBytesAvailable) '/' num2str(bytesNeeded) ' bytes available']);
    end
    
    imaginary_data = fread(client, numFloats, 'single'); % Read imaginary part
    disp(['Imaginary part samples received: ' num2str(length(imaginary_data))]);
    
    % Convert into complex vector
    rxSignal = complex(real_data, imaginary_data);
    
    % Convert single point float to double
    rxSignal = double(rxSignal);
    
    % Add noise
    if noise
        rxSignal = awgn(rxSignal, SNR, 'measured');
    end
    
    % Matched filter
    if filter
        % Custom RRC filter implementation for non-integer samples per symbol
        samplesPerSymbol = Rsamp/Rsym; % 12.5
        disp(['Samples per symbol: ', num2str(samplesPerSymbol)]);
        
        % Method 1: Create RRC filter with an integer SPS and then interpolate
        intSPS = 12; % Integer samples per symbol close to 12.5
        rrc_temp = rcosdesign(rolloff, span, intSPS, 'sqrt');
        
        % Create time vectors for original and interpolated filter
        t_orig = (0:length(rrc_temp)-1)/intSPS;
        t_interp = (0:(span*samplesPerSymbol))/samplesPerSymbol;
        
        % Interpolate the filter to the exact SPS
        rrc_filt = interp1(t_orig, rrc_temp, t_interp, 'spline');
        
        % Normalize filter energy
        rrc_filt = rrc_filt / sqrt(sum(rrc_filt.^2));
        
        % Apply filter
        rxSignal = conv(rrc_filt, rxSignal);
    end
    
    switch plotting
        case 'yes'
            figure('Name','Receiver'), subplot(1,2,1)
            pwelch(rxSignal,[],[],[],'centered', Rsamp)
            subplot(1,2,2)
            
            % Calculate the period length in samples
            samplesPerPeriod = Rsamp/freq;
            
            % Display exactly 5 periods
            plotSamples = min(length(rxSignal), ceil(5*samplesPerPeriod));
            
            % Plot only the first 5 periods
            t_plot = (0:plotSamples-1)/Rsamp * 1e6; % Time in microseconds
            plot(t_plot, real(rxSignal(1:plotSamples)),"b");
            hold on
            plot(t_plot, imag(rxSignal(1:plotSamples)),"g");
            legend("In-phase", "Quadrature");
            title("IQ Data (First 5 Periods) - ADC 125MHz");
            grid on
            xlabel('Time (us)');
            ylabel('Amplitude');
            
            % Add period markers
            for i = 1:5
                xline(i/freq*1e6, '--', ['Period ' num2str(i)]);
            end
        case 'no'
    end
end