function [rxSignal] = receive(client)
%%%%%%%%% basic receive fuc to get the rxdata %%%%%%%%%%%%%%
    % Parameters
    rolloff = 0.25; % RRC roll-off factor
    span = 20; % RRC filter transient lenght
    Rsamp = 105e6; % ADC sample rate 
    Rsym = 5e6; % symbol rate
    sps = Rsamp/ Rsym;
    SNR = 20;
    plotting = 'yes';
    noise = true;
    filter = true;
    
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
    
    imaginary_data = resample(imaginary_data, 21, 20);
    real_data = resample(real_data, 21, 20);

    % Convert into complex vector
    rxSignal = complex(real_data, imaginary_data);
    % Convert single point float to double
    rxSignal = double(rxSignal);
    
    % Add noise
    if noise
        rxSignal = awgn(rxSignal, SNR, 'measured');
    end
 
%%%%%%%%% basic receive func to get the rxdata %%%%%%%%%%%%%%    
    % Matched filter
    if filter
        rrc_filt = rcosdesign(rolloff, span, Rsamp/Rsym,"sqrt");
        filter_delay = span * sps / 2;
        rxSignal = conv(rrc_filt,rxSignal);
        rxSignal = rxSignal(filter_delay+1 : end-filter_delay);
    end
    
    switch plotting
        case 'yes'
            figure('Name','Reciever'), subplot(1,2,1)
            pwelch(rxSignal,[],[],[],'centered',40e6)
            subplot(1,2,2)
            plot((0:length(rxSignal)-1)/40, real(rxSignal),"b");
            hold on
            plot((0:length(rxSignal)-1)/40, imag(rxSignal),"g");
            legend("In-phase", "Quadrature");
            title("IQ Data")
            grid on
            xlabel('Time (us)')
        case 'no'
    end



end