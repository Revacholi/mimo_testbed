function [rxSignal] = receive(client)
%%%%%%%%% basic receive fuc to get the rxdata %%%%%%%%%%%%%%
    % Parameters
    rolloff = 0.25; % RRC roll-off factor
    span = 8; % RRC filter transient lenght
    Rsamp = 100e6; % ADC sample rate (100 MHz)
    Rsym = 10e6; % symbol rate
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
    
    % Convert into complex vector
    rxSignal = complex(real_data, imaginary_data);
    % Convert single point float to double
    rxSignal = double(rxSignal);
    
    % Add noise
    if noise
        rxSignal = awgn(rxSignal, SNR, 'measured');
    end
 
%%%%%%%%% basic receive func to get the rxdata %%%%%%%%%%%%%%    




%%%%%%%%% Load the pilot sequences used in transmission  %%%%%%%%%%%%%%   

    pilotData = load('pilot_sequence.mat');
    pilotSeq = pilotData.pilotSeq;
    endPilotSeq = pilotData.endPilotSeq;
    pilotLength = length(pilotSeq);
    disp(['Loaded start pilot sequence of length ', num2str(pilotLength)]);
    disp(['Loaded end pilot sequence of length ', num2str(length(endPilotSeq))]);

    % catch
    %     warning('Could not load pilot sequence. Synchronization may be inaccurate.');
    %     % Create a default pilot sequence if file not found
    %     pilotLength = 63;
    %     pilotRoot = 7;
    %     pilotSeq = generateZadoffChu(pilotRoot, pilotLength);
    %     pilotSeq = pilotSeq * sqrt(1/mean(abs(pilotSeq).^2));
    % end
    
    if filter
        % Create RRC filter with exactly 10 samples per symbol (100MHz/10MHz)
        samplesPerSymbol = Rsamp/Rsym; % 10
        rrc_filt = rcosdesign(rolloff, span, samplesPerSymbol, 'sqrt');
        
        % Apply filter
        rxSignal = conv(rrc_filt, rxSignal);
    end
    
    % Perform synchronization using the pilot sequences
    % Create the expected filtered pilot sequences
    if filter
        % Upsample the start pilot
        pilotSeqUp = upsample(pilotSeq, samplesPerSymbol);
        % Filter the upsampled pilot
        refPilot = conv(rrc_filt, pilotSeqUp);
        
        % Upsample the end pilot
        endPilotSeqUp = upsample(endPilotSeq, samplesPerSymbol);
        % Filter the upsampled end pilot
        refEndPilot = conv(rrc_filt, endPilotSeqUp);
        
        disp(['Ref start pilot length: ', num2str(length(refPilot))]);
        disp(['Ref end pilot length: ', num2str(length(refEndPilot))]);
    else
        refPilot = pilotSeq;
        refEndPilot = endPilotSeq;
    end
    
    % Debug: Check pilot energy
    pilotEnergy = sum(abs(refPilot).^2);
    endPilotEnergy = sum(abs(refEndPilot).^2);
    signalEnergy = sum(abs(rxSignal).^2);
    disp(['Reference start pilot energy: ', num2str(pilotEnergy)]);
    disp(['Reference end pilot energy: ', num2str(endPilotEnergy)]);
    disp(['Signal energy: ', num2str(signalEnergy)]);
    disp(['Start pilot to signal energy ratio: ', num2str(pilotEnergy/signalEnergy)]);
    disp(['End pilot to signal energy ratio: ', num2str(endPilotEnergy/signalEnergy)]);
    
    % Manual normalized correlation for start pilot
    [start_corr_result, start_lags] = xcorr(rxSignal, refPilot);
    
    % Since xcorr with 'normalized' doesn't work for different length vectors
    rx_energy = sqrt(sum(abs(rxSignal).^2));
    pilot_energy = sqrt(sum(abs(refPilot).^2));
    norm_factor = rx_energy * pilot_energy;
    
    if norm_factor > 0
        start_corr_result = abs(start_corr_result) / norm_factor;
    end
    
    [start_corr_peak, start_max_idx] = max(start_corr_result);
    start_peak_lag = start_lags(start_max_idx);
    
    % Manual normalized correlation for end pilot
    [end_corr_result, end_lags] = xcorr(rxSignal, refEndPilot);
    
    end_pilot_energy = sqrt(sum(abs(refEndPilot).^2));
    end_norm_factor = rx_energy * end_pilot_energy;
    
    if end_norm_factor > 0
        end_corr_result = abs(end_corr_result) / end_norm_factor;
    end
    
    [end_corr_peak, end_max_idx] = max(end_corr_result);
    end_peak_lag = end_lags(end_max_idx);
    
    % Print correlation results
    disp('Correlation results:');
    disp(['Start pilot correlation: Peak value = ', num2str(start_corr_peak), ...
          ', Peak lag = ', num2str(start_peak_lag)]);
    disp(['End pilot correlation: Peak value = ', num2str(end_corr_peak), ...
          ', Peak lag = ', num2str(end_peak_lag)]);
    
%%%%%%%%%%%%%%%%%% Calculate start and end of data %%%%%%%%%%%%%%%%%%
    % For start pilot, if lag is positive, pilot starts at that position in the signal
    if start_peak_lag > 0
        pilot_start = start_peak_lag;
        % Calculate pilot end considering the filter length
        pilot_end = pilot_start + length(refPilot) - 1;
        data_start = pilot_end + 1;
        disp(['Start pilot detected at sample: ', num2str(pilot_start)]);
        disp(['Start pilot ends at sample: ', num2str(pilot_end)]);
    else
        % If lag is negative, the beginning of our signal contains the pilot
        pilot_start = 1;
        pilot_end = length(refPilot) - abs(start_peak_lag);
        data_start = pilot_end + 1;
        disp(['Start pilot partially detected, starts before sample 1, offset: ', num2str(start_peak_lag)]);
        disp(['Start pilot ends at sample: ', num2str(pilot_end)]);
    end
    
    % For end pilot, calculate where it starts and ends
    if end_peak_lag > 0
        end_pilot_start = end_peak_lag;
        end_pilot_end = end_pilot_start + length(refEndPilot) - 1;
        data_end = end_pilot_start;
        disp(['End pilot detected at sample: ', num2str(end_pilot_start)]);
        disp(['End pilot ends at sample: ', num2str(end_pilot_end)]);
    else
        % If lag is negative, this is unexpected for end pilot
        warning('End pilot correlation produced unexpected negative lag');
        end_pilot_start = length(rxSignal) - length(refEndPilot) + 1;
        end_pilot_end = length(rxSignal);
        data_end = end_pilot_start;
        disp(['End pilot estimated to start at sample: ', num2str(end_pilot_start)]);
        disp(['End pilot estimated to end at sample: ', num2str(end_pilot_end)]);
    end
    
    % Make sure indices are within bounds
    pilot_start = min(max(1, pilot_start), length(rxSignal));
    data_start = min(max(1, data_start), length(rxSignal));
    end_pilot_start = min(max(1, end_pilot_start), length(rxSignal));
    data_end = min(max(1, data_end), length(rxSignal));
    
    % Handle case where end pilot is detected before start pilot (due to phase differences)
    if data_end <= data_start
        disp(['End pilot detected before start pilot (at ', num2str(end_pilot_start), ' vs ', num2str(pilot_start), ')']);
        disp('This is normal due to phase differences or signal wrapping. Processing will continue.');
        
        % In this case, we have two options:
        % 1. Use data from start pilot to end of signal (simplest approach)
        % 2. Use data from start pilot to end of signal + beginning of signal to end pilot (more complex)
        
        % For now, just use the simple approach
        disp(['Using data from start pilot (', num2str(data_start), ') to end of signal']);
        data_end = length(rxSignal);
    end
    
    % Extract the data portion between pilots
    if data_start < data_end && data_start < length(rxSignal) && data_end > 1
        disp(['Extracting data portion from sample ', num2str(data_start), ' to ', num2str(data_end)]);
        data_portion = rxSignal(data_start:data_end);
    else
        warning('Could not extract valid data portion. Using entire signal.');
        data_portion = rxSignal;
    end
    
%%%%%%%%%%%%%%%%%%% Reorganize signal to put complete pilot at the beginning %%%%%%%%%%%%%%%%%%
    if pilot_start > 1
        % Print debug information about the pilot position
        disp(['Start pilot detected at position: ', num2str(pilot_start), ' of ', num2str(length(rxSignal))]);
        disp(['Pilot length: ', num2str(length(refPilot)), ' samples']);
        
        % Extract the entire start pilot sequence
        pilot_section = rxSignal(pilot_start:pilot_end);
        disp(['Extracted start pilot section length: ', num2str(length(pilot_section))]);
        
        % Extract segment from detected pilot position to end of signal (including pilot)
        if pilot_start < length(rxSignal)
            segment_from_pilot = rxSignal(pilot_start:end);
            segment_from_pilot = segment_from_pilot(:); % Ensure column vector
            
            % Extract segment from beginning to just before pilot
            if pilot_start > 1
                segment_before_pilot = rxSignal(1:pilot_start-1);
                segment_before_pilot = segment_before_pilot(:); % Ensure column vector
                
                % Direct approach to identify silent regions:
                % 1. Calculate moving average of absolute signal values
                % 2. Apply a very strict threshold to identify silent regions
                
                % Calculate the signal envelope (using absolute values and smoothing)
                window_len = 10; % Shorter window for more precise detection
                abs_signal = abs(segment_before_pilot);
                signal_env = movmean(abs_signal, window_len);
                
                % Determine a dynamic threshold based on signal characteristics
                % Using a very low percentage of mean signal level
                mean_signal = mean(abs_signal(abs_signal > 0.01*max(abs_signal)));
                threshold = 0.05 * mean_signal; % 5% of mean active signal level
                
                % Identify silent samples
                silent_mask = signal_env < threshold;
                
                % Add buffer regions around silent areas - safely handle array lengths
                buffer_size = min(5, length(silent_mask)); % Ensure buffer size doesn't exceed array length
                for i = 1:buffer_size
                    % Safely add buffer before silent regions
                    if length(silent_mask) > i
                        temp1 = [false(i,1); silent_mask(1:end-i)];
                        silent_mask = silent_mask | temp1;
                    end
                    
                    % Safely add buffer after silent regions
                    if length(silent_mask) > i
                        temp2 = [silent_mask(i+1:end); false(i,1)];
                        silent_mask = silent_mask | temp2;
                    end
                end
                
                % Find runs of silent samples
                silent_runs = find(diff([0; silent_mask; 0]) ~= 0);
                if length(silent_runs) >= 2 % At least one silent region
                    silent_start = silent_runs(1:2:end-1);
                    silent_end = silent_runs(2:2:end);
                    
                    for i = 1:length(silent_start)
                        run_length = silent_end(i) - silent_start(i);
                        if run_length > 50 % Only report significant silent regions
                            disp(['Silent region: ', num2str(silent_start(i)), '-', ...
                                  num2str(silent_end(i)), ' (', num2str(run_length), ' samples)']);
                        end
                    end
                end
                
                % Remove all silent samples
                active_samples = ~silent_mask;
                segment_before_pilot_cleaned = segment_before_pilot(active_samples);
                
                disp(['Original segment_before_pilot length: ', num2str(length(segment_before_pilot))]);
                disp(['Removed ', num2str(sum(silent_mask)), ' silent samples']);
                disp(['Cleaned segment_before_pilot length: ', num2str(length(segment_before_pilot_cleaned))]);
                
                % Use the cleaned segment
                segment_before_pilot = segment_before_pilot_cleaned;
            else
                segment_before_pilot = [];
            end
            
            % Now let's also clean up the segment_from_pilot to remove any silent regions
            
            % Calculate envelope for from_pilot segment
            abs_from_pilot = abs(segment_from_pilot);
            from_pilot_env = movmean(abs_from_pilot, window_len);
            
            % Use the same threshold as before
            from_pilot_silent = from_pilot_env < threshold;
            
            % Add buffer regions - safely handle array lengths
            buffer_size = min(5, length(from_pilot_silent)); % Ensure buffer size doesn't exceed array length
            for i = 1:buffer_size
                % Safely add buffer before silent regions
                if length(from_pilot_silent) > i
                    temp1 = [false(i,1); from_pilot_silent(1:end-i)];
                    from_pilot_silent = from_pilot_silent | temp1;
                end
                
                % Safely add buffer after silent regions
                if length(from_pilot_silent) > i
                    temp2 = [from_pilot_silent(i+1:end); false(i,1)];
                    from_pilot_silent = from_pilot_silent | temp2;
                end
            end
            
            % But protect the pilot sequence area - never remove samples from pilot
            protected_length = min(length(segment_from_pilot), pilotLength * samplesPerSymbol);
            from_pilot_silent(1:protected_length) = false;
            
            % Find and report significant silent regions
            silent_runs = find(diff([0; from_pilot_silent; 0]) ~= 0);
            if length(silent_runs) >= 2
                silent_start = silent_runs(1:2:end-1);
                silent_end = silent_runs(2:2:end);
                
                for i = 1:length(silent_start)
                    run_length = silent_end(i) - silent_start(i);
                    if run_length > 50
                        disp(['Silent region in from_pilot: ', num2str(silent_start(i)), '-', ...
                              num2str(silent_end(i)), ' (', num2str(run_length), ' samples)']);
                    end
                end
            end
            
            % Remove silent samples from from_pilot
            from_pilot_active = ~from_pilot_silent;
            segment_from_pilot_cleaned = segment_from_pilot(from_pilot_active);
            
            disp(['Original segment_from_pilot length: ', num2str(length(segment_from_pilot))]);
            disp(['Removed ', num2str(sum(from_pilot_silent)), ' silent samples']);
            disp(['Cleaned segment_from_pilot length: ', num2str(length(segment_from_pilot_cleaned))]);
            
            % Use the cleaned segment
            segment_from_pilot = segment_from_pilot_cleaned;
            
            % Combine all segments
            % Start with start pilot + data up to end of signal
            reorg_signal = segment_from_pilot;
            
            % Add segment before start pilot if available
            if ~isempty(segment_before_pilot)
                reorg_signal = [reorg_signal; segment_before_pilot];
            end
            
            disp(['Final reorganized signal length: ', num2str(length(reorg_signal))]);
        else
            % If pilot detected at end, just use original signal
            reorg_signal = rxSignal;
            warning('Pilot detected at very end of signal. Using original signal.');
        end
    else
        % If start pilot is already at beginning, no need to reorganize extensively
        % Still need to check for the end pilot placement
        reorg_signal = rxSignal;
        disp('Start pilot sequence already at beginning of signal');
    end
    
    % Debug - check reorganized signal length
    disp(['Original signal length: ', num2str(length(rxSignal))]);
    disp(['Reorganized signal length: ', num2str(length(reorg_signal))]);
    
    % Ensure reorganized signal and original signal have same length
    if length(reorg_signal) ~= length(rxSignal)
        warning(['Signal length mismatch after reorganization. ', ...
                'Original: ', num2str(length(rxSignal)), ...
                ', Reorganized: ', num2str(length(reorg_signal))]);
                
        % Make sure lengths match
        if length(reorg_signal) < length(rxSignal)
            reorg_signal = [reorg_signal; zeros(length(rxSignal) - length(reorg_signal), 1)];
        else
            reorg_signal = reorg_signal(1:length(rxSignal));
        end
    end
    
    % Return the reorganized signal
    synced_signal = reorg_signal;
    
    disp(['Start pilot peak lag: ', num2str(start_peak_lag)]);
    disp(['Start pilot start index: ', num2str(pilot_start)]);
    disp(['Data start index: ', num2str(data_start)]);
    disp(['End pilot peak lag: ', num2str(end_peak_lag)]);
    disp(['End pilot start index: ', num2str(end_pilot_start)]);
    disp(['Data end index: ', num2str(data_end)]);
    disp(['Valid data length: ', num2str(data_end - data_start)]);
    disp(['Reorganized signal length: ', num2str(length(reorg_signal))]);

    if strcmp(plotting, 'yes')
        figure('Name','Receiver')
        
        % Plot correlation results for debugging
        subplot(2,1,1)
        plot(start_lags, start_corr_result);
        title('Start Pilot Correlation');
        hold on;
        % Mark the peak
        plot(start_peak_lag, start_corr_peak, 'ro', 'MarkerSize', 10);
        grid on;
        xlabel('Lag');
        ylabel('Magnitude');
        
        % Plot original signal with detected position
        subplot(2,1,2)
        plot((0:length(rxSignal)-1)/100, real(rxSignal),"b");
        hold on
        plot((0:length(rxSignal)-1)/100, imag(rxSignal),"g");
        
        % Add vertical lines for start pilot and data
        if start_peak_lag > 0
            xline(pilot_start/100, '--m', 'Start Pilot Start');
            xline(pilot_end/100, '--c', 'Start Pilot End');
        end
        
        % Add vertical line at data start point
        if data_start > 1 && data_start < length(rxSignal)
            xline(data_start/100, '--r', 'Data Start');
        end
        
        % Add vertical lines for end pilot
        if end_pilot_start > 1 && end_pilot_start < length(rxSignal)
            xline(end_pilot_start/100, '--g', 'End Pilot Start');
            if end_pilot_end < length(rxSignal)
                xline(end_pilot_end/100, '--y', 'End Pilot End');
            end
        end
        
        title('Original Signal with Detected Positions');
        legend("In-phase", "Quadrature");
        grid on
        xlabel('Time (us)')
        
        % Create additional figure to show reorganized signal
        figure('Name','Reorganized Signal')
        subplot(1,2,1)
        pwelch(reorg_signal,[],[],[],'centered',Rsamp)
        title('Reorganized Signal Spectrum')
        
        subplot(1,2,2)
        plot((0:length(reorg_signal)-1)/100, real(reorg_signal),"b");
        hold on
        plot((0:length(reorg_signal)-1)/100, imag(reorg_signal),"g");
        
        % Mark the pilot positions in reorganized signal
        % The start pilot should now be at the beginning
        reorg_pilot_end = length(pilot_section);
        if reorg_pilot_end < length(reorg_signal)
            xline(0, '--m', 'Start Pilot Start (reorg)');
            xline(reorg_pilot_end/100, '--c', 'Start Pilot End (reorg)');
        end
        
        % Mark the end pilot if it's within the reorganized signal
        % This is more complex now that we've reorganized the signal, so we'll approximate
        if end_pilot_start > pilot_start
            % End pilot was after start pilot in original signal
            reorg_end_pilot_start = end_pilot_start - pilot_start + 1;
            reorg_end_pilot_end = reorg_end_pilot_start + length(refEndPilot) - 1;
            
            if reorg_end_pilot_start < length(reorg_signal)
                xline(reorg_end_pilot_start/100, '--g', 'End Pilot Start (reorg)');
                if reorg_end_pilot_end < length(reorg_signal)
                    xline(reorg_end_pilot_end/100, '--y', 'End Pilot End (reorg)');
                end
            end
        else
            % End pilot was before start pilot in original signal
            % It could be either at the beginning or the end of the reorganized signal
            % For simplicity, let's check for correlation in the reorganized signal
            [reorg_end_corr, reorg_end_lags] = xcorr(reorg_signal, refEndPilot);
            reorg_end_norm_factor = sqrt(sum(abs(reorg_signal).^2)) * end_pilot_energy;
            
            if reorg_end_norm_factor > 0
                reorg_end_corr = abs(reorg_end_corr) / reorg_end_norm_factor;
            end
            
            [~, reorg_end_max_idx] = max(reorg_end_corr);
            reorg_end_peak_lag = reorg_end_lags(reorg_end_max_idx);
            
            if reorg_end_peak_lag > 0
                reorg_end_pilot_start = reorg_end_peak_lag;
                reorg_end_pilot_end = reorg_end_pilot_start + length(refEndPilot) - 1;
                
                if reorg_end_pilot_start < length(reorg_signal)
                    xline(reorg_end_pilot_start/100, '--g', 'End Pilot Start (reorg)');
                    if reorg_end_pilot_end < length(reorg_signal)
                        xline(reorg_end_pilot_end/100, '--y', 'End Pilot End (reorg)');
                    end
                end
            end
        end
        
        % Remove the legend from this plot
        legend off;
        
        % Analyze signal for silent regions (near-zero values)
        window_size = 100;
        num_windows = floor(length(reorg_signal) / window_size);
        window_energy = zeros(num_windows, 1);
        
        for i = 1:num_windows
            start_idx = (i-1)*window_size + 1;
            end_idx = min(start_idx + window_size - 1, length(reorg_signal));
            window_data = reorg_signal(start_idx:end_idx);
            window_energy(i) = sum(abs(window_data).^2);
        end
        
        % Set threshold for silent period detection (1% of max energy)
        energy_threshold = 0.01 * max(window_energy);
        
        % Find windows with energy below threshold
        silent_windows = find(window_energy < energy_threshold);
        
        % Mark silent regions on plot
        for i = 1:length(silent_windows)
            start_pos = (silent_windows(i)-1)*window_size/100;
            xline(start_pos, '--m');
        end
        
        % Set axis limits to match original plot for better comparison
        xlim([0 length(reorg_signal)/100]); % Same x-axis range as original signal
        ylim([-0.8 0.8]); % Set y-axis range
        
        title("Reorganized Signal (Pilot at Beginning)")
        grid on
        xlabel('Time (us)')
        
    end
    
    % Return the synchronized signal
    rxSignal = synced_signal;
end