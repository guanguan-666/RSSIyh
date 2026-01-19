% STM32 HIL Simulation Script (Corrected Version)
% Purpose: Hardware-in-the-Loop simulation for STM32 PID controller
% Protocol: 
%   - Upload: 0xA5 + Target(4-byte float) + Current(4-byte float) + 0x5A (10 bytes)
%   - Download: 0xA5 + Output(4-byte float) + 0x5A (6 bytes)
% 
% Fix: Corrected float conversion with explicit little-endian handling
%
% Author: Fixed version addressing endianness issues
% Date: 2026-01-19

function stm32_hil_simulation()
    % Clear workspace (optional, for cleaner debugging)
    close all;
    clc;
    
    fprintf('=================================================\n');
    fprintf('  STM32 HIL Simulation (Corrected Float Parser)\n');
    fprintf('=================================================\n\n');
    
    % Configuration
    COM_PORT = 'COM3';  % Change to your serial port
    BAUD_RATE = 115200;
    SIMULATION_STEPS = 50;
    
    % Try to find available serial ports
    try
        ports = serialportlist('available');
        if ~isempty(ports)
            fprintf('Available serial ports: %s\n', strjoin(ports, ', '));
            % Use first available port if COM3 doesn't exist
            if ~ismember(COM_PORT, ports) && ~isempty(ports)
                COM_PORT = ports(1);
                fprintf('Using port: %s\n', COM_PORT);
            end
        end
    catch
        fprintf('Note: Unable to list serial ports. Using default %s\n', COM_PORT);
    end
    
    % Initialize serial port
    try
        % Try new serialport (R2019b+)
        s = serialport(COM_PORT, BAUD_RATE);
        configureTerminator(s, '');
        flush(s);
        use_new_serial = true;
        fprintf('Serial port opened: %s @ %d baud (new API)\n', COM_PORT, BAUD_RATE);
    catch
        % Fall back to legacy serial
        try
            s = serial(COM_PORT, 'BaudRate', BAUD_RATE);
            set(s, 'Timeout', 1);
            fopen(s);
            use_new_serial = false;
            fprintf('Serial port opened: %s @ %d baud (legacy API)\n', COM_PORT, BAUD_RATE);
        catch ME
            fprintf('ERROR: Cannot open serial port %s\n', COM_PORT);
            fprintf('Error message: %s\n', ME.message);
            fprintf('\nPlease check:\n');
            fprintf('1. Port name is correct\n');
            fprintf('2. Port is not in use by another application\n');
            fprintf('3. USB cable is connected\n');
            return;
        end
    end
    
    % Simulation parameters
    target_temp = 25.0;      % Target temperature (setpoint)
    current_temp = 20.0;     % Initial temperature
    pid_outputs = zeros(SIMULATION_STEPS, 1);  % Store PID outputs
    timestamps = zeros(SIMULATION_STEPS, 1);   % Store timestamps
    
    fprintf('\nStarting HIL simulation...\n');
    fprintf('Target Temperature: %.2f°C\n', target_temp);
    fprintf('Initial Temperature: %.2f°C\n\n', current_temp);
    fprintf('%-6s %-12s %-12s %-12s %-15s\n', 'Step', 'Target', 'Current', 'STM32_Output', 'Status');
    fprintf('%s\n', repmat('-', 1, 70));
    
    % Main simulation loop
    for step = 1:SIMULATION_STEPS
        tic;
        
        % 1. Send command to STM32 (Upload frame: 0xA5 + Target + Current + 0x5A)
        tx_frame = create_upload_frame(target_temp, current_temp);
        
        try
            if use_new_serial
                write(s, tx_frame, 'uint8');
            else
                fwrite(s, tx_frame, 'uint8');
            end
        catch ME
            fprintf('ERROR: Failed to send data at step %d: %s\n', step, ME.message);
            continue;
        end
        
        % 2. Wait for response with timeout
        pause(0.05);  % Give STM32 time to process (50ms)
        
        % 3. Receive response from STM32 (Download frame: 0xA5 + Output + 0x5A)
        try
            if use_new_serial
                if s.NumBytesAvailable >= 6
                    rx_data = read(s, 6, 'uint8');
                else
                    % Timeout waiting for data
                    fprintf('%-6d %-12.2f %-12.2f %-12s %-15s\n', step, target_temp, current_temp, 'TIMEOUT', 'No response');
                    continue;
                end
            else
                if s.BytesAvailable >= 6
                    rx_data = fread(s, 6, 'uint8');
                else
                    % Timeout waiting for data
                    fprintf('%-6d %-12.2f %-12.2f %-12s %-15s\n', step, target_temp, current_temp, 'TIMEOUT', 'No response');
                    continue;
                end
            end
            
            % 4. Parse response with corrected float conversion
            [pid_output, valid, debug_info] = parse_stm32_response(rx_data);
            
            if valid
                pid_outputs(step) = pid_output;
                timestamps(step) = toc;
                
                % 5. Update plant model (simple first-order system)
                % current_temp = current_temp + pid_output * 0.01;
                % For realistic simulation, apply actuator limits
                actuator_effect = max(-5, min(5, pid_output * 0.01));
                current_temp = current_temp + actuator_effect;
                
                fprintf('%-6d %-12.2f %-12.2f %-12.2f %-15s\n', step, target_temp, current_temp, pid_output, 'OK');
            else
                fprintf('%-6d %-12.2f %-12.2f %-12s %-15s\n', step, target_temp, current_temp, 'PARSE_ERR', debug_info.error);
            end
            
        catch ME
            fprintf('ERROR: Failed to receive data at step %d: %s\n', step, ME.message);
        end
        
        % Rate limiting (20Hz control loop)
        elapsed = toc;
        if elapsed < 0.05
            pause(0.05 - elapsed);
        end
    end
    
    % Close serial port
    try
        if use_new_serial
            delete(s);
        else
            fclose(s);
            delete(s);
        end
        fprintf('\nSerial port closed.\n');
    catch
        fprintf('\nWarning: Error closing serial port.\n');
    end
    
    % Plot results
    plot_results(pid_outputs, timestamps);
    
    fprintf('\n=================================================\n');
    fprintf('  Simulation completed successfully!\n');
    fprintf('=================================================\n');
end

%% Helper function: Create upload frame
function frame = create_upload_frame(target, current)
    % Frame format: 0xA5 + Target(4-byte float) + Current(4-byte float) + 0x5A
    frame = zeros(10, 1, 'uint8');
    frame(1) = uint8(hex2dec('A5'));  % Header
    
    % Convert floats to little-endian bytes
    target_bytes = float_to_little_endian_bytes(target);
    current_bytes = float_to_little_endian_bytes(current);
    
    frame(2:5) = target_bytes;
    frame(6:9) = current_bytes;
    frame(10) = uint8(hex2dec('5A'));  % Tail
end

%% Helper function: Convert float to little-endian bytes
function bytes = float_to_little_endian_bytes(value)
    % Convert float to IEEE 754 single precision, little-endian
    % MATLAB's typecast is platform-dependent, so we ensure little-endian
    
    % Convert to single precision
    value_single = single(value);
    
    % Convert to uint32
    value_uint32 = typecast(value_single, 'uint32');
    
    % Check system endianness
    [~, ~, endian] = computer;
    
    if endian == 'L'
        % Little-endian system - direct conversion
        bytes = typecast(value_uint32, 'uint8');
    else
        % Big-endian system - need to swap
        bytes = typecast(swapbytes(value_uint32), 'uint8');
    end
    
    % Ensure column vector
    bytes = bytes(:);
end

%% Helper function: Parse STM32 response with corrected float conversion
function [output, valid, debug_info] = parse_stm32_response(data)
    % Initialize output
    output = 0.0;
    valid = false;
    debug_info = struct();
    debug_info.error = 'Unknown';
    debug_info.raw_bytes = data;
    
    % Check frame length
    if length(data) ~= 6
        debug_info.error = sprintf('Invalid length: %d (expected 6)', length(data));
        return;
    end
    
    % Store raw bytes for debugging
    debug_info.raw_hex = sprintf('%02X ', data);
    
    % Check header and tail
    if data(1) ~= hex2dec('A5')
        debug_info.error = sprintf('Invalid header: 0x%02X (expected 0xA5)', data(1));
        return;
    end
    
    if data(6) ~= hex2dec('5A')
        debug_info.error = sprintf('Invalid tail: 0x%02X (expected 0x5A)', data(6));
        return;
    end
    
    % Extract float bytes (bytes 2-5)
    float_bytes = data(2:5);
    
    % *** CRITICAL FIX: Correct little-endian float conversion ***
    % The STM32 sends float in little-endian byte order (LSB first)
    % We need to ensure MATLAB interprets these bytes correctly
    
    % Method 1: Direct typecast (works if MATLAB is on little-endian system)
    % Convert 4 bytes to uint32 first (maintaining byte order)
    value_uint32 = typecast(uint8(float_bytes), 'uint32');
    
    % Check system endianness and adjust if needed
    [~, ~, endian] = computer;
    
    if endian == 'B'
        % Big-endian system - need to swap bytes
        value_uint32 = swapbytes(value_uint32);
    end
    
    % Convert uint32 to single precision float
    output = typecast(value_uint32, 'single');
    
    % Convert to double for MATLAB processing
    output = double(output);
    
    % Store debug info
    debug_info.float_bytes_hex = sprintf('%02X %02X %02X %02X', float_bytes(1), float_bytes(2), float_bytes(3), float_bytes(4));
    debug_info.uint32_value = value_uint32;
    debug_info.float_value = output;
    debug_info.endian = endian;
    
    % Sanity check: typical PID output should be in reasonable range
    % Allow for wider range to accommodate different tuning
    if isnan(output) || isinf(output)
        debug_info.error = sprintf('Invalid float: NaN or Inf');
        valid = false;
        return;
    end
    
    % Additional validation: Check for reasonable PID output range
    % Typical PID outputs are -1000 to +1000, but we'll be generous
    if abs(output) > 10000
        debug_info.warning = sprintf('Large value: %.2f (may be valid)', output);
    end
    
    valid = true;
    debug_info.error = 'None';
end

%% Helper function: Plot results
function plot_results(pid_outputs, timestamps)
    if all(timestamps == 0)
        fprintf('No valid data to plot.\n');
        return;
    end
    
    % Find valid data points
    valid_idx = timestamps > 0;
    valid_outputs = pid_outputs(valid_idx);
    valid_times = cumsum(timestamps(valid_idx));
    
    if isempty(valid_outputs)
        fprintf('No valid data to plot.\n');
        return;
    end
    
    % Create figure
    figure('Name', 'STM32 HIL Simulation Results', 'NumberTitle', 'off');
    
    % Plot PID output over time
    subplot(2,1,1);
    plot(1:length(valid_outputs), valid_outputs, 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Step');
    ylabel('PID Output');
    title('STM32 PID Output Over Time');
    
    % Plot histogram
    subplot(2,1,2);
    histogram(valid_outputs, 20);
    grid on;
    xlabel('PID Output Value');
    ylabel('Frequency');
    title('Distribution of PID Output Values');
    
    % Display statistics
    fprintf('\n--- Statistics ---\n');
    fprintf('Valid samples: %d\n', length(valid_outputs));
    fprintf('Mean output: %.2f\n', mean(valid_outputs));
    fprintf('Std deviation: %.2f\n', std(valid_outputs));
    fprintf('Min output: %.2f\n', min(valid_outputs));
    fprintf('Max output: %.2f\n', max(valid_outputs));
end
