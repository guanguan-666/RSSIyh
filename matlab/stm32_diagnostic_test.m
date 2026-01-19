% STM32 Float Conversion Diagnostic Script
% Purpose: Test and validate float parsing from STM32 serial data
% This script tests the corrected float conversion logic with known values
%
% Date: 2026-01-19

function stm32_diagnostic_test()
    clear all;
    clc;
    
    fprintf('=================================================\n');
    fprintf('  STM32 Float Conversion Diagnostic Test\n');
    fprintf('=================================================\n\n');
    
    % Test cases from problem statement
    fprintf('Running test cases from problem statement...\n\n');
    
    % Test Case 1: Value 3.64 (from problem statement)
    % Raw bytes: A5 00 00 C8 C2 5A
    test_case_1 = uint8([hex2dec('A5'), hex2dec('00'), hex2dec('00'), ...
                         hex2dec('C8'), hex2dec('C2'), hex2dec('5A')]);
    
    fprintf('Test Case 1: Expected value = 3.64\n');
    fprintf('Raw bytes: ');
    fprintf('%02X ', test_case_1);
    fprintf('\n');
    [value1, valid1, debug1] = parse_stm32_response_diagnostic(test_case_1);
    if valid1
        fprintf('✓ Parsed value: %.2f\n', value1);
        fprintf('  Status: %s\n', abs(value1 - 3.64) < 0.01 ? 'PASS' : 'FAIL');
    else
        fprintf('✗ Parse failed: %s\n', debug1.error);
    end
    fprintf('\n');
    
    % Test Case 2: Value -84.00 (from problem statement)
    % Need to calculate the hex bytes for -84.00
    neg84_bytes = create_test_frame(-84.00);
    
    fprintf('Test Case 2: Expected value = -84.00\n');
    fprintf('Raw bytes: ');
    fprintf('%02X ', neg84_bytes);
    fprintf('\n');
    [value2, valid2, debug2] = parse_stm32_response_diagnostic(neg84_bytes);
    if valid2
        fprintf('✓ Parsed value: %.2f\n', value2);
        fprintf('  Status: %s\n', abs(value2 - (-84.00)) < 0.01 ? 'PASS' : 'FAIL');
    else
        fprintf('✗ Parse failed: %s\n', debug2.error);
    end
    fprintf('\n');
    
    % Test Case 3: Value 0.00 (edge case)
    zero_bytes = create_test_frame(0.00);
    
    fprintf('Test Case 3: Expected value = 0.00\n');
    fprintf('Raw bytes: ');
    fprintf('%02X ', zero_bytes);
    fprintf('\n');
    [value3, valid3, debug3] = parse_stm32_response_diagnostic(zero_bytes);
    if valid3
        fprintf('✓ Parsed value: %.2f\n', value3);
        fprintf('  Status: %s\n', abs(value3 - 0.00) < 0.01 ? 'PASS' : 'FAIL');
    else
        fprintf('✗ Parse failed: %s\n', debug3.error);
    end
    fprintf('\n');
    
    % Test Case 4: Value 100.00 (typical max)
    hundred_bytes = create_test_frame(100.00);
    
    fprintf('Test Case 4: Expected value = 100.00\n');
    fprintf('Raw bytes: ');
    fprintf('%02X ', hundred_bytes);
    fprintf('\n');
    [value4, valid4, debug4] = parse_stm32_response_diagnostic(hundred_bytes);
    if valid4
        fprintf('✓ Parsed value: %.2f\n', value4);
        fprintf('  Status: %s\n', abs(value4 - 100.00) < 0.01 ? 'PASS' : 'FAIL');
    else
        fprintf('✗ Parse failed: %s\n', debug4.error);
    end
    fprintf('\n');
    
    % Test Case 5: Value -50.50 (negative decimal)
    neg50_bytes = create_test_frame(-50.50);
    
    fprintf('Test Case 5: Expected value = -50.50\n');
    fprintf('Raw bytes: ');
    fprintf('%02X ', neg50_bytes);
    fprintf('\n');
    [value5, valid5, debug5] = parse_stm32_response_diagnostic(neg50_bytes);
    if valid5
        fprintf('✓ Parsed value: %.2f\n', value5);
        fprintf('  Status: %s\n', abs(value5 - (-50.50)) < 0.01 ? 'PASS' : 'FAIL');
    else
        fprintf('✗ Parse failed: %s\n', debug5.error);
    end
    fprintf('\n');
    
    % Additional: Test with actual problem bytes if available
    fprintf('=================================================\n');
    fprintf('Testing with raw problem bytes...\n\n');
    
    % From problem: "A5 00 00 20 41 ..." - this should be 10.0 or similar
    problem_bytes = uint8([hex2dec('A5'), hex2dec('00'), hex2dec('00'), ...
                           hex2dec('20'), hex2dec('41'), hex2dec('5A')]);
    
    fprintf('Problem bytes: ');
    fprintf('%02X ', problem_bytes);
    fprintf('\n');
    [valuep, validp, debugp] = parse_stm32_response_diagnostic(problem_bytes);
    if validp
        fprintf('✓ Parsed value: %.2f\n', valuep);
        fprintf('  Hex float bytes: %s\n', debugp.float_bytes_hex);
        fprintf('  uint32: 0x%08X\n', debugp.uint32_value);
    else
        fprintf('✗ Parse failed: %s\n', debugp.error);
    end
    fprintf('\n');
    
    fprintf('=================================================\n');
    fprintf('  Diagnostic test completed!\n');
    fprintf('=================================================\n');
end

%% Helper function: Create test frame with specific float value
function frame = create_test_frame(value)
    % Frame format: 0xA5 + Float(4-byte) + 0x5A
    frame = zeros(6, 1, 'uint8');
    frame(1) = uint8(hex2dec('A5'));  % Header
    
    % Convert float to little-endian bytes
    value_bytes = float_to_little_endian_bytes(value);
    frame(2:5) = value_bytes;
    
    frame(6) = uint8(hex2dec('5A'));  % Tail
end

%% Helper function: Convert float to little-endian bytes
function bytes = float_to_little_endian_bytes(value)
    % Convert float to IEEE 754 single precision, little-endian
    
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

%% Helper function: Parse STM32 response (diagnostic version with verbose output)
function [output, valid, debug_info] = parse_stm32_response_diagnostic(data)
    % Initialize output
    output = 0.0;
    valid = false;
    debug_info = struct();
    debug_info.error = 'Unknown';
    debug_info.raw_bytes = data;
    
    % Check frame length
    if length(data) ~= 6
        debug_info.error = sprintf('Invalid length: %d (expected 6)', length(data));
        fprintf('  [DEBUG] Frame length error: got %d bytes, expected 6\n', length(data));
        return;
    end
    
    % Store raw bytes for debugging
    debug_info.raw_hex = sprintf('%02X ', data);
    
    % Check header and tail
    if data(1) ~= hex2dec('A5')
        debug_info.error = sprintf('Invalid header: 0x%02X (expected 0xA5)', data(1));
        fprintf('  [DEBUG] Header error: 0x%02X\n', data(1));
        return;
    end
    
    if data(6) ~= hex2dec('5A')
        debug_info.error = sprintf('Invalid tail: 0x%02X (expected 0x5A)', data(6));
        fprintf('  [DEBUG] Tail error: 0x%02X\n', data(6));
        return;
    end
    
    % Extract float bytes (bytes 2-5)
    float_bytes = data(2:5);
    fprintf('  [DEBUG] Float bytes extracted: %02X %02X %02X %02X\n', ...
            float_bytes(1), float_bytes(2), float_bytes(3), float_bytes(4));
    
    % *** CRITICAL FIX: Correct little-endian float conversion ***
    % The STM32 sends float in little-endian byte order (LSB first)
    
    % Step 1: Convert 4 bytes to uint32 (maintaining byte order)
    value_uint32 = typecast(uint8(float_bytes), 'uint32');
    fprintf('  [DEBUG] uint32 after typecast: 0x%08X (%u)\n', value_uint32, value_uint32);
    
    % Step 2: Check system endianness and adjust if needed
    [~, ~, endian] = computer;
    fprintf('  [DEBUG] System endianness: %s\n', endian);
    
    if endian == 'B'
        % Big-endian system - need to swap bytes
        fprintf('  [DEBUG] Big-endian detected - swapping bytes\n');
        value_uint32 = swapbytes(value_uint32);
        fprintf('  [DEBUG] uint32 after swap: 0x%08X (%u)\n', value_uint32, value_uint32);
    end
    
    % Step 3: Convert uint32 to single precision float
    output_single = typecast(value_uint32, 'single');
    fprintf('  [DEBUG] Float after typecast: %.6f\n', output_single);
    
    % Step 4: Convert to double for MATLAB processing
    output = double(output_single);
    fprintf('  [DEBUG] Final double value: %.6f\n', output);
    
    % Store debug info
    debug_info.float_bytes_hex = sprintf('%02X %02X %02X %02X', ...
                                         float_bytes(1), float_bytes(2), ...
                                         float_bytes(3), float_bytes(4));
    debug_info.uint32_value = value_uint32;
    debug_info.float_value = output;
    debug_info.endian = endian;
    
    % Sanity check
    if isnan(output) || isinf(output)
        debug_info.error = sprintf('Invalid float: NaN or Inf');
        fprintf('  [DEBUG] Validation failed: NaN or Inf\n');
        valid = false;
        return;
    end
    
    % Additional validation
    if abs(output) > 10000
        debug_info.warning = sprintf('Large value: %.2f (may be valid)', output);
        fprintf('  [DEBUG] Warning: Large value %.2f\n', output);
    end
    
    valid = true;
    debug_info.error = 'None';
    fprintf('  [DEBUG] Validation passed!\n');
end
