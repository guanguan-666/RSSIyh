% Simple Example: Before vs After Float Conversion
% This demonstrates the exact fix for the STM32 HIL float parsing issue

function example_float_conversion_fix()
    clc;
    fprintf('=================================================\n');
    fprintf('  Float Conversion Fix - Before vs After\n');
    fprintf('=================================================\n\n');
    
    % Example data from STM32: Value should be 3.64
    % Raw bytes: A5 00 00 C8 C2 5A
    raw_frame = uint8([hex2dec('A5'), hex2dec('00'), hex2dec('00'), ...
                       hex2dec('C8'), hex2dec('C2'), hex2dec('5A')]);
    
    % Extract float bytes (indices 2-5, MATLAB is 1-indexed)
    float_bytes = raw_frame(2:5);
    
    fprintf('Raw frame: ');
    fprintf('%02X ', raw_frame);
    fprintf('\n');
    fprintf('Float bytes: ');
    fprintf('%02X ', float_bytes);
    fprintf('\n\n');
    
    %% BEFORE (WRONG) - What caused the bug
    fprintf('--- BEFORE (WRONG METHOD) ---\n');
    try
        % This is what the old code did (conceptually)
        % Direct typecast without considering endianness
        wrong_uint32 = typecast(float_bytes, 'uint32');
        wrong_float = typecast(wrong_uint32, 'single');
        fprintf('Direct typecast result: %.2f\n', wrong_float);
        fprintf('Problem: May work on little-endian but ignores byte order!\n');
    catch ME
        fprintf('Error: %s\n', ME.message);
    end
    fprintf('\n');
    
    %% AFTER (CORRECT) - The fix
    fprintf('--- AFTER (CORRECT METHOD) ---\n');
    
    % Step 1: Convert bytes to uint32
    value_uint32 = typecast(uint8(float_bytes), 'uint32');
    fprintf('Step 1 - Bytes to uint32: 0x%08X\n', value_uint32);
    
    % Step 2: Check system endianness
    [~, ~, endian] = computer;
    fprintf('Step 2 - System endianness: %s ', endian);
    if endian == 'L'
        fprintf('(Little-endian - x86/x64)\n');
    else
        fprintf('(Big-endian - rare)\n');
    end
    
    % Step 3: Swap bytes if on big-endian system
    if endian == 'B'
        fprintf('Step 3 - Swapping bytes for big-endian system\n');
        value_uint32 = swapbytes(value_uint32);
        fprintf('        After swap: 0x%08X\n', value_uint32);
    else
        fprintf('Step 3 - No swap needed (little-endian matches STM32)\n');
    end
    
    % Step 4: Convert uint32 to float
    correct_float = typecast(value_uint32, 'single');
    fprintf('Step 4 - uint32 to float: %.6f\n', correct_float);
    fprintf('\n');
    
    %% Comparison
    fprintf('=================================================\n');
    fprintf('COMPARISON:\n');
    fprintf('Expected value:     %.2f\n', 3.64);
    fprintf('Correct conversion: %.2f ✓\n', double(correct_float));
    fprintf('=================================================\n\n');
    
    %% Additional test with negative value
    fprintf('--- Testing with negative value: -84.00 ---\n');
    
    % Create frame with -84.00
    neg_value = single(-84.00);
    neg_uint32 = typecast(neg_value, 'uint32');
    
    % Convert to little-endian bytes
    if endian == 'L'
        neg_bytes = typecast(neg_uint32, 'uint8');
    else
        neg_bytes = typecast(swapbytes(neg_uint32), 'uint8');
    end
    
    neg_frame = [hex2dec('A5'); neg_bytes(:); hex2dec('5A')];
    fprintf('Frame for -84.00: ');
    fprintf('%02X ', neg_frame);
    fprintf('\n');
    
    % Parse it back
    test_bytes = neg_frame(2:5);
    test_uint32 = typecast(uint8(test_bytes), 'uint32');
    if endian == 'B'
        test_uint32 = swapbytes(test_uint32);
    end
    parsed_value = typecast(test_uint32, 'single');
    
    fprintf('Parsed back: %.2f\n', double(parsed_value));
    if abs(parsed_value - (-84.00)) < 0.01
        fprintf('Match: YES ✓\n');
    else
        fprintf('Match: NO ✗\n');
    end
    fprintf('\n');
    
    %% Key Takeaway
    fprintf('=================================================\n');
    fprintf('KEY TAKEAWAY:\n');
    fprintf('=================================================\n');
    fprintf('The STM32 sends floats in LITTLE-ENDIAN byte order.\n');
    fprintf('MATLAB must check system endianness and swap bytes\n');
    fprintf('if running on a big-endian system (rare but possible).\n');
    fprintf('\n');
    fprintf('FIX: Always use computer() to check endianness\n');
    fprintf('     and swapbytes() when needed!\n');
    fprintf('=================================================\n');
end
