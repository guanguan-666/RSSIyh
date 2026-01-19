# Quick Start Guide - STM32 HIL Simulation

## Quick Setup (5 minutes)

### 1. Hardware Connection
```
STM32 (PB10/TX) ──→ USB Adapter (RX)
STM32 (PB11/RX) ←── USB Adapter (TX)
STM32 (GND)     ──→ USB Adapter (GND)
```

### 2. Start STM32 HIL Mode
```bash
# Connect to STM32 terminal (PuTTY, screen, etc.)
msh > pid_matlab 1

# You should see:
# [System] LoRa PAUSED.
# ╔════════════════════════════════╗
# ║   MATLAB HIL Mode STARTED      ║
# ╚════════════════════════════════╝
```

### 3. Run MATLAB Simulation
```matlab
% In MATLAB:
>> cd matlab
>> stm32_hil_simulation
```

### 4. Stop When Done
```matlab
% MATLAB: Press Ctrl+C
% STM32 terminal:
msh > pid_matlab 0
```

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Can't open port | Check port name, close other apps |
| No response | Verify STM32 is in HIL mode |
| Parse errors | Check TX/RX wiring (may be swapped) |
| All zeros | Run diagnostic test first |

## Testing Without Hardware

Run diagnostic script to verify float parsing:
```matlab
>> stm32_diagnostic_test
```

Should show all tests PASS.

## Files

- **stm32_hil_simulation.m** - Main HIL simulation
- **stm32_diagnostic_test.m** - Test float parsing
- **README.md** - Full documentation
- **QUICK_START.md** - This file

## Default Configuration

```matlab
COM_PORT = 'COM3';          % Windows
% COM_PORT = '/dev/ttyUSB0'; % Linux
% COM_PORT = '/dev/cu.usbserial-*'; % macOS

BAUD_RATE = 115200;
SIMULATION_STEPS = 50;
```

To change port, edit line ~16 in `stm32_hil_simulation.m`

## What Fixed the Problem?

**Before (WRONG):**
```matlab
val_u32 = typecast(raw_data(idx+1:idx+4), 'uint32');
u = double(typecast(val_u32, 'single'));
% Problem: No endianness handling!
```

**After (CORRECT):**
```matlab
value_uint32 = typecast(uint8(float_bytes), 'uint32');

[~, ~, endian] = computer;
if endian == 'B'
    value_uint32 = swapbytes(value_uint32);
end

output = typecast(value_uint32, 'single');
% Fixed: Explicit little-endian handling!
```

## Protocol Summary

```
Upload (MATLAB→STM32): [A5][Target_4B][Current_4B][5A] = 10 bytes
Download (STM32→MATLAB): [A5][Output_4B][5A] = 6 bytes

Float: IEEE 754, Little-endian
Example: 3.64 = bytes [varies by encoding]
```

## Need Help?

1. Check full README.md for details
2. Run diagnostic test
3. Verify hardware connections
4. Check STM32 firmware is running

## Expected Output

```
Step   Target       Current      STM32_Output Status
1      25.00        20.00        15.32        OK
2      25.00        20.15        14.87        OK
3      25.00        20.30        14.42        OK
...
```

Values should vary and converge to target!
