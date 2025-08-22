# CyclePeakLookahead (SynthEdit SEM v3)

A per-cycle look-ahead peak detector with ≤30 ms latency.  
- **Always uses the DAW/host sample rate via `getSampleRate()`.**

## Build
```bash
cmake -S . -B build -DSE_SDK_DIR="C:/Path/To/SynthEdit_SDK3"
cmake --build build --config Release
