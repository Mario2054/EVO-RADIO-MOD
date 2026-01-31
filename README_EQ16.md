# ESP32 Radio EVO 3.19 - New EQ16 + Analyzer Architecture

## Overview
This document describes the new modular architecture for 16-band graphic equalizer and spectrum analyzer implemented for ESP32 Radio EVO 3.19.

## New File Structure

```
src/
├─ EQ16_GraphicEQ.h / .cpp        # 16-band graphic equalizer
├─ EQ_AnalyzerCore.h / .cpp       # Core 1 spectrum analyzer
├─ EQ_AnalyzerDisplay.h / .cpp    # Display styles 5 & 6  
├─ EQ_AnalyzerHook.h / .cpp       # Audio processing bridge
└─ main.cpp                       # Minimal integration
```

## Architecture Components

### 1. EQ16_GraphicEQ (16-Band Graphic Equalizer)
- **Frequency Range**: 20 Hz - 20 kHz (16 bands)
- **Gain Range**: ±12 dB per band
- **Filter Type**: IIR peaking filters (2nd order)
- **Processing**: Real-time stereo processing
- **Interface**: Menu-driven band selection and gain adjustment

**Key Features:**
- ISO standard 1/3 octave frequency spacing
- Individual IIR filters per band (32 total for stereo)
- Save/load presets to SD card
- Compatible with existing 3-band tone controls

### 2. EQ_AnalyzerCore (Core 1 Spectrum Analyzer)
- **Algorithm**: Goertzel (lightweight alternative to FFT)
- **Processing Core**: Core 1 (dedicated task)
- **Bands**: 16 frequency bands matching EQ16
- **Performance**: Non-blocking, minimal impact on audio
- **Buffer**: 512-sample Goertzel windows with 50% overlap

**Key Features:**
- Real-time frequency analysis without blocking audio
- Peak detection with configurable decay
- Thread-safe communication via FreeRTOS queues
- Automatic gain scaling and logarithmic display conversion

### 3. EQ_AnalyzerDisplay (Styles 5 & 6)
- **Style 5**: Segmented bars with equal segments + half-segment peaks
- **Style 6**: Narrow continuous bars with peak indicators
- **Display**: 256x64 OLED with configurable update rates
- **Features**: Smoothing, peak hold, mini analyzer views

**Key Features:**
- Configurable smoothing factor (0.0 - 0.95)
- Peak hold with adjustable decay time
- Mini analyzer view for status displays
- Automatic data freshness detection

### 4. EQ_AnalyzerHook (Audio Processing Bridge)
- **Purpose**: Single integration point with Audio.cpp
- **Interface**: Clean C++ and C-style functions
- **Processing**: Sample routing to analyzer and equalizer
- **Performance**: Real-time monitoring and optimization

**Key Features:**
- Non-intrusive integration with existing audio pipeline
- Separate enable/disable for analyzer and equalizer
- Performance monitoring and load balancing
- Thread-safe sample processing

## Integration with main.cpp

### Minimal Changes Required
1. **Include headers**: Add 4 new includes
2. **Initialization**: Replace `eq_analyzer_init()` with new system
3. **Menu system**: Enhanced equalizer menu with 16-band support
4. **Button handling**: Extended for band navigation

### Button Controls
- **AUD**: Toggle equalizer menu (single press), switch 3/16-band mode (double press)
- **UP/DOWN**: Navigate bands (16-band) or tone controls (3-band)  
- **RIGHT/LEFT**: Increase/decrease gain values
- **Menu timeout**: Auto-return to normal display

## Performance Characteristics

### Processing Load
- **EQ16**: ~50-100 μs per audio buffer (depending on buffer size)
- **Analyzer**: Runs independently on Core 1, <1ms per analysis window
- **Display**: 30 FPS updates with minimal CPU impact
- **Memory**: ~8KB RAM for all components combined

### Real-time Performance
- **Audio latency**: No additional latency from EQ processing
- **Analysis delay**: ~23ms (512 samples at 44.1kHz)
- **Display update**: Smooth 30 FPS with peak hold
- **Queue depths**: Configurable for different performance needs

## Configuration Options

### EQ16_GraphicEQ Settings
```cpp
#define EQ16_BANDS 16           // Number of bands
#define EQ16_MIN_GAIN -12       // Minimum gain (dB)
#define EQ16_MAX_GAIN 12        // Maximum gain (dB)
```

### EQ_AnalyzerCore Settings
```cpp
#define GOERTZEL_WINDOW_SIZE 512    // Analysis window
#define GOERTZEL_OVERLAP 256        // Window overlap
#define EQ_ANALYZER_UPDATE_RATE 30  // Updates per second
```

### EQ_AnalyzerDisplay Settings
```cpp
#define STYLE5_SEGMENTS 8           // Segments per bar
#define STYLE6_BAR_WIDTH 14         // Narrow bar width
#define ANALYZER_MAX_HEIGHT 48      // Display height
```

## Usage Examples

### Basic Initialization
```cpp
// In setup()
EQ_AnalyzerHook::init();

EQ_AnalyzerDisplay analyzerDisplay;
analyzerDisplay.begin(&u8g2);
analyzerDisplay.setStyle(STYLE_5_SEGMENTS);

EQ_AnalyzerHook::enableAnalyzer(true);
EQ_AnalyzerHook::enableEqualizer(false);
```

### Equalizer Control
```cpp
// Set specific band gain
eq16.setBandGain(5, 3);  // Band 5, +3dB

// Navigate bands
eq16.selectNextBand();
eq16.selectPrevBand();

// Adjust current band
eq16.increaseBandGain();
eq16.decreaseBandGain();

// Save/load settings
eq16.saveToSD("/eq16_preset1.txt");
eq16.loadFromSD("/eq16_preset1.txt");
```

### Display Control
```cpp
// Manual display updates
analyzerDisplay.setActive(true);
analyzerDisplay.drawAnalyzer(true, false);  // Show labels, no scale

// Configuration
analyzerDisplay.setSmoothingFactor(0.7f);
analyzerDisplay.setPeakHoldTime(1.5f);
analyzerDisplay.setUpdateRate(25);  // 25 FPS
```

## Backward Compatibility

### Existing Features Preserved
- 3-band tone controls remain functional
- Existing FFT analyzer code can coexist
- SD card configuration files maintained
- Web interface compatibility preserved

### Migration Path
- Old APMS_GraphicEQ16 functionality replaced by EQ16_GraphicEQ
- Old analyzer functions redirected to new core
- Menu system enhanced, not replaced
- Configuration files can be migrated

## Troubleshooting

### Common Issues
1. **Compilation errors**: Ensure all new files are included in build
2. **Audio distortion**: Check EQ16 gain levels (avoid excessive boost)
3. **Display artifacts**: Verify U8G2 display initialization
4. **Performance issues**: Monitor Core 1 task utilization

### Debug Functions
```cpp
EQ_AnalyzerHook::printStatus();        // System status
EQ_AnalyzerCore::printStatus();        // Analyzer status  
analyzerDisplay.printConfiguration();   // Display config
eq16.printBandGains();                 // Current EQ settings
```

### Performance Monitoring
```cpp
uint32_t loadTime = EQ_AnalyzerHook::getProcessingLoad();
Serial.printf("Average processing: %d μs\n", loadTime);
```

## Future Enhancements

### Planned Features
1. **More display styles**: Additional visualization modes
2. **EQ presets**: Built-in and user-defined presets
3. **Auto-EQ**: Automatic room correction
4. **Spectrum recording**: Save frequency response data
5. **Web interface**: Remote EQ control

### Extension Points
- Additional filter types (high-pass, low-pass, notch)
- Variable Q factor per band
- Crossover frequency adjustment
- Multi-band compressor/limiter
- Real-time room measurement

---

*This architecture provides a solid foundation for advanced audio processing while maintaining compatibility with the existing ESP32 Radio EVO 3.19 codebase.*

Przepisujemy po koleji wszystkie funkcje do mometu obsługi wyswietlania na OLED wtedy robimy przerwę  i wpisujemy komentaż tu wstawiamy obsługę nowej bibioteki która w orginale jest na liniach nr lini od której zaczyna sie obsługa danej funcji i końca gdzie sie znajdue w orginalnym pilku  i tak zrobimy całę main potem zjmiemy sie obsługa poszczegulnych funcji obsługi OLED z nowa bibioteka dasz radę coś takiego zrobić z całym main .Zobacz w którym miejscu jest tak samo ieśli sa różnice to zamień main_oled ma być tak njak w bakcup  przepisz z bakup src main do main_oled  zacznij od linii 200