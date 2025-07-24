# Enhanced LD2412 Sensor Demo

This is an advanced example program for the LD2412 presence detection sensor that demonstrates comprehensive error handling, connectivity monitoring, and debugging capabilities.

## Features

### 🔍 Enhanced Connectivity Monitoring
- Automatic sensor connectivity checks every 5 seconds
- Firmware version requests to validate communication
- Watchdog timer to detect communication timeouts
- Comprehensive error tracking and recovery mechanisms

### 📊 Advanced Data Processing
- Real-time sensor data validation and analysis
- Detailed logging with timestamps and packet counters
- Detection of unusual readings and out-of-range values
- Engineering mode support for detailed diagnostics

### 🚨 Error Handling & Recovery
- Multi-level error detection and reporting
- Automatic recovery attempts for communication failures
- Consecutive error counting with escalating responses
- System health monitoring and status reporting

### 🔘 Interactive Button Controls
- **Single Click**: Toggle Engineering Mode on/off
- **Double Click**: Request firmware version information
- **Long Press**: Print comprehensive system status

### 📈 System Monitoring
- Real-time system status dashboard
- Periodic health reports every 30 seconds
- Memory usage tracking
- Communication statistics and error rates

## Hardware Requirements

- ESP32-C6 (or compatible ESP32 variant)
- LD2412 presence detection sensor
- Connection via UART (pins configurable in uart_config.h)

### Default Pin Configuration
```
UART TX: GPIO 5
UART RX: GPIO 4
Boot Button: GPIO 9 (ESP32-C6) / GPIO 0 (other variants)
```

## Software Architecture

### Core Components

1. **Enhanced Main Application** (`ld2412-example.c`)
   - System initialization and task management
   - Timer-based monitoring
   - Button event handling
   - Comprehensive status reporting

2. **Enhanced LD2412 Driver** (`enhanced_ld2412.c/h`)
   - Improved parsing with validation
   - Connectivity testing functions
   - Firmware information tracking
   - Enhanced error detection

3. **Enhanced UART Handler** (`enhanced_uart_config.c/h`)
   - Robust frame processing
   - Statistical logging
   - Error counting and reporting
   - Queue management with timeouts

4. **Button Interface** (`button_config.c/h`)
   - Multi-function button support
   - Event-driven callbacks
   - Integration with main application

### Task Architecture

```
┌─────────────────────┬──────────┬──────────────────────────┐
│ Task Name           │ Priority │ Function                 │
├─────────────────────┼──────────┼──────────────────────────┤
│ uart_event_task     │    5     │ UART event handling      │
│ uart_tx_task        │    4     │ UART transmission        │
│ enhanced_uart_rx    │    4     │ Enhanced UART reception  │
│ sensor_data_task    │    3     │ Data processing & validation │
│ connectivity_task   │    2     │ Connectivity monitoring  │
│ system_monitor      │    1     │ System health reporting  │
└─────────────────────┴──────────┴──────────────────────────┘
```

### Timer Architecture

- **Connectivity Timer**: 5-second intervals for connectivity checks
- **Firmware Timer**: 10-second intervals for firmware version requests
- **Watchdog Timer**: 3-second timeout for data reception monitoring

## Building and Running

### Prerequisites
- ESP-IDF v5.0 or later
- LD2412 component properly installed in managed_components

### Build Commands
```bash
cd ld2412-example
idf.py build
idf.py flash monitor
```

### Expected Output

During normal operation, you should see output similar to:

```
I (1234) LD2412_ADVANCED_DEMO: ╔══════════════════════════════════════════════════╗
I (1234) LD2412_ADVANCED_DEMO: ║        LD2412 ADVANCED SENSOR DEMO v2.0          ║
I (1234) LD2412_ADVANCED_DEMO: ║          Enhanced with Error Handling            ║
I (1234) LD2412_ADVANCED_DEMO: ╚══════════════════════════════════════════════════╝

I (1500) LD2412_ADVANCED_DEMO: 🔧 Initializing UART communication...
I (1600) LD2412_ADVANCED_DEMO: 🔧 Initializing button interface...
I (1700) LD2412_ADVANCED_DEMO: ✅ All timers started successfully
I (1800) LD2412_ADVANCED_DEMO: ✅ All tasks created successfully

I (2000) ENHANCED_LD2412: 📋 LD2412 Firmware Type: 24.12
I (2001) ENHANCED_LD2412: 📋 LD2412 Firmware Version: V01.02.03.04.05.06
I (2001) ENHANCED_LD2412: ✅ Firmware info reception confirmed

I (2500) LD2412_ADVANCED_DEMO: 📊 DATA #1 | Target State: 0 | Moving: 0 cm | Stationary: 0 cm | Time: 2500000 us
I (2600) LD2412_ADVANCED_DEMO: 📊 DATA #2 | Target State: 1 | Moving: 150 cm | Stationary: 0 cm | Time: 2600000 us
```

## Debugging Features

### Log Levels
The application uses multiple log levels for different types of information:
- `ESP_LOGI`: General information and normal operation
- `ESP_LOGW`: Warnings for unusual but non-critical conditions
- `ESP_LOGE`: Errors requiring attention
- `ESP_LOGD`: Detailed debugging information (compile-time configurable)

### Error Codes and Meanings

| Error Type | Description | Recovery Action |
|------------|-------------|-----------------|
| Connectivity Timeout | No response to firmware requests | Automatic retry with exponential backoff |
| Invalid Frame Format | Malformed UART data received | Frame discarded, statistics updated |
| Queue Timeout | System queue operations fail | Queue reset and reinitialization |
| Consecutive Errors | Multiple errors in sequence | Full communication reset |

### Status Dashboard
Press and hold the button for 2 seconds to display the comprehensive status dashboard:

```
╔══════════════════════════════════════════════════╗
║                SYSTEM STATUS REPORT              ║
╠══════════════════════════════════════════════════╣
║ Sensor Connected:      YES                       ║
║ Engineering Mode:      INACTIVE                  ║
║ Firmware Info Rcvd:    YES                       ║
║ Total Data Packets:    1234                      ║
║ Total Errors:          5                         ║
║ Consecutive Errors:    0                         ║
║ Last Data Time:        12345678 us               ║
║ Free Heap:             128 KB                    ║
║ Uptime:                12345678 us               ║
╚══════════════════════════════════════════════════╝
```

## Troubleshooting

### Common Issues

1. **No sensor data received**
   - Check UART wiring (TX/RX pins)
   - Verify sensor power supply (3.3V/5V as required)
   - Check baud rate configuration (default: 57600)

2. **Firmware version not detected**
   - Sensor may not be in configuration mode
   - Check for proper sensor initialization delay
   - Verify UART communication parameters

3. **High error rates**
   - Check for electromagnetic interference
   - Verify cable quality and length
   - Consider reducing UART baud rate

### Debug Commands via Button

- **Single Click**: Toggle engineering mode to see detailed sensor data
- **Double Click**: Force firmware version request to test connectivity
- **Long Press**: Display full system status for diagnostics

## Advanced Configuration

### Timing Parameters
Modify these constants in `ld2412-example.c` to adjust behavior:

```c
#define CONNECTIVITY_CHECK_INTERVAL_MS     5000    // Connectivity check frequency
#define FIRMWARE_READ_INTERVAL_MS          10000   // Firmware request frequency  
#define SENSOR_DATA_TIMEOUT_MS             3000    // Data timeout threshold
#define MAX_CONSECUTIVE_ERRORS             5       // Error threshold for recovery
#define CONNECTIVITY_TIMEOUT_MS            2000    // Response timeout
```

### UART Configuration
Modify `uart_config.h` for different pin assignments:

```c
#define UART_TXD_PIN                    5          // TX pin
#define UART_RXD_PIN                    4          // RX pin
#define TX_BUF_SIZE                     1024       // TX buffer size
#define RX_BUF_SIZE                     1024       // RX buffer size
```

## Performance Metrics

The enhanced example provides detailed performance monitoring:

- **Frame Processing Rate**: Typically 10-20 frames per second
- **Error Rate**: Should be < 1% under normal conditions
- **Memory Usage**: ~32KB heap for tasks and buffers
- **CPU Usage**: < 5% on ESP32-C6 at 160MHz

## Future Enhancements

Potential improvements for this example:
- Web interface for remote monitoring
- Data logging to flash storage
- OTA firmware update capability
- Multi-sensor support
- Custom alert thresholds
- Data export functionality

## License

This enhanced example is provided under the same license as the original LD2412 component. Refer to the component documentation for specific license terms.
