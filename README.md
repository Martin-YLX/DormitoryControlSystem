# STC Serial Control Panel

A desktop application based on **Python + Tkinter** for serial communication with a lower computer.  
It sends and receives fixed-length frames (6 bytes, starting with `AA 55`) and provides a simple control interface and logging system.

[🔗 macOS Version](https://github.com/Martin-YLX/DormitoryControlSystemForMac)

[📖 中文版说明 (README-CN.md)](README-CN.md)

---

## 📘 Project Information

- **Project Name**: Dormitory Control System
- **Completion Time**: 2025  
- **Course**: Electronic and Computer Systems Training (Summer 2025)  
- **Software Used**: PyCharm, Keil uVision  

---

## ✨ Features

- **Startup Splash**: Displays progress bar and group information at startup.  
- **Serial Management**:
  - Automatically lists available ports.
  - Select baud rate (9600 ~ 230400).
  - One-click connect/disconnect.  
- **Command Sending**:
  - Built-in command table (door, light, anti-forget system, eye-protection mode).
  - iOS-style toggle switches mapped to serial commands.
  - Manual HEX input supported.  
- **Logging System**:
  - Logs TX (sent) and RX (received) HEX data.
  - Automatically extracts and displays full 6-byte frames starting with `AA 55`.
  - Auto-scrolling for real-time monitoring.  
- **Custom UI Component**: An animated iOS-style toggle switch implemented with Tkinter Canvas.  

---

## 📝 Communication Protocol

- **Header**: `AA 55`  
- **Total Length**: 6 bytes  
- **Padding/Truncation**: Pad with `00` if shorter; truncate if longer.  

**Examples**:  
- Turn on light: `AA 55 02 01 00 00`  
- Turn off light: `AA 55 02 00 00 00`  

All outgoing data is processed through `build_frame_total6()` to ensure consistency.

---

## 🚀 Getting Started

### 1. Requirements
- Python 3.8+
- Dependencies:
  ```bash
  pip install pyserial
  ```

### 2. Run
```bash
python main.py
```

The splash screen will show first, followed by the main control panel.

---

## 🔑 Core Code Overview

1. **Protocol Handling**
   - `parse_hex_string()`: Parse user input HEX strings.
   - `build_frame_total6()`: Enforce 6-byte fixed length.  
2. **Serial Communication**
   - `SerialClient`: Encapsulates pyserial, runs a background receive thread.
   - `send_bytes()`: Unified sending method.  
3. **Receiving & Framing**
   - `rx_q`: Thread-safe queue (worker → UI).
   - `_poll_rx()`: Polls the queue in the main thread and logs data.
   - `_extract_and_handle_frames()`: Finds header, handles sticky packets, extracts frames.  
4. **Command Entry**
   - `CMD_HEX`: Semantic command table (e.g., `"LIGHT 1" → "AA 55 02 01"`).
   - `_send_cmd_hex(key)`: Unified interface for sending commands.
