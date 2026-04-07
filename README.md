## ESP32 Biphasic Waveform Generator

This project implements a real-time biphasic waveform generator using ESP32 (ESP-IDF), designed for H-bridge-based hardware systems.

# Features
  * Biphasic pulse generation with precise microsecond timing
  * Real-time control using FreeRTOS tasks
  * ADC-based current measurement with windowed sampling
  * PI control loop for dynamic current regulation
  * Modular firmware architecture (PWM, ADC, control, UI)

# System Overview
The system generates controlled biphasic pulses and regulates output current using feedback from ADC measurements. A PI controller dynamically adjusts PWM duty cycle to maintain target current levels.

# Technologies
 * ESP32 (ESP-IDF)
 * FreeRTOS
 * ADC (oneshot mode + calibration)
 * PWM (LEDC)
 * Embedded C
   
#Architecture
 * PWM module → waveform generation
 * ADC module → current sensing
 * Control module → PI loop
 * UI module → system interaction

# Notes
This project focuses on low-level embedded development, real-time control, and hardware–software integration.

Author
Talha Eren Yazıcı
