# RTOS-Based-Multi-Sensor-Environmental-Analyzer

# Project Description

This project monitors ambient conditions using acquired temperature, humidity, pressure and gas sensor data. When the acquired data exceeds predefined thresholds and the ambient is assessed as unhealthy, the system triggers a loud buzzer to alert the user. Additionally, it continuously provides real-time information about the ambient conditions via the dashboard.

# Software Architecture

The system operates based on event-driven task structure using CMSIS-RTOS v2. Below is a breakdown of the  architecture:

* __STM32 Side__

A sound sensor was connected to an external interrupt pin. When a sound is detected, an interrupt is triggered and a binary semaphore is released. A LED is turned on to indicate that the interrupt has occured.

The sensor task waits until the binary semaphore is acquired. Once acquired, it initiates the measurement process.

__The BME280 sensor__ is used to measure temperature, humidity and pressure. A driver sourced from GitHub was adapted for RTOS compatibility (e.g., with mutex protection during I2C access). No additional filtering was applied because the sensor already includes internal filtering from the factory.

__The MQ135 sensor__ is used to measure gas quality. A Moving Average Filter was implemented to obtain more reliable and stable data. The sensor's raw output is inversely proportional to air quality: a higher analog value indicates poor air quality. To present results as a percentage (where 100% represents clean air), the filtered value is subtracted from 100.

Sensor readings are taken every 3 seconds using `vTaskDelayUntil`. Readings sensor data is sent to a message queue. 

An UART task reads the data from the queue. Values are converted to a JSON format using `snprintf`. UART communication is protected using a mutex and the data is sent via UART.

* __ESP32 Side__

The ESP32 receives the JSON-formatted data from the STM32 via UART, parses it, and publishes it to the Adafruit IO Dashboard using the MQTT protocol. On the ESP32 side, the received data is also used for ambient evaluation. If parameters fall outside the predefined thresholds (e.g., temperature < 15°C and humidity < 50%), a warning is triggered. A buzzer is activated when the ambient conditions are poor. These evaluation messages are also published to the dashboard for user feedback.

Figure 1 : System Overview

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/system_overview.jpeg" alt="System Overview" width="500"/>

Figure 2 : Adafruit IO Dashboard

<img src="https://github.com/ssenanb/RTOS-Based-Multi-Sensor-Environmental-Analyzer/blob/main/dashboard.png" alt="System Overview" width="500"/>

# Hardware Components

* __STM32F0DISC__ – Main microcontroller unit for sensor acquisition and processing

* __ESP32 WROOM 32D__ – Wi-Fi module for MQTT communication with Adafruit IO  

* __USB to TTL (Serial) Communication Module__ – For debugging via UART  

* __BME280 Sensor__ – Measures temperature, humidity, and pressure

* __MQ135 Sensor__ – Detects air quality (gas concentration)  

* __Sound Sensor__ – Used for triggering  
  
* __LED__ – Visual indicator for interrupt status  

* __Resistor (330 ohm)__ – Current limiting resistor for the LED  

* __Active Buzzer__ – Audible alert when environment is unhealthy  

* __Jumper Cables__ – Used to connect components  

* __Breadboard__ – For prototyping and sensor wiring

* __USB Cable / Power Supply__ – Powering the STM32 and ESP32 modules

# Software Components

* __STM32CubeIDE__ – Embedded firmware development for STM32

* __Arduino IDE__ – Programming  the ESP32 module

* __C/C++ Programming Languages__ – Core programming languages for embedded development

* __Adafruit IO Dashboard__ – Visualization of environmental sensor data  

* __Termitte Terminal (for debug)__ – Serial debugging and log monitoring via UART 

* __FreeRTOS (CMSIS-RTOS v2)__ – Real-time task management on STM32

* __Digital Filtering (Moving Average)__ – Smoothing sensor readings 

* __MQTT Protocol__ – Data transmission from ESP32 to Adafruit IO  

* __GPIO / PWM / ADC / UART Communication Peripherals__ – Use of STM32 hardware features




