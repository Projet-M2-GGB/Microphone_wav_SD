# Audio Recognition with AI Model and WAV File Processing

## Overview
This project enables recording audio, processing the recorded WAV file, and performing activity recognition using a pre-trained AI model on an STM32 microcontroller platform. The system captures audio using a microphone, saves the data to an SD card, and processes it for recognition via a neural network model. The system is designed to recognize specific activities like "down," "go," "left," "right," "stop," and "up."

## Features
- **Audio Recording**: Records audio in WAV format using the STM32 audio interface.
- **AI Processing**: Uses a pre-trained neural network to classify activities based on recorded audio.
- **SD Card Integration**: Supports SD card operations for storing recorded audio files and logs.
- **Real-time Processing**: Detects button presses to trigger audio recording and recognition, with LED feedback.
- **FFT and Spectrogram**: Applies Fast Fourier Transform (FFT) and spectrograms for feature extraction.
- **UART Debugging**: Sends system status and results over UART for monitoring.

## Hardware Requirements
- STM32 microcontroller (e.g., STM32F746G Discovery board)
- SD Card 
- User button for triggering recording
- UART for debug messages
- nRF24L01 radio module

## Software Requirements
- STM32 HAL library
- FatFS library for SD card operations
- AI model integration (AI network for classification)
- DSP CMSIS functions (FFT and Mel spectrogram computation)
- BSP waverecorder and SDRAM
- STM32CubeIDE for development

## Key Functions
### AI Model Functions
- **AI_Init()**: Initializes the AI model, loading weights and biases and setting up input/output buffers.
- **AI_Process()**: Runs inference on processed audio data and returns the predicted activity.

### Audio Processing Functions
- **ReadWAVFileInfo()**: Reads the WAV file header to extract metadata (e.g., sample rate, channels).
- **normalize_audio()**: Normalizes the audio data to prepare it for AI processing.
- **apply_fft()**: Computes the FFT to extract frequency-domain features.
- **calculate_mel_spectrogram()**: Converts FFT output into a Mel spectrogram for AI model input.

### SD Card and Logging Functions
- **SDCard_InitAndFormat()**: Initializes and formats the SD card for file storage.
- **new_log()**: Creates a log file on the SD card and writes relevant content.

### User Interaction Functions
- **check_button_release()**: Monitors the button press and resets the `button_pressed` flag when released.

## Workflow
1. **SD Card Initialization**: The system initializes and formats the SD card if necessary.
2. **Audio Recording**: Waits for a button press; on press, recording starts and saves audio as a WAV file.
3. **Audio Processing**: Processes the recorded file, normalizing audio and computing a Mel spectrogram.
4. **Activity Recognition**: The AI model predicts the activity and outputs the result via UART.
5. **Message transmission**: The command is sent via the nRF24L01 radio module.

## Troubleshooting
- **SD Card Issues**: Ensure the SD card is properly inserted and formatted (FAT32 recommended).
- **Audio Processing Errors**: If preprocessing fails, verify that the WAV file format is correct (16kHz, stereo, 16-bit).
- **AI Model Errors**: If AI model initialization fails, check that the weights and activation buffers are correctly loaded.
- **Low Accuracy**: Ensure the audio is clear and properly processed before being fed into the AI model.

## License
This project is open-source and available under the MIT License.

