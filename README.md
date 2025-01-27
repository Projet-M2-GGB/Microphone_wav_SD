Audio Recognition with AI Model and WAV File Processing
Overview

This project enables recording audio, processing the recorded WAV file, and performing activity recognition using a pre-trained AI model on an STM32 microcontroller platform. The system uses a microphone to capture audio, saves the data to an SD card, and then processes it for recognition via a neural network model. The system is designed to recognize specific activities like "down," "go," "left," "right," "stop," and "up."
Features

    Audio Recording: Records audio in WAV format using the STM32 audio interface.
    AI Processing: Uses a pre-trained neural network to classify activities based on recorded audio.
    SD Card Integration: Supports SD card operations for storing recorded audio files and logs.
    Real-time Processing: Detects button presses to trigger audio recording and recognition, with LED feedback.
    FFT and Mel Spectrogram: Applies Fast Fourier Transform (FFT) and Mel spectrograms for feature extraction.

Hardware Requirements

    STM32 microcontroller (e.g., STM32F746G)
    Audio input device (e.g., microphone)
    SD Card (formatted and mounted)
    LED for visual feedback
    User button for triggering recording
    UART for debug messages

Software Requirements

    STM32 HAL library
    FatFS library for SD card operations
    AI model integration (AI network for classification)
    DSP functions (FFT and Mel spectrogram)

Setup Instructions

    Hardware Setup:
        Connect the microphone to the STM32 board for audio input.
        Insert an SD card into the STM32 board.
        Connect an LED and button to appropriate GPIO pins on the STM32.
    Software Setup:
        Clone the repository and open the project in STM32CubeIDE.
        Build and flash the program to the STM32 microcontroller.
        Ensure that the necessary peripheral drivers for SAI (Serial Audio Interface), SD card, and UART are properly configured in STM32CubeMX.

Key Functions
1. AI_Init()

Initializes the AI model by loading weights and biases, and setting up the input and output buffers.
2. AI_Process()

Takes in processed audio data, runs it through the AI model, and returns the predicted activity.
3. ReadWAVFileInfo()

Reads the WAV file header to gather metadata about the audio file (e.g., sample rate, number of channels).
4. normalize_audio()

Normalizes the audio data to prepare it for AI processing.
5. apply_fft()

Applies Fast Fourier Transform (FFT) to the audio data to extract frequency-domain features.
6. calculate_mel_spectrogram()

Computes the Mel spectrogram from the FFT output, which is used as input for the AI model.
7. SDCard_InitAndFormat()

Initializes and formats the SD card, preparing it for file storage.
8. new_log()

Creates a new log file on the SD card and writes content to it.
9. check_button_release()

Checks for the button press and resets the button_pressed flag when the button is released.
Workflow

    SD Card Initialization: The system initializes the SD card and formats it if necessary.
    Audio Recording: The system waits for the button press. Once the button is pressed, audio recording begins, and the file is saved in WAV format.
    Audio Processing: Once the recording stops, the WAV file is processed. The audio is normalized, transformed into a Mel spectrogram, and fed into the AI model.
    Activity Recognition: The AI model predicts the activity based on the processed audio data, and the result is printed to the UART terminal.
    LED Feedback: During recording, the LED is toggled to visually indicate that the system is active.

Example Output

SD card init...
AI model initialized successfully.
Waiting for input to record...
Button pressed...
Recording started...
Recording stopped.
Reading WAV file info...
Predicted activity: go (confidence: 0.92)

Troubleshooting

    SD Card Issues: Ensure the SD card is properly inserted and formatted. Use a reliable SD card.
    Audio Processing Errors: If preprocessing fails, verify the WAV file format (16kHz, mono, 16-bit).
    AI Model Errors: If the AI model initialization fails, check that the weights and activation buffers are correctly loaded.

License

This project is licensed under the terms of the STM32 software license. For details, please refer to the LICENSE file.
