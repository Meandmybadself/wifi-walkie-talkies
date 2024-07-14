#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <opus.h>

// WiFi credentials
const char* ssid = "Estate";
const char* password = "wireless";

// WebSocket server details
const char* wsHost = "192.168.1.151";
const int wsPort = 8080;

// Audio configuration
#define SAMPLE_RATE 16000
#define SAMPLE_BITS 16
#define CHANNELS 1
#define I2S_PORT I2S_NUM_0

// I2S configuration for Adafruit Feather ESP32 V2 with I2S Microphone
#define I2S_BCLK_PIN 14    // Bit clock
#define I2S_LRCLK_PIN 15   // Left/Right clock (Word Select)
#define I2S_DATA_IN_PIN 32  // Data in (from microphone)

// Button configuration
const int BUTTON_PIN = 38;  // Using GPIO38 for the on-board button
volatile bool buttonPressed = false;

WebSocketsClient webSocket;
bool isTransmitting = false;
OpusEncoder *encoder;

void IRAM_ATTR buttonISR() {
  buttonPressed = true;
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give some time for Serial to initialize
  Serial.println("Starting setup...");
  
  Serial.println("Setting up WiFi...");
  setupWiFi();
  Serial.println("WiFi setup complete.");
  
  delay(1000);
  Serial.println("Setting up WebSocket...");
  setupWebSocket();
  Serial.println("WebSocket setup complete.");
  
  delay(1000);
  Serial.println("Setting up I2S...");
  setupI2S();
  Serial.println("I2S setup complete.");
  
  delay(1000);
  Serial.println("Setting up Opus...");
  setupOpus();
  Serial.println("Opus setup complete.");
  
  delay(1000);
  Serial.println("Setting up Button...");
  setupButton();
  Serial.println("Button setup complete.");

  Serial.println("All setup complete");
}

void loop() {
  webSocket.loop();
  
  if (buttonPressed) {
    handleButtonPress();
  }
  
  if (isTransmitting) {
    captureAndSendAudio();
  }
  
  delay(10); // Small delay to prevent flooding the serial output
}

void setupWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }
}

void setupWebSocket() {
  webSocket.begin(wsHost, wsPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  Serial.println("WebSocket setup complete");
}

void setupI2S() {
  Serial.println("Starting I2S setup...");

  Serial.println("Configuring I2S...");
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  Serial.println("I2S config created");

  Serial.println("Configuring I2S pins...");
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK_PIN,
    .ws_io_num = I2S_LRCLK_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_IN_PIN
  };
  Serial.println("I2S pin config created");

  Serial.printf("BCLK: %d, LRCLK: %d, DATA_IN: %d\n", I2S_BCLK_PIN, I2S_LRCLK_PIN, I2S_DATA_IN_PIN);

  Serial.println("Installing I2S driver...");
  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed to install I2S driver: %d\n", err);
    return;
  }
  Serial.println("I2S driver installed successfully");

  Serial.println("Setting I2S pins...");
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S pins: %d\n", err);
    return;
  }
  Serial.println("I2S pins set successfully");

  Serial.println("Setting I2S clock...");
  err = i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S clock: %d\n", err);
    return;
  }
  Serial.println("I2S clock set successfully");

  Serial.println("I2S setup complete");
}

void setupOpus() {
  int error;
  Serial.println("setupOpus");
  encoder = opus_encoder_create(SAMPLE_RATE, CHANNELS, OPUS_APPLICATION_AUDIO, &error);
  Serial.println("setupOpus complete");
  if (error != OPUS_OK) {
    Serial.printf("Failed to create Opus encoder: %d\n", error);
  } else {
    Serial.println("Opus encoder created successfully");
    
    opus_encoder_ctl(encoder, OPUS_SET_BITRATE(8000));  // Reduce bitrate
    opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(1));  // Reduce complexity
    
    opus_int32 bitrate;
    opus_encoder_ctl(encoder, OPUS_GET_BITRATE(&bitrate));
    Serial.printf("Initial Opus bitrate: %d\n", bitrate);

    opus_int32 complexity;
    opus_encoder_ctl(encoder, OPUS_GET_COMPLEXITY(&complexity));
    Serial.printf("Initial Opus complexity: %d\n", complexity);
  }
}

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  Serial.println("Button setup complete on pin " + String(BUTTON_PIN));
}

void handleButtonPress() {
  static unsigned long lastDebounceTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > 200) {  // 200ms debounce time
    Serial.println("Button pressed");
    isTransmitting = !isTransmitting;
    Serial.println("Transmitting state: " + String(isTransmitting));
    lastDebounceTime = currentTime;
  }

  buttonPressed = false;  // Reset the flag
}

void captureAndSendAudio() {
  const int samples = 480;  // 30ms at 16kHz
  int32_t buffer_32[samples];
  int16_t audio_buffer[samples];
  size_t bytes_read;
  
  esp_err_t err = i2s_read(I2S_PORT, buffer_32, samples * sizeof(int32_t), &bytes_read, portMAX_DELAY);
  if (err != ESP_OK) {
    Serial.printf("Failed to read I2S data: %d\n", err);
    return;
  }
  
  Serial.printf("Bytes read from I2S: %d\n", bytes_read);

  // Print raw 32-bit samples
  Serial.println("First 5 raw 32-bit samples:");
  for (int i = 0; i < 5; i++) {
    Serial.printf("0x%08X ", buffer_32[i]);
  }
  Serial.println();

  // Convert 32-bit samples to 16-bit and remove DC offset
  int32_t sum = 0;
  for (int i = 0; i < samples; i++) {
    // The SPH0645LM4H left-aligns 18-bit samples in 32-bit words
    // Shift right by 14 to get 18-bit value, then by 2 more to get 16-bit value
    audio_buffer[i] = buffer_32[i] >> 16;
    sum += audio_buffer[i];
  }
  int32_t avg = sum / samples;

  // Remove DC offset and print first few samples
  Serial.print("First 5 audio samples (after DC offset removal): ");
  for (int i = 0; i < 5; i++) {
    audio_buffer[i] -= avg;
    Serial.printf("%d ", audio_buffer[i]);
  }
  Serial.println();

  // Check if all samples are zero or very close to zero
  bool all_zero = true;
  for (int i = 0; i < samples; i++) {
    if (abs(audio_buffer[i]) > 10) {  // Allow for some small non-zero values due to noise
      all_zero = false;
      break;
    }
  }

  if (all_zero) {
    Serial.println("Warning: All audio samples are zero or very close to zero!");
    return;
  }

  unsigned char encoded[1275];  // Max size for one frame
  opus_int32 encoded_bytes = opus_encode(encoder, audio_buffer, samples, encoded, sizeof(encoded));
  
  if (encoded_bytes > 0) {
    Serial.printf("Opus encoded bytes: %d\n", encoded_bytes);
    
    Serial.print("First 10 bytes of encoded data: ");
    for (int i = 0; i < 10 && i < encoded_bytes; i++) {
      Serial.printf("%02X ", encoded[i]);
    }
    Serial.println();
    
    bool sent = webSocket.sendBIN(encoded, encoded_bytes);
    Serial.printf("WebSocket send result: %s\n", sent ? "Success" : "Failure");
  } else {
    Serial.printf("Opus encoding failed with error code: %d\n", encoded_bytes);
    
    opus_int32 bitrate;
    opus_encoder_ctl(encoder, OPUS_GET_BITRATE(&bitrate));
    Serial.printf("Opus bitrate: %d\n", bitrate);

    opus_int32 complexity;
    opus_encoder_ctl(encoder, OPUS_GET_COMPLEXITY(&complexity));
    Serial.printf("Opus complexity: %d\n", complexity);
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("Disconnected from WebSocket server");
      break;
    case WStype_CONNECTED:
      Serial.println("Connected to WebSocket server");
      break;
    case WStype_TEXT:
      Serial.println("Received unexpected text message");
      break;
    case WStype_BIN:
      Serial.println("Received binary message");
      break;
    default:
      Serial.println("Received unknown message type");
      break;
  }
}