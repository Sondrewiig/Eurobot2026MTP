bool startSentToPi = false;

void onEspNowReceive(const uint8_t *mac, const uint8_t *data, int len) {
  char msg[32];

  int copyLen = min(len, 31);
  memcpy(msg, data, copyLen);
  msg[copyLen] = '\0';

  if (strcmp(msg, "START") == 0 && !startSentToPi) {
    startSentToPi = true;

    // Send start signal to Raspberry Pi over USB serial
    Serial.println("START");
  }
}

void setupEspNowReceiver() {
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onEspNowReceive);

  Serial.print("ESP-NOW receiver ready. MAC: ");
  Serial.println(WiFi.macAddress());
}