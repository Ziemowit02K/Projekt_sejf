#include <Arduino.h>
#include <esp32-hal-ledc.h>
#include <HardwareSerial.h>

// === Piny ===
#define IR_SENSOR_PIN 4
#define BUZZER_PIN 18 //5
#define BUTTON_PIN 5 //18
#define MODEM_RX 25
#define MODEM_TX 26
#define LED_CZERWONA 12
#define LED_ZIELONA  14
#define LED_NIEBIESKA 13


// === Obiekty i zmienne ===
HardwareSerial sim800(1);
bool alarmEnabled = false;
bool lastButtonReading = HIGH;
bool currentButtonState = HIGH;
bool alarmON = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastSmsTime = 0;
const unsigned long smsInterval = 300000; // 5 minut
int syrena = 1;

// === Funkcja AT ===
void sendAT(String cmd, int wait_ms = 1000) {
  Serial.print(">> ");
  Serial.println(cmd);
  sim800.println(cmd);
  delay(wait_ms);
  while (sim800.available()) {
    Serial.write(sim800.read());
  }
  Serial.println();
}

// === Wysyłanie SMS ===
void sendSMS() {
  Serial.println("📤 Wysyłanie SMS...");
  

  sim800.println("AT+CMGS=\"602470359\"");
  delay(1000);

  String response = "";
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // maks. 5 sek. oczekiwania na '>'
    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      if (c == '>') {
        sim800.print("Sejf otwarty!");
        sim800.write(26);  // CTRL+Z
        Serial.println("✅ SMS wysłany.");
        return;
      }
    }
  }

  Serial.println("❌ Błąd: brak odpowiedzi > do wysłania treści.");
}

void diodes() {
    if (alarmEnabled)
    {
      if (alarmON) 
      {
        if(syrena % 2 ==0 )
        {
          digitalWrite(LED_CZERWONA, HIGH);
          digitalWrite(LED_NIEBIESKA, LOW);
        }
        else
        {
          digitalWrite(LED_CZERWONA, LOW);
          digitalWrite(LED_NIEBIESKA, HIGH);
        }
      }
      else
      {
        digitalWrite(LED_CZERWONA, HIGH);
        digitalWrite(LED_NIEBIESKA, LOW);
        digitalWrite(LED_ZIELONA, LOW);
      }
    }
    else
    {
      digitalWrite(LED_ZIELONA, HIGH);
      digitalWrite(LED_CZERWONA, LOW);
      digitalWrite(LED_NIEBIESKA, LOW);
    }
  }

void checkIncomingSMS() {
  sim800.println("AT+CMGL=\"REC UNREAD\"");
  delay(1000);

  String fullResponse = "";
  while (sim800.available()) {
    fullResponse += (char)sim800.read();
  }

  if (fullResponse.length() > 0) {
    Serial.println("📥 Odebrano SMS:");
    Serial.println(fullResponse);

    // Szukamy treści SMS jako drugiej linii po nagłówku +CMGL:
    int cmglIndex = fullResponse.indexOf("+CMGL:");
    if (cmglIndex != -1) {
      int msgStart = fullResponse.indexOf('\n', cmglIndex); // koniec nagłówka
      if (msgStart != -1) {
        int msgEnd = fullResponse.indexOf('\n', msgStart + 1); // koniec wiadomości
        String body = fullResponse.substring(msgStart + 1, msgEnd);
        body.trim();

        Serial.print("📩 Treść SMS: [");
        Serial.print(body);
        Serial.println("]");

       if (body.indexOf("a") != -1) {
        alarmEnabled = true;
        Serial.println("✅ Alarm WŁĄCZONY przez SMS");
      } else if (body.indexOf("d") != -1) {
        alarmEnabled = false;
        Serial.println("⛔ Alarm WYŁĄCZONY przez SMS");
      } else {
        Serial.println("⚠️ Nieznana komenda SMS");
      }


        // Usuń wszystkie wiadomości
        sim800.println("AT+CMGD=1,4");
        delay(500);
      }
    }
  }
}




void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_CZERWONA, OUTPUT);
  pinMode(LED_ZIELONA, OUTPUT);
  pinMode(LED_NIEBIESKA, OUTPUT);

  ledcAttach(BUZZER_PIN, 3000, 8); // 3 kHz, 8-bit
  lastSmsTime = millis() - smsInterval;

  Serial.println("🔧 Inicjalizacja SIM800L...");
  sim800.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

sendAT("AT+CFUN=1,1");  // reset modułu
delay(5000);            // czekaj aż wróci


  // Inicjalizacja SIM
  sendAT("AT");
  sendAT("AT+CPIN=\"4278\"");
  
  while (true) {
    sim800.println("AT+CPIN?");
    delay(1000);
    String response = "";
    while (sim800.available()) response += (char)sim800.read();
    Serial.print(response);
    if (response.indexOf("READY") >= 0) break;
    Serial.println("⏳ Czekam na kartę SIM...");
  }

  

  while (true) {
    sim800.println("AT+CREG?");
    delay(1000);
    String response = "";
    while (sim800.available()) response += (char)sim800.read();
    Serial.print(response);
    if (response.indexOf("+CREG: 0,1") >= 0) break;
    Serial.println("📡 Czekam na sieć...");
  }

  sendAT("AT+CMGF=1");
  sendAT("AT+CSCS=\"GSM\"");
  Serial.println("✅ SIM800L gotowy.");
}

void loop() {
  // — Przycisk z debounce —
  diodes();
  checkIncomingSMS();
  bool reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        alarmEnabled = !alarmEnabled;
        Serial.print("🔔 Alarm ");
        Serial.println(alarmEnabled ? "WŁĄCZONY" : "WYŁĄCZONY");
      }
    }
  }
  lastButtonReading = reading;

  // — Czujnik IR —
  int irState = digitalRead(IR_SENSOR_PIN);
  Serial.print("IR = "); Serial.print(irState);
  Serial.print(" | Alarm = "); Serial.print(alarmEnabled);
  Serial.print(" | Czas od SMS = "); Serial.println(millis() - lastSmsTime);

  if (irState == HIGH && alarmEnabled) {
    ledcWrite(BUZZER_PIN, 255);
    Serial.println("❌ Brak obiektu – BUZZER ON");
    digitalWrite(LED_ZIELONA, LOW);
    alarmON = true;
    

    if (millis() - lastSmsTime >= smsInterval) {
      Serial.println("📨 Warunki spełnione – wysyłam SMS");
      sendSMS();
      lastSmsTime = millis();
    } else {
      Serial.println("⏳ Jeszcze nie czas na kolejny SMS");
    }

  } else {
    ledcWrite(BUZZER_PIN, 0);
    Serial.println("✅ Obiekt wykryty lub alarm wyłączony");
    alarmON = false;

  }
  syrena++;
  delay(500); // odświeżanie co 0.5 sekundy
}
