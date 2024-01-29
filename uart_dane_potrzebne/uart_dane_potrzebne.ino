#include <WiFi.h>
#include <HTTPClient.h>
#include <HardwareSerial.h>

const char *ssid = "esp1";
const char *password = "12345678q";
const char *serverIP = "192.168.4.2"; // Adres IP pierwszego modułu ESP32
const int port = 80;

const int uart_rx_pin = 16; // Domyślny pin RX dla UART
const int uart_tx_pin = 17; // Domyślny pin TX dla UART

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, uart_rx_pin, uart_tx_pin);

  // Utwórz punkt dostępu
  WiFi.softAP(ssid, password);
  Serial.println("Access Point created");

  // Adres IP punktu dostępu
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(apIP);

  // Rozpocznij serię zapytań GET do pierwszego modułu ESP32
  while (true)
  {
    sendGETRequest();

    delay(5000); // Odczekaj 5 sekund przed wysłaniem kolejnego zapytania
  }
}

void sendGETRequest()
{
  HTTPClient http;

  String url = "http://" + String(serverIP) + "/";
  Serial.print("Sending GET request to: ");
  Serial.println(url);

  // Wyślij zapytanie GET
  http.begin(url);

  int httpCode = http.GET();
  if (httpCode > 0)
  {
    String payload = http.getString();
    Serial.println("Received response: " + payload);

    // Wyślij dane przez UART do Arduino Uno
    //Serial2.println(payload);

    // Ekstrakcja i wysłanie wartości z czujników
    
    float pm25Value = extractSensorValue(payload, "<div class=\"card pm25\">", "<span class=\"reading\">", "</span>");
    float pm10Value = extractSensorValue(payload, "<div class=\"card pm10\">", "<span class=\"reading\">", "</span>");
    float pm1Value = extractSensorValue(payload, "<div class=\"card pm1\">", "<span class=\"reading\">", "</span>");
    float temperatureValue = extractSensorValue(payload, "<div class=\"card temperature\">", "<span class=\"reading\">", "</span>");
    float lightValue = extractSensorValue(payload, "<div class=\"card light\">", "<span class=\"reading\">", "</span>");
    float pressureValue = extractSensorValue(payload, "<div class=\"card pressure\">", "<span class=\"reading\">", "</span>");
    float humidityValue = extractSensorValue(payload, "<div class=\"card humidity\">", "<span class=\"reading\">", "</span>");
    float eTVOCValue = extractSensorValue(payload, "<div class=\"card eTVOC\">", "<span class=\"reading\">", "</span>");
    float eCO2Value = extractSensorValue(payload, "<div class=\"card eCO2\">", "<span class=\"reading\">", "</span>");

    Serial2.println("PM25: " + String(pm25Value) + " ug/m3");
    Serial2.println("PM10: " + String(pm10Value) + " ug/m3");
    Serial2.println("PM1: " + String(pm1Value) + " ug/m3");
    Serial2.println("Temperature: " + String(temperatureValue) + " C");
    Serial2.println("Light: " + String(lightValue) + " mV");
    Serial2.println("Pressure: " + String(pressureValue) + " Pa");
    Serial2.println("Humidity: " + String(humidityValue) + " RH");
    Serial2.println("eTVOC: " + String(eTVOCValue) + " ppb");
    Serial2.println("eCO2: " + String(eCO2Value) + " ppm");
  }
  else
  {
    Serial.println("Failed to connect or receive response");
  }

  http.end();
}

void loop()
{
  // Brak dodatkowych operacji w pętli głównej
}

float extractSensorValue(String data, String cardStart, String valueStart, String valueEnd)
{
  int cardPos = data.indexOf(cardStart);
  if (cardPos == -1)
  {
    Serial.println("Błąd: Brak sensora w danych HTML.");
    return 0.0;
  }

  int start = data.indexOf(valueStart, cardPos) + valueStart.length();
  int end = data.indexOf(valueEnd, start);

  String readingValue = data.substring(start, end);
  return readingValue.toFloat();
}
