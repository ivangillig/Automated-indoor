#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

#define moistureSensor A0
#define wet 413
#define dry 819

int waterPump = 7;
int ambientSensor = 2;
float temperature;
int humidity;
DHT dht (ambientSensor, DHT22);

#define MOISTURE_INTERVAL 1000  // Intervalo para las lecturas del sensor de humedad del suelo (en milisegundos)
#define AFTER_WATERING_INTERVAL 5000 // Intervalo después de que dejo de regar
unsigned long previousMoistureMillis = 0;
int soilHumidity;
int previousSoilHumidity;

const long wateringInterval = 5000;
unsigned long previousMillisLCD = 0; 
unsigned long previousMillisPump = 0;
unsigned long previousMillisAfterWatering = 0;
const long intervalLCD = 1000; 
bool isPumpOn = false;  // Variable global para rastrear si la bomba está encendida
bool wasPumpOn = false;  // Variable global para rastrear el estado anterior de la bomba


void setup() {
  pinMode(waterPump, OUTPUT);
  digitalWrite(waterPump, HIGH); //Start the waterpump off 
  Serial.begin(9600);
  dht.begin();
  
  lcd.init();
  lcd.backlight(); 

  lcd.setCursor(0, 0);  // Establecer la posición del cursor en la primera línea
  lcd.print("Iniciando...");  // Mostrar el mensaje "Iniciando..." en la pantalla LCD
  delay(4000); 
  
  // Realiza una lectura inicial de los sensores
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  int value = analogRead(moistureSensor);
  soilHumidity = map(value, wet, dry, 100, 0);  // Actualiza la humedad del suelo

  // Actualiza el LCD con los valores iniciales
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(temperature);
  lcd.print((char)223);
  lcd.print(" H: ");
  lcd.print(humidity);
  lcd.print(" %");

  lcd.setCursor(0, 1);
  lcd.print("Hum tierra: ");
  lcd.print(soilHumidity);
  lcd.print(" %");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMoistureMillis >= MOISTURE_INTERVAL) {
    previousMoistureMillis = currentMillis;  // Actualiza el momento de la última lectura
    int value = analogRead(moistureSensor);
  Serial.println(value);
    soilHumidity = map(value, wet, dry, 100, 0);  // Actualiza la humedad del suelo
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();
   }

  if (currentMillis - previousMillisLCD >= intervalLCD) {
    previousMillisLCD = currentMillis;

      if (isPumpOn) {
          lcd.setCursor(0, 0);
          lcd.print("Regando...      ");
          lcd.print("Hum tierra:     ");
          lcd.setCursor(12, 1);
          lcd.print(soilHumidity);
          lcd.print("% ");
      } else {
          lcd.setCursor(0, 0);
          lcd.print("T: ");
          lcd.print(temperature);
          lcd.print((char)223);
          lcd.print(" H: ");
          lcd.print(humidity);
          lcd.print("%");
  
          lcd.setCursor(0, 1);
          lcd.print("Hum tierra:    ");
          lcd.setCursor(12, 1);
          lcd.print(soilHumidity);
          lcd.print("% ");
       }
    } 

  if (!isPumpOn && (currentMillis - previousMillisAfterWatering >= AFTER_WATERING_INTERVAL)) {
    if (soilHumidity < 55) {
        isPumpOn = true;
        previousMillisPump = currentMillis;
        digitalWrite(waterPump, LOW);
      }
  }
    
  if (isPumpOn && (currentMillis - previousMillisPump >= wateringInterval)) {
    isPumpOn = false;
    previousMillisAfterWatering = currentMillis;
    digitalWrite(waterPump, HIGH);
  }

  if (soilHumidity > 65) {  // Apagar la bomba si la humedad del suelo sube por encima del 65%
    isPumpOn = false;
    previousMillisAfterWatering = currentMillis;
    digitalWrite(waterPump, HIGH);
  }
}

void limpiarFilaLCD(int fila) {
  lcd.setCursor(0, fila); // Posiciona el cursor en la primera línea
  lcd.print("                "); // Escribe 16 espacios en blanco (para una pantalla de 16x2)
  lcd.setCursor(0, fila); 
}
