#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

LiquidCrystal_I2C lcd(0x27, 16, 4);

const int trigPin = 42;
const int echoPin = 40;
bool lowWaterLevel = false;
unsigned long lowWaterLevelStartTime;

unsigned long duration;
float distance;

#define moistureSensor 3
#define wet 413
#define dry 819
#define EMPTY_WATER_LEVEL 32.63
#define MIN_WATER_LEVEL 29

int waterPump = 5;
int ambientSensor = 1;
float temperature;
int humidity;
DHT dht (ambientSensor, DHT22);

#define MOISTURE_INTERVAL 1000  // Intervalo para las lecturas del sensor de humedad del suelo (en milisegundos)
#define AFTER_WATERING_INTERVAL 30000 // Intervalo después de que dejo de regar
unsigned long previousMoistureMillis = 0;
int soilHumidity;
int minSoilHumidity = 60;
int maxSoilHumidity = 70;
int previousSoilHumidity;

const long wateringInterval = 8000;
unsigned long previousMillisLCD = 0; 
unsigned long previousMillisPump = 0;
unsigned long previousMillisAfterWatering = 0;
const long intervalLCD = 1000; 
bool isPumpOn = false;  // Variable global para rastrear si la bomba está encendida
bool wasPumpOn = false;  // Variable global para rastrear el estado anterior de la bomba

String message;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
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
    soilHumidity = map(value, wet, dry, 100, 0);  // Actualiza la humedad del suelo
    temperature = dht.readTemperature();
    humidity = dht.readHumidity();

    

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
  
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
  
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH);
    distance = duration*0.0343/2;
    // Verificar la distancia medida
      if (distance > MIN_WATER_LEVEL) {
        Serial.println("Low water level");
        lowWaterLevel = true;
        lowWaterLevelStartTime = currentMillis;
      }
    }

  if (currentMillis - previousMillisLCD >= intervalLCD) {
    previousMillisLCD = currentMillis;

    Serial.print("Nivel de agua: ");
    Serial.println(distance);
    
    Serial.print("Humedad de tierra: ");
    Serial.println(soilHumidity);

    Serial.println(message);
      if (isPumpOn) {
          Serial.println("Regando... ");
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
    if (soilHumidity < minSoilHumidity) {
      if (lowWaterLevel) {
        Serial.println("Low water level");
        lcd.setCursor(0, 0);
        lcd.print("Low water level ");
        lcd.setCursor(0, 1);
        lcd.print("                 ");
       
        // Verificar si ha pasado el tiempo de duración del mensaje
        if (currentMillis - lowWaterLevelStartTime >= 3000) {
          lowWaterLevel = false;
        }
      }
       else {
        isPumpOn = true;
        previousMillisPump = currentMillis;
        digitalWrite(waterPump, LOW);
        message = "Water pump ON";
       }  
    }
  }
    
  if (isPumpOn && (currentMillis - previousMillisPump >= wateringInterval)) {
    isPumpOn = false;
    previousMillisAfterWatering = currentMillis;
    digitalWrite(waterPump, HIGH);
    message = "Water pump OFF";
  }

  if (soilHumidity > maxSoilHumidity) {  // Apagar la bomba si la humedad del suelo sube por encima del 65%
    isPumpOn = false;
    previousMillisAfterWatering = currentMillis;
    digitalWrite(waterPump, HIGH);
    message = "Water pump OFF";
  }
}


void limpiarFilaLCD(int fila) {
  lcd.setCursor(0, fila); // Posiciona el cursor en la primera línea
  lcd.print("                "); // Escribe 16 espacios en blanco (para una pantalla de 16x2)
  lcd.setCursor(0, fila); 
}
