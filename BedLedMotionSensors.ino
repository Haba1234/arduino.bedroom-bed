// frimware Optiboot v6.2 16MHz

// Enable debug prints
#define MY_DEBUG

//#define MY_REPEATER_NODE

// Enable and select radio type attached
#define MY_RADIO_NRF24

// Enable repeater feature
#define MY_REPEATER_FEATURE

#define MY_RF24_PA_LEVEL RF24_PA_HIGH

#define MY_NODE_ID 50

#include <SPI.h>
#include <MySensors.h>
#include <elapsedMillis.h>

bool ON_Led = false; // Включение LED
bool LED_switch = false; // дистанционное включение подсветки
bool Bed_LED_OFF = false;    // дистанционный запрет включения
bool send_on;
bool sens1_old;
bool sens2_old;
int PWM_OUT;
const int hysteresys = 10;
int sens3_old = 0; 
uint8_t level;
int PWM_time = 100;
byte PWM_max = 255;

elapsedMillis timeBrigth;
unsigned int interval = 5000; // задержка отправки уровней яркости (антиспам)

//unsigned long SLEEP_TIME = 120000; // Sleep time between reports (in milliseconds)
unsigned long CURRENT_TIME;
#define DIGITAL_INPUT_SENSOR1 2   // Датчик движения 1
#define DIGITAL_INPUT_SENSOR2 3   // Датчик движения 2
//#define INTERRUPT DIGITAL_INPUT_SENSOR-2 // Usually the interrupt = pin -2 (on uno/nano anyway)
#define SENSOR1 1      // id для Датчик движения 1
#define SENSOR2 2      // id для Датчик движения 2
#define SENSOR3 3      // id для датчика света
#define SENSOR4 4      // id для выключателя света
#define SENSOR5 5      // id для порог срабатывания датчика света
#define SENSOR6 6      // id для Запрет на включение
#define SENSOR7 7      // id для Скорость ШИМ (мс)
#define SENSOR8 8      // id для максимальная яркость от 0 до 255 (от ИоБ передается в % от 0 до 100)

#define OUTPUT_PWM 5    // Вывод для управления LED через ШИМ
#define FOTORESIST A2   // Датчик света

// Initialize motion message
MyMessage Sens1msg(SENSOR1, V_TRIPPED);
MyMessage Sens2msg(SENSOR2, V_TRIPPED);
MyMessage Sens3msg(SENSOR3, V_LEVEL);
MyMessage Sens4msg(SENSOR4, V_STATUS);
MyMessage Sens5msg(SENSOR5, V_LEVEL);
MyMessage Sens6msg(SENSOR6, V_STATUS);
MyMessage Sens71msg(SENSOR7, V_VAR1);
MyMessage Sens72msg(SENSOR7, V_VAR2);

void setup()  
{  
  pinMode(DIGITAL_INPUT_SENSOR1, INPUT);      // sets the motion sensor digital pin as input
  pinMode(DIGITAL_INPUT_SENSOR2, INPUT);
  pinMode(OUTPUT_PWM, OUTPUT);
  level = loadState(SENSOR5);    
}

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Bed LED", "1.5.2");

  // Register all sensors to gw (they will be created as child devices)
  present(SENSOR1, S_MOTION, "Motion 1");
  present(SENSOR2, S_MOTION, "Motion 2");
  present(SENSOR3, S_LIGHT_LEVEL, "Light");
  present(SENSOR4, S_LIGHT, "LED ON/OFF");
  present(SENSOR5, S_LIGHT_LEVEL, "Light level");
  present(SENSOR6, S_BINARY, "Switch");
  present(SENSOR7, S_CUSTOM, "Settings");
  send(Sens1msg.set("0"), false);
  send(Sens2msg.set("0"), false);
  //send(Sens4msg.set(LED_switch), false);
  wait(30);
  request(SENSOR4, V_STATUS); // выключатель света
  wait(50);
  request(SENSOR6, V_STATUS); // Запрет на включение
  wait(50);
  request(SENSOR7, V_VAR1); // ШИМ. Скорость (мс)
  wait(50);
  request(SENSOR7, V_VAR2); // ШИМ. Максимальная яркость
  wait(50);
}

void loop()     
{     
  // Read digital motion value
  boolean tripped1 = digitalRead(DIGITAL_INPUT_SENSOR1) == HIGH; // Читаем состояние датчика движения 1
  boolean tripped2 = digitalRead(DIGITAL_INPUT_SENSOR2) == HIGH; // Читаем состояние датчика движения 2
  int brigth = analogRead(FOTORESIST); // читаем значение с фоторезистора

  if (!LED_switch && !tripped1 && !tripped2) ON_Led = false;
  if (LED_switch) {
    ON_Led = true; //проверяем есть дистанционная команда вкл? 
  } else if (brigth < level)  // проверяем порог освещения комнаты
      if (tripped1 || tripped2) ON_Led = true; // Есть сработавший датчик?
  if (Bed_LED_OFF) ON_Led = false;   // есть запрет на включение подсветки
  
  if (tripped1 != sens1_old) {
    sens1_old = tripped1;
    send(Sens1msg.set(tripped1?"1":"0"), false);  // Send tripped value to gw
  }
  if (tripped2 != sens2_old) {
    sens2_old = tripped2;
    send(Sens2msg.set(tripped2?"1":"0"), false);  // Send tripped value to gw
  }
        
  if ((timeBrigth > interval)&&((brigth > sens3_old+hysteresys)||(brigth < sens3_old-hysteresys))){
    send(Sens3msg.set(brigth), false);            // Send light level value to gw  
    sens3_old = brigth;
    timeBrigth = 0; // сбрасываем таймер
  }
  
  if ((ON_Led) && (PWM_OUT < PWM_max)) PWM_OUT = PWM_OUT + 1;
  if ((!ON_Led) && (PWM_OUT > 0)) PWM_OUT = PWM_OUT - 1;
  analogWrite(OUTPUT_PWM, PWM_OUT);
  wait(PWM_time);
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  if ((message.sensor == SENSOR4)&&(message.type==V_STATUS)) {
     // Change relay state
     LED_switch = message.getBool()?true:false;
     bool test = send(Sens4msg.set(LED_switch), false);
     // Write some debug info
     #ifdef MY_DEBUG
        Serial.print("Send ACK: ");
        Serial.println(test);
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getBool());
     #endif
  }
  if ((message.sensor == SENSOR5)&&(message.type==V_LEVEL)) {
     // новый порог яркости
     level = message.getByte();
     saveState(SENSOR5, message.getByte());
     bool test = send(Sens5msg.set(level), false);
     // Write some debug info
     #ifdef MY_DEBUG
        Serial.print("Send ACK: ");
        Serial.println(test);
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getByte());
     #endif
  }
     if ((message.sensor == SENSOR6)&&(message.type==V_STATUS)) {
     // Change relay state
     Bed_LED_OFF = message.getBool()?true:false;
     bool test = send(Sens6msg.set(Bed_LED_OFF), false);
     // Write some debug info
     #ifdef MY_DEBUG
        Serial.print("Send ACK: ");
        Serial.println(test);
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(message.getBool());
     #endif
  }
  if (message.sensor == SENSOR7){
    if (message.type==V_VAR1){
      // Change relay state
      PWM_time = message.getInt();
      bool test = send(Sens71msg.set(PWM_time), false);
      // Write some debug info
      #ifdef MY_DEBUG
        Serial.print("Send ACK: ");
        Serial.println(test);
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(PWM_time);
      #endif
    }
    if (message.type==V_VAR2){
      // Change relay state
      int tmp = message.getInt();
      bool test = send(Sens72msg.set(tmp), false);
      PWM_max = byte(round(tmp * 255 / 100));
      // Write some debug info
      #ifdef MY_DEBUG
        Serial.print("Send ACK: ");
        Serial.println(test);
        Serial.print("Incoming change for sensor:");
        Serial.print(message.sensor);
        Serial.print(", New status: ");
        Serial.println(PWM_max);
      #endif
    }
  }
}
