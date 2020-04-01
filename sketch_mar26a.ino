#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "qsop.h"
#include <AltSoftSerial.h>

AltSoftSerial altSerial;

int _strcmp(char *s1, char *s2) {
  int i = 0;
  while (s1[i]) {
    if (s1[i] != s2[i]) {
      return 0;
    }
    i++;
  }
  return 1;
}
int _atoi(char const *c) {
  int value = 0;
  int positive = 1;
  if (*c == '\0') return 0;
  if (*c == '-' ) positive = -1;
  while (*c) {
    if ('0' <= *c && *c <= '9')
      value = ((value * 10) + *c) - '0';
    c++;
  }
  return value * positive;
}

void pcint_setup() {
  PCICR  |= 0b100;    // pcint active 0
  PCMSK2 |= 0x80;    // pcint pin position 0, ardu 8
}
void timer2_setup() {
  TCCR2A = 0x03; // Mode3 Fast PWM
  TCCR2B = 0b111; // clkT2S/64 (From prescaler) 4us Period
  TIMSK2 = 0x01; // Timer/Counter2 Overflow Interrupt Enable
  TCNT2  = 130;    // 16000000/64/(256-6)=1000Hz=1ms
}
void adc_setup() {

  //ADLAR = 0;

  DIDR0  = 0b1111;         // 디지털 IO 끄는것

  ADMUX = 0;
  ADMUX |= 0x01;         // MUX0
  //ADMUX |= 0x02;         // MUX1
  //ADMUX |= 0x04;         // MUX2
  //ADMUX |= 0x08;         // MUX3
  //ADMUX |= 0x10;
  //ADMUX |= 0x20;         // ADLAR: ADC Left Adjust Result
  ADMUX |= 0x40;         // REFS0 : Voltage Reference Selection
  //ADMUX |= 0x80;         // REFS1 : Voltage Reference Selection

  ADCSRA  = 0;
  ADCSRA |= 0x01;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x02;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x04;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x08;         // ADIE: ADC Interrupt Enable
  //ADCSRA |= 0x10;         // ADIF: ADC Interrupt Flag
  //ADCSRA |= 0x20;         // ADATE: ADC Auto Trigger Enable
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
  ADCSRA |= 0x80;         // ADEN: ADC Enable

  //ADMUX = 0, ADCSRA  = 0;
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
}

///////////////////////////////////////////////////
uint32_t command_timer          = 0;
uint8_t  command_temp           = 0;
uint8_t  command_power          = 0;
///////////////////////////////////////////////////
uint16_t ADC_RAW[4]             = {0,};
double   ADC_QUEUE[4]           = {0,};
double   &state_temp_water      = ADC_QUEUE[0];
double   &state_temp_heater     = ADC_QUEUE[1];
double   &state_temp_heatsink   = ADC_QUEUE[2];
uint32_t state_timer            = 0;
uint8_t  state_power            = 0;
///////////////////////////////////////////////////
/*alert CONTROL*/
uint8_t alert                   = 0;
uint8_t alert_send_flag         = 0;
///////////////////////////////////////////////////
/*POWER CONTROL*/
uint8_t duty                    = 0;
uint8_t count                   = 0;
uint8_t state_pin               = 0;
///////////////////////////////////////////////////
/*PID CONTROL*/
double pid_value                = 0;
///////////////////////////////////////////////////

void p_control() {
  double error = ((double)command_temp - (double)state_temp_water ) * 1;
  if (error < 0    ) error = 0;
  if (1023  < error) error = 1023;
  duty = (uint8_t)(error / 4);
  //double th = error;
  //if (th < 0   ) th = 0;
  //if (th < 1024) th = 1024;
  //duty = (uint8_t)(th / 4);
}

void alert_temp_cheack() {
  // 히터랑 물온도랑 차이가 30을 초과하면 문제가 있다고 판단
  if ((state_temp_heater - state_temp_water) > 30) {
    alert |= 0b001;
  }
  // 히트싱크 온도가 너무 높아지면 문제가 있다고 판단
  if ((state_temp_heatsink) > 90) {
    alert |= 0b010;
  }
  // 히터 온도가 너무 높아지면 문제가 있다고 판단
  if ((state_temp_heater) > 80) {
    alert |= 0b100;
  }
}

double adc_to_temp(unsigned int qq) {
  uint16_t *a = adc_to_temp_loockup_table;
  return ((int16_t)pgm_read_word_near(a + qq)) * 0.03125;
}
void serial_communication() {

  static char qerer[129];

  if (Serial.available()) {
    char s = Serial.read();
    if (s == '#') qerer[-1] = 0;
    else if (s == ';') {

      qerer[qerer[-1]] = 0;

      if (_strcmp("temp=", qerer)) {
        uint8_t command_temp_buffer = _atoi(qerer + 5);
        if ((0 < command_temp_buffer) && (command_temp_buffer <= 80)) {
          command_temp = command_temp_buffer;
        }
        else command_temp = 0;
        altSerial.print("ok:"), altSerial.println(command_temp);
      }

      else if (_strcmp("timer=", qerer)) {
        command_timer = _atoi(qerer + 5);
        altSerial.print("ok:"), altSerial.println(command_timer);
        state_timer = command_timer;
      }

      else if (_strcmp("power=", qerer)) {
        command_power = _atoi(qerer + 6);
        altSerial.print("ok:"), altSerial.println(command_power);
        state_power = command_power;
      }

      else if (_strcmp("data*", qerer) | _strcmp("d*", qerer)) {
        altSerial.print("ok:");
        altSerial.print(alert), altSerial.print(",");
        altSerial.print(state_timer), altSerial.print(",");
        altSerial.print(state_temp_water), altSerial.print(",");
        altSerial.print(state_temp_heater), altSerial.print(",");
        altSerial.print(state_temp_heatsink), altSerial.print("\n");
      }

      else if (_strcmp("reset*", qerer)) {
        command_temp  = 0;
        command_timer = 0;
        command_power = 0;
      }

      // test function
      else if (_strcmp("duty=", qerer)) {
        duty = _atoi(qerer + 5);
        altSerial.print("ok:"), altSerial.println(duty);
      }

      else if (_strcmp("start*", qerer)) {
        altSerial.print("ok:"), altSerial.println("start");
      }

    }
    else qerer[qerer[-1]] = s, qerer[-1] += 1, qerer[-1] &= 0b01111111;
  }
}

ISR(ADC_vect) {
  static uint8_t port_number;
  alert_temp_cheack();
  if (alert != 0) {
    //state_pin = 0, digitalWrite(4, LOW);
  }
  ADC_RAW[port_number] = ADCW;
  ADC_QUEUE[port_number] = adc_to_temp(ADCW);
  port_number++;
  port_number &= 0b11;
  ADMUX  &= 0xF0;
  ADMUX  |= port_number;
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
}
ISR(PCINT0_vect) {}
ISR(TIMER2_OVF_vect) {

  TCNT2  = 130;

  //if ((count > 0) && (state_timer != -1)) state_timer--;

  if (duty < (uint8_t)24) {
    p_control();
    //if ((count < duty) && (alert == 0) && (state_power == 1)) state_pin = 1, digitalWrite(4, HIGH);
    if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite(4, HIGH);
    else state_pin = 0, digitalWrite(4, LOW);
    count += 1;
  }

  else if (((uint8_t)24 <= duty) && (duty < (uint8_t)96)) {
    p_control();
    //if ((count < duty) && (alert == 0) && (state_power == 0)) state_pin = 1, digitalWrite(4, HIGH);
    if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite(4, HIGH);
    else state_pin = 0, digitalWrite(4, LOW);
    count += 4;
  }

  else if ((uint8_t)96 <= duty) {
    p_control();
    //if ((count < duty) && (alert == 0) && (state_power == 0)) state_pin = 1, digitalWrite(4, HIGH);
    if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite(4, HIGH);
    else state_pin = 0, digitalWrite(4, LOW);
    count += 16;
  }

}

void setup() {}
void loop() {
  cli();
  sei();
  Serial.begin(115200);
  altSerial.begin(9600);
  pinMode(4, digitalWrite);
  pinMode(10, digitalRead);
  pinMode(11, digitalRead);
  pinMode(12, digitalRead);
  pcint_setup();
  timer2_setup();
  adc_setup();
  //adc_to_temp_setup();
  while (1) {

    serial_communication();

    if (alert != 0) {
      if (alert &= 0b100) {

      }
      if (alert &= 0b010) {

      }
      if (alert &= 0b001) {

      }
      if (alert != 0b000) {
        if ((digitalRead(10) != HIGH) && (digitalRead(10) != HIGH)) alert = 0;
      }
    } else {

    }

    if (1) {
      Serial.print("command_temp: "); Serial.print(command_temp); Serial.print("  ");
      Serial.print("state_timer: "); Serial.print(state_timer); Serial.print("  ");
      Serial.print("pid_value: "); Serial.print(pid_value); Serial.print("  ");
      Serial.print("count: "); Serial.print(count * 4); Serial.print("  ");
      Serial.print("state_pin: "); Serial.print(state_pin * 512); Serial.print("  ");
      Serial.print("duty: "); Serial.print(duty); Serial.print("  ");
      Serial.print("alert: "); Serial.print(alert); Serial.print("  ");
      Serial.print("state_power: "); Serial.print(state_power); Serial.print("  ");
      Serial.print("state_temp_water: "); Serial.print(state_temp_water); Serial.print("  ");
      Serial.print("state_temp_heater: "); Serial.print(state_temp_heater); Serial.print("  ");
      Serial.print("state_temp_heatsink: "); Serial.print(state_temp_heatsink); Serial.print("  ");
      Serial.print("X: "); Serial.print(1024); Serial.print("  ");
      Serial.println("uT");
    }
    
  }
}
