/*
	Project Name:
	Battery operated garage parking assistant

	Author:
	Letsdoino

	Link to the full guide:
	https://letsdoino.com/2019/01/20/garage-parking-assistant/

*/

/* include libraries */
#include <PinChangeInterrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

/* ATtiny85 pin connections */
# define SR04_VCC_PIN 0 // sr04 power supply
# define TRIG_PIN 1 // sr04 trigger
# define ECHO_PIN 2 // sr04 echo
# define LED_PIN 3 // sr04 echo
# define PHOTORES_PIN 4 // photoresistor pin

/* device settings */
# define DISTANCE_LIMIT_MIN 150 // at this distance [cm] the LED blinks at minimum frequency
# define DISTANCE_LIMIT_MAX 5 // at this distance [cm] the LED blinks at maximum frequency
# define DEVICE_TIMER_ON 120 // device on timer [sec] from the garage open event

/* additional defines */
# define MAX_DISTANCE 400 // sr04 max distance in centimeters
# define swap(a,b) a ^= b; b ^= a; a ^= b;
# define sort(a,b) if(a>b){ swap(a,b); }
# ifndef cbi
# define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
# endif
# ifndef sbi
# define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
# endif

/* global variables */
int distance; // distance measured by sr04 sensor
int filt_distance; // filtered distance
int filt_distance_old = 400; // saved filtered distance
int photores_read = LOW; // status of the photoresistor pin
int lastReadings[5]; // array of the last 5 sr04 (filtered) distances
long duration; // echo pulse duration [us]
unsigned long blink_timer_start; // initial time for led blinking [ms]
unsigned long device_timer_start; // initial time device on timer [ms]

/* arduino setup function */
void setup() 
{
  /* disable adc, brown-out, pin change interrupt */
  MCUCR |= _BV(BODS) | _BV(BODSE); //turn off the brown-out detector
  ADCSRA &= ~ bit(ADEN); // disable the ADC
  bitSet(PRR, PRADC); // power down the ADC
  bitSet(PRR, PRUSI); // disable USI h/w  
  attachPCINT(digitalPinToPinChangeInterrupt(PHOTORES_PIN), wakeUpNow, RISING); // wake up at rising event

  /* set pin directions */
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PHOTORES_PIN, INPUT);  
  pinMode(SR04_VCC_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  /* turn on the sr04 and fill distance array */
  digitalWrite(SR04_VCC_PIN, HIGH);
  for(int i=0; i<5; i++) lastReadings[i] = MAX_DISTANCE;
} /* end of arduino setup function */

/* arduino loop function */
void loop() 
{
  /* device on notification and setup */
  disable_interrupt(PHOTORES_PIN); // disable photoresistor interrupt
  device_on_notification(); // led blinking notification
  digitalWrite(SR04_VCC_PIN, HIGH); // turn on the sr04
  device_timer_start = millis(); // save device on start time
  blink_timer_start=millis();

  /* parking assistant active loop */
  while(read_timer(device_timer_start) < DEVICE_TIMER_ON*1000)
  {
    push(read_distance()); // read the distance and push it into the readings buffer
    filt_distance = median(lastReadings[0],lastReadings[1],lastReadings[2],lastReadings[3],lastReadings[4]); // compute the fitered distance value
    (filt_distance > DISTANCE_LIMIT_MIN) ? digitalWrite(LED_PIN, 0) : blink(map(filt_distance, DISTANCE_LIMIT_MAX, DISTANCE_LIMIT_MIN, 50, 800)); // compute distance indicator
  } /* end of parking assistant active loop */

  /* device off setup before going to sleep */
  digitalWrite(SR04_VCC_PIN, LOW); // turn off the sr04
  digitalWrite(LED_PIN, LOW); // turn off the led
  enable_interrupt(PHOTORES_PIN); // enable photoresistor interrupt
  
  /* device sleep */
  photores_read = LOW;
  while (photores_read == LOW) { // make sure it exit from sleep when photoresistor pin is HIGH only (when light is detected)
    system_sleep(SLEEP_MODE_PWR_DOWN);
    delay(5);
    photores_read = digitalRead(PHOTORES_PIN);
  }
} /* end of arduino loop function */

/* led blink function */
void blink(int time) 
{
  if (read_timer(blink_timer_start)>=time) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // swap led status
    blink_timer_start=millis();
  }
}

/* device on notification function */
void device_on_notification() 
{
  for(int i=0; i<6; i++) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(350);
  }
}

/* enable pin change interrupt function */
void disable_interrupt (char pin) 
{
  disablePinChangeInterrupt(digitalPinToPinChangeInterrupt(pin));
  ;
}

/* enable pin change interrupt function */
void enable_interrupt (char pin) 
{
  enablePinChangeInterrupt(digitalPinToPinChangeInterrupt(pin));
  ;
}

/* computes the median of the last five distance reading. Thanks to:
http://dlacko.org/blog/2016/01/24/remove-impulse-noise-from-ultrasonic/ */
int median(int a, int b, int c, int d, int e)
{
    sort(a,b);
    sort(d,e);  
    sort(a,c);
    sort(b,c);
    sort(a,d);  
    sort(c,d);
    sort(b,e);
    sort(b,c);
    return c;
}

/* function to push the distance into the buffer (LIFO logic: last in first out) */
void push(int value)
{
  for(int i=4; i>=1; i--) lastReadings[i] = lastReadings[i-1];
  lastReadings[0]=value;
}

/* reads the distance in centimeters according to sr04 datasheet */
int read_distance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance= duration*0.034/2;
  return distance;
}

/* read timer function */
unsigned long read_timer(unsigned long timer_start)
{
  return (millis()-timer_start);
  ;
}

// system wakes up when watchdog is timed out
void system_sleep(char sleep_mode) 
{
  cbi(ADCSRA,ADEN);
  set_sleep_mode(sleep_mode);
  sleep_enable();
  sleep_mode();
  sleep_disable();
  sbi(ADCSRA,ADEN);
}

/* wake up after sleep function */
void wakeUpNow() 
{
  // do nothing
  ;
}
