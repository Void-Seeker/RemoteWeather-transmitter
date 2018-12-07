#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control
#include <stdio.h>
#include <math.h>
#include <RH_ASK.h>
#include <RHDatagram.h>
#include <Wire.h>
#include <SPI.h>
#include "SparkFunBME280.h"
//#define DEBUG
#define PWRPIN 2
#define TXPIN 13
#define CLIENT_ADDRESS 101
#define SERVER_ADDRESS 1
#define PROTOCOL 3579
//#define SLEEP_CYCLES 37
#define SLEEP_CYCLES 7
#define PACKET_COUNT 5
#define PACKET_DELAY 200
volatile int nbr_remaining;
RH_ASK driver(2000, NULL, TXPIN);
RHDatagram manager(driver, CLIENT_ADDRESS);
BME280 bme280;
// interrupt raised by the watchdog firing
// when the watchdog fires, this function will be executed
// remember that interrupts are disabled in ISR functions
// now we must check if the board is sleeping or if this is a genuine
// interrupt (hanging)
ISR(WDT_vect)
{
    // Check if we are in sleep mode or it is a genuine WDR.
    if(nbr_remaining > 0) {
        // not hang out, just waiting
        // decrease the number of remaining avail loops
        nbr_remaining = nbr_remaining - 1;
        wdt_reset();
    } else {
        // must be rebooted
        // configure
        MCUSR = 0;                          // reset flags

        // Put timer in reset-only mode:
        WDTCSR |= 0b00011000;               // Enter config mode.
        WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
        // set WDE (reset enable...4th from left), and set delay interval
        // reset system in 16 ms...
        // unless wdt_disable() in loop() is reached first

        // reboot
#ifdef DEBUG
        Serial.println("Rebooting system...");
#endif
        while(1);
    }
}

// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution by default
// hangs will correspond to watchdog fired when nbr_remaining <= 0 and will
// be determined in the ISR function
void configure_wdt(void)
{

    cli();                           // disable interrupts for changing the registers

    MCUSR = 0;                       // reset status register flags

    // Put timer in interrupt-only mode:
    WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
    // using bitwise OR assignment (leaves other bits unchanged).
    WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
    // clr WDE: reset disabled
    // and set delay interval (right side of bar) to 8 seconds

    sei();                           // re-enable interrupts
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{
#ifdef DEBUG
    Serial.flush();
#endif
    nbr_remaining = ncycles; // defines how many cycles should sleep

    // Set sleep to full power down.  Only external interrupts or
    // the watchdog timer can wake the CPU!
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    // Disable digital input buffers on all ADC0-ADC5 pins
    DIDR0 = 0x3F;
    // set I2C pin as input no pull up
    // this prevent current draw on I2C pins that
    // completly destroy our low power mode

    //Disable I2C interface so we can control the SDA and SCL pins directly
    TWCR &= ~(_BV(TWEN));

    // disable I2C module this allow us to control
    // SCA/SCL pins and reinitialize the I2C bus at wake up
    TWCR = 0;
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    // Turn off the ADC while asleep.
    power_adc_disable();
    power_twi_disable();
    //delay(100);
    pinMode(TXPIN, INPUT);
    digitalWrite(TXPIN, LOW);
    pinMode(PWRPIN, INPUT);
    digitalWrite(PWRPIN, LOW);

    while (nbr_remaining > 0) { // while some cycles left, sleep!

        // Enable sleep and enter sleep mode.
        sleep_mode();

        // CPU is now asleep and program execution completely halts!
        // Once awake, execution will resume at this point if the
        // watchdog is configured for resume rather than restart

        // When awake, disable sleep mode
        sleep_disable();
    }
    // put everything on again
    pinMode(PWRPIN, OUTPUT);
    digitalWrite(PWRPIN, HIGH);
    pinMode(TXPIN, OUTPUT);
    //delay(500);
    power_all_enable();
    //delay(500);
}
int getBattVolts()   // Returns actual value of Vcc (x 100)
{

    const long InternalReferenceVoltage = 1080L;  // Adjust this value to your boards specific internal BG voltage x1000
    // Start a conversion
    ADCSRA |= _BV( ADSC );
    // Wait for it to complete
    while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
    // Scale the value
    int results = (((InternalReferenceVoltage * 1024L) / ADC) + 120L) / 10L;
    return results;
}
void setup()
{
    //TODO: ADC config for AtMega328, other platforms may have other settings.
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
    pinMode(PWRPIN, OUTPUT);
    digitalWrite(PWRPIN, HIGH);
    delay(1000);
#ifdef DEBUG
    Serial.begin(9600);   // DEBUGging only
#endif
    if (!manager.init())
#ifdef DEBUG
        Serial.println("init failed");
#else
        ;
#endif
    Wire.begin();
    Wire.setClock(400000);   //Increase to fast I2C speed!
    bme280.setI2CAddress(0x76);
    configure_wdt();
}
void loop()
{
#ifdef DEBUG
    if (bme280.beginI2C()) Serial.println("Sensor initialized.");
    else {
        Serial.println("Sensor missing...");
        while (1) {}
    };
#else 
    if (bme280.beginI2C());
    else while (1) {};
#endif
    bme280.setMode(MODE_SLEEP);
    bme280.setFilter(4); //0 to 4 is valid. Filter coefficient. See 3.4.4
    bme280.setStandbyTime(1); //0 to 7 valid. Time between readings. See table 27.
    bme280.setTempOverSample(16); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    bme280.setPressureOverSample(16); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    bme280.setHumidityOverSample(16); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    bme280.readTempC();
    bme280.readFloatPressure();
    bme280.readFloatHumidity();
    bme280.setMode(MODE_FORCED);
    //unsigned long startTime = millis();
    while(bme280.isMeasuring() == false) ;
    while(bme280.isMeasuring() == true) ;
    //unsigned long endTime = millis();
    char msg[RH_ASK_MAX_MESSAGE_LEN];
    sprintf(msg,"%i,%i,%i,%i,%lu,%u", PROTOCOL, CLIENT_ADDRESS, getBattVolts(), (int)roundf(100.0*bme280.readTempC()), (unsigned long)roundf(bme280.readFloatPressure()), (unsigned int)roundf(bme280.readFloatHumidity()));
#ifdef DEBUG
    Serial.print("Readings: ");
    Serial.println(msg);
    Serial.println("Attempting to send...");
#endif
    for (int i=0; i<PACKET_COUNT; i++) {
        manager.sendto((uint8_t *)msg, strlen(msg), RH_BROADCAST_ADDRESS);
        manager.waitPacketSent();
        delay(PACKET_DELAY);
    }
#ifdef DEBUG
    Serial.println("Sending complete.");
    Serial.print("Entering sleep mode for ");
    Serial.print(SLEEP_CYCLES * 8);
    Serial.println(" seconds...");
#endif
    sleep(SLEEP_CYCLES);
#ifdef DEBUG
    Serial.println("Waking up...");
#endif
}
