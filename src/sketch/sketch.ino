#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <GyverButton.h>
#include <avr/sleep.h>
#include <LowPower.h>
#include <EEPROMex.h>
#include <Wire.h>
#include <SPI.h>

/*--- volts and resistance are indicated in ohms/volts * 10^-2, time are indicated in milliseconds ---*/

#define VOLT_MODE_STEP             10   /*--- the number of volts, which will vary per step in vary watt ---*/
#define VOLT_MODE_STEPS_PER_SECOND 10   /*--- number of change steps per second in vary volt ---*/

#define WATT_MODE_STEP             1    /*--- the number of watts, which will vary per step in vary watt ---*/
#define WATT_MODE_STEPS_PER_SECOND 10   /*--- number of change steps per second in vary watt ---*/
 
#define MIN_CHARGE_VOLTS           300  /* --- min charge of batteries, will not work at such voltage ---*/

#define MAX_WATTS                  200  /* --- power limit in watts --- */

#define VOLTAGE_DEVIDER_RES        2200 /* --- resistance of voltage devider resistor---*/

#define MAX_TURNED_ON_COIL_TIME    10000

/*--- do not change these settings ---*/
#define SCREEN_WIDTH               128
#define SCREEN_HEIGHT              32

#define MAX_CHARGE_VOLTS           420

#define MAX_COIL_RES               1000

#define MIN_COIL_RES               20

#define MIN_WATTS                  1

#define MIN_VOLTS                  50

#define TURN_ON_MEASURING_RES_PIN  5
#define MEASURE_RES_PIN            A0

#define TURN_ON_COIL               11

#define FIRE_BUTTON_PIN            3
#define LEFT_BUTTON_PIN            7
#define RIGHT_BUTTON_PIN           6

#define VARY_VOLT_MODE             67
#define VARY_WATT_MODE             11

#define NUM_VCC_READS              30  /*--- less than 255 ---*/
#define NUM_RES_READS              30  /*--- less than 255 ---*/

#define SLEEP_DELAY_IF_DISCHARGED  10000

#define IS_DISCHARGED              flags  & B00000001
#define IS_OVERCHARGED             flags  & B01000000
#define IS_OVER_COIL_RES           flags  & B00100000
#define IS_SMALL_COIL_RES          flags  & B10000000
#define IS_NO_COIL                 flags  & B00010000
#define IS_WAKED_UP                flags  & B00001000
#define IS_WAKED_PUZZLE            flags  & B00000010

#define SET_DISCHARGED             flags |= B00000001
#define SET_OVERCHARGED            flags |= B01000000
#define SET_OVER_COIL_RES          flags |= B00100000
#define SET_SMALL_COIL_RES         flags |= B10000000
#define SET_NO_COIL                flags |= B00010000
#define SET_WAKED_UP               flags |= B00001000
#define SET_WAKED_PUZZLE           flags |= B00000010
/*--- do not change these settings ---*/

const uint8_t PROGMEM bitmap[] = {
    B00001111, B11000000,
    B00111000, B01110000,
    B01110000, B00111000,
    B01100000, B00011000,
    B11000000, B00001100,
    B11000000, B00001100,
    B11000000, B00001100,
    B11000000, B00001100,
    B01100000, B00011000,
    B01100000, B00011000,
    B00110000, B00110000,
    B10011000, B01100100,
    B11111000, B01111100,
    B11111000, B01111100
};

uint32_t vcc;
uint32_t res;

uint32_t vcc_buff;
uint32_t res_buff;

uint32_t volts_to;
uint32_t watts_to;

uint8_t mode;
uint8_t flags;

uint8_t vcc_reads;
uint8_t res_reads;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 4);

GButton fireBtn(FIRE_BUTTON_PIN);
GButton leftBtn(LEFT_BUTTON_PIN);
GButton rightBtn(RIGHT_BUTTON_PIN);

void setup()
{
    Serial.begin(9600);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        while (1)
            ;
    }

    pinMode(TURN_ON_MEASURING_RES_PIN, OUTPUT);

    pinMode(TURN_ON_COIL, OUTPUT);

    digitalWrite(TURN_ON_MEASURING_RES_PIN, LOW);
    
    digitalWrite(TURN_ON_COIL, LOW);

    rightBtn.setClickTimeout(200);
    rightBtn.setTimeout(300);
    rightBtn.setDebounce(0);

    leftBtn.setClickTimeout(200);
    leftBtn.setTimeout(300);
    leftBtn.setDebounce(0);

    fireBtn.setClickTimeout(200);
    fireBtn.setTimeout(300);
    fireBtn.setDebounce(0);

    rightBtn.setTickMode(AUTO);
    leftBtn.setTickMode(AUTO);
    fireBtn.setTickMode(AUTO);

    display.clearDisplay();

    volts_to = EEPROM.readInt(0);
    watts_to = EEPROM.readInt(2);
    mode     = EEPROM.readByte(4);

    if (!(mode == VARY_WATT_MODE || mode == VARY_VOLT_MODE)) {
        mode = VARY_VOLT_MODE;
    }

    for (int i = 0; i < NUM_VCC_READS; i++) {
        vcc += read_vcc();
    }

    vcc /= (float) NUM_VCC_READS;

    for (int i = 0; i < NUM_RES_READS; i++) {
        res += read_coil_res();
    }
    
    res /= (float) NUM_VCC_READS;

    if (volts_to > vcc) {
        volts_to = vcc;
    } else if (volts_to < MIN_VOLTS) {
        volts_to = MIN_VOLTS;
    }

    uint32_t max_watts = ((vcc / (float)res) * vcc) / 100;
        
    if (watts_to > max_watts || watts_to > MAX_WATTS) {
        watts_to = (max_watts < MAX_WATTS) ?  max_watts : MAX_WATTS;
    }  else if (watts_to < MIN_WATTS) {
        watts_to = MIN_WATTS;
    }
}

void loop()
{
    goto sleep_end;       
sleep:
    sleep();
    if (IS_WAKED_PUZZLE) {
        detachInterrupt(1);
        digitalWrite(TURN_ON_COIL, LOW);
        digitalWrite(TURN_ON_MEASURING_RES_PIN, LOW);
        if (wake_puzzle()){
          display.ssd1306_command(SSD1306_DISPLAYON);
        } else {
          goto sleep;
        }
    }
sleep_end:
      
    flags = 0;
    
    vcc_buff += read_vcc();
    vcc_reads++;
    
    if (vcc_reads == NUM_VCC_READS) {
        vcc = vcc_buff / (float) NUM_VCC_READS;
        vcc_buff = 0;
        vcc_reads = 0;
    }

    res_buff += read_coil_res();
    res_reads++;

    if (res_reads == NUM_RES_READS) {
        res = res_buff / (float) NUM_RES_READS;
        res_buff = 0;
        res_reads = 0;
    }

    if (vcc < MIN_CHARGE_VOLTS) {
        SET_DISCHARGED;
        update_display_buf();
        delay(SLEEP_DELAY_IF_DISCHARGED);
        goto sleep;
    }
    else if (vcc > MAX_CHARGE_VOLTS) {
        SET_OVERCHARGED;
    }
    
    if (res == 0) {
        SET_NO_COIL;
    }
    else if (res >= MAX_COIL_RES) {
        SET_OVER_COIL_RES;
    }
    else if (res < MIN_COIL_RES) {
        SET_SMALL_COIL_RES;
    }

    if (fireBtn.isHold()) {
        uint32_t next_time = millis() + MAX_TURNED_ON_COIL_TIME;
        try_turn_on_coil();
        while(fireBtn.isHold() && millis() < next_time)
            ;
        turn_off_coil();
    }

    if (leftBtn.isDouble()) {
        goto sleep;
    }

    if (rightBtn.isDouble()) {
        if (mode == VARY_WATT_MODE) {
            mode = VARY_VOLT_MODE;
        } else if(mode == VARY_VOLT_MODE){
            mode = VARY_WATT_MODE;
        }
    }
    
    switch (mode) {
    case VARY_VOLT_MODE: {     
        if (volts_to > vcc) {
            volts_to = vcc;
        } else if (volts_to < MIN_VOLTS) {
            volts_to = MIN_VOLTS;
        }
        
        if (rightBtn.isHold()) {
            uint32_t now_time;
            uint32_t last_time = millis();
            uint32_t timeout = (float)1000 / VOLT_MODE_STEPS_PER_SECOND;

            while (rightBtn.isHold()) {
                now_time = millis();
                if (now_time > last_time + timeout) {
                    if (volts_to + VOLT_MODE_STEP <= vcc) {
                        volts_to += VOLT_MODE_STEP;
                    }
                    else {
                        volts_to = vcc;
                    }
                    last_time = now_time;
                }
                update_display_buf();
                display.display();
            }
        }      
        if (leftBtn.isHold()) {
            uint32_t now_time;
            uint32_t last_time = millis();
            uint32_t timeout = (float)1000 / VOLT_MODE_STEPS_PER_SECOND;

            while (leftBtn.isHold()) {
                now_time = millis();
                if (now_time > last_time + timeout) {
                    if (volts_to > MIN_VOLTS + VOLT_MODE_STEP) {
                        volts_to -= VOLT_MODE_STEP;
                    }
                    else {
                        volts_to = MIN_VOLTS;
                    }
                    last_time = now_time;
                }
                update_display_buf();
                display.display();
            }
        }        
    } break;
    case VARY_WATT_MODE: {
        uint32_t max_watts = ((vcc / res) * vcc) / (float)100;
        
        if (watts_to > max_watts || watts_to > MAX_WATTS) {
            watts_to = (max_watts < MAX_WATTS) ?  max_watts : MAX_WATTS;
        }  else if (watts_to < MIN_WATTS) {
            watts_to = MIN_WATTS;
        }

        if (IS_NO_COIL || IS_OVER_COIL_RES || IS_SMALL_COIL_RES) {
          break;
        }
        
        if (rightBtn.isHold()) {
            uint32_t now_time;
            uint32_t last_time = millis();
            uint32_t timeout = (float)1000 / WATT_MODE_STEPS_PER_SECOND;

            while (rightBtn.isHold()) {
                now_time = millis();
                if (now_time > last_time + timeout) {
                    uint32_t temp_watts_to = watts_to + WATT_MODE_STEP;  

                    if (temp_watts_to < MAX_WATTS && temp_watts_to < max_watts) {
                        watts_to = temp_watts_to;
                    }
                    else {
                        watts_to = (max_watts < MAX_WATTS) ? max_watts : MAX_WATTS;
                    }
                    last_time = now_time;
                }
                update_display_buf();
                display.display();
            }
        }
        if (leftBtn.isHold()) {
            uint32_t now_time;
            uint32_t last_time = millis();
            uint32_t timeout = (float)1000 / WATT_MODE_STEPS_PER_SECOND;

            while (leftBtn.isHold()) {
                now_time = millis();
                if (now_time > last_time + timeout) {
                    if (watts_to > MIN_WATTS + WATT_MODE_STEP) {
                        watts_to -= WATT_MODE_STEP;
                    }
                    else {
                        watts_to = MIN_WATTS;
                    }
                    last_time = now_time;
                }
                update_display_buf();
                display.display();
            }
        }
    } break;
    }

    update_display_buf();
    display.display();
}

bool wake_puzzle() {
    uint32_t next_time = millis() + 1000;
  
    while(millis() < next_time) {
        if (fireBtn.isTriple()) {
            return true;
        }
    }
    
    return false;
}

void wake_up()
{
    SET_WAKED_PUZZLE;
}

void sleep()
{
    EEPROM.updateInt(0, volts_to);
    EEPROM.updateInt(2, watts_to);
    EEPROM.updateByte(4, mode);

    digitalWrite(TURN_ON_COIL, LOW);
    digitalWrite(TURN_ON_MEASURING_RES_PIN, LOW);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    attachInterrupt(1, wake_up, FALLING);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

void update_display_buf()
{
    display.clearDisplay();
    
    if (IS_DISCHARGED) {
        display.drawChar(4, 0, 'C', WHITE, BLACK, 2);
        display.drawChar(16, 0, 'H', WHITE, BLACK, 2);
        display.drawChar(28, 0, 'A', WHITE, BLACK, 2);
        display.drawChar(40, 0, 'R', WHITE, BLACK, 2);
        display.drawChar(52, 0, 'G', WHITE, BLACK, 2);
        display.drawChar(64, 0, 'E', WHITE, BLACK, 2);
        display.drawChar(76, 0, ' ', WHITE, BLACK, 2);
        display.drawChar(88, 0, 'T', WHITE, BLACK, 2);
        display.drawChar(100, 0, 'H', WHITE, BLACK, 2);
        display.drawChar(112, 0, 'E', WHITE, BLACK, 2);

        display.drawChar(22, 16, 'D', WHITE, BLACK, 2);
        display.drawChar(34, 16, 'E', WHITE, BLACK, 2);
        display.drawChar(46, 16, 'V', WHITE, BLACK, 2);
        display.drawChar(58, 16, 'I', WHITE, BLACK, 2);
        display.drawChar(70, 16, 'C', WHITE, BLACK, 2);
        display.drawChar(82, 16, 'E', WHITE, BLACK, 2);
        display.drawChar(94, 16, '!', WHITE, BLACK, 2);
        return;
    }
    
    switch (mode) {
    case VARY_VOLT_MODE: {
        uint32_t temp_volts_to = volts_to / 10;
        display.drawChar(36, 5, '0' + temp_volts_to % 10, WHITE, BLACK, 3);
        temp_volts_to /= 10;
        display.drawChar(0, 5, '0' + temp_volts_to % 10, WHITE, BLACK, 3);
        display.drawChar(18, 5, '.', WHITE, BLACK, 3);
        display.drawChar(54, 5, 'V', WHITE, BLACK, 3);
    } break;
    case VARY_WATT_MODE: {
        uint32_t temp_watts_to = watts_to;
        for (int i = 2; i >= 0, temp_watts_to > 0; i--) {
            display.drawChar(i * 18, 5, '0' + temp_watts_to % 10, WHITE, BLACK, 3);
            temp_watts_to /= 10;
        }
        display.drawChar(54, 5, 'W', WHITE, BLACK, 3);
    } break;
    }

    if (IS_NO_COIL) {
        display.drawChar(100, 0, '?', WHITE, BLACK, 2);
        display.drawChar(88, 0, '?', WHITE, BLACK, 2);
        display.drawChar(72, 0, '?', WHITE, BLACK, 2);
    }
    else if (IS_OVER_COIL_RES) {
        display.drawChar(100, 0, '+', WHITE, BLACK, 2);
        display.drawChar(88, 0, '+', WHITE, BLACK, 2);
        display.drawChar(72, 0, '+', WHITE, BLACK, 2);
    }
    else if (IS_SMALL_COIL_RES) {
        display.drawChar(100, 0, '-', WHITE, BLACK, 2);
        display.drawChar(88, 0, '-', WHITE, BLACK, 2);
        display.drawChar(72, 0, '-', WHITE, BLACK, 2);
    }
    else {
        uint32_t temp_res = res;
        display.drawChar(100, 0, '0' + temp_res % 10, WHITE, BLACK, 2);
        temp_res /= 10;
        display.drawChar(88, 0, '0' + temp_res % 10, WHITE, BLACK, 2);
        temp_res /= 10;
        display.drawChar(72, 0, '0' + temp_res % 10, WHITE, BLACK, 2);
        display.drawBitmap(112, 0, bitmap, 14, 14, WHITE);
    }

    display.drawRect(84, 12, 2, 2, WHITE);
    display.drawBitmap(112, 0, bitmap, 14, 14, WHITE);
    
    display.drawRoundRect(72, 16, 56, 16, 4, WHITE);

    if (IS_OVERCHARGED) {
        for (int i = 0; i < 8; i++) {
            display.drawChar(76 + i * 6, 20, '?', WHITE, BLACK, 1);
        }
    }
    else {
        display.fillRoundRect(72, 16, (vcc - MIN_CHARGE_VOLTS) * 56 / (float)(MAX_CHARGE_VOLTS - MIN_CHARGE_VOLTS), 16, 4, WHITE);
    }
}

void try_turn_on_coil() {
    switch(mode) {
    case VARY_VOLT_MODE:
    {
        uint32_t analog_write = (volts_to * 255) / (float) vcc;
        if (analog_write > 255) {
          break;
        }
        analogWrite(TURN_ON_COIL, analog_write);
    }
    break;
    case VARY_WATT_MODE: 
    {
        if (IS_NO_COIL || IS_OVER_COIL_RES || IS_SMALL_COIL_RES) {
          break;
        }
        uint32_t analog_write = (sqrt(watts_to * 100 * res) * 255) / (float) vcc;       
        if (analog_write > 255) {
          break;
        }       
        analogWrite(TURN_ON_COIL, analog_write);  
    }
    break;
    }
  
    return;
}

void turn_off_coil() {
    analogWrite(TURN_ON_COIL, 0);
    return;  
}

uint32_t read_coil_res()
{
    digitalWrite(TURN_ON_MEASURING_RES_PIN, HIGH);
    delay(2);
    int analog_read = analogRead(MEASURE_RES_PIN);
    if (analog_read <= 0) {
        return 0;
    }
    uint32_t volts = ((uint32_t)analog_read * vcc) / (float)1023;
    uint32_t res = ((volts * VOLTAGE_DEVIDER_RES) / (float)(vcc - volts));
    digitalWrite(TURN_ON_MEASURING_RES_PIN, LOW);
    return res;
}

uint32_t read_vcc()
{
    uint32_t result;
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC))
        ;
    result = ADCL;
    result |= ADCH << 8;
    result = 109600 / (float)result;
    return result;
}
