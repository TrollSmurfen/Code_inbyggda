from machine import Pin, I2C, PWM
import utime
from pico_i2c_lcd import I2cLcd

# LCD settings
I2C_ADDR = 0x27
I2C_NUM_ROWS = 2
I2C_NUM_COLS = 16

# Initialize I2C and LCD
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
lcd = I2cLcd(i2c, I2C_ADDR, I2C_NUM_ROWS, I2C_NUM_COLS)

# Ultrasonic sensor pins
TRIG = Pin(3, Pin.OUT)
ECHO = Pin(2, Pin.IN)

# Buzzer pin
buzzer = PWM(Pin(15))

# RGB LED pins
red = PWM(Pin(16))
green = PWM(Pin(17))
blue = PWM(Pin(18))

# Function to measure distance
def measure_distance():
    TRIG.low()
    utime.sleep_us(2)
    TRIG.high()
    utime.sleep_us(10)
    TRIG.low()

    while ECHO.value() == 0:
        signal_off = utime.ticks_us()
    while ECHO.value() == 1:
        signal_on = utime.ticks_us()

    time_passed = utime.ticks_diff(signal_on, signal_off)
    distance = (time_passed * 0.0343) / 2
    return distance

# Function to control RGB LED based on distance
def set_rgb_color(distance):
    if distance < 10:
        red.duty_u16(0)
        green.duty_u16(65534)
        blue.duty_u16(65534)
    elif 10 <= distance < 20:
        red.duty_u16(0)
        green.duty_u16(65534)
        blue.duty_u16(0)
    else:
        red.duty_u16(65534)
        green.duty_u16(0)
        blue.duty_u16(65534)

# Main loop
while True:
    distance = measure_distance()
    lcd.clear()
    lcd.putstr("Distance: {:.2f} cm".format(distance))
    set_rgb_color(distance)

    if 20 <= distance < 30:
        buzzer.freq(200)
        buzzer.duty_u16(32768)
    elif 10 <= distance < 20:
        buzzer.freq(2500)
        buzzer.duty_u16(32768)
    elif distance < 10:
        buzzer.freq(5000)
        buzzer.duty_u16(32768)
    else:
        buzzer.duty_u16(0)

    utime.sleep(0.3)
