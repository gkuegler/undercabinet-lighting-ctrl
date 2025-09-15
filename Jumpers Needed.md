# enable_led_pwm
When this is disabled, no dimming will occur.
The acceptance chirp is disabled.
The set thresh indicator is slower? or is just off then comes on at the end?

# enable_led_shutoff_timeout
When disabled, the LED will stay on.
When enabled, the LED will warn, then turn off after (n) hours and lock out the mcu.
This feature is primarily to protect the LED from being on for too long.
Will require a manual button press to resume normal ranging led control.
For repeat timeout violations the mcu shall shutdown?

# enable_grocery_detection
When enabled, the controller shall switch to manual mode if an object is held within the threshold distance for more than (n) seconds.
Alternatively, the # of samples needed and debounce time shall increase to add delay to the hand in/out reset.
Some future defined button sequence shall reset to normal.
The controller shall resume normal control when the object is removed after a set delay.
