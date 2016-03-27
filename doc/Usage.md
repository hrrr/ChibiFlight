
# Flashing

Flash using the debug port:
```
st-flash write build/ChibiFlight_Sparky2.bin 0x08000000
```

# Failsave

A basic failsve handling is implemented. The copter will disarm automatically if no data is received from transmitter after half a second. Please check this!

# Setup

Use a spektrum satellite bound to your transmitter, connected to the receiver port in Sparky2.
Connect the motors in the same order as in Cleanflight, that is:
```
4 2
3 1
```

Edit config.h and change
```
#define DEBUG_MODE       FALSE
```

to
```
#define DEBUG_MODE       TRUE
```


Compile and flash the board.

Connect the board throught the micro USB cable with the computer. The computer shoudl detect the board as a new COM port. Open this serial port in a console (the arduino serial monitor will work fine! Just select the right serial port and open the monitor.)

Do NOT connect the LiPo battery yet!!
Power on your transmitter.

You should see something like:

```
Ch1:  00C7 Ch2:  0400 Ch3:  0401 Ch4:  0400 Ch5:  06AA Ch6:  06AA Ch7:  0400 
Ch8:  0400 Ch9:  0400 Ch10: 0400 Ch11: FFFF Ch12: FFFF Ch13: FFFF Ch14: FFFF 
TARGET Throttle: 125 Roll: 0 Pitch: 0 Yaw: 0 Aux1: 888 Aux2: 888
GYRO Roll: 0 Pitch: 0 Yaw: 0
motor FL : 125 FR: 125
motor RL : 125 RR : 125
```

Ch1 to Ch14: Raw inputs from your transmitter. Actually only 6 channels supported.
Target: Target set from your current transmitter sticks.
Gyro: Current gyro read.
motor: current motor output (125-250 us one-shot range)

Channel inputs: Neutral for every channel is 0x400. Minimun is 0x100, maximun 0x700
Trim your transmitter so that:

Ch1: This must be the throttel channel. At minnimun the ch1 value must be lower than 0x100 (you wont be able to arm disarm if this does not go below 0x100!), at maximun it should be slightly higher than 0x700 (you wont reach full throttel if is does not reach at least 0x700)

Ch2: This must be your roll channel. Adjust your transmitter trim so that it is 0x400 in neutral. Rolling to the left must decrease this number, to the right increase it. Adjust your travel so that rollin at max to the left shows a nuber slightly below 0x100, and rolling at max to the right slightly over 0x700.

Ch3: This must be your pitch channel. As with the roll channel, neutral should be at 0x400. Pitching forward must decrease the number down to slighly below 0x100, and backwards up to slighly over 0x700

Ch4: This must be your yaw channel. Neutral at 0x400, yaw to the left at decreases the number down to slughly below 0x100 and to right increases it to slightly over 0x700

Ch5: Aux1 channel. Use a three possition switch. In the midlle possition it should be at 0x400. Check in the Target line that Aux1 changes between somthing lower than -100, something arround 0 and somehting higher than 100. This will be used to select one of three PID set of values.

Ch6: Aux2 channel. Use a 2 posstion switch.Check that Aux2 in the Target line changes between somthin lower than 0 and something higher than 0. This will be used to trigger the buzzer.

The other channels are not used in the current firmware.

Target: Your inputs from the transmitter are mapped to a throttel position and rotation rates in degrees/second.
the maximal rotation rates are defined in config.h:
```
// Rates in degrees/second
#define ROLL_RATE             500
#define PITCH_RATE            500
#define YAW_RATE              400
```

Move your transmitter sticks arround and check the throttel range goes from 125 to 250 and the rotation rates according to the defines in config.h. Mapping from transmitter sticks to rotation rates is linear, add expo in your transmitter if desired.

Gyro: Gyro reading in degress/second. Move your copter an check the readings change according to the movements. Rolling or yawing to the left should have negative sign, to the right postive. Roling formward negative, backwards possitive.

Arming disarming works as in cleanflight: Throttle at min, and yaw to the right to arm, throttle at min and yaw to the left to disarm. Check this works.
Motor outputs in the serial console will change according to the sticks once the copter is armed. The blue LED will light when armed. 

Once this is everything OK, you should change DEBUG_MODE back to FALSE, recompile and reflash.

# Usage

ALLWAYS calibrate your gyro before using. Roll backwards, throttle minimun and yaw max to the left will start gyro calibration. The blue LED will light during calibration (one second). Once calibrated you might arm and fly. The set of values to be used will be set while arming. Once armed changing the three possition switch will not change the PIDs

PIDs: In config.h there are 3 sets of PIDs. You can change wich PID set will be used with your Aux1 channel.
The _0 set will be used if Aux1 is smaller than -100. _2 if bigger than 100 and the _1 set if between -100 and 100 (tipically in the middle possition of the three possition switch)


# Blackbox

The SPI flash chip is used to recod flight data (Trasmitter values, PID calculations and output to the motors).
Check the code for details... I will write more detials here soon...

