# bender_base package

## Important Links

- [Pioneer P3-AT Beeps](http://robots.mobilerobots.com/wiki/Robot_Beeps)
- [Mobile Robots F.A.Q.](http://robots.mobilerobots.com/wiki/FAQ)
- [ARCOS configuration tool](http://robots.mobilerobots.com/wiki/ARCOS)

## Our Hardware

- Pioneer P3-AT
- Aria type: Pioneer
- Aria subtype: p3at-sh
- **Both pioneer robots in the lab include a Gyro accesory**. Important for odometry improvement.
- **Both pioneer robots are using the indoor wheels** and not the default ones for *all terrain*

## Robot Configuration

The pioneer 3AT holds a lot of configuration parameters on the onboard microcontroller. This parameters can be modified with the ARCOS tool or through the Aria library.

The ARCOS tool stores the modifications on the microcontroller EEPROM, so they are always available. This is the recommended configuration method by the Aria guys; they recommend to use a SERIAL to SERIAL connection between the machine and the pioneer, because any attempt to use a USB to serial converter can misconfigure the onboard memory. However, the USB to SERIAL mode was tested and works!.

On the other hand, ROSARIA uses the Aria library to modify parameters on the fly. Only a small subset of parameters are provided.

### Default Configuration Values

This values were taken from the operations manual.

- Aria default parameters: /usr/share/Aria/params/p3at-sh.p
- Weight: 14 [Kg] (with(out) batteries??)
- Payload: 40 [Kg]
- Max. linear velocity: 7 [m/s]
- Max. angular velocity: 140 [deg/s] = 2.4 [rad/s]

See also: Operations Manual p.65

### Demo program

- The original source is found under `/usr/share/doc/libaria-dev/examples`. A little dificult to use, so was added to the *rosaria* fork under the name `AriaDemo`
- The demo assumes a SERIAL to SERIAL connection!. You can set the port when using a USB to SERIAL converter. Run it by calling:

```bash
rosrun rosaria AriaDemo -rp /dev/bender/base
```

### ARCOS

Download from the mobilerobots wiki (see links above).

- Does work on a fixed pc with a SERIAL to SERIAL connection.
- It also WORKS on a laptop using the same USB to serial adapter used to control the robot through rosaria. However, you must give it the port name:

```bash
# This assumes you have installed the pioneer udev rule. 
# Otherwise, use something like /dev/ttyUSB0
cd <arcos_folder>
./ARCOScf -rp /dev/bender/base
```

## Hardware Maintenance

See the Operations Manual p.54-p.56 sections about drive lubrication and tightening the AT Drive Belt.

Also, make sure the batteries are in good state. 

## Calibration Procedure

Before proceeding, make sure you have followed the hardware maintenance instructions.

### 1.- Update firmware and backup parameters.

- Download the latest ARCOS tool.
- Run it and save the current FLASH values. MAKE SURE not to overwrite them!! as the confuguration could fail!.
- Upload the latest firmware image `*.mot`. 

### 2.- Update the "hard" parameters

Aria comes with a set of preconfigured files to be loaded on the robot upon startup. The file is selected to match the robot `SubType` parameter, which is set to `p3at-sh` by default.

For the following configuration you will need to create a custom configuration file. This step is done on the *indoor wheels* section. This requires sudo privileges.


#### Indoor Wheels

By default, the P3-AT is configured to use the *all terrain* wheels. This can be modified by using ARCOS to change the robot subtype from `p3at-sh` to `p3atiw-sh`. Relevant diff is:
 
```
  Subclass       p3at      --> p3atiw-sh   ; specific type of robot
  RobotWidth     505.00000 --> 490.00000   ; width in mm
  DistConvFactor 0.46500   --> 1.00000     ; multiplier to mm from robot units
```

Configuration files are found on the `Aria/params` directory. On this installation, the full path is: `/usr/share/Aria/params/`.

Modify the FLASH parameters with ARCOS to set the `SubType` to `p3atiw-sh`. Then attempt a connection with rosaria to verify that the loaded Aria/param file is `p3atiw-sh.p`.


#### Robot Geometry

TODO.

### Odometry calibration

This is based on the Operations Manual p.54. Follow the operations manual procedure for both, Standard Calibrations and the Gyroscope calibrations.

See the Op.Manual p.31 for a table with all ARCOS command numbers. They are very handy while tuning the base using the demo program.


#### Configure Max Velocity and Acceleration

First set the desired maximum accel and velocity. There are some *Top* maxes which cannot be modified:

```
TransVelTop 1500 TransAccelTop 2000
RotVelTop 360 RotAccelTop 300
```


You can modify the following values. Note this are the default firmware configurations. Please reduce this values!:

```
item         |default| recommended | ARCOS cmd # | range      |  units
------------------------------------------------------------------------------------------
transVelMax  |  750  |     600     |       6     | [0, 65535] |  (mm/sec) 
RotVelMax    |  100  |      40     |      10     | [0, 65535] |  (deg/sec)
TransAcc     |  300  | default     |       5     | [0,  4000] |  (mm/sec/sec) 
TransDecel   |  300  | default     |       5     | [0,  4000] |  (mm/sec/sec) 
RotAcc       |  100  | default     |      23     | [0, 65535] |  (deg/sec/sec) 
RotDecel     |  100  | default     |      23     | [0, 65535] |  (deg/sec/sec) 
```


#### Configure DriftFactor, TickMM and RevCount

Follow the op.Manual procedure on page 54. Remember to disable the Gyroscope!!. The gyro can be disabled for the current session using the **p** mode on the demo program. Also, the gyro can be disabled by using the ARCOS command #58 (argument 0 to disable, 2 to enable)

```
item         | default | recommended | ARCOS cmd # | range
------------------------------------------------------------------------------------------
DriftFactor  |     0   |       0     |      89     | [0, 200]
Ticks/mm     |   170   |     164     |      93     | [0, 200]
RevCount     | 32550   |   29500     |      88     | [-32768, 32767]
```

When finished, remember to enable the Gyro!.

#### Configure the PIDs

The Operations Manual gives some descriptions on the PID effects and explains how to tune them. See pages 34 and 52. Consider that Bender has a payload of ~54Kg.

```
item      | def. | recom. | ARCOS cmd# | range
------------------------------------------------------------------------------------------
RotKp     |  40  |  50    |      82    | [0, 65535]
RotKv     |  20  |  20    |      83    | [0, 65535]
RotKi     |   0  |  10    |      84    | [0, 65535]
TransKp   |  40  |  45    |      85    | [0, 65535]
TransKv   |  30  |  25    |      86    | [0, 65535]
TransKi   |   0  |   2    |      87    | [0, 65535]
```


#### Configure the onboard gyroscope

See the Operations Manual p.55. Calibrate the `GyroCW` (Cmd #38) and `GyroCCW` (Cmd #39) values as you did for `revCount`.

```
item      | def. | recom.  | ARCOS cmd# | range
------------------------------------------------------------------------------------------
GyroCW    | 940  | default |      38    | [1, 2000]
GyroCCW   | 950  | default |      39    | [1, 2000]
```

### Finally ... record configuration

Use ARCOS to load the new configuration into the robot FLASH memory:

```bash
# connect using ARCOS
./ARCOScf -rp /dev/ttyUSB0

# load updated configuration into ARCOS memory
> r bender.rop

# save to the FLASH
> save

# exit
Ctrl+C
```

Finally, make sure there is a updated version of `bender.rop` into this package!.

