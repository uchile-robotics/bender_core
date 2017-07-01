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

By default, the P3-AT is configured to use the *all terrain* wheels. This can be modified by using ARCOS to change the robot subtype from `p3at-sh` to `p3atiw-sh`. Furthermore, it is recommended to create a new configuration file:

- Configuration files are located under `Aria/params` directory. On this installation, the full path is: `/usr/share/Aria/params/`
- Make a copy of `p3atiw-sh.p` into `/usr/share/Aria/params/bender.p`


### 3.- Tuning odometry

This is based on the Operations Manual p.54. Follow the operations manual procedure for both, Standard Calibrations and the Gyroscope calibrations.

### Finally ... record configuration

Make sure there is a copy of `/usr/share/Aria/params/bender.p` and `bender.rop` into this package!. And update that configuration files to match the new parameters.
