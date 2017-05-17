Configuración USB2Dynamixel 
===========================

Comandos de utilidad
--------------------
Ver si el usuario pertenece al grupo *dialout* (0 si no pertenece)
~~~
$ groups $USER | grep -o -w -c dialout
~~~
Añadir al usuario al grupo *dialout* para habilitar escritura en puertos
~~~
$ sudo usermod -a -G dialout $USER
~~~
Lista de elementos en puertos USB
~~~
$ lsusb
...
Bus 001 Device 004: ID 0403:0000 Future Technology Devices International, Ltd H4SMK 7 Port Hub
...
~~~
El chip usado para la interfaz corresponde al FTDI FT232RL, que también es usado por placas Arduino.
El hardware conectado (mensajes del *kernel*) puede verse usando el comando:
~~~
$ dmesg | tail
...
[45838.847078] usbserial: USB Serial support registered for generic
[45838.851008] usbcore: registered new interface driver ftdi_sio
[45838.851026] usbserial: USB Serial support registered for FTDI USB Serial Device
[45838.851094] ftdi_sio 1-1.4:1.0: FTDI USB Serial Device converter detected
[45838.851143] usb 1-1.4: Detected FT232RL
[45838.851148] usb 1-1.4: Number of endpoints 2
[45838.851152] usb 1-1.4: Endpoint 1 MaxPacketSize 64
[45838.851155] usb 1-1.4: Endpoint 2 MaxPacketSize 64
[45838.851158] usb 1-1.4: Setting MaxPacketSize 64
[45838.851803] usb 1-1.4: FTDI USB Serial Device converter now attached to ttyUSB0
...
~~~
Además pueden listarse los dispositivos USB usando:
~~~
$  ls /sys/bus/usb-serial/devices/ -ltrah
~~~
Al final se muestra el punto de montaje, usualmente, /dev/ttyUSB0
