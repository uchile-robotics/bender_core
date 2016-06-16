Instalación
===========

Instalar *udev rules*
---------------------

Permite que los disositivos de control (USB2Dynamixel FTDI FT232RL) sea asignado a un puerto en especifico cada vez de se conecta.

~~~
$ sudo cp left_arm.rules /etc/udev/rules.d/left_arm.rules
$ sudo cp right_arm.rules /etc/udev/rules.d/right_arm.rules
~~~

Permisos para puertos
---------------------

Permite que los usuarios puedan escribir y leer un puerto serial.

~~~
$ sudo sudo chmod a+rw /dev/bender/left_arm
$ sudo sudo chmod a+rw /dev/bender/right_arm
$ sudo usermod -a -G dialout <user>
~~~


Desinstalar drivers Dynamixel del PPA ROS
-----------------------------------------

Los drivers son instalados desde el código, para desinstalar completamente usar:

~~~
$ sudo apt-get purge ros-indigo-dynamixel-driver ros-indigo-dynamixel-motor ros-indigo-dynamixel-msgs ros-indigo-dynamixel-controllers
~~~

Comandos útiles
---------------

Mostrar últimos mensajes del kernel
~~~
$ dmesg | tail
~~~

Listar dispositivos USB conectados
~~~
$ lsusb
~~~
