Librería Dynamixel
==================

Escrita en Python y multiplataforma. Posee una versión *standalone* lib_robotis.py.

Scripts de utilidad se encuentran en el paquete *dynamixel_driver* (/opt/ros/indigo/dynamixel_driver)


Comandos de utilidad
--------------------
Ver información de un servomotor. Ejemplo: información del servomotor con ID:31 conectado a USB2Dynamixel en puerto /dev/ttyUSB0 con una tasa de 500000 baud/s.
~~~
$ rosrun dynamixel_driver info_dump.py --port=/dev/ttyUSB0 --baud=500000 31
~~~

Configuracion de parametros
~~~
$ rosrun dynamixel_driver set_servo_config.py --help
~~~

Habilitar y deshabilitar torque
~~~
$ rosrun dynamixel_driver set_torque.py --help
~~~

Cambiar ID de servomotor
~~~
$ rosrun dynamixel_driver change_id.py --help
~~~
