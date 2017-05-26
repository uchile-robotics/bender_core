# bender_joy

## Arquitectura del sistema de teleoperación

Cada adaptador Wireless XBOX soporta hasta 4 joysticks, cada uno asignado a los dispositivos en `/dev/input/js{'0..3'}`. Al encender el joystick, éste será asignado aleatóreamente a uno de esos dispositivos, y no hay manera fácil de saber cual de ellos es el correcto.

Para evitar problemas al intentar leer un dispositivo `js` inactivo, es que se propone la siguiente arquitectura:

``` 
Adaptador Wireless Xbox 360
--> /dev/input/js0 --> node:driver0 --> topic:joy0 --> node:bender_joy/proxy0
--> /dev/input/js1 --> node:driver1 --> topic:joy1 --> node:bender_joy/proxy1
--> /dev/input/js2 --> node:driver2 --> topic:joy2 --> node:bender_joy/proxy2
--> /dev/input/js3 --> node:driver3 --> topic:joy3 --> node:bender_joy/proxy3
``` 

- Cada dispositivo es asociado a un driver (joy/joy_node), el que publicará en el respectivo tópico `joy{0..3}`, siempre que haya un control activo.
- A cada driver se le asocia un nodo (bender_joy/joy_proxy.py), que intercepta la señal y funciona a modo de manager, para routear los mensajes al tópico que se desee.

Actualmente se tienen los siguientes remaps desde el proxy:
``` 
node:proxy{0..3}:
--> topic: topic_A --> remap:topic: joy/base  --> node:bender_joy/joy_base
--> topic: topic_B --> remap:topic: joy/head  --> node:bender_joy/joy_head
--> topic: topic_X --> remap:topic: joy/tts   --> node:bender_joy/joy_tts
--> topic: topic_Y --> remap:topic: joy/arms  --> node:bender_joy/joy_arms
``` 

Luego, cada uno de los nodos finales, procesa un mensaje de tipo `sensor_msgs/Joy`, proveniente de hasta 4 drivers a la vez.


Lo anterior tiene las siguientes ventajas:
- Permite que con sólo un joystick se puedan manipular diversos componentes, pues mediante una combinación de botones, el proxy permite seleccionar el modo de operación, lo que hace forwarding del mensaje sólo al tópico que se requiera.
- Permite utilizar más de un joystick para manejar un mismo componente (equivalente a remapear 2 joysticks al mismo tópico), pero de manera dinámica.

Pero tiene la desventaja de cuadruplicar los nodos involucrados en el manejo del joystick.


## Uso

Sólo puede haber 1 Adaptador Wireless de joystick por PC. Cada adaptador soporta hasta 4 joysticks.

```sh
# Por cada pc con un adaptador
roslaunch bender_joy driver.launch --screen

# Nodos de control. Lanzar sólo 1 vez.
roslaunch bender_joy joy_interface.launch --screen
```

Si es que sólo se utilizará un computador, es preferible optar por lo siguiente:
```sh
# Por cada pc con un adaptador
roslaunch bender_joy joysticks.launch --screen

```


## TODO LIST

- joy interaction mouth off
- portar el joystick del brazo
- rehabilitar el plugin del joy_tts


