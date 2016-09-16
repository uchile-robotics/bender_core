# bender_joy

## Arquitectura del sistema de teleoperación




## Uso

Sólo puede haber 1 Adaptador Wireless de joystick por PC. Cada adaptador soporta hasta 4 joysticks.

```sh
# Por cada pc con un adaptador
roslaunch bender_joy driver.launch

# Nodos de control. Lanzar sólo 1 vez.
roslaunch bender_joy joy_interface.launch
```

## TODO

- elimino el joy number

