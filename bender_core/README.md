# bender_core

## Overview

Este package es la interfaz ROS del workspace `base_ws`. Su misión es proveer una interfaz mediente el concepto de *robot_skill* para todos los packages del workspace.

Además, contiene los archivos `.launch` para lanzar nodos del workspace.

Este package depende de todos los packages del workspace, por lo que puede ser utilizado en los `package.xml` de alto nivel, para obtener indirectamente todas las dependencias de bajo nivel.


## Arquitectura

## API

### Clase `robot`

Documentado en el mismo archivo: `src/bender_skills/robot.py`

### Clase `robot_skill`

Documentado en el mismo archivo: `src/bender_skills/robot_skill.py`


## Creando nuevos skills

Cada skill debe heredar de la clase `RobotSkill` e implementar al menos los métodos `start()`, `check()`, `setup()`, `pause()` y `shutdown()`. Es decisión del implementador la funcionalidad que tendrán, por ejemplo, no tiene sentido hacer shutdown o pausa del joystick, pues es imperativo que esté funcionando en todo momento.

Además de los métodos obligatorios, se espera que cada implementador agrege más funcionalidades a la clase. Por ejemplo, se espera que la HeadSkill provea una función cómo `head.set_emotion("happy")`.

En la carpeta `src/bender_core/core/` se presentan implementaciones de cada componente del core. Se recomienda utilizarlas como base para programar nuevas funcionalidades.


## Ejemplos

La carpeta `bender_core/samples` contiene nodos de ROS con ejemplos de diversas versiones de un robot, construido a partir de los skills de ejemplo. Pueden ser ejecutados con los siguientes comandos: 

```bash
rosrun bender_core corebot.py
```
