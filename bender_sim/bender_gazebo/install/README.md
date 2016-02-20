bender_gazebo
=============

El package `bender_gazebo` contiene archivos necesarios para la simulación del robot Bender en Gazebo.

![Bender en Gazebo](img/bender_gazebo.png)

Requisitos
----------

* Ubuntu 14.04 (Trusty)
* ROS Indigo
* Gazebo 2.2.5

Instalación
-----------

### Instalar dependencias

~~~
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
~~~

### Instalar modelos propios

Copiar carpetas `bender_gazebo/gazebo_models` a `~/.gazebo/models`

~~~
$ roscd bender_gazebo/gazebo_models/
$ cp -a . ~/.gazebo/models/
~~~

### Error "No namespace found" en Gazebo

Usualmente sucede al no tener acceso a la descarga de modelos desde la [base de datos online](http://models.gazebosim.org/) o no existe la carpeta `~\.gazebo\models`.

### Añadir OSRF PPA (Ubuntu 14.04 Trusty, ROS Indigo)

Permite obtener actualizaciones más reguralares de Gazebo.

~~~
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install --only-upgrade gazebo2
~~~
