bender_gazebo
=============

El package `bender_gazebo` contiene archivos necesarios para la simulación del robot Bender en Gazebo.

![Bender en Gazebo](img/bender_gazebo.png)

Requisitos
----------

* Ubuntu 14.04 (Trusty)
* ROS Indigo

Instalación
-----------

### Instalar dependencias

~~~
$ cd catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=indigo -y
~~~

### Añadir OSRF PPA (Ubuntu 14.04 Trusty, ROS Indigo) e instalar Gazebo 7

En ROS Indigo usa por defecto Gazebo 2, para obtener nuevas funcionalidades es recomendable usar Gazebo 7, para instalarlo desde `apt-get` es necesario instalar el PPA.

~~~
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
~~~

Para desinstalar por completo Gazebo 2 usar:

~~~
$ sudo apt-get purge gazebo2 libsdformat-dev libsdformat1 ros-indigo-gazebo-msgs ros-indigo-gazebo-plugins ros-indigo-gazebo-ros ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs
~~~

Finalmente, para instalar Gazebo 7 ejecutar en el terminal:

~~~
$ sudo apt-get install gazebo7 gazebo7-common gazebo7-plugin-base libgazebo7 libgazebo7-dev libsdformat4 libsdformat4-dev ros-indigo-gazebo7-msgs ros-indigo-gazebo7-plugins ros-indigo-gazebo7-ros ros-indigo-gazebo7-ros-control ros-indigo-gazebo7-ros-pkgs sdformat-sdf 
~~~

### Instalar modelos propios

Se deben instalar modelos básicos en `~/.gazebo/models`.

~~~
$ bash bender_sim/bender_gazebo/install/install.sh 
~~~

### Error "No namespace found" en Gazebo

Usualmente sucede al no tener acceso a la descarga de modelos desde la [base de datos online](http://models.gazebosim.org/) o no existe la carpeta `~\.gazebo\models`. Una vez se han descargado los modelos básicos, el programa iniciará de forma normal.

