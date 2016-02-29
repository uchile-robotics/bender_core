## FILES
## ----------------------------------------------------------

## file: bender.inc
# 
# . contiene definición del "robot bender"
# . diseñado para ser utilizado en archivo .world
# . robot es conformado por:
#     . base (poligono)
#     . lasers (simulado con 1 laser de gran FOV)
#
# 
## file: hokuyos.inc
# . contiene defs de lasers hokuyos, según especificaciones de
#   sus datasheets.
# . usado por bender.inc
#
#
## file: useful_models.inc
# . contiene modelos útiles representando  personas y objetos
# . tambien contiene una definición base "abstracta" de mapa,
#   el que debe ser refinado para su uso.
# . usado por bender.inc
#


## Usage
## ----------------------------------------------------------


Cada escenario a añadir debe ser puesto en una carpeta separada "foldername".
El directorio "foldername" identifica al mapa en cuestión, el que puede
ser referenciado desde bender.launch mediante tal nombre.

Cada descripción de escenario está compuesta por 4 archivos:
- map.inc     : 
- map.pgm     :
- map.yaml    :
- world.world :

Ej:
bender_stage/world/
  my_map/



## Configuration
## ----------------------------------------------------------

primero:
map.pgm: sobre recorte del mapa y edición con kolourpaint.

luego:
sobre tamaños... en map.inc y  map.yaml

finalmente: world.world
sobre propiedades del escenario y ventana
carga del robot y pose inicial
carga de objetos utilitarios.


# Useful Information
# ---------------------------------------------------

Tomando en cuenta que el origen de una imagen:
- para map_server está en la esquina superior izquierda y va así:

       
       y
       ^
       |
       |
       -----> x


- para stage va en el centro y en las mismas direcciones.


Si la imagen es de (w,h) pixeles,
con una resolución r [m/pixel]
y se desea un origen en ( _x_ , _y_ ) (obtenido desde map.yaml usado para el map server)

se debe usar un  floorplan (en el world file de stage) tq:

size: [ r*w r*h 0.0 ]
pose: [ r*w/2-_x_ r*h/2-_y_ 0]
