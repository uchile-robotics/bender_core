#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division
import yaml
import sys
import argparse
from math import pi

__author__ = 'Rodrigo Munoz'

''' Generar archivos de limites para URDF y Position Controllers  '''

def trunc(f, n):
    '''Truncar float f a n decimales'''
    slen = len('{num:.{decimal}f}'.format(num=f,decimal=n))
    return float(str(f)[:slen])

def main(input_file='info_motor.yaml', yaml_out='controller_config.yaml',
      xacro_out='urdf_limits.xacro', default_joint_speed = 0.9, soft_offset = 5, decimal = 5):
  
    print('Cargando datos desde {}...'.format(input_file))
    # Cargar archivo
    try:
        with open(input_file, 'r') as file:
            info_motor = yaml.load(file)
    except EnvironmentError:
        print('Error al leer el archivo.')
        sys.exit()
    # Diccionarios principales
    controller_config = dict()
    urdf_config = dict()
    # Parametros de motores
    motor_param = \
    {
        'mx106':  { 
            'range_degrees'     :  360,
            'encoder_resolution':  4096
        },
        'rx64':   { 
            'range_degrees'     :  300,
            'encoder_resolution':  1024
        },
        'rx28':   {
            'range_degrees'     :  300,
            'encoder_resolution':  1024
        },
        'ex106':  { 
            'range_degrees'     :  250.92,
            'encoder_resolution':  4096
        },
        'default':{ 
            'range_degrees'     :  300,
            'encoder_resolution':  1024
        }
    }
    # Procesar datos
    print('Procesando datos...')
    for name, data in info_motor.iteritems():
        # Seccion
        controller_name = name.replace('joint','controller')
        controller_config[controller_name] = dict()
        # Position controller
        position_controller = {'package':'dynamixel_controllers',
            'module':'joint_position_controller', 'type':'JointPositionController'}
        controller_config[controller_name].update({'controller':position_controller})
        # Joint name
        urdf_config[name] = dict()
        controller_config[controller_name].update({'joint_name':name})
        # Joint speed
        controller_config[controller_name].update({'joint_speed':default_joint_speed})
        # Motor info
        if not motor_param.has_key(data['model']):
            print('No hay informacion sobre modelo {}, usando parametros por defecto.'.format(data['model']))
        motor_values = motor_param.get(data['model'],motor_param['default'])
        # Radianes por tick del motor
        rad_per_tick = motor_values['range_degrees']/motor_values['encoder_resolution']*pi/180
        upper_limit, lower_limit = 0, 0
        motor = dict()
        motor.update({'id':data['id']})
        motor.update({'init':data['init']})
        # Valores considerando sentido de giro
        if data['flipped']:
            motor.update({'min':data['max']-soft_offset})
            motor.update({'max':data['min']+soft_offset})
            # Limites URDF
            upper_limit = (motor['init']-motor['max'])*rad_per_tick
            lower_limit = (motor['init']-motor['min'])*rad_per_tick
        else:
            motor.update({'min':data['min']+soft_offset})
            motor.update({'max':data['max']-soft_offset})
            # Limites URDF
            upper_limit = (motor['max']-motor['init'])*rad_per_tick
            lower_limit = (motor['min']-motor['init'])*rad_per_tick
        urdf_config[name].update({'upper':trunc(upper_limit, decimal), 'lower':trunc(lower_limit, decimal)})
        controller_config[controller_name].update({'motor':motor})

    # Guardar yaml controller
    print('Generando {}...'.format(yaml_out))
    controller_config_file = open(yaml_out, 'w')
    controller_config_file.write('# Generado con script desde configuracion de motores Dynamixel\n')
    controller_config_file.write(yaml.dump(controller_config,default_flow_style = False))
    controller_config_file.close()
    # Guardar urdf
    print('Generando {}...'.format(xacro_out))
    with open(xacro_out, 'w') as xacro_config:
        xacro_config.write('<?xml version="1.0"?>\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="bender_limits">\n')
        xacro_config.write('\t<!-- Generado con script desde configuracion de motores Dynamixel -->\n')
        for joint_name, data in urdf_config.iteritems():
            text = '\t<xacro:property name="{0}_upper" value="{1:.{2}f}" />\n'.format(joint_name, data['upper'], decimal)
            text += '\t<xacro:property name="{0}_lower" value="{1:.{2}f}" />\n\n'.format(joint_name, data['lower'], decimal)
            xacro_config.write(text)
        xacro_config.write('</robot>')

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('input_config', help='Archivo de configuracion de motores')
    parser.add_argument('controller_config', help='Archivo de configuracion de controladores')
    parser.add_argument('xacro_config', help='Archivo de configuracion de limites xacro')
    parser.add_argument('-d', '--decimal', help='Numero de decimales', type=int, default=5)
    parser.add_argument('-o', '--offset', help='Offset en ticks entre limite de controlador y motor', type=int, default=5)
    parser.add_argument('-s', '--speed', help='Velocidad maxima por defecto', type=float, default=0.8)
    args = parser.parse_args()
    main(input_file=args.input_config, yaml_out=args.controller_config,
        xacro_out=args.xacro_config, default_joint_speed=args.speed,
        soft_offset=args.offset, decimal=args.decimal)