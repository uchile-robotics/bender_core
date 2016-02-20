/*
 * laser_merger.h
 *
 *  Created on: 16-06-2013
 *      Author: matias pavez b.
 */

#ifndef SD_H_
#define SD_H_

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <vector>
#include <math.h>


typedef std::vector<float> array_;

//************ Funciones ***********//
void callback(const sensor_msgs::LaserScanPtr msg);

/* Hace el cambio de coordenadas */
void transform(array_ *d, float a_min, float a_delta, array_ *d_aux, array_ *a_aux, float *x_);

/* merge de los lasers ya transformados */
void merge(const array_ *A, const array_ *angles_A, const array_ *B, const array_ *angles_B);
array_ juntar(const array_ *A, const array_ *B);	/* Junta dos arrays en uno (los pega uno al lado del otro)*/

/* Quicksort*/
int particion(array_ *A, array_ *B, int izquierda, int derecha);
void quicksort(array_ *A, array_ *B, int first, int last );
void swap( float * const ptr1, float * const ptr2 );

/* Funciones auxiliares */
void setup_scan_common();
void setup_scan_ambos_mode();
void setup_scan_front_mode();
void setup_scan_rear_mode();

int redondear(float);

#endif /* SD_H_ */
