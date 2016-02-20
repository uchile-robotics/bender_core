#include "bender_sensors/laser_merger.h"
#include "bender_srvs/getLaserScan.h"

/* Algún Día:
 * TODO: Utilizar tf's de lasers para datos tales como x_front, x_rear, bender_radio, etc...
 */

// parameters
#define num_readings 1250
#define bender_radio 0.30	// used to crop sensor readings
float xfront = 0.245;
float xrear = -0.210;
std::string frame_front   = "/bender/sensors/laser_front_link";
std::string frame_rear    = "/bender/sensors/laser_rear_link";
std::string frame_virtual = "/bender/sensors/laser_virtual_link";

//************ Variables ***********//

// ROS
ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceServer server;

// auxiliary
array_ d_rear;				// l_rear  ranges
array_ d_front;				// l_front ranges
array_ a_rear;				// l_rear  headings
array_ a_front;				// l_front headings
array_ empty_array (1);		// empty array

enum working_mode_t { MODE_BOTH = 0, MODE_FRONT = 1, MODE_REAR = 2};

// Variables de control
char ready = 0;				// 0: None, 1: One, 2: Both
char last_last_frame = 0;	// 1: front, 2: rear
char last_frame = 0;		// 1: front, 2: rear
char actual_frame = 0;	    // 1: front, 2: rear
working_mode_t mode = MODE_BOTH;

// to publish
sensor_msgs::LaserScanPtr scan_out(new sensor_msgs::LaserScan());
bool initialized = false;

sensor_msgs::LaserScan last_front;
sensor_msgs::LaserScan last_rear;
bool valid_front = false;
bool valid_rear  = false;
ros::Duration max_lifetime(0.5); // 0.5 [s]

void callback(const sensor_msgs::LaserScan &msg){

	ROS_INFO_STREAM("callback: " + msg.header.frame_id);

	/* Get current frame */
	if(msg.header.frame_id  == frame_front){
		last_front = msg;
	}
	else if(msg.header.frame_id  == frame_rear){
		last_rear = msg;
	}

	ros::Time now = ros::Time::now();
	if (now - last_front > max_lifetime) {
		valid_front = false;
	}
	if (now - last_rear > max_lifetime) {
		valid_rear = false;
	}

	sensor_msgs::LaserScan out_scan;
	if (valid_front == false && valid_rear == false) {
		return;
	} else if (valid_front == true && valid_rear == true) {
		mode = MODE_BOTH;
		scan_out->angle_min = -M_PI;
		scan_out->angle_max = M_PI;
		scan_out->header.frame_id = frame_virtual;
		scan_out->angle_increment  = 2.0*M_PI/num_readings;
		scan_out->scan_time = 0.1;			// 10 [Hz]
		scan_out->time_increment = 0.0;		// ver forma de calcularlo (de)
		scan_out->range_min = 0.20;			// 20 [cm]
		scan_out->range_max = 8.00;			// 8.0 [m]

	} else if (valid_front == true && valid_rear == false) {
		mode = MODE_FRONT;
	} else if (valid_front == false && valid_rear == true) {
		mode = MODE_REAR;
	}

	// ojo con range max --> dejarlo como el del menor


}


void callback3(const sensor_msgs::LaserScanPtr msg){

	ROS_INFO_STREAM("callback: " + msg->header.frame_id);

	/* Get current frame */
	if(msg->header.frame_id  == frame_front){ actual_frame = 1;}
	else if(msg->header.frame_id  == frame_rear){ actual_frame = 2;}

	// Check working mode
	/* t-2 | t-1 | t
		 a    b    a   => both
		 b    a    b   => both
		 a    a    a   => front
		 b    b    b   => rear
		 ~    ~    ~   => Can't decide
	*/
	if( actual_frame != last_frame && actual_frame == last_last_frame){	// => mode = both

		// update
		if(mode != MODE_BOTH){
			setup_scan_ambos_mode();
			mode = MODE_BOTH;
		}

	} else if(actual_frame == last_frame && actual_frame == last_last_frame){

		if(actual_frame  == 1){                                         // => mode = front

			// update
			if(mode != MODE_FRONT){
				setup_scan_front_mode();
				mode = MODE_FRONT;
			}
		}	else if(actual_frame  == 2){                                // => mode = rear
	
			// update
			if(mode != MODE_REAR){
				setup_scan_rear_mode();
				mode = MODE_REAR;
			}
		}
	} else {
		// continue
	}
	last_last_frame = last_frame;
	last_frame = actual_frame;

	ROS_INFO_STREAM("transform: " + msg->header.frame_id);
	// transform
	if(actual_frame == 1){	// if "front_frame"

		if(mode == MODE_BOTH || mode == MODE_FRONT){	// if mode == both o front
			transform(&(msg->ranges), msg->angle_min, msg->angle_increment, &d_front, &a_front, &xfront);  // for normal   position  (roll = 0  , yaw = 0)
			//transform(&(msg->ranges), msg->angle_max, -msg->angle_increment, &d_front, &a_front, &xfront);   // for inverted position  (roll = 180, yaw = 0)
		} else {
			// if mode = "rear_mode", then continue
		}

	} else if(actual_frame == 2){ // if "rear_frame"

		if(mode == MODE_BOTH){	// if mode == both

			transform(&(msg->ranges), msg->angle_min + M_PI,  msg->angle_increment, &d_rear, &a_rear, &xrear); // for normal position   (roll = 0  , yaw = 180)
			//transform(&(msg->ranges), msg->angle_max + M_PI, -msg->angle_increment, &d_rear, &a_rear, &xrear);   // for inverted position (roll = 180, yaw = 180)
		} else{
			// if mode = "front_mode" or "rear_mode", then continue
		}
	}
	
	ROS_INFO_STREAM("publish: " + msg->header.frame_id);
	// publish
	if(mode == MODE_BOTH){	// if mode = both

		ready++;

		// wait for two scans
		if(ready==2){
			ROS_INFO_STREAM("publish both: " + msg->header.frame_id);

			merge(&d_rear, &a_rear, &d_front, &a_front);
			scan_out->header.stamp = ros::Time::now();
			pub.publish(scan_out);

			ready = 0;
		}

	} else if(mode == MODE_FRONT){	// if mode == front
		ROS_INFO_STREAM("publish front: " + msg->header.frame_id);

		// publish now!, merge is made with an empty scan
		merge(&empty_array, &empty_array, &d_front, &a_front);
		scan_out->header.stamp = ros::Time::now();
		pub.publish(scan_out);

	} else if(mode == MODE_REAR){	// if mode == rear
		ROS_INFO_STREAM("publish none: " + msg->header.frame_id);
		/* No publicar */
	}
}

/* merge de los lasers ya transformados */
void merge(const array_ *A, const array_ *angles_A, const array_ *B, const array_ *angles_B){

	float a_min = scan_out->angle_min;
	float a_max = scan_out->angle_max;
	float a_delta = scan_out->angle_increment;

	/* Juntar vectores en uno solo */
	array_ C  = juntar(A,B); 					// Vector de distancias
	array_ a_C  = juntar(angles_A, angles_B);	// Vector de ángulos
	int s_aC = a_C.size();
	int s_C = C.size();

	/* Ordenar los dos vectores, según ángulos (i.e, según &a_C) */
	quicksort(&C, &a_C, 0, s_aC -1);

	/* Estimación */
	float t=a_min;	// ángulo de barrido
	for(int index = 0; index < s_aC; index++){

		while( t - a_delta/2 < a_C[index]){
			/* Si estamos bajo el ángulo a
			estimar, avanzar en el barrido */
			t+=a_delta;
		}

		if( a_C[index] < t + a_delta/2){
		/* No se corrobora que  (t - a_delta/2 < a_C[index]),
		 pues es automáticamente true tras el while.	*/
			a_C[index] = t;	// Estimación
		}
	}

	/* Vector de salida, de tamaño determinado por el número
	de puntos de salida. Todos sus elementos en 0 */
	scan_out->ranges.assign((((a_max - a_min)/a_delta)+1),0);

	/* Rellenar vector */
	int index = 0;	
	for(int i=0; i < s_C; i++){

		index = redondear((a_C[i] - a_min)/a_delta);

		if(scan_out->ranges[index] == 0){

			/* Rellenar */
			scan_out->ranges[index] = C[i];

		} else if(scan_out->ranges[index] > C[i]){

			/* Quedarse con la menor medición */
			scan_out->ranges[index] = C[i];
		}
	}
}

/* Hace el cambio de coordenadas */
void transform(array_ *d, float a_min, float a_delta, array_ *dist, array_ *angles, float *x_){

	*dist = array_ ((*d).size());
	*angles = array_ ((*d).size());
	
	for(int i=0; i< (int)((*d).size()); i++){

		float theta = a_min + i*a_delta;
		float x = *x_ + (*d)[i]*cos(theta);
		float y = (*d)[i]*sin(theta);

		(*dist)[i] = sqrt(x*x+y*y);

		if( (*dist)[i] < bender_radio)	{
			(*dist)[i] = 0;
			(*angles)[i] = 0;
		} else {
		
			(*angles)[i] = -atan2(x,y) + M_PI/2;
			if((*angles)[i] > M_PI){

				(*angles)[i] = (*angles)[i] - 2*M_PI;
			}
		}
	}
}

/* REVISADO - FUNCIONA　シ　*/
/* Junta arrays A,B => [A B] */
array_ juntar(const array_ *A,const array_ *B){

	int s_a = (*A).size();
	int s_b = (*B).size();
	array_ C (s_a + s_b);

	for(int i=0; i < s_a; i++){

		C[i] = (*A)[i];
	}
	for(int i=0; i < s_b; i++){

		C[i+s_a] = (*B)[i];
	}

	return C;
}

/* REVISADO - FUNCIONA　シ　*/
void setup_scan_common(){

	scan_out->angle_min = -M_PI;
	scan_out->angle_max = M_PI;
	scan_out->header.frame_id = frame_virtual;
	scan_out->angle_increment  = 2.0*M_PI/num_readings;
	scan_out->scan_time = 0.1;			// 10 [Hz]
	scan_out->time_increment = 0.0;		// ver forma de calcularlo (de)
	scan_out->range_min = 0.20;			// 20 [cm]
	scan_out->range_max = 8.00;			// 8.0 [m]
}

/* REVISADO - FUNCIONA　シ　*/
void setup_scan_ambos_mode(){

	//ROS_INFO("[laser_merger]: Working with 2 lasers");
	scan_out->angle_min = -M_PI;
	scan_out->angle_max = M_PI;
	scan_out->angle_increment  = 2.0*M_PI/num_readings;
	// vel_x != 0;
	// acel_x != 0;
	// Escape vel != 0;
}

/* REVISADO - FUNCIONA　シ　*/
void setup_scan_front_mode(){

	//ROS_WARN("[laser_merger]: Working only with laser_front");
	scan_out->angle_min = -1.57;
	scan_out->angle_max = 1.57;
	scan_out->angle_increment  = 2.0*1.57/num_readings;
	// vel_x != 0;
	// acel_x != 0;
	// Escape_vel = 0;
}

/* REVISADO - FUNCIONA　シ　*/
void setup_scan_rear_mode(){

	ROS_WARN("[laser_merger]: Laser front desconectado!!, No se publicaran medidas hasta su reconexion");
	scan_out->angle_min = 0;
	scan_out->angle_max = 0;
	scan_out->angle_increment  = 2.0*0.0/num_readings;
	// vel_x = 0;
	// acel_x = 0;
	// Escape_vel = 0;
}

/* REVISADO - FUNCIONA　シ　*/
int redondear(float n){

	return  n - floor(n) > 0.5 ? ceil(n) : floor(n);
}

/* ------------------------- Quicksort  ---------------------------------------*/
/* La ordenación se hace según los valores de *B!! */

/* REVISADO - FUNCIONA　シ　*/
void quicksort(array_ *A, array_ *B, int first, int last ){

  int posicionActual;
  if ( first >= last )
    return;
  posicionActual = particion(A, B, first, last );
  quicksort( A, B, first, posicionActual - 1 );   
  quicksort( A, B, posicionActual + 1, last );    
}

/* REVISADO - FUNCIONA　シ　*/
int particion(array_ *A, array_ *B, int izquierda, int derecha ){

  int posicion = izquierda;
  while ( true ) {
    while ( (*B)[ posicion ] <= (*B)[ derecha ] && posicion != derecha )
      --derecha;
    if ( posicion == derecha )
      return posicion;
    if ( (*B)[ posicion ] > (*B)[ derecha ]) {
      swap( &((*A)[ posicion ]), &((*A)[ derecha ]) );
      swap( &((*B)[ posicion ]), &((*B)[ derecha ]) );
      posicion = derecha;
    }
    while ( (*B)[ izquierda ] <= (*B)[ posicion ] && izquierda != posicion )
      ++izquierda;
    if ( posicion == izquierda )
      return posicion;
    if ( (*B)[ izquierda ] > (*B)[ posicion ] ) {
      swap( &((*A)[ posicion ]), &((*A)[ izquierda ]) );
      swap( &((*B)[ posicion ]), &((*B)[ izquierda ]) );
      posicion = izquierda;
    }
  }
  return 0;
}

/* REVISADO - FUNCIONA　シ　*/
void swap(float * const ptr1, float * const ptr2){

  float temp = *ptr1;
  *ptr1 = *ptr2;
  *ptr2 = temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
bool serviceCB(bender_srvs::getLaserScan::Request &req, bender_srvs::getLaserScan::Response &res){

	res.scan = *scan_out;

	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////


/* ---------------------------- ^  Quicksort ^ ----------------------------------- */
/* ------------------------------------------------------------------------------- */

int main(int argc, char **argv){

	ROS_INFO("[laser_merger]: Configuring...");
	ros::init(argc, argv, "laser_merger");
	ros::NodeHandle n_;

	/* scan_out presetting */
	setup_scan_common();

	/* Topics */
	pub = n_.advertise<sensor_msgs::LaserScan>("/bender/sensors/laser_virtual/scan", 100);
	sub = n_.subscribe("/bender/sensors/laser_front/scan", 2, callback);
	sub = n_.subscribe("/bender/sensors/laser_rear/scan", 2, callback);
	server = n_.advertiseService("/bender/sensors/laser_virtual/get_scan", serviceCB);

	ROS_INFO("[laser_merger]: Config. Done");
	ros::spin();

	ROS_INFO("[laser_merger]: Quitting... \n");
	return 0;
}
