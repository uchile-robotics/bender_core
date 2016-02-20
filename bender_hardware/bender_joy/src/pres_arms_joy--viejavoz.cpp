 #include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <bender_msgs/Command.h>
#include <bender_srvs/Dummy.h>
#include <bender_srvs/PlanningGoalCartesian.h>
#include <bender_srvs/PlanningGoalState.h>
#include <bender_srvs/TorqueEnable.h>
#include <bender_srvs/LoadMode.h>
#include <bender_srvs/AngVel.h>
#include <bender_srvs/synthesize.h>

#include <bender_msgs/Emotion.h>

class Joystick {

public:
	Joystick();
	void speech_pres(std::string);
	void speak_Off();
	void show_emotion(std::string text,std::string tipo = "changeFace", int angle = 0);
	void inauguracion(bender_msgs::Command saludoStateR, bender_msgs::Command saludoStateL);

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	// ros stuff
	ros::NodeHandle nh_;
	ros::Subscriber joy_sub_;
	ros::ServiceClient inicialPoseR;
	ros::ServiceClient premanip1R;
	ros::ServiceClient premanip2R;
	ros::ServiceClient planstateR;
	ros::ServiceClient torqueenableR;
	ros::ServiceClient plancartR;

	ros::ServiceClient planstateL;
	ros::ServiceClient torqueenableL;
	ros::ServiceClient inicialPoseL;
	ros::ServiceClient premanip1L;
	ros::ServiceClient premanip2L;
	ros::ServiceClient plancartL;

	ros::ServiceClient cerrarL;
	ros::ServiceClient cerrarR;
	ros::ServiceClient movergripR;
	ros::ServiceClient movergripL;
	ros::ServiceClient movermunecaR;
	ros::ServiceClient movermunecaL;

	ros::ServiceClient orientarR;
	ros::ServiceClient orientarL;

	ros::ServiceClient speech_serv_;
	ros::Publisher speech_status;

	ros::Publisher face_pub_;

	ros::Publisher motoresR;
	ros::Publisher motoresL;

	// button state
	bool is_paused_;

	bool _torque;

	int contador;

};


Joystick::Joystick(){

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Joystick::joyCallback, this);


	// services
	inicialPoseR = nh_.serviceClient<bender_srvs::Dummy>("right_arm/posicion_inicial");
	premanip1R = nh_.serviceClient<bender_srvs::Dummy>("right_arm/posicion_premanipulacion1");
	premanip2R = nh_.serviceClient<bender_srvs::Dummy>("right_arm/posicion_premanipulacion2");
	planstateR = nh_.serviceClient<bender_srvs::PlanningGoalCartesian>("right_arm/grasp");
	plancartR= nh_.serviceClient<bender_srvs::PlanningGoalState>("right_arm/plan_state");
	torqueenableR = nh_.serviceClient<bender_srvs::TorqueEnable>("right_arm/torque_enable");
	cerrarR = nh_.serviceClient<bender_srvs::LoadMode>("right_arm/cerrar_grip");
	orientarR = nh_.serviceClient<bender_srvs::Dummy>("right_arm/orientar_grip");

	inicialPoseL = nh_.serviceClient<bender_srvs::Dummy>("left_arm/posicion_inicial");
	premanip1L = nh_.serviceClient<bender_srvs::Dummy>("left_arm/posicion_premanipulacion1");
	premanip2L = nh_.serviceClient<bender_srvs::Dummy>("left_arm/posicion_premanipulacion2");
	planstateL = nh_.serviceClient<bender_srvs::PlanningGoalCartesian>("left_arm/grasp");
	plancartL= nh_.serviceClient<bender_srvs::PlanningGoalState>("left_arm/plan_state");
	torqueenableL = nh_.serviceClient<bender_srvs::TorqueEnable>("left_arm/torque_enable");
	cerrarL = nh_.serviceClient<bender_srvs::LoadMode>("left_arm/cerrar_grip");
	orientarL = nh_.serviceClient<bender_srvs::Dummy>("left_arm/orientar_grip");


	speech_serv_ = nh_.serviceClient<bender_srvs::synthesize>("/bender/speech/synthesizer/synthesize");

	face_pub_ = nh_.advertise<bender_msgs::Emotion>("/bender/face/head", 1);

	movergripR = nh_.serviceClient<bender_srvs::AngVel>("right_arm/mover_grip_ang");
	movergripL = nh_.serviceClient<bender_srvs::AngVel>("left_arm/mover_grip_ang");

	movermunecaR = nh_.serviceClient<bender_srvs::AngVel>("right_arm/mover_right_muneca_ang");
	movermunecaL = nh_.serviceClient<bender_srvs::AngVel>("left_arm/mover_muneca_ang");

	// publishers
	motoresR = nh_.advertise<bender_msgs::Command>("right_arm_joints/command",1000);
	motoresL = nh_.advertise<bender_msgs::Command>("left_arm_joints/command",1000);

	is_paused_ = false;
	contador = 1;
	_torque = false;

}

void  Joystick::inauguracion(bender_msgs::Command saludoStateR, bender_msgs::Command saludoStateL){

bender_srvs::Dummy dum;

show_emotion("happy2");
speech_pres("Hola"); ros::Duration(0.7).sleep();
speech_pres("bienvenidos a nuestra nueva casa");ros::Duration(3).sleep();
speech_pres("el complejo de edificios Buchef ochocientos cincuenta y uno");ros::Duration(4.8).sleep();
speech_pres("espero que les haya gustado");ros::Duration(2.3).sleep();show_emotion("happy3");
speech_pres("a m'i, me encant'o");ros::Duration(2).sleep();show_emotion("happy2");
speech_pres("Para los que no me conocen, soy B'ender");ros::Duration(3.5).sleep();
speech_pres("el robot de servicio, que desarrollaron estudiantes de la Facultad en el laboratorio de rob'otica."); ros::Duration(7).sleep();
speak_Off();
ros::Duration(1).sleep();
show_emotion("","MoveX",10);

speech_pres("Hoy estamos celebrando"); ros::Duration(1.3).sleep();show_emotion("1313");ros::Duration(1).sleep();
speech_pres("as'i que los invito a que levanten sus copas "); ros::Duration(3.5).sleep();
speech_pres("y hagamos un brindis por Buchef Poniente"); ros::Duration(3).sleep();
speech_pres("supongo que ya est'an listos"); ros::Duration(2.5).sleep();show_emotion("happy3");
speech_pres("ji ji ji, salud"); ros::Duration(2.5).sleep();
show_emotion("happy2");
speak_Off();
ros::Duration(1).sleep();
show_emotion("","MoveX",20);

speech_pres("Quiero aprovechar mis segundos de fama"); ros::Duration(3).sleep();show_emotion("surprise");
speech_pres("ya que todos ustedes me est'an mirando"); ros::Duration(3).sleep();show_emotion("happy2");
speech_pres("para felicitar y agradecer a todos los que trabajaron en este proyecto"); ros::Duration(5.3).sleep();
speech_pres("No puedo nombrarlos a todos"); ros::Duration(2.3).sleep();
speech_pres("no tengo tan buena memoria"); ros::Duration(2).sleep();
show_emotion("sad2");ros::Duration(2).sleep();show_emotion("happy2");
speech_pres("ji ji ji"); ros::Duration(1.5).sleep();
speech_pres("Pero quiero destacar "); ros::Duration(1.7).sleep();
speech_pres("a los arquitectos de A4 + Borja Huidobro"); ros::Duration(3.8).sleep();
speech_pres("a Ingevec, que construy'o este mega proyecto"); ros::Duration(3.8).sleep();
speech_pres("mmmmm aunque con bastante demora"); ros::Duration(2).sleep();
show_emotion("angry2");ros::Duration(2).sleep();show_emotion("happy3");
speech_pres("ji ji ji"); ros::Duration(1.5).sleep();
speech_pres("Es broma amigos!!! "); ros::Duration(2).sleep();show_emotion("happy2");
speech_pres("Al IDIEM por todo el gerenciamiento y la inspecci'on t'ecnica de obra."); ros::Duration(5.5).sleep();
speech_pres("a todo esto, ustedes eran los que hac'ian sufrir a Ingevec"); ros::Duration(4.8).sleep();
speech_pres("ji ji ji"); ros::Duration(1).sleep();
speak_Off();
ros::Duration(2).sleep();
show_emotion("","MoveX",10);

speech_pres("Tambi'en al escultor Francisco Gacit'ua"); ros::Duration(3.5).sleep(); 
speech_pres("que nos cre'o una obra que hoy vuela en nuestro jol de acceso"); ros::Duration(4.4).sleep();
speech_pres("Finalmente agradecer al mentor de esta obra"); ros::Duration(3.5).sleep();
speech_pres("El ex decano Francisco Brieva Rodr'iguez"); ros::Duration(3.5).sleep();
speech_pres("pucha que tiene buen ojo"); ros::Duration(2).sleep();
speak_Off();
show_emotion("1313");ros::Duration(1).sleep();
show_emotion("","MoveX",0);

speech_pres("Gracias a todos ellos "); ros::Duration(2).sleep();
speech_pres("y a ustedes por venir a celebrar con nosotros. "); ros::Duration(3).sleep();
//(Debe levantar los brazos porque ésa será la señal para la siguiente puesta en escena)…. 
motoresR.publish(saludoStateR);
motoresL.publish(saludoStateL);
ros::Duration(3).sleep();
speech_pres("Ahora Buchef ochocientos cincuenta y uno"); ros::Duration(4).sleep();
speech_pres("comienza a tomar vida"); ros::Duration(2).sleep();
speech_pres("miren a su alrededor."); ros::Duration(1).sleep();speak_Off();ros::Duration(1).sleep();
show_emotion("surprise");
show_emotion("","MoveX",20);ros::Duration(1).sleep();show_emotion("","MoveX",-20);
ros::Duration(1).sleep();show_emotion("","MoveX",0);
ros::Duration(2).sleep();
show_emotion("happy2");

inicialPoseL.call(dum);
inicialPoseR.call(dum);

}


void  Joystick::show_emotion(std::string text,std::string tipo, int angle){
	bender_msgs::Emotion emotion;
	emotion.Order = tipo;
	emotion.Action = text;
	emotion.X = angle;
	face_pub_.publish(emotion);
}

void  Joystick::speak_Off(){
	bender_msgs::Emotion emotion;
	emotion.Order = "changeFace";
	emotion.Action = "speakOff";
	emotion.X = 0;
	face_pub_.publish(emotion);
}

void  Joystick::speech_pres(std::string text){
std::cout << text<<std::endl;
	bender_srvs::synthesize speech_text;
	speech_text.request.text = text;
	speech_serv_.call(speech_text);
}
/**
 * joyCallback features:
 * - up, down, left, right
 * - diferent velocities (depends on angle of stick)
 * - pause/resume
 */

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	bender_srvs::Dummy dum;
	bender_srvs::PlanningGoalCartesian planR,planL;
	bender_srvs::PlanningGoalState botStateL, topState1L, topState2L, botStateR, topState1R, topState2R;
	bender_srvs::TorqueEnable torque;
	bender_srvs::LoadMode carga;
	bender_srvs::AngVel angulovelocidad;

	bender_msgs::Command com1;
	bender_msgs::Command com2;

	bender_msgs::Command com3;
	bender_msgs::Command com4;

	bender_msgs::Command saludoStateL, saludoStateR;


	// CONTROL DEL BRAZO IZQUIERDO
	if ( joy->axes[2] == -1 && joy->axes[5] == 1 ) {

		if ( joy-> buttons[0] == 1 ){
			ROS_INFO("Left -> posicion inicial");
			inicialPoseL.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[1] == 1 ){
			ROS_INFO("Left -> premanipulacion 1");
			premanip1L.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[3] == 1 ){
			ROS_INFO("Left -> premanipulacion 2");
			premanip2L.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[13] == 1 ){
			ROS_INFO("Left -> estirando el brazo");
			angulovelocidad.request.angle = 0;
			angulovelocidad.request.velocity = 0.4;

			movermunecaL.call(angulovelocidad);

			planL.request.x = 70;
			planL.request.y = 28;
			planL.request.z = 100;
			planstateL.call(planL);

			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[12] == 1 ){
			ROS_INFO("Left -> cerrar grip con loadmode");
			carga.request.loadmode = 0;
			cerrarL.call(carga);
			contador++;
		}
		else if ( joy-> buttons[14] == 1 ){
			ROS_INFO("Left -> abrir grip");
			angulovelocidad.request.angle = 0.7;
			angulovelocidad.request.velocity = 0.3;
			movergripL.call(angulovelocidad);
			contador++;
		}
		else if ( joy-> buttons[11] == 1 ){
			ROS_INFO("Left -> cerrar grip sin loadmode");
			angulovelocidad.request.angle = 0.3;
			angulovelocidad.request.velocity = 0.3;
			movergripL.call(angulovelocidad);
			contador++;
		}
		else if ( joy->buttons[5] == 1){
			ROS_INFO("Contador = %d",contador);
		}
		else if ( joy->buttons[2] == 1 && joy->buttons[4] == 1 ){
			_torque = !_torque;
			torque.request.torque_enable = _torque;
			ROS_INFO("Left -> Torque = %s", _torque?"true":"false");
			torqueenableL.call(torque);
		}
		else if ( joy-> buttons[2] == 1 ){
			ROS_INFO("Left -> orientar grip");
			orientarL.call(dum);
			_torque = true;
			contador++;
		}
	}
	// CONTROL DEL BRAZO DERECHO
	if ( joy->axes[2] == 1 && joy->axes[5] == -1 ) {

		if ( joy-> buttons[0] == 1 ){
			ROS_INFO("Right -> posicion inicial");
			inicialPoseR.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[1] == 1 ){
			ROS_INFO("Right -> premanipulacion 1");
			premanip1R.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[3] == 1 ){
			ROS_INFO("Right -> premanipulacion 2");
			premanip2R.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[13] == 1 ){
			ROS_INFO("Right -> estirando el brazo");
			angulovelocidad.request.angle = 0;
			angulovelocidad.request.velocity = 0.4;

			movermunecaR.call(angulovelocidad);

			planR.request.x = 70;
			planR.request.y = -28;
			planR.request.z = 100;
			planstateR.call(planR);

			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[12] == 1 ){
			ROS_INFO("Right -> cerrar grip con loadmode");
			carga.request.loadmode = 0;
			cerrarR.call(carga);
			contador++;
		}
		else if ( joy-> buttons[14] == 1 ){
			ROS_INFO("Right -> abrir grip");
			angulovelocidad.request.angle = 0.7;
			angulovelocidad.request.velocity = 0.3;
			movergripR.call(angulovelocidad);
			contador++;
		}
		else if ( joy-> buttons[11] == 1 ){
			ROS_INFO("Right -> cerrar grip sin loadmode");
			angulovelocidad.request.angle = 0.3;
			angulovelocidad.request.velocity = 0.3;
			movergripR.call(angulovelocidad);
			contador++;
		}
		else if ( joy->buttons[5] == 1){
			ROS_INFO("Contador = %d",contador);
		}
		else if ( joy->buttons[2] == 1 && joy->buttons[5] == 1 ){
			_torque = !_torque;
			torque.request.torque_enable = _torque;
			ROS_INFO("Right -> Torque = %s", _torque?"true":"false");
			torqueenableR.call(torque);
		}
		else if ( joy-> buttons[2] == 1 ){
			ROS_INFO("Right -> orientar grip");
			orientarR.call(dum);
			_torque = true;
			contador++;
		}
	}

/*
Posiciones brazo derecho
s0: 1.25019434213
s1: 0.0609435227219
s2: 0.219870579597
s3: 1.3928545554

Posiciones brazo izquierdo
s0: -1.27473803473
s1: -0.104310693576
s2: -0.0255663464648
s3: 1.51710699922

*/

	// MOVIMIENTO CON AMBOS BRAZOS
	if ( joy->axes[2] == -1 && joy->axes[5] == -1 ) {

		if ( joy->buttons[10] == 1 ) {

			saludoStateR.positions.resize(8);
			saludoStateR.speed.resize(8);
			saludoStateR.select.resize(8);
			saludoStateL.positions.resize(8);
			saludoStateL.speed.resize(8);
			saludoStateL.select.resize(8);
			for(int i=0;i<8;i++)
			{
			//			ROS_INFO("1/2");
				saludoStateR.positions[i] = 0;
				saludoStateR.speed[i] = 0;
				saludoStateR.select[i]=false;
				//			ROS_INFO("1");
				if(i==0)
				{
					saludoStateR.positions[i] = 1.25019434213;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = -1.27473803473;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;
				}
				if(i==1)
				{
					saludoStateR.positions[i] = 0.0609435227219;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = -0.104310693576;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;
				}
				if(i==2)
				{
					saludoStateR.positions[i] = 0.219870579597;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = -0.0255663464648;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;			
				}
				if(i==3)
				{
					saludoStateR.positions[i] = 1.3928545554;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = 1.51710699922;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;	
				}
			}

			Joystick::inauguracion(saludoStateR,saludoStateL);


		}
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joy_arms_pres");
	Joystick joystick;
//	joystick.inauguracion();
	ros::spin();
}
