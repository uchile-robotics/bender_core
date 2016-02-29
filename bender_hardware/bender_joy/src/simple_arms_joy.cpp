 
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <bender_msgs/Command.h>
#include <bender_srvs/PlanningGoalCartesian.h>
#include <bender_srvs/PlanningGoalState.h>
#include <bender_srvs/TorqueEnable.h>
#include <bender_srvs/LoadMode.h>
#include <bender_srvs/AngVel.h>

class Joystick {

public:
	Joystick();

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
	inicialPoseR = nh_.serviceClient<std_srvs::Empty>("right_arm/posicion_inicial");
	premanip1R = nh_.serviceClient<std_srvs::Empty>("right_arm/posicion_premanipulacion1");
	premanip2R = nh_.serviceClient<std_srvs::Empty>("right_arm/posicion_premanipulacion2");
	planstateR = nh_.serviceClient<bender_srvs::PlanningGoalCartesian>("right_arm/grasp");
	plancartR= nh_.serviceClient<bender_srvs::PlanningGoalState>("right_arm/plan_state");
	torqueenableR = nh_.serviceClient<bender_srvs::TorqueEnable>("right_arm/torque_enable");
	cerrarR = nh_.serviceClient<bender_srvs::LoadMode>("right_arm/cerrar_grip");
	orientarR = nh_.serviceClient<std_srvs::Empty>("right_arm/orientar_grip");

	inicialPoseL = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_inicial");
	premanip1L = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_premanipulacion1");
	premanip2L = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_premanipulacion2");
	planstateL = nh_.serviceClient<bender_srvs::PlanningGoalCartesian>("left_arm/grasp");
	plancartL= nh_.serviceClient<bender_srvs::PlanningGoalState>("left_arm/plan_state");
	torqueenableL = nh_.serviceClient<bender_srvs::TorqueEnable>("left_arm/torque_enable");
	cerrarL = nh_.serviceClient<bender_srvs::LoadMode>("left_arm/cerrar_grip");
	orientarL = nh_.serviceClient<std_srvs::Empty>("left_arm/orientar_grip");

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

/**
 * joyCallback features:
 * - up, down, left, right
 * - diferent velocities (depends on angle of stick)
 * - pause/resume
 */

void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	std_srvs::Empty dum;
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
s0: 0.598252507275
s1: 0.27905718299
s2: 0.22498384889
s3: 1.73033032874

Posiciones brazo izquierdo
s0: -0.521553467881
s1: -0.242368964486
s2: 0.39883500485
s3: 1.77634975237
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
					saludoStateR.positions[i] = 0.598252507275;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = -0.521553467881;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;
				}
				if(i==1)
				{
					saludoStateR.positions[i] = 0.27905718299;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = -0.242368964486;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;
				}
				if(i==2)
				{
					saludoStateR.positions[i] = 0.22498384889;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = 0.39883500485;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;			
				}
				if(i==3)
				{
					saludoStateR.positions[i] = 1.73033032874;
					saludoStateR.speed[i] = 0.2;
					saludoStateR.select[i]=true;
					saludoStateL.positions[i] = 1.77634975237;
					saludoStateL.speed[i] = 0.2;
					saludoStateL.select[i]=true;	
				}
			}

			motoresR.publish(saludoStateR);
			motoresL.publish(saludoStateL);
		}
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joy_upper_arms");
	Joystick joystick;

	ros::spin();
}
