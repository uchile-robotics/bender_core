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

	ros::Publisher motores;
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

	inicialPoseL = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_inicial");
	premanip1L = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_premanipulacion1");
	premanip2L = nh_.serviceClient<std_srvs::Empty>("left_arm/posicion_premanipulacion2");
	planstateL = nh_.serviceClient<bender_srvs::PlanningGoalCartesian>("left_arm/grasp");
	plancartL= nh_.serviceClient<bender_srvs::PlanningGoalState>("left_arm/plan_state");
	torqueenableL = nh_.serviceClient<bender_srvs::TorqueEnable>("left_arm/torque_enable");
	cerrarL = nh_.serviceClient<bender_srvs::LoadMode>("left_arm/cerrar_grip");

	movergripR = nh_.serviceClient<bender_srvs::AngVel>("right_arm/mover_grip_ang");
	movergripL = nh_.serviceClient<bender_srvs::AngVel>("left_arm/mover_grip_ang");

	movermunecaR = nh_.serviceClient<bender_srvs::AngVel>("right_arm/mover_right_muneca_ang");
	movermunecaL = nh_.serviceClient<bender_srvs::AngVel>("left_arm/mover_muneca_ang");

	// publishers
	motores = nh_.advertise<bender_msgs::Command>("right_arm_joints/command",1000);
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


	// check buttons
	if ( joy->axes[2] == -1 ) {

		if ( joy-> buttons[0] == 1 ){
			ROS_INFO("posicion inicial");
			inicialPoseR.call(dum);
			inicialPoseL.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[1] == 1 ){
			ROS_INFO("premanipulacion 1");
			premanip1R.call(dum);
			premanip1L.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[3] == 1 ){
			ROS_INFO("premanipulacion 2");
			premanip2R.call(dum);
			premanip2L.call(dum);
			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[13] == 1 ){
			ROS_INFO("estirando el brazo");
			angulovelocidad.request.angle = 0;
			angulovelocidad.request.velocity = 0.4;

			movermunecaR.call(angulovelocidad);
			movermunecaL.call(angulovelocidad);

			planL.request.x = 70;
			planL.request.y = 28;
			planL.request.z = 100;
			planR.request.x = 70;
			planR.request.y = -28;
			planR.request.z = 100;
			planstateL.call(planL);
			planstateR.call(planR);

			_torque = true;
			contador++;
		}
		else if ( joy-> buttons[12] == 1 ){
			ROS_INFO("cerrar grip con loadmode");
			carga.request.loadmode = 0;
			cerrarL.call(carga);
			cerrarR.call(carga);
			contador++;
		}
		else if ( joy-> buttons[14] == 1 ){
			ROS_INFO("abrir grip");
			angulovelocidad.request.angle = 0.7;
			angulovelocidad.request.velocity = 0.3;
			movergripL.call(angulovelocidad);
			movergripR.call(angulovelocidad);
			contador++;
		}
		else if ( joy-> buttons[11] == 1 ){
			ROS_INFO("cerrar grip sin loadmode");
			angulovelocidad.request.angle = 0.3;
			angulovelocidad.request.velocity = 0.3;
			movergripL.call(angulovelocidad);
			movergripR.call(angulovelocidad);
			contador++;
		}
		else if ( joy->buttons[5] == 1){
			ROS_INFO("Contador = %d",contador);
		}
		else if ( joy->buttons[2] == 1 ){
			_torque = !_torque;
			torque.request.torque_enable = _torque;
			ROS_INFO("Torque = %s", _torque?"true":"false");
			torqueenableL.call(torque);
			torqueenableR.call(torque);
		}
		else if ( joy->buttons[10] == 1 ){
			ROS_INFO("John Travolta");
			//brazo abajo
			botStateR.request.s0 = -0.1;
			botStateR.request.s1 = 1.0;
			botStateR.request.s2 = -1.4777;
			botStateR.request.s3 = 1.7671;
			botStateL.request.s0 = 0.0;
			botStateL.request.s1 = -1.0;
			botStateL.request.s2 = -1.3;
			botStateL.request.s3 = 1.7671;
			//brazo al medio
			topState1R.request.s0 = 0.53229;
			topState1R.request.s1 = 0.70716;
			topState1R.request.s2 = -1.5;
			topState1R.request.s3 = 1.93588;
			topState1L.request.s0 = -0.53229;
			topState1L.request.s1 = -0.70716;
			topState1L.request.s2 = -1.5;
			topState1L.request.s3 = 1.93588;
			//brazo arriba
			topState2R.request.s0 = 0.53229;
			topState2R.request.s1 = 0.70716;
			topState2R.request.s2 = 0.9664;
			topState2R.request.s3 = 1.93588;
			topState2L.request.s0 = -0.53229;
			topState2L.request.s1 = -0.70716;
			topState2L.request.s2 = 0.9664;
			topState2L.request.s3 = 1.93588;

			//movimiento de arriba a abajo derecho
			//		  ROS_INFO("0");
			com1.positions.resize(8);
			com2.speed.resize(8);
			com2.select.resize(8);
			com2.positions.resize(8);
			com1.speed.resize(8);
			com1.select.resize(8);
			for(int i=0;i<8;i++)
			{
				//			ROS_INFO("1/2");
				com1.positions[i] = 0;
				com1.speed[i] = 0;
				com1.select[i]=false;
				//			ROS_INFO("1");
				if(i==2)
				{
					com1.positions[i] = -1.5;
					com1.speed[i] = 0.4;
					com1.select[i]=true;
					//			ROS_INFO("2");
				}
			}
			//		  ROS_INFO("3");
			//movimiento de abajo hacia arriba derecho
			for(int i=0;i<8;i++)
			{
				com2.positions[i] = 0;
				com2.speed[i] = 0;
				com2.select[i]=false;
				//			ROS_INFO("4");
				if(i==2)
				{
					com2.positions[i] = 0.9664;
					com2.speed[i] = 0.4;
					com2.select[i]=true;
					//				ROS_INFO("5");
				}
			}

			//movimiento de arriba a abajo izquierdo
			//		  ROS_INFO("0");
			com3.positions.resize(8);
			com4.speed.resize(8);
			com4.select.resize(8);
			com4.positions.resize(8);
			com3.speed.resize(8);
			com3.select.resize(8);
			for(int i=0;i<8;i++)
			{
				//			ROS_INFO("1/2");
				com3.positions[i] = 0;
				com3.speed[i] = 0;
				com3.select[i]=false;
				//			ROS_INFO("1");
				if(i==2)
				{
					com3.positions[i] = -1.5;
					com3.speed[i] = 0.4;
					com3.select[i]=true;
					//			ROS_INFO("2");
				}
			}
			//		  ROS_INFO("3");
			//movimiento de abajo hacia arriba izquierdo
			com2.positions.resize(8);
			for(int i=0;i<8;i++)
			{
				com4.positions[i] = 0;
				com4.speed[i] = 0;
				com4.select[i]=false;
				//			ROS_INFO("4");
				if(i==2)
				{
					com4.positions[i] = 0.9664;
					com4.speed[i] = 0.4;
					com4.select[i]=true;
					//				ROS_INFO("5");
				}
			}


			plancartR.call(botStateR); // ambos brazos abajo
			plancartL.call(botStateL);

			plancartR.call(topState2R); // sube el brazo derecho y baila
			ros::Duration(1).sleep();
			motores.publish(com1);
			ros::Duration(3.5).sleep();
			motores.publish(com2);
			ros::Duration(3.5).sleep();
			motores.publish(com1);
			ros::Duration(3.5).sleep();
			motores.publish(com2);
			ros::Duration(3.5).sleep();


			plancartR.call(botStateR); // baja el brazo derecho

			plancartL.call(topState2L); // sube el brazo izquierdo y baila
			ros::Duration(1).sleep();
			motoresL.publish(com3);
			ros::Duration(3.5).sleep();
			motoresL.publish(com4);
			ros::Duration(3.5).sleep();
			motoresL.publish(com3);
			ros::Duration(3.5).sleep();
			motoresL.publish(com4);
			ros::Duration(3.5).sleep();

			plancartL.call(botStateL);

		}
	}
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joy_upper_arms");
	Joystick joystick;

	ros::spin();
}
