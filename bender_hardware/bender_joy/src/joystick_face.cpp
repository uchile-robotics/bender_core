#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <bender_msgs/Emotion.h>
#include <bender_srvs/synthesize.h>

#define MAX_FACE_ANGLE 45
#define MIN_FACE_ANGLE -45
#define CENTER_FACE_ANGLE 0


class Joystick {

public:
	Joystick();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	// ros stuff
	ros::NodeHandle nh_;
	ros::Publisher face_pub_;
	ros::Subscriber joy_sub_;
	ros::ServiceClient speech_serv_;

	int face_intensity;
	int face_yaw_position;
		
	// button state
	bool is_paused_;

};

Joystick::Joystick():
		  face_intensity(1),
		  face_yaw_position(CENTER_FACE_ANGLE)
{
	
	face_pub_ = nh_.advertise<bender_msgs::Emotion>("/bender/face/head", 1);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Joystick::joyCallback, this);
	speech_serv_ = nh_.serviceClient<bender_srvs::synthesize>("/bender/speech/synthesizer/synthesize");

	is_paused_ = false;
}

/**
 * joyCallback features:
 * - up, down, left, right
 * - diferent velocities (depends on angle of stick)
 * - pause/resume
 */
void Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	bender_msgs::Emotion emotion;
	bender_srvs::synthesize speech_text;

	// check for pause
	if ( joy->axes[2]==-1) {

		if ( joy->buttons[0] == 1 ) {
			speech_text.request.text = "Hola, mi nombre es Bender.";
			speech_serv_.call(speech_text);

		} else if ( joy->buttons[1] == 1 ) {

			speech_text.request.text = "Tengo ganas de beber cerveza";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[2] == 1 ) {

			speech_text.request.text = "He sido desarollado en la Universidad de Chile";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[3] == 1 ) {

			speech_text.request.text = "Hay muchas cosas que puedo hacer, puedo detectar y reconocer personas.";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[4] == 1 ) {

			speech_text.request.text = "Detectar un reconocer objetos, navegar si chocar";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[5] == 1 ) {

			speech_text.request.text = "Mover mis brazos y manipular objetos.";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[6] == 1 ) {

			speech_text.request.text = "Algo que hago muy bien es expresar emociones";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[7] == 1 ) {

			speech_text.request.text = "Por ejemplo me puedo poner feliz";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[8] == 1 ) {

			speech_text.request.text = "o muy feliz";
			speech_serv_.call(speech_text);

			speech_text.request.text = "Si la gente me molesta me pongo triste";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[9] == 1 ) {

			speech_text.request.text = "aunque a veces tambien me enojo si no me quieren hacer caso";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[10] == 1 ) {

			speech_text.request.text = "Cuando me vienen a ver tambien me sorprendo";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[11] == 1 ) {

			speech_text.request.text = "Ahora vor a mostrarles una de mis habilidades";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[12] == 1 ) {

			speech_text.request.text = "Mi amigo Mauricio les va a explicar de que se trata";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[13] == 1 ) {

			speech_text.request.text = "Muchas gracias por venir";
			speech_serv_.call(speech_text);

		}else if ( joy->buttons[14] == 1 ) {

			speech_text.request.text = "Hola, te veo saludandome";
			speech_serv_.call(speech_text);
		}

	}else if ( joy->axes[5]==-1) {
		if ( joy->buttons[6] == 1 ) {

			emotion.Order = "changeFace";
			emotion.Action = "ashamed";
			emotion.X = 0;


		}else if ( joy->buttons[0] == 1){

			speech_text.request.text = "Hola, mi nombre es Bender.";
			speech_serv_.call(speech_text);
			ros::Duration(3).sleep();

			speech_text.request.text = "He sido desarollado en la Universidad de Chile";
			speech_serv_.call(speech_text);
			ros::Duration(5).sleep();

			speech_text.request.text = "Hay muchas cosas que puedo hacer, puedo detectar y reconocer personas.";
			speech_serv_.call(speech_text);
			ros::Duration(8).sleep();

			speech_text.request.text = "Detectar un reconocer objetos, navegar si chocar";
			speech_serv_.call(speech_text);
			ros::Duration(6).sleep();

			speech_text.request.text = "Mover mis brazos y manipular objetos.";
			speech_serv_.call(speech_text);
			ros::Duration(5).sleep();

			speech_text.request.text = "Algo que hago muy bien es expresar emociones";
			speech_serv_.call(speech_text);
			ros::Duration(6).sleep();

			speech_text.request.text = "Por ejemplo me puedo poner feliz";
			speech_serv_.call(speech_text);

			emotion.Order = "changeFace";
			emotion.Action = "happy1";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(4).sleep();


			speech_text.request.text = "o muy feliz";
			speech_serv_.call(speech_text);

			emotion.Order = "changeFace";
			emotion.Action = "happy3";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(3).sleep();


			speech_text.request.text = "Si la gente me molesta me pongo triste";
			speech_serv_.call(speech_text);
			emotion.Order = "changeFace";
			emotion.Action = "sad3";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(6).sleep();

			speech_text.request.text = "aunque a veces tambien me enojo si no me quieren hacer caso";
			speech_serv_.call(speech_text);
			emotion.Order = "changeFace";
			emotion.Action = "angry3";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(7).sleep();

			speech_text.request.text = "Cuando me vienen a ver tambien me sorprendo";
			speech_serv_.call(speech_text);
			emotion.Order = "changeFace";
			emotion.Action = "surprise";
			emotion.X = 0;
			face_pub_.publish(emotion);

			ros::Duration(6).sleep();

			emotion.Order = "changeFace";
			emotion.Action = "serious";
			emotion.X = 0;
			face_pub_.publish(emotion);


			speech_text.request.text = "Si tienen preguntas se las pueden hacer a Mauricio";
			speech_serv_.call(speech_text);
			ros::Duration(6).sleep();

			speech_text.request.text = "Muchas gracias por venir";
			speech_serv_.call(speech_text);
			ros::Duration(4).sleep();



		}else if ( joy->buttons[1] == 1){
			speech_text.request.text = "Hola, como estas";
			speech_serv_.call(speech_text);


		}else if ( joy->buttons[2] == 1){
			speech_text.request.text = "me saludas a mi o a alguien mas";
			speech_serv_.call(speech_text);


		}else if ( joy->buttons[3] == 1){
			speech_text.request.text = "Hola";
			speech_serv_.call(speech_text);


		}else if ( joy->buttons[8] == 1){
			emotion.Order = "changeFace";
			emotion.Action = "lost";
			emotion.X = 0;


		}else if ( joy->buttons[7] == 1){
			emotion.Order = "changeFace";
			emotion.Action = "no";
			emotion.X = 0;

		}else if ( joy->buttons[10] == 1){
			emotion.Order = "changeFace";
			emotion.Action = "ear";
			emotion.X = 0;

		}else if ( joy->buttons[4] == 1){
			emotion.Order = "changeFace";
			emotion.Action = "greetings";
			emotion.X = 0;

		}else if ( joy->buttons[5] == 1){
			emotion.Order = "changeFace";
			emotion.Action = "eyebrow";
			emotion.X = 0;

		}
		face_pub_.publish(emotion);

	}else if ( joy->buttons[0] == 1 ) {

		if (face_intensity==1){
			emotion.Order = "changeFace";
			emotion.Action = "happy1";
			emotion.X = 0;
		}else if (face_intensity==2){
			emotion.Order = "changeFace";
			emotion.Action = "happy2";
			emotion.X = 0;
		}else if (face_intensity==3){
			emotion.Order = "changeFace";
			emotion.Action = "happy3";
			emotion.X = 0;
		}
		face_pub_.publish(emotion);


	} else if ( joy->buttons[1] == 1 ) {

		if (face_intensity==1){
			emotion.Order = "changeFace";
			emotion.Action = "angry1";
			emotion.X = 0;
		}else if (face_intensity==2){
			emotion.Order = "changeFace";
			emotion.Action = "angry2";
			emotion.X = 0;
		}else if (face_intensity==3){
			emotion.Order = "changeFace";
			emotion.Action = "angry3";
			emotion.X = 0;
		}
		face_pub_.publish(emotion);
	}else if ( joy->buttons[2] == 1 ) {

		if (face_intensity==1){
			emotion.Order = "changeFace";
			emotion.Action = "sad1";
			emotion.X = 0;
		}else if (face_intensity==2){
			emotion.Order = "changeFace";
			emotion.Action = "sad2";
			emotion.X = 0;
		}else if (face_intensity==3){
			emotion.Order = "changeFace";
			emotion.Action = "sad3";
			emotion.X = 0;
		}
		face_pub_.publish(emotion);
	}else if ( joy->buttons[3] == 1 ) {

		emotion.Order = "changeFace";
		emotion.Action = "surprise";
		emotion.X = 0;
		face_pub_.publish(emotion);
	}else if ( joy->buttons[8] == 1 ) {

		emotion.Order = "changeFace";
		emotion.Action = "serious";
		emotion.X = 0;
		face_pub_.publish(emotion);

	}else if ( joy->buttons[4] == 1 ) {

		face_intensity--;
		if(face_intensity<1)
			face_intensity=1;

	}else if ( joy->buttons[5] == 1 ) {

		face_intensity++;
		if(face_intensity>3)
			face_intensity=3;

	}else if ( joy->buttons[10] == 1 ) {

		emotion.Order = "MoveX";
		emotion.Action = " ";
		emotion.X = face_yaw_position;
		face_pub_.publish(emotion);

	}else if ( joy->buttons[6] == 1 ) {

		emotion.Order = "MoveX";
		emotion.Action = " ";
		emotion.X = CENTER_FACE_ANGLE;
		face_yaw_position= CENTER_FACE_ANGLE;
		face_pub_.publish(emotion);

	}else if ( joy->buttons[13] == 1 ) {

		face_yaw_position+=5;
		if(face_yaw_position>MAX_FACE_ANGLE)
			face_yaw_position=MAX_FACE_ANGLE;

	}else if ( joy->buttons[14] == 1 ) {

		face_yaw_position-=5;
		if(face_yaw_position<MIN_FACE_ANGLE)
			face_yaw_position=MIN_FACE_ANGLE;

	}else if ( joy->buttons[12] == 1 ) {

		face_yaw_position=MIN_FACE_ANGLE;

	}else if ( joy->buttons[11] == 1 ) {

		face_yaw_position=MAX_FACE_ANGLE;

	}else if ( joy->axes[2] == -1 ) {
		if (joy->buttons[5] == 1){

		emotion.Order = "MoveX";
		emotion.Action = " ";
		emotion.X = 20;
		face_pub_.publish(emotion);
		ros::Duration(1).sleep();

		emotion.X = -40;
		face_pub_.publish(emotion);
		ros::Duration(1).sleep();

		}
	}

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "joystick_face");
	Joystick joystick;

	ros::spin();
}
