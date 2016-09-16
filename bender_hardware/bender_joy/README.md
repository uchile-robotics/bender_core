# bender_joy

## Arquitectura del sistema de teleoperación




## Uso

Sólo puede haber 1 Adaptador Wireless de joystick por PC. Cada adaptador soporta hasta 4 joysticks.

```sh
# Por cada pc con un adaptador
roslaunch bender_joy driver.launch

# Nodos de control. Lanzar sólo 1 vez.
roslaunch bender_joy joy_interface.launch
```

## TODO

- elimino el joy number
- joy interaction mouth off
- joy head check


# TODO: n channels
# TODO: default
#    force a default channel at start
#    default_channel: 0
# TODO: decir el tópico, nombre de config y botón que quedó configurado.


void  Joystick::synthesize(std::string text){
	bender_srvs::String speech_text;
	std::string talk = text;
	std::string text_evaluate = text;

	std::size_t found_enter,found_emotion;
	bool end = false;
	std::string emo;
	while(!end){
		found_enter = text_evaluate.find("//");//Enter
		found_emotion = text_evaluate.find("@");//Enter
		talk = text_evaluate;
		if (found_enter!=std::string::npos && found_enter<found_emotion){
			talk =  text_evaluate.substr (0,found_enter);
			text_evaluate = text_evaluate.substr (found_enter+3);
		}
		if (found_emotion!=std::string::npos && found_emotion<found_enter){
			talk =  text_evaluate.substr (0,found_emotion);

			std::size_t found_space = text_evaluate.find(" ",found_emotion);
			if (found_space!=std::string::npos){
				emo = text_evaluate.substr (found_emotion+1,found_space - found_emotion -1);
				text_evaluate = text_evaluate.substr (found_space);
			}else{
				emo = text_evaluate.substr (found_emotion+1);
				text_evaluate = text_evaluate.substr (found_emotion);
				end=true;
			}
			show_emotion(emo);
		}
		if (found_enter==std::string::npos && found_emotion==std::string::npos ) end=true;
		speech_text.request.data = talk;
		speech_serv_.call(speech_text);
	}

}

