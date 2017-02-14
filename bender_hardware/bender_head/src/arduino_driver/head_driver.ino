#include <SerialDXL.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// SERVO DXL basic config
#define SERVO_MODEL 100
#define SERVO_FIRMWARE 100

/**
 * DEVICEDXL_MIN_BUFFER = 6
 * Is the overhead for dynamixel like devices
 * Has MODEL_L, MODEL_H, FIRMWARE, ID, BAUDRATE and RETURN DELAY
 * This variables are inside DeviceDXL class
 */ 

// We add 5 UInt8 (5 bytes)
#define SERVO_MMAP_SIZE DEVICEDXL_MIN_BUFFER+4

// MMap position for command
#define SERVO_SELECT_COMMAND DEVICEDXL_MIN_BUFFER
#define SERVO_POS_COMMAND DEVICEDXL_MIN_BUFFER+1
#define LED_SELECT_COMMAND DEVICEDXL_MIN_BUFFER+2
#define LED_COLOR_COMMAND DEVICEDXL_MIN_BUFFER+3
//#define LED_UPDATE_COLOR_COMMAND DEVICEDXL_MIN_BUFFER+4

// Number of servos in the array
#define num_servos 6

// Pin numbers on Arduino Mega to control the servo devices
#define pin_dir 3
#define pin_reset 2
#define pin_servo0 6	//left_ear
#define pin_servo1 7	//mouth
#define pin_servo2 8	//left_eyebrow
#define pin_servo3 9	//right_ear
#define pin_servo4 10	//right_eyebrow
#define pin_servo5 11	//neck

// Pin numbers on Arduino Mega to control the LED devices
#define data_led_pin1 5
#define data_led_pin2 4
#define pixelNumber 16

/**
 * @brief SERVO control using DXL communication protocol
 * @details SERVO control using Dynamixel communication protocol over RS485.
 * This implementation uses 2 uint8_t variable to control state of 5 SERVOs in address 6 and 7
 * of memory map (MMap).
 * 
 * @param dir_pin Toggle communication pin.
 * @param reset_pin Pin for reset device
 * @param servo_pin SERVO pin.
 */
class HeadDXL: public DeviceDXL
{
  public:
    //HeadDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t numServos, uint8_t servos_pins[], Servo *servos[]):
	HeadDXL(uint8_t dir_pin, uint8_t reset_pin, uint8_t numServos, uint8_t servos_pins[]):
    DeviceDXL(SERVO_MODEL, SERVO_FIRMWARE, SERVO_MMAP_SIZE), // Call parent constructor
    dir_pin_(dir_pin),    // Direction pin for RS485
    reset_pin_(reset_pin), // Reset pin
	numServos_(numServos), // numero de servos
	//servos_pins_(servos_pins),    // SERVOS pins
	//servos_(servos),			// Puntero a objeto Servo
    servo_select_command_(SERVO_SELECT_COMMAND, MMap::Access::RW, 0U, 7U, 0U), // Servo command 1
    servo_pos_command_(SERVO_POS_COMMAND, MMap::Access::RW, 0U, 180U, 0U), // Servo command 2
	led_select_command_(LED_SELECT_COMMAND, MMap::Access::RW, 0U, 254U, 0U), // array LEDs command 1
	led_color_command_(LED_COLOR_COMMAND, MMap::Access::RW, 0U, 254U, 0U) // array LEDs command 2
//	led_update_color_command_(LED_UPDATE_COLOR_COMMAND, MMap::Access::RW, 0U, 1U, 0U) // array LEDs command 3
    {
      // Config pins
      pinMode(dir_pin_, OUTPUT);
      pinMode(reset_pin_, OUTPUT);
	  for(uint8_t i=0;i<numServos_;i++)
	  {
      	pinMode(servos_pins[i], OUTPUT);
	  }
    }

    void init(uint8_t servos_pins[], Servo servos[], Adafruit_NeoPixel LEDs[])
    {
      DEBUG_PRINTLN("INIT");
      /*
       * Register variables
       */
      DeviceDXL::init(msgBuf_, varList_);
      mmap_.registerVariable(&servo_select_command_);
      mmap_.registerVariable(&servo_pos_command_);
      mmap_.registerVariable(&led_select_command_);
      mmap_.registerVariable(&led_color_command_);
//      mmap_.registerVariable(&led_update_color_command_);
      
      /*
       * Load default values
       */
      DEBUG_PRINTLN("Load default");
      mmap_.load(); // Load values from EEPROM
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(servo_select_command_.data);
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(servo_pos_command_.data);
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(led_select_command_.data);
      DEBUG_PRINT("data: ");DEBUG_PRINTLN(led_color_command_.data);
//      DEBUG_PRINT("data: ");DEBUG_PRINTLN(led_update_color_command_.data);
      
      /*
       * Read sensor data
       * e.g. Use ADC, check buttons, etc.
       */

	  /*
	   * Attach the Servo's variables to the servos_pins
	   */
	  for(uint8_t i=0;i<numServos_;i++)
	  {
	  	servos[i].attach(servos_pins[i]);
	  }
      // Settings for LED arrays
	  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
	  #if defined (__AVR_ATtiny85__)
		if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
	  #endif
	  // End of trinket special code
      for(uint8_t i=0;i<2;i++)
	  {
	  	LEDs[i].begin();
	    LEDs[i].setBrightness(5);
	    LEDs[i].show(); // Initialize all pixels to 'off'
	  }
    }
	void moveServoTo(Servo *servo, uint8_t pos)
	{
		servo->write(pos);
		delay(2);
	}
	void setPixelsTo(Adafruit_NeoPixel *LEDs_ring1, Adafruit_NeoPixel *LEDs_ring2, uint8_t R_colors[], uint8_t G_colors[], uint8_t B_colors[], uint8_t size)
	{
		//Serial.println(sizeof(pixelPositions));
		for(uint8_t i=0; i<size; i++)
		{
			LEDs_ring1->setPixelColor(i, R_colors[i], G_colors[i], B_colors[i]);
			LEDs_ring2->setPixelColor(i, R_colors[i+16], G_colors[i+16], B_colors[i+16]);
		}
		LEDs_ring1->show();
		delay(5);
		LEDs_ring2->show();
		delay(5);
		DEBUG_PRINTLN("LEDs_rings->show()");
	}
	uint8_t colorLevel(uint8_t level_code)
	{
		if(level_code==0) return 0;	//min component color value
		else if(level_code==1) return 102;
		else if(level_code==2) return 153;
		else if(level_code==3) return 255;	//max component color value
	}
	uint8_t getRColor(uint8_t rgb_code){ return colorLevel((rgb_code & 0b00110000)>>4);}
	uint8_t getGColor(uint8_t rgb_code){ return colorLevel((rgb_code & 0b00001100)>>2);}
	uint8_t getBColor(uint8_t rgb_code){ return colorLevel((rgb_code & 0b00000011)>>0);}

	void swapServo(Servo *servo, angle_min, angle_max)
	{
		for(uint8_t i=angle_min;i<angle_max;i++)
		{
		    servo->write(i);                  // sets the servo position according to the scaled value
		    delay(15);                           // waits for the servo to get there
		}
		for(uint8_t i=angle_max;i>angle_min;i--)
		{
		    servo->write(i);                  // sets the servo position according to the scaled value
		    delay(15);                           // waits for the servo to get there
		}
	}

	void updateServos(Servo servos[])
	{
		if (servo_select_command_.data == 6)	//when both values updated, change state
		{
			if (servo_select_ == 0) moveServoTo(&servos[0], servo_pos_);
			else if (servo_select_ == 1) moveServoTo(&servos[1], servo_pos_);
			else if (servo_select_ == 2) moveServoTo(&servos[2], servo_pos_);
			else if (servo_select_ == 3) moveServoTo(&servos[3], servo_pos_);
			else if (servo_select_ == 4) moveServoTo(&servos[4], servo_pos_);
			else if (servo_select_ == 5) moveServoTo(&servos[5], servo_pos_);
		}
		else if (servo_select_command_.data == 7)	//when both values updated, change state
		{
			swapServo(&servos[1], 115, 135);
			delay(15);
		}
		else	//update both values, selct and position
		{
			servo_select_ = servo_select_command_.data;
			servo_pos_ = servo_pos_command_.data;
		}
	}
	void updateLEDs(Adafruit_NeoPixel LEDs[])
	{
		if (led_select_command_.data == 0xFE){	//array of colors updated, ready to show
			setPixelsTo(&LEDs[0], &LEDs[1], R_colors_, G_colors_, B_colors_, 16); //show colors in LEDs
			//DEBUG_PRINTLN("show command (0xFE)");
		}
		else if (led_select_command_.data == 0xFD){	//updating array of colors
			R_colors_[led_to_change_] = getRColor(new_color_code_);
			G_colors_[led_to_change_] = getGColor(new_color_code_);
			B_colors_[led_to_change_] = getBColor(new_color_code_);
			/*DEBUG_PRINT("new_color_code_ = ");DEBUG_PRINTLN(new_color_code_);
			DEBUG_PRINT("r_color = ");DEBUG_PRINTLN(getRColor(new_color_code_));
			DEBUG_PRINT("g_color = ");DEBUG_PRINTLN(getGColor(new_color_code_));
			DEBUG_PRINT("b_color = ");DEBUG_PRINTLN(getBColor(new_color_code_));*/
			//DEBUG_PRINTLN("change color command (0xFD)");
		}
		/*else if (led_select_command_.data == 0xFC){	//Leds commands are not used in this case
			//DEBUG_PRINTLN("Led command non used");
		}*/
		else if (led_select_command_.data != 0xFC) //update last led number and color received
		{
			led_to_change_ = led_select_command_.data;
			new_color_code_ = led_color_command_.data;
			//DEBUG_PRINTLN("variables updated");
		}
	}
    void update(Servo servos[], Adafruit_NeoPixel LEDs[])
    {
		//DEBUG_PRINTLN("UPDATE");

		updateServos(servos);
		updateLEDs(LEDs);
    }
    void SetDefaulState(Servo servos[], Adafruit_NeoPixel LEDs[])
    {
    	//Default colors
		uint8_t R_colors_default[32] = {0,0,0,153,153,153,0,0,0,0,153,0,0,0,0,0,0,0,0,153,153,153,0,0,0,0,153,0,0,0,0,0};
		uint8_t G_colors_default[32] = {0,0,0,0,0,0,0,0,0,0,0,0,153,153,153,153,0,0,0,0,0,0,0,0,0,0,0,0,153,153,153,153};
		uint8_t B_colors_default[32] = {153,153,153,0,0,0,0,0,0,153,0,153,0,0,0,0,153,153,153,0,0,0,0,0,0,153,0,153,0,0,0,0};
		
    	moveServoTo(&servos[0], 50);
		moveServoTo(&servos[1], 120);
		moveServoTo(&servos[2], 100);
		moveServoTo(&servos[3], 120);
		moveServoTo(&servos[4], 100);
		moveServoTo(&servos[5], 100);
		setPixelsTo(&LEDs[0], &LEDs[1], R_colors_default, G_colors_default, B_colors_default, 16); //show colors in LEDs
    }

    inline bool onReset()
    {
      DEBUG_PRINTLN("ON RESET");
      return digitalRead(reset_pin_) == HIGH ? true : false;
    }

    inline void setTX()
    {
      digitalWrite(dir_pin_,HIGH);
    }

    inline void setRX()
    {
      digitalWrite(dir_pin_,LOW);
    }

  private:
    const uint8_t dir_pin_; // Toggle communication direction pin
    const uint8_t reset_pin_; // Reset pin
	const uint8_t numServos_; // Numero de servos
    //const uint8_t servos_pins_[5]; // SERVOs pins
	//Servo *servos_[5]; //puntero a objeto Servo
    
    // SERVOs variables
    MMap::UInt8 servo_select_command_;
    MMap::UInt8 servo_pos_command_;

    // LEDs variables
    MMap::UInt8 led_select_command_;
    MMap::UInt8 led_color_command_;
    
    // Memory map
    uint8_t msgBuf_[SERVO_MMAP_SIZE];
    MMap::VariablePtr varList_[SERVO_MMAP_SIZE];
	
	//LEDs colors
	uint8_t R_colors_[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t G_colors_[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t B_colors_[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t led_to_change_ = 0;
	uint8_t new_color_code_ = 0;
	
	//servos
	uint8_t servo_select_ = 0;
	uint8_t servo_pos_ = 0;
};

//Objetos para posicionar los servos (PWM)
Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

Servo servos[]={servo0, servo1, servo2, servo3, servo4, servo5};
uint8_t servos_pins[] = {pin_servo0, pin_servo1, pin_servo2, pin_servo3, pin_servo4, pin_servo5};

//Objetos para manejar los LEDs
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(pixelNumber, data_led_pin1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(pixelNumber, data_led_pin2, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel LEDs[] = {strip1, strip2};

//General device
HeadDXL head_dxl(pin_dir, pin_reset, num_servos, servos_pins);
SerialDXL serialDxl;

void setup() {
  //Serial port for debug
  Serial.begin(115200);
  delay(50);
  
  // Init serial communication using Dynamixel format
  serialDxl.init(115200, &Serial3 , &head_dxl);

  head_dxl.init(servos_pins, servos, LEDs);
  head_dxl.SetDefaulState(servos, LEDs);
  head_dxl.reset();
  head_dxl.mmap_.serialize();
}

void loop() {
  // Update msg buffer
  while (Serial3.available())
    serialDxl.process(Serial3.read());

  head_dxl.mmap_.deserialize();
  head_dxl.update(servos, LEDs);
  head_dxl.mmap_.serialize();
}
