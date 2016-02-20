
#ifndef BENDER_JOY_XBOX_JOY_H
#define BENDER_JOY_XBOX_JOY_H

#include <sensor_msgs/Joy.h>

namespace xbox_joy {


	/*# # # # # # # # # # # # # # # # # # # # # # # # # # #
	# Xbox Controller Button List:                        #
	# - - - - - - - - - - - - - - - - - - - - - - - - - - #
	# Axis:                                               #
	# 0.- LS left, right   # 1.- LS up, down   # 2.- LT   #
	# 3.- RS left, right   # 4.- RS up, down   # 5.- RT   #
	#                                                     #
	# Buttons:                                            #
	# 0.- A   # 5.- RB     # 10.- RS                      #
	# 1.- B   # 6.- Back   # 11.- Left                    #
	# 2.- X   # 7.- Start  # 12.- Right                   #
	# 3.- Y   # 8.- Xbox   # 13.- Up                      #
	# 4.- LB  # 9.- LS     # 14.- Down                    #
	# # # # # # # # # # # # # # # # # # # # # # # # # # #*/

	// - - - - BUTTON DEFINITIONS - - - -
	const uint16_t _A_       = 1<<0;
	const uint16_t _B_       = 1<<1;
	const uint16_t _X_       = 1<<2;
	const uint16_t _Y_       = 1<<3;
	const uint16_t _LB_      = 1<<4;
	const uint16_t _RB_      = 1<<5;
	const uint16_t _BACK_    = 1<<6;
	const uint16_t _START_   = 1<<7;
	const uint16_t _XBOX_    = 1<<8;
	const uint16_t _LS_      = 1<<9;
	const uint16_t _RS_      = 1<<10;
	const uint16_t _LEFT_    = 1<<11;
	const uint16_t _RIGHT_   = 1<<12;
	const uint16_t _UP_      = 1<<13;
	const uint16_t _DOWN_    = 1<<14;

	// - - - - AXES NUMBERS - - - -
	const uint16_t _LS_HORZ_ = 0;
	const uint16_t _LS_VERT_ = 1;
	const uint16_t _LT_      = 2;
	const uint16_t _RS_HORZ_ = 3;
	const uint16_t _RS_VERT_ = 4;
	const uint16_t _RT_      = 5;

	/**
	 * Displays <x> as a 16 bit chain
	 */
	void show_button_mask(uint16_t x) {

		for(int i=(sizeof(uint16_t)*8)-1; i>=0; i--) {
			(x&(1<<i))?putchar('1'):putchar('0');
		}
		printf("\n");
	}

	uint16_t get_button_mask(const sensor_msgs::Joy::ConstPtr& joy) {

		uint16_t mask = 1;
		uint16_t result = 0;

		//printf("\n----\n");
		for (int i = 0; i < joy->buttons.size(); i++) {

			//printf("mask:   "); show_button_mask(mask);

			if (joy->buttons[i]) {
				result |= mask;
			}
			mask <<= 1;

		}
		//printf("result: "); show_button_mask(result);
		return result;
	}
}
#endif
