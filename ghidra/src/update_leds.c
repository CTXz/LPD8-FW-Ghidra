#define LED_PAD_1_GPIO 5
#define LED_PAD_2_GPIO 6
#define LED_PAD_3_GPIO 7
#define LED_PAD_4_GPIO 8
#define LED_PAD_5_GPIO 9
#define LED_PAD_6_GPIO 10
#define LED_PAD_7_GPIO 11
#define LED_PAD_8_GPIO 12
#define LED_PB_PAD_GPIO 13
#define LED_PB_PROG_CHNG_GPIO 14
#define LED_PB_CC_GPIO 15

#define UNKNOWN_FLAG_0_ADDR 0x20000011	      // uint8
#define UNKNOWN_ENUM_0_ADDR 0x20000012	      // unknown
#define PREV_MODE_0_ADDR 0x20000031	      // uint8
#define PREV_SELECTED_PROG_ADDR 0x20000032    // uint8
#define PREV_PAD_STATES_ARRAY_ADDR 0x20000033 // uint8[8]
// ... 0x2000003a
#define SELECTED_MODE_ADDR 0x2000003f // uint8
// potentially more "current" fields here
#define SELECTED_PROG_ADDR 0x20000042 // uint8
#define PREV_MODE_1_ADDR 0x20000043   // uint8
// 0x20000047 could be PREV_PROG_ADDR

#define PAD_STATES_ARRAY_ADDR 0x20000552 // pad_states[8]

#define PROG_4_SELECT_FLAG 0x2000050c		    // uint8_t
#define PROG_3_SELECT_FLAG PROG_4_SELECT_FLAG - 0x6 // uint8_t
#define PROG_2_SELECT_FLAG PROG_3_SELECT_FLAG - 0x6 // uint8_t
#define PROG_1_SELECT_FLAG PROG_2_SELECT_FLAG - 0x6 // uint8_t

#include <stm32f103xb.h>
#include <stdint.h>

typedef unknown void; // So the linter doesn't complain

typedef pad_state_t uint8_t;
#define PAD_STATE_RELEASED 0
#define PAD_STATE_PRESSED 1
#define PAD_STATE_UNKNOWN 0xFF

typedef enum
{
	MODE_PROG,
	MODE_PAD,
	MODE_CC,
	MODE_PROG_CHNG
} modes;

// These appear to be updated on on-push and on-release events only
typedef struct
{
	pad_state_t unknown0;
	pad_state_t unknown1;
	pad_state_t prog_chng; // offset: 2
	pad_state_t pad;       // offset: 3
	pad_state_t cc;	       // offset: 4
} pad_states;

/**
 * @ 0x08003420
 *
 */
void update_pad_leds()
{
	const uint8_t selected_mode = *(uint8_t *)SELECTED_MODE_ADDR; // uint8_t
	const uint8_t prev_mode = *(uint8_t *)PREV_MODE_0_ADDR;	      // uint8_t

	pad_state_t *prev_pd_states = PREV_PAD_STATES_ARRAY_ADDR; // -> pad_state_t[8]
	pad_states *pd_states = PAD_STATES_ARRAY_ADDR;		  // -> pad_states[8]

	// For every Pad
	for (uint8_t i = 0; i < 8; i++)
	{
		pad_state_t pd_state;

		switch (selected_mode)
		{
		case MODE_PAD:
			pd_state = pd_states[i].pad;
			break;
		case MODE_CC:
			pd_state = pd_states[i].cc;
			break;
		case MODE_PROG_CHNG:
			pd_state = pd_states[i].prog_chng;
			break;
		default:
			for (uint8_t j = 0; j < 8; j++)
			{
				prev_pd_states[j] = PAD_STATE_UNKNOWN;
			}

			return; // PROGRAM mode already deals with leds in update_leds
		}

		// Only update pad LED if pad_state has changed or mode has changed
		if (prev_pd_states[j] != pd_state && *prev_mode != selected_mode)
		{
			prev_pd_states[j] = pd_state;
			*prev_mode = selected_mode;

			if (pd_state == PAD_STATE_PRESSED)
			{
				switch (pad)
				{
				case 0:
					GPIOB->ODR |= (1 << LED_PAD_1_GPIO);
					break;
				case 1:
					GPIOB->ODR |= (1 << LED_PAD_2_GPIO);
					break;
				case 2:
					GPIOB->ODR |= (1 << LED_PAD_3_GPIO);
					break;
				case 3:
					GPIOB->ODR |= (1 << LED_PAD_4_GPIO);
					break;
				case 4:
					GPIOB->ODR |= (1 << LED_PAD_5_GPIO);
					break;
				case 5:
					GPIOB->ODR |= (1 << LED_PAD_6_GPIO);
					break;
				case 6:
					GPIOB->ODR |= (1 << LED_PAD_7_GPIO);
					break;
				case 7:
					GPIOB->ODR |= (1 << LED_PAD_8_GPIO);
					break;
				default:
					break;
				}
			}
			else
			{
				switch (pad)
				{
				case 0:
					GPIOB->ODR &= ~(1 << LED_PAD_1_GPIO);
					break;
				case 1:
					GPIOB->ODR &= ~(1 << LED_PAD_2_GPIO);
					break;
				case 2:
					GPIOB->ODR &= ~(1 << LED_PAD_3_GPIO);
					break;
				case 3:
					GPIOB->ODR &= ~(1 << LED_PAD_4_GPIO);
					break;
				case 4:
					GPIOB->ODR &= ~(1 << LED_PAD_5_GPIO);
					break;
				case 5:
					GPIOB->ODR &= ~(1 << LED_PAD_6_GPIO);
					break;
				case 6:
					GPIOB->ODR &= ~(1 << LED_PAD_7_GPIO);
					break;
				case 7:
					GPIOB->ODR &= ~(1 << LED_PAD_8_GPIO);
					break;
				default:
					break;
				}
			}
		}
	}
}

/**
 *  @ 0x08003da8
 *
 * The following function is responsible for
 * handling/updating the LEDs of the LPD8.
 * This includes:
 * 	- Setting the LEDs of the mode pb pads upon mode change
 * 	- Holding the pad LEDs while selecting the program in PROGRAM mode
 * 	- Calling update_pad_leds which specifically handles the momentary
 * 	  LED toggles when Pads are pad_state and released
 *
 */
void update_leds(void)
{
	const uint8_t unknown_flag = *(uint8_t *)UNKNOWN_FLAG_0_ADDR; // uint8_t
	const uint8_t selected_mode = *(uint8_t *)SELECTED_MODE_ADDR; // uint8_t
	const int unknown_enum = *(int *)UNKNOWN_ENUM_0_ADDR;	      // unknown

	uint8_t *prev_mode = PREV_MODE_1_ADDR;		       // -> uint8_t
	uint8_t *selected_prog = SELECTED_PROG_ADDR;	       // -> uint8_t
	uint8_t *prev_selected_prog = PREV_SELECTED_PROG_ADDR; // -> uint8_t
	pad_states *pd_states = PAD_STATES_ARRAY_ADDR;	       // -> pad_states[8]

	// I have yet to find out what sets this flag != 0
	if (unknown_flag == 0)
	{
		/* Check if mode has been switched */
		if (*prev_mode != selected_mode)
		{
			// Clear PB Pad LEDs
			GPIOB->ODR &= ~(1 << LED_PB_PAD_GPIO);
			GPIOB->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
			GPIOB->ODR &= ~(1 << LED_PB_CC_GPIO);

			/*
			 * Resets the 'prog_chng' fields of the 'pad_states' array to prevent Pad LEDs from
			 * getting stuck in ON state during mode switching.
			 *
			 * The 'pad_states' fields update only during on-push and on-release events. This
			 * prevents the LED from becoming stuck in ON mode when re-entering into PROGRAM
			 * CHANGE mode after exiting PROGRAM CHANGE mode while one or more pads were being held.
			 *
			 * The fact that this is limited to the 'prog_chng' field is rather odd. Initially I suspected
			 * that this was intended as CC's and NOTE (Pad mode) messages are send in a pairs of two
			 * (Pad: Push: Note ON and OFF, CC: Push: CC with val 0-127 Release: CC with val 0).
			 * Contrary Program Change messages are send as a single message only (Push: Program Change 0-8,
			 * Release: No message). A stuck LED would thus correspond to an unterminated NOTE or CC message.
			 * However, this contradicts the MIDI output, which does not update/change the message type after
			 * a mode switch until the pad is released.
			 *
			 * There are two possible ways to "fix" this:
			 * 	1. Set all pad_states fields to 0 on mode switch, not just the 'prog_chng' field
			 * 	or
			 * 	2. Change the MIDI logic to behave like the LED logic
			 */
			for (uint8_t i = 0; i < 8; i++)
			{
				pd_states[i].prog_chng = 0;
			}
			*prev_mode = selected_mode;
		}

		// Set LEDs according to selected mode
		switch (selected_mode)
		{
		case MODE_PAD:
			GPIOB->ODR |= (1 << LED_PB_PAD_GPIO);
			break;
		case MODE_CC:
			GPIOB->ODR |= (1 << LED_PB_CC_GPIO);
			break;
		case MODE_PROG_CHNG:
			GPIOB->ODR |= (1 << LED_PB_PROG_CHNG_GPIO);
			break;
		case MODE_PROG:
			// Clear all Pad LEDs
			GPIOB->ODR &= ~(1 << LED_PAD_1_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_2_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_3_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_4_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_5_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_6_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_7_GPIO);
			GPIOB->ODR &= ~(1 << LED_PAD_8_GPIO);

			if ((uint8_t *)PROG_1_SELECT_FLAG == 1)
				*selected_prog = 1;
			else if ((uint8_t *)PROG_2_SELECT_FLAG == 1)
				*selected_prog = 2;
			else if ((uint8_t *)PROG_3_SELECT_FLAG == 1)
				*selected_prog = 3;
			else if ((uint8_t *)PROG_4_SELECT_FLAG == 1)
				*selected_prog = 4;

			if (*prev_selected_prog != *selected_prog)
			{
				FUN_0800442c();

				// Set some fields in the some struct to 0
				for (uint8_t i = 0; i < 8; i++)
				{
					uint8_t *start = PAD_STATES_ARRAY_ADDR - 0x28;
					size_t offset = i * 5;
					start[offset] = 0;

					pd_states[i].unknown0 = 0;
					pd_states[i].unknown1 = 0;
					pd_states[i].prog_chng = 0;
					pd_states[i].pad = 0;
					pd_states[i].cc = 0;
				}
			}

			switch (*selected_prog)
			{
			case 1:
				GPIOB->ODR |= (1 << LED_PAD_1_GPIO);
				break;
			case 2:
				GPIOB->ODR |= (1 << LED_PAD_2_GPIO);
				break;
			case 3:
				GPIOB->ODR |= (1 << LED_PAD_3_GPIO);
				break;
			case 4:
				GPIOB->ODR |= (1 << LED_PAD_4_GPIO);
				break;
			default:
				break;
			}

		default:
			break;
		}

		update_pad_leds();
	}
	else
	{
		/* Clear all LEDs */
		GPIOB->ODR &= ~(1 << LED_PB_PAD_GPIO);
		GPIOB->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
		GPIOB->ODR &= ~(1 << LED_PB_CC_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_1_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_2_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_3_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_4_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_5_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_6_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_7_GPIO);
		GPIOB->ODR &= ~(1 << LED_PAD_8_GPIO);

		switch (unknown_enum)
		{
		case 1:
			GPIOB->ODR |= (1 << LED_PB_PAD_GPIO);
			break;
		case 2:
			GPIOB->ODR |= (1 << LED_PB_CC_GPIO);
			break;
		case 3:
			GPIOB->ODR |= (1 << LED_PB_PROG_CHNG_GPIO);
			break;
		case 4:
			GPIOB->ODR |= (1 << LED_PAD_1_GPIO);
			break;
		case 5:
			GPIOB->ODR |= (1 << LED_PAD_2_GPIO);
			break;
		case 6:
			GPIOB->ODR |= (1 << LED_PAD_3_GPIO);
			break;
		case 7:
			GPIOB->ODR |= (1 << LED_PAD_4_GPIO);
			break;
		case 8:
			GPIOB->ODR |= (1 << LED_PAD_5_GPIO);
			break;
		case 9:
			GPIOB->ODR |= (1 << LED_PAD_6_GPIO);
			break;
		case 10:
			GPIOB->ODR |= (1 << LED_PAD_7_GPIO);
			break;
		case 11:
			GPIOB->ODR |= (1 << LED_PAD_8_GPIO);
			break;
		default:
			break;
		}
	}
	*prev_selected_prog = *selected_prog;
}