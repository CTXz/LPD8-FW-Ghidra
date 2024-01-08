#include <stdint.h>
#include <stdbool.h>

#include <stm32f103xb.h>
#include <stm32f101xe.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_usb.h>

#define RCC_CFGR_USBPRE_BB (uint32_t *)(PERIPH_BB_BASE + (RCC_CFGR_OFFSET_BB * 32) + (RCC_CFGR_USBPRE_Pos * 4))
#define RCC_CR_PLLON_BB (uint32_t *)(PERIPH_BB_BASE + (RCC_CR_OFFSET_BB * 32) + (RCC_CR_PLLON_Pos * 4))

#define CONST_UINT8_8_PTR_LUT_8TO15_080056EC 0x080056EC	  // = [8, 9, 10, 11, 12, 13, 14, 15]
#define CONST_UINT8_127_PTR_LUT_1TO127_080056E4 0x080056E4	  // = [1, 2, ... 126, 127, 127]
#define CONST_UINT8_PTR_PRE_HPRE_TO_RSHFT_080056DC 0x080056DC // Converts PRE and HPRE values to right shifts for clock division
#define CONST_CHAR_PTR_UNKNOWN_0x8005871 0x8005871

/**
 * /////////////////////////////////////////////////
 * // NVIC
 * /////////////////////////////////////////////////
 *
 * The LPD8 has two vector tables:
 * 	- One for the bootloader, used for firmware updates.
 * 	  The bootloader can be entered by holding PROGRAM on boot
 * 	- One for the firmware
 * The vector tables are located at:
 * 	- 0x08000000 for the bootloader
 * 	- 0x08002000 for the firmware
 *
 * ISR's for the bootloader are prefixed with ISR_BL_,
 * ISR's for the firmware are prefixed with ISR_FW_
 */
#define NVIC_FIRMWARE_VECTOR_TABLE_OFFSET 0x2000
#define UINT32_PTR_NVIC_VETOR_TABLE_BASE 0x08000000
#define UINT32_PTR_NVIC_FIRMWARE_VECTOR_TABLE_OFFSET_MASK 0x1FFFFF80
#define UINT32_PTR_NVIC_PRIO_BASE 0xE000E400

#define UINT8_PTR_UNKNOWN_FLAG_0x20000000 0x20000000
#define UINT8_PTR_MIDI_BUFFER_REMAINING_SPACE_0x20000004 0x20000004
#define UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008 0x20000008
#define UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c 0x2000000c // buffer_end - MIDI_BUFFER_SIZE
#define UINT8_PTR_UNKNOWN_FLAG_0x20000011 0x20000011
#define UNKNOWN_ENUM_PTR_0x20000012 0x20000012
#define UNKNOWN_PTR_0x20000014 0x20000014
#define UINT8_PTR_SYSTICK_DECREMENTER_0_0x20000018 0x20000018 // Decrements on every SysTick Interrupt
#define BOOL_8_PTR_SCALED_KNOB_VALS_CHANGED_0x20000019 0x20000019
#define UINT8_8_PTR_PREV_SCALED_KNOB_VALS_0x20000021 0x20000021
#define UINT8_8_PTR_PREV_PREV_SCALED_KNOB_VALS_0x20000029 0x20000029
#define UINT8_PTR_PREV_MODE_0x20000031 0x20000031
#define UINT8_PTR_PREV_SELECTED_PROG_0x20000032 0x20000032
#define UINT8_8_PTR_PREV_PADS_STATE_0x20000033 0x20000033
#define UINT8_PTR_SYSTICK_DECREMENTER_1_0x2000003c 0x2000003c // Decrements on every SysTick Interrupt
#define UINT8_PTR_DMA1_STORE_CNTR_0x2000003d 0x2000003d
#define UINT8_PTR_DEBOUNCE_COUNTER_0x2000003e 0x2000003e
#define UINT8_PTR_SELECTED_MODE_0x2000003f 0x2000003f
#define UINT8_PTR_KNOB_ANALOG_VALS_READY_0x20000040 0x20000040
#define UINT8_PTR_PAD_ANALOG_VALS_READY_0x20000041 0x20000041
#define UINT8_PTR_SELECTED_PROG_0x20000042 0x20000042
#define UINT8_PTR_PREV_MODE_0x20000043 0x20000043
#define UINT16_PTR_PREV_MODE_PB_IDR_BITS_0x20000044 0x20000044
#define UINT16_PTR_PREV_MODE_PB_IDR_BITS_0x20000046 0x20000046
#define UINT16_PTR_MODE_PB_IDR_BITS_CPY_0x20000048 0x20000048
#define UINT16_PTR_USB_SOF_COUNTER_0x2000004C 0x2000004C // Start-Of-Frame Counter: Increments every 1ms
#define UINT32_PTR_UNKNOWN_0x2000004D 0x2000004D
#define UINT16_PTR_UNKNOWN_0x20000054 0x20000054
#define UINT16_PTR_USB_ISTR_CPY_0x20000056 0x20000056
#define UINT32_PTR_USB_CNTR_MASK_0x20000094 0x20000094
#define UINT32_PTR_PTR_UNKNOWN_0x20000098 0x20000098
#define UNKNOWN_PTR_PTR_PTR_0x2000009C 0x2000009C
#define UNKNOWN_PTR_0x200000A0 0x200000A0
#define UNKNOWN_PTR_PTR_200000E8 0x200000E8
#define FPTR_PTR_FW_USB_RESET_CALLBACK_200000EC 0x200000EC
#define UINT32_PTR_UNKNOWN_0x20000118 0x20000118
#define UINT16_16_PTR_DMA1_MEM_0x2000013C 0x2000013C
#define UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c 0x2000015c
#define UINT8_PTR_MIDI_BUFFER_END_0x2000024C 0x2000024C
#define PROGRAM_SETTINGS_5_PTR_0x200003CC 0x200003CC
#define UINT16_8_PTR_PREV_ACCEPTED_KNOB_ANALOG_VALS_0x200004EA 0x200004EA
#define PAD_HANDLING_DATA_8_PTR_0x200004FA 0x200004FA
#define PENDING_MIDI_8_PTR_RELEASE_MIDI_0x2000052a 0x2000052a
#define PAD_STATES_8_PTR_0x20000552 0x20000552
#define UINT16_16_PTR_PREV_DMA1_MEM_0x2000057A 0x2000057A
#define FILTERED_ANALOG_INPUTS_PTR_0x2000059A 0x2000059A
#define UINT32_PTR_UNKNOWN_200005BC 0x200005BC

// TODO Examine data structure
#define PROG_4_SELECT_FLAG 0x2000050c		    // uint8_t
#define PROG_3_SELECT_FLAG PROG_4_SELECT_FLAG - 0x6 // uint8_t
#define PROG_2_SELECT_FLAG PROG_3_SELECT_FLAG - 0x6 // uint8_t
#define PROG_1_SELECT_FLAG PROG_2_SELECT_FLAG - 0x6 // uint8_t

#define LED_GPIO_PORT GPIOB
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
#define GPIOA_USB_FULL_SPEED_PU GPIO_PIN_8

#define PB_GPIO_PORT GPIOC
#define PB_PROG_GPIO 6
#define PB_PAD_GPIO 7
#define PB_PROG_CHNG_GPIO 8
#define PB_CC_GPIO 9
#define PB_IDR_MSK (1 << PB_PROG_GPIO) | (1 << PB_PAD_GPIO) | (1 << PB_PROG_CHNG_GPIO) | (1 << PB_CC_GPIO)

#define ADC_CR1_OFFSET 0x04
#define ADC_CR2_OFFSET 0x08
#define ADC_SMPR1_OFFSET 0x0c
#define ADC_SMPR2_OFFSET 0x10
#define ADC_SQR1_OFFSET 0x2c
#define ADC_SQR2_OFFSET 0x30
#define ADC_SQR3_OFFSET 0x34
#define GPIO_CRL_OFFSET 0x00
#define GPIO_CRH_OFFSET 0x04
#define GPIO_BSRR_OFFSET 0x10
#define GPIO_BRR_OFFSET 0x14
#define GPIO_CR_CNF_MODE 0x0F

#define IWDG_RL_VAL 1250
#define DMA1_NUM_DATA 16
#define DMA1_STORE_INTERVAL 4
#define ADC_N_CONV 0x10
#define SYSTICK_RELOAD_AHB_DIV 8000
#define MIDI_MAX_CHANNEL 15   // 0xF
#define MIDI_MAX_DATA_VAL 127 // 0x7F
#define MIDI_BUFFER_SIZE 240  // 0xF0
#define MODE_PB_SYSTICK_SCAN_INTERVALS 4
#define MODE_PB_DEBOUNCE_THRESHOLD 10
#define MODE_PB_DEBOUNCE_COUNT 240
#define ADC_KNOB_CHANGE_THRESHOLD 7
#define ADC_KNOB_NOISE_GATE 15 // 0x0F
#define MAX_ADC_KNOB_VAL 0x3FF
#define MAX_ADC_PAD_VAL 0x2A0
#define PAD_analog_CONSIDERED_RELEASED 64
#define PAD_analog_CONSIDERED_PRESSED MIDI_MAX_DATA_VAL + 2
#define PAD_PRESS_INCR_COMPLETE 4
#define PAD_RELEASE_DECR_START 30
#define PAD_MODE_RELEASE_DATA2 MIDI_MAX_DATA_VAL
#define CC_MODE_RELEASE_DATA2 0x00
#define PROG_CHNG_MODE_RELEASE_DATA2 0x00
#define PROG_CHNG_MODE_PRESS_DATA2 0x00
#define N_KNOBS 8
#define N_PADS 8
#define N_KNOBSOR_PADS 8

// #define EXCEEDS_THRESHOLD(x, y, threshold) ((x) < (y) + (threshold) || (y) < (x) + (threshold))
#define EXCEEDS_THRESHOLD(x, y, threshold) ((x) + (threshold) <= (y) || (y) + (threshold) <= (x))

typedef uint32_t unknown; // So the linter doesn't complain
typedef void (*fptr)(void);


typedef uint8_t midi_data_t;	     // Only 7 bits are used
typedef uint16_t filtered_adc_val_t; // For ADC vals downscaled to 10 bits
typedef uint32_t systick_reload_t;   // Only 3 Bytes (24 bits) are used
typedef uint32_t bool32_t;

typedef uint32_t SWS_t;
#define SWS_HSI 0x0
#define SWS_HSE 0x1
#define SWS_PLL 0x2
#define SWS_INVALID 0x3

typedef uint8_t pad_state_t;
#define PAD_STATE_RELEASED 0
#define PAD_STATE_PRESSED 1
#define PAD_STATE_UNSET 0xFF

typedef uint8_t midi_cmd_msb_t;
#define MIDI_CMD_NOTE_OFF_MSB 0x8
#define MIDI_CMD_NOTE_ON_MSB 0x9
#define MIDI_CMD_CC_MSB 0xB
#define MIDI_CMD_PROG_CHNG_MSB 0xC

typedef uint8_t mode_t;
#define MODE_PROG 0x0
#define MODE_PAD 0x1
#define MODE_CC 0x2
#define MODE_PROG_CHNG 0x3
#define MODE_UNSET 0xFF

typedef uint32_t gpio_operation_t;
#define GPIO_CFG_OP_IN_ANALOG 0x00
#define GPIO_CFG_OP_IN_PD 0x28
#define GPIO_CFG_OP_IN_PU 0x48
#define GPIO_CFG_OP_OTHER 0x10

typedef uint8_t push_setting_t;
#define MOMENTARY 0x0
#define TOGGLE 0x1

typedef uint8_t ready_t;
#define NOT_READY 0x0
#define READY 0x1

typedef uint8_t pad_confirmed_t;
#define CONFIRMED_RELEASED 0x0
#define CONFIRMED_PRESSED 0x1
#define UNCHANGED 0x2

typedef uint8_t pad_toggled_t;
#define TOGGLED_OFF 0x0
#define TOGGLED_ON 0x1

typedef uint8_t midi_pending_t;
#define NOT_PENDING 0x0
#define PENDING 0x1

typedef int32_t set_SYSTICK_param_t;
#define SYSTICK_RESET_VAL 0x0
#define SYSTICK_ENABLE 0x1
#define SYSTICK_DISABLE 0x2

typedef uint8_t GPIO_CR_CNF_MODE_t;
#define GPIO_CR_CNF_MODE_INPUT_ANALOG 0b0000
#define GPIO_CR_CNF_MODE_INPUT_FLOATING 0b0100
#define GPIO_CR_CNF_MODE_INPUT_PULL_UP_DOWN 0b1000
#define GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_10MHZ 0b0001
#define GPIO_CR_CNF_MODE_OUTPUT_OPEN_DRAIN_10MHZ 0b0101
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_PUSH_PULL_10MHZ 0b1001
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_OPEN_DRAIN_10MHZ 0b1101
#define GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_2MHZ 0b0010
#define GPIO_CR_CNF_MODE_OUTPUT_OPEN_DRAIN_2MHZ 0b0110
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_PUSH_PULL_2MHZ 0b1010
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_OPEN_DRAIN_2MHZ 0b1110
#define GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_50MHZ 0b0011
#define GPIO_CR_CNF_MODE_OUTPUT_OPEN_DRAIN_50MHZ 0b0111
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_PUSH_PULL_50MHZ 0b1011
#define GPIO_CR_CNF_MODE_OUTPUT_ALT_OPEN_DRAIN_50MHZ 0b1111

// Size = 16
typedef struct {
	uint32_t system_clock;
	uint32_t ahb_clock;
	uint32_t apb1_clock;
	uint32_t apb2_clock;
} clock_freqs;

// Size = 9
typedef struct {
	uint32_t gpios;
	GPIO_CR_CNF_MODE_t other_cr_bits; // CNF + MODE, only used if op == GPIO_CFG_OP_OTHER
	gpio_operation_t op;
} gpio_cfg;

// Size = 4
typedef struct {
	filtered_adc_val_t knobs[N_KNOBS];
	filtered_adc_val_t pads[N_PADS];
} filtered_analog_inputs;

// Size = 5
typedef struct {
	midi_pending_t pending;
	midi_cmd_msb_t cmd_msb;
	uint8_t cmd;
	uint8_t data1;
	uint8_t data2;
} pending_midi;

// These appear to be updated on on-push and on-release events only
// Size = 5
typedef struct {
	pad_toggled_t pad_toggled;
	pad_toggled_t cc_toggled;
	pad_state_t prog_chng; // offset: 2
	pad_state_t pad;       // offset: 3
	pad_state_t cc;	       // offset: 4
} pad_states;

// Size = 4
typedef struct {
	uint8_t note;
	uint8_t pc;
	uint8_t cc;
	push_setting_t type;
} pad_settings;

// Size = 6
typedef struct {
	pad_confirmed_t last_confirmed_state;
	uint8_t press_incr;
	uint8_t rel_decr;
	uint8_t unknown0;
	uint16_t adc_eval;
} pad_handling_data;

// Size = 3
typedef struct {
	uint8_t cc;
	uint8_t leftmost;
	uint8_t rightmost;
} knob_settings;

// Size = 57
typedef struct {
	uint8_t midi_ch;
	pad_settings pads[N_PADS];
	knob_settings knobs[N_KNOBS];
} program_settings;

/**
 * @ 0x08004e7c, Called by Firmware IVT Entry @ 0x0800203c
 * Progress: DONE
 */
void ISR_FW_SysTick_Handler(void)
{
	uint8_t *systick_decr_0 = UINT8_PTR_SYSTICK_DECREMENTER_0_0x20000018;
	uint8_t *systick_decr_1 = UINT8_PTR_SYSTICK_DECREMENTER_1_0x2000003c;

	if (*systick_decr_0 != 0)
		*systick_decr_0 = *systick_decr_0 - 1;

	if (*systick_decr_1 != 0)
		*systick_decr_1 = *systick_decr_1 - 1;
}

/**
 * @ 0x08005518
 * Progress: ALMOST DONE
 * TODO: Make inline with two params
 */
void nvic_init_firmware_VTOR(void)
{
	const uint32_t base = UINT32_PTR_NVIC_VETOR_TABLE_BASE;
	const uint32_t offset = NVIC_FIRMWARE_VECTOR_TABLE_OFFSET;
	SCB->VTOR = base | (offset & UINT32_PTR_NVIC_FIRMWARE_VECTOR_TABLE_OFFSET_MASK);
}

/**
 * @ 0x08004094
 * Status: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm if SetPriority is indeed the correct function
 */
inline void _NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
	NVIC_SetPriority(IRQn, priority);
}

/**
 * @ 0x08004ee8
 * Progress: DONE
 */
void SYSTICK_set_RELOAD(systick_reload_t reload)
{
	SysTick->LOAD = reload;
}

/**
 * @ 0x08004ea6
 * Progress: DONE
 */
void SYSTICK_set_TICKINT(bool enable)
{
	if (enable)
		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	else
		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

/**
 * @ 0x08004e80
 * Progress: DONE
 * See https://developer.arm.com/documentation/dui0552/a/cortex-m3-peripherals/system-timer--systick?lang=en
 */
void SYSTICK_action(set_SYSTICK_param_t action)
{
	switch (action) {
	case SYSTICK_ENABLE:
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		break;
	case SYSTICK_DISABLE:
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
		break;
	default:
		SysTick->VAL = 0;
		break;
	}
}

/**
 * @0x080044ec
 * Progress: DONE
 */
void RCC_set_APB2ENR(uint32_t msk, bool set)
{
	if (set)
		RCC->APB2ENR |= msk;
	else
		RCC->APB2ENR &= ~msk;
}

/**
 * @0x08004108
 * Progress: DONE
 */
void AIRCR_set(uint32_t msk)
{
	SCB->AIRCR = SCB_AIRCR_VECTKEY_Msk | msk;
}

/**
 * @0x08005160
 * Progress: DONE
 */
void USB_FS_PU_set(bool set)
{
	if (set)
		GPIOA->BSRR = GPIO_BSRR_BS8;
	else
		GPIOA->BRR = GPIO_BRR_BR8;
}


/**
 * @ 0x08004558
 * Progress: DONE
 * See P.99, Section 7.3 - RCC Register in RM0008
 */
void RCC_get_clock_freqs(clock_freqs *cfreqs)
{
	//// Selected System Clock Frequency

	const SWS_t SWS = RCC->CFGR & RCC_CFGR_SWS;

	/* The original code does not normalize the SWS bits
	 * (i.e. it does not shift them to the LSB), hence the
	 * akward shifts by two in the case statements */

	switch (SWS) {
	// High Speed External Clock
	case (SWS_HSE << 2):
		cfreqs->system_clock = HSE_VALUE;
		break;

	// Phase Locked Loop
	case (SWS_PLL << 2):
		const uint32_t PLLMUL = RCC->CFGR & RCC_CFGR_PLLMULL;
		const uint32_t PLLMUL_normed = PLLMUL >> RCC_CFGR_PLLMULL_Pos;
		const uint32_t PLLSRC = RCC->CFGR & RCC_CFGR_PLLSRC;
		const uint32_t PLLXTPRE = RCC->CFGR & RCC_CFGR_PLLXTPRE;

		const uint32_t mul = PLLMUL_normed + 2; // + 2 because 0b0000 = x2

		// PLL Entry Clock Source
		if (PLLSRC == 0)
			cfreqs->system_clock = (HSI_VALUE / 2) * mul;
		else if (PLLXTPRE)
			cfreqs->system_clock = (HSE_VALUE / 2) * mul;
		else
			cfreqs->system_clock = HSE_VALUE * mul;

	// High Speed Internal Clock or Invalid
	case (SWS_HSI << 2):
	default:
		cfreqs->system_clock = HSI_VALUE;
		break;
	}

	const uint8_t *div_shifts = CONST_UINT8_PTR_PRE_HPRE_TO_RSHFT_080056DC;

	//// AHB Frequency
	const uint32_t HPRE = RCC->CFGR & RCC_CFGR_HPRE;
	const uint32_t HPRE_normed = HPRE >> RCC_CFGR_HPRE_Pos;
	cfreqs->ahb_clock = cfreqs->system_clock >> div_shifts[HPRE_normed];

	//// APB1 Frequency
	const uint32_t PPRE1 = RCC->CFGR & RCC_CFGR_PPRE1;
	const uint32_t PPRE1_normed = PPRE1 >> RCC_CFGR_PPRE1_Pos;
	cfreqs->apb1_clock = cfreqs->ahb_clock >> div_shifts[PPRE1_normed];

	//// APB2 Frequency
	const uint32_t PPRE2 = RCC->CFGR & RCC_CFGR_PPRE2;
	const uint32_t PPRE2_normed = PPRE2 >> RCC_CFGR_PPRE2_Pos;
	cfreqs->apb2_clock = cfreqs->ahb_clock >> div_shifts[PPRE2_normed];
}

/**
 * @ 0x08004ec0
 * Progress: DONE
 */
void init_SYSTICK(void)
{
	clock_freqs cfreqs;
	RCC_get_clock_freqs(&cfreqs);
	SYSTICK_set_RELOAD(cfreqs.ahb_clock / SYSTICK_RELOAD_AHB_DIV);
	SYSTICK_set_TICKINT(true);
	SYSTICK_action(SYSTICK_ENABLE);
}

/**
 * @ 0x080051a0
 * Progress: INCOMPLETE
 * TODO: Resolve unknowns
*/
void init_usb(void)
{
	RCC_set_USBPRE_BB(true);
	RCC_set_APB2ENR(1 << 23, true); // Reserved according to datasheet?!
	AIRCR_set(SCB_AIRCR_PRIGROUP_Msk); // Only use subpriorities and no group priorities
	_NVIC_SetPriority(20, 0); // Set RCC Global Interrupt to priority 0?
	RCC_set_APB2ENR(RCC_APB2ENR_IOPAEN, true); // Enable GPIOA clock

	// Set USB Full Speed Pull-Up toggle pin to output
	const gpio_cfg cfg = {
		.gpios = GPIOA_USB_FULL_SPEED_PU,
		.op = GPIO_CFG_OP_OTHER,
		.other_cr_bits = GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_2MHZ
	};
	GPIOs_cfg(GPIOA, &cfg);

	uint32_t *unknown_1 = UINT32_PTR_UNKNOWN_0x2000004D;
	uint32_t *unknown_2 = UINT32_PTR_PTR_UNKNOWN_0x20000098;
	uint32_t ***unknown_3 = UNKNOWN_PTR_PTR_PTR_0x2000009C;
	uint32_t *unknown_4 = UNKNOWN_PTR_0x200000A0;

	*unknown_1 = 2;
	*unknown_2 = UINT32_PTR_UNKNOWN_200005BC;
	*unknown_3 = UNKNOWN_PTR_PTR_200000E8; // Callback function?
	*unknown_4 = UINT32_PTR_UNKNOWN_0x20000118;

	// (**unknown_3)(); jumps to 0x80050d0
}

/**
 * @ 0x80050d0
 * Progress: INCOMPLETE
 */
void UndefinedFunction_80050d0(void)
{
	// undefined4 *puVar1;
	// int *piVar2;
	// int iVar3;

	// init_USB_per();
	// piVar2 = UINT32_PTR_PTR_UNKNOWN_0x20000098;
	// iVar3 = *UINT32_PTR_PTR_UNKNOWN_0x20000098;
	// *(undefined *)(iVar3 + 8) = *(undefined *)(DAT_08005104 + 7);
	// *(undefined *)(iVar3 + 9) = 0;
	// *(undefined *)(iVar3 + 10) = 0;
	// *DAT_0800510c = 0xbf00;
	// puVar1 = DAT_08005110;
	// *DAT_08005110 = 0xbf00;
	// puVar1[1] = 0;
	// while (*(char *)(*piVar2 + 9) == '\0') {
	// 	piVar2 = (int *)FUN_08004092();
	// }
	// return;

	init_USB_per();

	uint32_t *unknown_1 = UINT32_PTR_PTR_UNKNOWN_0x20000098;
	uint32_t *unknown_2 = *unknown_1; // 0x200005bc

	// Some Struct
	uint8_t *unknown_3 = *unknown_1 + 8;
	uint8_t *unknown_4 = *unknown_1 + 9;
	uint8_t *unknown_5 = *unknown_1 + 10;

	// Some String
	char* unknown_6 = CONST_CHAR_PTR_UNKNOWN_0x8005871;
	*unknown_3 = unknown_6;

	uint32_t *init_USB_CNTR = UINT32_PTR_USB_CNTR_MASK_0x20000094;
	// TODO Check if different
	*init_USB_CNTR = USB_CNTR_ESOFM
	                 | USB_CNTR_SOFM
	                 | USB_CNTR_RESETM
	                 | USB_CNTR_SUSPM
	                 | USB_CNTR_WKUPM
	                 | USB_CNTR_ERRM
	                 | USB_CNTR_CTRM;

	USB->CNTR = *init_USB_CNTR;
	USB->ISTR = 0;

	/* Likely raised by some USB interrupt
	   upon successful USB initialization */
	while (*unknown_4 == 0)
		nop2();
}


/**
 * @ 0x80044ac
 * Progress: DONE
 */
uint32_t init_USB_per(void)
{
	uint32_t *init_USB_CNTR = UINT32_PTR_USB_CNTR_MASK_0x20000094;

	USB->CNTR = USB_CNTR_FRES; // Power up and reset USB peripheral
	USB->ISTR = 0; // Clear interrupt status register

	// Configure USB interrupts
	*init_USB_CNTR = USB_CNTR_ESOFM
	                 | USB_CNTR_SOFM
	                 | USB_CNTR_RESETM
	                 | USB_CNTR_SUSPM
	                 | USB_CNTR_WKUPM
	                 | USB_CNTR_ERRM
	                 | USB_CNTR_CTRM;

	USB->CNTR = *init_USB_CNTR;

	// Toggle USB Full Speed Pull-Up high (this will notify the host about the device)
	USB_FS_PU_set(true);

	return 0;
}

/**
 * @ 0x08004090
 * Progress: DONE
 */
void nop1(void)
{
	return;
}

/**
 * @ 0x08004092
 * Progress: DONE
 */
void nop2(void)
{
	return;
}

/**
 * @ 0x08001abc
 * Progress: DONE
 * Called by Bootloader IVT Entry @ 0x0800008C
 */
void ISR_BL_USB_HP(void)
{
	FUN_08001b6c();
	return;
}

/**
 * @ 0x08001bc8
 * Progress: DONE
 * Called by Bootloader IVT Entry @ 0x08000090
 */
void ISR_BL_USB_LP_ISR(void)
{
	FUN_08001b6c();
	return;
}

/**
 * @ 0x08005178
 * Progress: DONE
 * Called by Firmware IVT Entry @ 0x0800208C
 */
void ISR_FW_USB_HP_ISR(void)
{
	USB_handler();
	return;
}

/**
 * @ 0x0080052e0
 * Progress: DONE
 * Called by Firmware IVT Entry @ 0x08002090
 */
void ISR_FW_USB_LP_ISR(void)
{
	USB_handler();
	return;
}

/**
 * @ 0x08005228
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * Firmware USB Interrupt Handler
 * Notes: USB->ISTR can be overwritten since it ignores 1's.
 * 	  This allows for atomic operations on the register.
 * TODO: Decompile called functions
 */

void USB_handler(void)
{
	uint16_t *sof_counter = UINT16_PTR_USB_SOF_COUNTER_0x2000004C;
	uint16_t *usb_istr_cpy = UINT16_PTR_USB_ISTR_CPY_0x20000056;
	uint32_t *init_usb_cntr = UINT32_PTR_USB_CNTR_MASK_0x20000094;

	*usb_istr_cpy = USB->ISTR;
	const uint16_t relevant_istr = *usb_istr_cpy & *init_usb_cntr;

	// Reset Request
	if (relevant_istr & USB_ISTR_RESET) {
		USB->ISTR = ~(USB_ISTR_RESET);
		// (**(code **)(FPTR_PTR_FW_USB_RESET_CALLBACK_200000EC))();
		fptr f_ptr = (fptr)(*(uint32_t *)(FPTR_PTR_FW_USB_RESET_CALLBACK_200000EC)); // Jumps to 0x8005019 (USB_RESET_callback)
	}

	// Error
	if (relevant_istr & USB_ISTR_ERR) {
		USB->ISTR = ~(USB_ISTR_ERR);
	}

	// Wakeup
	if (relevant_istr & USB_ISTR_WKUP) {
		USB->ISTR = ~(USB_ISTR_WKUP);
		FUN_08004784(0);
	}

	// Suspend mode request
	if (relevant_istr & USB_ISTR_SUSP) {
		FUN_08004cdc();
		USB->ISTR = ~(USB_ISTR_SUSP);
	}

	// Start-Of-Frame (Sent every 1ms)
	if (relevant_istr & USB_ISTR_SOF) {
		USB->ISTR = ~(USB_ISTR_SOF);
		*sof_counter++;
	}

	// Expected Start-Of-Frame
	if (relevant_istr & USB_ISTR_ESOF) {
		USB->ISTR = ~(USB_ISTR_ESOF);
		FUN_08004784(7);
	}

	// Correct Transfer
	if (relevant_istr & USB_ISTR_CTR) {
		FUN_080025ac();
	}
}

/**
 * @ 0x0800483c
 * Progress: DONE
 * Description: Sets the buffer table address.
 * 		Last 3 bits zeroed to ensure 8-byte
 * 		alignment.
 */
void USB_BTABLE_set(uint32_t addr)
{
	USB->BTABLE = addr & 0xfffffff8;
	return;
}

/**
 * @ 0x0800498c
 * Progress: DONE
 * Description: Applies the mask to the selected
 * 		endpoint register
 * Note: The STAT_TX, DTOG_TX, EP_TYPE, STAT_RX and DTOG_RX bits
 * 	 are cleared before the mask is applied.
 */
void USB_set_EPR(uint32_t epid, uint32_t msk)
{
	uint32_t *USB_EPR = USB_BASE + (epid * 4); // EPR's are 4 bytes wide
	*USB_EPR &= ~(USB_EP0R_STAT_TX // Macro hints at EP0R, but can be used for any EP
	              | USB_EP_DTOG_TX
	              | USB_EP0R_EP_TYPE
	              | USB_EP0R_STAT_RX
	              | USB_EP_DTOG_RX
	             );

	*USB_EPR |= msk;
}

/**
 * @ 0x08004960
 * Progress: DONE
 * Description: Toggles the STAT_TX bits of the selected
 * 		endpoint register
 * Note: DTOG_TX, STAT_RX and DTOG_RX bits are cleared before toggling
 * Note: The provided mask must correspond to the EPR register
 * Note: See datasheet for more info on STAT_TX bits and toggling
 */
void USB_toggle_STAT_TX(uint32_t epid, uint32_t toggle_msk)
{
	uint32_t *USB_EPR = USB_BASE + (epid * 4); // EPR's are 4 bytes wide
	uint32_t mut = *USB_EPR & ~(USB_EP_DTOG_TX
	                            | USB_EP0R_STAT_RX
	                            | USB_EP_DTOG_RX);

	if (toggle_msk & (1 << 4))
		mut ^= USB_EP0R_STAT_TX_0;

	if (toggle_msk & (1 << 5))
		mut ^= USB_EP0R_STAT_TX_1;

	*USB_EPR = mut;
}


/**
 * @ 0x08004894
 * Progress: DONE
 * Description: Sets the reception buffer address for the selected endpoint
 * Note: Due to APB bridge limitations, the datasheet
 * 	 demands that the BTABLE is accessed by multiplying
 * 	 the "USB local address" by 2. Their wording regarding this is
 * 	 incredibly unspecific and there seems to be no clarification
 * 	 if the BTABLE address must also be multiplied by 2. The provided
 * 	 address equations in the datasheet do not seem to explicitly
 * 	 indicate this, however, since the LPD8 firmware does do this, I
 * 	 simply must assume that multiplying the BTABLE address by 2 is
 * 	 in fact correct.
 */
void USB_set_ADDR_RX(uint32_t epid, uint32_t addr)
{
	const uint16_t BTABLE_ADDR = USB->BTABLE;
	const uint32_t USB_ADDR_RX_offset_local = BTABLE_ADDR + (epid * 8) + 4;
	const uint32_t USB_ADDR_RX_offset = USB_ADDR_RX_offset_local * 2; // See 23.5.3 in RM0008

	uint32_t *USB_ADDR_RX = USB_PMAADDR + USB_ADDR_RX_offset;

	*USB_ADDR_RX = addr & 0xfffffffe;
}


/**
 * @ 0x08005018
 * Progress: Incomplete
 */
void USB_RESET_callback(void)
{
	// int iVar1;

	// iVar1 = *UINT32_PTR_PTR_UNKNOWN_0x20000098;
	// *(undefined *)(iVar1 + 9) = 0;
	// *(undefined *)(iVar1 + 10) = 0;
	// USB_BTABLE_set(0);
	// USB_set_EPR(0,0x200);
	// USB_toggle_STAT_TX(0,0x10);
	// USB_set_ADDR_RX(0,0x40);
	// FUN_08004924(0,0x50);
	// FUN_08002774(0);
	// FUN_080048b4(0,0x10);
	// FUN_080048f8(0,0x3000);
	// USB_set_EPR(1,0);
	// FUN_08004924(1,0xa0);
	// FUN_08004944(1,0x40);
	// USB_toggle_STAT_TX(1,0x30);
	// USB_set_ADDR_RX(1,0x60);
	// FUN_08002774(1);
	// FUN_080048b4(1,0x40);
	// FUN_080048f8(1,0x3000);
	// FUN_08004858(0);
	// return;

	uint32_t unknown_struct_base = *(uint32_t *)UINT32_PTR_PTR_UNKNOWN_0x20000098;
	uint8_t *unknown_struct_element_1 = *(uint8_t *)(unknown_struct_base + 9);
	uint8_t *unknown_struct_element_2 = *(uint8_t *)(unknown_struct_base + 10);

	*unknown_struct_element_1 = 0;
	*unknown_struct_element_2 = 0;

	USB_BTABLE_set(0);
	USB_set_EPR(0,0x200);
	USB_toggle_STAT_TX(0,0x10);
	USB_set_ADDR_RX(0,0x40);
	FUN_08004924(0,0x50);
	FUN_08002774(0);
	FUN_080048b4(0,0x10);
	FUN_080048f8(0,0x3000);
	USB_set_EPR(1,0);
	FUN_08004924(1,0xa0);
	FUN_08004944(1,0x40);
	USB_toggle_STAT_TX(1,0x30);
	USB_set_ADDR_RX(1,0x60);
	FUN_08002774(1);
	FUN_080048b4(1,0x40);
	FUN_080048f8(1,0x3000);
	FUN_08004858(0);
}

/**
 * @ 0x08001b6c
 * Progress: INCOMPLETE
 * Bootloader USB Interrupt Handler
 */
void FUN_08001b6c(void)
{
	// ushort uVar1;
	// ushort *puVar2;
	// int midi_buf_tail;
	// int usb_istr_minus_0xc44;

	// midi_buf_tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c;
	// usb_istr_minus_0xc44 = UART5;
	// *(short *)(unknown_1) = (short)*(undefined4 *)(USB->ISTR);
	// puVar2 = UINT16_PTR_UNKNOWN_0x20000054;
	// if ((int)((uint)(*(ushort *)(unknown_1) & *UINT16_PTR_UNKNOWN_0x20000054) << 0x15) < 0) {
	// 	*(undefined4 *)(USB->ISTR) = 0xfbff;
	// 	(**(code **)(DAT_08001bc4 + 4))();
	// }
	// uVar1 = *puVar2;
	// if ((int)((uint)(*(ushort *)(unknown_1) & uVar1) << 0x12) < 0) {
	// 	*(undefined4 *)(USB->ISTR) = 0xdfff;
	// }
	// if ((int)((uint)(*(ushort *)(unknown_1) & uVar1) << 0x10) < 0) {
	// 	FUN_080002c0();
	// 	return;
	// }
	// return;

	uint16_t *unknown_1 = UNKNOWN_PTR_0x20000014;
	uint16_t *unknown_2 = UINT16_PTR_UNKNOWN_0x20000054;

	*unknown_1 = USB->ISTR;
	if ((*unknown_1 & *unknown_2) << 0x15 < 0) {
		USB->ISTR = 0xfbff;
		// (**(code **)(DAT_08001bc4 + 4))();
	}

	if ((*unknown_1 & *unknown_2) << 0x12 < 0) {
		USB->ISTR = 0xdfff;
	}

	if ((*unknown_1 & *unknown_2) << 0x10 < 0) {
		FUN_080002c0();
	}
}

/**
 * @0x08004338
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm USB Full Speed resistor pull down
 */
void init_GPIOs()
{
	RCC_set_APB2ENR(RCC_APB2ENR_IOPAEN, true);
	RCC_set_APB2ENR(RCC_APB2ENR_IOPBEN, true);
	RCC_set_APB2ENR(RCC_APB2ENR_IOPCEN, true);

	// Set unused GPIOs to input with pull-up
	const uint32_t GPIO_10TO15_MSK = 0xFC00;
	const gpio_cfg cfg = {
		.gpios = GPIO_10TO15_MSK,
		.op = GPIO_CFG_OP_IN_PU
	};
	GPIOs_cfg(GPIOC, &cfg);

	// Set Push Button GPIOs to input with pull-up
	const uint32_t GPIO_6TO9_MSK = 0x03C0;
	const gpio_cfg cfg = {
		.gpios = GPIO_6TO9_MSK,
		.op = GPIO_CFG_OP_IN_PU
	};
	GPIOs_cfg(GPIOC, &cfg);

	// Set LED GPIOs to 2Mhz push pull output
	const uint32_t GPIO_5TO15_MSK = 0xFFE0;
	const gpio_cfg cfg = {
		.gpios = GPIO_5TO15_MSK,
		.other_cr_bits = GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_2MHZ,
		.op = GPIO_CFG_OP_OTHER
	};
	GPIOs_cfg(GPIOB, &cfg);

	// Set USB Full Speed resistor to pull down
	const gpio_cfg cfg = {
		.gpios = GPIOA_USB_FULL_SPEED_PU,
		.other_cr_bits = GPIO_CR_CNF_MODE_OUTPUT_PUSH_PULL_2MHZ,
		.op = GPIO_CFG_OP_OTHER
	};
	GPIOs_cfg(GPIOA, &cfg);
	USB_FS_PU_set(false);

	// Set all LEDs to OFF
	LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PAD_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PB_CC_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_1_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_2_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_3_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_4_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_5_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_6_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_7_GPIO);
	LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_8_GPIO);
}

/**
 * @ 0x08005670
 * Progress: DONE
 */
void init_midi_buffer(void)
{
	uint8_t *remaining_space = UINT8_PTR_MIDI_BUFFER_REMAINING_SPACE_0x20000004;
	uint8_t **tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c;
	uint8_t **head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008;

	*head = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
	*tail = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
	*remaining_space = MIDI_BUFFER_SIZE;
}

/**
 * @ 0x08003abc
 * Progress: DONE
 */
void set_IWDG_KR(uint32_t key)
{
	IWDG->KR = key;
}

/**
 * @ 0x08003aa4
 * Progress: DONE
 */
void set_IWDG_PR(uint32_t pr)
{
	IWDG->PR = pr;
}

/**
 * @ 0x08003ab0
 * Progress: DONE
 */
void set_IWDG_RLR(uint32_t rl)

{
	// *(undefined4 *)(DAT_08003ab8 + 8) = rl;
	// return;
	IWDG->RLR = rl;
}

/**
 * @ 0x08003a70
 * Progress: DONE
 */
void init_IWDG(void)
{
	set_IWDG_KR(IWDG_KEY_WRITE_ACCESS_ENABLE);
	set_IWDG_PR(IWDG_PR_PR_0 | IWDG_PR_PR_1); // divider/32
	set_IWDG_RLR(IWDG_RL_VAL);
	reload_IWDG();
	IWDG->KR = IWDG_KEY_ENABLE;
	return;
}

/**
 * @ 0x080023e6
 * Progress: DONE
 */
void ADC_set_DMA(uint32_t adc_base, bool enable)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	if (enable)
		*ADC_CR2 |= ADC_CR2_DMA;
	else
		*ADC_CR2 &= ~ADC_CR2_DMA;
}

/**
 * @ 0x080023d0
 * Progress: DONE
 */
void ADC_set_ADON(uint32_t adc_base, bool enable)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	if (enable)
		*ADC_CR2 |= ADC_CR2_ADON;
	else
		*ADC_CR2 &= ~ADC_CR2_ADON;
}

/**
 * @ 0x08002558
 * Progress: DONE
 */
void ADC_set_RSTCAL(uint32_t adc_base)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	*ADC_CR2 |= ADC_CR2_RSTCAL;
}

/**
 * @ 0x0800248e
 * Progress: DONE
 */
bool ADC_read_RSTCAL(uint32_t adc_base)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	// Real code uses shift to MSB and then checks if negative
	return *ADC_CR2 & ADC_CR2_RSTCAL;
}

/**
 * @ 0x08002578
 * Progress: DONE
 */
void ADC_set_CAL(uint32_t adc_base)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	*ADC_CR2 |= ADC_CR2_CAL;
}

/**
 * @ 0x08002480
 * Progress: DONE
 */
bool ADC_read_CAL(uint32_t adc_base)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	// Real code uses shift to MSB and then checks if negative
	return *ADC_CR2 & ADC_CR2_CAL;
}

/**
 * @ 0x08002562
 * Progress: DONE
 */
void ADC_set_EXTTRIG_SWSTART(uint32_t adc_base, bool enable)
{
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	if (enable)
		*ADC_CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL;
	else
		*ADC_CR2 &= ~(ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL);
}

/**
 * @ 0x0800249c
 * Progress: DONE
 */
void ADC_set_DUALMOD_SCAN_CONT_EXTSEL_ALIGN_nconv(
    uint32_t adc_base,
    uint32_t msk_DUALMOD, bool SCAN,
    bool CONT, uint32_t msk_EXTSEL, uint32_t msk_ALIGN,
    uint8_t n_conv)
{
	uint32_t *ADC_CR1 = adc_base + ADC_CR1_OFFSET;
	uint32_t *ADC_CR2 = adc_base + ADC_CR2_OFFSET;
	uint32_t *ADC_SQR1 = adc_base + ADC_SQR1_OFFSET;

	*ADC_CR1 &= ~(ADC_CR1_SCAN | ADC_CR1_DUALMOD);
	*ADC_CR1 |= msk_DUALMOD;
	*ADC_CR1 |= SCAN ? ADC_CR1_SCAN : 0;

	*ADC_CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_EXTSEL);
	*ADC_CR2 |= CONT ? ADC_CR2_CONT : 0;
	*ADC_CR2 |= msk_EXTSEL;
	*ADC_CR2 |= msk_ALIGN;

	*ADC_SQR1 &= ~(ADC_SQR1_L);
	*ADC_SQR1 |= (n_conv - 1) << ADC_SQR1_L_Pos; // 0000 = 1 conversion, hence -1
}

/**
 * @ 0x08002d7c
 * Progress: DONE
 */
void DMA_CCR_set_EN(uint32_t *dma_ccr, bool enable)
{
	if (enable)
		*dma_ccr |= DMA_CCR_EN;
	else
		*dma_ccr &= ~DMA_CCR_EN;
}

/**
 * @ 0x08002eb0
 * Progress: DONE
 */
void DMA_set_CPAR_CMAR_DIR_CNDTR_PINC_MINC_PSIZE_MSIZE_CIRC_PL_MEM2MEM(
    uint32_t *dma_ccr,
    uint32_t *cpar, uint32_t *cmar,
    uint32_t dir_msk, uint32_t cndtr,
    uint32_t pinc_msk, uint32_t minc_msk,
    uint32_t psize_msk, uint32_t msize_msk,
    uint32_t circ_msk, uint32_t pl_msk,
    uint32_t mem2mem_msk)
{
	uint32_t *dma_cndtr = dma_ccr + 0x4;
	uint32_t *dma_cpar = dma_cndtr + 0x4;
	uint32_t *dma_cmar = dma_cpar + 0x4;

	*dma_ccr &= ~(DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_PINC | DMA_CCR_MINC | DMA_CCR_PSIZE | DMA_CCR_MSIZE | DMA_CCR_PL | DMA_CCR_MEM2MEM);

	*dma_ccr |= dir_msk | circ_msk | pinc_msk | minc_msk | psize_msk | msize_msk | pl_msk | mem2mem_msk;

	*dma_cndtr = cndtr;
	*dma_cpar = cpar;
	*dma_cmar = cmar;
}

/**
 * @ 0x08002d94
 * Progress: DONE
 */
void DMA_clear_IFCR(uint32_t *dma_ccr)
{
	uint32_t *dma_cndtr = dma_ccr + 0x4;
	uint32_t *dma_cpar = dma_cndtr + 0x4;
	uint32_t *dma_cmar = dma_cpar + 0x4;

	*dma_ccr &= ~DMA_CCR_EN; // Disable DMA

	// Reset DMA registers
	*dma_ccr = 0;
	*dma_cndtr = 0;
	*dma_cpar = 0;
	*dma_cmar = 0;

	switch ((uintptr_t)dma_ccr) {
	case DMA1_Channel1_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF1 |
		              DMA_IFCR_CHTIF1 |
		              DMA_IFCR_CTCIF1 |
		              DMA_IFCR_CTEIF1;
		break;
	case DMA1_Channel2_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF2 |
		              DMA_IFCR_CHTIF2 |
		              DMA_IFCR_CTCIF2 |
		              DMA_IFCR_CTEIF2;
		break;
	case DMA1_Channel3_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF3 |
		              DMA_IFCR_CHTIF3 |
		              DMA_IFCR_CTCIF3 |
		              DMA_IFCR_CTEIF3;
		break;
	case DMA1_Channel4_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF4 |
		              DMA_IFCR_CHTIF4 |
		              DMA_IFCR_CTCIF4 |
		              DMA_IFCR_CTEIF4;
		break;
	case DMA1_Channel5_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF5 |
		              DMA_IFCR_CHTIF5 |
		              DMA_IFCR_CTCIF5 |
		              DMA_IFCR_CTEIF5;
		break;
	case DMA1_Channel6_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF6 |
		              DMA_IFCR_CHTIF6 |
		              DMA_IFCR_CTCIF6 |
		              DMA_IFCR_CTEIF6;
		break;
	case DMA1_Channel7_BASE:
		DMA1->IFCR |= DMA_IFCR_CGIF7 |
		              DMA_IFCR_CHTIF7 |
		              DMA_IFCR_CTCIF7 |
		              DMA_IFCR_CTEIF7;
		break;
	case DMA2_Channel1_BASE:
		DMA2->IFCR |= DMA_IFCR_CGIF1 |
		              DMA_IFCR_CHTIF1 |
		              DMA_IFCR_CTCIF1 |
		              DMA_IFCR_CTEIF1;
		break;
	case DMA2_Channel2_BASE:
		DMA2->IFCR |= DMA_IFCR_CGIF2 |
		              DMA_IFCR_CHTIF2 |
		              DMA_IFCR_CTCIF2 |
		              DMA_IFCR_CTEIF2;
		break;
	case DMA2_Channel3_BASE:
		DMA2->IFCR |= DMA_IFCR_CGIF3 |
		              DMA_IFCR_CHTIF3 |
		              DMA_IFCR_CTCIF3 |
		              DMA_IFCR_CTEIF3;
		break;
	case DMA2_Channel4_BASE:
		DMA2->IFCR |= DMA_IFCR_CGIF4 |
		              DMA_IFCR_CHTIF4 |
		              DMA_IFCR_CTCIF4 |
		              DMA_IFCR_CTEIF4;
		break;
	case DMA2_Channel5_BASE:
		DMA2->IFCR |= DMA_IFCR_CGIF5 |
		              DMA_IFCR_CHTIF5 |
		              DMA_IFCR_CTCIF5 |
		              DMA_IFCR_CTEIF5;
		break;
	default:
		break;
	}
}

/**
 * @ 0x08002e90
 * Progress: DONE
 * See P. 284, Section 13.4.1 - DMA_ISR Register in RM0008
 */
bool DMA_read_ISR(uint32_t msk)
{
	// 28th bit chooses between DMA1 and DMA2
	if (msk & (DMA_ISR_TEIF7 << 1))
		return DMA2->ISR & msk;
	else
		return DMA1->ISR & msk;
}

/**
 * @ 0x08002d68
 * Progress: DONE
 */
void DMA_set_IFCR(uint32_t msk)
{
	// 28th bit chooses between DMA1 and DMA2
	if (msk & (DMA_ISR_TEIF7 << 1))
		DMA2->IFCR |= msk;
	else
		DMA1->IFCR |= msk;
}

/**
 * @ 0x08004738
 * Progress: DONE
 */
void RCC_set_SW(uint32_t msk)
{
	RCC->CFGR &= ~RCC_CFGR_SW; // HSI selected as system clock
	RCC->CFGR |= msk;
}

/**
 * @ 0x0800474c
 * Progress: DONE
 */
void RCC_set_USBPRE_BB(bool32_t set)
{
	*RCC_CFGR_USBPRE_BB = set;
}

/**
 * @ 0x08004630
 * Progress: DONE
 */
uint32_t RCC_get_SWS(void)
{
	return RCC->CFGR & RCC_CFGR_SWS;
}

/**
 * @ 0x08004718
 * Progress: DONE
 */
void RCC_set_PLLON_BB(bool32_t set)
{
	*RCC_CR_PLLON_BB = set;
}

/**
 * @ 0x08004504
 * Progress: DONE
 */
void RCC_set_APB2ENR(uint32_t msk, bool set)
{
	if (set)
		RCC->APB2ENR |= msk;
	else // clear
		RCC->APB2ENR &= ~msk;
}

/**
 * @ 0x080044d4
 * Progress: DONE
 */
void RCC_set_AHBENR(uint32_t msk, bool set)
{
	if (set)
		RCC->AHBENR |= msk;
	else
		RCC->AHBENR &= ~msk;
}

/**
 * @ 0x080024e4
 * Progress: ALMOST DONE
 * TODO: Explain the seemingly "magic" multiplications and subtractions
 * 	 Quick reference to datasheet should do it.
 */
void ADC_set_SMPR_SQR(uint32_t adc_base, uint8_t channel, uint8_t nth_conv, uint32_t smp_bits)
{
	uint32_t *ADC_SMPR1 = adc_base + ADC_SMPR1_OFFSET;
	uint32_t *ADC_SMPR2 = adc_base + ADC_SMPR2_OFFSET;
	uint32_t *ADC_SQR1 = adc_base + ADC_SQR1_OFFSET;
	uint32_t *ADC_SQR2 = adc_base + ADC_SQR2_OFFSET;
	uint32_t *ADC_SQR3 = adc_base + ADC_SQR3_OFFSET;

	if (channel < 10) {
		*ADC_SMPR2 &= ~(ADC_SMPR2_SMP0 << (channel * 3));
		*ADC_SMPR2 |= smp_bits << (channel * 3);
	} else {
		*ADC_SMPR1 &= ~(ADC_SMPR1_SMP10 << ((channel - 10) * 3));
		*ADC_SMPR1 |= smp_bits << ((channel - 10) * 3);
	}

	if (nth_conv < 7) {
		*ADC_SQR3 &= ~(ADC_SQR3_SQ1 << ((nth_conv - 1) * 5));
		*ADC_SQR3 |= channel << ((nth_conv - 1) * 5);
	} else if (nth_conv < 13) {
		*ADC_SQR2 &= ~(ADC_SQR2_SQ7 << ((nth_conv - 7) * 5));
		*ADC_SQR2 |= channel << ((nth_conv - 7) * 5);
	} else {
		*ADC_SQR1 &= ~(ADC_SQR1_SQ13 << ((nth_conv - 0xd) * 5));
		*ADC_SQR1 |= channel << ((nth_conv - 0xd) * 5);
	}
}

/**
 * @ 0x080021cc
 * Progress: ALMOST DONE
 * TODO: Add more comments for DMA and ADC stuff
 */
void init_analog(void)
{
	//// Enable GPIO Port A - C clocks
	RCC_set_APB2ENR(1 << RCC_APB2ENR_IOPAEN, true);
	RCC_set_APB2ENR(1 << RCC_APB2ENR_IOPBEN, true);
	RCC_set_APB2ENR(1 << RCC_APB2ENR_IOPCEN, true);

	//// Initialize Analog Inputs

	// Knobs 1-8
	const uint32_t GPIO_0TO7_MSK = 0xFF;
	GPIOs_cfg(GPIOA, &(gpio_cfg) {
		GPIO_0TO7_MSK, 0, GPIO_CFG_OP_IN_ANALOG
	});

	// Pads 1-2
	const uint32_t GPIO_0TO3_MSK = 0x0F;
	GPIOs_cfg(GPIOB, &(gpio_cfg) {
		GPIO_0TO3_MSK, 0, GPIO_CFG_OP_IN_ANALOG
	});

	// TODO: Pads 2-3 Missing??

	// Pads 5-8
	const uint32_t GPIO_2TO5_MSK = 0x3C;
	GPIOs_cfg(GPIOC, &(gpio_cfg) {
		GPIO_2TO5_MSK, 0, GPIO_CFG_OP_IN_ANALOG
	});

	RCC_set_AHBENR(1 << RCC_AHBENR_DMA1EN, true);
	RCC_set_APB2ENR(1 << RCC_APB2ENR_ADC1EN, true);

	DMA_clear_IFCR(DMA1_Channel1_BASE);
	DMA_set_CPAR_CMAR_DIR_CNDTR_PINC_MINC_PSIZE_MSIZE_CIRC_PL_MEM2MEM(
	    DMA1_Channel1_BASE,
	    ADC1->DR,
	    UINT16_16_PTR_DMA1_MEM_0x2000013C,
	    0x0,
	    DMA1_NUM_DATA,
	    0x0,
	    DMA_CCR_MINC,
	    DMA_CCR_PSIZE_0,
	    DMA_CCR_MSIZE_0,
	    DMA_CCR_CIRC,
	    DMA_CCR_PL_1,
	    0x0);
	DMA_CCR_set_EN(DMA1_Channel1_BASE, true);

	ADC_set_DUALMOD_SCAN_CONT_EXTSEL_ALIGN_nconv(
	    ADC1_BASE,
	    ADC_CR1_DUALMOD_1 | ADC_CR1_DUALMOD_2, // Regular simultaneous mode only
	    true,
	    false,
	    ADC_CR2_EXTSEL, // SWSTART
	    0x0,	    // Right-aligned
	    ADC_N_CONV);

	// Order in which ADC Channels are sampled
	ADC_set_SMPR_SQR(ADC1_BASE, 0, 1, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 1, 2, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 2, 3, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 3, 4, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 4, 5, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 5, 6, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 6, 7, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 7, 8, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 8, 9, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 9, 10, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 10, 11, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 11, 12, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 12, 13, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 13, 14, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 14, 15, 6);
	ADC_set_SMPR_SQR(ADC1_BASE, 15, 16, 6);

	ADC_set_DMA(ADC1_BASE, true);
	ADC_set_ADON(ADC1_BASE, true);
	ADC_set_RSTCAL(ADC1_BASE);
	while (ADC_read_RSTCAL(ADC1_BASE) != 0)
		;
	ADC_set_CAL(ADC1_BASE);
	while (ADC_read_CAL(ADC1_BASE) != 0)
		;

	ADC_SET_ADON(ADC2_BASE, true);
	ADC_SET_RSTCAL(ADC2_BASE);
	while (ADC_READ_RSTCAL(ADC2_BASE) != 0)
		;
	ADC_SET_CAL(ADC2_BASE);
	while (ADC_READ_CAL(ADC2_BASE) != 0)
		;

	ADC_SET_EXTTRIG_SWSTART(ADC1_BASE, true);
}

/**
 * @ 0x080039aa
 * Progress: DONE
 *
 * The following function is used to configure a selection of GPIOs
 * to a specific mode. To do so, the function takes a pointer to the
 * GPIO base address and a pointer to a gpio_cfg struct as arguments.
 * The gpio_cfg struct contains information about which GPIOs to configure
 * and which mode to configure them to. The most important field of the
 * gpio_cfg struct is the 'op' field, which must be set in accordance to
 * the logic of this function. The GPIO_CFG_OP_* macros have been provided
 * for this purpose.
 *
 * Honestly, the logic for this function is a bit convoluted. I would not
 * be suprised if compiler optimization is to blame for this.
 */

void GPIOs_cfg(uint32_t *gpio_base, gpio_cfg *gpio_cfg)
{
	// By default, configure GPIOs to the mode specified
	// by the lower 4 bits of the 'op' field
	// Ex. for op == GPIO_CFG_OP_IN_PD or
	// op == GPIO_CFG_OP_IN_PU, the lower 4 bits are 0x8,
	// which corresponds to Input with Pull-Down/Pull-Up
	// respectively
	uint8_t cnf_mode_bits = gpio_cfg->op & 0x0F;

	uint32_t *GPIO_BSRR = gpio_base + GPIO_BSRR_OFFSET;
	uint32_t *GPIO_BRR = gpio_base + GPIO_BRR_OFFSET;
	uint32_t *GPIO_CRL = gpio_base + GPIO_CRL_OFFSET;
	uint32_t *GPIO_CRH = gpio_base + GPIO_CRH_OFFSET;

	// If GPIO_CFG_OP_OTHER is set, the cnf_mode bits
	// are taken from the 'other_cr_bits' field instead
	// From further code analysis: This case appears to
	// only be used when configuring GPIOs as outputs
	if (gpio_cfg->op & GPIO_CFG_OP_OTHER)
		cnf_mode_bits |= gpio_cfg->other_cr_bits;

	// Configure GPIO's 0-7, if any of them must be configured
	if (gpio_cfg->gpios & 0xFF) {
		uint32_t crl = *GPIO_CRL;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++) {
			uint8_t msk = 1 << i_gpio;

			if (gpio_cfg->gpios & msk) {
				crl &= ~(GPIO_CR_CNF_MODE << (i_gpio * 4));
				crl |= cnf_mode_bits << (i_gpio * 4);

				if (gpio_cfg->op == GPIO_CFG_OP_IN_PD)
					*GPIO_BRR = msk; // Set to Pull-Down
				else if (gpio_cfg->op == GPIO_CFG_OP_IN_PU)
					*GPIO_BSRR = msk; // Set to Pull-Up
			}
		}
		*GPIO_CRL = crl;
	}

	// Same logic as above, but for GPIO's 8-15
	if (gpio_cfg->gpios > 0xFF) {
		uint32_t crh = *GPIO_CRH;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++) {
			uint8_t msk = 1 << (i_gpio + 8); // + 8 due to GPIO's 8-15

			if (gpio_cfg->gpios & msk) {
				crh &= ~(GPIO_CR_CNF_MODE << (i_gpio * 4));
				crh |= cnf_mode_bits << (i_gpio * 4);

				if (gpio_cfg->op == GPIO_CFG_OP_IN_PD)
					*GPIO_BRR = msk;
				else if (gpio_cfg->op == GPIO_CFG_OP_IN_PU)
					*GPIO_BSRR = msk;
			}
		}
		*GPIO_CRH = crh;
	}
}

/**
 * @ 0x08005688
 * Progress: ALMOST DONE
 * TODO: Explain why a subtraction of 0x10 is performed
 *       to calculate the remaining space
 */
void write_midi_buffer(void *data, uint32_t size)
{
	// TODO: Clarify which pointers are only used for their address

	uint8_t *remaining_space = UINT8_PTR_MIDI_BUFFER_REMAINING_SPACE_0x20000004;
	uint8_t *buffer_end_addr = UINT8_PTR_MIDI_BUFFER_END_0x2000024C;
	uint8_t **head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008;
	uint8_t **tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c; // buffer_end_addr - MIDI_BUFFER_SIZE

	for (uint8_t i = 0; i < size; i++) {
		uint8_t *next_element = *head + 1;

		// When buffer_end_addr is reached, continue buffer
		// writing at buffer_end_addr - MIDI_BUFFER_SIZE
		if (next_element == buffer_end_addr) {
			next_element = *tail; // 0x2000015C
		}

		// Buffer is full
		if (next_element == *tail)
			break;

		// Store data into buffer and move to next element
		**head = *(uint8_t *)data;
		*head = next_element;

		// Move to next byte of data
		data++;
	}

	const uint32_t last_element_addr = *head;
	const uint32_t tail_addr = *tail;

	if (*tail <= *head)
		*remaining_space = (tail_addr - last_element_addr) - 0x10;
	else
		*remaining_space = last_element_addr - tail_addr;
}

/**
 * @ 0x080035bc
 * Progress: INCOMPLETE
 */
// FUN_080035bc(0,puVar5,0x39);
// FUN_080035bc(0x40,puVar16,0x39);
// FUN_080035bc(0x80,puVar14,0x39);
// FUN_080035bc(0xc0,puVar15,0x39);
uint32_t FUN_080035bc(uint32_t offset, uint8_t *src, uint32_t n)
{
	if (offset + n > 0xFF)
		return 0;

	uint32_t *dest = UINT8_PTR_MIDI_BUFFER_END_0x2000024C + offset;

	for (uint16_t i = 0; i < n; i++) {
		*dest = *src;
		src++;
		dest++;
	}

	return 1;
}

/**
 * @ 0x08003b10
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm purpose of ready flag,
 * 	 Understand addition of 1 to range
 * 	 Confirm write_mid output
 */
void eval_knobs(void)
{
	const uint8_t systick_decr_0 = *(uint8_t *)UINT8_PTR_SYSTICK_DECREMENTER_0_0x20000018;
	const uint8_t sel_prog = *(uint8_t *)UINT8_PTR_SELECTED_PROG_0x20000042;
	program_settings *all_prog_settings = PROGRAM_SETTINGS_5_PTR_0x200003CC;
	program_settings *sel_prog_settings = &all_prog_settings[sel_prog];
	bool *scaled_knob_vals_changed = BOOL_8_PTR_SCALED_KNOB_VALS_CHANGED_0x20000019;
	midi_data_t *prev_scaled_knob_vals = UINT8_8_PTR_PREV_SCALED_KNOB_VALS_0x20000021;
	midi_data_t *prev_prev_scaled_knob_vals = UINT8_8_PTR_PREV_PREV_SCALED_KNOB_VALS_0x20000029;
	filtered_adc_val_t *prev_accepted_knob_analog_vals = UINT16_8_PTR_PREV_ACCEPTED_KNOB_ANALOG_VALS_0x200004EA;
	filtered_analog_inputs *filtered_analog_inputs = FILTERED_ANALOG_INPUTS_PTR_0x2000059A;
	filtered_adc_val_t *knob_analog_vals = &filtered_analog_inputs->knobs;
	ready_t *knob_analog_rdy = UINT8_PTR_KNOB_ANALOG_VALS_READY_0x20000040;

	/*
	 * Appears to be reset after intervals of 4 calls
	 * Probably SysTick related
	 */
	if (*knob_analog_rdy != READY)
		return;

	*knob_analog_rdy = NOT_READY;

	// Treat invalid MIDI channels as channel 0 (aka. 1)
	uint8_t midi_ch = sel_prog_settings->midi_ch;
	if (midi_ch > MIDI_MAX_CHANNEL)
		midi_ch = 0;

	for (uint8_t i = 0; i < N_KNOBS; i++) {
		uint8_t knob_cc = sel_prog_settings->knobs[i].cc;
		uint8_t knob_rightmost = sel_prog_settings->knobs[i].rightmost;
		uint8_t knob_leftmost = sel_prog_settings->knobs[i].leftmost;
		bool *scaled_knob_val_changed = &scaled_knob_vals_changed[i];
		midi_data_t *prev_knob_scaled_val = &prev_scaled_knob_vals[i];
		midi_data_t *prev_prev_knob_scaled_val = &prev_prev_scaled_knob_vals[i];
		filtered_adc_val_t *prev_accepted_knob_analog_val = &prev_accepted_knob_analog_vals[i];
		filtered_adc_val_t *knob_analog_val = &knob_analog_vals[i];

		const uint32_t _prev_accepted_knob_analog_val = (uint32_t)*prev_accepted_knob_analog_val;
		const uint32_t _knob_analog_val = (uint32_t)*knob_analog_val;

		// Knobs with CC 0 are ignored/disabled
		if (knob_cc == 0)
			continue;

		// Only update if knob has been moved beyond threshold
		if (!EXCEEDS_THRESHOLD(
		        _knob_analog_val,
		        _prev_accepted_knob_analog_val,
		        ADC_KNOB_CHANGE_THRESHOLD))
			continue;

		/* Scale ADC value to range specified by knob_leftmost and knob_rightmost
		 * Scaling is done with the following formula:
		 * scaled = b +/- (p * r) = knob_leftmost + ((knob_ad_val/0x3f7) * (knob_rightmost - knob_leftmost + 1))
		 * Where:
		 * 	- b is the lower/upper bound of the range, here: knob_leftmost
		 * 	- An addition is performed in the leftmost knob value is smaller than the rightmost
		 * 	  knob value, otherwise a subtraction is performed (knob operates "inverted")
		 *      - r is the range, calculated by the difference between the minimum and maximum
		 * 	  configured knob values, with 1 added to the result, here: (knob_rightmost - knob_leftmost) + 1
		 * 	- p is the percentage of the range added/subtracted to the lower/upper bound. The percentage
		 * 	  is calculated by dividing the knob ADC value by 0x3f7, which is the nearly the maximum value
		 * 	  of the ADC (0x3ff).
		 *
		 * To avoid the need for floating point arithmetic, p has been split up:
		 * 		scaled = b +/r (p * r) = l + (adc_val * r) / 0x3f7
		 * The product of adc_var * r is likely to be large, allowing better precision
		 * when dividing by 0x3f7 without the need for floating point arithmetic.
		 *
		 * I suspect the division by 0x3F7 is to ensure that 100% can always be reached
		 * even if the change threshold (7) doesn't allow for 0x3FF to be reached.
		 * As an example:
		 * 	Assume the previous accepted ADC value is 0x3F2 and the current
		 * 	ADC value is 0x3F9. In this case, no higher ADC values can be reached
		 * 	anymore. as the change threshold is 7 (0x3FF - 0x3F9 < 7). However, since
		 * 	the division is done by 0x3F7, the result will be floored to 1 (100%),
		 * 	still allowing the full range to be used.
		 * Theoretically 0x3F8 (0x3FF - 7 = 0x3F8) could be used as divisor, since the threshold
		 * is exceeded when a difference of greater EQUAL 7 is reached. Correct me if I'm wrong.
		 *
		 * As for the addition of 1 to the range, I'm still rather uncertain why this is done.
		 */

		uint8_t scaled;

		if (knob_leftmost < knob_rightmost) {
			uint8_t range = (knob_rightmost - knob_leftmost) + 1;
			scaled = knob_leftmost +
			         (_knob_analog_val * range) /
			         (MAX_ADC_KNOB_VAL - ADC_KNOB_CHANGE_THRESHOLD - 1);

			if (knob_rightmost < scaled)
				scaled = knob_rightmost;
		} else {
			uint8_t range = (knob_rightmost - knob_leftmost) + 1;
			scaled = knob_leftmost -
			         (_knob_analog_val * range) /
			         (MAX_ADC_KNOB_VAL - ADC_KNOB_CHANGE_THRESHOLD - 1);

			if (knob_rightmost > knob_leftmost)
				scaled = knob_rightmost;
		}

		if (scaled > MIDI_MAX_DATA_VAL)
			scaled = MIDI_MAX_DATA_VAL;

		if (*prev_knob_scaled_val != scaled) {

			/*
			 * Filter "spikes", ie. very short changes in values
			 * that are immediately reverted. Such spikes could
			 * likely be the result of ADC noise, etc.
			 *
			 * The criteria for a change the be discarded is the following:
			 * 	- The previous scaled value is different from the current scaled value
			 * 	- The scaled value two iterations ago is equal to the current scaled value
			 * 	- The magnitude of the spike/change in ADC value is less than 15 (0x0F).
			 * 	  This can be interpreted as a sort of "noise gate".
			 */
			if (*prev_prev_knob_scaled_val == scaled &&
			    *scaled_knob_val_changed &&
			    !EXCEEDS_THRESHOLD(
			        _prev_accepted_knob_analog_val,
			        _knob_analog_val,
			        ADC_KNOB_NOISE_GATE))
				continue;

			*scaled_knob_val_changed = true;
		} else {
			*scaled_knob_val_changed = false;
		}

		*prev_prev_knob_scaled_val = *prev_knob_scaled_val;
		*prev_knob_scaled_val = scaled;

		/*
		 * Send a MIDI CC message once the time is right.
		 * The SysTick decrementer is likely employed to
		 * prevent flooding the MIDI buffer.
		 */
		if (systick_decr_0 == 0) {

			if (knob_cc > MIDI_MAX_DATA_VAL)
				knob_cc = MIDI_MAX_DATA_VAL;

			uint8_t data[4] = {
				MIDI_CMD_CC_MSB | midi_ch,
				knob_cc, scaled,
				0x00
			};

			write_midi_buffer(&data, sizeof(data));
		}

		*prev_accepted_knob_analog_val = *knob_analog_val;
	}
}

/**
 * @ 0x080023fc
 * Progress: DONE
 */
void read_analog(void)
{
	uint16_t *prev_dma1_mem = UINT16_16_PTR_PREV_DMA1_MEM_0x2000057A;
	uint16_t *dma1_mem = UINT16_16_PTR_DMA1_MEM_0x2000013C;
	filtered_adc_val_t *filtered = FILTERED_ANALOG_INPUTS_PTR_0x2000059A;
	uint8_t *store_cntr = UINT8_PTR_DMA1_STORE_CNTR_0x2000003d;
	ready_t *knob_analog_rdy = UINT8_PTR_KNOB_ANALOG_VALS_READY_0x20000040;
	ready_t *pad_analog_rdy = UINT8_PTR_PAD_ANALOG_VALS_READY_0x20000041;

	const bool dma1_tcif1 = DMA_read_ISR(DMA_ISR_TCIF1);

	// Check if transfer not complete
	if (!dma1_tcif1)
		return;

	for (uint8_t i = 0; i < DMA1_NUM_DATA; i++) {
		const uint16_t dma_val = dma1_mem[i];

		/* Dispose top 4 bits since ADC only uses 12 bits
		 *
		 * Not sure why this is necessary? Perhaps the unused
		 * bits may contain garbage data when the ADC writes
		 * to memory via DMA?
		 *
		 * As an additional note, this could easily be done
		 * with a bit mask as well... */
		uint32_t dma_val_mut = (uint32_t)dma_val;
		dma_val_mut = dma_val_mut << 20;
		dma_val_mut = dma_val_mut >> 20;

		prev_dma1_mem[i] = (uint16_t)dma_val_mut;
	}

	(*store_cntr)++;

	if (store_cntr >= DMA1_STORE_INTERVAL) {
		*store_cntr = 0;

		for (uint8_t i = 0; i < DMA1_NUM_DATA; i++) {
			filtered[i] = prev_dma1_mem[i] >> 4;
			prev_dma1_mem[i] = 0;
		}

		*knob_analog_rdy = READY;
		*pad_analog_rdy = READY;
	}

	DMA_set_IFCR(DMA_IFCR_CTCIF1);
	ADC_set_EXTTRIG_SWSTART(ADC1_BASE, true);
}

/**
 * @ 0x08003130
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm ready flag,
 * 	 Confirm if midi output actually transmitts the MSB
 */
void eval_pads(void)
{
	const uint8_t sel_prog = *(uint8_t *)UINT8_PTR_SELECTED_PROG_0x20000042;
	const uint8_t *plus_1_max_127 = CONST_UINT8_127_PTR_LUT_1TO127_080056E4;
	const mode_t sel_mode = *(uint8_t *)UINT8_PTR_SELECTED_MODE_0x2000003f;

	uint8_t *pad_analog_rdy = UINT8_PTR_PAD_ANALOG_VALS_READY_0x20000041;
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_PTR_RELEASE_MIDI_0x2000052a;
	uint8_t *all_prog_settings = PROGRAM_SETTINGS_5_PTR_0x200003CC;
	program_settings *sel_prog_settings = &all_prog_settings[sel_prog];
	pad_states *pads_states = PAD_STATES_8_PTR_0x20000552;
	pad_handling_data *pads_hd = PAD_HANDLING_DATA_8_PTR_0x200004FA;
	filtered_analog_inputs *filtered_analog_inputs = FILTERED_ANALOG_INPUTS_PTR_0x2000059A;
	filtered_adc_val_t *pad_analog_vals = &filtered_analog_inputs->pads;

	uint8_t status_msbyte = 0;
	uint8_t data1 = 0;
	uint8_t data2_press = 0;
	uint8_t data2_release = 0;

	if (*pad_analog_rdy != READY)
		return;

	*pad_analog_rdy = NOT_READY;

	// Treat invalid MIDI channels as channel 0 (aka. 1)
	uint8_t midi_ch = sel_prog_settings->midi_ch;
	if (midi_ch > MIDI_MAX_CHANNEL)
		midi_ch = 0;

	// msg = in_r3; TODO Inspect
	for (uint8_t i = 0; i < N_PADS; i++) {

		pad_handling_data *pad_hd = &pads_hd[i];
		filtered_adc_val_t pad_analog_val = pad_analog_vals[i];
		pad_states *states = &pads_states[i];
		pad_settings *settings = &sel_prog_settings->pads[i];
		pending_midi *on_release_midi = &pads_on_release_midi[i];

		pad_confirmed_t changed = UNCHANGED;
		switch (pad_hd->last_confirmed_state) {
		case CONFIRMED_PRESSED:

			if (pad_analog_val <= PAD_analog_CONSIDERED_RELEASED) {

				if (pad_hd->rel_decr == 0) {
					pad_hd->last_confirmed_state = CONFIRMED_RELEASED;
					changed = CONFIRMED_RELEASED;
				} else {
					pad_hd->rel_decr--;
				}

				pad_hd->adc_eval = 0;
				pad_hd->press_incr = 0;
			} else {
				pad_hd->rel_decr = PAD_RELEASE_DECR_START;
			}
			break;

		case CONFIRMED_RELEASED:

			if (pad_analog_val < PAD_analog_CONSIDERED_PRESSED)
				break;

			if (pad_hd->press_incr == 0 ||
			    pad_hd->adc_eval < pad_analog_val) {
				pad_hd->adc_eval = pad_analog_val;
			}

			if (pad_hd->press_incr < PAD_PRESS_INCR_COMPLETE) {
				pad_hd->press_incr++;
			} else {
				pad_hd->last_confirmed_state = CONFIRMED_PRESSED;
				changed = CONFIRMED_PRESSED;

				if (pad_hd->adc_eval > MAX_ADC_PAD_VAL)
					pad_hd->adc_eval = MAX_ADC_PAD_VAL;

				/* Scale ADC value to MIDI range (0 - 127)
				 * Scaling is done with the following formula:
				 * 	scaled = ((a - (mm + 1)) / (ma - mm)) * mm
				 * Where:
				 * 	- mm is the maximum MIDI data value (127)
				 * 	- ma is the maximum allowed/possible ADC value (672)
				 * 		- Note that the lower limit is set by
				 * 		  PAD_analog_CONSIDERED_PRESSED, which is
				 * 		  mm + 2. This prevents a underflow.
				 * 	- a is the maximum recorded ADC value (caps at ma)
				 *
				 * (a - mm + 1) / (ma - mm) is the percentage, which is then
				 * multiplied by mm to get the scaled MIDI value (0 - 127)
				 *
				 * To avoid the need for floating point arithmetic, the formula
				 * has rewritten to:
				 * 	scaled = ((a - (mm + 1)) * mm) / (ma - mm)
				 * The product of (a - (mm + 1)) * mm is likely to be large, allowing
				 * better precision when dividing by (ma - mm) without the need for
				 * floating point arithmetic.
				 *
				 */
				const uint32_t num = (pad_hd->adc_eval - (MIDI_MAX_DATA_VAL + 1)) * MIDI_MAX_DATA_VAL;
				const uint16_t den = MAX_ADC_PAD_VAL - MIDI_MAX_DATA_VAL;
				pad_hd->adc_eval = num / den;
			}
			break;

		default:
			// TODO: Research if it is even possible to enter this branch
			pad_hd->adc_eval = 0;
			pad_hd->press_incr = 0;
			pad_hd->rel_decr = PAD_RELEASE_DECR_START;
		}

		switch (changed) {
		case CONFIRMED_RELEASED:

			if (sel_mode == MODE_PROG_CHNG) {
				states->pad_toggled = TOGGLED_OFF;
				break;
			}

			if (on_release_midi->pending != PENDING)
				break;

			bool is_momentary = settings->type != TOGGLE; // Toggle terminates on push
			bool is_prog = (sel_mode != MODE_CC && sel_mode != MODE_PAD);

			/* is_prog can only be true if user releases pad in PROG mode
			 * after switching from CC or PAD mode while holding the pad.
			 * If the pad is only pressed in PROG mode, on_release_midi->pending
			 * will never be PENDING.
			 *
			 * I speculate this is a fallback feature to ensure the NOTE or
			 * CC message has been terminated before changing the program.
			 *
			 * Pulling off this trick without changing the program and
			 * and returning back to the prior mode can result in duplicate
			 * MIDI messages being sent. This can be achieved with all modes,
			 * including PROG_CHNG.
			 */

			if (is_momentary || is_prog) {
				on_release_midi->pending = NOT_PENDING;

				if (on_release_midi->data1 <= MIDI_MAX_DATA_VAL) {
					states->prog_chng = PAD_STATE_RELEASED;

					if (sel_mode == MODE_PAD)
						states->pad = PAD_STATE_RELEASED;
					else if (sel_mode == MODE_CC)
						states->cc = PAD_STATE_RELEASED;

					write_midi_buffer(&on_release_midi->cmd_msb, 4); // TODO: Check if cmd_msb really is transfered
				}
			}

			break;

		case CONFIRMED_PRESSED:

			uint32_t _pressure_midi;

			if (pad_hd->adc_eval <= MIDI_MAX_DATA_VAL)
				_pressure_midi = pad_hd->adc_eval;
			else
				_pressure_midi = MIDI_MAX_DATA_VAL;

			/*
			 * Redundant as guards above and below already ensure this
			 * Maybe this was intended for a conversion to a non-linear scale?
			 * Kept for authenticity sake
			 */
			midi_data_t pressure_midi = plus_1_max_127[_pressure_midi];

			if (pressure_midi == 0) // Should never happen due to line above...
				pressure_midi = 1;
			else if (pressure_midi > MIDI_MAX_DATA_VAL) // Again redundant...
				pressure_midi = MIDI_MAX_DATA_VAL;

			// Prepare MIDI message

			switch (sel_mode) {
			case MODE_PAD:
				status_msbyte = MIDI_CMD_NOTE_ON_MSB;
				data1 = settings->note;
				data2_press = pressure_midi;
				data2_release = PAD_MODE_RELEASE_DATA2;
			case MODE_CC:
				status_msbyte = MIDI_CMD_CC_MSB;
				data1 = settings->cc;
				data2_press = pressure_midi;
				data2_release = CC_MODE_RELEASE_DATA2;
			case MODE_PROG_CHNG:
				status_msbyte = MIDI_CMD_PROG_CHNG_MSB;
				data1 = settings->pc;
				data2_press = PROG_CHNG_MODE_PRESS_DATA2;
				data2_release = PROG_CHNG_MODE_RELEASE_DATA2;
			case MODE_PROG:
				continue;
			}

			uint8_t msg[4] = {status_msbyte,
			                  (status_msbyte << 4) | midi_ch,
			                  data1,
			                  data2_press
			                 };

			states->prog_chng = PAD_STATE_PRESSED;

			pad_toggled_t toggled;
			if (sel_mode == MODE_PAD) {
				states->pad = PAD_STATE_PRESSED;

				if (settings->type == TOGGLE) {
					states->pad_toggled = !states->pad_toggled;
					toggled = states->pad_toggled;

					if (toggled == TOGGLED_OFF) {
						msg[0] = MIDI_CMD_NOTE_OFF_MSB;
						msg[1] = (MIDI_CMD_NOTE_OFF_MSB << 4) | midi_ch;
						msg[2] = data1;
						msg[3] = data2_release;
					}
				}
			} else if (sel_mode == MODE_CC) {
				states->cc = PAD_STATE_PRESSED;

				if (settings->type == TOGGLE) {
					states->cc_toggled = !states->cc_toggled;
					toggled = states->cc_toggled;

					if (toggled == TOGGLED_OFF) {
						msg[3] = CC_MODE_RELEASE_DATA2;
					}
				}
			}

			if (toggled == TOGGLED_OFF) {
				states->prog_chng = PAD_STATE_RELEASED;

				if (sel_mode == MODE_PAD)
					states->pad = PAD_STATE_RELEASED;
				else if (sel_mode == MODE_CC)
					states->cc = PAD_STATE_RELEASED;
			}

			if (status_msbyte == MIDI_CMD_NOTE_ON_MSB) {
				on_release_midi->cmd_msb = MIDI_CMD_NOTE_OFF_MSB;
				on_release_midi->cmd = midi_ch | (MIDI_CMD_NOTE_OFF_MSB << 4);
			} else {
				on_release_midi->cmd_msb = status_msbyte;
				on_release_midi->cmd = msg[1];
			}

			on_release_midi->pending = PENDING;
			on_release_midi->data1 = data1;
			on_release_midi->data2 = data2_release;

			if (data1 <= MIDI_MAX_DATA_VAL)
				write_midi_buffer(msg, sizeof(msg));

			break;

		case UNCHANGED:
		default:
			break;
		}
	}
}

/**
 * @ 0x08003d10
 * Progress: DONE
 */
void read_mode_pbs(void)
{
	uint8_t *systick_decr = UINT8_PTR_SYSTICK_DECREMENTER_1_0x2000003c;
	uint8_t *debounce = UINT8_PTR_DEBOUNCE_COUNTER_0x2000003e;
	uint16_t *prev_mode_pbs = UINT16_PTR_PREV_MODE_PB_IDR_BITS_0x20000044;
	uint16_t *mode_pbs = UINT16_PTR_MODE_PB_IDR_BITS_CPY_0x20000048;

	// Mode Push buttons are read every 4 SysTick intervals.
	if (*systick_decr != 0)
		return;

	*systick_decr = MODE_PB_SYSTICK_SCAN_INTERVALS;

	// Read mode push buttons. Negation due to PU's.
	const uint32_t _mode_pbs = ~(PB_GPIO_PORT->IDR) & PB_IDR_MSK;
	if (*prev_mode_pbs != _mode_pbs) {
		*debounce = 0;
		*prev_mode_pbs = _mode_pbs;
		return;
	}

	/*
	 * Debouncing/Bounce-filtering?
	 * The following code likely ensures stable mode push button states
	 * by applying changes only if the buttons have maintained the same
	 * state for at least MODE_PB_DEBOUNCE_THRESHOLD executions.
	 *
	 * I'm uncertain why the debounce counter reaches 240 even tough it's
	 * only checked against MODE_PB_DEBOUNCE_THRESHOLD. A simpler approach
	 * would be to count up to MODE_PB_DEBOUNCE_THRESHOLD directly and
	 * cease incrementing when the threshold is reached.
	 * The debounce counter is also not used anywhere else...
	 * Perhaps this is some compiler bogus related to the data type of the
	 * debounce counter? Not sure...
	 */
	if (*debounce < 240) {
		if (++(*debounce) == MODE_PB_DEBOUNCE_THRESHOLD)
			*mode_pbs = _mode_pbs;
	}
}

/**
 * @ 0x08003c80
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Resolve unknown_flag and what the SYSEx message does
 */
void eval_mode_pbs(void)
{
	const uint16_t mode_pbs = *(uint16_t *)UINT16_PTR_MODE_PB_IDR_BITS_CPY_0x20000048;
	const uint8_t unknown_flag = *(uint8_t *)UINT8_PTR_UNKNOWN_FLAG_0x20000011;

	unknown *prev_mode_pbs = UINT16_PTR_PREV_MODE_PB_IDR_BITS_0x20000046;
	mode_t *selected_mode = UINT8_PTR_SELECTED_MODE_0x2000003f;

	if (*prev_mode_pbs != mode_pbs) {

		*prev_mode_pbs = mode_pbs;

		// Suspecting this is some sort of debug flag...
		if (unknown_flag != 0) {

			// Shift bits to start at bit 0. PROG is first GPIO.
			uint8_t shifted2lsbits = (mode_pbs >> PB_PROG_GPIO);

			// Terminates some sort of SysEx message that conveys mode pb gpio info??!
			uint8_t data[12] = {0x04, 0x47, 0x00, 0x75, 0x04, 0x6b, 0x00, 0x02,
			                    0x07, 0x5a, shifted2lsbits, 0xf7
			                   };

			return;
		}

		// Real code uses shift to MSB and then checks if negative
		// Masks achieve the same result and are more readable
		if (mode_pbs & (1 << PB_PROG_GPIO))
			*selected_mode = MODE_PROG;
		else if (mode_pbs & (1 << PB_PAD_GPIO))
			*selected_mode = MODE_PAD;
		else if (mode_pbs & (1 << PB_PROG_CHNG_GPIO))
			*selected_mode = MODE_PROG_CHNG;
		else if (mode_pbs & (1 << PB_CC_GPIO))
			*selected_mode = MODE_CC;
	}
}

/**
 * @ 0x0800442c
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Resolve pad_toggled and cc_toggled fields of pad_states struct
 */
void rst_pads(void)
{
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_PTR_RELEASE_MIDI_0x2000052a; // -> pending_midi[N_PADS]
	pad_states *pads_states = PAD_STATES_8_PTR_0x20000552;			     // -> pad_states[N_PADS]

	for (uint8_t i = 0; i < N_PADS; i++) {
		if (pads_on_release_midi[i].pending == PENDING) {
			pads_on_release_midi[i].pending = NOT_PENDING;
			pads_states[i].pad_toggled = false;
			pads_states[i].cc_toggled = false;
			pads_states[i].prog_chng = PAD_STATE_RELEASED;
			pads_states[i].pad = PAD_STATE_RELEASED;
			pads_states[i].cc = PAD_STATE_RELEASED;

			if (pads_on_release_midi[i].cmd >> 4 == MIDI_CMD_NOTE_OFF_MSB &&
			    pads_on_release_midi[i].data1 <= MIDI_MAX_DATA_VAL) {
				// Write NOTE OFF for pad to midi buffer
				write_midi_buffer(&(pads_on_release_midi[i].cmd_msb), 4);
			}
		}
	}
}

/**
 * @ 0x08003420
 * Progress: DONE
 *
 * Called by `update_leds`, this function manages and updates the LEDs
 * for all 8 pads. It handles momentary LED toggles when pads are pressed
 * and released. The only exception is during PROGRAM mode (where LEDs are
 * continuously held ON for the selected program), which is managed in
 * the `update_leds` function instead.
 */
void update_pad_leds()
{
	const mode_t selected_mode = *(uint8_t *)UINT8_PTR_SELECTED_MODE_0x2000003f; // uint8_t

	mode_t *prev_mode = UINT8_PTR_PREV_MODE_0x20000031;			   // uint8_t
	pad_state_t *prev_pads_state = UINT8_8_PTR_PREV_PADS_STATE_0x20000033; // -> pad_state_t[N_PADS]
	pad_states *pads_states = PAD_STATES_8_PTR_0x20000552;		   // -> pad_states[N_PADS]

	// For every Pad
	for (uint8_t i = 0; i < N_PADS; i++) {
		pad_state_t pad_state;

		switch (selected_mode) {
		case MODE_PAD:
			pad_state = pads_states[i].pad;
			break;
		case MODE_CC:
			pad_state = pads_states[i].cc;
			break;
		case MODE_PROG_CHNG:
			pad_state = pads_states[i].prog_chng;
			break;
		default:
			for (uint8_t j = 0; j < N_PADS; j++) {
				prev_pads_state[j] = PAD_STATE_UNSET;
			}

			return; // PROGRAM mode already deals with leds in update_leds
		}

		// Only update pad LED if pad_state has changed or mode has changed
		if (prev_pads_state[i] != pad_state && *prev_mode != selected_mode) {
			prev_pads_state[i] = pad_state;
			*prev_mode = selected_mode;

			if (pad_state == PAD_STATE_PRESSED) {
				switch (i) {
				case 0:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_1_GPIO);
					break;
				case 1:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_2_GPIO);
					break;
				case 2:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_3_GPIO);
					break;
				case 3:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_4_GPIO);
					break;
				case 4:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_5_GPIO);
					break;
				case 5:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_6_GPIO);
					break;
				case 6:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_7_GPIO);
					break;
				case 7:
					LED_GPIO_PORT->ODR |= (1 << LED_PAD_8_GPIO);
					break;
				default:
					break;
				}
			} else {
				switch (i) {
				case 0:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_1_GPIO);
					break;
				case 1:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_2_GPIO);
					break;
				case 2:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_3_GPIO);
					break;
				case 3:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_4_GPIO);
					break;
				case 4:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_5_GPIO);
					break;
				case 5:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_6_GPIO);
					break;
				case 6:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_7_GPIO);
					break;
				case 7:
					LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_8_GPIO);
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
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Resolve unknown_flag, unknown_enum and uknown0 and cc_toggled fields
 *       of the pad_states struct

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
	const uint8_t unknown_flag = *(uint8_t *)UINT8_PTR_UNKNOWN_FLAG_0x20000011; // uint8_t
	const mode_t selected_mode = *(mode_t *)UINT8_PTR_SELECTED_MODE_0x2000003f; // uint8_t
	const int unknown_enum = *(int *)UNKNOWN_ENUM_PTR_0x20000012;		// unknown

	mode_t *prev_mode = UINT8_PTR_PREV_MODE_0x20000043;				     // -> uint8_t
	uint8_t *selected_prog = UINT8_PTR_SELECTED_PROG_0x20000042;		     // -> uint8_t
	uint8_t *prev_selected_prog = UINT8_PTR_PREV_SELECTED_PROG_0x20000032;	     // -> uint8_t
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_PTR_RELEASE_MIDI_0x2000052a; // -> pending_midi[N_PADS]
	pad_states *pads_states = PAD_STATES_8_PTR_0x20000552;			     // -> pad_states[N_PADS]

	// I have yet to find out what sets this flag != 0
	if (unknown_flag == 0) {
		/* Check if mode has been switched */
		if (*prev_mode != selected_mode) {
			// Clear PB Pad LEDs
			LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PAD_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PB_CC_GPIO);

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
			for (uint8_t i = 0; i < N_PADS; i++) {
				pads_states[i].prog_chng = 0;
			}
			*prev_mode = selected_mode;
		}

		// Set LEDs according to selected mode
		switch (selected_mode) {
		case MODE_PAD:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_PAD_GPIO);
			break;
		case MODE_CC:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_CC_GPIO);
			break;
		case MODE_PROG_CHNG:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_PROG_CHNG_GPIO);
			break;
		case MODE_PROG:
			// Clear all Pad LEDs
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_1_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_2_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_3_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_4_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_5_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_6_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_7_GPIO);
			LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_8_GPIO);

			if ((uint8_t *)PROG_1_SELECT_FLAG == 1)
				*selected_prog = 1;
			else if ((uint8_t *)PROG_2_SELECT_FLAG == 1)
				*selected_prog = 2;
			else if ((uint8_t *)PROG_3_SELECT_FLAG == 1)
				*selected_prog = 3;
			else if ((uint8_t *)PROG_4_SELECT_FLAG == 1)
				*selected_prog = 4;

			if (*prev_selected_prog != *selected_prog) {
				rst_pads();

				for (uint8_t i = 0; i < N_PADS; i++) {
					pads_on_release_midi[i].pending = NOT_PENDING;
					pads_states[i].pad_toggled = false;
					pads_states[i].cc_toggled = false;
					pads_states[i].prog_chng = PAD_STATE_RELEASED;
					pads_states[i].pad = PAD_STATE_RELEASED;
					pads_states[i].cc = PAD_STATE_RELEASED;
				}
			}

			switch (*selected_prog) {
			case 1:
				LED_GPIO_PORT->ODR |= (1 << LED_PAD_1_GPIO);
				break;
			case 2:
				LED_GPIO_PORT->ODR |= (1 << LED_PAD_2_GPIO);
				break;
			case 3:
				LED_GPIO_PORT->ODR |= (1 << LED_PAD_3_GPIO);
				break;
			case 4:
				LED_GPIO_PORT->ODR |= (1 << LED_PAD_4_GPIO);
				break;
			default:
				break;
			}

		default:
			break;
		}

		update_pad_leds();
	} else {
		/* Clear all LEDs */
		LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PAD_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PB_CC_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_1_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_2_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_3_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_4_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_5_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_6_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_7_GPIO);
		LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_8_GPIO);

		switch (unknown_enum) {
		case 1:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_PAD_GPIO);
			break;
		case 2:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_CC_GPIO);
			break;
		case 3:
			LED_GPIO_PORT->ODR |= (1 << LED_PB_PROG_CHNG_GPIO);
			break;
		case 4:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_1_GPIO);
			break;
		case 5:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_2_GPIO);
			break;
		case 6:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_3_GPIO);
			break;
		case 7:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_4_GPIO);
			break;
		case 8:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_5_GPIO);
			break;
		case 9:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_6_GPIO);
			break;
		case 10:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_7_GPIO);
			break;
		case 11:
			LED_GPIO_PORT->ODR |= (1 << LED_PAD_8_GPIO);
			break;
		default:
			break;
		}
	}
	*prev_selected_prog = *selected_prog;
}

/**
 * @ 0x08003a94
 * Progress: DONE
 */
void reload_IWDG(void)
{
	IWDG->KR = IWDG_KEY_RELOAD;
}

/**
 * @0x008002584
 * Progress: DONE
 */
void deinit_midi_buffer(void)
{
	uint8_t *midi_buf_rem_space = UINT8_PTR_MIDI_BUFFER_REMAINING_SPACE_0x20000004;
	uint8_t **midi_buf_tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c;
	uint8_t **midi_buf_head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008;

	*midi_buf_rem_space = 0;
	*midi_buf_tail = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
	*midi_buf_head = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
}

/**
 * @ 0x08004ef0
 * Progress: INCOMPLETE
 * TODO: FUN_0800468e
 */
void init(void)
{
	uint8_t *unknown_flag_0 = UINT8_PTR_UNKNOWN_FLAG_0x20000000;

	FUN_0800468e();
	nvic_init_firmware_VTOR();
	init_GPIOs();
	init_SYSTICK();
	init_analog();
	init_usb();
	init_midi_buffer();
	init_IWDG();

	*unknown_flag_0 = 0;
}

/**
 * @ 0x08005550
 * Progress: INCOMPLETE
 */
void main_loop()
{
	uint8_t *unknown_flag_0 = UINT8_PTR_UNKNOWN_FLAG_0x20000000;
	uint32_t *unknown_1 = UINT32_PTR_PTR_UNKNOWN_0x20000098;
	mode_t *prev_mode = UINT8_PTR_PREV_MODE_0x20000043;

	FUN_08004ef0();
	FUN_08005318();

	while (true) {
		while (true) {
			while (true) {
				reload_IWDG();

				if (*unknown_flag_0 == 0)
					break;

				*prev_mode = MODE_UNSET;

				LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PAD_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PB_PROG_CHNG_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PB_CC_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_1_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_2_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_3_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_4_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_5_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_6_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_7_GPIO);
				LED_GPIO_PORT->ODR &= ~(1 << LED_PAD_8_GPIO);
			}

			read_analog();
			eval_knobs();
			eval_pads();
			read_mode_pbs();
			eval_mode_pbs();
			update_leds();

			if (*unknown_1 == 0)
				break;

			FUN_08002790();
			FUN_08003d5c();
		}

		deinit_midi_buffer();
	}
}

/**
 * @ 0x080049b8
 * Progress: INCOMPLETE
*/
void FUN_080049b8(uint32_t param_1)
{
	// uint uVar1;
	// byte *prog_settings;

	// if (param_1 < 5) {
	// 	prog_settings = (byte *)(param_1 * 0x39 + PROGRAM_SETTINGS_5_PTR_0x200003CC);
	// 	if (0xf < *prog_settings) {
	// 		*prog_settings = 0xf;
	// 	}
	// 	i = 0;
	// 	do {
	// 		if (0x7f < prog_settings[1 + i * 4]) {
	// 			prog_settings[1 + i * 4] = 0x7f;
	// 		}
	// 		if (0x7f < prog_settings[2 + i * 4]) {
	// 			prog_settings[2 + i * 4] = 0x7f;
	// 		}
	// 		if (0x7f < prog_settings[3 + i * 4]) {
	// 			prog_settings[3 + i * 4] = 0x7f;
	// 		}
	// 		if (1 < prog_settings[4 + i * 4]) {
	// 			prog_settings[4 + i * 4] = 1;
	// 		}
	// 		i = i + 1 & 0xff;
	// 	} while (i < 8);
	// 	i = 0;
	// 	do {
	// 		if (0x7f < prog_settings[i * 3 + 0x21]) {
	// 			prog_settings[i * 3 + 0x21] = 0x7f;
	// 		}
	// 		if (0x7f < prog_settings[i * 3 + 0x22]) {
	// 			prog_settings[i * 3 + 0x22] = 0;
	// 		}
	// 		if (0x7f < prog_settings[i * 3 + 0x23]) {
	// 			prog_settings[i * 3 + 0x23] = 0x7f;
	// 		}
	// 		i = i + 1 & 0xff;
	// 	} while (i < 8);
	// }
	// return;

	if (param_1 >= 5)
		return;

	program_settings *all_prog_settings = PROGRAM_SETTINGS_5_PTR_0x200003CC;
	program_settings *sel_prog_settings = &all_prog_settings[param_1];

	if (sel_prog_settings->midi_ch > MIDI_MAX_CHANNEL)
		sel_prog_settings->midi_ch = MIDI_MAX_CHANNEL;

	for (uint8_t i = 0; i < N_PADS; i++) {
		pad_settings *psettings = &sel_prog_settings->pads[i];

		if (psettings->note > MIDI_MAX_DATA_VAL)
			psettings->note = MIDI_MAX_DATA_VAL;

		if (psettings->cc > MIDI_MAX_DATA_VAL)
			psettings->cc = MIDI_MAX_DATA_VAL;

		if (psettings->pc > MIDI_MAX_DATA_VAL)
			psettings->pc = MIDI_MAX_DATA_VAL;

		if (psettings->type > TOGGLE)
			psettings->type = TOGGLE;
	}

	for (uint8_t i = 0; i < N_PADS; i++) {
		knob_settings *ksettings = &sel_prog_settings->knobs[i];

		if (ksettings->cc > MIDI_MAX_DATA_VAL)
			ksettings->cc = MIDI_MAX_DATA_VAL;

		if (ksettings->leftmost > MIDI_MAX_DATA_VAL)
			ksettings->leftmost = MIDI_MAX_DATA_VAL;

		if (ksettings->rightmost > MIDI_MAX_DATA_VAL)
			ksettings->rightmost = MIDI_MAX_DATA_VAL;
	}