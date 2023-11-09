#include <stdint.h>
#include <stdbool.h>

#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>

#define UINT8_UNKNOWN_FLAG_20000000 0x20000000
#define UINT8_MIDI_BUFFER_REMAINING_SPACE_20000004 0x20000004
#define UINT8_PTR_PTR_MIDI_BUFFER_HEAD_20000008 0x20000008
#define UINT8_PTR_PTR_MIDI_BUFFER_TAIL_2000000c 0x2000000c // buffer_end - MIDI_BUFFER_SIZE
#define UINT8_UNKNOWN_FLAG_20000011 0x20000011
#define UNKNOWN_ENUM_20000012 0x20000012
#define UINT8_PREV_MODE_0x20000031 0x20000031
#define UINT8_PREV_SELECTED_PROG_20000032 0x20000032
#define UINT8_8_PREV_PADS_STATE_20000033 0x20000033 // .. 0x2000003A
#define UINT8_UNKNOWN_COUNTER_2000003c 0x2000003c
#define UINT8_UNKNOWN_COUNTER_2000003e 0x2000003e
#define UINT8_SELECTED_MODE_2000003f 0x2000003f
#define UINT8_SELECTED_PROG_20000042 0x20000042
#define UINT8_PREV_MODE_20000043 0x20000043
#define UINT16_PREV_MODE_PADS_IDR_BITS_20000044 0x20000044
#define UINT16_PREV_MODE_PADS_IDR_BITS_20000046 0x20000046
#define UINT16_MODE_PB_IDR_BITS_CPY_20000048 0x20000048
#define UINT8_UNKNOWN_FLAG_20000098 0x20000098
#define UINT8_MIDI_BUFFER_END_2000024C 0x2000024C
#define PAD_MIDI_8_0x2000052a 0x2000052a
#define PAD_STATES_8_20000552 0x20000552

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

#define PB_GPIO_PORT GPIOC
#define PB_PROG_GPIO 6
#define PB_PAD_GPIO 7
#define PB_PROG_CHNG_GPIO 8
#define PB_CC_GPIO 9
#define PB_IDR_MSK (1 << PB_PROG_GPIO) | (1 << PB_PAD_GPIO) | (1 << PB_PROG_CHNG_GPIO) | (1 << PB_CC_GPIO)

#define ADC_CR1_OFFSET 0x4
#define ADC_CR2_OFFSET 0x8
#define ADC_SMPR1_OFFSET 0x0c
#define ADC_SMPR2_OFFSET 0x10
#define ADC_SQR1_OFFSET 0x2c
#define ADC_SQR2_OFFSET 0x30
#define ADC_SQR3_OFFSET 0x34
#define GPIO_CRL_OFFSET 0x0
#define GPIO_CRH_OFFSET 0x4
#define GPIO_CR_CNF_MODE 0xF

#define MIDI_MAX_DATA_VAL 127 // 0x7F
#define MIDI_BUFFER_SIZE 240  // 0xF0

typedef uint32_t unknown; // So the linter doesn't complain

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

typedef struct
{
	pad_state_t state;
	midi_cmd_msb_t cmd_msb;
	uint8_t cmd;
	uint8_t data1;
	uint8_t data2;
} pad_midi;

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
void ADC_set_DUALMOD_SCAN_CONT_ALIGN_EXTSEL_nconv(
    uint32_t adc_base,
    uint32_t msk_DUALMOD, bool SCAN,
    bool CONT, uint32_t msk_ALIGN, uint32_t msk_EXTSEL,
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
void DMA_set_DIR_CIRC_PINC_MINC_PSIZE_MSIZE_PL_MEM2MEM_CNDTR_CPAR_CMAR(
    uint32_t *dma_ccr,
    uint32_t cpar, uint32_t cmar,
    uint32_t dir_msk, uint32_t cndtr,
    uint32_t pinc_msk, uint32_t minc_msk,
    uint32_t psize_msk, uint32_t msize_msk,
    uint32_t circ_msk, uint32_t pl_msk,
    uint32_t mem2mem_msk)
{
	uint32_t *dma_cndtr = dma_ccr + 0x4;
	uint32_t *dma_cpar = dma_cndtr + 0x4;
	uint32_t *dma_cmar = dma_cpar + 0x4;

	*dma_ccr &= ~(DMA_CCR_DIR |
		      DMA_CCR_CIRC |
		      DMA_CCR_PINC |
		      DMA_CCR_MINC |
		      DMA_CCR_PSIZE |
		      DMA_CCR_MSIZE |
		      DMA_CCR_PL |
		      DMA_CCR_MEM2MEM);

	*dma_ccr |= dir_msk |
		    circ_msk |
		    pinc_msk |
		    minc_msk |
		    psize_msk |
		    msize_msk |
		    pl_msk |
		    mem2mem_msk;

	*dma_cndtr = cndtr;
	*dma_cpar = cpar;
	*dma_cmar = cmar;
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
 */
void ADC_set_SMPR_SQR(uint32_t adc_base, uint8_t channel, uint8_t nth_conv, uint32_t smp_bits)
{
	uint32_t *ADC_SMPR1 = adc_base + ADC_SMPR1_OFFSET;
	uint32_t *ADC_SMPR2 = adc_base + ADC_SMPR2_OFFSET;
	uint32_t *ADC_SQR1 = adc_base + ADC_SQR1_OFFSET;
	uint32_t *ADC_SQR2 = adc_base + ADC_SQR2_OFFSET;
	uint32_t *ADC_SQR3 = adc_base + ADC_SQR3_OFFSET;

	if (channel < 10)
	{
		*ADC_SMPR2 &= ~(ADC_SMPR2_SMP0 << (channel * 3));
		*ADC_SMPR2 |= smp_bits << (channel * 3);
	}
	else
	{
		*ADC_SMPR1 &= ~(ADC_SMPR1_SMP10 << ((channel - 10) * 3));
		*ADC_SMPR1 |= smp_bits << ((channel - 10) * 3);
	}

	if (nth_conv < 7)
	{
		*ADC_SQR3 &= ~(ADC_SQR3_SQ1 << ((nth_conv - 1) * 5));
		*ADC_SQR3 |= channel << ((nth_conv - 1) * 5);
	}
	else if (nth_conv < 13)
	{
		*ADC_SQR2 &= ~(ADC_SQR2_SQ7 << ((nth_conv - 7) * 5));
		*ADC_SQR2 |= channel << ((nth_conv - 7) * 5);
	}
	else
	{
		*ADC_SQR1 &= ~(ADC_SQR1_SQ13 << ((nth_conv - 0xd) * 5));
		*ADC_SQR1 |= channel << ((nth_conv - 0xd) * 5);
	}
}

/**
 * @ 0x080039aa
 * Progress: INCOMPLETE
 * TODO: Resolve unknown_ptr_0, understand cnfd_mode_bits and declare
 * 	 defines for GPIO reg offsets
 */

//   local_34[0] = 0xff;
//   local_2c = 0;
//   FUN_080039aa(DAT_080023b4,local_34);
//   local_34[0] = 0xf;
//   local_2c = 0;
//   FUN_080039aa(DAT_080023b8,local_34);
//   local_34[0] = 0x3c;
//   local_2c = 0;
//   FUN_080039aa(DAT_080023bc,local_34);

//                      DAT_080023b4                                    XREF[1]:     init_dma_adc:080021f4(R)
// 080023b4 00 08 01 40     undefined4 40010800h
//                      DAT_080023b8                                    XREF[1]:     init_dma_adc:08002204(R)
// 080023b8 00 0c 01 40     undefined4 40010C00h
//                      DAT_080023bc                                    XREF[1]:     init_dma_adc:08002214(R)
// 080023bc 00 10 01 40     undefined4 40011000h
void FUN_080039aa(uint32_t *gpio_base, uint16_t *gpio_msk, uint16_t *cnf_mode_msk)
{
	uint32_t *unknown_ptr_0 = gpio_msk + 4;
	uint8_t cnf_mode_bits = *unknown_ptr_0 & 0xf;

	uint32_t *GPIO_BSRR = gpio_base + 0x10;
	uint32_t *GPIO_BRR = gpio_base + 0x14;
	uint32_t *GPIO_CRL = gpio_base + 0x00;
	uint32_t *GPIO_CRH = gpio_base + 0x04;

	// Check MSB
	if ((*unknown_ptr_0 << 0x1b) < 0)
	{
		cnf_mode_bits |= *cnf_mode_msk;
	}

	if (*gpio_msk & 0xFF)
	{ // Check any of GPIO's 0-7 are selected in the mask

		uint32_t crl = *GPIO_CRL;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++)
		{

			uint8_t msk = 1 << i_gpio;

			if ((*gpio_msk & msk))
			{
				crl &= ~(GPIO_CR_CNF_MODE << (i_gpio * 4));
				crl |= cnf_mode_bits << (i_gpio * 4);

				if (*unknown_ptr_0 == 0x28)
					*GPIO_BRR = msk;
				else if (*unknown_ptr_0 == 0x48)
					*GPIO_BSRR = msk;
			}
		}

		*GPIO_CRL = crl;
	}

	if (*gpio_msk > 0xFF)
	{ // Check any of GPIO's 8-15 are selected in the mask

		uint32_t crh = *GPIO_CRH;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++)
		{

			uint8_t msk = 1 << (i_gpio + 8);

			if ((*gpio_msk & msk))
			{
				crh &= ~(GPIO_CR_CNF_MODE << (i_gpio * 4));
				crh |= cnf_mode_bits << (i_gpio * 4);

				if (*unknown_ptr_0 == 0x28)
					*GPIO_BRR = msk;
				else if (*unknown_ptr_0 == 0x48)
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

	uint8_t *remaining_space = UINT8_MIDI_BUFFER_REMAINING_SPACE_20000004;
	uint8_t *buffer_end = UINT8_MIDI_BUFFER_END_2000024C;
	uint8_t **head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_20000008;
	uint8_t **tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_2000000c; // buffer_end - MIDI_BUFFER_SIZE

	for (uint8_t i = 0; i < size; i++)
	{
		uint8_t *next_element = *head + 1;

		// When buffer_end is reached, continue buffer
		// writing at buffer_end - MIDI_BUFFER_SIZE
		if (next_element == buffer_end)
		{
			next_element = *tail; // 0x2000015C
		}

		// Buffer is full
		if (next_element == *tail)
			break;

		// Store data into buffer and move to next element
		**head = *(uint8_t *)data;
		*head = next_element;

		// Move to next byte of data
		data = data + 1;
	}

	const uint32_t last_element_addr = *head;
	const uint32_t tail_addr = *tail;

	if (*tail <= *head)
		*remaining_space = (tail_addr - last_element_addr) - 0x10;
	else
		*remaining_space = last_element_addr - tail_addr;
}

/**
 * @ 0x08003d10
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Resolve unknown_counter_0 and unknown_counter_1
 *       1 is likely used for debouncing,
 * 	 not so sure about 0
 */
void read_mode_pbs(void)

{
	uint8_t *unknown_counter_0 = UINT8_UNKNOWN_COUNTER_2000003c;
	uint8_t *unknown_counter_1 = UINT8_UNKNOWN_COUNTER_2000003e;
	uint16_t *prev_mode_pbs = UINT16_PREV_MODE_PADS_IDR_BITS_20000044;
	uint16_t *mode_pbs = UINT16_MODE_PB_IDR_BITS_CPY_20000048;

	if (*unknown_counter_0 == 0)
	{
		// Counter is likely  decremented by interrupt (IVT 0x0800203c, ISR 0x08004e7c)
		*unknown_counter_0 = 4;

		// Read mode push buttons. Negation due to PU's.
		const uint32_t _mode_pbs = ~(PB_GPIO_PORT->IDR) & PB_IDR_MSK;
		if (*prev_mode_pbs != _mode_pbs)
		{
			*unknown_counter_1 = 0;
			*prev_mode_pbs = _mode_pbs;
			return;
		}

		// Debouncing?
		if (*unknown_counter_1 < 240) // TODO: Determine how much this is in us/ms...
		{
			// Apply state only after 9 counts
			if (*unknown_counter_1 == 9) // TODO: Determine how much this is in us/ms...
				*mode_pbs = _mode_pbs;

			*unknown_counter_1++;
		}
	}
}

/**
 * @ 0x08003c80
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Resolve unknown_flag and what the SYSEx message does
 */
void eval_mode_pbs(void)
{
	const uint16_t mode_pbs = *(uint16_t *)UINT16_MODE_PB_IDR_BITS_CPY_20000048;
	const uint8_t unknown_flag = *(uint8_t *)UINT8_UNKNOWN_FLAG_20000011;

	unknown *prev_mode_pbs = UINT16_PREV_MODE_PADS_IDR_BITS_20000046;
	mode_t *selected_mode = UINT8_SELECTED_MODE_2000003f;

	if (*prev_mode_pbs != mode_pbs)
	{

		*prev_mode_pbs = mode_pbs;

		// Suspecting this is some sort of debug flag...
		if (unknown_flag != 0)
		{

			// Shift bits to start at bit 0. PROG is first GPIO.
			uint8_t shifted2lsbits = (mode_pbs >> PB_PROG_GPIO);

			// Terminates some sort of SysEx message that conveys mode pb gpio info??!
			uint8_t data[12] = {0x04, 0x47, 0x00, 0x75, 0x04, 0x6b, 0x00, 0x02,
					    0x07, 0x5a, shifted2lsbits, 0xf7};

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
 * TODO: Resolve unknown0 and unknown1 fields of pad_states struct
 */
void rst_pads(void)
{
	pad_midi *pads_midi = PAD_MIDI_8_0x2000052a;	 // -> pad_midi[8]
	pad_states *pads_states = PAD_STATES_8_20000552; // -> pad_states[8]

	for (uint8_t i = 0; i < 8; i++)
	{
		if (pads_midi[i].state == PAD_STATE_PRESSED)
		{
			pads_midi[i].state = PAD_STATE_RELEASED;
			pads_states[i].unknown0 = 0;
			pads_states[i].unknown1 = 0;
			pads_states[i].prog_chng = PAD_STATE_RELEASED;
			pads_states[i].pad = PAD_STATE_RELEASED;
			pads_states[i].cc = PAD_STATE_RELEASED;

			if (pads_midi[i].cmd >> 4 == MIDI_CMD_NOTE_OFF_MSB &&
			    pads_midi[i].data1 <= MIDI_MAX_DATA_VAL)
			{
				// Write NOTE OFF for pad to midi buffer
				write_midi_buffer(&(pads_midi[i].cmd_msb), 4);
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
	const mode_t selected_mode = *(uint8_t *)UINT8_SELECTED_MODE_2000003f; // uint8_t

	mode_t *prev_mode = UINT8_PREV_MODE_0x20000031;			 // uint8_t
	pad_state_t *prev_pads_state = UINT8_8_PREV_PADS_STATE_20000033; // -> pad_state_t[8]
	pad_states *pads_states = PAD_STATES_8_20000552;		 // -> pad_states[8]

	// For every Pad
	for (uint8_t i = 0; i < 8; i++)
	{
		pad_state_t pad_state;

		switch (selected_mode)
		{
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
			for (uint8_t j = 0; j < 8; j++)
			{
				prev_pads_state[j] = PAD_STATE_UNSET;
			}

			return; // PROGRAM mode already deals with leds in update_leds
		}

		// Only update pad LED if pad_state has changed or mode has changed
		if (prev_pads_state[i] != pad_state && *prev_mode != selected_mode)
		{
			prev_pads_state[i] = pad_state;
			*prev_mode = selected_mode;

			if (pad_state == PAD_STATE_PRESSED)
			{
				switch (i)
				{
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
			}
			else
			{
				switch (i)
				{
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
 * TODO: Resolve unknown_flag, unknown_enum and uknown0 and unknown1 fields
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
	const uint8_t unknown_flag = *(uint8_t *)UINT8_UNKNOWN_FLAG_20000011; // uint8_t
	const mode_t selected_mode = *(mode_t *)UINT8_SELECTED_MODE_2000003f; // uint8_t
	const int unknown_enum = *(int *)UNKNOWN_ENUM_20000012;		      // unknown

	mode_t *prev_mode = UINT8_PREV_MODE_20000043;			 // -> uint8_t
	uint8_t *selected_prog = UINT8_SELECTED_PROG_20000042;		 // -> uint8_t
	uint8_t *prev_selected_prog = UINT8_PREV_SELECTED_PROG_20000032; // -> uint8_t
	pad_midi *pads_midi = PAD_MIDI_8_0x2000052a;			 // -> pad_midi[8]
	pad_states *pads_states = PAD_STATES_8_20000552;		 // -> pad_states[8]

	// I have yet to find out what sets this flag != 0
	if (unknown_flag == 0)
	{
		/* Check if mode has been switched */
		if (*prev_mode != selected_mode)
		{
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
			for (uint8_t i = 0; i < 8; i++)
			{
				pads_states[i].prog_chng = 0;
			}
			*prev_mode = selected_mode;
		}

		// Set LEDs according to selected mode
		switch (selected_mode)
		{
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

			if (*prev_selected_prog != *selected_prog)
			{
				rst_pads();

				// Reset states of all pads
				for (uint8_t i = 0; i < 8; i++)
				{
					pads_midi[i].state = PAD_STATE_RELEASED;
					pads_states[i].unknown0 = 0;
					pads_states[i].unknown1 = 0;
					pads_states[i].prog_chng = PAD_STATE_RELEASED;
					pads_states[i].pad = PAD_STATE_RELEASED;
					pads_states[i].cc = PAD_STATE_RELEASED;
				}
			}

			switch (*selected_prog)
			{
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
	}
	else
	{
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

		switch (unknown_enum)
		{
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
 * @ 0x08005550
 * Progress: INCOMPLETE
 */
void main_loop()
{
	uint8_t *unknown_flag_0 = UINT8_UNKNOWN_FLAG_20000000;
	uint8_t *unknown_flag_1 = UINT8_UNKNOWN_FLAG_20000098;
	mode_t *prev_mode = UINT8_PREV_MODE_20000043;

	FUN_08004ef0();
	FUN_08005318();

	while (true)
	{
		while (true)
		{
			while (true)
			{
				FUN_08003a94();

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

			FUN_080023fc();
			FUN_08003b10();
			FUN_08003130();
			read_mode_pbs();
			eval_mode_pbs();
			update_leds();

			if (*unknown_flag_1 == 0)
				break;

			FUN_08002790();
			FUN_08003d5c();
		}

		FUN_08002584();
	}
}