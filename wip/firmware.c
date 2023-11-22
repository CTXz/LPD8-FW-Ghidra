#include <stdint.h>
#include <stdbool.h>

#include <stm32f103xb.h>
#include <stm32f101xe.h>
#include <stm32f1xx_hal.h>

#define CONST_UINT8_8_LUT_8TO15_080056EC 0x080056EC    // = [8, 9, 10, 11, 12, 13, 14, 15]
#define CONST_UINT8_127_LUT_1TO127_080056E4 0x080056E4 // = [1, 2, ... 126, 127, 127]

#define UINT32_NVIC_VECTOR_TABLE_OFFSET 0x2000
#define UINT32_NVIC_VETOR_TABLE_BASE 0x08000000

#define UINT8_UNKNOWN_FLAG_0x20000000 0x20000000
#define UINT8_MIDI_BUFFER_REMAINING_SPACE_0x20000004 0x20000004
#define UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008 0x20000008
#define UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c 0x2000000c // buffer_end - MIDI_BUFFER_SIZE
#define UINT8_UNKNOWN_FLAG_0x20000011 0x20000011
#define UNKNOWN_ENUM_0x20000012 0x20000012
#define DMA1_MEM_0x2000013C 0x2000013C
#define UINT8_SYSTICK_DECREMENTER_0_0x20000018 0x20000018 // Decrements on every SysTick Interrupt
#define BOOL_8_SCALED_KNOB_VALS_CHANGED_0x20000019 0x20000019
#define UINT8_8_PREV_SCALED_KNOB_VALS_0x20000021 0x20000021
#define UINT8_8_PREV_PREV_SCALED_KNOB_VALS_0x20000029 0x20000029
#define UINT8_PREV_MODE_0x20000031 0x20000031
#define UINT8_PREV_SELECTED_PROG_0x20000032 0x20000032
#define UINT8_8_PREV_PADS_STATE_0x20000033 0x20000033
#define UINT8_SYSTICK_DECREMENTER_1_0x2000003c 0x2000003c // Decrements on every SysTick Interrupt
#define UINT8_DEBOUNCE_COUNTER_0x2000003e 0x2000003e
#define UINT8_SELECTED_MODE_0x2000003f 0x2000003f
#define UINT8_UNKNOWN_READY_0x20000040 0x20000040
#define UINT8_UNKNOWN_READY_0x20000041 0x20000041
#define UINT8_SELECTED_PROG_0x20000042 0x20000042
#define UINT8_PREV_MODE_0x20000043 0x20000043
#define UINT16_PREV_MODE_PB_IDR_BITS_0x20000044 0x20000044
#define UINT16_PREV_MODE_PB_IDR_BITS_0x20000046 0x20000046
#define UINT16_MODE_PB_IDR_BITS_CPY_0x20000048 0x20000048
#define UINT8_UNKNOWN_FLAG_0x20000098 0x20000098
#define UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c 0x2000015c
#define UINT8_MIDI_BUFFER_END_0x2000024C 0x2000024C
#define PROGRAM_SETTINGS_5_0x200003CC 0x200003CC
#define UINT16_8_PREV_ACCEPTED_KNOB_ADC_VALS_0x200004EA 0x200004EA
#define PAD_HANDLING_DATA_8_0x200004FA 0x200004FA
#define PENDING_MIDI_8_RELEASE_MIDI_0x2000052a 0x2000052a
#define PAD_STATES_8_0x20000552 0x20000552
#define UINT16_8_KNOB_ADC_VALS_0x2000059A 0x2000059A
#define UINT16_8_PAD_ADC_VALS_0x200005AA 0x200005AA

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

#define DMA1_NUM_DATA 0x10
#define ADC_N_CONV 0x10
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
#define PAD_ADC_CONSIDERED_RELEASED 64
#define PAD_ADC_CONSIDERED_PRESSED MIDI_MAX_DATA_VAL + 2
#define PAD_PRESS_INCR_COMPLETE 4
#define PAD_RELEASE_DECR_START 30
#define PAD_MODE_RELEASE_DATA2 MIDI_MAX_DATA_VAL
#define CC_MODE_RELEASE_DATA2 0x00
#define PROG_CHNG_MODE_RELEASE_DATA2 0x00
#define PROG_CHNG_MODE_PRESS_DATA2 0x00
#define N_PADS 8
#define N_KNOBS 8
#define N_PADS_OR_KNOBS 8

// #define EXCEEDS_THRESHOLD(x, y, threshold) ((x) < (y) + (threshold) || (y) < (x) + (threshold))
#define EXCEEDS_THRESHOLD(x, y, threshold) ((x) + (threshold) <= (y) || (y) + (threshold) <= (x))

typedef uint32_t unknown; // So the linter doesn't complain

typedef uint8_t midi_data_t; // Only 7 bits are used
typedef uint16_t adc_val_t;  // LPD8 configures ADC to 10-bit resolution

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

typedef struct
{
	uint32_t gpios;
	uint32_t other_cr_bits; // CNF + MODE, only used if op == GPIO_CFG_OP_OTHER
	gpio_operation_t op;
} gpio_cfg;

typedef struct
{
	midi_pending_t pending;
	midi_cmd_msb_t cmd_msb;
	uint8_t cmd;
	uint8_t data1;
	uint8_t data2;
} pending_midi;

// These appear to be updated on on-push and on-release events only
typedef struct
{
	pad_toggled_t pad_toggled;
	pad_toggled_t cc_toggled;
	pad_state_t prog_chng; // offset: 2
	pad_state_t pad;       // offset: 3
	pad_state_t cc;	       // offset: 4
} pad_states;

typedef struct
{
	uint8_t note;
	uint8_t pc;
	uint8_t cc;
	push_setting_t type;
} pad_settings;

typedef struct
{
	pad_confirmed_t last_confirmed_state;
	uint8_t press_incr;
	uint8_t rel_decr;
	uint8_t unknown0;
	uint16_t adc_eval;
} pad_handling_data;

typedef struct
{
	uint8_t cc;
	uint8_t leftmost;
	uint8_t rightmost;
} knob_settings;

typedef struct
{
	uint8_t midi_ch;
	pad_settings pads[N_PADS];
	knob_settings knobs[N_KNOBS];
} program_settings;

/**
 * @ 0x08004e7c, Called by IVT Entry @ 0x0800203c
 * Progress: DONE
 */
void SysTick_Handler(void)
{
	uint8_t *systick_decr_0 = UINT8_SYSTICK_DECREMENTER_0_0x20000018;
	uint8_t *systick_decr_1 = UINT8_SYSTICK_DECREMENTER_1_0x2000003c;

	if (*systick_decr_0 != 0)
		*systick_decr_0 = *systick_decr_0 - 1;

	if (*systick_decr_1 != 0)
		*systick_decr_1 = *systick_decr_1 - 1;
}

/**
 * @ 0x08005518
 * Progress: ALMOST DONE
 * TODO: Confirm why 0x1FFFFF80
 * 	 Make inline with two params
 */
void nvic_init_vector_table(void)
{
	const uint32_t base = UINT32_NVIC_VETOR_TABLE_BASE;
	const uint32_t offset = UINT32_NVIC_VECTOR_TABLE_OFFSET;
	SCB->VTOR = base | (offset & 0x1FFFFF80);
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

	switch ((uintptr_t)dma_ccr)
	{
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
	GPIOs_cfg(GPIOA, &(gpio_cfg){GPIO_0TO7_MSK, 0, GPIO_CFG_OP_IN_ANALOG});

	// Pads 1-2
	const uint32_t GPIO_0TO3_MSK = 0x0F;
	GPIOs_cfg(GPIOB, &(gpio_cfg){GPIO_0TO3_MSK, 0, GPIO_CFG_OP_IN_ANALOG});

	// TODO: Pads 2-3 Missing??

	// Pads 5-8
	const uint32_t GPIO_2TO5_MSK = 0x3C;
	GPIOs_cfg(GPIOC, &(gpio_cfg){GPIO_2TO5_MSK, 0, GPIO_CFG_OP_IN_ANALOG});

	RCC_set_AHBENR(1 << RCC_AHBENR_DMA1EN, true);
	RCC_set_APB2ENR(1 << RCC_APB2ENR_ADC1EN, true);

	DMA_CLEAR_IFCR(DMA1_Channel1_BASE);
	DMA_set_CPAR_CMAR_DIR_CNDTR_PINC_MINC_PSIZE_MSIZE_CIRC_PL_MEM2MEM(
	    DMA1_Channel1_BASE,
	    ADC1->DR,
	    DMA1_MEM_0x2000013C,
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
 * be suprised if the compiler optimization is to blame for this.
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
	if (gpio_cfg->gpios & 0xFF)
	{
		uint32_t crl = *GPIO_CRL;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++)
		{
			uint8_t msk = 1 << i_gpio;

			if (gpio_cfg->gpios & msk)
			{
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
	if (gpio_cfg->gpios > 0xFF)
	{
		uint32_t crh = *GPIO_CRH;

		for (uint8_t i_gpio = 0; i_gpio < 8; i_gpio++)
		{
			uint8_t msk = 1 << (i_gpio + 8); // + 8 due to GPIO's 8-15

			if (gpio_cfg->gpios & msk)
			{
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

	uint8_t *remaining_space = UINT8_MIDI_BUFFER_REMAINING_SPACE_0x20000004;
	uint8_t *buffer_end = UINT8_MIDI_BUFFER_END_0x2000024C;
	uint8_t **head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008;
	uint8_t **tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c; // buffer_end - MIDI_BUFFER_SIZE

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
 * @ 0x08003b10
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm purpose of ready flag,
 * 	 Understand addition of 1 to range
 * 	 Confirm write_mid output
 */
void eval_knobs(void)
{
	const uint8_t systick_decr_0 = *(uint8_t *)UINT8_SYSTICK_DECREMENTER_0_0x20000018;
	const uint8_t sel_prog = *(uint8_t *)UINT8_SELECTED_PROG_0x20000042;
	program_settings *all_prog_settings = PROGRAM_SETTINGS_5_0x200003CC;
	program_settings *sel_prog_settings = &all_prog_settings[sel_prog];
	bool *scaled_knob_vals_changed = BOOL_8_SCALED_KNOB_VALS_CHANGED_0x20000019;
	midi_data_t *prev_scaled_knob_vals = UINT8_8_PREV_SCALED_KNOB_VALS_0x20000021;
	midi_data_t *prev_prev_scaled_knob_vals = UINT8_8_PREV_PREV_SCALED_KNOB_VALS_0x20000029;
	adc_val_t *prev_accepted_knob_adc_vals = UINT16_8_PREV_ACCEPTED_KNOB_ADC_VALS_0x200004EA;
	adc_val_t *knob_adc_vals = UINT16_8_KNOB_ADC_VALS_0x2000059A;
	ready_t *ready = UINT8_UNKNOWN_READY_0x20000040;

	/*
	 * Appears to be reset after intervals of 4 calls
	 * Probably SysTick related
	 */
	if (*ready != READY)
		return;

	*ready = NOT_READY;

	// Treat invalid MIDI channels as channel 0 (aka. 1)
	uint8_t midi_ch = sel_prog_settings->midi_ch;
	if (midi_ch > MIDI_MAX_CHANNEL)
		midi_ch = 0;

	for (uint8_t i = 0; i < N_KNOBS; i++)
	{
		uint8_t knob_cc = sel_prog_settings->knobs[i].cc;
		uint8_t knob_rightmost = sel_prog_settings->knobs[i].rightmost;
		uint8_t knob_leftmost = sel_prog_settings->knobs[i].leftmost;
		bool *scaled_knob_val_changed = &scaled_knob_vals_changed[i];
		midi_data_t *prev_knob_scaled_val = &prev_scaled_knob_vals[i];
		midi_data_t *prev_prev_knob_scaled_val = &prev_prev_scaled_knob_vals[i];
		adc_val_t *prev_accepted_knob_adc_val = &prev_accepted_knob_adc_vals[i];
		adc_val_t *knob_adc_val = &knob_adc_vals[i];

		const uint32_t _prev_accepted_knob_adc_val = (uint32_t)*prev_accepted_knob_adc_val;
		const uint32_t _knob_adc_val = (uint32_t)*knob_adc_val;

		// Knobs with CC 0 are ignored/disabled
		if (knob_cc == 0)
			continue;

		// Only update if knob has been moved beyond threshold
		if (!EXCEEDS_THRESHOLD(
			_knob_adc_val,
			_prev_accepted_knob_adc_val,
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

		if (knob_leftmost < knob_rightmost)
		{
			uint8_t range = (knob_rightmost - knob_leftmost) + 1;
			scaled = knob_leftmost +
				 (_knob_adc_val * range) /
				     (MAX_ADC_KNOB_VAL - ADC_KNOB_CHANGE_THRESHOLD - 1);

			if (knob_rightmost < scaled)
				scaled = knob_rightmost;
		}
		else
		{
			uint8_t range = (knob_rightmost - knob_leftmost) + 1;
			scaled = knob_leftmost -
				 (_knob_adc_val * range) /
				     (MAX_ADC_KNOB_VAL - ADC_KNOB_CHANGE_THRESHOLD - 1);

			if (knob_rightmost > knob_leftmost)
				scaled = knob_rightmost;
		}

		if (scaled > MIDI_MAX_DATA_VAL)
			scaled = MIDI_MAX_DATA_VAL;

		if (*prev_knob_scaled_val != scaled)
		{

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
				_prev_accepted_knob_adc_val,
				_knob_adc_val,
				ADC_KNOB_NOISE_GATE))
				continue;

			*scaled_knob_val_changed = true;
		}
		else
		{
			*scaled_knob_val_changed = false;
		}

		*prev_prev_knob_scaled_val = *prev_knob_scaled_val;
		*prev_knob_scaled_val = scaled;

		/*
		 * Send a MIDI CC message once the time is right.
		 * The SysTick decrementer is likely employed to
		 * prevent flooding the MIDI buffer.
		 */
		if (systick_decr_0 == 0)
		{

			if (knob_cc > MIDI_MAX_DATA_VAL)
				knob_cc = MIDI_MAX_DATA_VAL;

			uint8_t data[4] = {
			    MIDI_CMD_CC_MSB | midi_ch,
			    knob_cc, scaled,
			    0x00};

			write_midi_buffer(&data, sizeof(data));
		}

		*prev_accepted_knob_adc_val = *knob_adc_val;
	}
}

/**
 * @ 0x08003130
 * Progress: ALMOST DONE / AWAITING MORE INFO
 * TODO: Confirm ready flag,
 * 	 Confirm if midi output actually transmitts the MSB
 */
void eval_pads(void)
{
	const uint8_t sel_prog = *(uint8_t *)UINT8_SELECTED_PROG_0x20000042;
	const uint8_t *plus_1_max_127 = CONST_UINT8_127_LUT_1TO127_080056E4;
	const mode_t sel_mode = *(uint8_t *)UINT8_SELECTED_MODE_0x2000003f;

	uint8_t *ready = UINT8_UNKNOWN_READY_0x20000041;
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_RELEASE_MIDI_0x2000052a;
	uint8_t *all_prog_settings = PROGRAM_SETTINGS_5_0x200003CC;
	program_settings *sel_prog_settings = &all_prog_settings[sel_prog];
	pad_states *pads_states = PAD_STATES_8_0x20000552;
	pad_handling_data *pads_hd = PAD_HANDLING_DATA_8_0x200004FA;
	adc_val_t *pad_adc_vals = UINT16_8_PAD_ADC_VALS_0x200005AA;

	uint8_t status_msbyte = 0;
	uint8_t data1 = 0;
	uint8_t data2_press = 0;
	uint8_t data2_release = 0;

	if (*ready != READY)
		return;

	*ready = NOT_READY;

	// Treat invalid MIDI channels as channel 0 (aka. 1)
	uint8_t midi_ch = sel_prog_settings->midi_ch;
	if (midi_ch > MIDI_MAX_CHANNEL)
		midi_ch = 0;

	// msg = in_r3; TODO Inspect
	for (uint8_t i = 0; i < N_PADS; i++)
	{

		pad_handling_data *pad_hd = &pads_hd[i];
		adc_val_t pad_adc_val = pad_adc_vals[i];
		pad_states *states = &pads_states[i];
		pad_settings *settings = &sel_prog_settings->pads[i];
		pending_midi *on_release_midi = &pads_on_release_midi[i];

		pad_confirmed_t changed = UNCHANGED;
		switch (pad_hd->last_confirmed_state)
		{
		case CONFIRMED_PRESSED:

			if (pad_adc_val <= PAD_ADC_CONSIDERED_RELEASED)
			{

				if (pad_hd->rel_decr == 0)
				{
					pad_hd->last_confirmed_state = CONFIRMED_RELEASED;
					changed = CONFIRMED_RELEASED;
				}
				else
				{
					pad_hd->rel_decr--;
				}

				pad_hd->adc_eval = 0;
				pad_hd->press_incr = 0;
			}
			else
			{
				pad_hd->rel_decr = PAD_RELEASE_DECR_START;
			}
			break;

		case CONFIRMED_RELEASED:

			if (pad_adc_val < PAD_ADC_CONSIDERED_PRESSED)
				break;

			if (pad_hd->press_incr == 0 ||
			    pad_hd->adc_eval < pad_adc_val)
			{
				pad_hd->adc_eval = pad_adc_val;
			}

			if (pad_hd->press_incr < PAD_PRESS_INCR_COMPLETE)
			{
				pad_hd->press_incr++;
			}
			else
			{
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
				 * 		  PAD_ADC_CONSIDERED_PRESSED, which is
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

		switch (changed)
		{
		case CONFIRMED_RELEASED:

			if (sel_mode == MODE_PROG_CHNG)
			{
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

			if (is_momentary || is_prog)
			{
				on_release_midi->pending = NOT_PENDING;

				if (on_release_midi->data1 <= MIDI_MAX_DATA_VAL)
				{
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

			switch (sel_mode)
			{
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
					  data2_press};

			states->prog_chng = PAD_STATE_PRESSED;

			pad_toggled_t toggled;
			if (sel_mode == MODE_PAD)
			{
				states->pad = PAD_STATE_PRESSED;

				if (settings->type == TOGGLE)
				{
					states->pad_toggled = !states->pad_toggled;
					toggled = states->pad_toggled;

					if (toggled == TOGGLED_OFF)
					{
						msg[0] = MIDI_CMD_NOTE_OFF_MSB;
						msg[1] = (MIDI_CMD_NOTE_OFF_MSB << 4) | midi_ch;
						msg[2] = data1;
						msg[3] = data2_release;
					}
				}
			}
			else if (sel_mode == MODE_CC)
			{
				states->cc = PAD_STATE_PRESSED;

				if (settings->type == TOGGLE)
				{
					states->cc_toggled = !states->cc_toggled;
					toggled = states->cc_toggled;

					if (toggled == TOGGLED_OFF)
					{
						msg[3] = CC_MODE_RELEASE_DATA2;
					}
				}
			}

			if (toggled == TOGGLED_OFF)
			{
				states->prog_chng = PAD_STATE_RELEASED;

				if (sel_mode == MODE_PAD)
					states->pad = PAD_STATE_RELEASED;
				else if (sel_mode == MODE_CC)
					states->cc = PAD_STATE_RELEASED;
			}

			if (status_msbyte == MIDI_CMD_NOTE_ON_MSB)
			{
				on_release_midi->cmd_msb = MIDI_CMD_NOTE_OFF_MSB;
				on_release_midi->cmd = midi_ch | (MIDI_CMD_NOTE_OFF_MSB << 4);
			}
			else
			{
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
	uint8_t *systick_decr = UINT8_SYSTICK_DECREMENTER_1_0x2000003c;
	uint8_t *debounce = UINT8_DEBOUNCE_COUNTER_0x2000003e;
	uint16_t *prev_mode_pbs = UINT16_PREV_MODE_PB_IDR_BITS_0x20000044;
	uint16_t *mode_pbs = UINT16_MODE_PB_IDR_BITS_CPY_0x20000048;

	// Mode Push buttons are read every 4 SysTick intervals.
	if (*systick_decr != 0)
		return;

	*systick_decr = MODE_PB_SYSTICK_SCAN_INTERVALS;

	// Read mode push buttons. Negation due to PU's.
	const uint32_t _mode_pbs = ~(PB_GPIO_PORT->IDR) & PB_IDR_MSK;
	if (*prev_mode_pbs != _mode_pbs)
	{
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
	if (*debounce < 240)
	{
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
	const uint16_t mode_pbs = *(uint16_t *)UINT16_MODE_PB_IDR_BITS_CPY_0x20000048;
	const uint8_t unknown_flag = *(uint8_t *)UINT8_UNKNOWN_FLAG_0x20000011;

	unknown *prev_mode_pbs = UINT16_PREV_MODE_PB_IDR_BITS_0x20000046;
	mode_t *selected_mode = UINT8_SELECTED_MODE_0x2000003f;

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
 * TODO: Resolve pad_toggled and cc_toggled fields of pad_states struct
 */
void rst_pads(void)
{
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_RELEASE_MIDI_0x2000052a; // -> pending_midi[N_PADS]
	pad_states *pads_states = PAD_STATES_8_0x20000552;			     // -> pad_states[N_PADS]

	for (uint8_t i = 0; i < N_PADS; i++)
	{
		if (pads_on_release_midi[i].pending == PENDING)
		{
			pads_on_release_midi[i].pending = NOT_PENDING;
			pads_states[i].pad_toggled = false;
			pads_states[i].cc_toggled = false;
			pads_states[i].prog_chng = PAD_STATE_RELEASED;
			pads_states[i].pad = PAD_STATE_RELEASED;
			pads_states[i].cc = PAD_STATE_RELEASED;

			if (pads_on_release_midi[i].cmd >> 4 == MIDI_CMD_NOTE_OFF_MSB &&
			    pads_on_release_midi[i].data1 <= MIDI_MAX_DATA_VAL)
			{
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
	const mode_t selected_mode = *(uint8_t *)UINT8_SELECTED_MODE_0x2000003f; // uint8_t

	mode_t *prev_mode = UINT8_PREV_MODE_0x20000031;			   // uint8_t
	pad_state_t *prev_pads_state = UINT8_8_PREV_PADS_STATE_0x20000033; // -> pad_state_t[N_PADS]
	pad_states *pads_states = PAD_STATES_8_0x20000552;		   // -> pad_states[N_PADS]

	// For every Pad
	for (uint8_t i = 0; i < N_PADS; i++)
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
			for (uint8_t j = 0; j < N_PADS; j++)
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
	const uint8_t unknown_flag = *(uint8_t *)UINT8_UNKNOWN_FLAG_0x20000011; // uint8_t
	const mode_t selected_mode = *(mode_t *)UINT8_SELECTED_MODE_0x2000003f; // uint8_t
	const int unknown_enum = *(int *)UNKNOWN_ENUM_0x20000012;		// unknown

	mode_t *prev_mode = UINT8_PREV_MODE_0x20000043;				     // -> uint8_t
	uint8_t *selected_prog = UINT8_SELECTED_PROG_0x20000042;		     // -> uint8_t
	uint8_t *prev_selected_prog = UINT8_PREV_SELECTED_PROG_0x20000032;	     // -> uint8_t
	pending_midi *pads_on_release_midi = PENDING_MIDI_8_RELEASE_MIDI_0x2000052a; // -> pending_midi[N_PADS]
	pad_states *pads_states = PAD_STATES_8_0x20000552;			     // -> pad_states[N_PADS]

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
			for (uint8_t i = 0; i < N_PADS; i++)
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

				for (uint8_t i = 0; i < N_PADS; i++)
				{
					pads_on_release_midi[i].pending = NOT_PENDING;
					pads_states[i].pad_toggled = false;
					pads_states[i].cc_toggled = false;
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
	uint8_t *midi_buf_rem_space = UINT8_MIDI_BUFFER_REMAINING_SPACE_0x20000004;
	uint8_t **midi_buf_tail = UINT8_PTR_PTR_MIDI_BUFFER_TAIL_0x2000000c;
	uint8_t **midi_buf_head = UINT8_PTR_PTR_MIDI_BUFFER_HEAD_0x20000008;

	*midi_buf_rem_space = 0;
	*midi_buf_tail = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
	*midi_buf_head = UINT8_PTR_MIDI_BUFFER_TAIL_0x2000015c;
}

/**
 * @ 0x08005550
 * Progress: INCOMPLETE
 */
void main_loop()
{
	uint8_t *unknown_flag_0 = UINT8_UNKNOWN_FLAG_0x20000000;
	uint8_t *unknown_flag_1 = UINT8_UNKNOWN_FLAG_0x20000098;
	mode_t *prev_mode = UINT8_PREV_MODE_0x20000043;

	FUN_08004ef0();
	FUN_08005318();

	while (true)
	{
		while (true)
		{
			while (true)
			{
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

			FUN_080023fc();
			eval_knobs();
			eval_pads();
			read_mode_pbs();
			eval_mode_pbs();
			update_leds();

			if (*unknown_flag_1 == 0)
				break;

			FUN_08002790();
			FUN_08003d5c();
		}

		deinit_midi_buffer();
	}
}