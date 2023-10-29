void update_pb_leds(void)

{
  int iVar1;
  char *pcVar2;
  uint i;
  int iVar3;
  uint *GPIO_B_ODR;
  int AFIO_Base;
  char current_mode;
  char *prev_mode;
  int unkwn_pb_led_struct;
  
  unkwn_pb_led_struct = unkwn_pb_led_struct_addr;
  prev_mode = prev_mode_addr;
  AFIO_Base = AFIO_Base_addr;
                    /* AFIO_Base = 0x40010000 */
  pcVar2 = DAT_08004074;
  if (*DAT_08004070 == 0) {
    current_mode = *prev_mode_addr;
                    /* On Mode Change */
    if (*prev_mode_addr != current_mode) {
      GPIO_B_ODR = (uint *)(AFIO_Base_addr + 0xc0c);
      *GPIO_B_ODR = *GPIO_B_ODR & 0xdfff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xbfff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0x7fff;
      i = 0;
      do {
        *(undefined *)(i * 5 + unkwn_pb_led_struct + 2) = 0;
                    /* Discard MSB 16bits */
        i = i + 1 & 0xff;
      } while (i < 8);
      *prev_mode = current_mode;
    }
                    /* LEDs */
    if (current_mode == 1) {
                    /* Enables PAD LED */
      *(uint *)(AFIO_Base + 0xc0c) = *(uint *)(AFIO_Base + 0xc0c) | 0x2000;
    }
    else if (current_mode == 2) {
                    /* Enabled CC LED */
      *(uint *)(AFIO_Base + 0xc0c) = *(uint *)(AFIO_Base + 0xc0c) | 0x8000;
    }
    else if (current_mode == 3) {
                    /* PROG CHNG */
      *(uint *)(AFIO_Base + 0xc0c) = *(uint *)(AFIO_Base + 0xc0c) | 0x4000;
    }
    else if (current_mode == 0) {
      GPIO_B_ODR = (uint *)(AFIO_Base + 0xc0c);
      *GPIO_B_ODR = *GPIO_B_ODR & 0xffdf;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xffbf;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xff7f;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xfeff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xfdff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xfbff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xf7ff;
      *GPIO_B_ODR = *GPIO_B_ODR & 0xefff;
      if (*(char *)(unkwn_pb_led_struct_addr + -0x58) == '\x01') {
        *pcVar2 = '\x01';
      }
      else if (*(char *)(unkwn_pb_led_struct_addr + -0x52) == '\x01') {
        *pcVar2 = '\x02';
      }
      else if (*(char *)(unkwn_pb_led_struct_addr + -0x4c) == '\x01') {
        *pcVar2 = '\x03';
      }
      else if (*(char *)(unkwn_pb_led_struct_addr + -0x46) == '\x01') {
        *pcVar2 = '\x04';
      }
      if (*(char *)(DAT_08004088 + 1) != *pcVar2) {
        FUN_0800442c();
        i = 0;
        AFIO_Base = unkwn_pb_led_struct_addr + -0x28;
        do {
          iVar1 = i * 5;
          *(undefined *)(AFIO_Base + iVar1) = 0;
          iVar3 = iVar1 + unkwn_pb_led_struct;
          *(undefined *)(iVar3 + 2) = 0;
          *(undefined *)(iVar3 + 3) = 0;
          *(undefined *)(iVar3 + 4) = 0;
          *(undefined *)(unkwn_pb_led_struct + iVar1) = 0;
          *(undefined *)(iVar3 + 1) = 0;
          i = i + 1 & 0xff;
        } while (i < 8);
      }
      current_mode = *pcVar2;
      if (current_mode == '\x01') {
        *GPIO_B_ODR = *GPIO_B_ODR | 0x20;
      }
      else if (current_mode == '\x02') {
        *GPIO_B_ODR = *GPIO_B_ODR | 0x40;
      }
      else if (current_mode == '\x03') {
        *GPIO_B_ODR = *GPIO_B_ODR | 0x80;
      }
      else if (current_mode == '\x04') {
        *GPIO_B_ODR = *GPIO_B_ODR | 0x100;
      }
    }
    FUN_08003420();
  }
  else {
    GPIO_B_ODR = (uint *)(AFIO_Base_addr + 0xc0c);
    *GPIO_B_ODR = *GPIO_B_ODR & 0xdfff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xbfff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0x7fff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xffdf;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xffbf;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xff7f;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xfeff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xfdff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xfbff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xf7ff;
    *GPIO_B_ODR = *GPIO_B_ODR & 0xefff;
    switch(*DAT_0800408c) {
    case 1:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x2000;
      break;
    case 2:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x8000;
      break;
    case 3:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x4000;
      break;
    case 4:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x20;
      break;
    case 5:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x40;
      break;
    case 6:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x80;
      break;
    case 7:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x100;
      break;
    case 8:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x200;
      break;
    case 9:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x400;
      break;
    case 10:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x800;
      break;
    case 0xb:
      *GPIO_B_ODR = *GPIO_B_ODR | 0x1000;
    }
  }
  *(char *)(DAT_08004088 + 1) = *pcVar2;
  return;
}
