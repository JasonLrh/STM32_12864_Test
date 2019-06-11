# STM32_12864_Test
## 特点 | Features 
1,能驱动SSD1306控制下的 OLED 12864。
2,实现了ARM环境中的std重定向，具体实现方法见docs。
3,使用 STM32CubeMx + VScode + Makefile + arm-none-eabi-gcc，完全免费的生态链！

##注意 | Patient 
如果你使用STM32CubeMx并且想重定向std，请修改软件自动生成的Makefile，具体操作为
1,CFLAGS参数后添加 --specs=nano.specs -u _printf_float
2,LIBS后的参数至少要有 -lc -lm -lnosys -lgcc -lrdimon
3,重定向函数与某些工具链中的函数如fputc()等不同，应该为 (以UART为例，仅初始化printf)
caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}

int link(char *old, char *new) {
  return -1;
}

int _close(int file)
{
  return -1;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{

  return 0;
}

void abort(void)
{
  /* Abort called */
  while(1);
}
          
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&std_uart,(uint8_t *)ptr,len,UART_Defult_Wait_Time);
  while(HAL_UART_GetState(&std_uart) != HAL_UART_STATE_READY);
  return len;
}
