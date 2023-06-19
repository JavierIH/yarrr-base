#include "serial.h"

void serial_init(uint32_t baudrate) {
    rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART3);

	/* Enable the USART3 interrupt. */
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Setup GPIO pin GPIO_USART3_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

	/* Setup GPIO pin GPIO_USART3_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, baudrate);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX_RX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Enable USART3 Receive interrupt. */
	USART_CR1(USART3) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART3);
}

volatile uint8_t data;

void usart3_isr(void){
    serial_receiver();
}


void serial_print(char *s){
	while (*s != 0){
		usart_send_blocking(USART3, *s);
		s++;
	}
}	

void serial_send(uint8_t *buffer, uint16_t len){
	for(int i=0; i<len; i++){
		usart_send_blocking(USART3, *(buffer+i));
	}
}

void serial_send_ch(uint8_t ch){
	usart_send_blocking(USART3, ch);
}

//uint16_t serial_receive(uint8_t *buffer){
    //usart_
//}