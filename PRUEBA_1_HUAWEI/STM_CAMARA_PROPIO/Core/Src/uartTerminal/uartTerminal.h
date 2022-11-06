/*
 * uartTerminal.h
 *
 *  Created on: 26 oct 2022
 *      Author: wikst
 */

#ifndef UARTTERMINAL_UARTTERMINAL_H_
#define UARTTERMINAL_UARTTERMINAL_H_

RET uartTerminal_init(UART_HandleTypeDef *huart);
RET uartTerminal_send(uint8_t data);
uint8_t uartTerminal_recv();
RET uartTerminal_recvTry(uint8_t *data);

#endif /* SRC_UARTTERMINAL_UARTTERMINAL_H_ */
