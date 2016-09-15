/*
 * global.h
 *
 *  Created on: Sep 13, 2016
 *      Author: ToaN
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#define Led_Base 	GPIO_PORTF_BASE
#define Button_Base GPIO_PORTF_BASE

#define Led_1		GPIO_PIN_1
#define Led_2 		GPIO_PIN_2
#define Button_1	GPIO_PIN_4
#define Button_2	GPIO_PIN_3

// define cac trang thai
#define soft_start  0
#define idle 		1
#define run			2
#define error		3
// trabg thai nhay led
#define fast 1
#define slow 0
// khai bao cac co nho
uint8_t flag_err;
#define ERROR 1
#define NON_ERROR 0

uint8_t flag_run;
uint8_t flag_state;

uint16_t timer_cnt=0;     // bien dem timer tang sau moi lan tran
uint8_t flag_timer_check = 0; // bao cho timer biet la dang trong che do kiem tra nhan giu trong 1s hay nhan nhanh 2 lan
uint8_t check_double_cnt = 0 ; // dung de dem so lan nhan trong truong hop nhan nhanh 2 lan
#define check_1s 	 0
#define check_double 1




#endif /* GLOBAL_H_ */
