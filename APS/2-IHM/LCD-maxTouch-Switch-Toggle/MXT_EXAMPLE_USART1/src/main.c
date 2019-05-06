/**
* \file
*
* \brief Example of usage of the maXTouch component with USART
*
* This example shows how to receive touch data from a maXTouch device
* using the maXTouch component, and display them in a terminal window by using
* the USART driver.
*
* Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products.
* It is your responsibility to comply with third party license terms applicable
* to your use of third party software (including open source software) that
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/

/**
* \mainpage
*
* \section intro Introduction
* This simple example reads data from the maXTouch device and sends it over
* USART as ASCII formatted text.
*
* \section files Main files:
* - example_usart.c: maXTouch component USART example file
* - conf_mxt.h: configuration of the maXTouch component
* - conf_board.h: configuration of board
* - conf_clock.h: configuration of system clock
* - conf_example.h: configuration of example
* - conf_sleepmgr.h: configuration of sleep manager
* - conf_twim.h: configuration of TWI driver
* - conf_usart_serial.h: configuration of USART driver
*
* \section apiinfo maXTouch low level component API
* The maXTouch component API can be found \ref mxt_group "here".
*
* \section deviceinfo Device Info
* All UC3 and Xmega devices with a TWI module can be used with this component
*
* \section exampledescription Description of the example
* This example will read data from the connected maXTouch explained board
* over TWI. This data is then processed and sent over a USART data line
* to the board controller. The board controller will create a USB CDC class
* object on the host computer and repeat the incoming USART data from the
* main controller to the host. On the host this object should appear as a
* serial port object (COMx on windows, /dev/ttyxxx on your chosen Linux flavour).
*
* Connect a terminal application to the serial port object with the settings
* Baud: 57600
* Data bits: 8-bit
* Stop bits: 1 bit
* Parity: None
*
* \section compinfo Compilation Info
* This software was written for the GNU GCC and IAR for AVR.
* Other compilers may or may not work.
*
* \section contactinfo Contact Information
* For further information, visit
* <A href="http://www.atmel.com/">Atmel</A>.\n
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
Icons
Icon made by Gregor Cresnar from www.flaticon.com - padloCk
Icon made by Freepik from www.flaticon.com - Play,Chevron,drop
Icon made by catkuro from www.flaticon.com - Bubble
Icon made by Nikita Golubev from www.flaticon.com - dumb Bell
Icon made by Pixelmeetup from www.flaticon.com - Spiral
Icon made by itim2101 from www.flaticon.com - contract
Icon made by Smashicons from www.flaticon.com - clock




*/

#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "conf_board.h"
#include "conf_example.h"
#include "conf_uart_serial.h"
#include "Icons/Mast.h"
#include "Icons/play.h"
#include "Icons/pause.h"
#include "Icons/padlocked.h"
#include "Icons/padlock.h"
#include "Icons/blankText.h"
#include "maquina1.h"


#define MAX_ENTRIES        3
#define STRING_LENGTH     70

#define USART_TX_MAX_LENGTH     0xff

#define YEAR        2018
#define MOUNTH      3
#define DAY         19
#define WEEK        12
#define HOUR        15
#define MINUTE      45
#define SECOND      0

//Buzzer
#define BUZZ_PIO           PIOD
#define BUZZ_PIO_ID        13
#define BUZZ_PIO_IDX       22u
#define BUZZ_PIO_IDX_MASK  (1u << BUZZ_PIO_IDX)

//Led 1 PA0 - Porta Aberta/Fechada
#define LED_PIO_ID	   ID_PIOA
#define LED_PIO        PIOA
#define LED_PIN		   0
#define LED_PIN_MASK (1<<LED_PIN)

//Led 2 PC30 - Acionamento Valvula
#define LED2_PIO_ID	   ID_PIOC
#define LED2_PIO        PIOC
#define LED2_PIN		   30
#define LED2_PIN_MASK (1<<LED2_PIN)

//Led 3 PB2 - Acionamento Motor
#define LED3_PIO_ID	   ID_PIOB
#define LED3_PIO        PIOB
#define LED3_PIN		   2
#define LED3_PIN_MASK (1<<LED3_PIN)

//Led 4 PC8 - Porta Travada/Destravada
// LED
#define LED4_PIO      PIOC
#define LED4_PIO_ID   ID_PIOC
#define LED4_PIN      8
#define LED4_PIN_MASK (1 << LED4_PIN)

//Button 1 PD28 - Porta Aberta/Fechada
#define BUT_PIO_ID			  ID_PIOD
#define BUT_PIO				  PIOD
#define BUT_PIN				  28
#define BUT_PIN_MASK	(1 << BUT_PIN)

//Button 2 PC31 - Trava Externa
#define BUT2_PIO_ID			  ID_PIOC
#define BUT2_PIO				  PIOC
#define BUT2_PIN				  31
#define BUT2_PIN_MASK	(1 << BUT2_PIN)

//Buzzer PD22 - Alarme Sonoro
#define BUZZ_PIO           PIOD
#define BUZZ_PIO_ID        ID_PIOD
#define BUZZ_PIO_IDX       0u
#define BUZZ_PIO_IDX_MASK  (1u << BUZZ_PIO_IDX)

struct ili9488_opt_t g_ili9488_display_opt;
const uint32_t BUTTON_W = 120;
const uint32_t BUTTON_H = 120;
const uint32_t BUTTON_BORDER = 1;
const uint32_t BUTTON_X = 240;
const uint32_t BUTTON_Y = 0;

volatile char playing = 0;
volatile char locked = 0;
volatile char touched = 0;
volatile char rtc_happen=0;
volatile char rtc_sec_happen=0;
volatile char stop_from_door=0;
volatile char lockings=0;
uint8_t total_time[32];

char open_door = 0;
int has_centrifug = 0;
int valvula_on = 0;
int minutes_to_finish, seconds_to_finish;
int enx_q = 0;

volatile uint32_t YEAR2,MONTH2,DAY2,WEEK2,HOUR2,MINUTE2,SECOND2;
volatile int enx_time_30 = 0;

volatile t_ciclo *now_using = &c_rapido;

void pin_toggle(Pio *pio, uint32_t mask){
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}
static void Button1_Handler(uint32_t id, uint32_t mask)
{
	if(open_door==1){
		pio_set(LED_PIO,LED_PIN_MASK);
		open_door=0;
	}
	else{
		open_door=1;
		playing=0;
		pio_clear(LED_PIO,LED_PIN_MASK);
		pio_set(LED2_PIO,LED2_PIN_MASK);
		rtc_disable_interrupt(RTC,RTC_IER_SECEN);
	}
	stop_from_door=1;
}
static void Button2_Handler(uint32_t id, uint32_t mask)
{
	if(locked==1){
		locked=0;
	}
	else{
		locked=1;
	}
	lockings=1;
}
//Usando o TC1 para debounce no touch
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup��o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	touched = 0;
	pmc_disable_periph_clk(TC0);
}
//TC4 controle do PWM do motor
void TC4_Handler(void){
	volatile uint32_t ul_dummy;
	ul_dummy = tc_get_status(TC1, 1);
	/** Muda o estado do LED */
	pin_toggle(LED3_PIO, LED3_PIN_MASK);
	UNUSED(ul_dummy);
}
void draw_text(){
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_rectangle(320,30,480,60);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
	
	if (playing==1)
	{
		sprintf(total_time,"%3d:%2d",minutes_to_finish,seconds_to_finish);
		ili9488_draw_pixmap(320,30,blankText.width,blankText.height,blankText.data);
	}
	ili9488_draw_string(320, 30, total_time);
}
void draw_button() {
	uint8_t enx_time[32];
	uint8_t enx_quant[32];
	uint8_t rpm[32];
	uint8_t rpm_time[32];
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));
	ili9488_draw_pixmap(0, 0, Mast.width, Mast.height, Mast.data);
	
	if(playing==0) {
		ili9488_draw_pixmap(240,200,play.width,play.height,play.data);
		} else {
		ili9488_draw_pixmap(240,200,pause.width,pause.height,pause.data);
	}
	if(locked){
		ili9488_draw_pixmap(120,200,padlocked.width,padlocked.height,padlocked.data);
	}
	else {
		ili9488_draw_pixmap(120,200,padlock.width,padlock.height,padlock.data);
	}
	
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));

	if (now_using->heavy)
	{
		ili9488_draw_string(50, 175, "On");
		//sprintf(enx_time,"%d Minutos",now_using->enxagueTempo+now_using->enxagueTempo/5);
		sprintf(enx_time,"%d Minutos",valvula_on);
		sprintf(rpm_time,"%d Minutos",now_using->centrifugacaoTempo+now_using->centrifugacaoTempo/5);
		if (playing==0)
		{
			sprintf(total_time,"%3d Minutos",(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo)+(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo)/5);
		}
	}
	else {
		ili9488_draw_string(50, 175, "Off");
		sprintf(enx_time,"%d Minutos",now_using->enxagueTempo);
		sprintf(rpm_time,"%d Minutos",now_using->centrifugacaoTempo);
		if (playing==0)
		{
			sprintf(total_time,"%3d Minutos",(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo));
		}
	}
	
	if(now_using->bubblesOn)
	{
		ili9488_draw_string(300, 175, "On");
	}
	else {
		ili9488_draw_string(300, 175, "Off");
	}
	
	//Principal
	ili9488_draw_string(80, 30, now_using->nome);
	//ili9488_draw_string(320, 30, total_time);
	
	
	sprintf(enx_quant,"%d vezes",now_using->enxagueQnt);
	
	
	ili9488_draw_string(50, 90, enx_quant);
	ili9488_draw_string(300, 90, enx_time);
	
	
	sprintf(rpm,"%4d rpm",now_using->centrifugacaoRPM);
	
	draw_text();
	ili9488_draw_string(50, 135, rpm);
	ili9488_draw_string(300, 135, rpm_time);
	pmc_enable_periph_clk(ID_TC1);
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter � meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup�c�o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup�c�o no TC canal 0 */
	/* Interrup��o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/*
	*  Verifica por qual motivo entrou
	*  na interrupcao, se foi por segundo
	*  ou Alarm
	*/
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		rtc_happen=1;
	}
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC)
	{
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
		rtc_sec_happen=1;
	}
	
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
	
}
void BUT_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);

	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrup��o */
	pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
	pio_handler_set(BUT2_PIO, BUT2_PIO_ID, BUT2_PIN_MASK, PIO_IT_FALL_EDGE, Button2_Handler);

	/* habilita interrup�c�o do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 1);
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID, 2);
};
void LED_init(int estado){
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	pmc_enable_periph_clk(LED4_PIO_ID);
	pmc_enable_periph_clk(BUZZ_PIO_ID);
	pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
	pio_set_output(LED2_PIO, LED2_PIN_MASK, estado, 0, 0 );
	pio_set_output(LED3_PIO, LED3_PIN_MASK, estado, 0, 0 );
	pio_set_output(LED4_PIO, LED4_PIN_MASK, estado, 0, 0 );
	pio_set_output(BUZZ_PIO, BUZZ_PIO_IDX_MASK, estado, 0, 0 );
	
};
void RTC_init(){
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);
	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(RTC, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 0);
	NVIC_EnableIRQ(RTC_IRQn);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(RTC,  RTC_IER_ALREN);
	
}

static void configure_lcd(void){
	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_WHITE);
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_WHITE);

	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
}
/**
* \brief Set maXTouch configuration
*
* This function writes a set of predefined, optimal maXTouch configuration data
* to the maXTouch Xplained Pro.
*
* \param device Pointer to mxt_device struct
*/
static void mxt_init(struct mxt_device *device)
{
	enum status_code status;

	/* T8 configuration object data */
	uint8_t t8_object[] = {
		0x0d, 0x00, 0x05, 0x0a, 0x4b, 0x00, 0x00,
		0x00, 0x32, 0x19
	};

	/* T9 configuration object data */
	uint8_t t9_object[] = {
		0x8B, 0x00, 0x00, 0x0E, 0x08, 0x00, 0x80,
		0x32, 0x05, 0x02, 0x0A, 0x03, 0x03, 0x20,
		0x02, 0x0F, 0x0F, 0x0A, 0x00, 0x00, 0x00,
		0x00, 0x18, 0x18, 0x20, 0x20, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x02,
		0x02
	};

	/* T46 configuration object data */
	uint8_t t46_object[] = {
		0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x03,
		0x00, 0x00
	};
	
	/* T56 configuration object data */
	uint8_t t56_object[] = {
		0x02, 0x00, 0x01, 0x18, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
		0x1E, 0x1E, 0x1E, 0x1E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00
	};

	/* TWI configuration */
	twihs_master_options_t twi_opt = {
		.speed = MXT_TWI_SPEED,
		.chip  = MAXTOUCH_TWI_ADDRESS,
	};

	status = (enum status_code)twihs_master_setup(MAXTOUCH_TWI_INTERFACE, &twi_opt);
	Assert(status == STATUS_OK);

	/* Initialize the maXTouch device */
	status = mxt_init_device(device, MAXTOUCH_TWI_INTERFACE,
	MAXTOUCH_TWI_ADDRESS, MAXTOUCH_XPRO_CHG_PIO);
	Assert(status == STATUS_OK);

	/* Issue soft reset of maXTouch device by writing a non-zero value to
	* the reset register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_RESET, 0x01);

	/* Wait for the reset of the device to complete */
	delay_ms(MXT_RESET_TIME);

	/* Write data to configuration registers in T7 configuration object */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 0, 0x20);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 1, 0x10);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 2, 0x4b);
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_POWERCONFIG_T7, 0) + 3, 0x84);

	/* Write predefined configuration data to configuration objects */
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_GEN_ACQUISITIONCONFIG_T8, 0), &t8_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_TOUCH_MULTITOUCHSCREEN_T9, 0), &t9_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_SPT_CTE_CONFIGURATION_T46, 0), &t46_object);
	mxt_write_config_object(device, mxt_get_object_address(device,
	MXT_PROCI_SHIELDLESS_T56, 0), &t56_object);

	/* Issue recalibration command to maXTouch device by writing a non-zero
	* value to the calibrate register */
	mxt_write_config_reg(device, mxt_get_object_address(device,
	MXT_GEN_COMMANDPROCESSOR_T6, 0)
	+ MXT_GEN_COMMANDPROCESSOR_CALIBRATE, 0x01);
}
void draw_screen(void) {
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GRAY));
	ili9488_draw_rectangle(0,0,480,320);
	ili9488_fill(COLOR_CONVERT(COLOR_GRAY));
}
uint32_t convert_axis_system_x(uint32_t touch_y) {
	// entrada: 4096 - 0 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_WIDTH - ILI9488_LCD_WIDTH*touch_y/4096;
}
uint32_t convert_axis_system_y(uint32_t touch_x) {
	// entrada: 0 - 4096 (sistema de coordenadas atual)
	// saida: 0 - 320
	return ILI9488_LCD_HEIGHT*touch_x/4096;
}
void update_screen(uint32_t tx, uint32_t ty) {
	if (locked)
	{
		PASS;
	}
	else if(ty >= BUTTON_Y && ty <= BUTTON_Y+BUTTON_H) {
		
		if(tx >= BUTTON_X && tx <= BUTTON_X + BUTTON_W && open_door==0) {
			if (playing == 0)
			{
				playing = 1;
				
				
				if (now_using->heavy)
				{
					minutes_to_finish=(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo)+(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo)/5;
					
				}
				else{
					minutes_to_finish=(now_using->centrifugacaoTempo+ now_using->enxagueQnt*now_using->enxagueTempo);
				}
				seconds_to_finish=2*now_using->enxagueQnt;
				enx_q = 0;
				has_centrifug = 0;
				valvula_on = 0;
				RTC_init();
				
				
				
				pio_clear(LED4_PIO,LED4_PIN_MASK);
				rtc_enable_interrupt(RTC,  RTC_IER_SECEN);
				rtc_get_date(RTC,&YEAR2,&MONTH2,&DAY2,&WEEK2);
				rtc_get_time(RTC,&HOUR2,&MINUTE2,&SECOND2);
				rtc_set_date_alarm(RTC, 1, MONTH2, 1, DAY2);
				rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2, 1, SECOND2+1);
				
				pio_set(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
				
				if(now_using->heavy){
					enx_time_30 = ((now_using->enxagueTempo)-30)+ (now_using->enxagueTempo)/5;
				}
				else{
					enx_time_30 = ((now_using->enxagueTempo)-30);
				}
				
			}
			else if (playing == 1)
			{
				playing = 0;
				pio_set(LED4_PIO,LED4_PIN_MASK);
				pio_set(LED3_PIO,LED3_PIN_MASK);
				pio_set(LED2_PIO,LED2_PIN_MASK);
				pmc_disable_periph_clk(ID_TC4);
				NVIC_DisableIRQ((IRQn_Type) ID_TC4);
				rtc_disable_interrupt(RTC,  RTC_IER_SECEN);
			}
		}
		else if (tx >= 120 && tx <= 240)
		{
			locked = 1;
		}
		else if(tx >= 0 && tx <= 120 && playing==0){
			t_ciclo *man = now_using->previous;
			now_using = man;
		}
		else if(tx >= 360 && tx <= 480 && playing==0){
			t_ciclo *man = now_using->next;
			now_using = man;
		}
		draw_button();
	}
}
void mxt_handler(struct mxt_device *device)
{
	/* USART tx buffer initialized to 0 */
	char tx_buf[STRING_LENGTH * MAX_ENTRIES] = {0};
	uint8_t i = 0; /* Iterator */

	/* Temporary touch event data struct */
	struct mxt_touch_event touch_event;

	/* Collect touch events and put the data in a string,
	* maximum 2 events at the time */
	do {
		/* Temporary buffer for each new touch event line */
		char buf[STRING_LENGTH];
		
		/* Read next next touch event in the queue, discard if read fails */
		if (mxt_read_touch_event(device, &touch_event) != STATUS_OK) {
			continue;
		}
		
		// eixos trocados (quando na vertical LCD)
		uint32_t conv_x = convert_axis_system_x(touch_event.x);
		uint32_t conv_y = convert_axis_system_y(touch_event.y);
		
		/* Format a new entry in the data string that will be sent over USART */
		sprintf(buf, "Nr: %1d, X:%4d, Y:%4d, Status:0x%2x conv X:%3d Y:%3d\n\r",
		touch_event.id, touch_event.x, touch_event.y,
		touch_event.status, conv_x, conv_y);
		
		if (touched==0)
		{
			update_screen(conv_x, conv_y);
			touched=1;
		}
		
		/* Add the new string to the string buffer */
		strcat(tx_buf, buf);
		i++;

		/* Check if there is still messages in the queue and
		* if we have reached the maximum numbers of events */
	} while ((mxt_is_message_pending(device)) & (i < MAX_ENTRIES));

	/* If there is any entries in the buffer, send them over USART */
	if (i > 0) {
		usart_serial_write_packet(USART_SERIAL_EXAMPLE, (uint8_t *)tx_buf, strlen(tx_buf));
	}
}
int main(void)
{
	struct mxt_device device; /* Device data container */

	/* Initialize the USART configuration struct */
	const usart_serial_options_t usart_serial_options = {
		.baudrate     = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength   = USART_SERIAL_CHAR_LENGTH,
		.paritytype   = USART_SERIAL_PARITY,
		.stopbits     = USART_SERIAL_STOP_BIT
	};
	
	c_rapido.next = &c_diario;
	c_rapido.previous = &c_teste;

	c_diario.next = &c_pesado;
	c_diario.previous = &c_rapido;

	c_pesado.next = &c_enxague;
	c_pesado.previous = &c_diario;

	c_enxague.next = &c_centrifuga;
	c_enxague.previous = &c_pesado;

	c_centrifuga.next = &c_teste;
	c_centrifuga.previous = &c_enxague;
	
	c_teste.next = &c_rapido;
	c_teste.previous = &c_centrifuga;

	sysclk_init(); /* Initialize system clocks */
	board_init();  /* Initialize board */
	configure_lcd();
	draw_screen();
	draw_button(0);
	/* Initialize the mXT touch device */
	mxt_init(&device);
	TC_init(TC0, ID_TC1, 1, 1);
	
	LED_init(1);
	BUT_init();
	pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
	
	/* Initialize stdio on USART */
	stdio_serial_init(USART_SERIAL_EXAMPLE, &usart_serial_options);

	printf("\n\rmaXTouch data USART transmitter\n\r");
	

	while (true) {
		/* Check for any pending messages and run message handler if any
		* message is found in the queue */
		if (mxt_is_message_pending(&device)) {
			mxt_handler(&device);
		}
		if(rtc_happen){
			
			
			//valvulas da bomba de �gua funcionam. Quando acaba, da 2s antes do outro enxague
			if(enx_q < (now_using->enxagueQnt)){
				if(valvula_on==0){
					valvula_on++;
					pin_toggle(LED2_PIO,LED2_PIN_MASK);
					rtc_get_date(RTC,&YEAR2,&MONTH2,&DAY2,&WEEK2);
					rtc_get_time(RTC,&HOUR2,&MINUTE2,&SECOND2);
					rtc_set_date_alarm(RTC, 1, MONTH2, 1, DAY2);
					rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2+now_using->enxagueTempo, 1, SECOND2);
				}
				else if(valvula_on){
					pin_toggle(LED2_PIO,LED2_PIN_MASK);
					valvula_on=0;
					enx_q++;
					rtc_get_date(RTC,&YEAR2,&MONTH2,&DAY2,&WEEK2);
					rtc_get_time(RTC,&HOUR2,&MINUTE2,&SECOND2);
					rtc_set_date_alarm(RTC, 1, MONTH2, 1, DAY2);
					rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2, 1, SECOND2+2);
				}
			}
			
			else if(has_centrifug==0 && now_using->centrifugacaoRPM>0){
				rtc_get_date(RTC,&YEAR2,&MONTH2,&DAY2,&WEEK2);
				rtc_get_time(RTC,&HOUR2,&MINUTE2,&SECOND2);
				
				rtc_set_date_alarm(RTC, 1, MONTH2, 1, DAY2);
				if(now_using->heavy){
					rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2+(now_using->centrifugacaoTempo)+(now_using->centrifugacaoTempo)/5, 1, SECOND2);
				}
				else{
					rtc_set_time_alarm(RTC, 1, HOUR2, 1, MINUTE2+(now_using->centrifugacaoTempo)+(now_using->centrifugacaoTempo)/5, 1, SECOND2);
				}
				TC_init(TC1, ID_TC4, 1, now_using->centrifugacaoRPM/60);
				pmc_enable_periph_clk(ID_TC4);
				has_centrifug=1;
			}
			
			else {
				//fim do ciclo - BIP
				
				
				playing = 0;
				NVIC_DisableIRQ((IRQn_Type) ID_TC4);
				rtc_disable_interrupt(RTC,  RTC_IER_SECEN);
				pio_set(LED4_PIO,LED4_PIN_MASK);
				pio_set(LED3_PIO,LED3_PIN_MASK);
				pio_set(LED2_PIO,LED2_PIN_MASK);
				draw_button();
				for(int x = 0;x<3;x++){
					pio_set(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
					delay_ms(400);
					pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
					delay_ms(400);
				}
			}
			rtc_happen=0;
		}
		
		if(rtc_sec_happen){
			pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
			if(playing==0){
				rtc_disable_interrupt(ID_RTC,RTC_IER_SECEN);
			}
			/* limpa interrupcao segundos */
			rtc_clear_status(RTC, RTC_SCCR_SECCLR);
			if(seconds_to_finish==0 && minutes_to_finish>0){
				minutes_to_finish-=1;
				seconds_to_finish=59;
			}
			else if(seconds_to_finish==0 && minutes_to_finish==0){
				//Finish do time, n nescessariamente do sistema. Erro de mais ou menos 2 segundos entre interrupcao de segundo e de eventos.
				minutes_to_finish = 00;
				seconds_to_finish = 00;
			}
			else{
				seconds_to_finish--;
			}
			draw_text();
			rtc_sec_happen=0;
		}
		if(stop_from_door){
			draw_button();
			stop_from_door=0;
			pio_set(LED4_PIO,LED4_PIN_MASK);
			pio_set(LED3_PIO,LED3_PIN_MASK);
			pio_set(LED2_PIO,LED2_PIN_MASK);
			pmc_disable_periph_clk(ID_TC4);
			NVIC_DisableIRQ((IRQn_Type) ID_TC4);
			rtc_disable_interrupt(RTC,  RTC_IER_SECEN);
		}
		if(lockings){
			lockings=0;
			draw_button();
		}
		
		
		
		
	}
	return 0;
}
