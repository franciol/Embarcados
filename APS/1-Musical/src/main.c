/**
** Entrega realizada em parceria com:
**  - Francisco Ciol Rodrigues Aveiro
**  - Eric Fernando Otofuji Abrantes
**
**  - url v�deo: https://youtu.be/PedGtK7MFJw
**
**	-Musicas
**		-https://www.youtube.com/watch?v=1pOHf_N8CDY(Banana) - Pisca Regularmente
**			(Red Velvet - Power Up)

**		-https://www.youtube.com/watch?v=y6120QOlsfU (Darude SandStorm) - N�o Pisca
**			(Darude Sandstorm)
**		-https://www.youtube.com/watch?v=27mB8verLK8 (Piratas do Caribe) - Pisca Irregular
**			(Musica tema de piratas do caribe - Klaus Badelt)
**			Link do codigo do piratas do caribe: https://github.com/xitangg/-Pirates-of-the-Caribbean-Theme-Song/blob/master/Pirates_of_the_Caribbean_-_Theme_Song.ino
**/

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

#define BUZZ_PIO           PIOD
#define BUZZ_PIO_ID        13
#define BUZZ_PIO_IDX       22u
#define BUZZ_PIO_IDX_MASK  (1u << BUZZ_PIO_IDX)

#define BUT2_PIO           PIOB
#define BUT2_PIO_ID        11
#define BUT2_PIO_IDX       0u
#define BUT2_PIO_IDX_MASK  (1u << BUT2_PIO_IDX)

#define BUT3_PIO           PIOC
#define BUT3_PIO_ID        12
#define BUT3_PIO_IDX       30u
#define BUT3_PIO_IDX_MASK  (1u << BUT3_PIO_IDX)

#define LED_PIO				PIOC
#define LED_PIO_ID			12
#define LED_PIO_IDX			8u
#define LED_PIO_IDX_MASK	(1u << LED_PIO_IDX)

/************************************************************************/
/* constants                                                            */
/************************************************************************/
//Defining note frequency
#define NOTE_G3	 198
#define NOTE_A3  222
#define NOTE_B3	 249
#define NOTE_C4  262
#define NOTE_CS4 280
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 373
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988
//*****************************************

//*****************************************
int musica1[] = {
	
	//Note of the song, 0 is a rest/pulse
	NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	NOTE_A4, NOTE_G4, NOTE_A4, 0,
	
	NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	NOTE_A4, NOTE_G4, NOTE_A4, 0,
	
	NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_D5, NOTE_E5, NOTE_A4, 0,
	NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	NOTE_C5, NOTE_A4, NOTE_B4, 0,

	NOTE_A4, NOTE_A4,
	//Repeat of first part
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	NOTE_A4, NOTE_G4, NOTE_A4, 0,

	NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
	NOTE_A4, NOTE_G4, NOTE_A4, 0,
	
	NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
	NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0,
	NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
	NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
	
	NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
	NOTE_D5, NOTE_E5, NOTE_A4, 0,
	NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
	NOTE_C5, NOTE_A4, NOTE_B4, 0,
	//End of Repeat

	NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

	NOTE_E5, 0, 0, NOTE_F5, 0, 0,
	NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
	NOTE_D5, 0, 0, NOTE_C5, 0, 0,
	NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};


int tempo1[] = {
	//duration of each note (in ms) Quarter Note is set to 250 ms
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
	
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
	
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 125, 250, 125,

	125, 125, 250, 125, 125,
	250, 125, 250, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 375,

	250, 125,
	//Rpeat of First Part
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
	
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 125,
	
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 250, 125, 125,
	125, 125, 125, 250, 125,

	125, 125, 250, 125, 125,
	250, 125, 250, 125,
	125, 125, 250, 125, 125,
	125, 125, 375, 375,
	//End of Repeat
	
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500,

	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 125, 125, 125, 375,
	250, 125, 375, 250, 125, 375,
	125, 125, 125, 125, 125, 500
};

int musica2[] = {
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_B3,
	0,NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_A3,
	0,NOTE_A3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_B3,
	0,NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_A3,
	0,NOTE_A3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_B3,
	0,NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_A3,
	0,NOTE_A3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_B3,
	0,NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3,
	
	NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_D4,
	0,NOTE_D4,0,NOTE_FS4,NOTE_E4,NOTE_FS4,NOTE_E4,NOTE_D4,
	NOTE_B3,0,NOTE_D4,NOTE_CS4,NOTE_D4,NOTE_A3,
	0,NOTE_A3,0,NOTE_D4,NOTE_CS4,NOTE_E4,NOTE_D4,NOTE_A3
	
	
	
};

int tempo2[] = {
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125,
	125,255,125,255,125,125,
	125,125,125,125,125,125,125,125
	
};

int musica3[] = {
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0,
	NOTE_E5,0,NOTE_E5,0,NOTE_E5,0,NOTE_D5,0,NOTE_D5,0,NOTE_D5,0,NOTE_D5,0,NOTE_A4,0,
	
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0,
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0,
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0,
	
	NOTE_E5,0,NOTE_E5,0,NOTE_E5,0,NOTE_D5,0,NOTE_D5,0,NOTE_D5,0,NOTE_D5,0,NOTE_A4,0,
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0,
	NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_B4,0,NOTE_E5,0
	
};


int tempo3[] = {
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5,
	380,5,125,5,255,5,255,5,380,5,125,5,255,5,255,5
};






/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile int isPlaying;
volatile Bool musica;
volatile int qual;
volatile int notaA;
/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/
void but2_callback(void){
	musica = true;
}

void but3_callback(void) {
	delay_ms(200);
	isPlaying = !isPlaying;
}

void pisca(void) {
	if(qual == 1){
		pio_clear(LED_PIO,LED_PIO_IDX_MASK);
		pio_set(LED_PIO,LED_PIO_IDX_MASK);
		//delay_ms(4000);
	}
	else if(qual == 0){
		pio_set(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(200);
		pio_clear(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(200);
	}
	else if(qual == 2){
		pio_set(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(200);
		pio_clear(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(200);
		pio_set(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(500);
		pio_clear(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(500);
		pio_set(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(500);
		pio_clear(LED_PIO,LED_PIO_IDX_MASK);
		delay_ms(200);
	}
}
/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


int nota(double freq, double tm,double velocidade,int notaAt)
{
	if(isPlaying==1){
		freq = 7*freq;
		pio_set(LED_PIO,LED_PIO_IDX_MASK);

		double val = 1000000 / ((freq) * 2);
		
		
		if(freq == 0){
			pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
			delay_us((tm*1000)/velocidade);
		}
		
		else{
			pio_clear(LED_PIO,LED_PIO_IDX_MASK);
			long numCycles = freq * tm/ 1000;
			
			for(int i = 0;i<(numCycles/velocidade);i++){
				pio_set(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
				delay_us(val);
				pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
				delay_us(val);
			}
		}
		
		pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
		return notaAt+1;
	}
	return notaAt;
}



// Fun��o de inicializa��o do uC
void init(void)
{
	isPlaying = 1;
	musica = false;
	qual = 0;
	sysclk_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	pmc_enable_periph_clk(BUZZ_PIO_ID);
	pmc_enable_periph_clk(BUT3_PIO_ID);
	pmc_enable_periph_clk(BUT2_PIO_ID);
	pmc_enable_periph_clk(LED_PIO_ID);
	
	pio_set_output(BUZZ_PIO,BUZZ_PIO_IDX_MASK,0,0,0);
	pio_set_output(LED_PIO,LED_PIO_IDX_MASK,0,0,0);

	pio_configure(BUT2_PIO,PIO_INPUT,BUT2_PIO_IDX,PIO_PULLUP);
	pio_configure(BUT3_PIO,PIO_INPUT,BUT3_PIO_IDX,PIO_PULLUP | PIO_DEBOUNCE);
	
	pio_handler_set(BUT2_PIO,
	BUT2_PIO_ID,
	BUT2_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but2_callback);
	
	pio_handler_set(BUT3_PIO,
	BUT3_PIO_ID,
	BUT3_PIO_IDX_MASK,
	PIO_IT_FALL_EDGE,
	but3_callback);
	
	
	pio_enable_interrupt(BUT2_PIO,BUT2_PIO_IDX_MASK);
	pio_enable_interrupt(BUT3_PIO,BUT3_PIO_IDX_MASK);
	
	NVIC_EnableIRQ(BUT2_PIO_ID);
	NVIC_SetPriority(BUT2_PIO_ID,4);
	
	NVIC_EnableIRQ(BUT3_PIO_ID);
	NVIC_SetPriority(BUT3_PIO_ID,4);

}



/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
	init();
	// Desativa watchdog
	WDT->WDT_MR = WDT_MR_WDDIS;
	notaA = 0;
	// super loop
	// aplicacoes embarcadas n�o devem sair do while(1).
	while (1)
	{
		pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
		pisca();
		if(musica==true){
			delay_ms(500);
			if(qual==0){
				qual = 1;
				notaA = 0;
			}
			else if(qual==1){
				qual = 2;
				notaA = 0;
			}
			else{
				qual = 0;
				notaA = 0;
			}
			musica = false;
		}
		
		
		
		if(qual==0 && isPlaying==1){
			while(notaA<(sizeof(musica2)/sizeof(musica2[0]))){
				notaA = nota(musica2[notaA],tempo2[notaA],0.7,notaA);
			}
			notaA = 0;
			isPlaying = 0;
			pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
		}
		else if(qual==1 && isPlaying==1){
			while(notaA<(sizeof(musica3)/sizeof(musica3[0]))){
				notaA = nota(musica3[notaA],tempo3[notaA],1.3,notaA);
				
			}
			
			notaA = 0;
			isPlaying = 0;
			pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
		}
		else if(qual==2 && isPlaying==1){
			while(notaA<(sizeof(musica1)/sizeof(musica1[0]))){
				notaA = nota(musica1[notaA],tempo1[notaA],1.3,notaA);
			}
			
			notaA = 0;
			isPlaying = 0;
			pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
		}
		
		
		
		
		
		
		
		
		
		
	}
	return 0;
}
