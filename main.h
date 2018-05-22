#include "stm32f405xx.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rtc.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_tim.h"
#include "string.h"
#include "math.h"

#define F_CPU       168000000UL
#define AHB1        F_CPU
#define APB1        F_CPU/4
#define APB1_TIM    APB1*2
#define APB2        F_CPU/2
#define APB2_TIM    APB2*2
#define SysTicksClk 10000
#define SysTicks    F_CPU/SysTicksClk

#define	MEM_ADDRESS 		0x0800C000
#define RADIO_FREQ_ADR	    MEM_ADDRESS + 0x04
#define TDA_MAIN_LOUD_ADR	MEM_ADDRESS + 0x10
#define TDA_TREB_ADR	    MEM_ADDRESS + 0x14
#define TDA_MIDD_ADR	    MEM_ADDRESS + 0x18
#define TDA_BASS_ADR	    MEM_ADDRESS + 0x1C
#define TDA_SATT_ADR	    MEM_ADDRESS + 0x20


enum PINs
{
    PIN0=0, PIN1,   PIN2,   PIN3,
    PIN4,   PIN5,   PIN6,   PIN7,
    PIN8,   PIN9,   PIN10,  PIN11,
    PIN12,  PIN13,  PIN14,  PIN15
};

#define AMP_ON      GPIOB->BSRR |= GPIO_BSRR_BR3
#define AMP_OFF     GPIOB->BSRR |= GPIO_BSRR_BS3

#define BT_ON       GPIOC->BSRR |= GPIO_BSRR_BS13
#define BT_OFF      GPIOC->BSRR |= GPIO_BSRR_BR13

#define BULB_CH_ON  GPIOB->BSRR |= GPIO_BSRR_BS6
#define BULB_CH_OFF GPIOB->BSRR |= GPIO_BSRR_BR6

#define BT_SOUCE     GPIOA->IDR & GPIO_PIN_10 //BT1 <-> Source/onoff
#define BT_PREV      GPIOA->IDR & GPIO_PIN_11 //BT2 <-> PREV
#define BT_NEXT      GPIOA->IDR & GPIO_PIN_12 //BT3 <-> NEXT
#define BT_PP        GPIOA->IDR & GPIO_PIN_15 //BT4 <-> Play/payse 
#define BT_VOL_UP    GPIOC->IDR & GPIO_PIN_10 //BT5 <-> VOL+
#define BT_VOL_DOWN  GPIOC->IDR & GPIO_PIN_11 //BT6 <-> VOL-

#define BT_ENC_A     GPIOB->IDR & GPIO_PIN_5 //ENC_A <-> Pref
#define BT_ENC_B     GPIOB->IDR & GPIO_PIN_4 //ENC_B <-> Reserved

#define BT_CLK_DOWN GPIOA->IDR & GPIO_PIN_8  //Clock DOWN
#define BT_CLK_UP   GPIOA->IDR & GPIO_PIN_9  //Clock UP


enum STATEs
{
    LOAD=0,
	AUDIO_OFF,
    MAIN,
    SET_TIME,
    TDA_SETT
};

enum SET_TIME_STATEs
{
    SET_TIME_HOUR=0,
    SET_TIME_MIN,
    SET_DATE_YEAR,
    SET_DATE_MON,
    SET_DATE_DAY,
    SET_DATE_WEEK_DAY,
    SET_SET
};

enum INPUTs
{
    FM=0,
    BT,
    USB,
    AUX
};

enum TDA_SET_STATEs
{
    TDA_SET_LOUDNESS=0,
    TDA_SET_TREB,
    TDA_SET_MIDD,
    TDA_SET_BASS,
    TDA_SET_SPEAKER_ATT
};

enum TDA_SET_LOUD_STATEs
{
    TDA_SET_LOUD_CENT_FREQ=0,
    TDA_SET_LOUD_ATTENUATION,
    TDA_SET_LOUD_HIGN_BOOST
};
enum TDA_SET_TREB_STATEs
{
    TDA_SET_TREB_CENT_FREQ=0,
    TDA_SET_TREB_ATTENUATION
};
enum TDA_SET_MIDD_STATEs
{
    TDA_SET_MIDD_CENT_FREQ=0,
    TDA_SET_MIDD_ATTENUATION,
    TDA_SET_MIDD_Q_FACTOR
};
enum TDA_SET_BASS_STATEs
{
    TDA_SET_BASS_CENT_FREQ=0,
    TDA_SET_BASS_ATTENUATION,
    TDA_SET_BASS_Q_FACTOR
};
enum TDA_SET_SATT_STATES
{
    TDA_SET_SATT_L_F=0,
    TDA_SET_SATT_R_F,
    TDA_SET_SATT_L_R,
    TDA_SET_SATT_R_R
};


/*--------------------------------------------------------------------------------*/
//TDA Defs
#define TDA7419_ADDRESS		0x88
#define TDA_MAIN_SOURCE		0x00
#define TDA_SOURCE_MUTE		0x07

#define TDA_SOFT_MUTE       0x02
#define TDA_MUTE            0xC6
#define TDA_UNMUTE          0xC7

#define TDA_VOLUME		    0x03

#define TDA_MAIN_LOUDNESS   0x01
#define TDA_TREBLE_FILTER   0x04
#define TDA_MIDDLE_FILTER   0x05
#define TDA_BASS_FILTER     0x06
#define TDA_M_B_CENT_FREQ   0x08
#define TDA_SPEAKER_ATT_LF  0x0A
#define TDA_SPEAKER_ATT_RF  0x0B
#define TDA_SPEAKER_ATT_LR  0x0C
#define TDA_SPEAKER_ATT_RR  0x0D

//const uint8_t TDA_inputs[4] = {0x82, 0x84, 0x81, 0x83};
const uint8_t TDA_inputs[4] = {0xFA, 0xF9, 0xFC, 0xFB};



/*--------------------------------------------------------------------------------*/
//TFT commands
uint8_t TFT_reset[7] = {'r','e','s','t',255,255,255};

uint8_t loading_txt[13] = {'l','o','d','.','v','a','l','=','0','0',255,255,255};

uint8_t TFT_TIME[56] = {'t','i','m','e','.','t','x','t','=','"',0,0,':',0,0,'"',255,255,255,\
                        'd','a','t','e','.','t','x','t','=','"',' ',' ',0,0,'.',0,0,'.',0,0,' ',\
						' ',' ',' ','d','a','y','o','f','w','e','e','k','"',255,255,255}; 

uint8_t pages[4][16] = {{'p','a','g','e',' ','l','o','a','d',255,255,255},
                        {'p','a','g','e',' ','a','o','f','f',255,255,255},
                        {'p','a','g','e',' ','m','a','i','n',255,255,255},
                        {'p','a','g','e',' ','s','e','t','t',255,255,255}};

uint8_t tft_apage[12] = {'v','a','0','.','v','a','l','=','1',255,255,255};
                                
uint8_t input_tft[4][32] = {{'i','n','p','.','t','x','t','=','"','F','M','"',255,255,255,'i','n','p','.','b','c','o','=','0','3','4','8','1','5',255,255,255},
                            {'i','n','p','.','t','x','t','=','"','B','T','"',255,255,255,'i','n','p','.','b','c','o','=','0','1','4','8','4','7',255,255,255},
                            {'i','n','p','.','t','x','t','=','"','U','S','B','"',255,255,255,'i','n','p','.','b','c','o','=','6','3','4','8','8',255,255,255},
                            {'i','n','p','.','t','x','t','=','"','A','U','X','"',255,255,255,'i','n','p','.','b','c','o','=','6','4','5','1','2',255,255,255},};
                                
uint8_t set_time_tft[7][24] = {{'s','e','t','.','v','a','l','=','0',255,255,255,'t','_','h','.','v','a','l','=','1',255,255,255},
                               {'t','_','h','.','v','a','l','=','0',255,255,255,'t','_','m','.','v','a','l','=','1',255,255,255},
                               {'t','_','m','.','v','a','l','=','0',255,255,255,'d','_','y','.','v','a','l','=','1',255,255,255},
                               {'d','_','y','.','v','a','l','=','0',255,255,255,'d','_','m','.','v','a','l','=','1',255,255,255},
                               {'d','_','m','.','v','a','l','=','0',255,255,255,'d','_','d','.','v','a','l','=','1',255,255,255},
                               {'d','_','d','.','v','a','l','=','0',255,255,255,'d','_','w','.','v','a','l','=','1',255,255,255},
                               {'d','_','w','.','v','a','l','=','0',255,255,255,'s','e','t','.','v','a','l','=','1',255,255,255}};

uint8_t set_time_txt[97] = {'t','_','h','.','t','x','t','=','"','0','0','"',255,255,255,\
                            't','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','m','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','d','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','y','.','t','x','t','=','"','0','0','"',255,255,255,\
                            'd','_','w','.','t','x','t','=','"','0','0','0','0','0','0','0','0','0','"',255,255,255,};

uint8_t day_of_week[7][9] = {" Monday  ",
                             " Tuesday ",
                             "Wednesday",
                             "Thursday ",
                             " Friday  ",
                             "Saturday ",
                             " Sunday  "};

uint8_t main_FM_text[22] = 	   {'t','e','x','t','.','t','x','t','=','"','0','0','0','.','0',' ','F','M','"',255,255,255};
uint8_t main_BT_text[4][21] = {{'t','e','x','t','.','t','x','t','=','"',' ','N','O',' ','D','e','v','"',255,255,255},
							   {'t','e','x','t','.','t','x','t','=','"','C','o','n','n','e','c','t','"',255,255,255},
							   {'t','e','x','t','.','t','x','t','=','"',' ','P','L','A','Y',' ',' ','"',255,255,255},
							   {'t','e','x','t','.','t','x','t','=','"',' ','P','A','U','S','E',' ','"',255,255,255}};

uint8_t main_AUX_text[17] =  {'t','e','x','t','.','t','x','t','=','"','A','U','X','"',255,255,255};

uint8_t main_VOL_text[32] = {'v','o','l','.','t','x','t','=','"','-','7','9','"',255,255,255,'v','o','l','.','b','c','o','=','6','5','5','3','5',255,255,255};
uint8_t main_VOL_mute[32] = {'v','o','l','.','t','x','t','=','"','M','U','T','"',255,255,255,'v','o','l','.','b','c','o','=','6','3','4','8','8',255,255,255};

uint8_t main_USB_text[44] = {'t','e','x','t','.','t','x','t','=','"','0','0','0','/','0','0','0',' ','_','_','_','_','_','_',' ',' ',' ',' ',' ',' ','P','A','U','S','E',' ',' ',' ',' ',' ','"',255,255,255};
uint8_t main_NO_USB_text[21] = {'t','e','x','t','.','t','x','t','=','"',' ','N','O',' ','U','S','B','"',255,255,255};
    
uint8_t main_text_font_4[14] = {'t','e','x','t','.','f','o','n','t','=','4',255,255,255};
uint8_t main_text_font_5[14] = {'t','e','x','t','.','f','o','n','t','=','5',255,255,255};
uint8_t main_text_font_7[14] = {'t','e','x','t','.','f','o','n','t','=','7',255,255,255};
	
uint8_t ADC_text[25] = {'A','D','C','.','t','x','t','=','"','0','0','.','0','V',' ',' ',' ','0','0','0','%','"',255,255,255};
    
uint8_t tda_sett_pages[5][20] = {{'p','a','g','e',' ','t','d','a','_','l','o','u','d',255,255,255},
                                 {'p','a','g','e',' ','t','d','a','_','t','r','e','b',255,255,255},
                                 {'p','a','g','e',' ','t','d','a','_','m','i','d','d',255,255,255},
                                 {'p','a','g','e',' ','t','d','a','_','b','a','s','s',255,255,255},
                                 {'p','a','g','e',' ','t','d','a','_','s','a','t','t',255,255,255}};

uint8_t tda_set_LTMB[3][48] = {{'a','t','t','.','b','c','o','=','6','5','5','3','5',255,255,255,'h','_','q','.','b','c','o','=','6','5','5','3','5',255,255,255,'c','_','f','.','b','c','o','=','6','3','4','8','8',255,255,255},
                               {'h','_','q','.','b','c','o','=','6','5','5','3','5',255,255,255,'c','_','f','.','b','c','o','=','6','5','5','3','5',255,255,255,'a','t','t','.','b','c','o','=','6','3','4','8','8',255,255,255},
                               {'c','_','f','.','b','c','o','=','6','5','5','3','5',255,255,255,'a','t','t','.','b','c','o','=','6','5','5','3','5',255,255,255,'h','_','q','.','b','c','o','=','6','3','4','8','8',255,255,255}};
                                   
uint8_t tda_set_satt[4][48] = {{'l','_','f','.','v','a','l','=','1',255,255,255,'r','_','f','.','v','a','l','=','0',255,255,255,\
                                'l','_','r','.','v','a','l','=','0',255,255,255,'r','_','r','.','v','a','l','=','0',255,255,255},
                               {'l','_','f','.','v','a','l','=','0',255,255,255,'r','_','f','.','v','a','l','=','1',255,255,255,\
                                'l','_','r','.','v','a','l','=','0',255,255,255,'r','_','r','.','v','a','l','=','0',255,255,255},
                               {'l','_','f','.','v','a','l','=','0',255,255,255,'r','_','f','.','v','a','l','=','0',255,255,255,\
                                'l','_','r','.','v','a','l','=','1',255,255,255,'r','_','r','.','v','a','l','=','0',255,255,255},
                               {'l','_','f','.','v','a','l','=','0',255,255,255,'r','_','f','.','v','a','l','=','0',255,255,255,\
                                'l','_','r','.','v','a','l','=','0',255,255,255,'r','_','r','.','v','a','l','=','1',255,255,255}};

uint8_t tda_set_loud[55] = {'c','_','f','.','t','x','t','=','"','_','_','_','_',' ','H','z','"',255,255,255,\
                            'a','t','t','.','t','x','t','=','"','-','_','_',' ','d','B','"',255,255,255,\
                            'h','_','q','.','t','x','t','=','"','_','_','_','"',255,255,255};

uint8_t tda_set_treb[39] = {'c','_','f','.','t','x','t','=','"','1','_','.','_','k','H','z','"',255,255,255,\
                            'a','t','t','.','t','x','t','=','"','-','_','_',' ','d','B','"',255,255,255};

uint8_t tda_set_midd[56] = {'c','_','f','.','t','x','t','=','"','_','_','0','0',' ','H','z','"',255,255,255,\
                            'a','t','t','.','t','x','t','=','"','-','_','_',' ','d','B','"',255,255,255,\
                            'h','_','q','.','t','x','t','=','"','_','.','_','_','"',255,255,255};

uint8_t tda_set_bass[55] = {'c','_','f','.','t','x','t','=','"','_','_','0',' ','H','z','"',255,255,255,\
                            'a','t','t','.','t','x','t','=','"','-','_','_',' ','d','B','"',255,255,255,\
                            'h','_','q','.','t','x','t','=','"','_','.','_','_','"',255,255,255};

uint8_t tda_set_satt_val[64] = {'l','_','f','.','t','x','t','=','"','-','7','9','"',255,255,255,\
                                'r','_','f','.','t','x','t','=','"','-','7','9','"',255,255,255,\
                                'l','_','r','.','t','x','t','=','"','-','7','9','"',255,255,255,\
                                'r','_','r','.','t','x','t','=','"','-','7','9','"',255,255,255};


																 
/*--------------------------------------------------------------------------------*/
// BT
#define BT_RX_BUFF_SIZE 20

enum BT_querys
{
    BT_STATUS=0,
    BT_PLAY_PAUSE,
    BT_FORWARD,
    BT_BACKWARD,
	BT_DISC
};
uint8_t bt_tx_query[5][7] = {{'A','T','#','M','V',13,10},
                             {'A','T','#','M','A',13,10},
                             {'A','T','#','M','D',13,10},
                             {'A','T','#','M','E',13,10},
                             {'A','T','#','M','J',13,10}};
enum BT_Statuses
{
    BT_NO_DEV=0,
    BT_CONN,
    BT_PLAY,
    BT_PAUSE
};

/*--------------------------------------------------------------------------------*/

#define USB_CMD_NEXT        0x01
#define USB_CMD_PREV        0x02
#define USB_CMD_VOL         0x06
#define USB_CMD_SOURCE      0x09
#define USB_CMD_RESET       0x0C
#define USB_CMD_PLAY        0x0D
#define USB_CMD_PAUSE       0x0E
#define USB_CMD_PLAY        0x0D
#define USB_CMD_PLAY_MODE   0x11

#define USB_Q_STATUS        0x42
#define USB_Q_PLAY_MODE     0x45
#define USB_Q_TRACK_COUNT   0x48
#define USB_Q_TRACK_NUMBER  0x4C
#define USB_Q_TRACK_LONG    0x51
#define USB_Q_TRACK_TIME    0x50
#define USB_Q_TRACK_NAME    0x52

enum USB_Statuses
{
    USB_STOP=0,
    USB_PLAY,
    USB_PAUSE
};

uint8_t USB_command[4]  = {0x7E, 0x02, 0, 0xEF};
uint8_t USB_command5[5] = {0x7E, 0x03, 0, 0, 0xEF};

/*--------------------------------------------------------------------------------*/
//ADC
#define ADC_BUF_NUM 2
/*--------------------------------------------------------------------------------*/
void TIM8_TRG_COM_TIM14_IRQHandler(void);
//void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
void UART5_IRQHandler(void);
//void UART4_IRQHandler(void);
                                
void Init_RCC(void);
void SysTick_Handler(void);

uint32_t flash_read(uint32_t address);
uint8_t flash_ready(void);
void flash_erase_sector(uint8_t sector);
void flash_write(uint32_t address, uint32_t data);
void flash_write_newdata(void);
	
void Init_GPIO(void);
void Init_RTC(void);
void Init_TFT(void);
void TFT_send(uint8_t *buff, uint8_t size);
void Init_KEYs_TIM(void);

void Init_BT(void);
void BT_send(uint8_t query);

void Init_USB(void);
uint8_t USB_send(uint8_t CMD);
uint8_t USB_send_par(uint8_t CMD, uint8_t PAR);

void Init_I2C1(void);
uint8_t I2C1_Send(uint8_t addres,uint8_t *buff, uint16_t size);
uint8_t TEA_set_freq(uint16_t freq);
                            
uint8_t Init_TDA(void);

void Init_ADC(void);

void Init_IWDG(void);
void IWDG_res(void);
