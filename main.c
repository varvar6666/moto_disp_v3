#include "main.h"

#ifdef DEBUG_BT

void UART5_IRQHandler(void)
{
    if(UART5->SR & USART_SR_RXNE)
        USART3->DR = UART5->DR;
}
    
void USART3_IRQHandler(void)
{
    if(USART3->SR & USART_SR_RXNE)
        UART5->DR = USART3->DR;
}
        
#endif

uint8_t OFF_counter = 0;

uint8_t I2C_res, USB_res;

uint16_t RADIO_FREQ = 1040;

int8_t VOLUME = 0;

uint32_t RawFreq;
uint32_t truefreq = 0;
uint8_t Flag = 0;

int main(void)
{
/**---------------------------------------------------------**/
 /*          Init all                                       */
    Init_RCC();                             //Clocks

    Init_GPIO();                            //GPIO
    
    AMP_OFF;                                //Off Amplifier
    BT_OFF;                                 //Off BlueTooth
    BULB_CH_ON;                             //Start check bulbs
    MUTE;
    
    FLASH->KEYR = 0x45670123;               //Enable access to FLASH
    FLASH->KEYR = 0xCDEF89AB;
    
    Init_TFT();                             //TFT
    TFT_send(TFT_reset,sizeof(TFT_reset));  //TFT reset to clear
    
    for(uint32_t delay = 0;delay < 10000000;delay++){}; // delay for TFT start
    
    loading_txt[8] = '1';
    loading_txt[9] = '4';
    TFT_send(loading_txt, sizeof(loading_txt)); //Add information for loading start
    
    Init_RTC();                             //Real-Time-Clock
    
    Init_ADC();                             //ADC
    
    Init_Pulse_IN();                        //Speed pulse IN
    
    Init_BT();                              //Bluetooth
    
    Init_USB();                             //MP3 player
    
    USB_res = USB_send_par(USB_CMD_SOURCE, 0x00);
    if(USB_res == 0)
    {
        USB_res = USB_send_par(USB_CMD_VOL, 0x1e);
        USB_res = USB_send_par(USB_CMD_PLAY_MODE, 0x00);
    }
    
    Init_I2C1();                            //I2C for TDA & TEA
        
    uint32_t tmp_mem;
    tmp_mem = flash_read(MEM_ADDRESS);
    if(tmp_mem != 0xFFFFFFFF)
    {
        GLOBAL_STATE = (0xFF000000 & tmp_mem) >> 24;
        if((GLOBAL_STATE != GLOBAL_STATE_AUDIO_OFF) && (GLOBAL_STATE != GLOBAL_STATE_MAIN))
            GLOBAL_STATE = GLOBAL_STATE_AUDIO_OFF;
        AUDIO_INPUT = (0xFF0000 & flash_read(MEM_ADDRESS)) >> 16;
        if(AUDIO_INPUT > AUDIO_AUX)
           AUDIO_INPUT = AUDIO_FM; 
        VOLUME = (0xFF00 & flash_read(MEM_ADDRESS)) >> 8;
    }
    else
    {
#ifdef DEBUG
        GLOBAL_STATE = GLOBAL_STATE_MAIN;
#else        
        GLOBAL_STATE = GLOBAL_STATE_AUDIO_OFF;
#endif
        AUDIO_INPUT = AUDIO_BT;
        VOLUME = 0;
    }
    TFT_send(pages[GLOBAL_STATE], sizeof(pages[GLOBAL_STATE]));
    
    tmp_mem = flash_read(RADIO_FREQ_ADR);
    if(tmp_mem != 0xFFFFFFFF) 
    {    
        RADIO_FREQ = (0xFFFF0000 & tmp_mem) >> 16;
        if((RADIO_FREQ > 1080) && (RADIO_FREQ < 880))
            RADIO_FREQ = 1040;
    }
    else
        RADIO_FREQ = 1040;
    
//        tmp_mem = flash_read(TDA_MAIN_LOUD_ADR);
//        TDA_loudness.high_boost  = (0xFF000000 & tmp_mem) >> 24;
//        TDA_loudness.center_freq = (0xFF0000   & tmp_mem) >> 16;
//        TDA_loudness.atteniation = (0xFF00     & tmp_mem) >> 8;
//        
//        tmp_mem = flash_read(TDA_TREB_ADR);
//        TDA_treble.center_freq = (0xFF000000 & tmp_mem) >> 24;
//        TDA_treble.atteniation = (0xFF0000   & tmp_mem) >> 16;
//        
//        tmp_mem = flash_read(TDA_MIDD_ADR);
//        TDA_middle.Q_factot    = (0xFF000000 & tmp_mem) >> 24;
//        TDA_middle.center_freq = (0xFF0000   & tmp_mem) >> 16;
//        TDA_middle.atteniation = (0xFF00     & tmp_mem) >> 8;
//        
//        tmp_mem = flash_read(TDA_BASS_ADR);
//        TDA_bass.Q_factot    = (0xFF000000 & tmp_mem) >> 24;
//        TDA_bass.center_freq = (0xFF0000   & tmp_mem) >> 16;
//        TDA_bass.atteniation = (0xFF00     & tmp_mem) >> 8;
//        
//        tmp_mem = flash_read(TDA_SATT_ADR);
//        TDA_sp_att.left_front  = (0xFF000000 & tmp_mem) >> 24;
//        TDA_sp_att.right_front = (0xFF0000   & tmp_mem) >> 16;
//        TDA_sp_att.left_rear   = (0xFF00     & tmp_mem) >> 8;
//        TDA_sp_att.right_rear  = (0xFF       & tmp_mem);
    
    I2C_res = Init_TDA();                   //TDA
        
    I2C_res = TEA_set_freq(RADIO_FREQ);     //FM
    
/**---------------------------------------------------------**/
 /*     Display on TFT                                      */
    if(GLOBAL_STATE == GLOBAL_STATE_MAIN)
    {
        TFT_send(input_tft[AUDIO_INPUT], sizeof(input_tft[AUDIO_INPUT]));
        
        TFT_send_vol();
        
        TFT_switch_audio_input();
        
        AMP_ON;
        UNMUTE;
    }
    
    Init_KEYs_TIM();

    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    
#ifdef DEBUG
    LED1_ON;
#endif
    BULB_CH_OFF;
    
    while (1)
    {
        if(Flag)
        {
            Flag = 0;
            truefreq = RawFreq;
        
        }
    }
}
/**=========================================================**/
 /*------------Main clocks Init-----------------------------*/
/**---------------------------------------------------------**/
void Init_RCC(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    /* Enable external resonator */
    RCC->CR |= RCC_CR_HSEON;

    do
    {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    }    
    while((HSEStatus == 0) && (StartUpCounter != ((uint32_t)1000U)));
    
    if( (RCC->CR & RCC_CR_HSIRDY) != RESET)
    {
        /* Включаем буфер предвыборки FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTEN;

        /* Конфигурируем Flash на 2 цикла ожидания */
    	/* Это нужно потому, что Flash не может работать на высокой частоте */        
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_5WS;   
        
        /* HCLK = SYSCLK || AHB prescaler*/
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB clk = 100MHz
        
    	/* PCLK1 = HCLK || APB Low speed prescaler (APB1)*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV4;

        /* PCLK2 = HCLK || APB high-speed prescaler (APB2)*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;
        
        /* Set PLL input sourse*/
        RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
        
        /*Set PLL M prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
        RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos); //8
        
        /*Set PLL N prescaler */
        RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
        RCC->PLLCFGR |= (168 << RCC_PLLCFGR_PLLN_Pos);
        
        /*Set PLL P prescaler */
        //RCC->PLLCFGR |= RCC_PLLCFGR_PLLP;
        
        RCC->CR |= RCC_CR_PLLON;
        
        while ((RCC->CR & RCC_CR_PLLRDY) == 0)
        {}
            
        /*Set SYSCLK as PLL */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;
            
        while ((RCC->CFGR & RCC_CFGR_SWS) !=  RCC_CFGR_SWS_PLL)
        {}
    }
    SysTick_Config(SysTicks);
}

 /*----------System timer interrup handler------------------*/
/**---------------------------------------------------------**/
void SysTick_Handler(void)
{
    static uint32_t del = 0;
    
    del++;

    if (del == (SysTicksClk/2)) // Per = 1s
    {
        del = 0;
        
#ifdef DEBUG
        if (GPIOB->ODR & GPIO_ODR_OD2)
        {
            GPIOB->BSRR |= GPIO_BSRR_BR2;
        }
        else
        {
            GPIOB->BSRR |= GPIO_BSRR_BS2;
        }
#else
        GPIOB->BSRR |= GPIO_BSRR_BS2;
#endif
        
    }
}

/**=========================================================**/
 /*----------------GPIO Init--------------------------------*/
/**---------------------------------------------------------**/
void Init_GPIO(void)
{
    RCC-> AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                     RCC_AHB1ENR_GPIOBEN |
                     RCC_AHB1ENR_GPIOCEN |
					 RCC_AHB1ENR_GPIODEN;
  
    //LEDs
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN2*2 | // LED_WORK
                    GPIO_MODE_OUTPUT_PP << PIN0*2 | // Green 1
                    GPIO_MODE_OUTPUT_PP << PIN1*2;  // Red   1
    GPIOC->MODER |= GPIO_MODE_OUTPUT_PP << PIN4*2 | // Green 2
                    GPIO_MODE_OUTPUT_PP << PIN5*2;  // Red   2
	
    //Amplifier ON/OFF
    GPIOB->MODER &= ~((uint32_t)0x3     << PIN6*2);
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN6*2;
    
    //BT ON/OFF
    GPIOB->MODER |= GPIO_MODE_OUTPUT_PP << PIN14*2;
    
    //Bulb Check ON/OFF
    GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << PIN8*2;
    
    //MUTE TDA
    GPIOC->MODER |= GPIO_MODE_OUTPUT_PP << PIN13*2;
    
    //Set GPIOA PIN0 as usart4 TX <===> MP3
    GPIOA->AFR[0] |= GPIO_AF8_UART4 << PIN0*4 |
                     GPIO_AF8_UART4 << PIN1*4;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN0*2 |
                    GPIO_PULLUP << PIN1*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN0*2 |
                    GPIO_MODE_AF_PP << PIN1*2;
#ifndef DEBUG_TFT
	//Set GPIOB PIN10 as usart3 TX & PIN11 as usart3 RX <===> TFT
    GPIOB->AFR[1] |= GPIO_AF7_USART3 << (PIN10*4-32) |
                     GPIO_AF7_USART3 << (PIN11*4-32);
    GPIOB->PUPDR |= GPIO_PULLUP << PIN10*2 |
					GPIO_PULLUP << PIN11*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN10*2 |
					  GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN10*2 |
					GPIO_MODE_AF_PP << PIN11*2;
#endif
             
    //Set GPIOC PIN12 as usart5 TX & GPIOD PIN2 as usart5 RX <===> BT
    GPIOC->AFR[1] |= GPIO_AF8_UART5 << (PIN12*4-32);
    GPIOD->AFR[0] |= GPIO_AF8_UART5 <<  PIN2*4;
    GPIOC->PUPDR |= GPIO_PULLUP << PIN12*2;
    GPIOD->PUPDR |= GPIO_PULLUP << PIN2*2;
    GPIOC->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN12*2;
    GPIOD->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN2*2;
    GPIOC->MODER |= GPIO_MODE_AF_PP << PIN12*2;
    GPIOD->MODER |= GPIO_MODE_AF_PP << PIN2*2;
		
    //GPIOB PIN8,9 I2C1 
    GPIOB->AFR[1] |= GPIO_AF4_I2C1 << (PIN8*4-32) |
                     GPIO_AF4_I2C1 << (PIN9*4-32);
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN8*2 |
                    GPIO_MODE_AF_PP << PIN9*2;
    GPIOB->PUPDR |= GPIO_PULLUP << PIN8*2 |
                    GPIO_PULLUP << PIN9*2;
    GPIOB->OTYPER |= GPIO_OTYPER_OT8 |
                     GPIO_OTYPER_OT9;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN8*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN9*2;    
    
    //Buttons IN
    GPIOA->MODER &= ~((uint32_t)0x3 << PIN15*2);
    GPIOA->PUPDR &= ~((uint32_t)0x3 << PIN15*2);
    GPIOB->MODER &= ~((uint32_t)0x3 << PIN4*2);
    GPIOB->PUPDR &= ~((uint32_t)0x3 << PIN4*2);

    GPIOA->MODER |= GPIO_MODE_INPUT << PIN10*2 | //BT1
                    GPIO_MODE_INPUT << PIN11*2 | //BT2
                    GPIO_MODE_INPUT << PIN12*2 | //BT3
                    GPIO_MODE_INPUT << PIN15*2;  //BT4
    GPIOC->MODER |= GPIO_MODE_INPUT << PIN10*2 | //BT5
                    GPIO_MODE_INPUT << PIN11*2;  //BT6
                    
	GPIOB->MODER |= GPIO_MODE_INPUT << PIN12*2 | //BT_CLK_DOWN
                    GPIO_MODE_INPUT << PIN13*2 | //BT_CLK_UP
                    GPIO_MODE_INPUT << PIN3*2 |  //ENC_B
                    GPIO_MODE_INPUT << PIN4*2 |  //ENC_A
                    GPIO_MODE_INPUT << PIN5*2;   //ENC_BT
                    
		
	//Set GPIOA PIN3,4 GPIOC PIN10 as analog input
    GPIOA->MODER |=	GPIO_MODE_ANALOG << PIN6*2 | //ADC3 - IN
                    GPIO_MODE_ANALOG << PIN7*2;  //ADC4 - prevmo
    GPIOA->OSPEEDR |=	GPIO_SPEED_FREQ_VERY_HIGH << PIN6*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN7*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN6*2 |
                    GPIO_NOPULL << PIN7*2;
                    
    //GPIOA PIN5 Pilse in
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN5*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN5*2;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN5*2;
    GPIOA->AFR[0] |= GPIO_AF1_TIM2 << PIN5*4;
    
}

/**=========================================================**/
 /*-----------------RTC Init--------------------------------*/
/**---------------------------------------------------------**/
void Init_RTC(void)
{
    if((RCC->BDCR & RCC_BDCR_RTCEN)!=RCC_BDCR_RTCEN)//Проверка работы часов, если не включены, то инициализировать
    {
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;//Включить тактирование PWR и Backup
        PWR->CR |= PWR_CR_DBP; //Разрешить доступ к Backup области
        
        RCC->BDCR |= RCC_BDCR_BDRST;//Сбросить Backup область
        RCC->BDCR &= ~RCC_BDCR_BDRST;
       
        RCC->BDCR |= RCC_BDCR_RTCEN | 
                     RCC_BDCR_RTCSEL_0;        //Выбрать LSE источник (кварц 32768) и подать тактирование
        
        RCC->BDCR |= RCC_BDCR_LSEON;//Включить LSE
        while ((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON){} //Дождаться включения
        PWR->CR &= ~PWR_CR_DBP;//запретить доступ к Backup области
    }
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    EXTI->PR |= EXTI_PR_PR17;
    EXTI->IMR |= EXTI_IMR_MR17;
    EXTI->EMR &= ~EXTI_EMR_MR17;
    EXTI->RTSR |= EXTI_RTSR_TR17;
    
    RTC->CR &= ~RTC_CR_ALRAE;
    while(!(RTC->ISR & RTC_ISR_ALRAWF)){};
    RTC->ALRMAR = RTC_ALRMAR_MSK1 |
                  RTC_ALRMAR_MSK2 |
                  RTC_ALRMAR_MSK3 |
                  RTC_ALRMAR_MSK4;
        
    RTC->CR |= RTC_CR_ALRAE |
               RTC_CR_ALRAIE;

    RTC->WPR = 0xFF;

}

 /*-----------RTC 1 second handler--------------------------*/
/**---------------------------------------------------------**/
void RTC_Alarm_IRQHandler(void)
{
#ifdef DEBUG
    LED4_ON;
#endif
    
    RTC->ISR &= ~RTC_ISR_ALRAF;
    EXTI->PR |= EXTI_PR_PR17;
    
    TFT_TIME[14] = ((RTC->TR & RTC_TR_MNU_Msk)>> RTC_TR_MNU_Pos)+ 0x30; // minute
    TFT_TIME[13] = ((RTC->TR & RTC_TR_MNT_Msk)>> RTC_TR_MNT_Pos)+ 0x30;
    
    TFT_TIME[11] = ((RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos) + 0x30; // hour
    TFT_TIME[10] = ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos) + 0x30;
    
    TFT_TIME[30] =  (RTC->DR & RTC_DR_DU_Msk)   				+ 0x30; // day
    TFT_TIME[29] = ((RTC->DR & RTC_DR_DT_Msk) >> RTC_DR_DT_Pos) + 0x30;
    
    TFT_TIME[33] = ((RTC->DR & RTC_DR_MU_Msk) >> RTC_DR_MU_Pos) + 0x30; // month
    TFT_TIME[32] = ((RTC->DR & RTC_DR_MT_Msk) >> RTC_DR_MT_Pos) + 0x30;
    
    TFT_TIME[36] = ((RTC->DR & RTC_DR_YU_Msk) >> RTC_DR_YU_Pos) + 0x30; // year
    TFT_TIME[35] = ((RTC->DR & RTC_DR_YT_Msk) >> RTC_DR_YT_Pos) + 0x30;
    
    memcpy(&TFT_TIME[50], &day_of_week[((RTC->DR & RTC_DR_WDU_Msk) >> RTC_DR_WDU_Pos) - 1],9);

    TFT_send(TFT_TIME, sizeof(TFT_TIME));
    
#ifdef DEBUG
    LED4_OFF;
#endif
}

/**=========================================================**/
 /*-----------------TFT Init--------------------------------*/
/**---------------------------------------------------------**/
void Init_TFT(void)
{
#ifdef DEBUG_BT

    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART3->BRR = APB1/921600;
    USART3->CR1 = USART_CR1_UE |
                  USART_CR1_TE |
                  USART_CR1_RE |
                  USART_CR1_RXNEIE;
   
    NVIC_EnableIRQ(USART3_IRQn);
#else
    //UART3 to TFT
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    USART3->BRR = APB1/115200;
    USART3->CR1 = USART_CR1_UE |
                  USART_CR1_TE;
    USART3->CR3 = USART_CR3_DMAT;
    
    //DMA1_Stream4 for UART4
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream3->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream3->PAR = (uint32_t) &USART3->DR;
    
#endif
}

 /*------------TFT send function----------------------------*/
/**---------------------------------------------------------**/
void TFT_send(uint8_t *buff, uint8_t size)
{
#ifdef DEBUG
    LED2_ON;
#endif
    while(DMA1_Stream3->NDTR != 0){};
    while(!(USART3->SR & USART_SR_TC)){};
        
    USART3->SR = 0;
        
    DMA1_Stream3->M0AR = (uint32_t) buff;
    DMA1_Stream3->NDTR = size;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
                  DMA_LIFCR_CHTIF3 | 
                  DMA_LIFCR_CFEIF3 |
                  DMA_LIFCR_CTEIF3;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
#ifdef DEBUG
    LED2_OFF;
#endif
}

/**=========================================================**/
 /*--------------BlueTooth Init-----------------------------*/
/**---------------------------------------------------------**/
void Init_BT(void)
{
#ifdef DEBUG_BT
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    UART5->BRR = APB1/921600;
    UART5->CR1 = USART_CR1_UE | 
                 USART_CR1_TE | 
                 USART_CR1_RE |
                 USART_CR1_RXNEIE;
    
    NVIC_EnableIRQ(UART5_IRQn);
    
    BT_ON;
    LED1_ON;
    
#else
    
    //MX_USART3_UART_Init();
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    UART5->BRR = APB1/921600;
    UART5->CR1 = USART_CR1_UE | 
                 USART_CR1_TE | 
                 USART_CR1_RE |
				 USART_CR1_IDLEIE;
    UART5->CR3 = USART_CR3_DMAR |
                 USART_CR3_DMAT;

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    DMA1_Stream0->CR = DMA_SxCR_MINC |
					   DMA_SxCR_TCIE |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream0->PAR = (uint32_t) &UART5->DR;
    DMA1_Stream0->M0AR = (uint32_t) bt_rx_buff;
    DMA1_Stream0->NDTR = BT_RX_BUFF_SIZE;
    DMA1_Stream0->CR |= DMA_SxCR_EN;    
    
    DMA1_Stream7->CR &= ~DMA_SxCR_EN;
    DMA1_Stream7->CR = DMA_SxCR_DIR_0 |
                       DMA_SxCR_MINC |
                       DMA_SxCR_CHSEL_2;
    DMA1_Stream7->PAR = (uint32_t) &UART5->DR;
    
	NVIC_EnableIRQ(UART5_IRQn);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    
#endif
}

 /*----Send command to BlueTooth function-------------------*/
/**---------------------------------------------------------**/
void BT_send(uint8_t query)
{
    DMA1->HIFCR = DMA_HIFCR_CTCIF7 |
                  DMA_HIFCR_CHTIF7 | 
                  DMA_HIFCR_CFEIF7 |
                  DMA_HIFCR_CTEIF7;	
        
    DMA1_Stream7->M0AR = (uint32_t) bt_tx_query[query];
    DMA1_Stream7->NDTR = 7;
    DMA1_Stream7->CR |= DMA_SxCR_EN;
}

 /*----------Recive information about BT UART5-------------*/
/**---------------------------------------------------------**/
void UART5_IRQHandler(void)
{
	if(UART5->SR & USART_SR_IDLE)
	{
		volatile uint32_t tmp;
		tmp = UART5->SR;
		tmp = UART5->DR;
		(void)tmp;
		DMA1_Stream0->CR &= ~DMA_SxCR_EN;
	}
}

 /*--------------BT UART5 DMA recive Handler----------------*/
/**---------------------------------------------------------**/
void DMA1_Stream0_IRQHandler(void)
{
	DMA1->LIFCR = DMA_LIFCR_CTCIF0 |
                  DMA_LIFCR_CHTIF0 | 
                  DMA_LIFCR_CFEIF0 |
                  DMA_LIFCR_CTEIF0 |
				  DMA_LIFCR_CDMEIF0;
	
	DMA1_Stream0->M0AR = (uint32_t) bt_rx_buff;
	DMA1_Stream0->NDTR = BT_RX_BUFF_SIZE;
	DMA1_Stream0->CR |= DMA_SxCR_EN;
    
    if((bt_rx_buff[0] == 13)&&(bt_rx_buff[1] == 10))
	{
        switch(bt_rx_buff[3])
        {
            case 'I':{
                        BT_Status = BT_NO_DEV;
                    break;}
            case 'U':{
                        if(bt_rx_buff[4] == '1')
                            BT_Status = BT_NO_DEV;
                        else if(((bt_rx_buff[4] == '3')||(bt_rx_buff[4] == '5'))&&(BT_Status == BT_NO_DEV))
                            BT_Status = BT_CONN;
                    break;}
            case 'P':{
                        if (BT_Status == BT_PLAY)
                            BT_Status = BT_PAUSE;
                    break;}
            case 'R':{
                        BT_Status = BT_PLAY;
                    break;}
        
        };
        if ((GLOBAL_STATE == GLOBAL_STATE_MAIN) && (AUDIO_INPUT == AUDIO_BT))
            TFT_send(main_BT_text[BT_Status],sizeof(main_BT_text[BT_Status]));
    }
}

/**=========================================================**/
 /*----------------USB<=>MP3 Player Init--------------------*/
/**---------------------------------------------------------**/
void Init_USB(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    
    UART4->BRR = APB1/9600;
    UART4->CR1 = USART_CR1_UE |
                 USART_CR1_TE |
				 USART_CR1_RE;
}

 /*--------USB<=>MP3 Player send command function-----------*/
/**---------------------------------------------------------**/
uint8_t USB_send(uint8_t CMD)
{
    uint8_t tmp;
    uint32_t delay = 0;
    USB_command[2] = CMD;
    
    if(UART4->SR & USART_SR_RXNE)
        tmp = UART4->DR;
    
    for(uint8_t i = 0;i<4;i++)
    {
        while(!(UART4->SR & USART_SR_TC));
        UART4->DR = USB_command[i];
    }
    
    switch(CMD)
    {
        case USB_Q_STATUS:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_status |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_COUNT:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.count |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_NUMBER:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.num |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_LONG:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.tlong |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_TIME:
        {
            for(uint8_t i = 4;i>0;i--)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.time |= ((tmp - ((tmp >= 0x61)? 0x57:0x30)) << ((i-1)*4));
            }
            break;
        }
        case USB_Q_TRACK_NAME:
        {
            for(uint8_t i = 0;i<11;i++)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 2000000)
                    return 1;
                };
                tmp = UART4->DR;
                usb_track_info.name[i] = tmp;
            }
            break;
        }
        case USB_CMD_PLAY:
        case USB_CMD_PAUSE:
        case USB_CMD_NEXT:
        case USB_CMD_PREV:
        {
            for(uint8_t i = 0;i<2;i++)
            {
                delay = 0;
                while(!(UART4->SR & USART_SR_RXNE))
                {
                delay++;
                if(delay == 1000000)
                    return 1;
                };
                uint8_t tmp = UART4->DR;
            }           
            break;
        }
    }
        
    for(uint32_t i = 0;i<5500000;i++){};
    return 0;
}

 /*--USB<=>MP3 Player send command  with parameter function-*/
/**---------------------------------------------------------**/
uint8_t USB_send_par(uint8_t CMD, uint8_t PAR)
{
    uint32_t delay = 0;
    uint8_t tmp;
    USB_command5[2] = CMD;
    USB_command5[3] = PAR;
    
    if(UART4->SR & USART_SR_RXNE)
        tmp = UART4->DR;
    
    for(uint8_t i = 0;i<5;i++)
    {
        while(!(UART4->SR & USART_SR_TXE));
        UART4->DR = USB_command5[i];
    }
    
    if(CMD == USB_CMD_SOURCE)
        for(uint8_t i = 0;i<5;i++)
        {
            delay = 0;
            while(!(UART4->SR & USART_SR_RXNE))
            {
            delay++;
            if(delay == 2000000)
                return 1;
            };
            tmp = UART4->DR;
            if(tmp == 'S')
                return 1;
        }
    else
        for(uint8_t i = 0;i<2;i++)
        {
            delay = 0;
            while(!(UART4->SR & USART_SR_RXNE))
            {
            delay++;
            if(delay == 2000000)
                return 1;
            };
            tmp = UART4->DR;
        }
    for(uint32_t i = 0;i<4500000;i++){};
    return 0;
}

/**=========================================================**/
 /*----------------I2C1 Init--------------------------------*/
/**---------------------------------------------------------**/
void Init_I2C1(void)
{
    //I2C1 Init
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR2 = 45 << I2C_CR2_FREQ_Pos;// |
    I2C1->CCR = APB1/200000;
    I2C1->TRISE = APB1/1000000+1;
    I2C1->CR1 = I2C_CR1_PE;
}

 /*---------I2C1 Send data function-------------------------*/
/**---------------------------------------------------------**/
uint8_t I2C1_Send(uint8_t addres,uint8_t *buff, uint16_t size)
{
    uint16_t i;
    
    I2C1->CR1 |= I2C_CR1_START;             // формирование сигнала старт
        
    while (!(I2C1->SR1 & I2C_SR1_SB))       // ждем окончания формирования сигнала "Старт"
    {
        if(I2C1->SR1 & I2C_SR1_BERR)
        return 1;
    }
    (void) I2C1->SR1;(void) I2C1->SR2;

    I2C1->DR = addres;                        // Передаем адресс
    
    while (!(I2C1->SR1 & I2C_SR1_ADDR))     // ожидаем окончания передачи адреса
    {
        if((I2C1->SR1 & I2C_SR1_BERR) || (I2C1->SR1 & I2C_SR1_AF))
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 2;
        }
    }
    (void) I2C1->SR1;(void) I2C1->SR2;

    for(i=0;i<size;i++)
    {
        I2C1->DR = buff[i];
        while (!(I2C1->SR1 & I2C_SR1_BTF))  // ожидаем окончания передачи
        {
            if((I2C1->SR1 & I2C_SR1_BERR) || (I2C1->SR1 & I2C_SR1_AF))
            {   
                I2C1->CR1 |= I2C_CR1_STOP;
                return 3;
            }
        }
        (void) I2C1->SR1;(void) I2C1->SR2;
    }
    
    I2C1->CR1 |= I2C_CR1_STOP;              // формирование сигнала "Стоп"
    return 0;
}

/**=========================================================**/
 /*-------FM<=>TEA Set Freq funqtion------------------------*/
/**---------------------------------------------------------**/
uint8_t TEA_set_freq(uint16_t freq)
{
    uint8_t tea[5];
    
	freq = (4*(freq*100000UL+225000UL))/32768;
    
	tea[0] = freq >> 8;
    tea[1] = freq & 0xff;
    
    tea[2] = 0x10;
    tea[3] = 0x12;
    tea[4] = 0;//high ingection
    return I2C1_Send(0xC0, tea,sizeof(tea));
}

/**=========================================================**/
 /*------------TDA Inti funqtion----------------------------*/
/**---------------------------------------------------------**/
uint8_t Init_TDA(void)
{
//	init_buff[1] = (STATE == MAIN) ? TDA_inputs[INPUT_SEL] : TDA_SOURCE_MUTE; // Main source = SE2(FM), gain = 0;
//    init_buff[2] = (TDA_loudness.atteniation & 0xF)|((TDA_loudness.center_freq << 4) & 0x30)| ((TDA_loudness.high_boost << 6) & 0x40); // Loudless
//    init_buff[3] = 0xC7; // CLK FM off, SM step 2.56, 0.96, I2C, off
//    init_buff[4] = VOLUME > 0 ? VOLUME : 16-VOLUME; // VOLUME
//    init_buff[5] = 0x80 | ((TDA_treble.center_freq << 5)&0x60) | (TDA_treble.atteniation > 0 ? (TDA_treble.atteniation | 0x10):(abs(TDA_treble.atteniation))); // Ref out ext + treble off;
//    init_buff[6] = ((TDA_middle.Q_factot << 5) & 0x60) | ((TDA_middle.atteniation > 0 ? (TDA_middle.atteniation | 0x10):(abs(TDA_middle.atteniation))) & 0x1F); // mid off
//    init_buff[7] = ((TDA_bass.Q_factot << 5) & 0x60) | ((TDA_bass.atteniation > 0 ? (TDA_bass.atteniation | 0x10):(abs(TDA_bass.atteniation))) & 0x1F); // bass off
//    init_buff[9] = ((TDA_bass.center_freq << 4) & 0x30) | ((TDA_middle.center_freq << 2) & 0xC); // sub off all
//    init_buff[11]= TDA_sp_att.left_front  > 0 ? TDA_sp_att.left_front  : 16-TDA_sp_att.left_front; //Lefr  Front
//    init_buff[12]= TDA_sp_att.right_front > 0 ? TDA_sp_att.right_front : 16-TDA_sp_att.right_front;//Right Front
//    init_buff[13]= TDA_sp_att.left_rear   > 0 ? TDA_sp_att.left_rear   : 16-TDA_sp_att.left_rear;  //Lefr  Rear
//    init_buff[14]= TDA_sp_att.right_rear  > 0 ? TDA_sp_att.right_rear  : 16-TDA_sp_att.right_rear; //Right Rear

    uint8_t I2C_buff[2], res;
    I2C_buff[0] = TDA_MAIN_SOURCE;      //Addr  0: Main Selector
    I2C_buff[1] = (GLOBAL_STATE == GLOBAL_STATE_MAIN) ? TDA_inputs[AUDIO_INPUT] : TDA_SOURCE_MUTE;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = TDA_SOFT_MUTE;        //Addr  4: Soft mute - OFF
    I2C_buff[1] = 0x1D;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 7;            //Addr  7: Loudness
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = TDA_VOLUME;            //Addr  8: Volume/Output gain
    I2C_buff[1] = VOLUME > 0 ? VOLUME : 32-VOLUME;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 9;            //Addr  9: Treble filter
    I2C_buff[1] = 0x0F;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 10;           //Addr 10: Middle filter
    I2C_buff[1] = 0x0F;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 11;           //Addr 11: Bass filter
    I2C_buff[1] = 0x0F;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 12;           //Addr 12: Subwoofer/middle/bass
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 13;           //Addr 13: Speaker attenuation Front Lefr
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 14;           //Addr 14: Speaker attenuation Front Right
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 15;           //Addr 15: Speaker attenuation Rear Lefr
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 16;           //Addr 16: Speaker attenuation Rear Right
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 17;           //Addr 17: Speaker attenuation SW Lefr
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 18;           //Addr 18: Speaker attenuation SW Right
    I2C_buff[1] = 0x00;
    res |= I2C1_Send(TDA7718_ADDRESS, I2C_buff, sizeof(I2C_buff));
   
    return res;
}

/**=========================================================**/
 /*-------------FLASH functions-----------------------------*/
 /*----------------FLASH read-------------------------------*/
/**---------------------------------------------------------**/
uint32_t flash_read(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}
 /*----------------FLASH ready-------------------------------*/
/**---------------------------------------------------------**/
//Функция возврщает true когда можно стирать или писать память.
uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

 /*------------FLASH earse sector---------------------------*/
/**---------------------------------------------------------**/
//Функция стирает одну страницу. В качестве адреса можно использовать любой
//принадлежащий диапазону адресов той странице которую нужно очистить.
void flash_erase_sector(uint8_t sector) 
{
		FLASH->CR |= FLASH_CR_SER; //Устанавливаем бит стирания одной страницы
		FLASH->CR |= sector << FLASH_CR_SNB_Pos; // Задаем её адрес
		FLASH->CR |= FLASH_CR_STRT; // Запускаем стирание
		
		while(!flash_ready())//Ждем пока страница сотрется.
		
		FLASH->CR&= ~(FLASH_CR_SER); //Сбрасываем бит обратно
}

 /*----------------FLASH write-------------------------------*/
/**---------------------------------------------------------**/
void flash_write(uint32_t address, uint32_t data)
{
		FLASH->CR |= FLASH_CR_PG; //Разрешаем программирование флеша
		
		FLASH->CR |= FLASH_CR_PSIZE_1;
		
		while(!flash_ready()); //Ожидаем готовности флеша к записи
		
		*(__IO uint32_t*)address = (uint32_t)data; //Пишем младшие 2 бата
		
		while(!flash_ready());
		
		FLASH->CR &= ~(FLASH_CR_PG); //Запрещаем программирование флеша
}

 /*-----------FLASH write new data--------------------------*/
/**---------------------------------------------------------**/
void flash_write_newdata(void)
{
    uint32_t tmp_mem;
    
	flash_erase_sector(3); //start from 0x0800C000
	if(GLOBAL_STATE == GLOBAL_STATE_TDA_SETT)
        tmp_mem = (GLOBAL_STATE_MAIN << 24 )   | (AUDIO_INPUT << 16) | (0xFF00 & (VOLUME << 8 ));
    else
        tmp_mem = (GLOBAL_STATE << 24 )  | (AUDIO_INPUT << 16) | (0xFF00 & (VOLUME << 8 ));
    
    flash_write(MEM_ADDRESS, tmp_mem);
    
    tmp_mem = RADIO_FREQ << 16;
	flash_write(RADIO_FREQ_ADR, tmp_mem); //Radio Freq
    
//    tmp_mem = (TDA_loudness.high_boost << 24) | (TDA_loudness.center_freq << 16) | (0xFF00 & (TDA_loudness.atteniation << 8));
//    flash_write(TDA_MAIN_LOUD_ADR, tmp_mem); //Main loud
//    
//    tmp_mem = (TDA_treble.center_freq  << 24) | (0xFF0000 & (TDA_treble.atteniation   << 16));
//    flash_write(TDA_TREB_ADR, tmp_mem); //Treble
//    
//    tmp_mem = (TDA_middle.Q_factot     << 24) | (TDA_middle.center_freq   << 16) | (0xFF00 & (TDA_middle.atteniation   << 8));
//    flash_write(TDA_MIDD_ADR, tmp_mem); //Middle
//    
//    tmp_mem = (TDA_bass.Q_factot       << 24) | (TDA_bass.center_freq     << 16) | (0xFF00 & (TDA_bass.atteniation     << 8));
//    flash_write(TDA_BASS_ADR, tmp_mem); //Bass
//    
//    tmp_mem = (0xFF000000 & (TDA_sp_att.left_front   << 24)) | (0xFF0000 & (TDA_sp_att.right_front   << 16)) | (0xFF00 & (TDA_sp_att.left_rear     << 8)) | (TDA_sp_att.right_rear & 0xFF);
//    flash_write(TDA_SATT_ADR, tmp_mem); //Speaker attenuation
}

/**=========================================================**/
 /*----------------ADC Init---------------------------------*/
/**---------------------------------------------------------**/
void Init_ADC(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    ADC1->CR2 = ADC_CR2_ADON | 
                ADC_CR2_DMA |
                ADC_CR2_DDS |
                ADC_CR2_EXTSEL_3 | // Timer 3 TRGO event
                ADC_CR2_EXTEN_0;
    ADC1->CR1 = ADC_CR1_SCAN;
    ADC1->SMPR2 = ADC_SMPR2_SMP6_1 | ADC_SMPR2_SMP6_2 | // clock num for ADC3
                  ADC_SMPR2_SMP7_1 | ADC_SMPR2_SMP7_2;  // clock num for ADC4
    ADC1->SQR1 = (ADC_BUF_NUM-1) << ADC_SQR1_L_Pos; // 2 conversation
    ADC1->SQR3 = 6 << ADC_SQR3_SQ1_Pos |
                 7 << ADC_SQR3_SQ2_Pos;	
    //DMA Init
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    DMA2_Stream0->CR |= DMA_SxCR_CIRC |
                        DMA_SxCR_MINC | 
                        DMA_SxCR_MSIZE_0 | 
                        DMA_SxCR_PSIZE_0 |
						DMA_SxCR_TCIE;
    DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t) ADC_Buff;
    DMA2_Stream0->NDTR = ADC_BUF_NUM;
    
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2->LIFCR |= DMA_LIFCR_CFEIF0 |
                   DMA_LIFCR_CDMEIF0 |
                   DMA_LIFCR_CTEIF0 |
                   DMA_LIFCR_CHTIF0 |
                   DMA_LIFCR_CTCIF0;
    //Timer 3s Init
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;//Timer for ADC
    TIM3->PSC = APB1_TIM/10000-1;
    TIM3->ARR = 5000;
    TIM3->CR2 |= TIM_CR2_MMS_1;
    TIM3->CR1 |= TIM_CR1_CEN;
}

 /*----------ADC interrupt handler--------------------------*/
/**---------------------------------------------------------**/
void DMA2_Stream0_IRQHandler(void)
{
#ifdef DEBUG
    LED3_ON;
#endif
    DMA2->LIFCR |= DMA_LIFCR_CFEIF0 |
                   DMA_LIFCR_CDMEIF0 |
                   DMA_LIFCR_CTEIF0 |
                   DMA_LIFCR_CHTIF0 |
                   DMA_LIFCR_CTCIF0;
	uint8_t V_IN = (ADC_Buff[0]*182)/0xFFF; //182 = 3.64(Vref) * 5(devider on pcb) * 10(for calculations)
	uint8_t P_IN = (ADC_Buff[1]*182)/0xFFF; //182 = 3.64(Vref) * 5(devider on pcb) * 10(for calculations)
    
    ADC_text[9]  =  V_IN/100 + 0x30;
    ADC_text[10] = (V_IN -(V_IN/100)*100)/10 + 0x30;
    ADC_text[12] = (V_IN -(V_IN/10)*10) + 0x30;
    
    ADC_text[17] =  P_IN/100 + 0x30;
    ADC_text[18] = (P_IN -(P_IN/100)*100)/10 + 0x30;
    ADC_text[19] = (P_IN -(P_IN/10)*10) + 0x30;
    
    TFT_send(ADC_text, sizeof(ADC_text));
#ifdef DEBUG
    LED3_OFF;
#endif
}

/**=========================================================**/
 /*----------------Pulse IN---------------------------------*/
/**---------------------------------------------------------**/
void Init_Pulse_IN(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | 
                    RCC_APB1ENR_TIM4EN;
    
    TIM4->PSC = APB1_TIM/10000 - 1;
    TIM4->ARR = 10000 - 1;
    TIM4->CR1 = TIM_CR1_DIR |
                TIM_CR1_OPM;
    TIM4->CR2 = TIM_CR2_MMS_0;
    TIM4->DIER = TIM_DIER_UIE;
    
    TIM2->PSC = 0;
    TIM2->CR2 = TIM_CR2_MMS_1;
    TIM2->SMCR |= TIM_SMCR_ECE | 
                  TIM_SMCR_TS_0 |
                  TIM_SMCR_TS_1 |
                  TIM_SMCR_SMS_0 |
                  TIM_SMCR_SMS_2;
                  
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_CEN;
    
}

/*-------Pulse IN interrupt handler-------------------------*/
/**---------------------------------------------------------**/
void TIM4_IRQHandler(void)
{
    TIM4->SR &= ~TIM_SR_UIF;
    
    RawFreq = (uint32_t)TIM2->CNT;
    Flag = 1;
    TIM2->CNT = 0;
    
    TIM4->CR1 |= TIM_CR1_CEN;
}

/**=========================================================**/
 /*-----------Init Timer for key----------------------------*/
/**---------------------------------------------------------**/
void Init_KEYs_TIM(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = APB1_TIM/10000-1;
    TIM14->ARR = 10000/5-1;
    TIM14->DIER = TIM_DIER_UIE;
    TIM14->CR1 = TIM_CR1_CEN; 
}

/*-------Key Timer interrupt handler------------------------*/
/**---------------------------------------------------------**/
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    TIM14->SR = 0;
    
    static uint8_t delay_send = 98;
    
    uint8_t i2c_buff[2];
    
    delay_send++;
    
/*--++--Source button/OFF Audio--++--*/
    if(BT_SOUCE) 
    {
        OFF_counter++;
        if((OFF_counter >= 20)&&(GLOBAL_STATE == GLOBAL_STATE_MAIN))
        {
            OFF_counter = 20;
            BT_OFF;
            AMP_OFF;
            MUTE;
            
            i2c_buff[0] = TDA_MAIN_SOURCE;
            i2c_buff[1] = TDA_SOURCE_MUTE;
            I2C_res = I2C1_Send(TDA7718_ADDRESS, i2c_buff, sizeof(i2c_buff));
            
            GLOBAL_STATE = GLOBAL_STATE_AUDIO_OFF;
						
			flash_write_newdata();
            
            TFT_send(pages[GLOBAL_STATE], sizeof(pages[GLOBAL_STATE]));
			delay_send = 98;

            USB_send(USB_CMD_PAUSE);
        }
    }
    else
    {
        if((OFF_counter < 20)&&(OFF_counter != 0))
        {
            if(GLOBAL_STATE == GLOBAL_STATE_MAIN)
            {
                AUDIO_INPUT++;
                if(AUDIO_INPUT>AUDIO_AUX) AUDIO_INPUT = AUDIO_FM;
            }
            if(GLOBAL_STATE == GLOBAL_STATE_AUDIO_OFF)
            {
                GLOBAL_STATE = GLOBAL_STATE_MAIN;
            }
            if((GLOBAL_STATE == GLOBAL_STATE_AUDIO_OFF) || (GLOBAL_STATE == GLOBAL_STATE_MAIN))
            {
				flash_write_newdata();
            
                TFT_send(pages[GLOBAL_STATE], sizeof(pages[GLOBAL_STATE]));

                i2c_buff[0] = TDA_MAIN_SOURCE;
                i2c_buff[1] = TDA_inputs[AUDIO_INPUT];
                I2C_res = I2C1_Send(TDA7718_ADDRESS, i2c_buff, sizeof(i2c_buff));
                    
                i2c_buff[0] = TDA_VOLUME;
                i2c_buff[1] = VOLUME > 0 ? VOLUME : 32-VOLUME;
                I2C_res = I2C1_Send(TDA7718_ADDRESS, i2c_buff, sizeof(i2c_buff));
                
                UNMUTE;
                
                TFT_send(input_tft[AUDIO_INPUT], sizeof(input_tft[AUDIO_INPUT]));
                
                TFT_switch_audio_input();
                
                TFT_send_vol();
                AMP_ON;
                delay_send = 98;
            }
        }
        OFF_counter = 0;
    }
    
/*--++--Volume UP button--++--*/
    if(BT_VOL_UP)
    {
        switch(GLOBAL_STATE)
        {
            case GLOBAL_STATE_MAIN:
            {
                break;
            }
            case GLOBAL_STATE_SET_TIME:
            {
                break;
            }
            case GLOBAL_STATE_TDA_SETT:
            {
                break;
            }
        }
    }
    
    if(((GLOBAL_STATE == GLOBAL_STATE_MAIN)||(GLOBAL_STATE == GLOBAL_STATE_AUDIO_OFF))&&(delay_send >= 100))
    {
		delay_send = 0;
        
        BT_send(BT_STATUS);
        
        TFT_send(TFT_TIME, sizeof(TFT_TIME));
        if((AUDIO_INPUT == AUDIO_USB) && (USB_res == 0))
        {
            USB_send(USB_Q_STATUS);
            USB_send(USB_Q_TRACK_NUMBER);
            USB_send(USB_Q_TRACK_NAME);
            
            main_USB_text[10] =  usb_track_info.num/100 + 0x30;
            main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
            main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
        
            main_USB_text[18] = usb_track_info.name[0];
            main_USB_text[19] = usb_track_info.name[1];
            main_USB_text[20] = usb_track_info.name[2];
            main_USB_text[21] = usb_track_info.name[3];
            main_USB_text[22] = usb_track_info.name[4];
            main_USB_text[23] = usb_track_info.name[5];
            
            if(usb_status == 1)
            {
                main_USB_text[30] = 'P';
                main_USB_text[31] = 'L';
                main_USB_text[32] = 'A';
                main_USB_text[33] = 'Y';
                main_USB_text[34] = ' ';
            }else if(usb_status == 2)
            {
                main_USB_text[30] = 'P';
                main_USB_text[31] = 'A';
                main_USB_text[32] = 'U';
                main_USB_text[33] = 'S';
                main_USB_text[34] = 'E';
            }
            
            TFT_send(main_USB_text, sizeof(main_USB_text));            
        }
    }
}

/*--------Send to TFT AUDIO IN info-------------------------*/
/**---------------------------------------------------------**/
void TFT_switch_audio_input(void)
{
    switch (AUDIO_INPUT)
    {
        case AUDIO_FM:
        {
            TFT_send(main_text_font_7, sizeof(main_text_font_7));
        
            main_FM_text[10] =  RADIO_FREQ/1000 + 0x30;
            main_FM_text[11] = (RADIO_FREQ -(RADIO_FREQ/1000)*1000)/100 + 0x30;
            main_FM_text[12] = (RADIO_FREQ -(RADIO_FREQ/100)*100)/10 + 0x30;
            main_FM_text[14] = (RADIO_FREQ -(RADIO_FREQ/10)*10) + 0x30;
            
            TFT_send(main_FM_text, sizeof(main_FM_text));
            break;
        }
          
        case AUDIO_BT:
        {
            TFT_send(main_text_font_7, sizeof(main_text_font_7));
            
            BT_ON;
        
            BT_send(BT_STATUS);
            break;
        }
        
        case AUDIO_USB:
        {
            BT_OFF;
            BT_Status = BT_NO_DEV;
            
            USB_res = USB_send_par(USB_CMD_SOURCE, 0x00);
            if(USB_res == 1)
            {
                TFT_send(main_NO_USB_text, sizeof(main_NO_USB_text));
            }
            else
            {
                TFT_send(main_text_font_5, sizeof(main_text_font_5));
                
                USB_send_par(USB_CMD_VOL, 0x1e);
                USB_send_par(USB_CMD_PLAY_MODE, 0x00);  
                USB_send(USB_Q_TRACK_COUNT);
                USB_send(USB_Q_TRACK_NUMBER);
                USB_send(USB_Q_TRACK_NAME);
            
                main_USB_text[14] =  usb_track_info.count/100 + 0x30;
                main_USB_text[15] = (usb_track_info.count -(usb_track_info.count/100)*100)/10 + 0x30;
                main_USB_text[16] = (usb_track_info.count -(usb_track_info.count/10)*10) + 0x30;
            
                main_USB_text[10] =  usb_track_info.num/100 + 0x30;
                main_USB_text[11] = (usb_track_info.num -(usb_track_info.num/100)*100)/10 + 0x30;
                main_USB_text[12] = (usb_track_info.num -(usb_track_info.num/10)*10) + 0x30;
/*                    
                main_USB_text[35] =  (usb_track_info.tlong/60)/10 + 0x30;
                main_USB_text[36] = ((usb_track_info.tlong/60) -((usb_track_info.tlong/60)/10)*10) + 0x30;
                main_USB_text[38] =  (usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10 + 0x30;
                main_USB_text[39] = ((usb_track_info.tlong-((usb_track_info.tlong/60)*60)) -((usb_track_info.tlong-((usb_track_info.tlong/60)*60))/10)*10) + 0x30;
            
                main_USB_text[25] =  (usb_track_info.time/60)/10 + 0x30;
                main_USB_text[26] = ((usb_track_info.time/60) -((usb_track_info.time/60)/10)*10) + 0x30;
                main_USB_text[28] =  (usb_track_info.time-((usb_track_info.time/60)*60))/10 + 0x30;
                main_USB_text[29] = ((usb_track_info.time-((usb_track_info.time/60)*60)) -((usb_track_info.time-((usb_track_info.time/60)*60))/10)*10) + 0x30;
*/                        
                main_USB_text[18] = usb_track_info.name[0];
                main_USB_text[19] = usb_track_info.name[1];
                main_USB_text[20] = usb_track_info.name[2];
                main_USB_text[21] = usb_track_info.name[3];
                main_USB_text[22] = usb_track_info.name[4];
                main_USB_text[23] = usb_track_info.name[5];
                
                main_USB_text[30] = 'P';
                main_USB_text[31] = 'A';
                main_USB_text[32] = 'U';
                main_USB_text[33] = 'S';
                main_USB_text[34] = 'E';
            
                TFT_send(main_USB_text, sizeof(main_USB_text));
            }
            break;
        }
        case AUDIO_AUX:
        {
            TFT_send(main_text_font_7, sizeof(main_text_font_7));
        
            TFT_send(main_AUX_text, sizeof(main_AUX_text));
            break;
        }
    }
}

void TFT_send_vol(void)
{
    main_VOL_text[9] =   VOLUME > 0 ? '+' : '-';
    main_VOL_text[10] = (VOLUME > 0 ? VOLUME/10 : -VOLUME/10) + 0x30;
    main_VOL_text[11] = (VOLUME > 0 ? (VOLUME-(VOLUME/10)*10) : (-VOLUME-(-VOLUME/10)*10)) + 0x30;
    TFT_send(main_VOL_text, sizeof(main_VOL_text));     
}
