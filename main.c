#include "main.h"

#define DEBUG //RELEASE
//#define DEBUG_BT

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






uint8_t BT_Status = BT_NO_DEV;
uint8_t bt_rx_buff[BT_RX_BUFF_SIZE];

uint8_t I2C_res, USB_res;

int main(void)
{
    Init_RCC();

    Init_GPIO();
    
    AMP_OFF;
    BT_ON;
    BULB_CH_ON;

    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    
    Init_TFT();
    TFT_send(TFT_reset,sizeof(TFT_reset)); //RESET TFT
    
    for(uint32_t delay = 0;delay < 10000000;delay++){}; // delay for TFT start

    loading_txt[8] = '1';
    loading_txt[9] = '4';
    TFT_send(loading_txt, sizeof(loading_txt));
        
    Init_RTC();

    Init_I2C1(); 

    uint8_t I2C_buff[2] = {0x00, 0x00};
    
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 4;
    I2C_buff[1] = 0x1F;
    I2C_res= I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 7;
    I2C_buff[1] = 0x1F;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 8;
    I2C_buff[1] = 0x40;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 9;
    I2C_buff[1] = 0x0F;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 10;
    I2C_buff[1] = 0x10;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 11;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 12;
    I2C_buff[1] = 0x2D;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 13;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 14;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 15;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));

    I2C_buff[0] = 16;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 17;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));
    
    I2C_buff[0] = 18;
    I2C_buff[1] = 0x00;
    I2C_res = I2C1_Send(TDA7718_Adr, I2C_buff, sizeof(I2C_buff));


    //I2C_res = TEA_set_freq(1028); // Init FM
    Init_BT();
    
    
    
    
    
    while (1)
    {
    
    }
}

void Init_RCC(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
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
    
    //Set GPIOA PIN0 as usart4 TX <===> MP3
    GPIOA->AFR[0] |= GPIO_AF8_UART4 << PIN0*4 |
                     GPIO_AF8_UART4 << PIN1*4;
    GPIOA->PUPDR |= GPIO_PULLUP << PIN0*2 |
                    GPIO_PULLUP << PIN1*2;
    GPIOA->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN0*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOA->MODER |= GPIO_MODE_AF_PP << PIN0*2 |
                    GPIO_MODE_AF_PP << PIN1*2;
    
	//Set GPIOB PIN10 as usart3 TX & PIN11 as usart3 RX <===> TFT
    GPIOB->AFR[1] |= GPIO_AF7_USART3 << (PIN10*4-32) |
                     GPIO_AF7_USART3 << (PIN11*4-32);
    GPIOB->PUPDR |= GPIO_PULLUP << PIN10*2 |
					GPIO_PULLUP << PIN11*2;
    GPIOB->OSPEEDR |= GPIO_SPEED_FREQ_VERY_HIGH << PIN10*2 |
					  GPIO_SPEED_FREQ_VERY_HIGH << PIN1*2;
    GPIOB->MODER |= GPIO_MODE_AF_PP << PIN10*2 |
					GPIO_MODE_AF_PP << PIN11*2;
                    
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
    GPIOA->MODER |=	GPIO_MODE_ANALOG << PIN3*2 | //ADC3 - IN
                    GPIO_MODE_ANALOG << PIN4*2;  //ADC4 - prevmo
    GPIOA->OSPEEDR |=	GPIO_SPEED_FREQ_VERY_HIGH << PIN3*2 |
                      GPIO_SPEED_FREQ_VERY_HIGH << PIN4*2;
    GPIOA->PUPDR |= GPIO_NOPULL << PIN3*2 |
                    GPIO_NOPULL << PIN4*2;
    
}



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
}

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

void TFT_send(uint8_t *buff, uint8_t size)
{
    LED2_ON;
    while(DMA1_Stream3->NDTR != 0){};
        
    DMA1_Stream3->M0AR = (uint32_t) buff;
    DMA1_Stream3->NDTR = size;
    DMA1->LIFCR = DMA_LIFCR_CTCIF3 |
                  DMA_LIFCR_CHTIF3 | 
                  DMA_LIFCR_CFEIF3 |
                  DMA_LIFCR_CTEIF3;
    DMA1_Stream3->CR |= DMA_SxCR_EN;
      
    LED2_OFF;
}

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
        
    BT_ON;
    
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	NVIC_EnableIRQ(UART5_IRQn);
    
#endif
}

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
//        if ((STATE == MAIN) && (INPUT_SEL == BT))
//            TFT_send(main_BT_text[BT_Status],sizeof(main_BT_text[BT_Status]));
    }
}
void Init_I2C1(void)
{
    //I2C1 Init
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR2 = 45 << I2C_CR2_FREQ_Pos;// |
    I2C1->CCR = APB1/200000;
    I2C1->TRISE = APB1/1000000+1;
    I2C1->CR1 = I2C_CR1_PE;
}

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
