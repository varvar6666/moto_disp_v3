#include "main.h"

#define DEBUG 1
#define DEBUG_BT 1

#if DEBUG_BT

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




int main(void)
{
    Init_RCC();

    Init_GPIO();
    
    Init_TFT();



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
        /* ???????? ????? ??????????? FLASH */
        FLASH->ACR |= FLASH_ACR_PRFTEN;

        /* ????????????? Flash ?? 2 ????? ???????? */
    	/* ??? ????? ??????, ??? Flash ?? ????? ???????? ?? ??????? ??????? */        
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
        
#if DEBUG
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

void Init_TFT(void)
{
#if DEBUG_BT

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

void Init_BT(void)
{
#if DEBUG_BT
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

