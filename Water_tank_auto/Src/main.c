#include <stdint.h>

// ---------------- RCC ----------------
#define RCC_APB2ENR   (*(volatile uint32_t*)0x40021018)
#define RCC_APB1ENR   (*(volatile uint32_t*)0x4002101C)

// ---------------- GPIOA --------------
#define GPIOA_CRL     (*(volatile uint32_t*)0x40010800)
#define GPIOA_CRH     (*(volatile uint32_t*)0x40010804)
#define GPIOA_IDR     (*(volatile uint32_t*)0x40010808)
#define GPIOA_ODR     (*(volatile uint32_t*)0x4001080C)

// ---------------- AFIO ----------------
#define AFIO_EXTICR1  (*(volatile uint32_t*)0x40010008)

// ---------------- EXTI ----------------
#define EXTI_IMR      (*(volatile uint32_t*)0x40010400)
#define EXTI_RTSR     (*(volatile uint32_t*)0x40010408)
#define EXTI_PR       (*(volatile uint32_t*)0x40010414)

// ---------------- TIM1 ----------------
#define TIM1_CR1  (*(volatile uint32_t*)0x40012C00)
#define TIM1_SR   (*(volatile uint32_t*)0x40012C10)
#define TIM1_CNT  (*(volatile uint32_t*)0x40012C24)
#define TIM1_PSC  (*(volatile uint32_t*)0x40012C28)
#define TIM1_ARR  (*(volatile uint32_t*)0x40012C2C)

// ---------------- TIM2 ----------------
#define TIM2_CR1      (*(volatile uint32_t*)0x40000000)
#define TIM2_CNT      (*(volatile uint32_t*)0x40000024)
#define TIM2_PSC      (*(volatile uint32_t*)0x40000028)
#define TIM2_ARR      (*(volatile uint32_t*)0x4000002C)

// ---------------- NVIC ----------------
#define NVIC_ISER0    (*(volatile uint32_t*)0xE000E100)

// ---------------- Pin Defines ---------
#define TRIG_PIN      0
#define ECHO_PIN      1
#define RELAY_PIN     2
#define ALERT_PIN     3
#define MANUAL_BTN    5
#define MODE_PIN      10

volatile uint8_t motor_state = 0;
volatile uint32_t echo_time = 0;

// ================= TIM1 INIT =================
void TIM1_Init(void) {
    RCC_APB2ENR |= (1 << 11);   // Enable TIM1 clock

    TIM1_CR1 = 0;
    TIM1_PSC = 8 - 1;          // 1 MHz tick = 1us
    TIM1_ARR = 0xFFFF;
    TIM1_CNT = 0;
}

// ================= Delay =================
void delay_us(uint32_t us) {
    TIM1_ARR = us;
    TIM1_CNT = 0;
    TIM1_SR &= ~1;
    TIM1_CR1 |= 1;

    while(!(TIM1_SR & 1));
}

void delay_ms(uint32_t ms) {
    while(ms--) delay_us(1000);
}

// ================= GPIO Init =============
void GPIO_Init(void){
    RCC_APB2ENR |= (1<<2) | (1<<0);  // GPIOA + AFIO clock

    // TRIG PA0 output
    GPIOA_CRL &= ~(0xF << (0*4));
    GPIOA_CRL |=  (0x3 << (0*4));

    // ECHO PA1 floating input
    GPIOA_CRL &= ~(0xF << (1*4));
    GPIOA_CRL |=  (0x4 << (1*4));

    // RELAY PA2 output
    GPIOA_CRL &= ~(0xF << (2*4));
    GPIOA_CRL |=  (0x3 << (2*4));

    // ALERT PA3 output
    GPIOA_CRL &= ~(0xF << (3*4));
    GPIOA_CRL |=  (0x3 << (3*4));

    // MANUAL BTN PA5 pull-down
    GPIOA_CRL &= ~(0xF << (5*4));
    GPIOA_CRL |=  (0x8 << (5*4));
    GPIOA_ODR &= ~(1<<5);

    // MODE PA10 pull-down
    GPIOA_CRH &= ~(0xF << ((10-8)*4));
    GPIOA_CRH |=  (0x8 << ((10-8)*4));
    GPIOA_ODR &= ~(1<<10);

    GPIOA_ODR &= ~((1<<TRIG_PIN)|(1<<RELAY_PIN)|(1<<ALERT_PIN));
}

// ============== TIM2 Init ================
void TIM2_Init(void){
    RCC_APB1ENR |= (1<<0);
    TIM2_PSC = 8 - 1;       // 1 MHz tick
    TIM2_ARR = 0xFFFF;
    TIM2_CR1 = 1;
}

// =========== EXTI3 Interrupt Init =========
void EXTI_Alert_Init(void){
    AFIO_EXTICR1 &= ~(0xF << 12);  // EXTI3 = PA3
    AFIO_EXTICR1 |=  (0x0 << 12);

    EXTI_IMR  |= (1<<3);   // Enable EXTI3
    EXTI_RTSR |= (1<<3);   // Rising edge

    NVIC_ISER0 |= (1<<9);  // IRQ9 enable
}

// =============== Distance Function ==============
float get_distance(void){
    GPIOA_ODR &= ~(1<<0); delay_us(2);
    GPIOA_ODR |=  (1<<0); delay_us(10);
    GPIOA_ODR &= ~(1<<0);

    while(!(GPIOA_IDR & (1<<1)));

    TIM2_CNT = 0;
    while(GPIOA_IDR & (1<<1));

    echo_time = TIM2_CNT;

    return (echo_time * 0.0343f) / 2;
}

// =============== Motor control =================
void motor_on(void){
    GPIOA_ODR &= ~(1<<RELAY_PIN);  // Relay ON (active low)
    motor_state = 1;
}

void motor_off(void){
    GPIOA_ODR |= (1<<RELAY_PIN);
    motor_state = 0;
}

// ========== Pulse to Trigger EXTI ============
void trigger_distance_interrupt(void){
    GPIOA_ODR |=  (1<<ALERT_PIN);
    delay_us(1);
    GPIOA_ODR &= ~(1<<ALERT_PIN);
}

// =========== EXTI IRQ Handler =================
void EXTI3_IRQHandler(void){
    if(EXTI_PR & (1<<3)){
        motor_off();
        EXTI_PR |= (1<<3);
    }
}

// ================== MAIN ======================
int main(void){
    GPIO_Init();
    TIM1_Init();
    TIM2_Init();
    EXTI_Alert_Init();

    motor_off(); // safe state

    while(1){
        uint8_t mode = (GPIOA_IDR & (1<<MODE_PIN)) ? 1 : 0;

        if(mode == 0){
            float d = get_distance();

            if(d < 10 && motor_state){
                trigger_distance_interrupt();
            }
            else if(d > 20 && !motor_state){
                motor_on();
            }
        }
        else {
            if(GPIOA_IDR & (1<<MANUAL_BTN))
                motor_on();
            else
                motor_off();
        }

        delay_ms(100);
    }
}
