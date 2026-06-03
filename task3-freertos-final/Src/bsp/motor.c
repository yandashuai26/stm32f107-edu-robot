#include "stm32f10x.h"                  // Device header
#include "motor.h"
#include "FreeRTOS_demo.h"

#define MOTOR_MIN_HZ       500           // ������Ƶ��
#define MOTOR_MAX_HZ       3000          // ������Ƶ��
#define MOTOR_RAMP_STEP    1            // ÿ�μӼ��ٵĲ���ֵ(Hz)

static motor_t g_motor;                  // ���ȫ��״̬

// ���Ŀ�굽���ź�����ISR��give��vTaskMotorReached��take��
SemaphoreHandle_t MotorTargetReachedSemaphore;

// ����Ƶ�ʴ�С , ���� MOTOR_MIN_HZ ��ISR�л�ͣ��
static uint32_t clamp_hz(uint32_t hz)
{
    if (hz < MOTOR_MIN_HZ) return MOTOR_MIN_HZ;
    if (hz > MOTOR_MAX_HZ) return MOTOR_MAX_HZ;
    return hz;
}

// ͻȻ�ı��ٶȴ�С , �ⲿ�޷�����
// ֱ������TIM5��PWMƵ��, ռ�ձȹ̶�50%
// ARR = 100000/hz - 1, CCR = ARR/2
static void set_freq(uint32_t hz)
{
    uint32_t arr_add_1 = 100000 / hz;                  // ARR+1 = ��ʱ��ʱ��/hz = 100k/hz
    TIM_SetAutoreload(TIM5, arr_add_1 - 1);            // �����Զ���װ��ֵ
    TIM_SetCompare1(TIM5, arr_add_1 / 2);              // ���ñȽ�ֵ, ռ�ձ�50%
}


/************************************************************

// �����ʼ��  PA0-->PUL   PE12-->DIR   PB11-->ENA(�͵�ƽ��Ч)
// ��ʼ����TIM5���1000Hz PWM, ENA�͵�ƽʹ�ܵ������
// TIM5�����ж��ݲ�����, ��motor_start()�Ŵ�

****************************************************************/
void motor_init()
{
	// ��ʼ������ṹ��, �趨Ĭ��ֵ 
    memset(&g_motor, 0, sizeof(g_motor));             // ȫ������
    g_motor.target_pulses  = 10000000;                // Ĭ��Ŀ��������(�ܴ�, ����ͨ���ﵽ�����ͣ��)
    g_motor.current_hz     = 1000;                    // ��ǰƵ�ʳ�ʼ1000Hz
    g_motor.target_hz      = 1000;                    // Ŀ��Ƶ�ʳ�ʼ1000Hz
    g_motor.user_speed_hz  = 1000;                    // �û��趨�ٶȳ�ʼ1000Hz
    // running/speed_changing/target_reached/stop_pending ��Ϊ0

	// ʹ�� GPIOA , GPIOB , GPIOE , TIM5ʱ�� 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // PA0  -> TIM5_CH1 PWM���
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   // PB11 -> ENA ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);   // PE12 -> DIR ����
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,  ENABLE);   // TIM5 -> PWM����+�������

	// ���ÿ������� PB11(ENA) PE12(DIR) 
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // �������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              // PB11 -> ENA
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;              // PE12 -> DIR
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOB, GPIO_Pin_11);                     // ENA�͵�ƽ -> ʹ�ܵ������
    GPIO_SetBits(GPIOE,  GPIO_Pin_12);                      // DIR�ߵ�ƽ -> Ĭ�Ϸ���(��û���״�ķ���)

	// ����TIM5ʱ��: 72MHz/720=100kHz, 100kHz/100=1000Hz 
    TIM_InternalClockConfig(TIM5);                          // TIM5ʹ���ڲ�ʱ��
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // ����Ƶ
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;                 // ARR=99, PWMƵ��=1000Hz
    TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;              // PSC=719, ��ʱ��ʱ��=100kHz
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            // �߼���ʱ����, �����ò���
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

	// ����TIM5_CH1 PWM���, ռ�ձȹ̶�50%
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);                       // �Ƚṹ�帳��ʼֵ
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // PWMģʽ1: CNT<CCRʱ�����Ч��ƽ
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // ��Ч��ƽΪ��
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // ���ʹ��
    TIM_OCInitStructure.TIM_Pulse = 50;                           // CCR=50, ռ�ձ�=50/100=50%
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	// ����PA0Ϊ����������� -> TIM5_CH1(PUL) 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;              // �����������
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;                    // PA0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	// ����TIM5�ж����ȼ�(NVIC����2,��FreeRTOS_demo.c)
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;              // TIM5�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;    // ��ռ���ȼ�1
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;           // �����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;              // NVIC��ʹ��
    NVIC_Init(&NVIC_InitStructure);

	//δʹ���ж�

	// ʹ��TIM5, PWM��ʼ���
    TIM_Cmd(TIM5, ENABLE);                                      // ��ʱ����ʼ����, PWM���1000Hz
    // ��ʱ�����1000HzĬ���ٶ���ת, ��û����������ͼӼ��ٹ���
}

// �������Ŀ�굽���ź�������FreeRTOS_Start�е��ã�
void motor_target_semaphore_create(void)
{
    MotorTargetReachedSemaphore = xSemaphoreCreateBinary();
    configASSERT(MotorTargetReachedSemaphore != NULL);
}


/************************************************************

// �������

****************************************************************/

// ���õ������, ���Ӽ��ٱ���
// ����: ��ͣ�� -> �ȴ���ȫֹͣ -> �л��������� -> ��������
// ����Լ250~500ms (ȡ���ڵ�ǰ�ٶ�)
void motor_set_dir(uint8_t dir)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);  // ��ȡ��ǰ����
    if (current_dir == dir) return;                                     // ������ͬ, ����Ҫ����

    motor_stop();                                                       // б�¼���ͣ��

    while (g_motor.running) {        // �ȴ�ISR���ͣ������
        vTaskDelay(pdMS_TO_TICKS(1));                                   // �ó�CPU, 1ms������
    }                                                                   // running==0ʱ�˳�

    GPIO_WriteBit(GPIOE, GPIO_Pin_12, (BitAction)dir);                  // �л���������
    motor_start();                                                      // б�¼�������
}

// ֱ�ӷ�ת����, ��ͣ�����Ӽ���
// ����λ���������ڴ���˲�����, Ҫ�������Ӧ
void motor_change_dir(void)
{
    uint8_t current_dir = GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_12);  // ��ȡ��ǰ����
    if (current_dir == 0)
        GPIO_SetBits(GPIOE, GPIO_Pin_12);    // ��ǰ�� -> ��תΪ��
    else
        GPIO_ResetBits(GPIOE, GPIO_Pin_12);  // ��ǰ�� -> ��תΪ��
}


/************************************************************

// �ٶȿ���

****************************************************************/

// ���õ��Ŀ���ٶ�
// �ڲ����ݵ��״̬ѡ��ͬ����:
//   running=1 (������, ISRʹ��): �����Ӽ���б��, ͬʱȡ����������ͣ������
//   running=0 (����/ͣ��):     ֱ���޸�PWMƵ��������Ч
//     ����̬: TIM5������, ISR�ر� -> PWMƵ��ֱ�Ӹı���ת��
//     ͣ��̬: TIM5�ر�         -> ���ļĴ���ֵ, motor_start()����ʱ�Դ�500Hzб����
void motor_set_speed(uint32_t hz)
{
    g_motor.user_speed_hz = clamp_hz(hz);             // �޷�����¼�û������ٶ�

    if (g_motor.running) {
        // �����������, ͨ��ISRб�¼Ӽ���
        g_motor.target_hz     = g_motor.user_speed_hz; // ����б��Ŀ��
        g_motor.speed_changing = 1;                    // ����ISR�е�б���߼�
        g_motor.stop_pending   = 0;                    // ȡ��֮ǰ���ܴ�����ͣ������
    } else {
        // ���δ����(���л���ͣ��), ֱ������Ƶ�ʼ���
        g_motor.current_hz = g_motor.user_speed_hz;    // ͬ����ǰƵ��
        set_freq(g_motor.current_hz);                  // ֱ��д��TIM5�Ĵ���
    }
}

// ��ͣ��: б�¼��ٵ�MOTOR_MIN_HZ(500Hz), Ȼ����ISR���Զ��ر�TIM5
// �����3��״̬, ͣ�����Բ�ͬ:
//   running=1 (������): ����ͣ����־, ����ISR�𲽼��ٵ�500Hz���Զ��ر�TIM5
//   running=0 (����/��ͣ��): ֱ�ӹر�TIM5�����ж�
void motor_stop(void)
{
    if (g_motor.running) {
        // �����������, ����ͣ������
        g_motor.target_hz     = MOTOR_MIN_HZ;     // Ŀ��Ƶ����Ϊ���(500Hz)
        g_motor.speed_changing = 1;                // ����ISRб�¼���
        g_motor.stop_pending   = 1;                // ��֪ISR: ���ٵ�500Hz��ִ��ͣ��

        TIM_Cmd(TIM5, ENABLE);                     // ȷ��TIM5������
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); // ȷ�������ж���ʹ��
        // ������ISR���: ����->����500Hz->�ر�TIM5->����running
    } else {
        // ���δ����, ֱ�ӹر�TIM5����
        TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE); // �رո����ж�
        TIM_Cmd(TIM5, DISABLE);                     // �ر�TIM5, PWMֹͣ���
    }
}

// �������: ��MOTOR_MIN_HZ(500Hz)��, ��б�¼��ٵ�user_speed_hz
// ������������ֱ�ӷ���, �����ظ�����
void motor_start(void)
{
    if (g_motor.running) return;                    // ��������, ���ظ�����

    g_motor.running        = 1;                     // �������״̬
    g_motor.current_hz     = MOTOR_MIN_HZ;          // �������500Hz��
    g_motor.target_hz      = g_motor.user_speed_hz; // б��Ŀ��Ϊ�û��趨�ٶ�
    g_motor.speed_changing = 1;                     // ����б��
    g_motor.stop_pending   = 0;                     // ���ͣ����־

    set_freq(g_motor.current_hz);                   // �����500Hz
    TIM_Cmd(TIM5, ENABLE);                          // ȷ��TIM5����
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);      // ʹ�ܸ����ж�, ISR��ʼ����
    // ISR���𲽽�Ƶ�ʴ�500Hz�ӵ�user_speed_hz
}


/************************************************************

// �������API

****************************************************************/

// �����������, ͨ������λ������ʼ���г�ʱ����
void motor_reset_pulse_count(void)
{
    g_motor.current_pulses = 0;                     // �ѷ�����������
}

// ��ȡ��ǰ�ѷ�����������
uint32_t motor_get_pulse_count(void)
{
    return g_motor.current_pulses;
}

// �趨Ŀ��������, �����target_reached��־�Զ���1
void motor_set_pulse_count(uint32_t need_pulses)
{
    g_motor.target_pulses = need_pulses;            // 设定目标值, ISR中会不断比较
    g_motor.target_reached = 0;                     // 新目标，清除旧到达标志
}

// ��ѯ�Ƿ񵽴�Ŀ��������, ��ȡ���Զ�����(һ����֪ͨ)
// ����ֵ: 0=δ����  1=�ѵ���
uint8_t motor_is_target_reached(void)
{
    uint8_t temp = g_motor.target_reached;          // �ȶ���
    if (temp)
        g_motor.target_reached = 0;                 // �����Զ�����, ��ֹ�ظ�֪ͨ
    return temp;
}


/************************************************************

// TIM5�жϷ�����: ������� + �ٶ�б�� + ��ͣ��
// ÿ��TIM5�����¼�(��ÿ��PWM����)����һ��
// �ⲿ����ֻ�����޸ı�־λ, �������ٶȱ仯�ڴ�ISR�ڲ����

****************************************************************/
void TIM5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // ȷ���Ǹ����ж�
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);       // ����жϱ�־

        if (!g_motor.running) return;                     // ������̬, ������
                                                          // (TIM5���������е�ISR��Ӧ�ü���)

        g_motor.current_pulses++;                         // �����ڷ���һ������, ����+1

        // �ٶ�б��: �𲽵���current_hz�ƽ�target_hz
        if (g_motor.speed_changing)
        {
            // ��Ҫ����: current_hz < target_hz
            if (g_motor.current_hz < g_motor.target_hz)
            {
                g_motor.current_hz += MOTOR_RAMP_STEP;    // ÿ��+10Hz
                if (g_motor.current_hz > g_motor.target_hz)
                    g_motor.current_hz = g_motor.target_hz; // ��ֹ����
                set_freq(g_motor.current_hz);             // ����TIM5Ƶ�ʼĴ���
            }
            // ��Ҫ����: current_hz > target_hz
            else if (g_motor.current_hz > g_motor.target_hz)
            {
                g_motor.current_hz -= MOTOR_RAMP_STEP;    // ÿ��-10Hz
                if (g_motor.current_hz < g_motor.target_hz)
                    g_motor.current_hz = g_motor.target_hz; // ��ֹ����
                set_freq(g_motor.current_hz);             // ����TIM5Ƶ�ʼĴ���
            }

            // б�����: ��ǰƵ���ѵ���Ŀ��Ƶ��
            if (g_motor.current_hz == g_motor.target_hz)
            {
                g_motor.speed_changing = 0;               // �˳�б��ģʽ, �ٶ��ȶ�
            }
        }

        // ����Ŀ����: �ж��Ƿ��ѷ����㹻����������
        if (!g_motor.target_reached &&
            g_motor.current_pulses >= g_motor.target_pulses)
        {
            g_motor.target_reached = 1;                   // ��λ��־, ��motor_is_target_reached()��ѯ
                   // �ͷ��ź���������vTaskMotorReached�����е㵽��
                   xSemaphoreGiveFromISR(MotorTargetReachedSemaphore, &xHigherPriorityTaskWoken);
        }

        // ͣ������ж�: б�½��� + stop_pending��λ (stop_pending��λ��motor_stop�� 1 )
        // ֻ�е�����б�½���(speed_changing==0) �� ͣ��������Ч(stop_pending==1)
        if (!g_motor.speed_changing && g_motor.stop_pending)
        {
            g_motor.stop_pending = 0;                     // ���ͣ������
            g_motor.running      = 0;                     // ���ֹͣ
            TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);   // �رո����ж�
            TIM_Cmd(TIM5, DISABLE);                       // �ر�TIM5, PWMֹͣ
            // ��ʱmotor_set_dir()�е�while(g_motor.running)ѭ�������˳�
        }
    }
}
