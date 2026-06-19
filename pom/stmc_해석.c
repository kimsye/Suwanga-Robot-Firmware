/* 핵심 기능에 필요한 파일들을 불러옵니다. (파이썬의 import와 같음) */
#include "main.h"
#include "adc.h"   // 가변저항(아날로그) 값을 읽기 위한 라이브러리
#include "usart.h" // PC로 데이터를 쏴주기(시리얼 통신) 위한 라이브러리
#include "gpio.h"  // 핀(버튼 등)의 입출력을 제어하기 위한 라이브러리

/* ========================================================== */
/* 전역 변수 선언 (데이터를 담아둘 바구니) */
/* ========================================================== */
uint32_t adc[9];       // 9개의 가변저항 값을 저장할 배열 (0~8번 칸)
char msg[128];         // PC로 보낼 글자들을 묶어둘 텍스트 상자 (최대 128글자)
uint8_t sw_toggle = 0; // 조이스틱/스위치의 On/Off 상태를 기억하는 변수 (0 또는 1)

void SystemClock_Config(void);

/* ========================================================== */
/* 아날로그(가변저항) 값을 디지털 숫자로 읽어오는 함수 */
/* ========================================================== */
void Read_ADC_Channel(uint32_t channel, uint32_t *value)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    // 1. 읽어올 핀(채널)을 설정합니다.
    sConfig.Channel = channel;
    sConfig.Rank = 1;

    // 2. 센서 값을 얼마나 길게 읽을지(샘플링 타임) 설정합니다. 
    // (충분히 길게 설정해야 값이 정확하게 읽힙니다)
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    sConfig.Offset = 0;

    // 3. 설정한 내용을 ADC 하드웨어에 적용합니다.
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // 4. ADC 변환(아날로그 -> 숫자)을 시작합니다.
    HAL_ADC_Start(&hadc1);

    // 5. 변환이 끝날 때까지 잠시 기다립니다. (기다리지 않으면 쓰레기 값이 들어옵니다)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    // 6. 변환이 완료된 0~4095 사이의 숫자를 가져와서 value(배열의 특정 칸)에 저장합니다.
    *value = HAL_ADC_GetValue(&hadc1);

    // 7. 다음 채널을 읽기 위해 ADC를 잠시 끕니다.
    HAL_ADC_Stop(&hadc1);
}

/* ========================================================== */
/* 메인 함수 (프로그램의 시작점) */
/* ========================================================== */
int main(void)
{
    // MCU 기본 설정 및 초기화 (준비 운동)
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    /* ========================================================== */
    /* 무한 루프 (로봇이 켜져 있는 동안 계속 반복되는 구역) */
    /* ========================================================== */
    while (1)
    {
        // [1] 9개의 가변저항 값을 차례대로 읽어서 adc 배열에 저장합니다.
        Read_ADC_Channel(ADC_CHANNEL_0, &adc[0]); // A0 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_1, &adc[1]); // A1 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_4, &adc[2]); // A4 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_5, &adc[3]); // A5 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_6, &adc[4]); // A6 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_7, &adc[5]); // A7 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_8, &adc[6]); // B0 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_9,  &adc[7]); // PB1 핀 읽기
        Read_ADC_Channel(ADC_CHANNEL_10, &adc[8]); // PC0 핀 읽기

        // [2] 스위치(버튼) 상태 확인 로직 (에지 디텍션)
        static uint8_t prev = 1; // 이전 버튼 상태 (기본값 1: 안 눌림)
        uint8_t now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2); // 현재 버튼 상태 읽기 (0: 눌림)
        
        // 만약 '방금 전엔 안 눌렸는데(1)', '지금은 눌렸다면(0)' = 즉, 누르는 순간!
        if(prev == 1 && now == 0) 
        {
            sw_toggle = !sw_toggle; // 스위치 상태를 뒤집습니다 (0->1, 1->0)
            HAL_Delay(10);          // 버튼이 물리적으로 튀는 현상(채터링/노이즈)을 무시하기 위해 10ms 대기
        }
        prev = now; // 다음 번 확인을 위해 '현재 상태'를 '이전 상태'로 저장해둠


        // [3] PC로 보낼 데이터 포장하기 (문자열 만들기)
        // "%lu"는 숫자 하나, "%d"는 스위치 상태, "\r\n"은 엔터(줄바꿈)를 뜻합니다.
        int len = sprintf(msg,
        "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%d\r\n",
        adc[0], adc[1], adc[2],
        adc[3], adc[4], adc[5],
        adc[6], adc[7], adc[8],
        sw_toggle);

        // [4] 포장된 데이터(msg)를 USB 시리얼(UART2)을 통해 PC로 발사합니다.
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

        // [5] 40ms(0.04초) 동안 잠시 쉬고 루프의 처음으로 돌아갑니다.
        HAL_Delay(40); 
    }
}

/* 아래는 보드의 심장 박동(클럭 주파수)과 에러 처리를 담당하는 기본 설정 코드들입니다. */
// (초기 설정용이므로 당장 깊게 파고들지 않아도 괜찮습니다.)
void SystemClock_Config(void) { /* ... 생략 ... */ }
void Error_Handler(void) { /* ... 생략 ... */ }

1. Read_ADC_Channel 함수: "테이블 돌아다니며 주문받기"
손님(가변저항)들이 앉아있는 테이블 번호(Channel)로 가서, "얼마나 꺾였나요?" 하고 물어본 뒤 숫자(0~4095)를 받아 수첩(*value)에 적는 행동입니다.
아날로그를 디지털로 바꾸는 데는 아주 미세한 시간이 필요하기 때문에, 변환이 끝날 때까지 기다리는 과정(PollForConversion)이 포함되어 있습니다.

2. 메인 무한 루프 while (1): "무한 반복 업무"
로봇에 전원이 들어와 있는 한 이 구역은 영원히 반복됩니다. 종업원(STM32)은 쉬지 않고 일합니다.

수첩(adc 배열)을 들고 1번 테이블부터 9번 테이블까지 쫙 돌면서 가변저항 값을 적습니다.

카운터에 가서 버튼이 눌렸는지 확인합니다(sw_toggle).

3. 버튼 로직 (prev와 now): "누르는 순간 포착하기"
이 코드는 굉장히 똑똑하게 짜인 부분입니다. 컴퓨터는 1초에 수만 번 명령을 처리하므로, 사람이 버튼을 '딸깍' 누르는 짧은 순간에도 무한 루프는 수백 번 돌아갑니다.
만약 단순히 "버튼이 눌렸냐?"만 물어보면 숫자가 0,1,0,1,0... 하며 미친 듯이 바뀔 겁니다.
그래서 '조금 전 상태(prev)'와 '지금 상태(now)'를 비교합니다. "아까는 안 눌렸는데 지금은 눌린 그 찰나의 순간!"에만 스위치를 딱 한 번 켜거나 끄는(!sw_toggle) 것입니다.

4. sprintf와 HAL_UART_Transmit: "주문서 포장해서 주방(파이썬)으로 던지기"
수첩에 적힌 10개의 숫자(가변저항 9개 + 스위치 1개)를 그대로 주방에 던지면 요리사(파이썬)가 알아볼 수 없습니다.

sprintf: "1024, 2048, 512, ... , 1 [엔터]" 처럼 쉼표로 예쁘게 구분된 하나의 문자열 상자(msg)로 포장합니다.

Transmit: 포장된 상자를 통신선(UART)을 통해 PC의 파이썬으로 휙 던집니다.

HAL_Delay(40): 너무 빨리 던지면 PC가 과부하 걸리니까, 0.04초 숨을 고르고 다시 처음부터 시작합니다. (초당 25번 송신)

🔗 파이썬 코드와의 연결 고리
바로 이 C 코드가 1초에 25번씩 던져주는 "값,값,값...[엔터]" 문자열을, 서현님의 파이썬 코드에 있는 line.split(",") 함수가 받아서 쉼표 단위로 촥촥 잘라 쓰는 것입니다!