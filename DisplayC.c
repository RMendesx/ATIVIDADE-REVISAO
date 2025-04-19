/* ------------------------------------------------------------------------------------------------------------------------ */
/* INCLUDE DE BIBLIOTECAS */
/* ------------------------------------------------------------------------------------------------------------------------ */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "matriz_led.pio.h"

/* ------------------------------------------------------------------------------------------------------------------------ */
/* DEFINIÇÕES DE PINOS E VARIÁVEIS GLOBAIS */
/* ------------------------------------------------------------------------------------------------------------------------ */
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C      // Endereço I2C do display
#define NUM_PIXELS 25      // número de leds na matriz
#define LED_PIN 7          // pino de saída do led
#define LED_VERDE 11       // pino de saída do led verde
#define LED_AZUL 12        // pino de saída do led azul
#define LED_VERMELHO 13    // pino de saída do led vermelho
#define BOTAO_A 5          // Pino do botão A
#define BOTAO_B 6          // Pino do botão B
#define JOYSTICK_X_PIN 26  // GPIO para eixo X
#define JOYSTICK_Y_PIN 27  // GPIO para eixo Y
#define JOYSTICK_PB 22     // GPIO para botão do Joystick
#define BUZZER_A 21        // Pino do buzzer A
#define BUZZER_B 10        // Pino do buzzer B
#define DEBOUNCE_DELAY 400 // Tempo de debounce em milissegundos
#define DISPLAY_WIDTH 128  // Largura do display
#define DISPLAY_HEIGHT 64  // Altura do display
#define ADC_MAX_VALUE 4095 // Valor máximo do ADC

char c = '0';          // Variável para armazenar o número a ser exibido";
char last_char = '\0'; // valor inicial impossível (nenhum número)


/* ------------------------------------------------------------------------------------------------------------------------ */
/* CONFIGURAÇÃO DO BUZZER */
/* ------------------------------------------------------------------------------------------------------------------------ */
void buzzer_on(uint gpio, uint freq_hz) // Inicializa um buzzer com frequência em Hz
{
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(gpio);

  uint32_t clock = 125000000; // Clock da PWM da Pico
  uint32_t wrap = clock / freq_hz;

  pwm_set_wrap(slice, wrap);
  pwm_set_chan_level(slice, PWM_CHAN_A, wrap / 2); // 50% duty
  pwm_set_enabled(slice, true);
}

void buzzer_off(uint gpio) // Desliga o buzzer (desativa PWM e limpa o pino)
{
  uint slice = pwm_gpio_to_slice_num(gpio);
  pwm_set_enabled(slice, false);
  gpio_set_function(gpio, GPIO_FUNC_SIO);
  gpio_set_dir(gpio, GPIO_OUT);
  gpio_put(gpio, 0);
}

int64_t desligar_buzzer_callback(alarm_id_t id, void *user_data) // Callback para desligar o buzzer após um tempo
{
  uint gpio_buzzer = (uint)(uintptr_t)user_data;
  buzzer_off(gpio_buzzer);
  return 0;
}


/* ------------------------------------------------------------------------------------------------------------------------ */
/* CONFIGURAÇÃO DOS BOTÕES E LED RGB */
/* ------------------------------------------------------------------------------------------------------------------------ */
volatile uint32_t last_interrupt_time = 0; // Variável para armazenar o tempo do último evento de interrupção

int64_t apagar_led_callback(alarm_id_t id, void *user_data) // Callback para apagar o LED após um tempo
{
  uint gpio_led = (uint)(uintptr_t)user_data;
  gpio_put(gpio_led, 0);
  return 0;
}

void gpio_callback(uint gpio, uint32_t events) // Função de callback para interrupções dos botões
{
  static uint32_t last_gpio = 0;

  uint32_t current_time = to_ms_since_boot(get_absolute_time());

  if (gpio == last_gpio && (current_time - last_interrupt_time < DEBOUNCE_DELAY)) // Debounce dos botões
    return;

  last_interrupt_time = current_time;
  last_gpio = gpio;

  if (gpio == BOTAO_A) // Se o botão A for pressionado
  {
    buzzer_on(BUZZER_A, 2000); // Liga o buzzer A
    buzzer_on(BUZZER_B, 2000); // Liga o buzzer B

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false); // Adiciona alarme para desligar o buzzer A após 100ms
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false); // Adiciona alarme para desligar o buzzer B após 100ms

    c = c - 1;   // Decrementa o valor de c a cada pressionamento do botão A
    if (c < '0') // Se c for menor que 0, volta para 9
      c = '9';

    if (gpio_get(LED_VERDE) == 0)      // Verifique o estado atual do LED
    {                         
      gpio_put(LED_VERDE, 1);          // Acende o LED se ele estiver apagado
      printf("Led verde ligado\n");    // Informa a ativação do LED verde

      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_VERDE, false); // Adiciona alarme para desligar o LED após 1000ms
      printf("Led verde desligado\n"); // Informa a desativação do LED verde
      printf("O número diminuiu\n");   // Informa que o número diminuiu
    }
  }
  else if (gpio == BOTAO_B) // Se o botão B for pressionado
  {
    buzzer_on(BUZZER_A, 2000); // Liga o buzzer A
    buzzer_on(BUZZER_B, 2000); // Liga o buzzer B

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false); // Adiciona alarme para desligar o buzzer A após 100ms
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false); // Adiciona alarme para desligar o buzzer B após 100ms

    c = c + 1; // Incrementa o valor de c a cada pressionamento do botão B
    if (c > '9') // Se c for maior que 9, volta para 0
      c = '0';

    if (gpio_get(LED_AZUL) == 0)     // Verifique o estado atual do LED
    {
      gpio_put(LED_AZUL, 1);          // Acende o LED se ele estiver apagado
      printf("Led azul ligado\n");    // Informa a ativação do LED azul

      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_AZUL, false); // Adiciona alarme para desligar o LED após 1000ms
      printf("Led azul desligado\n"); // Informa a desativação do LED azul
      printf("O número aumentou\n");  // Informa que o número aumentou
    }
  }
  else if (gpio == JOYSTICK_PB)
  {
    buzzer_on(BUZZER_A, 2000); // Liga o buzzer A
    buzzer_on(BUZZER_B, 2000); // Liga o buzzer B

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false); // Adiciona alarme para desligar o buzzer A após 100ms
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false); // Adiciona alarme para desligar o buzzer B após 100ms

    c = '0'; // Reinicia o valor de c para 0

    if (gpio_get(LED_VERMELHO) == 0)    // Verifique o estado atual do LED
    {
      gpio_put(LED_VERMELHO, 1);          // Acende o LED se ele estiver apagado
      printf("Led vermelho ligado\n");    // Informa a ativação do LED vermelho

      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_VERMELHO, false); // Adiciona alarme para desligar o LED após 1000ms
      printf("Led vermelho desligado\n"); // Informa a desativação do LED vermelho
      printf("número reiniciado\n");      // Informa que o número foi reiniciado
    }
  }
}


/* ------------------------------------------------------------------------------------------------------------------------ */
/* CONFIGURAÇÃO DA MATRIZ DE LEDS */
/* ------------------------------------------------------------------------------------------------------------------------ */
PIO pio; // PIO para controle da matriz de LEDs
uint sm; // State machine para controle da matriz de LEDs
float r = 1.0, g = 1.0, b = 1.0; // Intensidade de cores do LED RGB

uint matrix_rgb(float r, float g, float b) // Rotina para definição da intensidade de cores do LED RGB
{
  unsigned char R, G, B;
  R = r * 1; 
  G = g * 1;
  B = b * 1;
  return (G << 24) | (R << 16) | (B << 8); 
}

int getIndex(int x, int y) // Função para converter a posição do matriz para uma posição do vetor.
{
  if (y % 2 == 0) // Verifica se a linha é par ou ímpar
  {
    return 24 - (y * 5 + x); // Linha par (esquerda para direita).
  }
  else
  {
    return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
  }
}

void desenho_pio(double *desenho) // Rotina para desenhar na matriz de LEDs
{
  for (int16_t i = 0; i < NUM_PIXELS; i++) 
  {
    uint32_t valor_led = matrix_rgb(desenho[i] * r, desenho[i] * g, desenho[i] * b);
    pio_sm_put_blocking(pio, sm, valor_led);
  }
}

double numer[10][25] = 
{
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
}; // Matriz de LEDs para os números 0 a 9


/* ------------------------------------------------------------------------------------------------------------------------ */
/* CONFIGURAÇÃO DO ADC */
/* ------------------------------------------------------------------------------------------------------------------------ */

int map_adc_to_display(int adc_value, int adc_max, int display_max) // Função para mapear o valor do ADC para a posição do display
{
  return (adc_value * display_max) / adc_max;
}


/* ------------------------------------------------------------------------------------------------------------------------ */
/* INÍCIO DO PROGRAMA */
/* ------------------------------------------------------------------------------------------------------------------------ */
int main()
{
  /* CONFIGURAÇÃO DOS PINOS GPIO, INTERRUPÇÕES, PIO, I2C E JOYSTICK*/
  
  pio = pio0; // PIO0 é usado para controle da matriz de LEDs
  bool frequenciaClock; 

  frequenciaClock = set_sys_clock_khz(125000, false); // Configura o clock do sistema para 125MHz
  stdio_init_all(); // Inicializa a comunicação serial

  gpio_init(LED_VERDE); // Inicializa o LED verde
  gpio_set_dir(LED_VERDE, GPIO_OUT); // Define o LED verde como saída
  
  gpio_init(LED_AZUL); // Inicializa o LED azul
  gpio_set_dir(LED_AZUL, GPIO_OUT); // Define o LED azul como saída

  gpio_init(LED_VERMELHO); // Inicializa o LED vermelho
  gpio_set_dir(LED_VERMELHO, GPIO_OUT); // Define o LED vermelho como saída

  gpio_init(BOTAO_A); // Inicializa o botão A
  gpio_set_dir(BOTAO_A, GPIO_IN); // Define o botão A como entrada
  gpio_pull_up(BOTAO_A); // Ativa o pull-up no botão A
  
  gpio_init(BOTAO_B); // Inicializa o botão B
  gpio_set_dir(BOTAO_B, GPIO_IN); // Define o botão B como entrada
  gpio_pull_up(BOTAO_B); // Ativa o pull-up no botão B

  gpio_init(JOYSTICK_PB); // Inicializa o botão do joystick
  gpio_set_dir(JOYSTICK_PB, GPIO_IN); // Define o botão do joystick como entrada
  gpio_pull_up(JOYSTICK_PB); // Ativa o pull-up no botão do joystick

  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, gpio_callback); // Configura interrupção para o botão A
  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, gpio_callback); // Configura interrupção para o botão B
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, gpio_callback); // Configura interrupção para o botão do joystick

  uint offset = pio_add_program(pio, &pio_matrix_program); // Adiciona o programa PIO para controle da matriz de LEDs
  sm = pio_claim_unused_sm(pio, true);                     // Reivindica uma máquina de estado não utilizada
  pio_matrix_program_init(pio, sm, offset, LED_PIN);       // Inicializa o programa PIO na máquina de estado reivindicada
  c = '0';                                                 // Inicializa o valor de c para 0

  i2c_init(I2C_PORT, 400 * 1000);                               // Inicializa o I2C na porta especificada com 400kHz
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Define o pino SDA como função I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Define o pino SCL como função I2C
  gpio_pull_up(I2C_SDA);                                        // Pull up na data line
  gpio_pull_up(I2C_SCL);                                        // Pull up na clock line
  ssd1306_t ssd;                                                // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd);                                         // Configura o display
  ssd1306_send_data(&ssd);  

  ssd1306_fill(&ssd, false); // Limpa o display
  ssd1306_send_data(&ssd);   // Atualiza o display

  bool cor = true;

  adc_init(); // Inicializa o ADC
  adc_gpio_init(JOYSTICK_X_PIN); // Inicializa o pino do eixo X do joystick
  adc_gpio_init(JOYSTICK_Y_PIN); // Inicializa o pino do eixo Y do joystick

  uint16_t adc_value_x; // Variável para armazenar o valor do eixo X
  uint16_t adc_value_y; // Variável para armazenar o valor do eixo Y

  
  /* LOOP INFINITO DO PROGRAMA */
  while (true)
  {
    adc_select_input(0); // seleciona o canal X (GPIO 26)
    adc_value_x = adc_read(); // lê o valor do ADC do eixo X
    adc_select_input(1); // seleciona o canal Y (GPIO 27)
    adc_value_y = adc_read(); // lê o valor do ADC do eixo Y

    int x_pos = DISPLAY_HEIGHT - 7.5 - map_adc_to_display(adc_value_x, ADC_MAX_VALUE, DISPLAY_HEIGHT - 7); // Mapeia o valor do ADC para a posição do display
    int y_pos = map_adc_to_display(adc_value_y, ADC_MAX_VALUE, DISPLAY_WIDTH - 7); // Mapeia o valor do ADC para a posição do display

    ssd1306_fill(&ssd, !cor);                         // Limpa o display
    ssd1306_rect(&ssd, x_pos, y_pos, 8, 8, cor, cor); // Desenha o quadrado mapeado
    ssd1306_send_data(&ssd);                          // Atualiza o display

    if (c != last_char) 
    {

      last_char = c; // atualiza o último caractere processado

      switch (c) // Verifica o valor de c e desenha o número correspondente na matriz de LEDs
      {
      case '0': // Desenha o número 0
        desenho_pio(numer[0]); 
        printf("número 0!\n\n");
        break;

      case '1': // Desenha o número 1
        desenho_pio(numer[1]);
        printf("número 1!\n\n");
        break;

      case '2': // Desenha o número 2
        desenho_pio(numer[2]);
        printf("número 2!\n\n");
        break;

      case '3': // Desenha o número 3
        desenho_pio(numer[3]);
        printf("número 3!\n\n");
        break;

      case '4': // Desenha o número 4
        desenho_pio(numer[4]);
        printf("número 4!\n\n");
        break;

      case '5': // Desenha o número 5
        desenho_pio(numer[5]);
        printf("número 5!\n\n");
        break;

      case '6': // Desenha o número 6
        desenho_pio(numer[6]);
        printf("número 6!\n\n");
        break;

      case '7': // Desenha o número 7
        desenho_pio(numer[7]);
        printf("número 7!\n\n");
        break;

      case '8': // Desenha o número 8
        desenho_pio(numer[8]);
        printf("número 8!\n\n");
        break;

      case '9': // Desenha o número 9
        desenho_pio(numer[9]);
        printf("número 9!\n\n");
        break;
      }
    }
  }
}