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

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
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
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define ADC_MAX_VALUE 4095



char c = '0';              // Variável para armazenar o número a ser exibido";



// Inicializa um buzzer com frequência em Hz
void buzzer_on(uint gpio, uint freq_hz) {
  gpio_set_function(gpio, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(gpio);

  uint32_t clock = 125000000; // Clock da PWM da Pico
  uint32_t wrap = clock / freq_hz;

  pwm_set_wrap(slice, wrap);
  pwm_set_chan_level(slice, PWM_CHAN_A, wrap / 2); // 50% duty
  pwm_set_enabled(slice, true);
}

void buzzer_off(uint gpio) {
  uint slice = pwm_gpio_to_slice_num(gpio);
  pwm_set_enabled(slice, false);
  gpio_set_function(gpio, GPIO_FUNC_SIO);
  gpio_set_dir(gpio, GPIO_OUT);
  gpio_put(gpio, 0);
}

// Desliga o buzzer (desativa PWM e limpa o pino)
int64_t desligar_buzzer_callback(alarm_id_t id, void *user_data) {
  uint gpio_buzzer = (uint)(uintptr_t)user_data;
  buzzer_off(gpio_buzzer);
  return 0;
}

volatile uint32_t last_interrupt_time = 0;

int64_t apagar_led_callback(alarm_id_t id, void *user_data)
{
  uint gpio_led = (uint)(uintptr_t)user_data;
  gpio_put(gpio_led, 0);
  return 0;
}

void gpio_callback(uint gpio, uint32_t events)
{
  static uint32_t last_gpio = 0;

  uint32_t current_time = to_ms_since_boot(get_absolute_time());

  // Verifica se é o mesmo botão pressionado e se está dentro do tempo de debounce
  if (gpio == last_gpio && (current_time - last_interrupt_time < DEBOUNCE_DELAY))
    return;

  last_interrupt_time = current_time;
  last_gpio = gpio;






  if (gpio == BOTAO_A)
  {
    buzzer_on(BUZZER_A, 2000);
    buzzer_on(BUZZER_B, 2000);

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false);
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false);

    c = c - 1; // Decrementa o valor de c a cada pressionamento do botão A
    if (c < '0')
      c = '9'; // Se c for menor que 0, volta para 9

    if (gpio_get(LED_VERDE) == 0)
    {                         // Verifique o estado atual do LED
      gpio_put(LED_VERDE, 1); // Acende o LED se ele estiver apagado
      printf("Led verde ligado\n");

      // Adiciona alarme para desligar o LED após 1000ms
      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_VERDE, false);
      printf("Led verde desligado\n");
    }
  }



  else if (gpio == BOTAO_B)
  {
    buzzer_on(BUZZER_A, 2000);
    buzzer_on(BUZZER_B, 2000);

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false);
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false);
    
    c = c + 1; // Incrementa o valor de c a cada pressionamento do botão B
    if (c > '9')
      c = '0'; // Se c for maior que 9, volta para 0

    if (gpio_get(LED_AZUL) == 0)
    {
      gpio_put(LED_AZUL, 1);
      printf("Led azul ligado\n");
      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_AZUL, false);
      printf("Led azul desligado\n");
    }
  }




  else if (gpio == JOYSTICK_PB)
  {
    buzzer_on(BUZZER_A, 2000);
    buzzer_on(BUZZER_B, 2000);

    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_A, false);
    add_alarm_in_ms(100, desligar_buzzer_callback, (void *)(uintptr_t)BUZZER_B, false);
    
    c = '0';

    if (gpio_get(LED_VERMELHO) == 0)
    {
      gpio_put(LED_VERMELHO, 1);
      printf("Led vermelho ligado\n");
  
      add_alarm_in_ms(1000, apagar_led_callback, (void *)(uintptr_t)LED_VERMELHO, false);
      printf("Led vermelho desligado\n");
    }
  }
}

// MATRIZ DE LEDS

PIO pio;
uint sm;
float r = 1.0, g = 1.0, b = 1.0;

// rotina para definição da intensidade de cores do led
uint matrix_rgb(float r, float g, float b)
{
  unsigned char R, G, B;
  R = r * 2;
  G = g * 0;
  B = b * 0;
  return (G << 24) | (R << 16) | (B << 8);
}

// Função para converter a posição do matriz para uma posição do vetor.
int getIndex(int x, int y)
{
  // Se a linha for par (0, 2, 4), percorremos da esquerda para a direita.
  // Se a linha for ímpar (1, 3), percorremos da direita para a esquerda.
  if (y % 2 == 0)
  {
    return 24 - (y * 5 + x); // Linha par (esquerda para direita).
  }
  else
  {
    return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
  }
}

void desenho_pio(double *desenho)
{
  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    uint32_t valor_led = matrix_rgb(desenho[i] * r, desenho[i] * g, desenho[i] * b);
    pio_sm_put_blocking(pio, sm, valor_led);
  }
}

double numer[10][25] = {
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
};

// Função para mapear o valor do ADC para a posição do display
int map_adc_to_display(int adc_value, int adc_max, int display_max)
{
  return (adc_value * display_max) / adc_max;
}

int main()
{
  pio = pio0;
  bool frequenciaClock;

  frequenciaClock = set_sys_clock_khz(125000, false);
  stdio_init_all();

  gpio_init(LED_VERDE);
  gpio_set_dir(LED_VERDE, GPIO_OUT);

  gpio_init(LED_AZUL);
  gpio_set_dir(LED_AZUL, GPIO_OUT);

  gpio_init(LED_VERMELHO);
  gpio_set_dir(LED_VERMELHO, GPIO_OUT);

  gpio_init(BOTAO_A);
  gpio_set_dir(BOTAO_A, GPIO_IN);
  gpio_pull_up(BOTAO_A);

  gpio_init(JOYSTICK_PB);
  gpio_set_dir(JOYSTICK_PB, GPIO_IN);
  gpio_pull_up(JOYSTICK_PB);

  gpio_init(BOTAO_B);
  gpio_set_dir(BOTAO_B, GPIO_IN);
  gpio_pull_up(BOTAO_B);

  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
  gpio_set_irq_enabled_with_callback(JOYSTICK_PB, GPIO_IRQ_EDGE_FALL, true, gpio_callback);

  uint offset = pio_add_program(pio, &pio_matrix_program);
  sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, LED_PIN);
  c = '0';







  // I2C Initialisation. Using it at 400Khz.
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
  gpio_pull_up(I2C_SDA);                                        // Pull up the data line
  gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
  ssd1306_t ssd;                                                // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd);                                         // Configura o display
  ssd1306_send_data(&ssd);                                      // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  bool cor = true;

  adc_init();
  adc_gpio_init(JOYSTICK_X_PIN);
  adc_gpio_init(JOYSTICK_Y_PIN);

  uint16_t adc_value_x;
  uint16_t adc_value_y;



  while (true)
  {
    adc_select_input(0); // seleciona o canal X (GPIO 26)
    adc_value_x = adc_read();
    adc_select_input(1); // seleciona o canal Y (GPIO 27)
    adc_value_y = adc_read();

    int x_pos = DISPLAY_HEIGHT - 7.5 - map_adc_to_display(adc_value_x, ADC_MAX_VALUE, DISPLAY_HEIGHT - 7);
    int y_pos = map_adc_to_display(adc_value_y, ADC_MAX_VALUE, DISPLAY_WIDTH - 7);

    ssd1306_fill(&ssd, !cor);                         // Limpa o display
    ssd1306_rect(&ssd, x_pos, y_pos, 8, 8, cor, cor); // Desenha o quadrado mapeado
    ssd1306_send_data(&ssd);                          // Atualiza o display                                     // Atualiza o display

    switch (c)
    {
    case '0':
      desenho_pio(numer[0]);
      printf("número 0!\n");
      break;
    case '1':
      desenho_pio(numer[1]);
      printf("número 1!\n");
      break;
    case '2':
      desenho_pio(numer[2]);
      printf("número 2!\n");
      break;
    case '3':
      desenho_pio(numer[3]);
      printf("número 3!\n");
      break;
    case '4':
      desenho_pio(numer[4]);
      printf("número 4!\n");
      break;
    case '5':
      desenho_pio(numer[5]);
      printf("número 5!\n");
      break;
    case '6':
      desenho_pio(numer[6]);
      printf("número 6!\n");
      break;
    case '7':
      desenho_pio(numer[7]);
      printf("número 7!\n");
      break;
    case '8':
      desenho_pio(numer[8]);
      printf("número 8!\n");
      break;
    case '9':
      desenho_pio(numer[9]);
      printf("número 9!\n");
      break;
    }
  }
}