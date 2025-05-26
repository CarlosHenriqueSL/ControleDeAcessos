/*
 *  Por: Carlos Henrique Silva Lopes
 *  Data: 21-05-2025
 *
 *  Controle de acessos usando semaforos e mutexes com o FreeRTOS
 *
 *  Descrição: Controle de acessos simultaneos.
 *  O botão A incrementa o numero de usuarios e o botao B decrementa. O joystick reinicia a contagem.
 *  O numero de usuarios eh exibido no Display OLED SSD1306.
 *  O sistema conta com buzzer para acessibilidade para pessoas cegas
 *  A matriz de LEDs exibe padrao para alerta quanto ao numero de usuarios.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "blink.pio.h"
#include "lib/ssd1306.h"
#include "lib/numeros.h"

// Definicao dos pinos
#define WS2812_PIN 7

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C

#define BUTTON_A 5
#define BUTTON_B 6
#define BUTTON_JOY 22
#define LED_PIN_GREEN 11
#define LED_PIN_BLUE 12
#define LED_PIN_RED 13

#define DEBOUNCE_MS 1250

#define BUZZER_PIN 21

SemaphoreHandle_t xMutex;
SemaphoreHandle_t xBinarySem;
SemaphoreHandle_t xCounterSem;

ssd1306_t ssd;

static uint32_t lastIrqTime = 0; // Registra o tempo da ultima interrupcao

volatile bool limiteAtingido = false;        // Verifica se o limite de 10 usuarios foi atingido
volatile bool botaoPressionado = false;      // True: se um botao foi pressionado nos ultimos 1.25s. Senao, false
volatile uint8_t numeroDeUsuarios = 0;       // Registra o numero de usuarios no sistema
volatile uint8_t ultimoBotaoPressionado = 0; // Detecta o ultimo botao pressionado, A, B ou Joystick

/*
 * Tarefa para o botao do joystick (gpio 22). Reinicia o numero de usuarios a 0 e mostra no display a
 * quantidade de foi retirada. A escrita no display, bem como para os outros botoes, eh protegida pelo
 * mutex. Utiliza um semaforo binario para registrar a interrupcao.
 */
void vTaskReset()
{
    static char buffer[13];
    while (true)
    {
        // Verifica semaforo binario liberado
        if (xSemaphoreTake(xBinarySem, portMAX_DELAY) == pdTRUE)
        {
            // Verifica mutex liberado para escrita no display
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                sprintf(buffer, "-%d usuario(s)!", numeroDeUsuarios);
                ssd1306_draw_string(&ssd, buffer, 5, 5);
                ssd1306_send_data(&ssd);
                xSemaphoreGive(xMutex);
            }
            vTaskDelay(pdMS_TO_TICKS(1250));
            numeroDeUsuarios = 0;
            xQueueReset(xCounterSem);
            botaoPressionado = false;
        }
    }
}

/*
 * Tarefa para o botao A (gpio 5). Adiciona um usuario no sistema e mostra "usuario adicionado!" no
 * display. A escrita no display eh protegida pelo mutex. Utiliza o semaforo contador para registrar a
 * interrupcao. Como o semaforo eh compartilhado com a tarefa de saida (botao B), ele verifica se
 * realmente foi o botao A o ultimo pressionado (Give() feito pelo botao A), e se o numero de usuarios eh
 * menor que 10. Se sim, prossegue com a remocao do usuario. Senao, devolve o semaforo.
 */
void vTaskSaida()
{
    while (true)
    {
        // Verifica semaforo contador liberado
        if (xSemaphoreTake(xCounterSem, portMAX_DELAY) == pdTRUE)
        {
            /* Verifica se realmente foi o botao B quem liberou o semaforo. E se ha usuarios que podem ser
               removidos (numero de usuarios diferente de 0) */
            if (ultimoBotaoPressionado == BUTTON_B && numeroDeUsuarios != 0)
            {
                // Verifica mutex liberado para escrita no display
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                {
                    ssd1306_draw_string(&ssd, "-1 usuario!", 5, 5);
                    ssd1306_send_data(&ssd);
                    xSemaphoreGive(xMutex);
                }
                vTaskDelay(pdMS_TO_TICKS(1250));
                numeroDeUsuarios--;
                botaoPressionado = false;
                printf("%d", numeroDeUsuarios);
            }
            else
            {
                // Devolve o semáforo se não for o botão B
                xSemaphoreGive(xCounterSem);
            }
        }
    }
}

/*
 * Tarefa para o botao B (gpio 6). Retira um usuario do sistema e mostra "-1 usuario!" no display.
 * A escrita no display eh protegida pelo mutex. Utiliza o semaforo contador para registrar a interrupcao.
 * Como o semaforo eh compartilhado com a tarefa de entrada (botao A), ele verifica se realmente foi o
 * botao B o ultimo pressionado (Give() feito pelo botao B), e se o numero de usuarios eh maior que 0.
 * Se sim, prossegue com a remocao do usuario. Senao, devolve o semaforo.
 */
void vTaskEntrada()
{
    while (true)
    {
        // Verifica semaforo contador liberado
        if (xSemaphoreTake(xCounterSem, portMAX_DELAY) == pdTRUE)
        {
            /* Verifica se realmente foi o botao A quem liberou o semaforo. E se ha usuarios que podem ser
               adicionados (numero de usuarios menor que 10) */
            if (ultimoBotaoPressionado == BUTTON_A && numeroDeUsuarios < 10)
            {
                // Verifica mutex liberado para escrita no display
                if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                {
                    ssd1306_draw_string(&ssd, "Novo usuario!", 5, 5);
                    ssd1306_send_data(&ssd);
                    xSemaphoreGive(xMutex);
                }
                vTaskDelay(pdMS_TO_TICKS(1250));
                numeroDeUsuarios++;
                botaoPressionado = false;
                printf("%d", numeroDeUsuarios);
            }
            else
            {
                // Devolve o semáforo se não for o botão A
                xSemaphoreGive(xCounterSem);
            }
        }
    }
}

// Funcao para tocar o buzzer, criada para desacoplamento da funcao vTaskBuzzer, tarefa que usa essa funcao
void tocarBuzzer(uint slice, uint chan, uint wrap, float freq)
{
    // Toca o buzzer com uma frequencia, espera e desliga
    pwm_set_clkdiv(slice, freq);
    pwm_set_chan_level(slice, chan, wrap / 2);
    vTaskDelay(pdMS_TO_TICKS(100));
    pwm_set_chan_level(slice, chan, 0);
}

/*
 * Tarefa para tocar o buzzer, emite um sinal sonoro sempre que um botao eh pressionado. O sinal muda a
 * depender da quantidade de usuarios no sistema. Tambem emite um som de alerta para caso o sistema esteja
 * cheio e tenha uma tentativa de acrescentar um novo usuario.
 */
void vTaskBuzzer()
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice, true);
    uint chan = pwm_gpio_to_channel(BUZZER_PIN);
    uint wrap = 125000000 / 3500;
    pwm_set_wrap(slice, wrap);

    while (true)
    {
        // Sinal para alertar que um novo usuario entrou sem problemas no sistema
        if (botaoPressionado && (0 <= numeroDeUsuarios && numeroDeUsuarios < 8))
        {
            tocarBuzzer(slice, chan, 125000000 / 3500, 2.0f);
            wrap = 0;
            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        /* Sinal para alertar que um novo usuario entrou sem problemas no sistema, mas esta mais proximo
           de encher (LED verde aceso)*/
        else if (botaoPressionado && numeroDeUsuarios == 8)
        {
            tocarBuzzer(slice, chan, 125000000 / 3500, 4.0f);
            wrap = 0;
            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        // Sinal para alertar que o sistema comporta apenas mais um usuario
        else if (botaoPressionado && numeroDeUsuarios == 9)
        {
            tocarBuzzer(slice, chan, 125000000 / 3500, 8.0f);
            wrap = 0;
            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        // Sinal para caso o sistema esteja cheio e haja a tentativa de um novo acrescimo dos usuarios
        if (botaoPressionado && numeroDeUsuarios == 10)
        {
            pwm_set_clkdiv(slice, 10.0f);
            pwm_set_chan_level(slice, chan, 0);
            wrap = 125000000 / 2500;
            pwm_set_chan_level(slice, chan, wrap / 2);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, wrap / 2);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, 0);
            vTaskDelay(pdMS_TO_TICKS(1250));
            wrap = 0;

            botaoPressionado = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

uint32_t matrix_rgb(double r, double g, double b)
{
    unsigned char R = r * 255;
    unsigned char G = g * 255;
    unsigned char B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Envia padrão para LEDs WS2812
void desenho_pio(double *desenho, PIO pio, uint sm, float r, float g, float b)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        pio_sm_put_blocking(pio, sm, matrix_rgb(desenho[24 - i] * r, desenho[24 - i] * g, desenho[24 - i] * b));
    }
}

/*
 * Tarefa para a matriz de LEDs. Exibe um padrao de reinicio caso o botao do joystick (gpio 22) seja
 * pressionado. Tambem emite um alerta caso o sistema esteja com 9 ou 10 usuarios.
 */
void vTaskMatriz()
{
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    uint sm = pio_claim_unused_sm(pio, true);
    blink_program_init(pio, sm, offset, WS2812_PIN);

    // Vetor para numero de usuarios == 9 ou numero de usuarios == 10
    double *feedback[3] = {alerta, erro, matrizDesligada};

    // Vetor para fazer a animacao de reinicio do sistema. Todos eles estao desenhados em lib/numeros.c
    double *animacaoReinicio[15] = {reinicio1, reinicio2, reinicio3, reinicio4, reinicio5, reinicio6,
                                    reinicio7, reinicio8, reinicio9, reinicio10, reinicio11, reinicio12,
                                    reinicio13, reinicio14, matrizDesligada};

    while (true)
    {
        // Animacao de reinicio, botao do joystick pressionado
        if (ultimoBotaoPressionado == BUTTON_JOY && botaoPressionado)
        {
            for (int i = 0; i < 15; i++)
            {
                desenho_pio(animacaoReinicio[i], pio, sm, 0.0, 1.0, 0.0);
                vTaskDelay(pdMS_TO_TICKS(84));
            }
        }
        else if (ultimoBotaoPressionado == BUTTON_A && botaoPressionado)
        {
            if (numeroDeUsuarios == 8)
            {
                desenho_pio(feedback[0], pio, sm, 1.0, 1.0, 0.0);
                vTaskDelay(pdMS_TO_TICKS(625));
                desenho_pio(feedback[1], pio, sm, 1.0, 1.0, 0.0);
                vTaskDelay(pdMS_TO_TICKS(625));
            }
            else if (numeroDeUsuarios == 10)
            {
                desenho_pio(feedback[2], pio, sm, 1.0, 0.0, 0.0);
                vTaskDelay(pdMS_TO_TICKS(400));
            }
        }
        desenho_pio(feedback[3], pio, sm, 0.0, 0.0, 0.0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/*
 * Tarefa para alternar entre os LEDs RGB com PWM. Varia junto com o numero de usuarios. Ate 8 usuarios
 * o LED azul esta aceso. Com 8 usuarios, e LED azul apaga e o LED verde acende. Com 9, exibe a cor amarela.
 * Com 10, exibe o vermelho.
 */
void vTaskRGB()
{
    while (true)
    {
        // LED azul aceso
        if (0 <= numeroDeUsuarios && numeroDeUsuarios < 8)
        {
            pwm_set_gpio_level(LED_PIN_GREEN, 0);
            pwm_set_gpio_level(LED_PIN_RED, 0);
            pwm_set_gpio_level(LED_PIN_BLUE, 255);
        }
        // LED verde aceso
        else if (numeroDeUsuarios == 8)
        {
            pwm_set_gpio_level(LED_PIN_RED, 0);
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 255);
        }
        // LED amarelo aceso (verde e vermelho)
        else if (numeroDeUsuarios == 9)
        {
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 255);
            pwm_set_gpio_level(LED_PIN_RED, 255);
        }
        // LED vermelho aceso
        else if (numeroDeUsuarios == 10)
        {
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 0);
            pwm_set_gpio_level(LED_PIN_RED, 255);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/*
 * Funcao usada pelo vTaskDisplay para desenhar um usuario. Desenha um usuario no display OLED, como o
 * desenho dos usuarios apresentam um padrao, a funcao requere apenas as coordenadas X e Y para fazer o
 * desenho.
 */
void desenharUsuario(int xStart, int yStart, bool cor)
{
    int xEnd = xStart + 10;

    ssd1306_line(&ssd, xStart, yStart, xEnd, yStart, cor);
    ssd1306_line(&ssd, xStart, yStart - 1, xEnd, yStart - 1, cor);
    ssd1306_line(&ssd, xStart, yStart - 2, xEnd, yStart - 2, cor);
    ssd1306_line(&ssd, xStart + 1, yStart - 3, xEnd - 1, yStart - 3, cor);
    ssd1306_line(&ssd, xStart + 2, yStart - 4, xEnd - 2, yStart - 4, cor);
    ssd1306_line(&ssd, xStart + 4, yStart - 5, xEnd - 4, yStart - 5, cor);
    ssd1306_line(&ssd, xStart + 4, yStart - 6, xEnd - 4, yStart - 6, cor);
    ssd1306_line(&ssd, xStart + 3, yStart - 7, xEnd - 3, yStart - 7, cor);
    ssd1306_line(&ssd, xStart + 2, yStart - 8, xEnd - 2, yStart - 8, cor);
    ssd1306_line(&ssd, xStart + 2, yStart - 9, xEnd - 2, yStart - 9, cor);
    ssd1306_line(&ssd, xStart + 2, yStart - 10, xEnd - 2, yStart - 10, cor);
    ssd1306_line(&ssd, xStart + 2, yStart - 11, xEnd - 2, yStart - 11, cor);
    ssd1306_line(&ssd, xStart + 3, yStart - 12, xEnd - 3, yStart - 12, cor);
}

/*
 * Tarefa para desenhos no display OLED. Primeiramente verifica se um botao foi pressionado, se sim,
 * aguarda pelo mutex para fazer a atualizacao do numero de usuarios que aparecem no display. Se nenhum
 * botao foi pressionado nos ultimos 1.25 segundps, exibe "Usuarios: x" na tela.
 */
void vTaskDisplay()
{
    // Configuracao do I2C para o display
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    static char buffer[13];
    bool cor = true;
    while (true)
    {
        // Verifica se o botao foi pressionado
        if (!botaoPressionado)
        {
            // Verifica se o mutex esta liberado
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                ssd1306_fill(&ssd, !cor);

                sprintf(buffer, "Usuarios: %d", numeroDeUsuarios);
                ssd1306_draw_string(&ssd, buffer, 5, 5);

                // Desenha usuarios com base na quantidade
                switch (numeroDeUsuarios)
                {
                case 10:
                    desenharUsuario(82, 57, cor);
                case 9:
                    desenharUsuario(64, 57, cor);
                case 8:
                    desenharUsuario(46, 57, cor);
                case 7:
                    desenharUsuario(28, 57, cor);
                case 6:
                    desenharUsuario(10, 57, cor);
                case 5:
                    desenharUsuario(82, 34, cor);
                case 4:
                    desenharUsuario(64, 34, cor);
                case 3:
                    desenharUsuario(46, 34, cor);
                case 2:
                    desenharUsuario(28, 34, cor);
                case 1:
                    desenharUsuario(10, 34, cor);
                }
                ssd1306_rect(&ssd, 17, 4, 94, 45, cor, !cor);
                ssd1306_send_data(&ssd);

                xSemaphoreGive(xMutex);
            }
        }
    }
}

// Funcao de callback para o botao de reset (botao do joystick)
void gpio_callback_reset(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Funcao de callback para o botao A e botao B
void gpio_callback(uint gpio, uint32_t events)
{
    ultimoBotaoPressionado = gpio;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xCounterSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Interrupcao para os tres botoes com debounce
void gpio_irq_handler(uint gpio, uint32_t events)
{
    // Se outro botao ja foi pressionado, retorna
    if (botaoPressionado)
        return;

    // Debounce de 1.25 segundos
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - lastIrqTime < DEBOUNCE_MS)
        return;
    lastIrqTime = now;

    if (gpio == BUTTON_A)
    {
        botaoPressionado = true;

        // Verifica se o limite ja foi atingido
        if (numeroDeUsuarios == 10)
        {
            limiteAtingido = true;
        }
        else
        {
            ultimoBotaoPressionado = BUTTON_A;
            gpio_callback(gpio, events);
        }
    }
    else if (gpio == BUTTON_B)
    {
        botaoPressionado = true;
        ultimoBotaoPressionado = BUTTON_B;
        gpio_callback(gpio, events);
    }
    else if (gpio == BUTTON_JOY)
    {
        botaoPressionado = true;
        ultimoBotaoPressionado = BUTTON_JOY;
        gpio_callback_reset(gpio, events);
    }
}

int main()
{
    stdio_init_all();

    // Inicializacao dos LEDs
    gpio_init(LED_PIN_GREEN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    // Inicializando o PWM para o LED verde, vermelho, azul e para o buzzer
    gpio_set_function(LED_PIN_GREEN, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN_BLUE, GPIO_FUNC_PWM);
    gpio_set_function(LED_PIN_RED, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_led_red = pwm_gpio_to_slice_num(LED_PIN_RED);
    uint slice_led_blue = pwm_gpio_to_slice_num(LED_PIN_BLUE);
    uint slice_led_green = pwm_gpio_to_slice_num(LED_PIN_GREEN);
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_clkdiv(slice_led_green, 500.0f);
    pwm_set_wrap(slice_led_green, 255);
    pwm_set_clkdiv(slice_led_blue, 500.0f);
    pwm_set_wrap(slice_led_blue, 255);
    pwm_set_clkdiv(slice_led_red, 500.0f);
    pwm_set_wrap(slice_led_red, 255);
    pwm_set_clkdiv(slice_buzzer, 10.0f);
    pwm_set_enabled(slice_led_green, true);
    pwm_set_enabled(slice_led_blue, true);
    pwm_set_enabled(slice_led_red, true);
    pwm_set_enabled(slice_buzzer, true);

    // Inicializacao e pull-up dos botoes (A, B e joystick)
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_init(BUTTON_JOY);
    gpio_set_dir(BUTTON_JOY, GPIO_IN);
    gpio_pull_up(BUTTON_JOY);

    // Contagem do semaforo contador, ate 10 usuarios
    int MAX = 10;

    // Criacao do mutex, semaforo binario e semaforo contador (ate 10 usuarios)
    xMutex = xSemaphoreCreateMutex();
    xBinarySem = xSemaphoreCreateBinary();
    xCounterSem = xSemaphoreCreateCounting(MAX, 0);

    // Interrupcao para os botoes
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_JOY, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Criacao das tarefas
    xTaskCreate(vTaskEntrada, "Tarefa para entrada do usuario", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskSaida, "Tarefa para saida do usuario", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskReset, "Tarefa para reset do sistema", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskMatriz, "Tarefa para a matriz de LEDs", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskRGB, "Tarefa dos LEDs", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskDisplay, "Tarefa do display", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskBuzzer, "Tarefa do buzzer", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}