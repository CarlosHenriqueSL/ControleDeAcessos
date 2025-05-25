/*
 *  Por: Carlos Henrique Silva Lopes
 *  Data: 21-05-2025
 *
 *  Exemplo de uso de semaforos com FreeRTOS
 *
 *  Descrição: Simulação de um semáforo com um botão.
 *  O botão A aciona o semáforo e o botão B reinicia o programa.
 *  O semáforo é representado por um display OLED SSD1306.
 *  O botão A é conectado ao pino 5. Quando pressionado, o semáforo muda de estado.
 *  Escreve no display "Aguardando evento..." e, quando o botão A é pressionado,
 *  muda para "Botão Pressionado!" por 4 segundos.
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

static uint32_t last_irq_time_joy = 0;

// Flag para alternar o modo. True = modo normal, false = modo noturno.
volatile bool modoNormalOn = true;
volatile bool limiteAtingido = false;
volatile bool botaoPressionado = false;
volatile uint8_t numeroDeUsuarios = 0;
volatile uint8_t ultimoBotaoPressionado = 0;

void vTaskReset()
{
    static char buffer[13];
    while (true)
    {
        // Tarefa está bloqueada até que o semáforo seja liberado
        // O semáforo é liberado na ISR do botão
        if (xSemaphoreTake(xBinarySem, portMAX_DELAY) == pdTRUE)
        {
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

void vTaskSaida()
{
    while (true)
    {
        if (xSemaphoreTake(xCounterSem, portMAX_DELAY) == pdTRUE)
        {
            if (ultimoBotaoPressionado == BUTTON_B && numeroDeUsuarios != 0)
            {
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

void vTaskEntrada()
{
    while (true)
    {
        if (xSemaphoreTake(xCounterSem, portMAX_DELAY) == pdTRUE)
        {
            if (ultimoBotaoPressionado == BUTTON_A && numeroDeUsuarios < 10)
            {
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

void vTaskBuzzer()
{
    uint slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_enabled(slice, true);
    uint chan = pwm_gpio_to_channel(BUZZER_PIN);
    uint wrap = 125000000 / 3500;
    pwm_set_wrap(slice, wrap);

    while (true)
    {
        if (botaoPressionado && (0 <= numeroDeUsuarios && numeroDeUsuarios < 8))
        {
            pwm_set_clkdiv(slice, 2.0f);
            pwm_set_chan_level(slice, chan, 0);
            wrap = 125000000 / 3500;
            pwm_set_chan_level(slice, chan, wrap / 2);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, 0);
            wrap = 0;

            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        else if (botaoPressionado && numeroDeUsuarios == 8)
        {
            pwm_set_clkdiv(slice, 4.0f);
            pwm_set_chan_level(slice, chan, 0);
            wrap = 125000000 / 3500;
            pwm_set_chan_level(slice, chan, wrap / 2);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, 0);
            wrap = 0;

            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        else if (botaoPressionado && numeroDeUsuarios == 9)
        {
            pwm_set_clkdiv(slice, 8.0f);
            pwm_set_chan_level(slice, chan, 0);
            wrap = 125000000 / 3500;
            pwm_set_chan_level(slice, chan, wrap / 2);
            vTaskDelay(pdMS_TO_TICKS(100));
            pwm_set_chan_level(slice, chan, 0);
            wrap = 0;

            vTaskDelay(pdMS_TO_TICKS(1250));
            botaoPressionado = false;
        }
        if (limiteAtingido)
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

            limiteAtingido = false;
            botaoPressionado = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Tarefa para alternar entre os LEDs RGB com PWM */
void vTaskRGB()
{
    while (true)
    {
        if (0 <= numeroDeUsuarios && numeroDeUsuarios < 8)
        {
            pwm_set_gpio_level(LED_PIN_GREEN, 0);
            pwm_set_gpio_level(LED_PIN_RED, 0);
            pwm_set_gpio_level(LED_PIN_BLUE, 255);
        }
        else if (numeroDeUsuarios == 8)
        {
            pwm_set_gpio_level(LED_PIN_RED, 0);
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 255);
        }
        else if (numeroDeUsuarios == 9)
        {
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 255);
            pwm_set_gpio_level(LED_PIN_RED, 255);
        }
        else if (numeroDeUsuarios == 10)
        {
            pwm_set_gpio_level(LED_PIN_BLUE, 0);
            pwm_set_gpio_level(LED_PIN_GREEN, 0);
            pwm_set_gpio_level(LED_PIN_RED, 255);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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
        if (!botaoPressionado)
        {
            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                ssd1306_fill(&ssd, !cor);

                sprintf(buffer, "Usuarios: %d", numeroDeUsuarios);
                ssd1306_draw_string(&ssd, buffer, 5, 5);

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

void gpio_callback_reset(uint gpio, uint32_t events)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xBinarySem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void gpio_callback(uint gpio, uint32_t events)
{
    ultimoBotaoPressionado = gpio;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xCounterSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void gpio_irq_handler(uint gpio, uint32_t events)
{
    if (botaoPressionado)
        return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_irq_time_joy < DEBOUNCE_MS)
        return;
    last_irq_time_joy = now;

    if (gpio == BUTTON_A)
    {
        botaoPressionado = true;

        if (numeroDeUsuarios == 10)
        {
            // chegamos ao limite: só tocamos o buzzer
            limiteAtingido = true;
            // **não** chamar gpio_callback nem dar mais um give no xCounterSem
        }
        else
        {
            // caso normal: libera o counting semaphore para vTaskEntrada
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

    gpio_init(LED_PIN_GREEN);
    gpio_set_dir(LED_PIN_GREEN, GPIO_OUT);
    gpio_init(LED_PIN_BLUE);
    gpio_set_dir(LED_PIN_BLUE, GPIO_OUT);
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);

    // Inicializando o PWM para o LED verde, LED vermelho e para o buzzer.
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
    pwm_set_enabled(slice_led_green, true);
    pwm_set_enabled(slice_led_blue, true);
    pwm_set_enabled(slice_led_red, true);
    pwm_set_enabled(slice_buzzer, true);

    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_init(BUTTON_JOY);
    gpio_set_dir(BUTTON_JOY, GPIO_IN);
    gpio_pull_up(BUTTON_JOY);

    int MAX = 10;

    xMutex = xSemaphoreCreateMutex();
    xBinarySem = xSemaphoreCreateBinary();
    xCounterSem = xSemaphoreCreateCounting(MAX, 0);

    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BUTTON_JOY, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Criacao das tarefas
    xTaskCreate(vTaskEntrada, "Tarefa para entrada do usuario", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskSaida, "Tarefa para entrada do usuario", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskReset, "Tarefa para entrada do usuario", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskRGB, "Tarefa dos LEDs", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskDisplay, "Tarefa dos LEDs", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vTaskBuzzer, "Tarefa dos LEDs", configMINIMAL_STACK_SIZE,
                NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}