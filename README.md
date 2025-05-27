# Controle de acessos com a Raspberry Pi Pico W
**EMBARCATECH - Fase 2**

**Sistema de controle de acessos usando FreeRTOS e a BitDogLab**

## Desenvolvedor
- **Carlos Henrique Silva Lopes**

---

### 📄 Descrição

Este é um projeto que simula um sistema de entrada/saída de usuários em uma placa **BitDogLab (RP2040)**, utilizando o **FreeRTOS** para gerenciar tarefas concorrentes. 
O sistema opera com feedback sonoro para acessibilidade de pessoas com deficiência visual.

---

### 🎯 Objetivo Geral

Utilizar tarefas do **FreeRTOS** para controlar LEDs, botões, buzzer, matriz WS2812 e display SSD1306.

---

### ⚙️ Funcionalidades

* **Controle de LEDs RGB:** Azul (de 0 a 7 usuários), verde para 8 usuários, amarelo para 9 usuários e vermelho para 10 usuários.
* **Matriz WS2812:** Exibe feedback visual caso o sistema esteja com 9 (!) ou 10 (X) usuários.
* **Display SSD1306:** Desenha um retêngulo com figuras representando a quantidade de pessoas no sistema, além de mostrar o número total em uma mensagem.
* **Buzzer *(PWM)*:** Sons distintos para cada cor dos LEDs.
* **Botão A/Botão B:** Acréscimo/decréscimo de usuários no sistema.

---

### 📌 Mapeamento de Pinos

| Função             | GPIO |
| ------------------ | ---- |
| WS2812 (Matriz)    | 7    |
| Botão A            | 5    |
| Botão B            | 6    |
| Botão joystick     | 22   |
| LED Verde (PWM)    | 11   |
| LED Azul (PWM)     | 12   |
| LED Vermelho (PWM) | 13   |
| Buzzer (PWM)       | 21   |

---

## Estrutura do Código

### Principais Arquivos
- **`ControleDeAcessos.c`**: Contém a lógica principal do programa, com todas as tarefas.
- **`lib/`**: Contém os arquivos com a lógica principal para desenhar no display ssd1306, também contém os arquivos com os desenhos da matriz
    de LED e para configuração do FreeRTOS.
- **`lib/numeros.c`**:  Contém os desenhos que serão feitos na matriz de LEDs.
- **`lib/ssd1306.c`**: Contém as funções para desenhar no display ssd1306.
- **`blink.pio`**: Contém a configuração em Assembly para funcionamento do pio.
- **`README.md`**: Documentação detalhada do projeto.
