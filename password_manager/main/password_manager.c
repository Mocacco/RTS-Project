/*
 * PROJETO: Sistema de Controle de Acesso com Senha (ESP32)
 * HARDWARE:
 * - ESP32 (DevKit V1 / WROOM)
 * - Display LCD 1602 via I2C (Pinos 21 SDA, 22 SCL)
 * - Servo Motor SG90 (Pino 13)
 * - Botão Push (Pino 0 - BOOT)
 *
 * FUNCIONALIDADES:
 * - Senha de tamanho variável (3 a 14 toques).
 * - Diferencia clique Curto (1), Duplo (2) e Longo (3).
 * - Modo de Configuração ao segurar botão por 5s.
 * - Timeout de digitação: Envia senha após 2.5s sem clicar.
 * - Timeout global: Reseta operação após 60s.
 * - Servo abre por 1 minuto ao acertar a senha.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h" // Biblioteca para PWM (Controle do Servo)
#include "esp_timer.h"   // Para contagem de tempo (microssegundos)

// ==========================================================
// --- 1. CONFIGURAÇÕES DE HARDWARE ---
// ==========================================================

// Pinos I2C padrão do ESP32 (Wire)
#define SDA_PIN 21
#define SCL_PIN 22
#define LCD_ADDR 0x27        // Endereço comum de displays I2C (pode ser 0x3F)
#define I2C_MASTER_NUM I2C_NUM_0

// Botão BOOT da placa (GPIO 0). Possui resistor pull-up físico na placa.
#define BUTTON_GPIO 0 

// Configuração do Servo Motor (PWM)
#define SERVO_PIN 13       
#define SERVO_FREQ 50        // 50Hz é a frequência padrão para servos analógicos (SG90)
#define SERVO_RES LEDC_TIMER_13_BIT // Resolução de 13 bits (Valores de 0 a 8191)
// Calibração dos pulsos (Baseado na resolução de 13 bits e freq 50Hz)
#define SERVO_MIN_PULSE 200  // ~0 graus (0.5ms)
#define SERVO_MAX_PULSE 1050 // ~180 graus (2.5ms)

// ==========================================================
// --- 2. CONSTANTES DE LÓGICA E TEMPO ---
// ==========================================================

#define DEBOUNCE_MS 50       // Tempo para ignorar ruído mecânico do botão
#define CLICK_WINDOW_MS 400  // Janela de tempo para detectar duplo clique
#define LONG_PRESS_MS 600    // Tempo segurando para considerar "Clique Longo"
#define SET_PASSWORD_MS 5000 // Tempo segurando para entrar no "Modo Configuração"
#define ATTEMPT_TIMEOUT_MS 60000 // Tempo máximo de uma sessão (1 min) antes de resetar
#define INPUT_SUBMIT_MS 2500 // Tempo sem digitar para confirmar a senha automaticamente

// Valores que representam os tipos de clique na senha
#define SINGLE_CLICK 1
#define DOUBLE_CLICK 2
#define LONG_PRESS 3 

// Limites de tamanho da senha
#define MIN_SENHA_LEN 3
#define MAX_SENHA_LEN 14

// Estados da Máquina de Estados Finita (FSM)
#define MODO_VERIFICAR_ACESSO 0 // Estado normal (Porta trancada)
#define MODO_CONFIRMAR_ANTIGA 1 // Pedindo senha atual para permitir troca
#define MODO_DEFINIR_NOVA 2     // Gravando a nova senha

// ==========================================================
// --- 3. DRIVER LCD (IMPLEMENTAÇÃO MANUAL I2C) ---
// Como não usamos biblioteca Arduino, implementamos o protocolo HD44780 aqui.
// ==========================================================

// Inicializa o barramento I2C do ESP32
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Ativa resistores internos (segurança)
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,          // 100kHz (Velocidade padrão I2C)
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Envia meio byte (4 bits) para o LCD (Modo 4-bit)
void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | 0x08; // 0x08 mantem o Backlight ACESO
    uint8_t data_packet[2] = {data | 0x04, data}; // Pulso no pino Enable (High -> Low)
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_packet, 2, 1000 / portTICK_PERIOD_MS);
}

// Envia um byte completo (divido em dois nibbles)
void lcd_send_byte(uint8_t val, uint8_t mode) {
    lcd_send_nibble(val & 0xF0, mode);        // Envia 4 bits mais significativos
    lcd_send_nibble((val << 4) & 0xF0, mode); // Envia 4 bits menos significativos
}

void lcd_cmd(uint8_t cmd) { lcd_send_byte(cmd, 0); } // RS=0 para Comandos
void lcd_char(char data)  { lcd_send_byte(data, 1); } // RS=1 para Caracteres

// Limpa a tela (comando lento, precisa de delay maior)
void lcd_clear() {
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(20)); 
}

// Posiciona o cursor (Linha 0 ou 1, Coluna 0-15)
void lcd_set_cursor(int row, int col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0; // Endereços da RAM do LCD
    lcd_cmd(addr + col);
}

// Imprime string
void lcd_print(char *str) {
    while (*str) lcd_char(*str++);
}

// Sequência mágica de inicialização do LCD (datasheet HD44780)
void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(100)); // Espera tensão estabilizar
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(10)); // Muda para modo 4 bits
    lcd_cmd(0x28); vTaskDelay(pdMS_TO_TICKS(1)); // Configura linhas e fonte
    lcd_cmd(0x0C); vTaskDelay(pdMS_TO_TICKS(1)); // Display ON, Cursor OFF
    lcd_cmd(0x06); vTaskDelay(pdMS_TO_TICKS(1)); // Direção de escrita
    lcd_clear();
}

// ==========================================================
// --- 4. DRIVER SERVO MOTOR (PWM) ---
// ==========================================================
void servo_init() {
    // Configura o Timer (Relógio base do PWM)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configura o Canal (Associa o Timer ao Pino)
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

// Converte graus (0-180) para o valor digital do PWM (Duty Cycle)
uint32_t angle_to_duty(int angle) {
    // Regra de três mapeando 0-180 graus para pulsos min-max
    return (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180) + SERVO_MIN_PULSE;
}

void set_servo_angle(int angle) {
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty); // Prepara valor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);    // Aplica valor
}

// ==========================================================
// --- 5. ESTRUTURAS E VARIÁVEIS GLOBAIS ---
// ==========================================================

// Tipos de mensagens que podem ser enviadas para a Task do LCD
typedef enum {
  MSG_IDLE, MSG_TYPING, MSG_CHECKING, MSG_SUCCESS, MSG_FAIL, 
  MSG_ASK_OLD_PASS, MSG_OLD_PASS_OK, MSG_OLD_PASS_FAIL, 
  MSG_CONFIG_SAVED, MSG_NO_PASSWORD, MSG_TIMEOUT, MSG_TOO_SHORT
} LcdMsgType;

// Estrutura da mensagem (Tipo + Valor opcional, ex: quantos asteriscos mostrar)
typedef struct { LcdMsgType type; int value; } DisplayMessage;

// Handles do FreeRTOS (ponteiros para controlar tarefas/filas)
QueueHandle_t xQueueDisplay;        // Fila de mensagens para o LCD
SemaphoreHandle_t xSemServoTrigger; // Semáforo para disparar o Servo
TaskHandle_t xTaskBotaoHandle = NULL; // Handle da tarefa do botão (para notificação)

// Variáveis de Estado do Sistema
int senha_definida = 0;         // 0 = Fábrica (Sem senha), 1 = Senha Criada
int tam_senha_correta = 0;      // Guarda o tamanho da senha atual (3 a 14)
int senha_correta[MAX_SENHA_LEN] = {0}; // Armazena a senha real
int senha_digitada[MAX_SENHA_LEN] = {0}; // Buffer do que o usuário está digitando
int indice_senha = 0;           // Posição atual do cursor de digitação
int modo_atual = MODO_VERIFICAR_ACESSO; // Estado da máquina

// Timers para lógica de timeout
int64_t attempt_start_time = 0; // Início da interação (para timeout de 60s)
int64_t last_click_time = 0;    // Último clique (para timeout de 2.5s)

// Função auxiliar para pegar tempo em milissegundos
int64_t millis() { return esp_timer_get_time() / 1000; }

// ==========================================================
// --- 6. FUNÇÕES DE LÓGICA DO SISTEMA ---
// ==========================================================

// Interrupção (ISR): Acorda a tarefa do botão quando houver mudança no pino
void IRAM_ATTR gpio_isr_handler(void* arg) {
    vTaskNotifyGiveFromISR(xTaskBotaoHandle, NULL);
}

// Envia comando para a fila do LCD (Thread-safe)
void send_lcd_msg(LcdMsgType type, int value) {
    // printf("--- [DEBUG LCD] Enviando mensagem tipo: %d valor: %d\n", type, value);
    DisplayMessage msg = { .type = type, .value = value };
    xQueueSend(xQueueDisplay, &msg, 0);
}

// Compara o buffer digitado com a senha salva
int validar_digitacao() {
    printf("--- [DEBUG LOGICA] Validando Senha...\n");
    
    // 1. Tamanhos diferentes? Erro imediato.
    if (indice_senha != tam_senha_correta) return 0;
    
    // 2. Compara caractere por caractere
    for (int i = 0; i < tam_senha_correta; i++) {
        if (senha_digitada[i] != senha_correta[i]) return 0;
    }
    return 1; // Sucesso
}

// Limpa os buffers para uma nova tentativa
void resetar_entrada() {
    printf("--- [DEBUG LOGICA] Resetando buffers.\n");
    indice_senha = 0;
    attempt_start_time = 0;
    last_click_time = 0;
    for(int i=0; i<MAX_SENHA_LEN; i++) senha_digitada[i] = 0;
}

// Função Central: Decide o que fazer quando a senha é submetida (após 2.5s)
void finalizar_entrada() {
    printf("--- [DEBUG LOGICA] Finalizando Entrada. Modo Atual: %d\n", modo_atual);

    // Validação: Senha muito curta (< 3)
    if (indice_senha < MIN_SENHA_LEN) {
        send_lcd_msg(MSG_TOO_SHORT, 0);
        resetar_entrada();
        
        // Se estava criando senha e errou o tamanho, cancela tudo por segurança
        if (modo_atual == MODO_DEFINIR_NOVA) {
            modo_atual = MODO_VERIFICAR_ACESSO; 
        }
        return;
    }

    // MÁQUINA DE ESTADOS:
    if (modo_atual == MODO_VERIFICAR_ACESSO) {
        // Tenta abrir a porta
        send_lcd_msg(MSG_CHECKING, 0);
        vTaskDelay(pdMS_TO_TICKS(500)); // Charme visual
        
        if (validar_digitacao()) {
            send_lcd_msg(MSG_SUCCESS, 0);
            printf("--- [DEBUG LOGICA] Acesso Permitido! Disparando Servo.\n");
            xSemaphoreGive(xSemServoTrigger); // Acorda a task do servo
        } else {
            send_lcd_msg(MSG_FAIL, 0);
        }
    }
    else if (modo_atual == MODO_CONFIRMAR_ANTIGA) {
         // Usuário quer trocar senha, precisa acertar a atual primeiro
         send_lcd_msg(MSG_CHECKING, 0);
         vTaskDelay(pdMS_TO_TICKS(500));
         if (validar_digitacao()) {
             modo_atual = MODO_DEFINIR_NOVA; // Autorizado
             send_lcd_msg(MSG_OLD_PASS_OK, 0);
         } else {
             modo_atual = MODO_VERIFICAR_ACESSO; // Negado
             send_lcd_msg(MSG_OLD_PASS_FAIL, 0);
         }
    }
    else if (modo_atual == MODO_DEFINIR_NOVA) {
        // Salva a nova senha na memória RAM
        printf("--- [DEBUG LOGICA] Salvando NOVA SENHA de tamanho %d.\n", indice_senha);
        tam_senha_correta = indice_senha;
        for(int i=0; i<tam_senha_correta; i++) senha_correta[i] = senha_digitada[i];
        
        senha_definida = 1; 
        send_lcd_msg(MSG_CONFIG_SAVED, 0);
        modo_atual = MODO_VERIFICAR_ACESSO; // Volta ao normal
    }

    resetar_entrada();
}

// Adiciona um clique ao buffer
void processar_clique(int tipo_clique) {
    // Bloqueia uso se não houver senha definida (exceto se for config)
    if (modo_atual == MODO_VERIFICAR_ACESSO && senha_definida == 0) {
        send_lcd_msg(MSG_NO_PASSWORD, 0);
        return; 
    }

    // Inicia cronômetro global no primeiro toque
    if (indice_senha == 0) attempt_start_time = millis();
    
    last_click_time = millis(); // Reseta o timeout de envio (2.5s)

    // Adiciona ao buffer se não estiver cheio
    if (indice_senha < MAX_SENHA_LEN) {
        senha_digitada[indice_senha] = tipo_clique;
        indice_senha++;
        send_lcd_msg(MSG_TYPING, indice_senha);
    }
    
    // Se encher o buffer (14 digitos), envia automaticamente
    if (indice_senha >= MAX_SENHA_LEN) {
        finalizar_entrada();
    }
}

// ==========================================================
// --- 7. TAREFAS (THREADS) ---
// ==========================================================

// --- TAREFA: Servo Motor ---
// Fica dormindo até receber o semáforo. Abre, espera 1min, fecha.
void task_servo(void *pvParameters) {
    servo_init();
    set_servo_angle(0); // Garante fechado ao iniciar

    while(1) {
        // Espera infinita pelo semáforo (sinal de "Senha Correta")
        if(xSemaphoreTake(xSemServoTrigger, portMAX_DELAY) == pdTRUE) {
            printf(">>> [DEBUG SERVO] Gatilho recebido! Abrindo porta...\n");
            set_servo_angle(180);
            
            printf(">>> [DEBUG SERVO] Aguardando 60 segundos...\n");
            vTaskDelay(pdMS_TO_TICKS(60000)); // Delay não bloqueia outras tasks
            
            printf(">>> [DEBUG SERVO] Tempo esgotado. Fechando porta...\n");
            set_servo_angle(0);
        }
    }
}

// --- TAREFA: LCD ---
// Consome mensagens da fila e atualiza o display.
// Isso evita que o I2C seja acessado por múltiplas tasks ao mesmo tempo.
void task_lcd(void *pvParameters) {
    DisplayMessage msg;
    i2c_master_init();
    lcd_init();
    lcd_set_cursor(0, 0);
    lcd_print("  Sistema Pronto"); 

    while (1) {
        // Fica bloqueado esperando mensagem na fila
        if (xQueueReceive(xQueueDisplay, &msg, portMAX_DELAY)) {
            switch (msg.type) {
                // AQUI ESTÃO TODAS AS MENSAGENS POSSÍVEIS DO DISPLAY
                case MSG_IDLE:
                    lcd_clear();
                    lcd_set_cursor(0, 0);
                    if (senha_definida) lcd_print("  Insira a Senha");
                    else lcd_print("  Sem Senha");
                    
                    if (!senha_definida) {
                         lcd_set_cursor(1, 0); lcd_print("Segure 5s Config");
                    }
                    break;
                case MSG_TYPING:
                    lcd_set_cursor(1, 0); lcd_print("[");
                    for(int i=0; i<msg.value; i++) lcd_print("*"); // Asteriscos
                    for(int i=msg.value; i<MAX_SENHA_LEN; i++) lcd_print(" "); // Limpa resto
                    lcd_print("]");
                    break;
                case MSG_CHECKING:
                    lcd_clear(); lcd_print("Verificando..."); break;
                case MSG_SUCCESS:
                    lcd_clear(); lcd_print("SENHA CORRETA!");
                    lcd_set_cursor(1, 0); lcd_print("Porta Aberta 1m");
                    vTaskDelay(pdMS_TO_TICKS(3000));
                    send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_FAIL:
                    lcd_clear(); lcd_print("SENHA INCORRETA!");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_TOO_SHORT:
                    lcd_clear(); lcd_print("MUITO CURTA!");
                    lcd_set_cursor(1, 0); lcd_print("Minimo 3 clicks");
                    vTaskDelay(pdMS_TO_TICKS(2000)); send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_ASK_OLD_PASS:
                    lcd_clear(); lcd_print("MODO TROCA SENHA");
                    lcd_set_cursor(1, 0); lcd_print("Confirme a Atual");
                    break;
                case MSG_OLD_PASS_OK:
                    lcd_clear(); lcd_print("Digite a Nova:"); break;
                case MSG_OLD_PASS_FAIL:
                    lcd_clear(); lcd_print("Senha Errada");
                    vTaskDelay(pdMS_TO_TICKS(2000)); send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_CONFIG_SAVED:
                    lcd_clear(); lcd_print("NOVA SENHA");
                    lcd_set_cursor(1, 0); lcd_print("GRAVADA!");
                    vTaskDelay(pdMS_TO_TICKS(2000)); send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_NO_PASSWORD:
                    lcd_clear(); lcd_print("ERRO: Nenhuma");
                    lcd_set_cursor(1, 0); lcd_print("Senha Definida!");
                    vTaskDelay(pdMS_TO_TICKS(2000)); send_lcd_msg(MSG_IDLE, 0);
                    break;
                case MSG_TIMEOUT:
                    lcd_clear(); lcd_print("TIMEOUT!");
                    vTaskDelay(pdMS_TO_TICKS(2000)); send_lcd_msg(MSG_IDLE, 0);
                    break;
            }
        }
    }
}

// --- TAREFA: Botão ---
// Responsável por ler o GPIO, debounce, timeouts e detectar tipos de clique.
void task_botao(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, // Interrupção na borda de descida (apertar)
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1 // Garante nível ALTO quando solto (Pull-up)
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);

    int contador_cliques = 0;
    int64_t last_release_time = 0; 
    
    printf("--- [DEBUG GERAL] Sistema Iniciado. Aguardando input no Botao...\n");

    while (1) {
        // Fica dormindo. Acorda se:
        // 1. Interrupção do botão ocorrer.
        // 2. Timeout de 100ms passar (para checar timers de senha).
        uint32_t notificado = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t agora = millis();

        // --- VERIFICAÇÃO DE TIMERS DE FUNDO ---

        // 1. Timeout Global (60s) - Limpa tudo se demorar demais
        if (indice_senha > 0 && attempt_start_time > 0) {
            if ((agora - attempt_start_time) > ATTEMPT_TIMEOUT_MS) {
                printf("--- [DEBUG TIMEOUT] Sessao expirou (60s). Cancelando.\n");
                send_lcd_msg(MSG_TIMEOUT, 0);
                resetar_entrada();
                modo_atual = MODO_VERIFICAR_ACESSO; 
            }
        }

        // 2. Timeout de Envio (2.5s) - Detecta fim da senha
        if (indice_senha > 0 && last_click_time > 0) {
            if ((agora - last_click_time) > INPUT_SUBMIT_MS) {
                printf("--- [DEBUG TIMEOUT] Fim da digitacao detectado (2.5s sem clique).\n");
                finalizar_entrada();
            }
        }

        // 3. Processa Pacote de Cliques Curtos/Duplos
        // Se soltou o botão e passou a janela de tempo (400ms), confirma o clique
        if (contador_cliques > 0 && last_release_time != 0 && (agora - last_release_time > CLICK_WINDOW_MS)) {
            if(gpio_get_level(BUTTON_GPIO) == 1) { 
                printf(">>> [DEBUG BOTAO] Pacote fechado: %d clique(s) curto(s).\n", contador_cliques);
                processar_clique(contador_cliques);
                contador_cliques = 0; last_release_time = 0;
            }
        }

        // --- PROCESSAMENTO FÍSICO DO BOTÃO (Quando notificado pela ISR) ---
        if (notificado > 0) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS)); // Espera trepidação mecânica passar
            
            if (gpio_get_level(BUTTON_GPIO) == 0) { // Confirma se ainda está apertado
                printf("--- [DEBUG BOTAO] Pressionado (Down)\n");
                int64_t press_start = millis();
                bool config_ativado = false;
                
                // Loop enquanto segura o botão
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    
                    // LÓGICA DE SEGURAR 5 SEGUNDOS (CONFIGURAÇÃO)
                    if (!config_ativado && (millis() - press_start) >= SET_PASSWORD_MS) {
                         printf(">>> [DEBUG BOTAO] Segurou 5 segundos!\n");
                        // Só permite entrar em config se estiver ocioso
                        if (modo_atual == MODO_VERIFICAR_ACESSO && indice_senha == 0) {
                            if (senha_definida == 1) {
                                printf("--- [DEBUG ESTADO] Indo para Confirmar Antiga.\n");
                                modo_atual = MODO_CONFIRMAR_ANTIGA;
                                send_lcd_msg(MSG_ASK_OLD_PASS, 0);
                            } else {
                                printf("--- [DEBUG ESTADO] Indo para Definir Nova (Sem senha previa).\n");
                                modo_atual = MODO_DEFINIR_NOVA;
                                send_lcd_msg(MSG_OLD_PASS_OK, 0);
                            }
                            resetar_entrada();
                            config_ativado = true;
                        } else {
                            printf("--- [DEBUG BOTAO] Ignorado: Ja esta no meio de uma operacao.\n");
                        }
                    }
                }
                
                // Botão Solto
                if (!config_ativado) {
                    int64_t duracao = millis() - press_start;
                    printf("--- [DEBUG BOTAO] Solto. Duracao: %lld ms\n", duracao);
                    
                    last_click_time = millis(); // Reseta timeout de envio
                    
                    // Diferencia Longo vs Curto
                    if (duracao >= LONG_PRESS_MS) {
                        printf(">>> [DEBUG BOTAO] Detectado: LONGO\n");
                        // Se tinha curtos pendentes, envia eles antes
                        if (contador_cliques > 0) { processar_clique(contador_cliques); contador_cliques = 0; }
                        processar_clique(LONG_PRESS);
                    } else {
                        contador_cliques++; // Acumula curtos para detectar duplos
                        printf(">>> [DEBUG BOTAO] Detectado: CURTO. Acumulado: %d\n", contador_cliques);
                        last_release_time = millis();
                    }
                }
            }
            ulTaskNotifyTake(pdTRUE, 0); // Limpa notificações acumuladas durante o processamento
        }
    }
}

// ==========================================================
// --- 8. APP MAIN ---
// ==========================================================
void app_main(void) {
    printf("\n\n=== INICIANDO SISTEMA DE SENHA I2C/SERVO ===\n");
    
    // Cria Fila e Semáforo
    xQueueDisplay = xQueueCreate(5, sizeof(DisplayMessage));
    xSemServoTrigger = xSemaphoreCreateBinary(); 

    // Cria as Tarefas com Stack de 4096 (Seguro para printf e I2C)
    xTaskCreate(task_lcd, "Task_LCD", 4096, NULL, 5, NULL);
    xTaskCreate(task_botao, "Task_Botao", 4096, NULL, 5, &xTaskBotaoHandle);
    xTaskCreate(task_servo, "Task_Servo", 4096, NULL, 5, NULL); 
}