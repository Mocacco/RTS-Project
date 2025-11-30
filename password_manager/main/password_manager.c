#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_timer.h"

// ==========================================================
// --- CONFIGURAÇÕES DE HARDWARE ---
// ==========================================================
#define SDA_PIN 21
#define SCL_PIN 22
#define LCD_ADDR 0x27
#define I2C_MASTER_NUM I2C_NUM_0

// BOTÃO NO GPIO 0
#define BUTTON_GPIO 0 

// SERVO
#define SERVO_PIN 13       
#define SERVO_FREQ 50      
#define SERVO_RES LEDC_TIMER_13_BIT 
#define SERVO_MIN_PULSE 200  
#define SERVO_MAX_PULSE 1050 

// ==========================================================
// --- CONSTANTES LÓGICAS ---
// ==========================================================
#define DEBOUNCE_MS 50
#define CLICK_WINDOW_MS 400 
#define LONG_PRESS_MS 600 
#define SET_PASSWORD_MS 5000 
#define ATTEMPT_TIMEOUT_MS 60000 
#define INPUT_SUBMIT_MS 2500 

#define SINGLE_CLICK 1
#define DOUBLE_CLICK 2
#define LONG_PRESS 3 

#define MIN_SENHA_LEN 3
#define MAX_SENHA_LEN 14

// --- ESTADOS DO SISTEMA ---
#define MODO_VERIFICAR_ACESSO 0
#define MODO_CONFIRMAR_ANTIGA 1
#define MODO_DEFINIR_NOVA 2

// ==========================================================
// --- DRIVER LCD MANUAL ---
// ==========================================================
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | 0x08; 
    uint8_t data_packet[2] = {data | 0x04, data};
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data_packet, 2, 1000 / portTICK_PERIOD_MS);
}

void lcd_send_byte(uint8_t val, uint8_t mode) {
    lcd_send_nibble(val & 0xF0, mode);
    lcd_send_nibble((val << 4) & 0xF0, mode);
}

void lcd_cmd(uint8_t cmd) { lcd_send_byte(cmd, 0); }
void lcd_char(char data)  { lcd_send_byte(data, 1); }

void lcd_clear() {
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(20)); 
}

void lcd_set_cursor(int row, int col) {
    uint8_t addr = (row == 0) ? 0x80 : 0xC0;
    lcd_cmd(addr + col);
}

void lcd_print(char *str) {
    while (*str) lcd_char(*str++);
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_nibble(0x30, 0); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_nibble(0x20, 0); vTaskDelay(pdMS_TO_TICKS(10));
    lcd_cmd(0x28); vTaskDelay(pdMS_TO_TICKS(1)); 
    lcd_cmd(0x0C); vTaskDelay(pdMS_TO_TICKS(1)); 
    lcd_cmd(0x06); vTaskDelay(pdMS_TO_TICKS(1)); 
    lcd_clear();
}

// ==========================================================
// --- FUNÇÕES DO SERVO MOTOR ---
// ==========================================================
void servo_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = SERVO_RES,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

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

uint32_t angle_to_duty(int angle) {
    return (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180) + SERVO_MIN_PULSE;
}

void set_servo_angle(int angle) {
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ==========================================================
// --- LÓGICA DO PROJETO ---
// ==========================================================

typedef enum {
  MSG_IDLE, MSG_TYPING, MSG_CHECKING, MSG_SUCCESS, MSG_FAIL, 
  MSG_ASK_OLD_PASS, MSG_OLD_PASS_OK, MSG_OLD_PASS_FAIL, 
  MSG_CONFIG_SAVED, MSG_NO_PASSWORD, MSG_TIMEOUT, MSG_TOO_SHORT
} LcdMsgType;

typedef struct { LcdMsgType type; int value; } DisplayMessage;

QueueHandle_t xQueueDisplay;
SemaphoreHandle_t xSemServoTrigger; 
TaskHandle_t xTaskBotaoHandle = NULL;

int senha_definida = 0; 
int tam_senha_correta = 0;
int senha_correta[MAX_SENHA_LEN] = {0};
int senha_digitada[MAX_SENHA_LEN] = {0};
int indice_senha = 0;
int modo_atual = MODO_VERIFICAR_ACESSO; 
int64_t attempt_start_time = 0; 
int64_t last_click_time = 0;    

int64_t millis() { return esp_timer_get_time() / 1000; }

void IRAM_ATTR gpio_isr_handler(void* arg) {
    vTaskNotifyGiveFromISR(xTaskBotaoHandle, NULL);
}

void send_lcd_msg(LcdMsgType type, int value) {
    DisplayMessage msg = { .type = type, .value = value };
    xQueueSend(xQueueDisplay, &msg, 0);
}

int validar_digitacao() {
    if (indice_senha != tam_senha_correta) return 0;
    for (int i = 0; i < tam_senha_correta; i++) {
        if (senha_digitada[i] != senha_correta[i]) return 0;
    }
    return 1;
}

void resetar_entrada() {
    indice_senha = 0;
    attempt_start_time = 0;
    last_click_time = 0;
    for(int i=0; i<MAX_SENHA_LEN; i++) senha_digitada[i] = 0;
}

void finalizar_entrada() {
    // 1. CHECAGEM DE TAMANHO MÍNIMO
    if (indice_senha < MIN_SENHA_LEN) {
        send_lcd_msg(MSG_TOO_SHORT, 0);
        resetar_entrada();
        
        // --- CORREÇÃO DO BUG AQUI ---
        // Se a senha for curta demais, cancelamos a operação e voltamos para o INÍCIO.
        // Isso impede que o display mostre "Insira Senha" mas a lógica continue em "Definir Nova".
        modo_atual = MODO_VERIFICAR_ACESSO; 
        
        return;
    }

    // 2. LÓGICA DE ESTADOS
    if (modo_atual == MODO_VERIFICAR_ACESSO) {
        send_lcd_msg(MSG_CHECKING, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        
        if (validar_digitacao()) {
            send_lcd_msg(MSG_SUCCESS, 0);
            xSemaphoreGive(xSemServoTrigger);
        } else {
            send_lcd_msg(MSG_FAIL, 0);
        }
        modo_atual = MODO_VERIFICAR_ACESSO;
    }
    else if (modo_atual == MODO_CONFIRMAR_ANTIGA) {
         send_lcd_msg(MSG_CHECKING, 0);
         vTaskDelay(pdMS_TO_TICKS(500));
         if (validar_digitacao()) {
             modo_atual = MODO_DEFINIR_NOVA;
             send_lcd_msg(MSG_OLD_PASS_OK, 0);
         } else {
             modo_atual = MODO_VERIFICAR_ACESSO;
             send_lcd_msg(MSG_OLD_PASS_FAIL, 0);
         }
    }
    else if (modo_atual == MODO_DEFINIR_NOVA) {
        tam_senha_correta = indice_senha;
        for(int i=0; i<tam_senha_correta; i++) senha_correta[i] = senha_digitada[i];
        
        senha_definida = 1; 
        send_lcd_msg(MSG_CONFIG_SAVED, 0);
        modo_atual = MODO_VERIFICAR_ACESSO;
    }

    resetar_entrada();
}

void processar_clique(int tipo_clique) {
    if (modo_atual == MODO_VERIFICAR_ACESSO && senha_definida == 0) {
        send_lcd_msg(MSG_NO_PASSWORD, 0);
        return; 
    }

    if (indice_senha == 0) {
        attempt_start_time = millis();
    }
    
    last_click_time = millis(); 

    if (indice_senha < MAX_SENHA_LEN) {
        senha_digitada[indice_senha] = tipo_clique;
        indice_senha++;
        send_lcd_msg(MSG_TYPING, indice_senha);
    }
    
    if (indice_senha >= MAX_SENHA_LEN) {
        finalizar_entrada();
    }
}

// --- TAREFA: Servo Motor ---
void task_servo(void *pvParameters) {
    servo_init();
    set_servo_angle(0); 

    while(1) {
        if(xSemaphoreTake(xSemServoTrigger, portMAX_DELAY) == pdTRUE) {
            printf("Servo: ABRINDO...\n");
            set_servo_angle(180);
            vTaskDelay(pdMS_TO_TICKS(60000)); 
            printf("Servo: FECHANDO...\n");
            set_servo_angle(0);
        }
    }
}

// --- TAREFA: LCD ---
void task_lcd(void *pvParameters) {
    DisplayMessage msg;
    i2c_master_init();
    lcd_init();
    lcd_set_cursor(0, 0);
    lcd_print("Sistema Pronto"); 

    while (1) {
        if (xQueueReceive(xQueueDisplay, &msg, portMAX_DELAY)) {
            switch (msg.type) {
                case MSG_IDLE:
                    lcd_clear();
                    lcd_set_cursor(0, 0);
                    if (senha_definida) lcd_print("Insira a Senha");
                    else lcd_print("Sem Senha");
                    
                    if (!senha_definida) {
                         lcd_set_cursor(1, 0); lcd_print("Segure 5s Config");
                    }
                    break;
                case MSG_TYPING:
                    lcd_set_cursor(1, 0); lcd_print("[");
                    for(int i=0; i<msg.value; i++) lcd_print("*");
                    for(int i=msg.value; i<MAX_SENHA_LEN; i++) lcd_print(" "); 
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
void task_botao(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1 
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);

    int contador_cliques = 0;
    int64_t last_release_time = 0; 

    while (1) {
        uint32_t notificado = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
        int64_t agora = millis();

        // 1. Timeout Global
        if (indice_senha > 0 && attempt_start_time > 0) {
            if ((agora - attempt_start_time) > ATTEMPT_TIMEOUT_MS) {
                send_lcd_msg(MSG_TIMEOUT, 0);
                resetar_entrada();
                modo_atual = MODO_VERIFICAR_ACESSO; 
            }
        }

        // 2. TIMEOUT DE ENVIO (Digitação Variável)
        if (indice_senha > 0 && last_click_time > 0) {
            if ((agora - last_click_time) > INPUT_SUBMIT_MS) {
                finalizar_entrada();
            }
        }

        if (contador_cliques > 0 && last_release_time != 0 && (agora - last_release_time > CLICK_WINDOW_MS)) {
            if(gpio_get_level(BUTTON_GPIO) == 1) { 
                processar_clique(contador_cliques);
                contador_cliques = 0; last_release_time = 0;
            }
        }

        if (notificado > 0) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_MS));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                int64_t press_start = millis();
                bool config_ativado = false;
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    if (!config_ativado && (millis() - press_start) >= SET_PASSWORD_MS) {
                        if (modo_atual == MODO_VERIFICAR_ACESSO && indice_senha == 0) {
                            if (senha_definida == 1) {
                                modo_atual = MODO_CONFIRMAR_ANTIGA;
                                send_lcd_msg(MSG_ASK_OLD_PASS, 0);
                            } else {
                                modo_atual = MODO_DEFINIR_NOVA;
                                send_lcd_msg(MSG_OLD_PASS_OK, 0);
                            }
                            resetar_entrada();
                            config_ativado = true;
                        }
                    }
                }
                if (!config_ativado) {
                    last_click_time = millis();
                    if ((millis() - press_start) >= LONG_PRESS_MS) {
                        if (contador_cliques > 0) { processar_clique(contador_cliques); contador_cliques = 0; }
                        processar_clique(LONG_PRESS);
                    } else {
                        contador_cliques++; last_release_time = millis();
                    }
                }
            }
            ulTaskNotifyTake(pdTRUE, 0);
        }
    }
}

void app_main(void) {
    xQueueDisplay = xQueueCreate(5, sizeof(DisplayMessage));
    xSemServoTrigger = xSemaphoreCreateBinary(); 

    xTaskCreate(task_lcd, "Task_LCD", 4096, NULL, 5, NULL);
    xTaskCreate(task_botao, "Task_Botao", 4096, NULL, 5, &xTaskBotaoHandle);
    xTaskCreate(task_servo, "Task_Servo", 4096, NULL, 5, NULL); 
}