#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

// ________________________________
// PIN
// ________________________________

//touch
#define TOUCH_PIN 25
// Joystick (ADC1)
#define JOY_H_ADC ADC1_CHANNEL_6   // GPIO32
#define JOY_V_ADC ADC1_CHANNEL_7   // GPIO34

// Botão do Joystick
#define JOY_BUTTON 33  // <<< CONFIRME ESSE PINO !!!

// Servos (PWM)
#define SERVO_X_PIN 13
#define SERVO_Y_PIN 12

// LED Status
#define LED_READY 26

// I2C MPU6050
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_PORT I2C_NUM_0

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// ________________________________
// CONFIG SERVOS
// ________________________________
#define SERVO_FREQ_HZ 50
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2400

#define LEDC_RESOLUTION LEDC_TIMER_14_BIT
#define LEDC_MAX_DUTY ((1 << 14) - 1)
#define SERVO_CH_X LEDC_CHANNEL_0
#define SERVO_CH_Y LEDC_CHANNEL_1

// Fator de suavização do movimento dos servos (filtro exponencial)
// quanto menor o valor, mais suave e lento
#define SMOOTH_ALPHA 0.04f

static const char *TAG = "Fase1+MPU";

// ________________________________
// VARIÁVEIS GLOBAIS
// ________________________________

float angleX_target = 90, angleY_target = 90;
float angleX_smooth = 90, angleY_smooth = 90;

int centerH = 2048;
int centerV = 2048;

// Calibração dos servos
float servo_min_X = 0;
float servo_max_X = 180;
float servo_min_Y = 0;
float servo_max_Y = 180;
float offset_X = 10.0f;  // Ajuste de correção no eixo X (se necessário)
float offset_Y = -4.0f;    // Ajuste de correção no eixo Y (se necessário)

// MPU variables
static float pitch_angle = 0.0f;
static float roll_angle  = 0.0f;
// Mutex para proteger o acesso concorrente às variáveis de pitch e roll
static SemaphoreHandle_t pitch_roll_mutex = NULL;

// ==========================================================
// FUNÇÕES AUXILIARES
// ==========================================================

int angle_to_pulse(float ang)
{
    if (ang < 0) ang = 0;
    if (ang > 180) ang = 180;

    return SERVO_MIN_US +
        (int)((SERVO_MAX_US - SERVO_MIN_US) * (ang / 180.0f));
}

int us_to_duty(int us)
{
    int period_us = (1000000 / SERVO_FREQ_HZ);
    return (us * LEDC_MAX_DUTY) / period_us;
}

float adc_to_angle(int raw, int center)
{
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;

    int delta = raw - center;
    float normalized = (float)delta / 2048.0f;

    return 90.0f + (normalized * 90.0f);
}

// ==========================================================
// Função para mover servos manualmente (USADO NA CALIBRAÇÃO)
// ==========================================================

void move_servo_manual(float angX, float angY)
{
    
    // Aplica o offset para corrigir a inclinação
    angX += offset_X;
    angY += offset_Y;

    int dutyX = us_to_duty(angle_to_pulse(angX));
    int dutyY = us_to_duty(angle_to_pulse(angY));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CH_X, dutyX);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CH_X);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_CH_Y, dutyY);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_CH_Y);
}

// ==========================================================
// MPU I2C
// ==========================================================

static esp_err_t i2c_master_init_local(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static void mpu_write(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1), true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

static int16_t mpu_read16(uint8_t reg)
{
    uint8_t h, l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1), true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | 1, true);
    i2c_master_read_byte(cmd, &h, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &l, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_PORT, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ((int16_t)h << 8) | l;
}

static void mpu6050_init_local(void)
{
    mpu_write(MPU_PWR_MGMT_1, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

// ==========================================================
// pitch & roll
// ==========================================================

static void compute_pitch_roll_from_accel(int16_t ax_raw, int16_t ay_raw, int16_t az_raw)
{
    float ax = ax_raw / 16384.0f;
    float ay = ay_raw / 16384.0f;
    float az = az_raw / 16384.0f;

    float pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 57.2958f;
    float roll  = atan2f(-ax, az) * 57.2958f;

    if (xSemaphoreTake(pitch_roll_mutex, pdMS_TO_TICKS(10)))
    {
        pitch_angle = pitch;
        roll_angle = roll;
        xSemaphoreGive(pitch_roll_mutex);
    }
}

// ==========================================================
// CALIBRAÇÃO COMPLETA
// ==========================================================


//função para colocar o led em modo calibração (piscando)
void blink_led_ready()
{
    static int led_state = 0;
    led_state = !led_state;
    gpio_set_level(LED_READY, led_state);
}

void wait_button_press()
{
    while (gpio_get_level(JOY_BUTTON) == 0) vTaskDelay(10);
    while (gpio_get_level(JOY_BUTTON) == 1) vTaskDelay(10);
    while (gpio_get_level(JOY_BUTTON) == 0) vTaskDelay(10);
    vTaskDelay(150 / portTICK_PERIOD_MS);
}

void run_calibration_mode()
{
    printf("\n========== MODO DE CALIBRAÇÃO ==========\n");

    // ------------------ EIXO X (MIN) ------------------
    printf("Mova o joystick para o MIN do eixo X\n");
    while (1)
    {
        blink_led_ready(); // chamando led
        int raw = adc1_get_raw(JOY_H_ADC);
        servo_min_X = adc_to_angle(raw, centerH);

        move_servo_manual(servo_min_X, angleY_smooth);
        // Envia os dados de calibração em formato JSON do eixo X para integração
        // com o script Python e visualização no Grafana
        printf("{\"type\":\"calib\",\"axis\":\"X\",\"limit\":\"MIN\",\"value\":%.1f}\n", servo_min_X);
        fflush(stdout);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        if (gpio_get_level(JOY_BUTTON) == 0) break;
    }
    wait_button_press();

    // ------------------ EIXO X (MAX) ------------------
    printf("Agora mova para o MAX do eixo X\n");
    while (1)
    {
        blink_led_ready();
        int raw = adc1_get_raw(JOY_H_ADC);
        servo_max_X = adc_to_angle(raw, centerH);

        move_servo_manual(servo_max_X, angleY_smooth);
        // Envia os dados de calibração em formato JSON do eixo X para integração
        // com o script Python e visualização no Grafana
        printf("{\"type\":\"calib\",\"axis\":\"X\",\"limit\":\"MAX\",\"value\":%.1f}\n", servo_max_X);
        fflush(stdout);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        if (gpio_get_level(JOY_BUTTON) == 0) break;
    }
    wait_button_press();

    // ------------------ EIXO Y (MIN) ------------------
    printf("Agora mova para o MIN do eixo Y\n");
    while (1)
    {
        blink_led_ready();
        int raw = adc1_get_raw(JOY_V_ADC);
        servo_min_Y = adc_to_angle(raw, centerV);

        move_servo_manual(angleX_smooth, servo_min_Y);
        // Envia os dados de calibração em formato JSON do eixo Y para integração
        // com o script Python e visualização no Grafana
        printf("{\"type\":\"calib\",\"axis\":\"Y\",\"limit\":\"MIN\",\"value\":%.1f}\n", servo_min_Y);
        fflush(stdout);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        if (gpio_get_level(JOY_BUTTON) == 0) break;
    }
    wait_button_press();

    // ------------------ EIXO Y (MAX) ------------------
    printf("Agora mova para o MAX do eixo Y\n");
    while (1)
    {
        blink_led_ready();
        int raw = adc1_get_raw(JOY_V_ADC);
        servo_max_Y = adc_to_angle(raw, centerV);

        move_servo_manual(angleX_smooth, servo_max_Y);
        // Envia os dados de calibração em formato JSON do eixo Y para integração
        // com o script Python e visualização no Grafana
        printf("{\"type\":\"calib\",\"axis\":\"Y\",\"limit\":\"MAX\",\"value\":%.1f}\n", servo_max_Y);
        fflush(stdout);

        vTaskDelay(150 / portTICK_PERIOD_MS);

        if (gpio_get_level(JOY_BUTTON) == 0) break;
    }
    wait_button_press();

    printf("\n===== CALIBRAÇÃO FINALIZADA =====\n\n");
}

// ==========================================================
// TASK — MPU
// ==========================================================

void task_mpu6050(void *pv)
{
    uint8_t base = MPU_ACCEL_XOUT_H;

    while (1)
    {
        int16_t ax = mpu_read16(base + 0);
        int16_t ay = mpu_read16(base + 2);
        int16_t az = mpu_read16(base + 4);

        compute_pitch_roll_from_accel(ax, ay, az);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==========================================================
// TASK — Leitura do Joystick
// ==========================================================

void task_read_joystick(void *pv)
{
    while (1)
    {
        int rawH = adc1_get_raw(JOY_H_ADC);
        int rawV = adc1_get_raw(JOY_V_ADC);

        float angX = adc_to_angle(rawH, centerH);
        float angY = adc_to_angle(rawV, centerV);

        // Proteção contra calibração invertida
        float minX = fminf(servo_min_X, servo_max_X);
        float maxX = fmaxf(servo_min_X, servo_max_X);
        float minY = fminf(servo_min_Y, servo_max_Y);
        float maxY = fmaxf(servo_min_Y, servo_max_Y);

        if (angX < minX) angX = minX;
        if (angX > maxX) angX = maxX;

        if (angY < minY) angY = minY;
        if (angY > maxY) angY = maxY;

        angleX_target = angX;
        angleY_target = angY;

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

// ==========================================================
// TASK — Controle dos Servos
// ==========================================================

void task_servo_control(void *pv)
{
    while (1)
    {
        angleX_smooth = SMOOTH_ALPHA * angleX_target +
                       (1 - SMOOTH_ALPHA) * angleX_smooth;

        angleY_smooth = SMOOTH_ALPHA * angleY_target +
                       (1 - SMOOTH_ALPHA) * angleY_smooth;

        move_servo_manual(angleX_smooth, angleY_smooth);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==========================================================
// TASK — Debug JSON
// ==========================================================

void task_debug(void *pv)
{
    while (1)
    {
        int state = gpio_get_level(TOUCH_PIN);

        // Se TOCOU → não printa MPU
        if (state == 0)
        {
            while (1)
            {
                vTaskDelay(pdMS_TO_TICKS(500));
            }

            // Ao atingir o final do labirinto, a task de debug é bloqueada
            // permanentemente para interromper a transmissão contínua do MPU.
        }
        else{
            // Se NÃO tocou → imprime MPU normalmente
            float p, r;

            if (xSemaphoreTake(pitch_roll_mutex, pdMS_TO_TICKS(10)))
            {
                p = pitch_angle;
                r = roll_angle;
                xSemaphoreGive(pitch_roll_mutex);
            }
            else
            {
                p = pitch_angle;
                r = roll_angle;
            }
            // Envia os dados de orientação em formato JSON para integração
            // com o script Python e visualização no Grafana
            printf("{\"type\":\"mpu\",\"pitch\": %.2f, \"roll\": %.2f}\n", p, r);
            fflush(stdout);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//=============
// Task Touch
//=============

void task_touch_sensor(void *pv)
{
    while (1)
    {
        int state = gpio_get_level(TOUCH_PIN);

        if (!state){
            // Envia os dados de condição de vitória em JSON para refletir no grafana
            printf("{\"type\":\"event\",\"status\":\"WIN\"}\n");
            fflush(stdout);
            gpio_set_level(LED_READY, 0);
            }

        vTaskDelay(pdMS_TO_TICKS(150));
    }
}
// ==========================================================
// HARDWARE SETUP
// ==========================================================

void setup_hardware()
{   
    gpio_set_direction(LED_READY, GPIO_MODE_OUTPUT);

    gpio_set_direction(JOY_BUTTON, GPIO_MODE_INPUT);
    gpio_pullup_en(JOY_BUTTON);

    gpio_set_direction(TOUCH_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(TOUCH_PIN);   

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOY_H_ADC, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOY_V_ADC, ADC_ATTEN_DB_11);

    ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = SERVO_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK});

    ledc_channel_config(&(ledc_channel_config_t){
        .channel    = SERVO_CH_X,
        .gpio_num   = SERVO_X_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0});

    ledc_channel_config(&(ledc_channel_config_t){
        .channel    = SERVO_CH_Y,
        .gpio_num   = SERVO_Y_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0});

    ESP_ERROR_CHECK(i2c_master_init_local());
}

// ==========================================================
// MAIN
// ==========================================================

void app_main(void)
{
    setup_hardware();

    vTaskDelay(pdMS_TO_TICKS(500));
    centerH = adc1_get_raw(JOY_H_ADC);
    centerV = adc1_get_raw(JOY_V_ADC);
    
    pitch_roll_mutex = xSemaphoreCreateMutex();
    
    // ================================
    // CALIBRAÇÃO 
    // ================================

    mpu6050_init_local();
    vTaskDelay(200 / portTICK_PERIOD_MS);  // tempo para estabilizar

    printf("{\"type\":\"event\",\"status\":\"CALIBRATING\"}\n"); // colocando modo de calibração no grafana
    fflush(stdout);

    run_calibration_mode();
    gpio_set_level(LED_READY, 1);
    
    printf("{\"type\":\"event\",\"status\":\"RUNNING\"}\n"); // retirando o modo calibração e inicando o modo jogando
    fflush(stdout);

    // ================================


    xTaskCreate(task_touch_sensor, "Touch", 2048, NULL, 4, NULL);
    xTaskCreate(task_read_joystick, "Joystick", 4096, NULL, 5, NULL);
    xTaskCreate(task_servo_control, "ServoCtrl", 4096, NULL, 6, NULL);
    xTaskCreate(task_mpu6050, "MPU6050", 4096, NULL, 7, NULL);
    xTaskCreate(task_debug, "Debug", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Sistema iniciado");
}
