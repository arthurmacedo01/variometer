#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" // Include for FreeRTOS queue
#include "driver/ledc.h"    // Include for LEDC PWM control
#include <inttypes.h>       // Include this header for PRId32
#include <math.h>           // Include for fabs function
#include "bmx280.h"         // Include for BMX280 sensor
#include "ble_lk8ex1.h"

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< I2C master data pin */
#define BUZZER_GPIO 25              /*!< GPIO number for buzzer */
#define LEDC_CHANNEL LEDC_CHANNEL_0 // Use LEDC channel 0 for PWM

QueueHandle_t xQueue; // Queue to pass x_hat2 to the buzzer task

// Constants for altitude calculation
#define SEA_LEVEL_PRESSURE 101325.0     // Sea-level pressure in Pa
#define TEMPERATURE_AT_SEA_LEVEL 288.15 // Temperature at sea level in K
#define TEMPERATURE_LAPSE_RATE 0.0065   // Temperature lapse rate in K/m
#define DEAD_BAND 0.1                   // Dead band in m/s
#define SENSIBILITY 700                 // Buzzer Sensibility

typedef struct
{
    float pressure;
    float x_hat2;
} SensorData_t;

// Function to convert pressure to altitude
float pressure_to_altitude(float pressure)
{
    return (1 - pow(pressure / SEA_LEVEL_PRESSURE, 0.190284)) * TEMPERATURE_AT_SEA_LEVEL / TEMPERATURE_LAPSE_RATE;
}

// Task to read and filter sensor data using Kalman Filter
void sensor_task(void *pvParameters)
{
    float T = 0.1; // Sampling time (100 ms)
    float Q1 = 1e-3, Q2 = 1e-3, Q3 = 1e-3;
    float R = 0.0406; // Measurement noise covariance
    float P11 = 0.006577, P12 = 0.006836, P13 = 0.002365;
    float P21 = 0.004157, P22 = 0.005179, P23 = 0.004428;
    float P31 = 0.001438, P32 = 0.002636, P33 = 0.002890;

    float x_hat1 = 0; // Estimated current pressure
    float x_hat2 = 0; // Estimated velocity
    float x_hat3 = 0; // Estimated acceleration

    // Initialize the BMX280 sensor
    bmx280_t *bmx280 = bmx280_create(I2C_NUM_0);
    if (!bmx280)
    {
        ESP_LOGE("sensor_task", "Could not create bmx280 driver.");
        return;
    }
    ESP_ERROR_CHECK(bmx280_init(bmx280));

    bmx280_config_t bmx_cfg = BMX280_DEFAULT_CONFIG;
    ESP_ERROR_CHECK(bmx280_configure(bmx280, &bmx_cfg));

    int i = 0;
    while (1)
    {
        ESP_ERROR_CHECK(bmx280_setMode(bmx280, BMX280_MODE_FORCE));
        do
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (bmx280_isSampling(bmx280));

        float temp = 0, pres = 0, hum = 0;
        ESP_ERROR_CHECK(bmx280_readoutFloat(bmx280, &temp, &pres, &hum));

        float current_pressure = pres;
        float current_altitude = pressure_to_altitude(current_pressure);

        if (i < 10)
        {
            x_hat1 = current_altitude;
            i++;
        }
        // Kalman Filter Prediction step
        float x_hat1_pred = x_hat1 + T * x_hat2 + 0.5 * T * T * x_hat3;
        float x_hat2_pred = x_hat2 + T * x_hat3;
        float x_hat3_pred = x_hat3;

        float P11_pred = P11 + T * (P21 + P12) + 0.5 * T * T * (P31 + P13) + T * T * P22 + 0.5 * T * T * T * P32 + Q1;
        float P12_pred = P12 + T * (P22 + P32) + 0.5 * T * T * P23 + T * P22;
        float P13_pred = P13 + T * P23 + 0.5 * T * T * P33;
        float P21_pred = P21 + T * (P22 + P32) + 0.5 * T * T * P23;
        float P22_pred = P22 + T * (P23 + P32) + T * T * P33 + Q2;
        float P23_pred = P23 + T * P33;
        float P31_pred = P31 + T * P32 + 0.5 * T * T * P33;
        float P32_pred = P32 + T * P33;
        float P33_pred = P33 + Q3;

        // Kalman Filter Measurement update step
        float y = current_altitude; // Measurement
        float S = P11_pred + R;
        float K1 = P11_pred / S;
        float K2 = P21_pred / S;
        float K3 = P31_pred / S;

        x_hat1 = x_hat1_pred + K1 * (y - x_hat1_pred);
        x_hat2 = x_hat2_pred + K2 * (y - x_hat1_pred);
        x_hat3 = x_hat3_pred + K3 * (y - x_hat1_pred);

        P11 = (1 - K1) * P11_pred;
        P12 = (1 - K1) * P12_pred;
        P13 = (1 - K1) * P13_pred;
        P21 = P21_pred - K2 * P11_pred;
        P22 = P22_pred - K2 * P12_pred;
        P23 = P23_pred - K2 * P13_pred;
        P31 = P31_pred - K3 * P11_pred;
        P32 = P32_pred - K3 * P12_pred;
        P33 = P33_pred - K3 * P13_pred;

        // ESP_LOGI("Kalman Filter", "current_pressure: %f, current_altitude: %f, x_hat1: %f, x_hat2: %f, x_hat3: %f", current_pressure, current_altitude, x_hat1, x_hat2, x_hat3);
        // ESP_LOGI("Kalman Filter", "P:\n%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n", P11, P12, P13, P21, P22, P23, P31, P32, P33);

        // In sensor_task
        SensorData_t sensorData;
        sensorData.pressure = current_pressure;
        sensorData.x_hat2 = x_hat2;

        if (xQueue != NULL)
        {
            xQueueOverwrite(xQueue, &sensorData);
        }

        vTaskDelay(T * 1000 / portTICK_PERIOD_MS); // 100ms delay (10 Hz)
    }
}

// Task to control the buzzer with a period inversely proportional to x_hat2
void buzzer_task(void *pvParameters)
{
    int buzzer_frequency;
    SensorData_t sensorData;

    while (1)
    {
        // Receive SensorData_t from the sensor task
        if (xQueueReceive(xQueue, &sensorData, portMAX_DELAY))
        {

            // Use ble_send_lk8ex1 to send data periodically
            float x_hat2 = sensorData.x_hat2;
            float pressure = sensorData.pressure;
            float altitude = 99999;
            float vario = x_hat2;
            float temperature = 99;
            float battery = 999;

            ble_send_lk8ex1(pressure, altitude, vario, temperature, battery);

            buzzer_frequency = 2000 + (int)(x_hat2 * SENSIBILITY);

            if (x_hat2 > 0)
                buzzer_frequency += 300; // Add 200 Hz if x_hat2 is positive
            else
                buzzer_frequency -= 300; // Subtract 200 Hz if x_hat2 is negative

            buzzer_frequency = buzzer_frequency < 100 ? 100 : buzzer_frequency;
            buzzer_frequency = buzzer_frequency > 5000 ? 5000 : buzzer_frequency;

            // Calculate the buzzer period inversely proportional to x_hat2
            if (fabs(x_hat2) > DEAD_BAND)
            {
                // Set the buzzer frequency
                ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, buzzer_frequency);
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 4000); // 50% duty cycle
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);

                if (x_hat2 > 0)
                    vTaskDelay(120 / portTICK_PERIOD_MS); // Buzzer on for 120ms
                else
                    vTaskDelay(70 / portTICK_PERIOD_MS); // Buzzer on for 70ms

                // Turn off the buzzer
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0); // 0% duty cycle (off)
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
            }
            vTaskDelay(200 / portTICK_PERIOD_MS); // Buzzer on for 70ms
        }
    }
}

void app_main(void)
{
    // Initialize the blue LED GPIO
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    // Turn on the blue LED at the beginning
    gpio_set_level(GPIO_NUM_2, 1);

    // Initialize I2C and the BMX280 sensor
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = false,
        .scl_pullup_en = false,
        .master = {
            .clk_speed = 100000}};

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // Initialize the buzzer GPIO with PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000, // Initial frequency
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER_GPIO,
        .duty = 0, // Start with buzzer off
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);

    // Create a queue to pass x_hat2 between the sensor task and buzzer task
    xQueue = xQueueCreate(1, sizeof(SensorData_t));
    if (xQueue == NULL)
    {
        ESP_LOGE("app_main", "Failed to create queue");
        return;
    }

    // Initialize BLE
    ESP_ERROR_CHECK(ble_init());

    // Create tasks for sensor reading/filtering and buzzer control
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 4096, NULL, 5, NULL);

    // Buzzer on for 1000ms
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 2000);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 4000);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);     
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL);

    // Turn off the blue LED at the end
    gpio_set_level(GPIO_NUM_2, 0);
}
