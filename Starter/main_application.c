#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <conio.h>

/*KERNEL INCLUDES*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/*HARDWARE SIMULATOR UTILITY FUNCTIONS*/
#include "HW_access.h"

/*SERIAL SIMULATOR CHANNEL TO USE*/
#define COM_CH_0 (0)
#define COM_CH_1 (1)
#define COM_CH_2 (2)

typedef float float_t;
typedef unsigned int unsigned_t;
#define R_BUF_SIZE (32)
#define NUM_SAMPLES 10

/*Global variables for averaging*/

static float_t average = (float_t)0; /*MISRA 4.9: Inicijalizacija na nulu*/
static uint8_t automatski = (uint8_t)0; /*MISRA 4.9: Inicijalizacija na nulu*/
static uint8_t vrata_status = (uint8_t)0; /*MISRA 4.9: Inicijalizacija na nulu*/
static int8_t prag = (int8_t)0; /*MISRA 4.9: Inicijalizacija na nulu*/
static float_t minValue = (float_t)FLT_MAX; /*Inicijalizacija na maksimalnu vrednost*/
static float_t maxValue = (float_t)FLT_MIN; /* Inicijalizacija na minimalnu vrednost*/
typedef float float_t;
typedef unsigned int unsigned_t;

/*TASK PRIORITIES*/
#define TASK_SERIAL_REC_PRI ((UBaseType_t)4) /* Za serijski prijem sa COM 0 */
#define TASK_DATA_PROC_PRI ((UBaseType_t)3)   /* Za obrada podataka */
#define TASK_SERIAL_SEND0_PRI (tskIDLE_PRIORITY + (UBaseType_t)2) /* Za serijski prenos sa COM 0 */
#define TASK_SERIAL_RECV1_PRI (tskIDLE_PRIORITY + (UBaseType_t)2) /* Za serijski prijem sa COM 1 */
#define TASK_SERIAL_RECV2_PRI (tskIDLE_PRIORITY + (UBaseType_t)2) /* Za serijski prijem sa COM 2 */
#define TASK_SERIAL_SEND1_PRI (tskIDLE_PRIORITY + (UBaseType_t)2) /* Za serijski prenos sa COM 1 */
#define TASK_LED_BAR_PRI (tskIDLE_PRIORITY + (UBaseType_t)1)      /* Za kontrolu LED bara (manualni) */
#define TASK_LED_BAR_AUT_PRI (tskIDLE_PRIORITY + (UBaseType_t)2)  /* Za kontrolu LED bara (automatski) */
#define TASK_SEND_TO_PC_PRI (tskIDLE_PRIORITY + (UBaseType_t)2)   /* Za slanje podataka PC-ju */
#define TASK_ISPIS_7SEG_PRI (tskIDLE_PRIORITY + (UBaseType_t)1)   /* Za ispis na 7-segmentnom displeju*/

/*TASKS FORWARD DECLARATIONS*/
static void SerialReceive_Task(void* pvParameters);
static void DataProcessing_Task(void* pvParameters);
static void SerialSend_Task0(void* pvParameters);
static void TimerCallback(TimerHandle_t tmh);
static void TimerCallback500(TimerHandle_t tmh);
static void TimerCallback7(TimerHandle_t tmh);
static void SerialReceive_Task2(void* pvParameters);
static void SerialReceive_Task1(void* pvParameters);
static void SerialSend_Task1(void* pvParameters);
static void LEDBar_Task(void* pvParameters);
static void LEDBar_Task1(void* pvParameters);
static void SendToPC_Task(void* pvParameters);
static void Ispis_7Seg(void* pvParameters);

/*7 - SEG NUMBER DATABASE - ALL HEX DIGITS[0 1 2 3 4 5 6 7 8 9 A B C D E F]*/
static const uint8_t hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };

/*RECEPTION DATA BUFFER - COM 0*/
static unsigned volatile r_point1; /*MISRA 4.9: Inicijalizacija na nulu*/

static unsigned volatile r_point2; /*MISRA 4.9: Inicijalizacija na nulu*/

static unsigned volatile r_point3; /*MISRA 4.9: Inicijalizacija na nulu*/

/*GLOBAL OS - HANDLES*/
static SemaphoreHandle_t RXC_BinarySemaphore;
static SemaphoreHandle_t TBE_BinarySemaphore;
static SemaphoreHandle_t RXC_BinarySemaphore1;
static SemaphoreHandle_t TBE_BinarySemaphore1;
static SemaphoreHandle_t RXC_BinarySemaphore2;
static SemaphoreHandle_t TBE_BinarySemaphore2;
static SemaphoreHandle_t SrednjaVrijednost;
static SemaphoreHandle_t Blinkanje;
static SemaphoreHandle_t Displej;
static QueueHandle_t Data_Queue;
static QueueHandle_t Data_Queue1;
static TimerHandle_t timer1;
static TimerHandle_t timer500;
static TimerHandle_t timer7;



/*LED BAR MANUELNI*/
static SemaphoreHandle_t LED_INT_BinarySemaphore;

/*INTERRUPTS*/
static void prvProcessTBEInterrupt(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (get_TBE_status(0) != 0) {
        if (xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPriorityTaskWoken) != pdPASS) {
            printf("Greska pri oslobadjanju semafora TBE_BinarySemaphore\n");
        }
    }

    if (get_TBE_status(1) != 0) {
        if (xSemaphoreGiveFromISR(TBE_BinarySemaphore1, &xHigherPriorityTaskWoken) != pdPASS) {
            printf("Greska pri oslobadjanju semafora TBE_BinarySemaphore1\n");
        }
    }

    if (get_TBE_status(2) != 0) {
        if (xSemaphoreGiveFromISR(TBE_BinarySemaphore2, &xHigherPriorityTaskWoken) != pdPASS) {
            printf("Greska pri oslobadjanju semafora TBE_BinarySemaphore2\n");
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static uint32_t prviProcessRXCInterrupt(void) {
    BaseType_t xHigherPTW = pdFALSE;

    if (get_RXC_status(0) != 0) {
        if (xSemaphoreGiveFromISR(RXC_BinarySemaphore, &xHigherPTW) != pdPASS) {
            printf("Greska.");
        }
    }
    if (get_RXC_status(1) != 0) {
        if (xSemaphoreGiveFromISR(RXC_BinarySemaphore1, &xHigherPTW) != pdPASS) {
            printf("Greska.");
        }
    }
    if (get_RXC_status(2) != 0) {
        if (xSemaphoreGiveFromISR(RXC_BinarySemaphore2, &xHigherPTW) != pdPASS) {
            printf("Greska.");
        }
    }

    portYIELD_FROM_ISR((uint32_t)xHigherPTW);
}

static uint32_t OnLED_ChangeInterrupt(void) {
    BaseType_t xHigherPTW = pdFALSE;

    if (xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW) != pdPASS) {
        printf("Greska.");
    }
    portYIELD_FROM_ISR((uint8_t)xHigherPTW);
}
/*PERIODIC TIMER CALLBACK*/
static void TimerCallback(TimerHandle_t tmh) {
    BaseType_t result = xSemaphoreGive(SrednjaVrijednost);
    //if (result != pdTRUE) {
     //   printf("Greska pri davanju semafora SrednjaVrijednost\n");
   // }
}

static void TimerCallback500(TimerHandle_t tmh) {
    BaseType_t result = xSemaphoreGive(Blinkanje);
    // if (result != pdTRUE) {
      //   printf("Greska pri davanju semafora Blinkanje\n");
    // }
}

static void TimerCallback7(TimerHandle_t tmh) {
    BaseType_t result = xSemaphoreGive(Displej);
    // if (result != pdTRUE) {
         // Ako funkcija xSemaphoreGive ne uspe, prijavite grešku
      //   printf("Greska pri davanju semafora Displej\n");
    // }
}



/*MAIN - SYSTEM STARTUP POINT*/
void main_demo(void);
void main_demo(void) {
    /*INITIALIZATION OF THE PERIPHERALS*/

    r_point1 = 0;
    r_point2 = 0;
    r_point3 = 0;

    if (init_LED_comm() != 0) { printf("error LED"); }
    if (init_7seg_comm() != 0) { printf("error 7seg"); }

    if (init_serial_uplink(COM_CH_0) != 0) { printf("ERROR KANAL 0"); } /*Initialize serial TX on channel 0*/
    if (init_serial_downlink(COM_CH_0) != 0) { printf("ERROR KANAL 0"); }

    if (init_serial_uplink(COM_CH_1) != 0) { printf("ERROR KANAL 1"); }
    if (init_serial_downlink(COM_CH_1) != 0) { printf("ERROR KANAL 1"); }


    if (init_serial_uplink(COM_CH_2) != 0) { printf("ERROR KANAL 2"); }

    if (init_serial_downlink(COM_CH_2) != 0) { printf("ERROR KANAL 2"); } /*Initialize serial RX on channel 0*/


    /*INTERRUPT HANDLERS*/
    vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prviProcessRXCInterrupt); // Serial reception interrupt handler
    vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);
    vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);


    /*BINARY SEMAPHORES*/

        // Inicijalizacija semafora
    TBE_BinarySemaphore = xSemaphoreCreateBinary();
    if (TBE_BinarySemaphore == NULL) {
        printf("Greska pri kreiranju TBE_BinarySemaphore\n");
    }

    TBE_BinarySemaphore1 = xSemaphoreCreateBinary();
    if (TBE_BinarySemaphore1 == NULL) {
        printf("Greska pri kreiranju TBE_BinarySemaphore1\n");
    }

    TBE_BinarySemaphore2 = xSemaphoreCreateBinary();
    if (TBE_BinarySemaphore2 == NULL) {
        printf("Greska pri kreiranju TBE_BinarySemaphore2\n");
    }



    RXC_BinarySemaphore = xSemaphoreCreateBinary();


    RXC_BinarySemaphore1 = xSemaphoreCreateBinary();
    if (RXC_BinarySemaphore1 == NULL) { printf("greska(RXC_BinarySemaphore1)"); }

    if (TBE_BinarySemaphore1 == NULL) { printf("greska(TBE_BinarySemaphore1)"); }
    RXC_BinarySemaphore2 = xSemaphoreCreateBinary();
    if (RXC_BinarySemaphore2 == NULL) { printf("greska(RXC_BinarySemaphore2)"); }

    if (TBE_BinarySemaphore2 == NULL) { printf("greska(TBE_BinarySemaphore2)"); }
    Displej = xSemaphoreCreateBinary();
    if (Displej == NULL) { printf("greska(Displej)"); }
    Blinkanje = xSemaphoreCreateBinary();
    if (Blinkanje == NULL) { printf("greska(Blinkanje)"); }
    LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
    if (LED_INT_BinarySemaphore == NULL) { printf("greska(LED_INT_BinarySemaphore)"); }

    SrednjaVrijednost = xSemaphoreCreateBinary();
    if (SrednjaVrijednost == NULL) { printf("greska(SrednjaVrijednost)"); }


    /*QUEUES*/
    Data_Queue = xQueueCreate(10, sizeof(float_t));
    if (Data_Queue == NULL) {
        printf("Greska prilikom kreiranja queue_senzor0\n");
    }

    Data_Queue1 = xQueueCreate(10, sizeof(float_t));
    if (Data_Queue1 == NULL) {
        printf("Greska prilikom kreiranja queue_senzor0\n");
    }

    /*TIMERS*/
    timer1 = xTimerCreate(NULL, pdMS_TO_TICKS(200U), pdTRUE, NULL, TimerCallback);
    if (timer1 == NULL) {
        printf("Tajmer1");
    }

    if (xTimerStart(timer1, portMAX_DELAY) != pdTRUE) { printf("ERROR TIMERSTAR1"); }

    timer500 = xTimerCreate(NULL, pdMS_TO_TICKS(500U), pdTRUE, NULL, TimerCallback500);
    if (timer500 == NULL) {
        printf("Tajmer2");
    }

    if (xTimerStart(timer500, portMAX_DELAY) != pdTRUE) { printf("ERROR TIMERSTAR500"); }


    timer7 = xTimerCreate(NULL, pdMS_TO_TICKS(1500U), pdTRUE, NULL, TimerCallback7);
    if (timer7 == NULL) {
        printf("Tajmer3");
    }

    if (xTimerStart(timer7, portMAX_DELAY) != pdTRUE) { printf("ERROR TIMERSTAR7"); }


    /*TASKS*/
    if (xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL) != pdPASS) {
        printf("Task1");
    }
    if (xTaskCreate(DataProcessing_Task, "DPx", configMINIMAL_STACK_SIZE, NULL, TASK_DATA_PROC_PRI, NULL) != pdPASS) {
        printf("Greska");
    }
    if (xTaskCreate(SerialSend_Task0, "disp", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND0_PRI, NULL) != pdPASS) { printf("GRESKA(SerialSend_Task0)"); }
    if (xTaskCreate(SerialReceive_Task1, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_RECV1_PRI, NULL) != pdPASS) { printf("GRESKA(SerialReceive_Task1)"); }
    if (xTaskCreate(SerialSend_Task1, "disp", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND1_PRI, NULL) != pdPASS) { printf("GRESKA(SerialSend_Task1)"); }
    if (xTaskCreate(SerialReceive_Task2, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_RECV2_PRI, NULL) != pdPASS) { printf("GRESKA(SerialReceive_Task2)"); }
    if (xTaskCreate(LEDBar_Task, "ST", configMINIMAL_STACK_SIZE, NULL, TASK_LED_BAR_PRI, NULL) != pdPASS) { printf("GRESKA(LEDBar_Task)"); }
    if (xTaskCreate(LEDBar_Task1, "ST", configMINIMAL_STACK_SIZE, NULL, TASK_LED_BAR_AUT_PRI, NULL) != pdPASS) { printf("GRESKA(LEDBar_Task1)"); }
    if (xTaskCreate(SendToPC_Task, "ST", configMINIMAL_STACK_SIZE, NULL, TASK_SEND_TO_PC_PRI, NULL) != pdPASS) { printf("GRESKA(SendToPC_Task)"); }
    if (xTaskCreate(Ispis_7Seg, "ST", configMINIMAL_STACK_SIZE, NULL, TASK_ISPIS_7SEG_PRI, NULL) != pdPASS) { printf("GRESKA(Ispis_7Seg)"); }




    /*START SCHEDULER*/
    vTaskStartScheduler();
    for (;;) {
    }
}




static void SerialSend_Task0(void* pvParameters) {
    static unsigned volatile t_point0 = 0;  // Use 'unsigned' for compatibility
    static const char trigger[] = "XYZ";

    for (;;) {
        if (t_point0 >= (sizeof(trigger) - 1u)) {
            t_point0 = 0u;
        }

        if (send_serial_character(COM_CH_0, (unsigned char)trigger[t_point0]) != 0) {
            printf("Greska\n");
        }
        t_point0++;

        if (xSemaphoreTake(TBE_BinarySemaphore, portMAX_DELAY) != pdPASS) {
            printf("Greska pri preuzimanju semafora TBE_BinarySemaphore\n");
            // Možda želite da preduzmete dodatne korake, kao što je ponovni pokušaj
        }
    }
}


static void SerialReceive_Task(void* pvParameters) {
    uint8_t cc = 0;
    static uint8_t r_buffer1[R_BUF_SIZE] = { 0 };
    char* endptr;  // Ensure endptr is declared before use

    for (;;) {
        // Wait for the semaphore to be available
        if (xSemaphoreTake(RXC_BinarySemaphore, portMAX_DELAY) != pdPASS) {
            printf("Greska pri prijemu semafora u SerialReceive_Task\n");
            continue;  // Continue the loop to retry semaphore acquisition
        }

        // Retrieve the character from the serial channel
        if (get_serial_character(COM_CH_0, &cc) != 0) {
            printf("Greska pri prijemu karaktera na kanalu 0\n");
            continue;  // Continue the loop to retry character reception
        }

        // Process received character
        if (cc == (uint8_t)0xfe) {
            r_point1 = 0;  // Reset buffer index on receiving start marker
        }
        else if (cc == (uint8_t)0x0d) {
            r_buffer1[r_point1] = 0u;  // Null-terminate the buffer
            //10.3, konvertovanje 0u

            // Convert string to float
            float_t broj = (float_t)strtof((const char*)r_buffer1, &endptr);

            // Check if conversion was successful
            if (endptr == (char*)r_buffer1) {
                printf("Greska u konverziji stringa u float\n");
            }
            else if (xQueueSend(Data_Queue, &broj, 0U) != pdPASS) {
                printf("Greska pri slanju podatka u Data_Queue\n");
            }

            r_point1 = 0;  // Reset buffer index after processing
        }
        else if (r_point1 < (unsigned_t)(R_BUF_SIZE - 1U)) {
            r_buffer1[r_point1++] = cc;  // Store character and increment index
        }
        else {
            // Handle buffer overflow
            printf("Prekoracenje kapaciteta bafera\n");
            r_point1 = 0;  // Reset index to avoid buffer overflow issues
        }
    }
}

static void DataProcessing_Task(void* pvParameters) {
    float_t broj;
    float_t broj1;


    float_t sensorReadings[NUM_SAMPLES] = { 0.0f };
    uint8_t currentIndex = 0U;
    float_t sum = 0.0f;
    uint8_t numReadings = 0U;



    for (;;) {

        if (xSemaphoreTake(SrednjaVrijednost, portMAX_DELAY) != pdPASS) { printf("Greska pri prijemu semafora u DataProcessing_Task"); }

        if (xQueueReceive(Data_Queue1, &broj1, 0U) == pdTRUE) {
            printf("Podatak primljen u Data_Queue1: %f\n", broj1);

        }

        if (xQueueReceive(Data_Queue, &broj, portMAX_DELAY) == pdTRUE) {

            if (broj < minValue) {
                minValue = broj;
            }
            if (broj > maxValue) {
                maxValue = broj;
            }

            float result = broj * 0.1f;  // Izračunavanje sa float tipom
            prag = 100 - (int8_t)result; // Kastovanje na int8_t
            printf("Podatak: %f, Prag: %d\n", (float)broj, (int)prag);

            if (prag == 0) {
                printf("Apsolutni mrak!\n");
            }
            else if (prag == 100) {
                printf("Osvjetljenje je maksimalno!\n");
            }
            else if (prag < 0) {
                if (set_LED_BAR(1, 0x40) != 0) {
                    printf("Greska");
                }
                printf("Kratka svjetla se automatski ukljucuju, a dnevna se iskljucuju!\n");
            }
            else if (prag > 100) {
                if (set_LED_BAR(1, 0x80) != 0) {
                    printf("Greska");
                }
                printf("Dnevna svjetla se ukljucuju!\n");
            }
            else {
                // Handle unexpected case or do nothing
                // This is added to comply with MISRA Rule 15.7
                printf("Prag vrednost nije u predvidjenom opsegu.\n");
            }



            if (numReadings < (uint8_t)NUM_SAMPLES) {
                numReadings++;
            }
            else {
                sum -= sensorReadings[currentIndex];
            }

            sensorReadings[currentIndex] = broj;
            sum += broj;
            currentIndex = (currentIndex + 1U) % (uint8_t)NUM_SAMPLES;

            average = sum / (float_t)NUM_SAMPLES;

            printf("Prosecna vrednost poslednjih %d brojeva je : %f\n", NUM_SAMPLES, (float)average);
            printf("Minimalna vrednost: %f\n", (float)minValue);
            printf("Maksimalna vrednost: %f\n", (float)maxValue);
        }

        if (xSemaphoreGive(SrednjaVrijednost) != pdPASS) {
            // Ako ne možete da oslobodite semafor, možda želite da logujete grešku ili obavite neku akciju
            printf("Greska pri oslobadjanju semafora SrednjaVrijednost\n");
        }
    }
}

static void SerialSend_Task1(void* pvParameters) {
    static unsigned volatile t_point1 = 0;  // Use 'unsigned' for compatibility
    static const char trigger1[] = "XYZ";

    for (;;) {
        if (t_point1 >= (sizeof(trigger1) - 1u)) {
            t_point1 = 0u;
        }

        if (send_serial_character(COM_CH_1, (unsigned char)trigger1[t_point1]) != 0) {
            printf("Greska\n");
        }
        t_point1++;

        if (xSemaphoreTake(TBE_BinarySemaphore1, portMAX_DELAY) != pdPASS) {
            // Ako se semafor ne može preuzeti, obavestite korisnika ili preduzmite druge akcije
            printf("Greska pri preuzimanju semafora TBE_BinarySemaphore1\n");
        }
    }
}
static void SerialReceive_Task1(void* pvParameters) {
    uint8_t cc = 0;
    static uint8_t r_buffer2[R_BUF_SIZE] = { 0 };
    float_t broj1;

    for (;;) {
        if (xSemaphoreTake(RXC_BinarySemaphore1, portMAX_DELAY) != pdPASS) {
            printf("Greska pri prijemu semafora u SerialReceive_Task1");
            continue;
        }

        if (get_serial_character(COM_CH_1, &cc) != 0) {
            printf("Greska pri prijemu karaktera na kanalu 1\n");
            continue;
        }

        if (cc == (uint8_t)0xfe) {
            r_point2 = 0u;
        }
        else if (cc == (uint8_t)0x0d) {
            r_buffer2[r_point2] = 0u;

            // Convert string to float
            broj1 = (float_t)strtof((const char*)r_buffer2, NULL);

            // Convert float to uint8_t
            vrata_status = (uint8_t)broj1;

            if (vrata_status == (uint8_t)0 || vrata_status == (uint8_t)1) {
                if (xQueueSend(Data_Queue1, &vrata_status, portMAX_DELAY) == pdPASS) {
                    printf("Unet broj na kanalu 1 je: %u\n", vrata_status);
                    if (vrata_status == (uint8_t)0) {
                        printf("Vrata su zatvorena!\n");
                    }
                    else if (vrata_status == (uint8_t)1) {
                        printf("Vrata su otvorena!\n");
                    }
                    else {
                        printf("Greska");
                    }
                }
                else {
                    printf("Greska prilikom smjestanja podatka u red\n");
                }
            }
            else {
                printf("Nepoznata vrednost na kanalu 1: %f\n", broj1);
            }

            r_point2 = 0;
        }
        else if (r_point2 < (unsigned int)(R_BUF_SIZE - 1)) {
            r_buffer2[r_point2++] = cc;
        }
        else {
            printf("Buffer overflow, reset\n");
            r_point2 = 0;
        }
    }
}
static void SerialReceive_Task2(void* pvParameters) {
    uint8_t cc = 0;
    static uint8_t r_buffer3[R_BUF_SIZE] = { 0 };

    for (;;) {
        if (xSemaphoreTake(RXC_BinarySemaphore2, portMAX_DELAY) != pdPASS) { printf("Greska pri prijemu semafora u SerialReceive_Task2"); }

        if (get_serial_character(COM_CH_2, &cc) != 0) {
            printf("Greska pri prijemu karaktera na kanalu 2\n");
            continue;
        }


        printf("Received character: %c (0x%02X)\n", cc, cc);

        if (cc == (uint8_t)0x0d) {
            r_buffer3[r_point3] = 0u;
            //kastovanje 
            printf("Command received: %s\n", r_buffer3);


            if (strncmp((const char*)r_buffer3, "MANUELNO", 8) == 0) {
                printf("Processing command: MANUELNO\n");
                automatski = 0u;

                if (send_serial_character(COM_CH_2, (unsigned char)'O') != 0) {
                    printf("Greska 488");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'K') != 0) {
                    printf("Greska 492");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'\r') != 0) {
                    printf("Greska 496");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'\n') != 0) {
                    printf("Greska 504");
                }
            }
            else if (strncmp((const char*)r_buffer3, "AUTOMATSKI", 10) == 0) {
                printf("Processing command: AUTOMATSKI\n");
                automatski = 1u;

                if (send_serial_character(COM_CH_2, (unsigned char)'O') != 0) {
                    printf("Greska 508");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'K') != 0) {
                    printf("Greska 502");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'\r') != 0) {
                    printf("Greska 504");
                }
                vTaskDelay(pdMS_TO_TICKS(100u));
                if (send_serial_character(COM_CH_2, (unsigned char)'\n') != 0) {
                    printf("Greska pri slanju karaktera");
                }
            }
            else {
                printf("Unknown command: %s\n", r_buffer3);
            }


            r_point3 = 0u;
        }
        else if (r_point3 < (unsigned_t)R_BUF_SIZE - 1u) {
            r_buffer3[r_point3++] = cc;
        }
        else {

            r_point3 = 0u;
            printf("Buffer overflow, reset\n");
        }
    }
}

static void LEDBar_Task(void* pvParameters) {
    uint8_t previous_output = 0x00u;
    uint8_t new_output = 0x00u;
    uint8_t d;
    TickType_t last_blink_time = 0; // Vreme poslednjeg trepćećeg signala
    TickType_t last_blink_time1 = 0; // Vreme poslednjeg trepćećeg signala
    TickType_t current_time; // Trenutno vreme
    TickType_t current_time1; // Trenutno vreme
    uint8_t blink_state = 0U; /*MISRA 4.9: Inicijalizacija na nulu*/
    uint8_t blink_state1 = 0U; /*MISRA 4.9: Inicijalizacija na nulu*/
    TickType_t blink_interval = pdMS_TO_TICKS(500u);

    for (;;) {
        if (xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY) != pdPASS) {
            printf("Greska pri prijemu semafora u LEDBar_Task\n");
        }

        if (automatski == 0u) {
            if (get_LED_BAR(0, &d) != 0) {
                printf("Greska led");
            }

            new_output = 0x00u;

            if ((d & 0x01u) != 0u) {
                new_output |= 0x80u;
                printf("Dnevna SVJETLA\n");
            }
            if ((d & 0x02u) != 0u) {
                new_output |= 0x40u;
                printf("KRATKA SVJETLA\n");
            }
            if ((d & 0x04u) != 0u) {
                new_output |= 0x20u;
                printf("DUGA SVJETLA\n");
            }
            if ((d & 0x08u) != 0u) {
                //&& ((d & 0x10u) != 0u)) {
                current_time = xTaskGetTickCount();

                // Koristi vTaskDelayUntil za preciznije upravljanje intervalom blikanja
                if ((current_time - last_blink_time) >= blink_interval) {
                    last_blink_time = current_time; // Ažuriraj vreme poslednjeg trepćećeg signala
                    blink_state ^= 1U; // Promeni stanje trepćećih LED

                    if (blink_state != 0U) {
                        new_output |= 0x10u; // Uključi trepćeće LED
                    }
                    else {
                        new_output &= ~0x10u; // Isključi trepćeće LED
                    }
                    printf("ZMIGAVCI: novo stanje trepćećih LED = %02X\n", new_output);
                }
            }
            if ((d & 0x10u) != 0u) {
                //&& ((d & 0x10u) != 0u)) {
                current_time1 = xTaskGetTickCount();

                // Koristi vTaskDelayUntil za preciznije upravljanje intervalom blikanja
                if ((current_time1 - last_blink_time1) >= blink_interval) {
                    last_blink_time1 = current_time1; // Ažuriraj vreme poslednjeg trepćećeg signala
                    blink_state1 ^= 1U; // Promeni stanje trepćećih LED

                    if (blink_state1 != 0U) {
                        new_output |= 0x08u; // Uključi trepćeće LED
                    }
                    else {
                        new_output &= ~0x08u; // Isključi trepćeće LED
                    }
                    printf("ZMIGAVCI: novo stanje trepćećih LED = %02X\n", new_output);
                }
            }

            if (new_output != previous_output) {
                if (set_LED_BAR(1, new_output) != 0) {
                    printf("Greska sa led barom");
                }
                previous_output = new_output;
            }
        }

        if (xSemaphoreGive(LED_INT_BinarySemaphore) != pdPASS) {
            printf("Greska pri davanju semafora");
        }
        vTaskDelay(pdMS_TO_TICKS(50u));
    }
}
static void LEDBar_Task1(void* pvParameters) {
    uint8_t previous_output = 0xFFu; // Početno stanje - sve LED diode uključene
    uint8_t output = 0xFFu;
    BaseType_t xStatus;

    // Inicijalno postavljanje svih LED-ova na početku
    if (set_LED_BAR(1, output) != 0) {
        printf("Greska 613: Neuspešno postavljanje LED bara\n");
    }

    for (;;) {
        // Primanje statusa vrata iz reda sa beskonačnim čekanjem
        xStatus = xQueueReceive(Data_Queue1, &vrata_status, 0);

        if (xStatus == pdTRUE) {
            if (vrata_status == 0u) {
                output = 0x00u; // Sve diode isključene
            }
            else if (vrata_status == 1u) {
                output = 0xFFu; // Sve diode uključene
            }
            else {
                printf("Ponovo unesi broj: Nevažeći status vrata\n");
                continue; // Ako broj nije 0 ili 1, nastavi sa sledećim ciklusom
            }

            // Provera i ažuriranje izlaza samo ako se promeni
            if (output != previous_output) {
                if (set_LED_BAR(1, output) != 0) {
                    printf("Greska 647: Neuspešno ažuriranje LED bara\n");
                }
                previous_output = output; // Ažuriranje prethodnog izlaza
            }
        }

        // Proverite srednju vrednost i isključite LED na poziciji 0x40 ako je potrebno
        if (average > 500.0f) {
            // Ako je LED na poziciji 0x40 uključena, isključite je
            if ((output & (uint8_t)0x40u) != 0) {
                vTaskDelay(pdMS_TO_TICKS(5000u)); // Sačekajte 5 sekundi pre nego što isključite LED
                output &= ~0x40u; // Isključite LED na poziciji 0x40

                // Ažurirajte LED bar ako se promeni stanje
                if (output != previous_output) {
                    if (set_LED_BAR(1, output) != 0) {
                        printf("Greska 647: Neuspešno ažuriranje LED bara\n");
                    }
                    previous_output = output; // Ažuriranje prethodnog izlaza
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100u)); // Pauza između iteracija
    }
}
static void Ispis_7Seg(void* pvParameters) {
    uint8_t d;
    for (;;) {
        BaseType_t xStatus;

        xStatus = xSemaphoreTake(Displej, portMAX_DELAY);
        if (xStatus != pdPASS) {
            // Obrada greške ako semafor nije uspešno preuzet
            printf("Greska pri preuzimanju semafora Displej\n");
            // Moguće dodatne akcije, kao što je obaveštavanje ili ponovni pokušaj
        }

        int integer_part = (int)average;
        int thousands = (integer_part / 1000) % 10;
        int hundreds = (integer_part / 100) % 10;
        int tens = (integer_part / 10) % 10;
        int ones = integer_part % 10;

        if (select_7seg_digit(0) != 0) {
            printf("Greska");
        }
        if (set_7seg_digit(hexnum[thousands]) != 0) {
            printf("Greska");
        }
        if (select_7seg_digit(1) != 0) {
            printf("Greska");
        }
        if (set_7seg_digit(hexnum[hundreds]) != 0) {
            printf("Greska");
        }
        if (select_7seg_digit(2) != 0) {
            printf("Greska");
        }
        if (set_7seg_digit(hexnum[tens]) != 0) {
            printf("Greska");
        }
        if (select_7seg_digit(3) != 0) {
            printf("Greska");
        }
        if (set_7seg_digit(hexnum[ones]) != 0) {
            printf("Greska");
        }


        if (automatski == 1u) {
            if (select_7seg_digit(4) != 0) {
                printf("Greska.");
            }
            if (set_7seg_digit(hexnum[0]) != 0) {
                printf("Greska.");
            }
        }
        else {
            if (select_7seg_digit(4) != 0) {
                printf("Greska");
            }
            if (set_7seg_digit(hexnum[1]) != 0) {
                printf("Greska.");
            }

        }



        int integer_part_max = (int)maxValue;


        int thousands_max = (integer_part_max / 1000) % 10;
        int hundreds_max = (integer_part_max / 100) % 10;
        int tens_max = (integer_part_max / 10) % 10;
        int ones_max = integer_part_max % 10;




        int integer_part_min = (int)minValue;


        int thousands_min = (integer_part_min / 1000) % 10;
        int hundreds_min = (integer_part_min / 100) % 10;
        int tens_min = (integer_part_min / 10) % 10;
        int ones_min = integer_part_min % 10;



        if (get_LED_BAR(0, &d) != 0) {
            printf("Greska 730");
        }


        if ((d & 0x40u) != 0U) {
            if (select_7seg_digit(0) != 0) {
                printf("Greska 735");
            }
            if (set_7seg_digit(hexnum[thousands_max]) != 0) {
                printf("Greska 738");
            }
            if (select_7seg_digit(1) != 0) {
                printf("Greska 741");
            }
            if (set_7seg_digit(hexnum[hundreds_max]) != 0) {
                printf("Greska 744");
            }
            if (select_7seg_digit(2) != 0) {
                printf("Greska 747");
            }
            if (set_7seg_digit(hexnum[tens_max]) != 0) {
                printf("Greska 750");
            }
            if (select_7seg_digit(3) != 0) {
                printf("Greska 753");
            }
            if (set_7seg_digit(hexnum[ones_max]) != 0) {
                printf("Greska 756");
            }
        }
        if ((d & 0x80u) != 0U) {
            if (select_7seg_digit(0) != 0) {
                printf("Greska 761");
            }
            if (select_7seg_digit(hexnum[thousands_min]) != 0) {
                printf("Greska 504");
            }
            if (select_7seg_digit(1) != 0) {
                printf("Greska 504");
            }
            if (set_7seg_digit(hexnum[hundreds_min]) != 0) {
                printf("Greska 504");
            }
            if (select_7seg_digit(2) != 0) {
                printf("Greska 504");
            }
            if (set_7seg_digit(hexnum[tens_min]) != 0) {
                printf("Greska 504");
            }
            if (select_7seg_digit(3) != 0) {
                printf("Greska 504");
            }
            if (set_7seg_digit(hexnum[ones_min]) != 0) {
                printf("Greska 504");
            }
        }
    }
}




static void SendToPC_Task(void* pvParameters) {
    char message[100];
    uint8_t d;
    for (;;) {

        if (get_LED_BAR(1, &d) != 0) {
            printf("Greska820");
        }

        sprintf(message,
            "Light Intensity: %f\n"
            "Daytime Running Lights: %s\n"
            "Low Beams: %s\n"
            "High Beams: %s\n"
            "Left Turn Signal: %s\n"
            "Right Turn Signal: %s\n",
            average,
            (d & 0x80U) ? "ON" : "OFF",
            (d & 0x40U) ? "ON" : "OFF",
            (d & 0x20U) ? "ON" : "OFF",
            (d & 0x08U) ? "ON" : "OFF",
            (d & 0x10U) ? "ON" : "OFF");


        for (char* p = message; *p != '\0'; p++) {
            if (send_serial_character(COM_CH_2, (unsigned char)*p) != 0) {
                //misra-kastovanje
                printf("Greska.");
            }
            xSemaphoreTake(TBE_BinarySemaphore2, portMAX_DELAY); // Take semaphore
        }


        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
