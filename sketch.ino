// FreeRTOS example program to collect, process and output data.
// Value reads from pin A0 every 100ms using interrupts and stores in buffer;
// First task calculates the average of read values each time the buffer fills and stores this average in a global variable.
// Second task reads Serial input and whenever command "avg" is recieved - prints last calculated average.

// Settings
static const uint16_t timer_prescaler = 80;       // Assuming 80 MHz frequency of a counter 
static const uint64_t timer_maxcount = 100000;    // interrupt should occur every 100ms
static const char command_avg[] = "avg";
static const TickType_t terminal_delay = pdMS_TO_TICKS(10);
enum { BUF_SIZE = 10 };
enum { CMD_BUF_SIZE = 255 };
enum { MSG_SIZE = 100 };
enum { MSG_QUEUE_SIZE = 5 };

// Pins
static const int adc_pin = A0;

// Structs
// Encapsulating Message struct for sending text messages between tasks.
typedef struct Message {
  char text[MSG_SIZE];
} Message;

// Globals
static float adc_avg;
static volatile uint16_t buf_0[BUF_SIZE];
static volatile uint16_t buf_1[BUF_SIZE];
static volatile uint16_t* write_to = buf_0;
static volatile uint16_t* read_from = buf_1;
static volatile uint8_t buf_overrun = 0;
static TaskHandle_t calcTask = NULL;
static SemaphoreHandle_t sem_done_reading = NULL;
static QueueHandle_t msg_queue;
static hw_timer_t* timer = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// Usefull functions
void IRAM_ATTR ptrSwap_vltl(void** ptr0, void** ptr1) {
  void* ptrTemp = *ptr0;
  *ptr0 = *ptr1;
  *ptr1 = ptrTemp;
}

// ISR
/******************************************************************************************************/
void IRAM_ATTR readADC() {
  static uint16_t idx = 0;
  static BaseType_t taskWoken = pdFALSE;

  if(idx < BUF_SIZE && buf_overrun == 0) {
    *(write_to+idx) = analogRead(adc_pin);
    idx++;
  }

  if(idx >= BUF_SIZE) {

    if(xSemaphoreTakeFromISR(sem_done_reading, &taskWoken) == pdFALSE) {
      buf_overrun = 1;
    }

    if (buf_overrun == 0) {
      idx = 0;
      ptrSwap_vltl((void**)&write_to, (void**)&read_from);

      vTaskNotifyGiveFromISR(calcTask, &taskWoken);
    }

  }

  if(taskWoken) {
    portYIELD_FROM_ISR();
  }
}

// Tasks
/******************************************************************************************************/

void calculateAverage(void* params) {
  float avg;
  Message msg;

  timer = timerBegin(0, timer_prescaler, true);
  timerAttachInterrupt(timer, &readADC, true);
  timerAlarmWrite(timer, timer_maxcount, true);
  timerAlarmEnable(timer);

  while(1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    avg = 0;
    for(int i = 0; i < BUF_SIZE; i++) {
      avg += (float)*(read_from+i);
      // vTaskDelay(pdMS_TO_TICKS(110));
    }
    avg /= BUF_SIZE;

    portENTER_CRITICAL(&spinlock);
    adc_avg = avg;
    portEXIT_CRITICAL(&spinlock);

    if(buf_overrun == 1) {
      strcpy(msg.text, "Error: Buffer overrun. Some data have been dropped");
      xQueueSend(msg_queue, &msg, 0);
    }

    portENTER_CRITICAL(&spinlock);
    buf_overrun = 0;
    xSemaphoreGive(sem_done_reading);
    portEXIT_CRITICAL(&spinlock);
  }
}

void serveTerminal(void* params) {
  char c;
  char cmd_buf[CMD_BUF_SIZE];
  uint8_t idx = 0;
  Message rcv_msg;

  memset(cmd_buf, 0, CMD_BUF_SIZE);

  while(1) {
    if(xQueueReceive(msg_queue, &rcv_msg, 0) == pdTRUE) {
      Serial.println(rcv_msg.text);
    }

    if(Serial.available() > 0) {
      c = Serial.read();

      if(idx < CMD_BUF_SIZE - 1) {
        cmd_buf[idx] = c;
        idx++;
      }

      if(c == '\n') {
        Serial.println();
        cmd_buf[idx-1] = '\0';
        if(strcmp(cmd_buf, command_avg) == 0) {
          Serial.print("Average: ");
          Serial.println(adc_avg);
        }
        memset(cmd_buf, 0, idx);
        idx = 0;
      } else {
        Serial.print(c);
      }

    }

    vTaskDelay(terminal_delay);
  }
}

// Main program
/******************************************************************************************************/

void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));
  Serial.println("Welcome to example data processing program!");
  Serial.print("Type \"avg\" to see the average of ");
  Serial.print(BUF_SIZE);
  Serial.println(" last measurments.");

  sem_done_reading = xSemaphoreCreateBinary();

  if(sem_done_reading == NULL) {
    Serial.println("Could not create semaphore");
    ESP.restart();
  }

  xSemaphoreGive(sem_done_reading);

  msg_queue = xQueueCreate(MSG_SIZE, sizeof(Message));

  xTaskCreatePinnedToCore(calculateAverage,
                          "Calculate average",
                          1024,
                          NULL,
                          1,
                          &calcTask,
                          0);

  xTaskCreatePinnedToCore(serveTerminal,
                          "Serve Terminal",
                          2048,
                          NULL,
                          2,
                          NULL,
                          1);   

  vTaskDelete(NULL);                                             
}

void loop() {

}
