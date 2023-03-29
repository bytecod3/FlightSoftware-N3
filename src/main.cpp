#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_BMP085.h>
#include "sensors.h"
#include "defs.h"
#include "state_machine.h"

/**
 * DEBUG 
 * debug functions
 * 1. display_data()
 * 2. counter_update()
 * 
 */
int state_leds[5] = {4, 5, 18, 23, 2};
int state;

State_machine fsm;

/* create Wi-Fi Client */
WiFiClient wifi_client;

/* create MQTT publish-subscribe client */
PubSubClient mqtt_client(wifi_client);

/* create gyroscope object */
Adafruit_MPU6050 gyroscope;

/* create altimeter objects */
Adafruit_BMP085 altimeter;

/* position integration variables */
long long current_time = 0;
long long previous_time = 0;

/* velocity integration variables */
double y_velocity = 0;
double y_displacement = 0;

float new_y_displacement = 0.0;
float old_y_displacement = 0.0;
double old_y_velocity = 0.0;
double new_y_velocity = 0.0;
double total_y_displacement = 0.0;

/* functions to initialize sensors */
void initialize_gyroscope(){
    /* attempt to initialize MPU6050 */
    if(!gyroscope.begin(0x68)){
        debugln("[-]Gyroscope allocation failed!");
        // loop forever until found
        while(true){}
    }

    debugln("[+]Gyroscope Initialized");
    gyroscope.setAccelerometerRange(MPU6050_RANGE_8_G);
    gyroscope.setGyroRange(MPU6050_RANGE_500_DEG);
    gyroscope.setFilterBandwidth(MPU6050_BAND_5_HZ);

    delay(SETUP_DELAY);
}

void initialize_altimeter(){
    if (!altimeter.begin()) {
        debugln("[-]Could not find a valid altimeter sensor");
        while (1) {}
    }

    debugln("[+]Altimeter initialized");
}

/* data variables */
/* gyroscope data */

struct Acceleration_Data{
    double ax;
    double ay; 
    double az;
};

struct Altimeter_Data{
    int32_t pressure;
    double altitude;
    double velocity;
    double AGL; /* altitude above ground level */
};

struct Telemetry_Data{
    float ax;
    float ay; 
    float az;
    int32_t pressure;
    float altitude;
    float velocity;
    float AGL; /* altitude above ground level */
};

// typedef struct{
//     int PRE_FLIGHT   =       0;
//     int POWERED_FLIGHT =      1;
//     int COASTING        =    2;
//     int APOGEE             =  3;
//     int BALLISTIC_DESCENT =   4;
//     int PARACHUTE_DESCENT  = 5;
//     int POST_FLIGHT        = 6;
// } FLIGHT_STATES;

/* create queue to store altimeter data
 * store pressure and altitude
 * */
QueueHandle_t gyroscope_data_queue;
QueueHandle_t altimeter_data_queue;
QueueHandle_t telemetry_data_queue; /* This queue will hold all the sensor data for transmission to ground station*/
QueueHandle_t filtered_data_queue;
QueueHandle_t flight_states_queue;

void connectToWifi(){
    /* Connect to a Wi-Fi network */
    debugln("[..]Scanning for network...");

    WiFi.begin(SSID, PASSWORD);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        debugln("[..]Scanning for network...");
    }

    debugln("[+]Network found");debug("[+]My IP address: "); debugln();
    debugln(WiFi.localIP());
}

void initializeMQTTParameters(){
    /* this functions creates an MQTT client for transmitting telemetry data */;
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
}

void readAltimeter(void* pvParameters){

    while(true){
        /* Read pressure.
         * This is the pressure from the sea level.
         * */
        struct Altimeter_Data altimeter_data;
        struct Telemetry_Data altimeter_telemetry_data;

        /* Read pressure
         * This is the pressure from the sea level
         * */
        altimeter_data.pressure = altimeter.readSealevelPressure();

        /* Read altitude
         * This is the altitude from the sea level
         * */
        altimeter_data.altitude = altimeter.readAltitude(SEA_LEVEL_PRESSURE);

        /*------------- APOGEE DETECTION ALGORITHM -------------------------------------*/

        /* approximate velocity from acceleration by integration for apogee detection */
        current_time = millis();

        /* differentiate displacement to get velocity */
        new_y_displacement = altimeter_data.altitude - ALTITUDE_OFFSET;
        y_velocity = (new_y_displacement - old_y_displacement) / (current_time - previous_time);

        /* update integration variables */
        previous_time = current_time;
        old_y_displacement = new_y_displacement;

        /* ------------------------ END OF APOGEE DETECTION ALGORITHM ------------------------ */

        /* subtract current altitude to get the maximum height reached */
        float rocket_height = altimeter_data.altitude - ALTITUDE_OFFSET;

        /* update altimeter data */
        altimeter_data.velocity = y_velocity;
        altimeter_data.AGL = rocket_height;

        /* send data to altimeter queue */
        if(xQueueSend(altimeter_data_queue, &altimeter_data, portMAX_DELAY) != pdPASS){
            debugln("[-]Altimeter queue full");
        }

        delay(TASK_DELAY);
    }
}

void readGyroscope(void* pvParameters){
    
    while(true){
        sensors_event_t a, g, temp;
        gyroscope.getEvent(&a, &g, &temp);
        
        struct Acceleration_Data gyro_data;
        /* 
        * Read accelerations on all axes
         * */
        gyro_data.ax = a.acceleration.x;
        gyro_data.ay = a.acceleration.y;
        gyro_data.az = a.acceleration.z;

        // FILTER THIS READINGS

        /* send data to gyroscope queue */
        if(xQueueSend(gyroscope_data_queue, &gyro_data, portMAX_DELAY) != pdPASS){
            debugln("[-]Gyro queue full");
        }

        delay(TASK_DELAY);
    }
}

#ifdef DISPLAY_DATA_DEBUG
void displayData(void* pvParameters){
   while(true){
       struct Acceleration_Data gyroscope_buffer;
       struct Altimeter_Data altimeter_buffer;

       if(xQueueReceive(gyroscope_data_queue, &gyroscope_buffer, portMAX_DELAY) == pdPASS){
           debugln("------------------------------");
            debug("x: "); debug(gyroscope_buffer.ax); debugln();
            debug("y: "); debug(gyroscope_buffer.ay); debugln();
            debug("z: "); debug(gyroscope_buffer.az); debugln();

       }else{
           /* no queue */
       }

        if(xQueueReceive(altimeter_data_queue, &altimeter_buffer, portMAX_DELAY) == pdPASS){
            debug("Pressure: "); debug(altimeter_buffer.pressure); debugln();
            debug("Altitude: "); debug(altimeter_buffer.altitude); debugln();
            debug("Velocity: "); debug(altimeter_buffer.velocity); debugln();
            debug("AGL: "); debug(altimeter_buffer.AGL); debugln();
           
        }else{
            /* no queue */
        }

       delay(10);
   }
}
#endif

#ifdef TRANSMIT_TELEMETRY_DEBUG
void transmitTelemetry(void* pvParameters){
    /* This function sends data to the ground station */

     /*  create two pointers to the data structures to be transmitted */
    
    char telemetry_data[20];
    struct Acceleration_Data gyroscope_data_receive;
    struct Altimeter_Data altimeter_data_receive;

    while(true){    
        
        /* receive data into respective queues */
        if(xQueueReceive(gyroscope_data_queue, &gyroscope_data_receive, portMAX_DELAY) == pdPASS){
            debugln("[+]Gyro data ready for sending ");
        }else{
            debugln("[-]Failed to receive gyro data");
        }

        if(xQueueReceive(altimeter_data_queue, &altimeter_data_receive, portMAX_DELAY) == pdPASS){
            debugln("[+]Altimeter data ready for sending ");
        }else{
            debugln("[-]Failed to receive altimeter data");
        }

        sprintf(telemetry_data,
                "%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %i \n",
                gyroscope_data_receive.ax,
                gyroscope_data_receive.ay,
                gyroscope_data_receive.az,
                altimeter_data_receive.AGL,
                altimeter_data_receive.altitude,
                altimeter_data_receive.velocity,
                altimeter_data_receive.pressure
            );

        if(mqtt_client.publish("n3/telemetry", telemetry_data)){
            debugln("[+]Data sent");
        } else{
            debugln("[-]Data not sent");
        }

    }
}

void reconnect(){
    /**
     * This function is used to re-establish connection to the MQTT server during flight
     * in case of a broken connection
     * 
    */

    while(!mqtt_client.connected()){
        debug("[..]Attempting MQTT connection...");
        String client_id = "[+]FC Client - ";
        client_id += String(random(0XFFFF), HEX);

        if(mqtt_client.connect(client_id.c_str())){
            debugln("[+]MQTT connected");
        }
        // todo: else
    }
}

#endif

#ifdef MQTT_TEST_DEBUG
void testMQTT(void *pvParameters){
    while(true){
        debugln("Publishing data");
        if(mqtt_client.publish("n3/telemetry", "Hello from flight!")){
            debugln("Data sent");
        }else{
            debugln("Unable to send data");
        }
    }
}
#endif

void counter_update(void* pvParameters){
    /* DEBUG
     * This function updates a counter variable to simulate different flight states
     * 
     * The counter will be typically be a value like altitude in real-life that changes with time
     * Depending on the value of altitude, we can know the different states of flight
     * 
     * This function simulates a 4 sec delay between each flight state
     */

    while(true){
        counter += 1;

        /* push counter to flight states queue -  this is just a simulation */
        if(xQueueSend(flight_states_queue, &counter, portMAX_DELAY) != pdPASS){
            debugln("[-]Failed to add counter to queue");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void flight_state_check(void* pvParameters){
    /* Test the Finite State Machine that will be used to notify ground station about flight events */
 
    while(true){
        int32_t flight_state;

        /* print the current flight state */
        if(xQueueReceive(flight_states_queue, &flight_state, portMAX_DELAY)){
            debugln("flight state received");

            /* simulate first state */
            switch (flight_state){
                case 0:
                    state = fsm.pre_flight();
                    debugln("PRE-FLIGHT"); debug(state); debugln();
                    break;

                case 1:
                    state = fsm.powered_flight();
                    debugln("POWERED FLIGHT:"); debug(state); debugln();
                    break;

                case 2:
                    state = fsm.coasting();
                    debugln("COASTING:"); debug(state); debugln();
                    break;

                case 3:
                    state = fsm.apogee();
                    debugln("APOGEE:"); debug(state); debugln();
                    break;

                case 4:
                    state = fsm.ballistic_descent();
                    debugln("BALLISTIC DESCENT:"); debug(state); debugln();
                    break;

                case 5:
                    state = fsm.parachute_deploy();
                    debugln("PARACHUTE DEPLOY:"); debug(state); debugln();
                    break;

                case 6:
                    state = fsm.post_flight(); 
                    debugln("POST FLIGHT:"); debug(state); debugln();
                    break;
                
                default:
                    break;

            }
            

        }

    }
}

void setup(){
    /* initialize serial */
    Serial.begin(115200);


    /* DEBUG: set up state simulation leds */
    for(auto pin: state_leds){
        pinMode(state_leds[pin], OUTPUT);
    }

    /* connect to WiFi*/
    connectToWifi();

    /* initialize sensors */
    initialize_gyroscope();
    initialize_altimeter();
    // todo: initialize flash memory

    // mqtt_client.setBufferSize(MQTT_BUFFER_SIZE);
    mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);

    debugln("Creating queues...");

    /* create gyroscope data queue */
    gyroscope_data_queue = xQueueCreate(GYROSCOPE_QUEUE_LENGTH, sizeof(struct Acceleration_Data));

    /* create altimeter_data_queue */   
    altimeter_data_queue = xQueueCreate(ALTIMETER_QUEUE_LENGTH, sizeof(struct Altimeter_Data));

    /* create queue to hols all the sensor's data */
    telemetry_data_queue = xQueueCreate(ALL_TELEMETRY_DATA_QUEUE_LENGTH, sizeof(struct Telemetry_Data));

    /* this queue will hold the flight states */
    flight_states_queue = xQueueCreate(FLIGHT_STATES_QUEUE_LENGTH, sizeof(int32_t));


    /* check if the queues were created successfully */
    if(gyroscope_data_queue == NULL){
        debugln("[-]Gyroscope data queue creation failed!");
    } else{
        debugln("[+]Gyroscope data queue creation success");
    }
    
    if(altimeter_data_queue == NULL){
        debugln("[-]Altimeter data queue creation failed!");
    } else{
        debugln("[+]Altimeter data queue creation success");
    }

    if(filtered_data_queue == NULL){
        debugln("[-]Filtered data queue creation failed!");
    } else{
        debugln("[+]Filtered data queue creation success");
    }

    if(flight_states_queue == NULL){
        debugln("[-]Flight states queue creation failed!");
    } else{
        debugln("[+]Flight states queue creation success");
    }

    /* Create tasks
     * All tasks have a stack size of 1024 words - not bytes!
     * ESP32 is 32 bit, therefore 32bits x 1024 = 4096 bytes
     * So the stack size is 4096 bytes
     * */
    debugln("Creating tasks...");

    /* TASK 1: READ ALTIMETER DATA */
   if(xTaskCreate(
           readAltimeter,               /* function that executes this task*/
           "readAltimeter",             /* Function name - for debugging */
           STACK_SIZE,                  /* Stack depth in words */
           NULL,                        /* parameter to be passed to the task */
           2,        /* Task priority - in this case 1 */
           NULL                         /* task handle that can be passed to other tasks to reference the task */
   ) != pdPASS){
    // if task creation is not successful
    debugln("[-]Read-Altimeter task creation failed!");

   }else{
    debugln("[+]Read-Altimeter task creation success");
   }

    /* TASK 2: READ GYROSCOPE DATA */
   if(xTaskCreate(
           readGyroscope,         
           "readGyroscope",
           STACK_SIZE,                  
           NULL,                       
           2,
           NULL    
   ) != pdPASS){
    debugln("[-]Read-Gyroscope task creation failed!");
   } else{
    debugln("[+]Read-Gyroscope task creation success!");
   }

    #ifdef DISPLAY_DATA_DEBUG
    /* TASK 3: DISPLAY DATA ON SERIAL MONITOR - FOR DEBUGGING */
    if(xTaskCreate(
            displayData,
            "displayData",
            STACK_SIZE,
            NULL,
            2,
            NULL
            ) != pdPASS){
        debugln("[-]Display data task creation failed!");
        }else{
        debugln("[+]Display data task creation success!");
        }
    #endif

    #ifdef MQTT_TEST_DEBUG
    /* TASK 4: TRANSMIT TELEMETRY DATA */
    if(xTaskCreate(
            transmitTelemetry,
            "transmit_telemetry",
            STACK_SIZE,
            NULL,
            2,
            NULL
    ) != pdPASS){
        debugln("[-]Transmit task failed to create");
    }else{
        debugln("[+]Transmit task created success");
    }
    #endif

    // if(xTaskCreate(
    //         testMQTT,
    //         "testMQTT",
    //         STACK_SIZE,
    //         NULL,
    //         1,
    //         NULL
    // ) != pdPASS){
    //     debugln("[-]Test mqtt task failed to create");
    // }else{
    //     debugln("[+]Test mqtt task created success");
    // }

    if(xTaskCreate(
            flight_state_check,
            "testFSM",
            STACK_SIZE,
            NULL,
            2,
            NULL
    ) != pdPASS){
        debugln("[-]FSM task failed to create");
    }else{
        debugln("[+]FSM task created success");
    }

    #ifdef FSM_COUNTER_DEBUG
    if(xTaskCreate(
            counter_update,
            "counter_update",
            STACK_SIZE,
            NULL,
            2,
            NULL
    ) != pdPASS){
        debugln("[-]Counter update task failed to create");
    }else{
        debugln("[+]Counter task created success");
    }
    #endif

}

void loop(){

    #ifdef MQTT_TEST_DEBUG
    if(!mqtt_client.connected()){
        /* try to reconnect if connection is lost */
        reconnect();
    }

    mqtt_client.loop();
    #endif


}