#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define SERVO_PIN PB1  // Assuming the servo is connected to pin PB1 (OC1A)
#define TRIGGER_PIN PD7
#define ECHO_PIN PD6

#define F_CPU 16000000UL // Define CPU frequency as 16 MHz
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
//Serial.begin(9600)
// PID Controller Structure
typedef struct {
    double Kp;  // Proportional gain
    double Ki;  // Integral gain
    double Kd;  // Derivative gain
    double previous_error;  // Previous error for derivative term
    double integral;  // Integral of errors for integral term
} PIDController;

// Function to initialize the PID controller
void initializePID(PIDController* pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->previous_error = 0.0;
    pid->integral = 0.0;
}

//-------namal
void updatePID(PIDController* pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
   
}
//----------

// Function to compute the output of the PID controller
double computePID(PIDController* pid, double setpoint, double measured_value, double dt) {
    double error = setpoint - measured_value;
    double Pout = pid->Kp * error;
    pid->integral += error * dt;
    double Iout = pid->Ki * pid->integral;
    double derivative = (error - pid->previous_error) / dt;
    double Dout = pid->Kd * derivative;
    pid->previous_error = error;
    double output = Pout + Iout + Dout;
    return output;
}

// Function to initialize the servo motor
void servo_init() {
    DDRB |= (1 << SERVO_PIN);  // Set SERVO_PIN as output
    TCCR1A |= (1 << WGM11) | (1 << COM1A1); // Fast PWM, non-inverting mode
    TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11); // Fast PWM, prescaler 8
    ICR1 = 19999;  // Set TOP to 20ms (50Hz PWM frequency)
}

// Function to set the servo position
//void servo_set_position(uint16_t pos) //AKKU REMOVED
void servo_set_position(long pos) 
{
   // OCR1A = (pos * 15) + 1000; // Map position (0 to 180) to duty cycle (1ms to 2ms)
   OCR1A = (pos * 5.5) + 1500;
   
    // Serial.print("Position: ");
    // Serial.println(pos);

}

// USART Initialization
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Send data via USART
void USART_Transmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

// Send string via USART
void USART_SendString(const char *str) {
    while (*str) {
        USART_Transmit(*str++);
    }
}

// Send trigger pulse for ultrasonic sensor
void send_trigger_pulse() {
    PORTD |= (1 << TRIGGER_PIN); // Set TRIGGER_PIN high
    _delay_us(10); // Wait for 10 microseconds
    PORTD &= ~(1 << TRIGGER_PIN); // Set TRIGGER_PIN low
}

// Measure pulse width for ultrasonic sensor
long pulseInn(uint8_t pin, uint8_t state) {
    int width = 0;
    uint8_t bit = (1 << pin);
    while (!(PIND & bit)) {}
    while (PIND & bit) {
        _delay_us(10);
        width += 10;
    }
    return width; 
}

// Convert microseconds to centimeters
long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2;
}

//----------
// Function to initialize the ADC
void ADC_init() {
    // Select Vref=AVcc
    ADMUX = (1 << REFS0);
    
    // Enable ADC and set prescaler to 128
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Function to read the ADC value from a specific channel
uint16_t ADC_read(uint8_t channel) {
    // Select the ADC channel (0-7)
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
    
    // Start a single conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return the ADC value
    return ADC;
}



int main(void) {
 
 //char buffer[32]; 
 char buffer[40];

    // PID parameters
    PIDController pid;
  
   initializePID(&pid, 3, 0.05, 0.08); 
    int count = 0;
    // Desired ball position (setpoint)
    double setpoint = 20.0; // Adjust as needed

    // Initialize servo and ultrasonic sensor
    servo_init();
    DDRD |= (1 << TRIGGER_PIN);  // Set TRIGGER_PIN as output
    DDRD &= ~(1 << ECHO_PIN);    // Set ECHO_PIN as input
    USART_Init(MYUBRR);

    //uint16_t adc_value;
     ADC_init();



    long duration;
    double measured_value;
    long servo_position;

    while (1) {
  //------------------
   
        
         double kpf = ADC_read(0);
         double kp = (kpf/1023)*12;
         double kif = ADC_read(1);
         double ki = (kif/1023)*1;
         double kdf = ADC_read(2);
         double kd = (kdf/1023)*2;
         double setpointf = ADC_read(3);
         setpoint = (setpointf/1023)*42;
         updatePID(&pid, kp, ki, kd);

        //---------------------------

        // Measure ball position
        send_trigger_pulse();
        duration = pulseInn(ECHO_PIN, 1);
        measured_value = microsecondsToCentimeters(duration);

        // Calculate servo position using PID
        double dt = 0.01;  // Time step (e.g., 10ms)
        double pid_output = computePID(&pid, setpoint, measured_value, dt);

       servo_position = (long)pid_output;  

        // Constrain servo position within valid range
       // if (servo_position > 180) servo_position = 180;
       // if (servo_position < 0) servo_position = 0;

        // Move servo
        servo_set_position(-servo_position);    
      //  USART_SendString(buffer);
       // Serial.begin(9600);
        //Serial.println(measured_value);

        //_delay_ms(10);  // Small delay to prevent overloading the system
        _delay_ms(10); 
        count++;
        if (count==20)
        {
          Serial.println(measured_value);
          count = 0;
        }

    }
}