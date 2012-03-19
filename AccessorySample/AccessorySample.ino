#include <Servo.h>

#include <Max3421e.h>
#include <Usb.h>
#include <AndroidAccessory.h>

// pin definition
#define PWM01 8
#define PWM02 9
#define PWM03 10

#define SERVO1 11
#define SERVO2 12
#define SERVO3 13

#define DOUT01 A0
#define DOUT02 A1
#define DOUT03 6
#define DOUT04 7

#define AIN01 A2
#define AIN02 A3
#define AIN03 A4
#define AIN04 A5

#define __ARDUINO_MEGA__
#if defined(__ARDUINO_MEGA__)
#define DIN01 A6
#define DIN02 A7
#define DIN03 A8
#define DIN04 A9
#else
#define DIN01 2
#define DIN02 3
#define DIN03 4
#define DIN04 5
#endif

/*
 * protocol
 */
#define START_BYTE 0x7f
#define CMD_DIGITAL_WRITE 0x0
#define CMD_ANALOG_WRITE 0x01
#define CMD_SERVO 0x02
#define UPDATE_DIGITAL_STATE 0x40
#define UPDATE_ANALOG_STATE 0x41


AndroidAccessory acc("PIGMAL LLC",
		     "OpenAccessorySample",
		     "Android Open Accessory sample implementation",
		     "1.0",
		     "http://pigmal.com",
		     "0123456789");

Servo servos[3];
byte dins[4];

void initPins()
{
    // digital out
    pinMode(DOUT01, OUTPUT);
    pinMode(DOUT02, OUTPUT);
    pinMode(DOUT03, OUTPUT);
    pinMode(DOUT04, OUTPUT);
    digitalWrite(DOUT01, LOW);
    digitalWrite(DOUT02, LOW);
    digitalWrite(DOUT03, LOW);
    digitalWrite(DOUT04, LOW);

    // pwm out
    digitalWrite(PWM01, 0);
    digitalWrite(PWM02, 0);
    digitalWrite(PWM03, 0);
    pinMode(PWM01, OUTPUT);
    pinMode(PWM02, OUTPUT);
    pinMode(PWM03, OUTPUT);

    // digital in
    pinMode(DIN01, INPUT);
    pinMode(DIN02, INPUT);
    pinMode(DIN03, INPUT);
    pinMode(DIN04, INPUT);
    // enable the internal pullups
    digitalWrite(DIN01, HIGH);
    digitalWrite(DIN02, HIGH);
    digitalWrite(DIN03, HIGH);
    digitalWrite(DIN04, HIGH);

    // analog in
    pinMode(DIN01, INPUT);
    pinMode(DIN02, INPUT);
    pinMode(DIN03, INPUT);
    pinMode(DIN04, INPUT);

}



void setup()
{
    Serial.begin(115200);
    Serial.print("\r\nStart");

    initPins();

    servos[0].attach(SERVO1);
    servos[1].attach(SERVO2);
    servos[2].attach(SERVO3);
    servos[0].write(90);
    servos[1].write(90);
    servos[2].write(90);
    dins[0] = digitalRead(DIN01);
    dins[1] = digitalRead(DIN02);
    dins[2] = digitalRead(DIN03);
    dins[3] = digitalRead(DIN04);
    
    acc.powerOn();
}




void loop()
{
    byte err;
    byte idle;
    static byte count = 0;
    byte msg[4];
    long touchcount;

    if (acc.isConnected()) {
	int len = acc.read(msg, sizeof(msg), 1);
	int i;
	byte b;
	uint16_t val;
	int x, y;
	char c0;
	
	if (len >= 3 && msg[0] == START_BYTE) {
	    Serial.print("receive:");
	    Serial.print(msg[1], HEX);
	    Serial.print(msg[2], HEX);
	    Serial.print(msg[3], HEX);
	    Serial.print("\r\n");
	    // assumes only one command per packet
	    switch (msg[1]) {
	    case CMD_ANALOG_WRITE:
		if (len < 3 + msg[2]) break;
		switch (msg[3]) {
		case 0x0:
		    analogWrite(PWM01, 255 - msg[4]);
		    break;
		case 0x1:
		    analogWrite(PWM02, 255 - msg[4]);
		    break;
		case 0x2:
		    analogWrite(PWM03, 255 - msg[4]);
		    break;
		}
		break;
	    case CMD_DIGITAL_WRITE:
		if (len < 3 + msg[2]) break;
		switch (msg[3]) {
		case 0x0:
		    digitalWrite(DOUT01, msg[4] ? HIGH : LOW);
		    break;
		case 0x1:
		    digitalWrite(DOUT02, msg[4] ? HIGH : LOW);
		    break;
		case 0x2:
		    digitalWrite(DOUT03, msg[4] ? HIGH : LOW);
		    break;
		case 0x3:
		    digitalWrite(DOUT04, msg[4] ? HIGH : LOW);
		    break;
		}
		break;
	    case CMD_SERVO:
		if (len < 3 + msg[2]) break;
                servos[msg[3]].write(map(msg[4], 0, 255, 0, 180));
		break;
	    default:
		break;
	    }
	}
	
	/*
	 * send button status
	 */
	msg[0] = START_BYTE;
	msg[1] = UPDATE_DIGITAL_STATE;
	b = digitalRead(DIN01);
	if (b != dins[0]) {
	    msg[2] = 2; //size
	    msg[3] = 0; //id
	    msg[4] = b ? 0 : 1;
	    acc.write(msg, 5);
	    dins[0] = b;
	}
	b = digitalRead(DIN02);
	if (b != dins[1]) {
	    msg[2] = 2;
	    msg[3] = 1;
	    msg[4] = b ? 0 : 1;
	    acc.write(msg, 5);
	    dins[1] = b;
	}
	b = digitalRead(DIN03);
	if (b != dins[2]) {
	    msg[2] = 2;
	    msg[3] = 2;
	    msg[4] = b ? 0 : 1;
	    acc.write(msg, 5);
	    dins[2] = b;
	}
	b = digitalRead(DIN04);
	if (b != dins[3]) {
	    msg[2] = 2;
	    msg[3] = 3;
	    msg[4] = b ? 0 : 1;
	    acc.write(msg, 5);
	    dins[3] = b;
	}

	/*
	 * send analog status
	 */
	msg[1] = UPDATE_ANALOG_STATE;
	switch (count++ % 0x10) {
	case 0x0:
	    val = analogRead(AIN01);
	    msg[2] = 3;
	    msg[3] = 0x0;
	    msg[4] = val >> 8;
	    msg[5] = val & 0xff;
	    acc.write(msg, 6);
	    break;
	case 0x4:
	    val = analogRead(AIN02);
	    msg[2] = 3;
	    msg[3] = 0x1;
	    msg[4] = val >> 8;
	    msg[5] = val & 0xff;
	    acc.write(msg, 6);
	    break;
	case 0x8:
	    val = analogRead(AIN03);
	    msg[2] = 3;
	    msg[3] = 0x2;
	    msg[4] = val >> 8;
	    msg[5] = val & 0xff;
	    acc.write(msg, 6);
	    break;
	case 0xc:
	    val = analogRead(AIN04);
	    msg[2] = 3;
	    msg[3] = 0x3;
	    msg[4] = val >> 8;
	    msg[5] = val & 0xff;
	    acc.write(msg, 6);
	    break;
	default:
	    break;
	}
    } else {
	// reset outputs to default values on disconnect
	analogWrite(PWM01, 255);
	analogWrite(PWM02, 255);
	analogWrite(PWM03, 255);
	servos[0].write(90);
	servos[1].write(90);
	servos[2].write(90);
	digitalWrite(DOUT01, LOW);
	digitalWrite(DOUT02, LOW);
	digitalWrite(DOUT03, LOW);
	digitalWrite(DOUT04, LOW);
    }
    
    delay(10);
}

