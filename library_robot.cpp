#include "library_robot.h"

void setup()
{
    // Setup pin out dan lain lain pada void setup arduino
    Serial.begin(9600);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(pwm3, OUTPUT);
    pinMode(pwm4, OUTPUT);

    pinMode(sPin[0], INPUT);
    pinMode(sPin[1], INPUT);
    pinMode(sPin[2], INPUT);
    pinMode(sPin[3], INPUT);
    pinMode(sPin[4], INPUT);
    pinMode(sPin[5], INPUT);
    pinMode(sPin[6], INPUT);

    pinMode(buzzer, OUTPUT);
    pinMode(led1, OUTPUT);
    pinMode(led2, OUTPUT);

    // Void setup saat di Arduino IDE
    initlibraryrobot();
}

void buzzeron(int jumlah)
{
    for (int a = 0; a <= jumlah; a++)
    {
        digitalWrite(buzzer, HIGH);
        digitalWrite(led1, LOW);
        digitalWrite(led2, HIGH);
        delay(300);
        digitalWrite(buzzer, LOW);
        digitalWrite(led1, HIGH);
        digitalWrite(led2, LOW);
        delay(300);
    }
}

// Program bermain lampu sensor
void lampusensor()
{
    pinMode(sPin[0], OUTPUT);
    pinMode(sPin[1], OUTPUT);
    pinMode(sPin[2], OUTPUT);
    pinMode(sPin[3], OUTPUT);
    pinMode(sPin[4], OUTPUT);
    pinMode(sPin[5], OUTPUT);

    for (int i = 0; i <= 5; i++)
    {
        digitalWrite(sPin[i], LOW);
        delay(50);
        digitalWrite(sPin[i], HIGH);
        delay(50);
    }
    for (int i = 5; i >= 0; i--)
    {
        digitalWrite(sPin[i], LOW);
        delay(50);
        digitalWrite(sPin[i], HIGH);
        delay(50);
    }
}

// read new sensor
int caseSensor()
{
    int sensor[7];
    int casesensor = 0;

    sensor[0] = analogRead(sPin[0]);
    sensor[0] = (sensor[0] > 220) ? 1 : 0;
    sensor[1] = analogRead(sPin[1]);
    sensor[1] = (sensor[1] > 220) ? 1 : 0;
    sensor[2] = analogRead(sPin[2]);
    sensor[2] = (sensor[2] > 220) ? 1 : 0;
    sensor[3] = analogRead(sPin[3]);
    sensor[3] = (sensor[3] > 220) ? 1 : 0;
    sensor[4] = analogRead(sPin[4]);
    sensor[4] = (sensor[4] > 220) ? 1 : 0;
    sensor[5] = analogRead(sPin[5]);
    sensor[5] = (sensor[5] > 220) ? 1 : 0;
    sensor[6] = analogRead(sPin[6]);
    sensor[6] = (sensor[6] > 220) ? 1 : 0;

    // Serial.print(sensor[0]);
    // Serial.print("-");
    // Serial.print(sensor[1]);
    // Serial.print("-");
    // Serial.print(sensor[2]);
    // Serial.print("-");
    // Serial.print(sensor[3]);
    // Serial.print("-");
    // Serial.print(sensor[4]);
    // Serial.print("-");
    // Serial.print(sensor[5]);
    // Serial.print("-");
    // Serial.println(sensor[6]);

    casesensor = (sensor[6] * 1) + (sensor[5] * 2) + (sensor[4] * 4) + (sensor[3] * 8) + (sensor[2] * 16) + (sensor[1] * 32) + (sensor[0] * 64);
    return casesensor;
}

// sampling data sensor
int samplingSensor()
{
    int sampling = 0;
    sampling = caseSensor();
    int error = 0;
    switch (sampling)
    {
    case 0b0000000:
        error = 1;
        break;
    case 0b1000000:
        error = -60;
        break;
    case 0b1100000:
        error = -50;
        break;
    case 0b0100000:
        error = -40;
        break;
    case 0b0110000:
        error = -30;
        break;
    case 0b0010000:
        error = -20;
        break;
    case 0b0011000:
        error = -10;
        break;
    case 0b0001000:
        error = 0;
        break;
    case 0b0001100:
        error = 10;
        break;
    case 0b0000100:
        error = 20;
        break;
    case 0b0000110:
        error = 30;
        break;
    case 0b0000010:
        error = 40;
        break;
    case 0b0000011:
        error = 50;
        break;
    case 0b0000001:
        error = 60;
        break;
    }
    return error;
}

// pid
float kp = 0.00;
float ki = 0.0;
float kd = 0.00;
void pidvalue(float kp_, float ki_, float kd_)
{
    kp = kp_;
    kd = kd_;
    ki = ki_;
}

// line follower baru
void lf_newcross(int speed_)
{
    float errorBf, error, lasterror = 0, sumerror = 0;
    float KP = kp, KI = ki, KD = kd;
    float P, I, D, out;
    float speedKa, speedKi;
    bool isFind = false;
    bool errorB = false;

    while (!isFind)
    {
        error = samplingSensor();
        errorB = detectcross();
        if (errorB == 1)
            isFind = true;
        if (error != 0)
            sumerror += error;
        else
            sumerror = 0;

        P = error * KP;
        D = (error - lasterror) * KD;
        I = sumerror * KI;
        out = P + I + D;

        error = lasterror;

        speedKi = speed_ + out;
        speedKa = speed_ - out;

        if (speedKi >= 255)
            speedKi = 255;
        if (speedKa >= 255)
            speedKa = 255;
        if (speedKi <= -255)
            speedKi = -255;
        if (speedKa <= -255)
            speedKa = -255;

        if (error > 2)
        {
            belokirimaju(speed_ + out);
        }
        else if (error < -2)
        {
            belokananmaju(speed_ + (out * -1));
        }
        else
        {
            testkeduamotor(speed_, speed_);
        }
    }
}

// Program membaca sensor dan menampilkan nilai di Serial monitor
void readsensor()
{
    int readPin[6];
    // kiri
    readPin[0] = analogRead(sPin[0]);
    readPin[0] = (readPin[0] > 350) ? 1 : 0;
    readPin[1] = analogRead(sPin[1]);
    readPin[1] = (readPin[1] > 350) ? 1 : 0;
    readPin[2] = analogRead(sPin[2]);
    readPin[2] = (readPin[2] > 350) ? 1 : 0;
    // kanan
    readPin[3] = analogRead(sPin[3]);
    readPin[3] = (readPin[3] > 350) ? 1 : 0;
    readPin[4] = analogRead(sPin[4]);
    readPin[4] = (readPin[4] > 350) ? 1 : 0;
    readPin[5] = analogRead(sPin[5]);
    readPin[5] = (readPin[5] > 350) ? 1 : 0;

    Serial.print(readPin[0]);
    Serial.print(readPin[1]);
    Serial.print(readPin[2]);
    Serial.print(readPin[3]);
    Serial.print(readPin[4]);
    Serial.print(readPin[5]);
    Serial.println();
}
// Kirim Serial
void kirimdata(int data)
{
    Serial.print(data);
}

// check complete
void ifReadcond(int data)
{
    bool isComplete = false;
    String _str;
    char mystr[10];
    int _strint = 0;
    while (!isComplete)
    {
        testkeduamotor(0, 0);
        while (Serial.available())
        {
            _str = Serial.readString();
            _strint = _str.toInt();
            Serial.print(_str);
        }
        if (_strint == data)
            isComplete = true;
    }
    isComplete = false;
}

void checkComplete()
{
    bool isComplete = false;
    String _str;
    char mystr[10];
    while (!isComplete)
    {
        testkeduamotor(0, 0);
        while (Serial.available())
        {
            _str = Serial.readString();
            Serial.print(_str);
        }
        if (_str == "0")
            isComplete = true;
    }
    isComplete = false;
}

// Program membaca garis dengan salah satu sensor yang di pilih dengan input nomor pin
int satusensor(int pin)
{
    int hasilbaca;
    hasilbaca = analogRead(sPin[pin]);
    if (hasilbaca > 350)
        hasilbaca = 1;
    return hasilbaca;
}

// Program untuk mendeteksi perempatan dengan membaca sensor paling kanan dan kiri
int detectcross()
{
    int hasilbaca;
    if (analogRead(sPin[0]) > 350 && analogRead(sPin[5]) > 350)
    {
        hasilbaca = 1;
    }
    else
    {
        hasilbaca = 0;
    }
    return hasilbaca;
}

// Program menjalankan motor bagian kanan dengan input speed
void motorjalankanan(int8_t speed)
{
    if (speed >= 0)
    {
        analogWrite(pwm1, speed);
        analogWrite(pwm2, 0);
    }
    if (speed < 0)
    {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, speed * -1);
    }
}

// Program menjalankan motor bagian kiri dengan input speed
void motorjalankiri(int8_t speed)
{
    if (speed >= 0)
    {
        analogWrite(pwm3, speed);
        analogWrite(pwm4, 0);
    }
    if (speed < 0)
    {
        analogWrite(pwm3, 0);
        analogWrite(pwm4, speed * -1);
    }
}

// Program menjalankan motor bagian kanan dengan input speed
void testmotorkanan(int8_t speed)
{
    motorjalankanan(speed);
}

// Program menjalankan motor bagian kiri dengan input speed
void testmotorkiri(int8_t speed)
{
    motorjalankiri(speed);
}

// Program menjalankan kedua motor dengan input speed kiri dan kanan
void testkeduamotor(int8_t speed_kiri, int8_t speed_kanan)
{
    motorjalankanan(speed_kanan);
    motorjalankiri(speed_kiri);
}

// Program belok kiri dengan input speed
void belokiri(int8_t speed)
{
    motorjalankanan(speed);
    motorjalankiri(-speed);
}

// Program belok kanan dengan input speed
void belokanan(int8_t speed)
{
    motorjalankanan(-speed);
    motorjalankiri(speed);
}

// Program belok kanan dengan speed kiri = 2x speed kanan
void belokirimaju(int8_t speed)
{
    motorjalankanan(speed);
    motorjalankiri(speed - (speed / 2));
}

// Program belok kiri dengan speed kanan = 2x speed kiri
void belokananmaju(int8_t speed)
{
    motorjalankanan(speed - (speed / 2));
    motorjalankiri(speed);
}

// Program motor maju dengan input speed dan timeout
void majutimer(int8_t speed, int delay_)
{
    bool isTimeout = false;
    unsigned long timeStart = millis();

    while (!isTimeout)
    {
        testkeduamotor(speed, speed);
        if ((unsigned long)millis() - timeStart > delay_)
        {
            isTimeout = true;
            testkeduamotor(0, 0);
        }
    }
}

// Program belok kanan mencari garis dengan input speed dan pin sensor yang akan mendeteksi
void linefindkanan(int8_t speed, int sensor)
{
    bool isFound = false;
    while (!isFound)
    {
        int pin = satusensor(sensor);
        belokanan(speed);
        if (pin == 1)
        {
            isFound = true;
        }
    }
    testkeduamotor(0, 0);
}

// Program belok kiri mencari garis dengan input speed dan pin sensor yang akan mendeteksi
void linefindkiri(int8_t speed, int sensor)
{
    bool isFound = false;
    while (!isFound)
    {
        int pin = satusensor(sensor);
        belokiri(speed);
        if (pin == 1)
        {
            isFound = true;
        }
    }
    testkeduamotor(0, 0);
}

// Program mencari error dari nilai pembacaan sensor
float sampling()
{
    int readPin[6];
    int ska = 0, ski = 0, error = 0;
    // kiri
    // kiri
    readPin[0] = analogRead(sPin[0]);
    readPin[0] = (readPin[0] > 350) ? 1 : 0;
    readPin[1] = analogRead(sPin[1]);
    readPin[1] = (readPin[1] > 350) ? 1 : 0;
    readPin[2] = analogRead(sPin[2]);
    readPin[2] = (readPin[2] > 350) ? 1 : 0;
    // kanan
    readPin[3] = analogRead(sPin[3]);
    readPin[3] = (readPin[3] > 350) ? 1 : 0;
    readPin[4] = analogRead(sPin[4]);
    readPin[4] = (readPin[4] > 350) ? 1 : 0;
    readPin[5] = analogRead(sPin[5]);
    readPin[5] = (readPin[5] > 350) ? 1 : 0;
    // data sensor
    // Serial.print(readPin[0]);
    // Serial.print(readPin[1]);
    // Serial.print(readPin[2]);
    // Serial.print(readPin[3]);
    // Serial.print(readPin[4]);
    // Serial.println(readPin[5]);
    // error
    ska = readPin[5] * 2 + readPin[4] * 4 + readPin[3] * 8;
    ski = readPin[0] * 2 + readPin[1] * 4 + readPin[2] * 8;

    error = ska - ski;
    // Serial.println(error);
    return error;
}

// Program line follower dengan delay timeout
void lf_delay(int8_t speed, int delay_)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.0, KD = 0.1;
    bool isFind = false;
    int find;
    unsigned long timeStart = millis();

    while (!isFind)
    {
        find = detectcross();
        if (find == 1 || (unsigned long)millis() - timeStart > delay_)
        {
            isFind = true;
        }
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;

        // Serial.println(errt1);
        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
    testkeduamotor(0, 0);
}

void lf_crossfindArah(int8_t speed, bool arah)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 1, KI = 0.00, KD = 5;
    bool isFind = false;
    int find;

    while (!isFind)
    {
        find = detectcross();
        if (find == 1)
        {
            isFind = true;
        }
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;
        // Serial.println(errt1);
        if (errt1 < -2)
        {
            belokananmaju((speed * -1) + out);
        }
        else if (errt1 > 2)
        {
            belokirimaju((speed * -1) + (out * -1));
        }
        else
        {
            testkeduamotor((speed * -1), (speed * -1));
        }
    }
    testkeduamotor(0, 0);
}
// program line follower mencari persimpangan
void lf_crossfind(int8_t speed)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.0, KD = 0.1;
    bool isFind = false;
    int find;

    while (!isFind)
    {
        find = detectcross();
        if (find == 1)
        {
            isFind = true;
        }
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;
        // Serial.println(errt1);
        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
    testkeduamotor(0, 0);
}

void lf_crsfindskip(int8_t speed, bool arah, int skip)
{
    bool isCom = false;
    int skipC = 0;
    while (!isCom)
    {
        lf_crossfind(speed);
        skipC++;
        delay(50);
        if (skipC == skip)
            isCom = true;
    }
    testkeduamotor(0, 0);
}

// Program line follower
void linefollower(int8_t speed)
{
    float errt1, errt0 = 0, Interr = 0;
    float KP = 0.15, KI = 0.0, KD = 0.1;
    while (true)
    {
        errt1 = sampling();
        if (errt1 != 0)
            Interr += errt1;
        else
            Interr = 0;

        float p = errt1 * KP;
        float d = (errt1 - errt0) * KD;
        float i = Interr * KI;
        float out = p + i + d;

        errt0 = errt1;

        if (errt1 > 2)
        {
            belokananmaju(speed + out);
        }
        else if (errt1 < -2)
        {
            belokirimaju(speed + (out * -1));
        }
        else
        {
            testkeduamotor(speed, speed);
        }
    }
}
// Akhir dari library