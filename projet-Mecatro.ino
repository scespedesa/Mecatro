// === Bibliothèques nécessaires ===
#include "MecatroUtils.h"
#include "Wire.h"
#include "sensorbar.h"
#include "AS5600.h"                          // Pour les encodeurs
#include "SparkFun_I2C_Mux_Arduino_Library.h" // Pour le multiplexeur I2C

// === Définition des constantes ===
#define CONTROL_LOOP_PERIOD 5                // Période de la boucle de contrôle en ms
#define WIFI_SSID "ArduinoMecatroGr5"        // SSID du WiFi
#define WIFI_PASSWRD "password1234"          // Mot de passe WiFi

// === Déclaration des variables globales ===
// Variables pour le fonctionnement général
float Ur, Ul, rAngle, rCumulative, rSpeed, lAngle, lCumulative, lSpeed;
float Vr, Vl;
bool serial = true;
long previusMillis = 0;


// Constantes PID
float kpt = 0.3077, kit = 6.807, kdt = 0.0;  // Translation
float kpr1 = 5.5, kir = 0.008, kdr = 21.5;   // Rotation

// Variables du contrôleur pour le moteur droit
float Us_eq = 0.0, offset_rAng = 0.0;
float Us_p = 0, Us_i = 0, Us_ip = 0, Us_d = 0, Us_dp = 0, Us_pid = 0, Us_pid_reel = 0;
float N = 50, Es = 0, Esp = 0, rdutycycle = 0, rPWM = 0;

// Variables du contrôleur pour le moteur gauche
float Ur_eq = 0.0, Ur_p1 = 0, Ur_i = 0, Ur_ip = 0, Ur_d = 0, Ur_dp = 0, Ur_pid = 0, Ur_pid_reel = 0;
float ldutycycle = 0, lPWM = 0; 
float Ts=5.0;//5ms

// Propriétés physiques
float r_roue = 0.04, l_dis = 0.257;
float delta_u = 0, delta_r = 0, delta_phi = 0;
float delta_rp=0,delta_phip=0;
float pos = 0, posp = 0;

// Variables pour le filtre
float Bk = 0;
int k = 0;

// Variables pour les vitesses angulaires
float angleActuel1, anglePrecedent1, vitesseAngulaire1 = 0.0;
unsigned long temp_prec1 = 0;
float angleActuel2, anglePrecedent2, vitesseAngulaire2 = 0.0;
unsigned long temp_prec2 = 0;

// === Capteurs et périphériques ===
const uint8_t SX1509_ADDRESS = 0x3E;        // Adresse I2C du capteur de ligne
SensorBar mySensorBar(SX1509_ADDRESS);      // Capteur de ligne
QWIICMUX multiplexer;                       // Multiplexeur I2C
AS5600 rightEncoder(&Wire1);                // Encodeur droit
AS5600 leftEncoder(&Wire1);                 // Encodeur gauche

// === Setup ===
void setup()
{
    Serial.begin(230400);                   // Initialiser la communication série
    Wire1.begin();                          // Initialiser le bus I2C secondaire
    if (serial && !multiplexer.begin(0x70, Wire1)) {
        Serial.println("Erreur : Multiplexeur I2C introuvable.");
        return;
    }

    bool isInit = true;

    // Configuration des encodeurs
    multiplexer.setPort(6);
    rightEncoder.begin();
    if (!rightEncoder.isConnected()) {
        Serial.println("Erreur : Encodeur droit non connecté.");
        isInit = false;
    }

    multiplexer.setPort(7);
    leftEncoder.begin();
    if (!leftEncoder.isConnected()) {
        Serial.println("Erreur : Encodeur gauche non connecté.");
        isInit = false;
    }

    if (isInit) {
        Wire1.setClock(400000);             // Configurer la vitesse I2C à 400 kHz
        mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }

    // Configuration du capteur de ligne
    mySensorBar.setBarStrobe();
    mySensorBar.clearInvertBits();
    if (!mySensorBar.begin()) {
        Serial.println("Échec de communication avec le capteur de ligne.");
        while (1);
    }
}


void loop()
{
    mecatro::run();
}

// Fonction de filtre moyenne glissante
float filtreMoyenneGlissante(float encodeur) {
    Bk = (float(k) / (k + 1)) * Bk + (1.0 / (k + 1)) * encodeur;
    k++;
    return Bk;
}

void reconstruireX() {
    delta_u = r_roue / 2 * (vitesseAngulaire1 + vitesseAngulaire2);
    delta_r = r_roue / l_dis * (vitesseAngulaire1 - vitesseAngulaire2);
    delta_phi = delta_phi + delta_r * Ts / 1000;
    position();
}

void controlePID() {
    reconstruireX();
    Es = 0.56 - delta_u; // 0.5

    Us_p = kpt * Es;
    Us_i = Us_ip + kit * (Ts / 1000) * Esp;
    Us_d = kdt * (Es - Esp);
    Us_pid = Us_p + Us_i + Us_d;
    Esp = Es;
    Us_ip = Us_i;
    Us_dp = Us_d;
    Us_pid_reel = Us_eq + Us_pid;

    Ur_p1 = kpr1 * (-pos / 100);
    if (-30 > pos || pos > 30) {
        Ur_i = Ur_ip + kir * (Ts / 1000) * posp; 
    } else {
        Ur_i = 0;
    }
    Ur_d = kdr * (-pos / 100 - posp);
    Ur_pid = Ur_p1 + Ur_i + Ur_d;
    Ur_pid_reel = Ur_eq + Ur_pid;

    delta_rp = delta_r;
    delta_phip = delta_phi;
    posp = -pos / 100;
    Ur_ip = Ur_i;
    Ur_dp = Ur_d;

    rPWM = (Us_pid_reel + Ur_pid_reel) / 2;
    lPWM = (Us_pid_reel - Ur_pid_reel) / 2;

    rdutycycle = min(max(rPWM, -1), 1); // -1 à 1
    ldutycycle = min(max(lPWM, -1), 1); // -1 à 1

    mecatro::setMotorDutyCycle(rdutycycle, ldutycycle);
}

float fenetre1[5];
int indiceFenetre1 = 0;
float sommeFenetre1 = 0;

float filtreMoyenneMobile1(float nouvelleValeur) {
    sommeFenetre1 -= fenetre1[indiceFenetre1];
    fenetre1[indiceFenetre1] = nouvelleValeur;
    sommeFenetre1 += nouvelleValeur;
    indiceFenetre1 = (indiceFenetre1 + 1) % 5;
    return sommeFenetre1 / 5;
}

const int tailleFenetre = 5;
float fenetre2[tailleFenetre];
int indiceFenetre2 = 0;
float sommeFenetre2 = 0;

float filtreMoyenneMobile2(float nouvelleValeur) {
    sommeFenetre2 -= fenetre2[indiceFenetre2];
    fenetre2[indiceFenetre2] = nouvelleValeur;
    sommeFenetre2 += nouvelleValeur;
    indiceFenetre2 = (indiceFenetre2 + 1) % tailleFenetre;
    return sommeFenetre2 / tailleFenetre;
}

int i = 0;
float alpha = 0.9;

float filtreMoyenneExponentielle(float nouvelleValeur, float Vi) {
    Vi = alpha * nouvelleValeur + (1 - alpha) * Vi;
    return Vi;
}

float trouverVitesse1(float angle1) {
    unsigned long temp_1 = millis();
    if (temp_1 - temp_prec1 >= CONTROL_LOOP_PERIOD) {
        angleActuel1 = filtreMoyenneExponentielle(angle1, angleActuel1);
        float deltaTemps = (temp_1 - temp_prec1) / 1000.0;
        vitesseAngulaire1 = filtreMoyenneMobile1((angleActuel1 - anglePrecedent1) / deltaTemps);
        anglePrecedent1 = angleActuel1;
        temp_prec1 = temp_1;
    }
    return vitesseAngulaire1;
}

float trouverVitesse2(float angle2) {
    unsigned long temp_2 = millis();
    if (temp_2 - temp_prec2 >= CONTROL_LOOP_PERIOD) {
        angleActuel2 = filtreMoyenneExponentielle(angle2, angleActuel2);
        float deltaTemps = (temp_2 - temp_prec2) / 1000.0;
        vitesseAngulaire2 = filtreMoyenneMobile2((angleActuel2 - anglePrecedent2) / deltaTemps);
        anglePrecedent2 = angleActuel2;
        temp_prec2 = temp_2;
    }
    return vitesseAngulaire2;
}

void position() {
    pos = (mySensorBar.getPosition() * 105 / 254); // en mètres
}

void mecatro::controlLoop() {
    float angle1;
    float angle2;
    multiplexer.setPort(6);
    angle1 = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
    vitesseAngulaire1 = trouverVitesse1(angle1);
    multiplexer.setPort(7);
    angle2 = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
    vitesseAngulaire2 = trouverVitesse2(angle2);

    unsigned long tempsActuel = millis(); // Temps actuel en millisecondes
    uint8_t valeurBrute = mySensorBar.getRaw();

    controlePID();
}
