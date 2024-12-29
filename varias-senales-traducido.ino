/* Code de démonstration pour lire les encodeurs du robot.

  Remarque : ce code nécessite les bibliothèques suivantes (installez-les via le gestionnaire de bibliothèques) :
     - SparkFun I2C Mux Arduino Library
     - AS5600 library
*/

// Inclusion de la bibliothèque actuelle
#include "MecatroUtils.h"

// Inclusion des bibliothèques AS5600 (pour les encodeurs) et SparkFun I2C Mux (pour le multiplexeur)
#include "AS5600.h"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

// Déclaration des variables globales
float Ur, Ul, Ar, Al, Vr, Vl, Bk;
int k = 0;

// Propriétés physiques
float r_roue = 0.04;  // Rayon des roues en mètres
float l_dis = 0.257;  // Distance entre les roues en mètres

float delta_u, delta_r, U_plus, U_moins;

float offset_lAng = 0.0;
float offset_rAng = 0.0;

// En-tête pour la communication I2C
#include "Wire.h"

// Définition de la période de la boucle de contrôle en millisecondes
#define CONTROL_LOOP_PERIOD 5

#define WIFI_SSID "ArduinoMecatroGr5"
#define WIFI_PASSWRD "password123"

long periodo = 2000.0;      // Période du signal en millisecondes 
float mini = 10.0;
float signalValue = 0.0;

unsigned long tiempoAnterior1 = 0;  // Variable pour stocker le temps précédent
float aumenta = 0;

// Déclaration du multiplexeur I2C
QWIICMUX multiplexer;

// Wire1 est utilisé pour le port Qwiic sur cette carte Arduino. 
// Tous les appareils doivent être assignés à ce Wire (soit ici (encodeurs) soit lors de l'initialisation (multiplexeur), selon les appareils).
AS5600 rightEncoder(&Wire1);
AS5600 leftEncoder(&Wire1);

void setup() {
  // Configuration de la communication série avec le PC (pour le débogage et l'enregistrement)
  Serial.begin(230400);
  // Démarrage de la communication I2C sur le port Qwiic
  Wire1.begin();

  // Initialisation du multiplexeur. Son adresse I2C est 0x70, et nous communiquons via le port Qwiic (Wire1).
  if (!multiplexer.begin(0x70, Wire1)) {
    Serial.println("Erreur : multiplexeur I2C non trouvé. Vérifiez le câblage.");
  } else {
    bool isInit = true;
    // Configuration du multiplexeur pour utiliser le port 0 pour communiquer avec l'encodeur droit
    multiplexer.setPort(6);
    rightEncoder.begin();
    offset_rAng = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
    if (!rightEncoder.isConnected()) {
      Serial.println("Erreur : impossible de se connecter à l'encodeur droit. Vérifiez le câblage.");
      isInit = false;
    }
    // Configuration du multiplexeur pour utiliser le port 3 pour communiquer avec l'encodeur gauche
    multiplexer.setPort(7);
    leftEncoder.begin();
    offset_lAng = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;
    if (!leftEncoder.isConnected()) {
      Serial.println("Erreur : impossible de se connecter à l'encodeur gauche. Vérifiez le câblage.");
      isInit = false;
    }

    if (isInit) {
      // Configuration de la vitesse d'horloge I2C à 400 kHz (mode rapide). À faire après l'initialisation pour éviter les réinitialisations.
      Wire1.setClock(400000);
      // Configuration de la commande des moteurs et de l'appel de la boucle de rétroaction.
      mecatro::configureArduino(CONTROL_LOOP_PERIOD);
    }
  }

  // Initialisation de la télémétrie
  unsigned int const nVariables = 12;
  String variableNames[nVariables] = {"Ur", "Ul", "U_plus", "U_moins", "delta_u", "delta_r", "Vr_f1", "Vl_f1", "Vr_f2", "Vl_f2", "Ar", "Al"};
  mecatro::initTelemetry(WIFI_SSID, WIFI_PASSWRD, nVariables, variableNames, CONTROL_LOOP_PERIOD);
}

void loop() {
  // Cette fonction est indispensable : sans elle, rien ne se passe !
  // Elle ne retourne jamais, placez tout votre code dans mecatro::controlLoop.
  mecatro::run();
}

float ventana1[10]; // Tableau pour stocker les dernières lectures
int indiceVentana1 = 0; // Indice actuel dans la fenêtre
float sumaVentana1 = 0; // Somme des valeurs dans la fenêtre

float filtroMediaMovil1(float valorNuevo) {
    // Soustrait la valeur la plus ancienne de la somme
    sumaVentana1 -= ventana1[indiceVentana1];
    
    // Remplace la valeur la plus ancienne par la nouvelle et l'ajoute à la somme
    ventana1[indiceVentana1] = valorNuevo;
    sumaVentana1 += valorNuevo;
    
    // Incrémente l'indice et le réinitialise s'il atteint la fin de la fenêtre
    indiceVentana1 = (indiceVentana1 + 1) % 10;
    
    // Calcule la moyenne
    return sumaVentana1 / 10;
}

const int ventanaTamano = 10; // Taille de la fenêtre de la moyenne mobile
float ventana2[ventanaTamano]; // Tableau pour stocker les dernières lectures
int indiceVentana2 = 0; // Indice actuel dans la fenêtre
float sumaVentana2 = 0; // Somme des valeurs dans la fenêtre

float filtroMediaMovil2(float valorNuevo) {
    // Soustrait la valeur la plus ancienne de la somme
    sumaVentana2 -= ventana2[indiceVentana2];
    
    // Remplace la valeur la plus ancienne par la nouvelle et l'ajoute à la somme
    ventana2[indiceVentana2] = valorNuevo;
    sumaVentana2 += valorNuevo;
    
    // Incrémente l'indice et le réinitialise s'il atteint la fin de la fenêtre
    indiceVentana2 = (indiceVentana2 + 1) % ventanaTamano;
    
    // Calcule la moyenne
    return sumaVentana2 / ventanaTamano;
}

// float alpha = 0.1; // Ajuste cette valeur entre 0 et 1 pour plus ou moins de lissage 0.0015 0.08-
int i=0;
float alpha=0.9;

float filtroMediaExponencial(float valorNuevo,float Vi) {
    // Applique un filtre de moyenne exponentielle
    Vi = alpha * valorNuevo + (1 - alpha) * Vi;
    return Vi;
}

// Variables pour le calcul de la vitesse
float anguloActual1; // Angle actuel (capteur 1)
float anguloAnterior1; // Angle précédent (capteur 1)
float velocidadAngular1 = 0.0; // Vitesse angulaire (capteur 1)
unsigned long temp_prev1 = 0; // Temps précédent (capteur 1)

// Variables pour le calcul de la vitesse
float anguloActual2; // Angle actuel (capteur 2)
float anguloAnterior2; // Angle précédent (capteur 2)
float velocidadAngular2 = 0.0; // Vitesse angulaire (capteur 2)
unsigned long temp_prev2 = 0; // Temps précédent (capteur 2)

float encontrarVelocidad1(float angulo1) {
    unsigned long temp_1 = millis();
    if (temp_1 - temp_prev1 >= CONTROL_LOOP_PERIOD) {
        // Applique un filtre exponentiel sur l'angle
        anguloActual1 = filtroMediaExponencial(angulo1, anguloActual1);
        // Calcule l'écart de temps (en secondes)
        float deltaTiempo = (temp_1 - temp_prev1) / 1000.0;
        // Calcule la vitesse angulaire
        velocidadAngular1 = (anguloActual1 - anguloAnterior1) / deltaTiempo;
        // Met à jour les valeurs précédentes
        anguloAnterior1 = anguloActual1;
        temp_prev1 = temp_1;
    }
    return velocidadAngular1;
}

float encontrarVelocidad2(float angulo2) {
    unsigned long temp_2 = millis();
    if (temp_2 - temp_prev2 >= CONTROL_LOOP_PERIOD) {
        // Applique un filtre exponentiel sur l'angle
        anguloActual2 = filtroMediaExponencial(angulo2, anguloActual2);
        // Calcule l'écart de temps (en secondes)
        float deltaTiempo = (temp_2 - temp_prev2) / 1000.0;
        // Calcule la vitesse angulaire
        velocidadAngular2 = (anguloActual2 - anguloAnterior2) / deltaTiempo;
        // Met à jour les valeurs précédentes
        anguloAnterior2 = anguloActual2;
        temp_prev2 = temp_2;
    }
    return velocidadAngular2;
}

// Cette fonction est appelée périodiquement, toutes les CONTROL_LOOP_PERIOD ms.
// Mettez tout votre code ici.
void mecatro::controlLoop() {
    float angulo1;
    float angulo2;

    multiplexer.setPort(6);

    // Récupère l'angle cumulatif de l'encodeur droit
    angulo1 = rightEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS;

    velocidadAngular1 = encontrarVelocidad1(angulo1) - offset_rAng;

    multiplexer.setPort(7);

    // Récupère l'angle cumulatif de l'encodeur gauche
    angulo2 = leftEncoder.getCumulativePosition() * AS5600_RAW_TO_RADIANS - offset_lAng;

    velocidadAngular2 = encontrarVelocidad2(angulo2);

    // Calculs pour les tensions des moteurs
    Ur = 1 * function();
    Ul = 1 * function();

    U_plus = Ur + Ul;
    U_moins = Ur - Ul;
    delta_u = r_roue / 2 * (velocidadAngular1 + velocidadAngular2);
    delta_r = r_roue / l_dis * (velocidadAngular1 - velocidadAngular2);

    // Enregistre les données pour le suivi
    mecatro::log(0, Ur);
    mecatro::log(1, Ul); 
    mecatro::log(2, U_plus); 
    mecatro::log(3, U_moins); 
    mecatro::log(4, delta_u); 
    mecatro::log(5, delta_r); 
    mecatro::log(6, velocidadAngular1); 
    mecatro::log(7, velocidadAngular2); 
    mecatro::log(8, filtroMediaMovil1(velocidadAngular1));
    mecatro::log(9, filtroMediaMovil2(velocidadAngular2)); 
    mecatro::log(10, anguloActual1); 
    mecatro::log(11, anguloActual2); 

    // Active des moteurs
    mecatro::setMotorDutyCycle(Ul, Ur);
}
float function() {
    unsigned long tiempoActual = millis();  // Obtient le temps actuel
    unsigned long tiempoTranscurrido = tiempoActual - tiempoAnterior1;

    // Change le signal toutes les 2 * période millisecondes
    if (tiempoTranscurrido <= 2 * periodo) {
      signalValue = creneau1(tiempoTranscurrido);
    } else if (tiempoTranscurrido <= 4 * periodo ) {
      signalValue = creneau0(tiempoTranscurrido);
    } else if (tiempoTranscurrido <= 5 * periodo ) {
      signalValue = 0;
    } else if (tiempoTranscurrido <= 6 * periodo ) {
      signalValue = sawtooth(tiempoTranscurrido);
    } else if (tiempoTranscurrido <= 7 * periodo ) {
      signalValue = 0;
    } else if (tiempoTranscurrido <= 8 * periodo ) {
      signalValue = senoNegativo(tiempoTranscurrido);
    } else if (tiempoTranscurrido <= 10 * periodo ) {
      signalValue = 0;
    } else {
        // Réinitialise le timer après avoir complété un cycle complet de signaux
        tiempoAnterior1 = tiempoActual;
    }

    return signalValue;
}

// Fonction de signal carré (niveau haut et bas)
float creneau0(unsigned long tiempoTranscurrido) {
    if ((tiempoTranscurrido / (periodo / 2)) % 2 == 0) {
        return 0.0;  // Niveau haut
    } else {
        return 1.0;  // Niveau bas
    }
}

// Fonction de signal carré (niveau positif et négatif)
float creneau1(unsigned long tiempoTranscurrido) {
    if ((tiempoTranscurrido / (periodo / 2)) % 2 == 0) {
        return -1.0;  // Niveau positif
    } else {
        return 1.0; // Niveau négatif
    }
}

// Fonction de signal en dent de scie
float sawtooth(unsigned long tiempoTranscurrido) {
    return float(tiempoTranscurrido % periodo) / (periodo);  // Augmente linéairement
}

// Signal sinusoïdal variant de 1 à -1
float senoNegativo(unsigned long tiempoTranscurrido) {
    float t = float(tiempoTranscurrido % periodo) / periodo;
    return sin(2 * PI * t);  // Variation de 1 à -1
}

// Fonction de signal sinusoïdal
float sinus(unsigned long tiempoTranscurrido) {
    unsigned long tiempoActual = millis();  // Obtient le temps actuel
    return sin(tiempoActual / 1000.0);  // Signal sinusoïdal
}

float creneaux() {
    unsigned long tiempoActual = millis();  // Obtient le temps actuel

    // Vérifie si la moitié de la période s'est écoulée
    if (tiempoActual - tiempoAnterior1 >= periodo / 2) {
        // Change l'état du signal
        signalValue = !signalValue; // Inverse le signe -signalValue
        tiempoAnterior1 = tiempoActual; // Met à jour le temps précédent
    }

    // Retourne la valeur actuelle du signal
    return signalValue;
}


