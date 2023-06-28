# Créer un serveur web avec l'ESP32

Il est possible de créer un serveur web avec l'ESP32 pour rendre les capteurs contrôlables depuis un site web ou bien une application mobile.

Pour comprendre le fonctionnement d'un serveur web, référez vous aux schémas dans le fichier [ServeurWebSchema.pdf](./ServeurWebSchema.pdf)

Pour faire ceci, le code de création du serveur vous est fourni dans `/lib/wifiTools.cpp`, vous n'avez pas besoin d'y toucher. Pour l'utiliser et créer votre serveur web, exécutez le code suivant dans le fichier `main.cpp`.

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include "../lib/wifiTools.cpp"

void setup()
{
  // Code exécuté une seule fois au démarrage
  Serial.begin(115200);
  initWifi("codingRoom1", "abcd123456789");
  Serial.print("Connectez vous a : ");
  Serial.println(getIpAdresse());
}

void loop()
{
  // Code effectué en boucle
  refreshWifi();
}
```

Si vous copiez-collez l'adresse où vous connecter dans un navigateur, vous devriez voir une page s'afficher avec **Projet Ada Lovelace** dessus. 

Si vous souhaitez maintenant créer des interfaces comme expliquées dans les diapos présentées précédemment avec des `GET` et `POST`, il vous faudra procéder comme suit : 

Les opérations se font en appelant la fonction server.on importé de `wifiTools`

Typiquement :

```cpp
server.on('/chemin', fonctionAExecuter)
```

Par exemple, si on souhaite récupérer la température, on peut mettre le code de récupération de la température dans une fonction et l'appeler de la manière suivante dans le fichier `main.cpp`

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include "../lib/wifiTools.cpp"

#define DHT_PIN 18
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

void recupererTemperature()
{
  if (isnan(dht.readTemperature()) == 0)
  {
    float temp = dht.readTemperature();
    char temperature[10];
    // On transforme la température (float) en chaine de caractères
    sprintf(temperature, "%f°C", temp); 
    Serial.println(temp);
    server.send(200, "text/plain", temperature);
  }
  else
  {
    Serial.println("Erreur de lecture de la température");
    server.send(200, "text/plain", "Erreur de lecture de la température");
  }
}

void setup()
{
  // Code exécuté une seule fois au démarrage
  Serial.begin(115200);
  dht.begin();
  if (initWifi("codingRoom1", "abcd123456789"))
  {
    Serial.print("Connectez vous a : ");
    Serial.println(getIpAdresse());
  }

  server.on("/temperature", recupererTemperature);
}

void loop()
{
  // Code effectué en boucle
  refreshWifi();
}
```

On peut aussi faire tourner la tête du Super Codeur à l'aide d'un servoMoteur, il nous suffit d'utiliser le code suivant dans le fichier `main.cpp`.

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Servo.h>
#include "../lib/wifiTools.cpp"

#define SERVO 23

Servo myservo;
int localPositionServoMoteur;

void tournerDroite()
{
  Serial.println("Je tourne l tête à droite");
  localPositionServoMoteur -= 45;
  myservo.write(localPositionServoMoteur);
  server.send(302, "text/plain", "");
}

void tournerGauche()
{
  Serial.println("Je tourne la tête à gauche");
  localPositionServoMoteur += 45;
  myservo.write(localPositionServoMoteur);
  server.send(302, "text/plain", "");
}

void setup()
{
  // Code exécuté une seule fois au démarrage
  Serial.begin(115200);
  if (initWifi("codingRoom1", "abcd123456789"))
  {
    Serial.print("Connectez vous à : ");
    Serial.println(getIpAdresse());
  }

  myservo.attach(SERVO);
  localPositionServoMoteur = 0;
  myservo.write(localPositionServoMoteur);
  server.on("/tournerDroite", tournerDroite);
  server.on("/tournerGauche", tournerGauche);
}

void loop()
{
  // Code effectué en boucle
  refreshWifi();
}
```