# Transformons l'ESP32 en serveur web

Maintenant que vous avez vu le fonctionnement global d'un serveur web, nous pouvons transformer l'ESP en serveur. Pour commencer, il faut initaliser tous les paramètres du réseau sur lequel vous allez connecter la carte dans le fichier `platformio.ini`.

[TODO] image

## Lancer le serveur web

Le code pour lancer les commandes web se trouvent dans `lib/wifiTools.cpp`. 

Pour démarrer le serveur, vous devez appeler la fonction `initWifi(wifiSsid, wifiPwd);` dans votre code comme suit.

```cpp
// récupère les fonction de wifiTools
#include "wifiTools.hpp"

// récupère les valeurs fournies dans platformio.ini
char wifiSsid[50] = WIFISSID;
char wifiPwd[50] = WIFIPWD;

void setup() {
    initWifi(wifiSsid, wifiPwd);
}

void loop() {
  // Vérifie la connection au wifi
  refreshWifi();
}
```

## Configurer les fonctions du serveur web
