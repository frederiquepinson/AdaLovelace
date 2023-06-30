# Autres capteurs 

## Jouer de la musique (MP3)

Nous allons utiliser un module DFPLayer afin de lire des MP3 depuis une carte micro-SD.

![dfplayer](https://ae01.alicdn.com/kf/HTB1O54OFuySBuNjy1zdq6xPxFXaT/A14-10-pi-ces-SAMIORE-ROBOT-Mini-lecteur-MP3-Module-TF-carte-U-disque-Mini-lecteur.jpg)

Les chansons doivent être placées dans un répertoire nommé `mp3` sur la carte micro-SD et doivent être renommées afin qu'elles commencent par quatre chiffres (indiquant l'ordre de lecture). Les chansons suivantes ont été installées sur votre carte micro-SD :
    
Chanson 0001 : 
* Titre:  Broke Inside My Mind (feat Ellie Griffiths)
* Auteur: Anitek
* Source: [https://soundcloud.com/anitek](https://soundcloud.com/anitek)
* Licence: [CC BY-NC-ND](http://creativecommons.org/licenses/by-nc-nd/3.0/deed.fr)
* Téléchargement (6MB): [https://www.auboutdufil.com/index.php?id=472](https://www.auboutdufil.com/index.php?id=472)

Chanson 0002 :
* Titre:  Summer Spliffs
* Auteur: Broke For Free
* Source: [http://brokeforfree.bandcamp.com/](http://brokeforfree.bandcamp.com/)
* Licence: [CC BY](https://creativecommons.org/licenses/by/3.0/deed.fr)
* Téléchargement (9MB): [https://www.auboutdufil.com/index.php?id=495](https://www.auboutdufil.com/index.php?id=495)

Chanson 0003 :
* Titre:  Barefoot Girl Pebble Road
* Auteur: Twizzle
* Source: [http://www.myspace.com/twizzlesizzles](http://www.myspace.com/twizzlesizzles)
* Licence: http://creativecommons.org/licenses/by-nc/2.0/deed.fr
* Téléchargement (4MB): [https://www.auboutdufil.com/index.php?id=432](https://www.auboutdufil.com/index.php?id=432)

Chanson 0004 : 
* Titre:  Final
* Auteur: K Soviet
* Source: [http://ksoviet.blogspot.com](http://ksoviet.blogspot.com)
* Licence: [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/4.0/deed.fr)
* Téléchargement (5MB): [https://www.auboutdufil.com/index.php?id=484](https://www.auboutdufil.com/index.php?id=484)

Chanson 0005 : 
* Titre:  Plastic Submarine
* Auteur: The Grammar Club
* Source: [https://www.facebook.com/TheGrammarClub](https://www.facebook.com/TheGrammarClub)
* Licence: [CC BY-NC-SA](https://creativecommons.org/licenses/by-nc-sa/3.0/deed.fr)
* Téléchargement (10MB): [https://www.auboutdufil.com/index.php?id=478](https://www.auboutdufil.com/index.php?id=478)


Insérez la carte micro-SD dans le module.

Une nouvelle fois, une bibliothèque a été rajoutée dans votre fichier de configuration.


![dfplayer2](https://raw.githubusercontent.com/DFRobot/DFRobotMediaWikiImage/master/Image/miniplayer_pin_map.png)


**Attention** : Aidez-vous du schéma ci-dessus pour faire les branchements. Faites toutefois attention, les broches de connexions sont en dessous du module si vous le mettez dans même position que sur le schéma.

Récupérez le cable avec la résistance intégrée et branchez le entre la broche **S** du port **11** et la broche **RX** du module.

Détachez un groupe de 3 fils :
* Branchez un fil entre la broche **S** du port RX2 et la broche **TX** du module
* Branchez un fil entre la broche **G** du port RX2 et la broche **GND**  du module
* Branchez un fil entre la broche **V** du port RX2 et la broche **VCC**  du module
* Branchez un fil entre ma broche **S** du port TX2 et la proche **RX**  du module

Pour les enceintes, détachez un groupe de 2 fils  :
* Branchez un fil entre une des broche côté enceinte et la broche **SPK_1** du module
* Branchez un fil entre l'autre broche côté enceinte et la broche **SPK_2** du module

Téléversez le programme suivant :

```C
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <Wire.h>

#define DFPLAYER_RX 16 // RX2
#define DFPLAYER_TX 17 // TX2

SoftwareSerial dfplayer(DFPLAYER_RX, DFPLAYER_TX); // RX, TX
DFRobotDFPlayerMini mp3;

String line;

String getKey(String str)
{
  return str.substring(0, str.indexOf('='));
}

String getValue(String str)
{
  if (str.indexOf('='))
  {
    return str.substring(str.indexOf('=') + 1);
  }
  else
  {
    return "";
  }
}

void menu_opcoes()
{
  Serial.println();
  Serial.println(F("================================================================================================================"));
  Serial.println(F("Commands:"));
  Serial.println(F(" [1-3] Sélectionner le fichier mp3"));
  Serial.println(F(" [s] stop"));
  Serial.println(F(" [p] play/pause"));
  Serial.println(F(" [+ or -] baisser/augmenter le volume"));
  Serial.println(F(" [< or >] piste précédente/suivante"));
  Serial.println();
  Serial.println(F("================================================================================================================"));
}

char command;
int pausa = 0;

void setup()
{
  Serial.begin(115200);
  dfplayer.begin(9600);

  Serial.println(mp3.begin(dfplayer));

  if (!mp3.begin(dfplayer))
  { // Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true)
    {
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  mp3.volume(1);
}

void loop()
{

  // Waits for data entry via serial
  while (Serial.available() > 0)
  {
    command = Serial.read();

    if ((command >= '1') && (command <= '9'))
    {
      Serial.print("Music reproduction");
      Serial.println(command);
      command = command - 48;
      mp3.play(command);
      menu_opcoes();
    }

    // Reproduction
    // Stop

    if (command == 's')
    {
      mp3.stop();
      Serial.println("Music Stopped!");
      menu_opcoes();
    }

    // Play/pause
    if (command == 'p')
    {
      pausa = !pausa;
      if (pausa == 0)
      {
        Serial.println("Continue...");
        mp3.start();
      }

      if (pausa == 1)
      {
        Serial.println("Music Paused!");
        mp3.pause();
      }

      menu_opcoes();
    }

    // Increases volume
    if (command == '+')
    {
      mp3.volumeUp();
      Serial.print("Current volume:");
      Serial.println(mp3.readVolume());
      menu_opcoes();
    }

    if (command == '<')
    {
      mp3.previous();
      Serial.println("Previous:");
      Serial.print("Current track:");
      Serial.println(mp3.readCurrentFileNumber() - 1);
      menu_opcoes();
    }

    if (command == '>')
    {
      mp3.next();
      Serial.println("next:");
      Serial.print("Current track:");
      Serial.println(mp3.readCurrentFileNumber() + 1);
      menu_opcoes();
    }

    // Decreases volume
    if (command == '-')
    {
      mp3.volumeDown();
      Serial.print("Current Volume:");
      Serial.println(mp3.readVolume());
      menu_opcoes();
    }
  }
}
```

Vous pouvez contrôler la lecture avec les commandes suivantes depuis le terminal :
* [1-3] Sélectionner le fichier mp3 a lancer
* [s] stop
* [p] play/pause
* [+ or -] baisser/augmenter le volume
* [< or >] piste précédente/suivante

---