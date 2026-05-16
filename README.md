# RMR

# 1.úloha: Lokalizácia a polohovanie robota v prostredí

## Lokalizácia
Na lokalizáciu robota bola použitá gyroodometria, teda kombinácia:
- odometrie z enkóderov kolies,
- údajov z gyroskopu.

Robot priebežne vypočítava:
- polohu v osi X,
- polohu v osi Y,
- orientáciu robota.

Gyroskop slúži na spresnenie orientácie a znižuje chybu spôsobenú prešmykovaním kolies. Pri spracovaní LiDAR dát sa navyše používa interpolácia pozície podľa časových značiek meraní, aby boli laserové dáta priradené správnej polohe robota.

## Polohovanie
Úlohou polohovania je dostať robota do zadaných súradníc.
Riadenie pohybu je rozdelené na:
- rotáciu na mieste,
- transláciu smerom k cieľu.

Robot:
1. najprv vypočíta smer k cieľu,
2. otočí sa správnym smerom,
3. následne sa pohybuje dopredu.

Na reguláciu bol použitý proporcionálny regulátor:
- uhlová rýchlosť závisí od chyby natočenia,
- lineárna rýchlosť závisí od vzdialenosti od cieľa.

Pri približovaní ku cieľu sa rýchlosť automaticky znižuje, aby robot nepredbehol cieľ.
Implementované bolo aj:
- obmedzenie maximálnej rýchlosti,
- plynulé zrýchľovanie a spomaľovanie,
- hysterézne prepínanie medzi rotáciou a transláciou.

# 3.úloha: Mapovanie

Na mapovanie prostredia bola použitá occupancy grid mapa. Prostredie je rozdelené na malé bunky, pričom každá bunka obsahuje informáciu:
- voľný priestor,
- prekážka,
- neznáme miesto.

## Spracovanie LiDAR dát
LiDAR poskytuje vzdialenosti prekážok v rôznych smeroch okolo robota.
Každé meranie je:
1. prevedené zo súradnicového systému LiDARu,
2. transformované do globálneho súradnicového systému mapy.

Transformácia využíva:
- aktuálnu pozíciu robota,
- orientáciu robota,
- vzdialenosť a uhol merania.

## Aktualizácia mapy
Miesto dopadu laserového lúča sa označí ako prekážka. Priestor medzi robotom a prekážkou sa označuje ako voľný pomocou Bresenhamovho algoritmu. Na odstránenie šumu:
- prekážka musí byť potvrdená viacerými meraniami,
- nesprávne detekované prekážky sa môžu postupne odstrániť.

# 4.úloha: Plánovanie dráhy

Na plánovanie dráhy bol použitý algoritmus Flood Fill.
## Flood Fill algoritmus
Algoritmus začína od cieľovej bunky:
1. cieľ dostane hodnotu 2,
2. susedné bunky dostanú hodnotu 3,
3. ďalšie bunky pokračujú rastúcimi hodnotami.

Takto vznikne mapa vzdialeností od cieľa. Robot následne:
- začína od svojej aktuálnej polohy,
- vždy vyberá susednú bunku s najnižšou hodnotou,
- až kým nedosiahne cieľ.

Použitá bola 8-susednosť, ktorá umožňuje:
- diagonálny pohyb,
- kratšie trajektórie,
- prirodzenejší pohyb robota.

## Inflácia prekážok
Pred plánovaním sa realizuje inflácia prekážok. Keďže robot má nenulové rozmery, prekážky sa zväčšujú o bezpečnostnú vzdialenosť, aby:
- robot nenarazil do steny,
- trajektória neviedla príliš blízko prekážok.

## Generovanie trajektórie
Výsledná trajektória obsahuje množstvo bodov. Pre efektívnejší pohyb sa extrahujú iba hlavné body:
- zmena smeru,
- začiatok trajektórie,
- koniec trajektórie.
Robot sa následne pohybuje medzi týmito bodmi pomocou regulácie pohybu.

---

# 5.úloha: Pravdepodobnostná lokalizácia v známej mape

Na lokalizáciu v známej mape bol implementovaný algoritmus Monte Carlo Localization (MCL). MCL využíva množinu particles, pričom každá particle reprezentuje možnú polohu robota.

## Inicializácia particles
Na začiatku sa vytvorí množstvo particles:
- rozmiestnených okolo aktuálnej odometrickej polohy,
- s náhodným šumom,
- s počiatočnými pravdepodobnosťami.

## Pohybový model
Particles sa pohybujú podľa:
- odometrie robota,
- náhodného šumu.

Šum simuluje:
- nepresnosti pohybu,
- prešmykovanie kolies,
- chyby senzorov.
Každá particle je vyhodnotená podľa zhody LiDAR meraní s mapou. Používa sa distance field:
- každá bunka obsahuje vzdialenosť od najbližšej prekážky.

Particles, ktorých merania lepšie zodpovedajú mape:
- získajú vyššiu váhu,
- majú väčšiu pravdepodobnosť prežitia.

## Prevzorkovanie
Po vyhodnotení sa realizuje prevzorkovanie pomocou ruletového výberu. Particles s vyššou váhou:
- zostávajú zachované,
- menej presné particles zanikajú.

Malé množstvo nových náhodných particles sa generuje pre:
- zachovanie diverzity,
- schopnosť zotavenia po strate lokalizácie.
- 
## Výpočet výslednej polohy
Výsledná poloha robota sa vypočíta ako vážený priemer všetkých particles:
- priemer X,
- priemer Y,
- priemer orientácie.
