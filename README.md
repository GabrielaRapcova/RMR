# RMR

# 1.úloha: Lokalizácia a polohovanie robota v prostredí

## Lokalizácia
Pri lokalizácii robota sme využili gyroodometriu, teda kombináciu údajov z enkóderov kolies a gyroskopu robota Kobuki. V callback funkcii processThisRobot() sa z rozdielu hodnôt enkóderov vypočítala prejdená vzdialenosť ľavého a pravého kolesa. Následne sa pomocou gyroskopu určilo aktuálne natočenie robota fi. Poloha robota sa potom aktualizovala podľa vzťahov zadaných v prezentácií k úlohe.

Gyroskop bol použitý na spresnenie orientácie robota, čím sa znížila chyba klasickej odometrie spôsobená prešmykom kolies. Pri polohovaní robota sme implementovali jednoduchý regulátor rozdelený na:
  - rotáciu robota na požadovaný smer,
  - následný pohyb dopredu.

Robot najprv vypočíta uhlovú chybu medzi aktuálnym smerom a cieľovým bodom. Ak chyba presahuje zadaný threshold, aktivuje sa režim rotácie (rotateMode) a robot sa otáča na mieste. Po zarovnaní smeru začne translácia smerom k cieľu. Rýchlosť pohybu sa znižuje postupne  približovaním k cieľu, aby robot nezastavil prudko. Uhlová rýchlosť je riadená proporcionálne podľa chyby uhla.

# 3.úloha: Mapovanie

Mapovanie bolo realizované pomocou mriežky obsadenia (occupancy grid). Prostredie bolo rozdelené na štvorcovú mriežku s rozlíšením 0.05 m na bunku. Každá bunka obsahuje informáciu, či je voľná alebo obsadená prekážkou.

Dáta z lidaru sa spracovávajú vo funkcii processThisLidar(). Každý laserový bod sa transformuje zo súradnicového systému robota do globálnych súradníc pomocou:

aktuálnej polohy robota,
uhla natočenia robota,
vzdialenosti a uhla merania lidaru.

Transformované body sa následne zapisujú do mapy ako prekážky. Na vykreslenie voľného priestoru medzi robotom a prekážkou sme použili Bresenhamov algoritmus (drawLine()), ktorý označuje bunky medzi robotom a nameraným bodom ako voľné. Ak sa určitá bunka opakovane potvrdí meraniami, označí sa ako obsadená. Mapa sa dá uložiť do súboru pomocou saveMap() a načítať späť cez loadMap().

# 4.úloha: Plánovanie dráhy

Na plánovanie dráhy sme použili flood fill algoritmus. Pred samotným plánovaním sa najprv zväčšia prekážky pomocou funkcie inflateObstacles(), aby robot plánoval bezpečnú trasu s určitou rezervou od stien. Polomer inflácie bol zvolený podľa rozmerov robota.

Vo funkcii floodFill():

1. cieľová bunka dostane hodnotu 2,
2. susedné bunky dostávajú postupne vyššie hodnoty,
3. algoritmus pokračuje pomocou BFS (fronty), až kým neohodnotí celý dostupný priestor.

Následne sa vo funkcii extractPath() vytvorí výsledná trajektória. Robot začína v štartovacej bunke a vždy sa presunie do susednej bunky s najnižšou hodnotou, až kým nepríde do cieľa.

Kvôli jednoduchšiemu riadeniu robota sa cesta následne zredukovala iba na hlavné body zmeny smeru (mainpoints). Robot potom postupne naviguje medzi týmito bodmi pomocou implementovaného polohovania.

# 5.úloha: Pravdepodobnostná lokalizácia v známej mape

Pravdepodobnostná lokalizácia bola implementovaná pomocou Monte Carlo Localization (MCL). Robot využíva množinu častíc (particles), kde každá častica predstavuje možný stav robota (x, y, fi) s určitou váhou pravdepodobnosti.

Inicializácia častíc prebieha vo funkcii initializeParticles(), kde sa častice rozmiestnia okolo aktuálnej polohy robota s náhodným šumom.

Algoritmus pozostáva zo štyroch krokov:

1. Motion update
Vo funkcii motionUpdate() sa každá častica posunie podľa odometrie robota. Pohyb je doplnený o náhodný šum pomocou normálneho rozdelenia (sampleNormal()), čím sa simuluje nepresnosť pohybu.

2. Measurement update
Vo funkcii measurementUpdate() sa porovnávajú lidarové merania s mapou prostredia. Na tento účel sa vytvorilo distance field (computeDistanceField()), ktoré pre každú bunku obsahuje vzdialenosť od najbližšej prekážky. Častice, ktorých predikované merania lepšie zodpovedajú mape, získajú vyššiu váhu.

3. Resampling
Funkcia resampleParticles() realizuje ruletový výber častíc podľa ich váh. Pravdepodobnejšie častice majú väčšiu šancu zostať zachované. Súčasne sa malé percento častíc generuje náhodne, aby sa zabránilo uviaznutiu algoritmu v nesprávnom riešení.

4. Odhad polohy
Vo funkcii estimatePose() sa výsledná poloha robota vypočíta ako vážený priemer všetkých častíc. Orientácia sa počíta pomocou priemerovania sínusov a kosínusov uhlov.
