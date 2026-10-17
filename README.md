# Kartodrom Gostyń – generator toru dla Assetto Corsa

Data-driven generator: oś trasy + szerokość + wysokości → siatki 3D (asfalt, krawężniki, pobocza, bandy, teren)
→ FBX dla ksEditor + struktura moda. Status: **grywalne MVP** (geometria z OSM, część parametrów tymczasowa).

## Wymagania
- macOS lub Windows: Python 3.9+ (tylko stdlib), Blender 4.x/5.x (testowane 5.2.2).
  Ścieżka Blendera: zmienna `BLENDER`, albo `blender` w PATH, albo `/Applications/Blender.app`.
- Windows: Assetto Corsa (ksEditor jest w `assettocorsa\sdk\editor\ksEditor.exe`).

## Użycie
```
python3 test_trackgen.py        # self-check geometrii (bez Blendera)
python3 build.py validate       # geometria + walidacja -> build/mesh.json, build/preview.svg
python3 build.py all            # walidacja + FBX (Blender headless) + pliki moda
python3 build.py import-osm     # ponowne pobranie osi z OSM (nie nadpisze pliku bez --force)
```
Wynik w `build/`:
```
build/preview.svg                         podgląd z góry (asfalt, krawężniki, bandy, spawny)
build/fbx/kartodrom_gostyn.fbx            + texture/*.png  -> do ksEditor
build/fbx/kartodrom_gostyn.blend          do podglądu w Blenderze
build/content/tracks/kartodrom_gostyn/    models.ini, data/surfaces.ini, ui/ui_track.json
```

## Dane wejściowe (`data/`) – geometrię poprawia się tylko tutaj
- `centerline.csv` – oś trasy `x,y,z,width` w metrach (x wschód, y północ). Puste `z`/`width` = 0 / `default_width_m`.
  Punkty w kolejności jazdy, pętla zamknięta niejawnie (bez powtórzonego pierwszego punktu).
- `track.json` – parametry. Klucze z listy `placeholder` to **wartości tymczasowe**, nie pomiary.

| Dane | Wartość | Źródło |
|---|---|---|
| długość | 800 m | kartodrom-gostyn.pl/tor |
| szerokość | 7–10 m | kartodrom-gostyn.pl/tor |
| zakręty | 15 | kartodrom-gostyn.pl/tor |
| oś toru | 133 pkt, 776 m po wygładzeniu | OSM ways 378990687 + 558923489, © OpenStreetMap contributors, ODbL |
| szerokość lokalna | 8 m wszędzie | **PLACEHOLDER** (środek zakresu) |
| wysokości | płasko | **PLACEHOLDER** – docelowo NMT 1 m z Geoportalu (GUGiK) |
| kierunek jazdy | odwrotnie do kolejności OSM (`reverse: true`), zgodnie z ruchem wskazówek zegara | wiedza lokalna autora projektu |
| start/meta | długa prosta wschodnia, w stronę nawrotu | wiedza lokalna; dokładny punkt `start_xy` **PLACEHOLDER** |
| krawężniki, pobocza, bandy, pola startowe | patrz `track.json` | **PLACEHOLDER** |

Ślad OSM ma kilka długich odcinków bez punktów (walidacja pokazuje je jako `input gap`) – sprawdź je na ortofotomapie
i dopisz punkty w `centerline.csv`.

## Walidacja
Błąd (build przerwany): długość poza 800 m ±5%, szerokość poza 7–10 m, nieciągłość próbek, dziury w asfalcie,
przecinające się krawędzie asfaltu/band, odwrócone lub zdegenerowane ściany, >65534 wierzchołków na obiekt (limit KN5).
Ostrzeżenie: odchyłka długości >1%, przerwy >20 m między punktami wejściowymi, liczba wykrytych zakrętów vs 15,
lista użytych wartości tymczasowych.

Pobocza i bandy są automatycznie przycinane do wolnej przestrzeni (ciasne nawroty, sąsiednie odcinki toru),
więc nie zachodzą na asfalt.

## Windows: FBX → KN5 → gra
1. Skopiuj `build/fbx/` (FBX + `texture/`) i `build/content/` na PC.
2. ksEditor → File → Import → `kartodrom_gostyn.fbx`.
3. Materiały (`road`, `kerb`, `grass`, `wall`, `ground`): shader `ksPerPixel`, `txDiffuse` = plik z `texture/`.
4. File → Save kn5 → Track → `content\tracks\kartodrom_gostyn\kartodrom_gostyn.kn5`.
5. Skopiuj `content\tracks\kartodrom_gostyn` do folderu AC i uruchom Practice.

Checklist pierwszego przejazdu: rozmiar toru realny (nie 100×), spawn na torze i przodem do kierunku jazdy,
czas okrążenia liczony na linii AC_TIME_0, trawa zwalnia, bandy zatrzymują.

Znane niewiadome do sprawdzenia w grze:
- auto obrócone tyłem → `"spawn_flip": true` w `track.json`, przebuduj;
- tor 100× za duży/mały → zmień `apply_scale_options` w `blender_build.py`;
- tor leży na boku / obiekty mają −90° na osi X → dodaj `bake_space_transform=True` do eksportu w `blender_build.py`;
- fizyka nawierzchni w `surfaces.ini` – wartości wyjściowe do strojenia.

## Nazewnictwo (AC)
Obiekty fizyczne: `1ROAD`, `1KERB`, `1GRASS`, `1WALL`, `2GRASS` (teren). Empties: `AC_START_n`, `AC_PIT_n`,
`AC_HOTLAP_START_0`, `AC_TIME_0_L/R` (oś Z = kierunek jazdy, Y = góra).

## Poza MVP
Linia AI (`ai/fast_lane.ai`, nagrywana w grze), `map.png`/`ui/preview.png`/`outline.png`, kamery, wysokości z NMT,
pit lane, skróty z OSM jako osobne layouty, prawdziwe tekstury, otoczenie (opony, trybuny, drzewa).
