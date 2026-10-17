# Kartodrom Gostyń – generator toru dla Assetto Corsa

Data-driven generator: oś trasy + szerokość + wysokości → siatki 3D (asfalt, krawężniki, pobocza, bandy, teren)
→ FBX dla ksEditor + struktura moda. Status: **MVP dopasowane do rzeczywistości** – oś i szerokość z ortofotomapy GUGiK,
wysokości z NMT GUGiK, bandy/krawężniki/start odrysowane ze zdjęcia; fizyka i część parametrów nadal tymczasowe.

## Wymagania
- macOS lub Windows: Python 3.9+ (tylko stdlib), Blender 4.x/5.x (testowane 5.2.2).
  Ścieżka Blendera: zmienna `BLENDER`, albo `blender` w PATH, albo `/Applications/Blender.app`.
- Windows: Assetto Corsa (ksEditor jest w `assettocorsa\sdk\editor\ksEditor.exe`).

## Użycie
```
python3 test_trackgen.py        # self-check geometrii (bez Blendera)
python3 build.py validate       # geometria + walidacja -> build/mesh.json, build/preview.svg
python3 build.py all            # walidacja + FBX (Blender headless) + pliki moda
```
Pozyskanie danych (jednorazowo, wynik jest w `data/` i w repo):
```
python3 build.py import-osm --force       # oś startowa z OSM
python3 build.py fetch-ortho              # ortofotomapa GUGiK -> build/cache/ortho.bmp (cache, poza repo)
python3 build.py fit --force              # oś + szerokość dopasowane do asfaltu na ortofotomapie
python3 build.py fetch-elevation          # wysokości NMT GUGiK -> z w centerline.csv + data/elevation.csv
```
Wynik w `build/`:
```
build/preview.svg                         podgląd z góry (asfalt, krawężniki, bandy, spawny)
build/ortho_overlay.svg                   model na ortofotomapie – główne narzędzie do sprawdzania danych
build/fbx/kartodrom_gostyn.fbx            + texture/*.png  -> do ksEditor
build/fbx/kartodrom_gostyn.blend          do podglądu w Blenderze
build/content/tracks/kartodrom_gostyn/    models.ini, data/surfaces.ini, ui/ui_track.json
```

## Dane wejściowe (`data/`) – geometrię poprawia się tylko tutaj
- `centerline.csv` – oś trasy `x,y,z,width` co 2 m (x wschód, y północ, z m n.p.m.). Pętla zamknięta niejawnie;
  kolejność jak w OSM, kierunek jazdy ustawia `reverse` w `track.json`.
- `elevation.csv` – siatka wysokości terenu co 4 m (NMT GUGiK).
- `features.json` – odrysowane z ortofotomapy: `start_finish` (linia szachownicy), `barriers` (bandy z opon, polilinie),
  `kerbs` (jasne pasy przy krawędzi – krawężnik powstaje tam, gdzie krawędź asfaltu jest bliżej niż `kerb_snap_m`),
  `extra_asphalt` (wypukłe wielokąty: skróty, łączniki, wjazdy z parkingu). Dokładność ~0,5–1 m.
- `track.json` – parametry. Klucze z listy `placeholder` to **wartości tymczasowe**, nie pomiary.

| Dane | Wartość | Źródło |
|---|---|---|
| długość | 800 m | kartodrom-gostyn.pl/tor |
| szerokość | 7–10 m | kartodrom-gostyn.pl/tor |
| zakręty | 15 | kartodrom-gostyn.pl/tor |
| oś toru | 782 m po dopasowaniu | OSM (ODbL) → dopasowane do ortofotomapy GUGiK (Geoportal.gov.pl) |
| szerokość lokalna | 6,6–10,4 m (mediana 7,1; prosta startowa ~10) | zmierzona na ortofotomapie |
| wysokości | 90,3–91,2 m n.p.m. (tor), teren 87–94 m | NMT GUGiK |
| kierunek jazdy | zgodnie z ruchem wskazówek zegara (`reverse: true`) | wiedza lokalna autora projektu |
| start/meta, bandy, krawężniki, dodatkowy asfalt | `features.json` | odrysowane z ortofotomapy |
| bandy: opony 0,6 m, czerwono-żółte; tarki: malowane czerwono-żółte | `track.json`, tekstury | szacunek ze zdjęć i nagrań onboard ([YouTube](https://www.youtube.com/@kartodromgostyn)) |
| rekord toru | 50,434 s (gokarty z wypożyczalni, 2025) | [kartodrom-gostyn.pl/rekordy](https://kartodrom-gostyn.pl/rekordy/) – cel przy strojeniu fizyki |
| pola startowe, boksy, fizyka nawierzchni | `track.json`, `surfaces.ini` | **PLACEHOLDER** |

## Walidacja
Błąd (build przerwany): długość poza 800 m ±5%, szerokość poza 5–13 m, nieciągłość próbek, dziury w asfalcie,
przecinające się krawędzie asfaltu/pobocza, banda na asfalcie, odwrócone lub zdegenerowane ściany,
>65534 wierzchołków na obiekt (limit KN5).
Ostrzeżenie: odchyłka długości >1%, szerokość poza oficjalnymi 7–10 m, przerwy >20 m między punktami wejściowymi,
nachylenie >8%, liczba wykrytych zakrętów vs 15, lista użytych wartości tymczasowych.

Pobocza są automatycznie przycinane do wolnej przestrzeni (ciasne nawroty, sąsiednie odcinki toru) i łagodnie
schodzą do poziomu terenu. Teren przy torze ma wysokość toru, dalej przechodzi w NMT. Na brzegu terenu stoi ściana.

## Windows: FBX → KN5 → gra
1. Skopiuj `build/fbx/` (FBX + `texture/`) i `build/content/` na PC.
2. ksEditor → File → Import → `kartodrom_gostyn.fbx`.
3. Materiały (`road`, `kerb`, `grass`, `wall`, `concrete`, `ground`): shader `ksPerPixel`, `txDiffuse` = plik z `texture/`.
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
Obiekty fizyczne: `1ROAD`, `2ROAD` (dodatkowy asfalt), `1KERB`, `1GRASS`, `1WALL` (opony), `2WALL` (mur na brzegu terenu),
`2GRASS` (teren). Empties: `AC_START_n`, `AC_PIT_n`,
`AC_HOTLAP_START_0`, `AC_TIME_0_L/R` (oś Z = kierunek jazdy, Y = góra).

## Poza MVP
Ortofotomapa jako tekstura terenu (następny krok), linia AI (`ai/fast_lane.ai`, nagrywana w grze),
`map.png`/`ui/preview.png`/`outline.png`, kamery, boksy na parkingu, prawdziwe tekstury, otoczenie (budynki, drzewa).

Źródła: ortofotomapa i NMT © GUGiK (Geoportal.gov.pl), oś początkowa © OpenStreetMap contributors (ODbL).
