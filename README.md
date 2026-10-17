# Kartodrom Gostyń – tor do Assetto Corsa

Generator moda toru [Kartodrom Gostyń](https://kartodrom-gostyn.pl/) do Assetto Corsa (2014). Geometria powstaje
z danych, a nie z ręcznie modelowanej siatki. Oś i szerokość toru są zmierzone na ortofotomapie GUGiK, wysokości
pochodzą z NMT GUGiK, a bandy z opon, tarki i linię startu odrysowano ze zdjęcia. Python liczy siatki, Blender
uruchamiany z wiersza poleceń eksportuje FBX, a ksEditor (narzędzie z AC) zamienia go na plik toru KN5.

**Status:** kompletny tor gotowy do konwersji. **Nie był jeszcze testowany w grze** – patrz
[Pierwszy przejazd](#5-pierwszy-przejazd-w-ac). Fizyka nawierzchni ma wartości wyjściowe do strojenia.

| | |
|---|---|
| Długość | 778 m po osi (oficjalnie 800 m) |
| Szerokość | 6,6–10,4 m, prosta startowa ok. 10 m |
| Wysokość | 90,3–91,2 m n.p.m. |
| Kierunek | zgodnie z ruchem wskazówek zegara |
| Pola startowe / boksy | 10 / 10 |
| Rekord toru (gokarty z wypożyczalni) | 50,434 s – punkt odniesienia przy strojeniu fizyki |

---

## Szybki start

```
git clone https://github.com/Emenowicz/ac-gostyn.git
cd ac-gostyn
python3 build.py fetch-ortho     # jednorazowo: ortofotomapa (~33 MB) do tekstury terenu
python3 build.py all             # walidacja + FBX + pliki moda -> build/
```
Potem [konwersja w ksEditor](#4-konwersja-w-kseditor-tylko-windows) i [instalacja w AC](#5-pierwszy-przejazd-w-ac).

---

## 1. Wymagania

| Do czego | Co | Uwagi |
|---|---|---|
| budowanie toru (Mac lub Windows) | [Python](https://www.python.org/downloads/) 3.9+ | tylko biblioteka standardowa, bez `pip install` |
| eksport FBX | [Blender](https://www.blender.org/download/) 4.x lub 5.x | testowane na 5.2.2 |
| konwersja i gra (tylko Windows) | Assetto Corsa ze Steam | ksEditor jest w `assettocorsa\sdk\editor\ksEditor.exe` |

Generator szuka Blendera kolejno: w zmiennej `BLENDER`, jako `blender` w `PATH`, w `/Applications/Blender.app` (macOS).
Na Windowsie najprościej ustawić zmienną (ścieżka zależy od wersji):
```
$env:BLENDER = "C:\Program Files\Blender Foundation\Blender 4.2\blender.exe"   # PowerShell
```
Na Windowsie zamiast `python3` zwykle używa się `py` albo `python`.

## 2. Budowanie

```
python3 test_trackgen.py         # szybki test geometrii (bez Blendera, kilka sekund)
python3 build.py validate        # tylko geometria + walidacja + podglądy
python3 build.py all             # walidacja + FBX (Blender w tle) + pliki moda
```
Bez `fetch-ortho` tor też się zbuduje, ale teren dostanie jednolity kolor zamiast zdjęcia.

Wynik w `build/`:
```
build/fbx/kartodrom_gostyn.fbx            model dla ksEditor
build/fbx/texture/*.png                   tekstury (ortho.png = ortofotomapa 4096², ok. 16 MB)
build/fbx/kartodrom_gostyn.blend          ten sam model do obejrzenia w Blenderze
build/content/tracks/kartodrom_gostyn/    folder moda: models.ini, data/surfaces.ini, ui/ui_track.json
build/preview.svg                         podgląd z góry
build/ortho_overlay.svg                   model na ortofotomapie – otwórz w przeglądarce, żeby sprawdzić dane
```

## 3. Walidacja

`build.py` przerywa budowanie, gdy tor jest niegrywalny, i ostrzega, gdy coś wymaga spojrzenia człowieka.

- **Błąd:** długość poza 800 m ±5%, szerokość poza 5–13 m, dziura w asfalcie, przecinające się krawędzie,
  zawinięte pobocze, banda na asfalcie, odwrócona ściana, ponad 65 534 wierzchołków na obiekt (limit KN5).
- **Ostrzeżenie:** szerokość poza oficjalnymi 7–10 m, banda na tarce (chyba że oznaczona `on_kerb`), nachylenie
  powyżej 8%, promień krawędzi asfaltu poniżej 2,5 m, lista wartości tymczasowych.

## 4. Konwersja w ksEditor (tylko Windows)

1. Skopiuj na PC z Windowsem cały folder `build/fbx/` (FBX razem z `texture/`) oraz `build/content/`.
2. Uruchom `ksEditor.exe` i zaimportuj `kartodrom_gostyn.fbx` (menu File).
3. Dla każdego materiału ustaw shader `ksPerPixel` i jako `txDiffuse` wybierz plik o tej samej nazwie z `texture/`:

   | Materiał | Tekstura | Gdzie |
   |---|---|---|
   | `road` | `road.png` | asfalt, płyty dodatkowego asfaltu |
   | `kerb` | `kerb.png` | tarki (paski czerwono-żółte) |
   | `wall` | `wall.png` | bandy z opon |
   | `concrete` | `concrete.png` | mur na brzegu terenu |
   | `ortho` | `ortho.png` | teren i pobocze (ortofotomapa) |

4. Zapisz jako tor: File → Save kn5 → Track, pod nazwą `kartodrom_gostyn.kn5`, do folderu
   `content\tracks\kartodrom_gostyn\` (tego z `build/content/`).

Nazwy menu mogą się nieco różnić między wersjami ksEditor.

## 5. Pierwszy przejazd w AC

1. Skopiuj `content\tracks\kartodrom_gostyn` (z plikiem `.kn5` w środku) do `assettocorsa\content\tracks\`.
2. Uruchom AC → Practice → tor „Kartodrom Gostyń”.
3. Sprawdź:
   - [ ] tor ma prawdziwy rozmiar (nie 100× za duży lub za mały);
   - [ ] auto pojawia się na prostej startowej, przodem w kierunku nawrotu;
   - [ ] czas okrążenia liczy się na linii z szachownicą;
   - [ ] trawa zwalnia, bandy z opon i mur zatrzymują;
   - [ ] czas okrążenia gokartem jest w okolicy 50–52 s.

### Rozwiązywanie problemów

| Objaw | Poprawka |
|---|---|
| tor 100× za duży lub za mały | zmień `apply_scale_options` w `blender_build.py` (`FBX_SCALE_ALL` → `FBX_SCALE_UNITS`) |
| tor leży na boku, obiekty obrócone o −90° | dodaj `bake_space_transform=True` do eksportu w `blender_build.py` |
| auto stoi tyłem do kierunku jazdy | `"spawn_flip": true` w `data/track.json` |
| auto spada przez tor | sprawdź, czy obiekty w ksEditor mają nazwy `1ROAD`, `1GRASS` itd. (nazwa = fizyka) |
| za mała lub za duża przyczepność | `FRICTION` w `SURFACES` w `build.py` (z tego powstaje `data/surfaces.ini` w folderze moda) |

Po każdej zmianie: `python3 build.py all`, ponowny import w ksEditor i zapis KN5.

## 6. Poprawianie toru

Geometrię poprawia się wyłącznie w `data/`. Generator tylko czyta te pliki, więc nie trzeba niczego edytować ręcznie
w modelu. Po zmianie uruchom `python3 build.py validate` i obejrzyj `build/ortho_overlay.svg`.

| Plik | Zawartość |
|---|---|
| `data/centerline.csv` | oś toru `x,y,z,width` co 2 m; x – wschód, y – północ [m], z – m n.p.m.; pętla zamknięta niejawnie |
| `data/features.json` | `start_finish` (linia startu), `barriers` (bandy z opon), `kerbs` (tarki), `extra_asphalt` (skróty, wjazdy) |
| `data/elevation.csv` | siatka wysokości terenu co 4 m |
| `data/track.json` | parametry; lista `placeholder` to wartości tymczasowe, `sources` to źródła danych |

Współrzędne są w metrach względem punktu `origin_lat_lon` z `track.json`. Tarka powstaje tam, gdzie krawędź asfaltu
leży bliżej niż `kerb_snap_m` od linii z `kerbs`. Banda z `"on_kerb": true` może stać na tarce.

Ponowne pobranie danych źródłowych (zwykle niepotrzebne, wynik jest w repo):
```
python3 build.py import-osm --force       # oś początkowa z OpenStreetMap
python3 build.py fetch-ortho              # ortofotomapa GUGiK -> build/cache/ (poza repo)
python3 build.py fit --force              # oś + szerokość dopasowane do asfaltu na ortofotomapie
python3 build.py fetch-elevation          # wysokości NMT GUGiK
```
`fit` i `import-osm` nadpisują `data/centerline.csv` – najpierw zacommituj zmiany.

## 7. Jak to działa

```
data/*.csv, *.json ──> trackgen.py ──> build/mesh.json ──> blender_build.py ──> FBX ──> ksEditor ──> KN5
   (dane)              (siatki, UV,                       (Blender w tle:        (Windows)
                        walidacja)                          materiały, eksport)
```
- `trackgen.py` – spline przez oś, siatki asfaltu, tarek, pobocza, band i terenu, punkty AC, walidacja (czysty Python).
- `fit.py` – odczyt ortofotomapy i wykrywanie krawędzi asfaltu.
- `build.py` – wiersz poleceń: pobieranie danych, podglądy, pliki moda, uruchomienie Blendera.
- `blender_build.py` – uruchamiany w Blenderze: obiekty, tekstury, punkty AC, eksport FBX.

Obiekty w AC (pierwsza cyfra + nazwa nawierzchni = fizyka): `1ROAD`, `2ROAD`, `1KERB`, `1GRASS`, `2GRASS` (teren),
`1WALL` (opony), `2WALL` (mur). Punkty: `AC_START_0..9`, `AC_PIT_0..9`, `AC_TIME_0_L/R`, `AC_HOTLAP_START_0`.

## 8. Do zrobienia

- strojenie fizyki po pierwszym przejeździe (cel: 50–52 s);
- linia AI (`ai/fast_lane.ai`, nagrywana w grze);
- mapa toru, obrazki do menu (`ui/preview.png`, `ui/outline.png`), kamery do powtórek;
- tekstura asfaltu z białymi liniami, budynki i drzewa.

## Źródła danych i licencje

| Dane | Źródło |
|---|---|
| ortofotomapa, numeryczny model terenu | © GUGiK, [Geoportal.gov.pl](https://www.geoportal.gov.pl/) |
| oś początkowa toru | © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors, licencja ODbL |
| długość, szerokość, liczba zakrętów, rekord | [kartodrom-gostyn.pl](https://kartodrom-gostyn.pl/tor/) |
| kolory tarek i band | zdjęcia toru i nagrania z [kanału YouTube toru](https://www.youtube.com/@kartodromgostyn) |
| weryfikacja tarek, pola startowe | zdjęcie z drona (wrzesień 2025), plan toru PZM, wiedza lokalna autora |
