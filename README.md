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
| Rekord toru (gokarty z wypożyczalni) | 50,434 s; tempo wyścigowe ok. 54 s – punkty odniesienia przy strojeniu fizyki |

---

## Szybki start

```
git clone https://github.com/Emenowicz/ac-gostyn.git
cd ac-gostyn
python3 build.py fetch-ortho     # jednorazowo: ortofotomapy (~80 MB, ok. 2,5 min) do tekstury terenu
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
build/fbx/texture/*.png                   tekstury (ortho.png / ortho_near.png = ortofotomapy 4096², po ok. 20 MB)
build/fbx/kartodrom_gostyn.blend          ten sam model do obejrzenia w Blenderze
build/content/tracks/kartodrom_gostyn/    folder moda: models.ini, data/surfaces.ini, ui/ui_track.json,
                                          ui/preview.png (widok z lotu ptaka, render Blendera), ui/outline.png (zarys)
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
3. Ustaw shader i tekstury każdego materiału (pliki z `texture/`). Materiały z mapą normalnych (`*_nm.png`,
   generowaną razem z teksturą) dostają `ksPerPixelNM` z `txDiffuse` = `<nazwa>.png` i `txNormal` = `<nazwa>_nm.png`:

   | Materiał | Shader | Tekstury | Gdzie |
   |---|---|---|---|
   | `road` | `ksMultilayer_fresnel_nm4` | `road_macro`, `mask_r`, detal `road`, `road_nm` – niżej | asfalt, płyty dodatkowego asfaltu |
   | `kerb` | `ksPerPixelNM` | `kerb`, `kerb_nm` | tarki (paski czerwono-żółte) |
   | `kerb_apron` | `ksPerPixelNM` | `kerb_apron`, `kerb_apron_nm` | wylewki z czerwonego betonu (`1APRON`) |
   | `kerb_concrete` | `ksPerPixelNM` | `kerb_concrete`, `kerb_concrete_nm` | szare betonowe wylewki (`2APRON`) |
   | `concrete` | `ksPerPixelNM` | `concrete`, `concrete_nm` | mur na brzegu terenu (`2WALL`) |
   | `tyre_red`, `tyre_yellow` | `ksPerPixelNM` | `tyre_red`, `tyre_red_nm` itd. | opony band (`TYRES_RED/YELLOW`, bez fizyki) |
   | `paint_white` | `ksPerPixelNM` | `paint_white`, `paint_white_nm` | linie krawędzi i korytarzy (`LINES_WHITE`, `LINES_LANES`) |
   | `paint_mint` | `ksPerPixelNM` | `paint_mint`, `paint_mint_nm` | pola startowe (`LINES_GRID`) |
   | `chequer` | `ksPerPixelNM` | `chequer`, `chequer_nm` | linia startu i mety (`LINES_START`) |
   | `ortho_near` | `ksMultilayer_fresnel_nm4` | `ortho_near`, `ortho_near_mask`, detale – niżej | pobocze i teren w okolicy toru (ortofotomapa 5 cm, `1GRASS`, `2GRASS`) |
   | `ortho` | `ksMultilayer_fresnel_nm4` | `ortho`, `ortho_mask`, detale – niżej | dalszy teren (`3GRASS`) |
   | `collider` | `ksPerPixelAT` | `collider` (w pełni przezroczysta) | kolizja band `1WALL`: niewidoczna, tylko fizyka |

   **Ortofotomapy z detalem.** Z bliska piksel 5 cm jest rozmyty, więc na zdjęcie nakłada się powtarzalny detal:
   trawa (`grass_detail`, kafel 1,5 m) albo goła ziemia – wytarte pobocza, piach, żwir (`soil_detail`, kafel 2 m).
   Który detal gdzie, mówi maska `<zdjęcie>_mask.png` liczona ze zdjęcia: R = trawa, G = bez trawy, B = A = 0.
   Kolor zostaje ze zdjęcia, detal (szarawy mnożnik o średniej 0,8) dodaje źdźbła, grudy i kamyki. Komplet ustawień
   z mnożnikami UV wyliczonymi z zasięgu zdjęcia zapisuje build do `build/fbx/ksEditor_multilayer.txt`:
   `txDiffuse` = zdjęcie, `txMask` = maska, `txDetailR` = `grass_detail`, `txDetailG` = `soil_detail`,
   `txNormalR/G` = ich `_nm`, `multR/multG` z pliku, `magicMult` = 1,25. Trawa i ziemia są matowe:
   `ksSpecular` = 0 i fresnel 0, inaczej pod niskim kątem pobocze robi się szarobiałe. **Nie sprawdzone jeszcze w AC**:
   gdyby pobocze wyszło ciemniejsze lub jaśniejsze niż zdjęcie, popraw `magicMult`; bez map normalnych wystarczy
   `ksMultilayer`. Podgląd w `.blend` pokazuje zdjęcie już z detalem (FBX niesie samo zdjęcie).

   **Asfalt** działa tak samo: bazą jest `road_macro` – jasność asfaltu ze zdjęcia 5 cm (zjeżdżona linia jazdy,
   jaśniejsze i ciemniejsze odcinki; bez białych linii, opon i trawy, przycięte do −10…+20 %, bo głębsze ciemne plamy
   to cień drzew, który AC rysuje sam), detalem kafel `road` 8 × 8 m (2048 px, bez wyróżniających się plam, żeby
   nie było widać powtórzeń). Maska `mask_r` = sam kanał R, `magicMult` = 2 (baza jest szara, 0,5 = mediana asfaltu
   toru). Ustawienia w tym samym pliku, sekcja `[road]`. Bez zdjęcia (`build.py` bez ortofotomapy) asfalt ma zwykłe
   UV kafla i wystarczy `ksPerPixelNM` z `road` / `road_nm`.

   Mapy normalnych są w konwencji OpenGL (zielony = +v). Gdyby faktura wyglądała jak wklęsła zamiast wypukłej,
   odwróć kanał zielony (albo ustaw mnożnik normalnych na ujemny).

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
| `data/features.json` | `start_finish` (linia startu), `grid` (środki namalowanych pól startowych), `painted_lines` (linie korytarzy), `barriers` (bandy z opon), `kerbs` (tarki), `extra_asphalt` (skróty, wjazdy) |
| `data/elevation.csv` | siatka wysokości terenu co 4 m |
| `data/track.json` | parametry; lista `placeholder` to wartości tymczasowe, `sources` to źródła danych |

Współrzędne są w metrach względem punktu `origin_lat_lon` z `track.json`. Tarka powstaje tam, gdzie krawędź asfaltu
leży bliżej niż `kerb_snap_m` od linii z `kerbs`; jej szerokość to `width` tej tarki (zmierzona przez `fit-kerbs`). Tarka zaczyna się
na krawędzi asfaltu, a krawędź asfaltu to malowana biała linia. `"style"` tarki: `stripes` (domyślnie, paski czerwono-żółte),
`apron` (wylewka z czerwonego betonu z jasnym brzegiem), `concrete` (szara betonowa wylewka). Obie wylewki są
podniesione o 3 cm jak tarka i mają własną nawierzchnię `APRON` ze słabszą wibracją niż `KERB`.
Tarka zwęża się do szpica na końcach na długości `"taper_m": [początek, koniec]` (w metrach, w kierunku jazdy;
domyślnie `kerb_taper_m` z `track.json`), bez zwężenia tam, gdzie styka się z inną tarką.
Banda z `"on_kerb": true` może stać na tarce, a `"height"` nadpisuje jej wysokość (np. 0,25 m dla pojedynczej opony leżącej przy wierzchołku).
Płyta z `extra_asphalt` może być dowolnym wielokątem bez samoprzecięć. Pola startowe `AC_START_*` stoją na punktach z `grid`
(kolejno od linii mety); gdy `grid_slots` w `track.json` jest większe, kolejne pola przedłużają obie kolumny o zmierzony
rozstaw rzędów. Bandy rysowane są jako słupki opon (`TYRES_*`, co 0,6 m, opony po ok. 0,2 m), a zderzenia liczy
niewidoczna bryła `1WALL`.

Ponowne pobranie danych źródłowych (zwykle niepotrzebne, wynik jest w repo):
```
python3 build.py import-osm --force       # oś początkowa z OpenStreetMap
python3 build.py fetch-ortho              # ortofotomapy GUGiK -> build/cache/ (poza repo): ortho = cały teren
                                          # (12 px/m), ortho_near = okolica toru (~19 px/m, nalot 2026 ma 5 cm);
                                          # w EPSG:2180, kaflami po 2000 px (w EPSG:4326 serwer dawał tylko ~62 %
                                          # rozdzielczości północ-południe); obrót siatki 2180 względem lokalnej
                                          # północy (~1,6°) przechodzi do UV terenu (`uv` w ortho*.json)
python3 build.py fit --force              # oś + szerokość dopasowane do asfaltu na ortofotomapie
                                          # (krawędź nie wychodzi za odrysowane bandy ani na płyty extra_asphalt)
python3 build.py fetch-elevation          # wysokości NMT GUGiK
python3 build.py fit-kerbs                # szerokość każdej tarki zmierzona na ortofotomapie -> features.json
```
`fit` i `import-osm` nadpisują `data/centerline.csv` – najpierw zacommituj zmiany.

**Uwaga:** GUGiK wymienił zdjęcie w usłudze WMS. Klasyfikator kolorów w `fit.py` był dostrojony do ortofotomapy pobranej
17.10.2026; na wersji z 3.11.2026 zalicza ciemne pobocza do asfaltu i `fit` daje tor szerszy o ok. 0,9 m. Przed ponownym
`fit` lub `fit-kerbs` trzeba przestroić `classify()`. Obecne krawędzie w `centerline.csv` zgadzają się z białymi liniami na
nowym zdjęciu (mediana odchyłki 2–4 cm); w kilku miejscach przesunięto je ręcznie na linię (patrz historia gita).
`fit-kerbs` nadpisuje też szerokości tarek ustawione ręcznie.

## 7. Jak to działa

```
data/*.csv, *.json ──> trackgen.py ──> build/mesh.json ──> blender_build.py ──> FBX ──> ksEditor ──> KN5
   (dane)              (siatki, UV,                       (Blender w tle:        (Windows)
                        walidacja)                          materiały, eksport)
```
- `trackgen.py` – spline przez oś, siatki asfaltu, tarek, pobocza, band i terenu, punkty AC, walidacja (czysty Python).
- `fit.py` – odczyt ortofotomapy i wykrywanie krawędzi asfaltu.
- `build.py` – wiersz poleceń: pobieranie danych, podglądy, pliki moda, uruchomienie Blendera.
- `blender_build.py` – uruchamiany w Blenderze: obiekty, tekstury, punkty AC, eksport FBX, render `ui/preview.png`.
- `textures.py` – proceduralne, bezszwowe tekstury i mapy normalnych (asfalt, beton, tarki, opony, malowanie);
  numpy z Blendera, więc dalej bez `pip install`.

Obiekty w AC (pierwsza cyfra + nazwa nawierzchni = fizyka): `1ROAD`, `2ROAD` (płyty),
`1APRON` i `2APRON` (czerwone i szare wylewki), `1KERB` (tarki w paski), `1GRASS`, `2GRASS`, `3GRASS` (teren), `1WALL` (kolizja band, niewidoczna), `2WALL` (mur). Bez fizyki: `TYRES_RED`, `TYRES_YELLOW` (opony),
`LINES_WHITE`, `LINES_LANES`, `LINES_GRID`, `LINES_START` (malowanie). Punkty: `AC_START_0..9`, `AC_PIT_0..9`, `AC_TIME_0_L/R`, `AC_HOTLAP_START_0`.

## 8. Do zrobienia

- strojenie fizyki po pierwszym przejeździe (cel: 50–52 s);
- linia AI (`ai/fast_lane.ai`, nagrywana w grze);
- mapa toru do gry (`map.png` + `data/map.ini`), kamery do powtórek;
- budynki i drzewa wokół toru (np. z modelu pokrycia terenu GUGiK), pachołki przy wierzchołkach, ogrodzenie;
- ostrzejsza trawa z bliska (tekstura detalu na ortofotomapie, np. przez Custom Shaders Patch).

## Źródła danych i licencje

| Dane | Źródło |
|---|---|
| ortofotomapa, numeryczny model terenu | © GUGiK, [Geoportal.gov.pl](https://www.geoportal.gov.pl/) |
| oś początkowa toru | © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors, licencja ODbL |
| długość, szerokość, liczba zakrętów, rekord | [kartodrom-gostyn.pl](https://kartodrom-gostyn.pl/tor/) |
| kolory tarek i band | zdjęcia toru i nagrania z [kanału YouTube toru](https://www.youtube.com/@kartodromgostyn) |
| weryfikacja tarek, pola startowe | zdjęcie z drona (wrzesień 2025), plan toru PZM, wiedza lokalna autora |
