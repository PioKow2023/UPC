# Projekt monitorowania UPS z wykorzystaniem ESP32-C3

## Konfiguracja sprzętowa

### Mikrokontroler
- Płytka deweloperska: ESP32-C3-DevKitM-1
- Środowisko programistyczne: PlatformIO z frameworkiem Arduino

### Wyświetlacz
- Typ: OLED 128x32
- Interfejs: I2C
- Pin SDA: GPIO8
- Pin SCL: GPIO9

### Wejścia pomiarowe
- Pomiar napięcia baterii: GPIO0 (z dzielnikiem napięcia)
- Pomiar napięcia wyjściowego: GPIO1 (z dzielnikiem napięcia)
- Detekcja zaniku zasilania sieciowego: GPIO6 (stan niski oznacza brak zasilania)
- Sterowanie przekaźnikiem: GPIO7 (wysoki stan aktywuje przekaźnik)

### Wyjścia sterujące
- Dioda LED RGB:
  - Czerwony: GPIO2
  - Zielony: GPIO3
  - Niebieski: GPIO4

## Funkcjonalności systemu

### Pomiar napięć
- Ciągły pomiar napięcia baterii i napięcia wyjściowego
- Wykorzystanie filtru średniej ruchomej dla stabilizacji pomiarów
- Kalibracja wartości napięć za pomocą współczynników korekty

### Wizualizacja na wyświetlaczu OLED
- Prezentacja napięcia baterii i napięcia wyjściowego w formacie cyfr siedmiosegmentowych
- Etykiety "BAT" i "OUT" wyświetlane pionowo po lewej i prawej stronie
- Linie poziome oddzielające sekcje informacyjne

### Reakcje na różne stany systemu

#### Normalne działanie (napięcie baterii powyżej 10V)
- Wyświetlanie bieżących napięć
- Dioda LED prezentuje kolory zależne od napięcia:
  - 10V-14V: Płynne przejście kolorów od czerwonego do zielonego
  - Powyżej 14V: Zielony kolor

#### Niskie napięcie baterii (poniżej 10V)
- Wyświetlanie komunikatu ostrzegawczego "LOW VOLTAGE" i "WARNING!" przez 2 sekundy
- Powrót do ekranu pomiarów z migającymi wartościami napięć (1Hz)
- Dioda LED "oddycha" czerwonym kolorem (cykl 4-sekundowy)
- Aktywacja przekaźnika odłączającego obciążenie

#### Zanik zasilania sieciowego
- Wykrycie stanu niskiego na GPIO6
- Wyświetlenie komunikatu "POWER LOST" i "RUNNING ON BAT"
- Migające etykiety "BAT" i "OUT" (500ms cykl)
- Przywrócenie zasilania sieciowego: komunikat "POWER RESTORED"

#### Wysokie napięcie (powyżej 14V)
- Dioda LED losowo miga kolorami czerwonym, zielonym i niebieskim

## Specjalne funkcje

### Automatyczne zarządzanie obciążeniem
- Przekaźnik automatycznie odłącza obciążenie przy napięciu poniżej 10V
- Przekaźnik automatycznie przywraca zasilanie po wzroście napięcia powyżej 11V

### Wskaźnik stanu LED
- Dynamiczna zmiana kolorów w zależności od napięcia
- Efekt "oddychania" przy niskim napięciu
- Losowe miganie kolorów przy wysokim napięciu
- Brak efektów pulsowania w normalnych warunkach pracy

## Biblioteki wykorzystane w projekcie
- Adafruit SSD1306
- Adafruit GFX Library
- PubSubClient
- U8g2