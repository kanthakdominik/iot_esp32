# iot_esp32 – firmware miernika promieniowania

LILYGO T-SIM7600G-H (ESP32-WROVER + modem LTE/GNSS SIM7600G-H) + licznik Geigera MightyOhm (SBM-20).
Protokół komunikacji: `iot_spring/docs/PROTOCOL.md` (MQTT v1).

## Co robi

- co sekundę (w rytmie linii z licznika MightyOhm) tworzy próbkę JSON: zliczenia, µSv/h,
  pozycja GNSS (gdy jest fix), czas UTC, numer trasy `r` i numer próbki `seq`;
- wysyła ją przez LTE do brokera MQTT (TLS + certyfikat urządzenia) na `radmon/v1/<deviceId>/telemetry`;
- **zawsze** dopisuje ją też na kartę SD (`/r<trasa>.jsonl`), jeśli karta jest włożona –
  dane nie giną bez zasięgu;
- po połączeniu publikuje status (`online`, wersja, trasa, przyczyna resetu, bateria);
  przy zerwaniu połączenia broker sam publikuje `{"online":false}` (Last Will);
- przyjmuje komendy z serwera: `new-route` (nowa trasa bez restartu), `battery`.

## Trasy

| Zdarzenie | Efekt |
|---|---|
| włączenie zasilania, przycisk **RST** | nowa trasa |
| komenda `new-route` (przycisk „Nowa trasa” w aplikacji) | nowa trasa, bez restartu modemu i bez utraty fixu GPS |
| spadek napięcia, crash, watchdog | **ta sama** trasa, numeracja próbek kontynuowana |

Przyczyna resetu jest wypisywana na Serial przy starcie (`Reset reason: ...`) i wysyłana w statusie.

## Dioda LED

| Stan | Znaczenie |
|---|---|
| miga co 0,5 s | brak fixu GNSS |
| świeci ciągle | fix GNSS |
| 3 szybkie mignięcia | rozpoczęto nową trasę |

## Kompilacja (Arduino IDE)

1. Płytka: **ESP32 Wrover Module** (pakiet *esp32 by Espressif*), Partition Scheme: *Default*, PSRAM: *Enabled*.
2. Biblioteki (Library Manager):
   - **TinyGSM** (Volodymyr Shymanskyy)
   - **SSLClient** (govorox)
   - **PubSubClient** (Nick O'Leary)
   - **ArduinoJson** (Benoit Blanchon) – wersja 7
3. `iot_esp32/secrets.example.h` → skopiuj jako `iot_esp32/secrets.h` i uzupełnij:
   - `deviceId` – **musi być równy CN certyfikatu** (np. `esp32-1`),
   - `apn` operatora karty SIM,
   - `broker` (`iot-manager.duckdns.org`) i port `8883`,
   - trzy bloki certyfikatów – gotowe do wklejenia w pliku `deploy/certs/<deviceId>/secrets_certs.h`
     wygenerowanym na serwerze przez `deploy/scripts/gen-certs.sh <deviceId>`.
4. Wgraj, otwórz Serial Monitor (115200).

`secrets.h` jest w `.gitignore` – nigdy go nie commituj (zawiera klucz prywatny urządzenia).

## Zmiany względem wersji 1.x (praca magisterska)

- ramka JSON zamiast CSV, nowe topiki `radmon/v1/...`, unikalny client id = `deviceId`;
- czas UTC z GNSS (zapasowo z sieci GSM z poprawką strefy czasowej);
- próbki bez fixu GPS są wysyłane (`"fix":false`) – serwer pokazuje je na wykresie, ale nie na mapie;
- trasa nie dzieli się po awaryjnym restarcie;
- tryb offline przez podwójny reset usunięty – zastępuje go stały zapis na SD
  (automatyczne dosyłanie zaległych danych: backlog w `ROADMAP.md`);
- GPS włączany raz przy starcie (wcześniej co sekundę), brak blokowania pętli przy utracie zasięgu;
- `SENSOR_TX` nieużywany – zwolniony pin IO0 (pod przyszły przycisk).

## Przejście z wersji 1.x (jednorazowo)

- Twoje dotychczasowe prawdziwe dane (APN, Wi-Fi) z `iot_esp32_online/secrets.h` zostały przeniesione do
  `iot_esp32/secrets.v1-backup.h` (ignorowany przez git) – przepisz z niego APN do nowego `secrets.h`,
  potem plik możesz usunąć. Certyfikaty i adres brokera z wersji 1.x (AWS) są już nieaktualne.
- `iot_esp32/secrets.h` był śledzony przez git – przed commitem: `git rm --cached iot_esp32/secrets.h`.
- Foldery `iot_esp32_online` i `iot_esp32_offline` (stare warianty) zostały usunięte – są w historii gita.
