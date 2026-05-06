# WiFi Image Streamer - Multiple Crazyflies

노트북이 WiFi 핫스팟(AP)이 되고, 여러 Crazyflie가 STA 모드로 접속하여 각자의 영상을 노트북으로 스트리밍하는 예제입니다.

## 구조

```
[노트북 WiFi 핫스팟]
        ↓ STA mode
  Crazyflie #1 (TCP server :5000)
  Crazyflie #2 (TCP server :5000)
  ...

[multi-viewer.py]
  → TCP connect to each Crazyflie
  → Display all streams in a grid
```

## 1. 노트북 핫스팟 설정 (macOS)

1. **System Preferences → Sharing → Internet Sharing**
2. Share connection from: Wi-Fi (또는 Ethernet)
3. To computers using: **Wi-Fi**
4. **Wi-Fi Options**: SSID와 Password 설정
5. **Internet Sharing** 체크박스 활성화

> macOS 기본 게이트웨이: `192.168.3.1`, Crazyflie에는 `192.168.3.x` IP가 할당됩니다.

## 2. 펌웨어 빌드 & 플래시

```bash
cd examples/other/wifi-img-streamer-multiple

make HOTSPOT_SSID=MyWifi HOTSPOT_PASSWORD=mypassword
```

빌드한 펌웨어를 AI-Deck에 플래시합니다.

## 3. Crazyflie 전원 ON

Crazyflie 전원을 켜면 AI-Deck이 지정한 핫스팟에 자동으로 접속합니다.  
접속 후 TCP 서버(포트 5000)가 열리고 클라이언트(뷰어)를 기다립니다.

접속된 IP 확인:
```bash
arp -a
```

## 4. 뷰어 실행

### 4-a. IP 직접 지정

```bash
python multi-viewer.py 192.168.3.2 192.168.3.3
```

### 4-b. 서브넷 자동 탐색

```bash
python multi-viewer.py --discover --subnet 192.168.3.0/24
```

### 4-c. 자동 탐색 + 추가 IP 지정

```bash
python multi-viewer.py --discover --subnet 192.168.3.0/24 192.168.3.10
```

### 옵션

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `IP [IP ...]` | — | Crazyflie IP 주소 (여러 개 가능) |
| `-p, --port` | `5000` | TCP 포트 |
| `--discover` | — | 서브넷 자동 탐색 활성화 |
| `--subnet` | `192.168.3.0/24` | 탐색할 서브넷 |

**ESC** 키로 종료합니다.

## 주의사항

- `WIFI_CTRL_WIFI_CONNECT data[0] = 0x00`이 ESP32 펌웨어에서 STA 모드로 동작하는지 확인이 필요합니다. 동작하지 않는 경우 `aideck-esp-firmware`의 `WIFI_CTRL_WIFI_CONNECT` 핸들러를 확인하세요.
- 핫스팟 서브넷은 OS 및 설정에 따라 다를 수 있습니다 (`--subnet` 옵션으로 조정).
- 여러 Crazyflie를 동시에 사용할 경우 펌웨어는 동일하게 빌드하여 각 드론에 플래시합니다.
