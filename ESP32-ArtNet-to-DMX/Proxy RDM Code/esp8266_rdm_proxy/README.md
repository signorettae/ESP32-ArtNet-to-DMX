# ESP8266 RDM Proxy Firmware

Transforma um ESP8266 em um **proxy RDM completo** com remapeamento de canais DMX512, 
interface web em tempo real, suporte a Art-Net e gerenciamento de Wi-Fi via WiFiManager.

---

## 📦 Arquivos do Projeto

```
esp8266_rdm_proxy/
├── esp8266_rdm_proxy.ino   ← Arquivo principal (setup/loop)
├── config.h                ← Constantes, pinos, structs compartilhados
├── storage.h / .cpp        ← Persistência via EEPROM
├── dmx_handler.h / .cpp    ← DMX512 físico (TX/RX via MAX485)
├── rdm_handler.h / .cpp    ← Proxy RDM completo (simulação de dispositivo)
├── artnet_handler.h / .cpp ← Receptor Art-Net via UDP
└── web_server.h / .cpp     ← Servidor web assíncrono + interface HTML
```

---

## 🔌 Conexões de Hardware

### ESP8266 (NodeMCU / Wemos D1 Mini) + MAX485

```
┌──────────────┬───────────────┬──────────────────────────┐
│  Pino MAX485 │  Pino ESP8266 │  Função                  │
├──────────────┼───────────────┼──────────────────────────┤
│  DI (TX In)  │  GPIO1 (TX)   │  Dados DMX Saída         │
│  RO (RX Out) │  GPIO3 (RX)   │  Dados DMX Entrada       │
│  DE + RE     │  GPIO5 (D1)   │  Controle de Direção     │
│  VCC         │  3.3V ou 5V   │  Alimentação             │
│  GND         │  GND          │  Terra                   │
└──────────────┴───────────────┴──────────────────────────┘

┌──────────────────┬───────────────┬──────────────────────┐
│  Componente      │  Pino ESP8266 │  Função              │
├──────────────────┼───────────────┼──────────────────────┤
│  Botão Reset     │  GPIO0 (D3)   │  Resetar WiFi (3s)   │
│  LED Wi-Fi       │  GPIO2 (D4)   │  Status Wi-Fi        │
│  LED DMX         │  GPIO4 (D2)   │  Atividade DMX       │
│  LED Erro        │  GPIO14 (D5)  │  Erros / Identify    │
└──────────────────┴───────────────┴──────────────────────┘
```

> ⚠️ **Atenção:** O pino GPIO1 (TX) é compartilhado com o Serial do Arduino IDE.
> Durante o upload e debug via Serial, o DMX será interrompido. Use UART alternativo
> ou desative o Serial em produção (`Serial.begin()` pode ser removido após testes).

---

## 📚 Bibliotecas Necessárias

Instale via Arduino IDE → Gerenciador de Bibliotecas:

| Biblioteca | Autor | Versão mínima |
|---|---|---|
| `WiFiManager` | tzapu | 2.0+ |
| `ESPAsyncWebServer` | me-no-dev | 1.2+ |
| `ESPAsyncTCP` | me-no-dev | 1.2+ |
| `ArduinoJson` | bblanchon | 6.x |
| `ArduinoOTA` | Arduino | (inclusa no ESP8266 core) |

> **ESPAsyncWebServer** e **ESPAsyncTCP** geralmente precisam ser instaladas
> manualmente via `.zip` ou GitHub — não estão no repositório oficial.

---

## ⚙️ Como Funciona

### Proxy RDM — Lógica de Simulação

O ESP8266 se apresenta na rede RDM com um **UID único** gerado a partir do Chip ID:
- Manufacturer ID: `0x4553` ("ES" para ESP)
- Device ID: 4 bytes do ESP.getChipId()

Quando um console de iluminação faz **discovery RDM**, o proxy responde como se
fosse o equipamento final. Para luminárias DMX sem RDM, o proxy "faz de conta"
que é o equipamento, respondendo a todos os comandos RDM internamente.

**PIDs suportados:**
- `DISC_UNIQUE_BRANCH` — Responde ao discovery com UID codificado
- `DISC_MUTE / UN_MUTE` — Gerencia estado de mute para discovery binárioa
- `DEVICE_INFO` — Informa modelo, footprint DMX, endereço
- `DMX_START_ADDRESS` — GET/SET do endereço inicial (também atualiza o offset!)
- `IDENTIFY_DEVICE` — Liga/desliga LED de erro para identificação física
- `DEVICE_LABEL` — Nome do dispositivo (editável via RDM e web)
- `MANUFACTURER_LABEL` — Fabricante
- `SOFTWARE_VERSION` — Versão do firmware
- `SUPPORTED_PARAMS` — Lista de PIDs suportados

### Remapeamento de Canais (Offset)

```
Offset = 130 → input canal 131 = output canal 1
                input canal 132 = output canal 2
                ...
                input canal 256 = output canal 126
```

Isso permite que um equipamento físico configurado no canal 1 seja
controlado por um console que envia dados a partir do canal 131.

Quando o console envia `SET DMX_START_ADDRESS = 131` via RDM, o proxy
automaticamente ajusta o offset para 130.

### Modos de Operação

| Modo | Descrição |
|---|---|
| **DMX Físico** | Usa apenas a entrada RS485 física |
| **Art-Net** | Usa apenas os pacotes UDP Art-Net |
| **Merge HTP** | Combina os dois — o maior valor vence por canal |
| **Merge DMX Prio** | DMX físico tem prioridade; Art-Net é fallback |
| **Merge Art-Net Prio** | Art-Net tem prioridade; DMX é fallback |

Fallback automático: se uma fonte não recebe dados por >3 segundos, a outra assume.

---

## 🌐 Interface Web

Acesse `http://<IP-do-ESP>` após conectar à rede Wi-Fi.

### Dashboard (`/`)
- Gráfico de barras com valores dos 512 canais (entrada e saída)
- Status de DMX e Art-Net (LED verde pulsando = ativo)
- Log de comandos RDM recebidos/respondidos
- Cards de informação rápida (UID, offset, endereço DMX)

### Configuração (`/config`)
- Ajuste do offset de canais
- Seleção de modo de entrada e merge
- Universo Art-Net
- Identidade RDM (label, fabricante, modelo)
- Reset de Wi-Fi e reset de fábrica

### API JSON
- `GET /api/dmx` — Buffers DMX (512 input + 512 output)
- `GET /api/status` — Status completo do sistema
- `GET /api/rdmlog` — Log de atividade RDM
- `POST /config` — Salvar configuração (JSON body)
- `POST /api/reset` — Resetar credenciais Wi-Fi
- `POST /api/factory` — Reset de fábrica

---

## 🚀 Primeiros Passos

1. **Instale as bibliotecas** listadas acima
2. **Configure o board:** Tools → Board → `NodeMCU 1.0` ou `Wemos D1 Mini`
3. **Configure a velocidade de upload:** 115200 ou 921600
4. **Faça o upload** do sketch
5. **Na primeira inicialização**, conecte-se ao AP `RDM-Proxy-AP` (senha: `rdmproxy`)
6. O portal cativo abrirá automaticamente — selecione sua rede Wi-Fi
7. Após conectar, acesse a interface web pelo IP exibido no Serial Monitor

### Reset de Wi-Fi
- **Botão físico:** Segure o botão em GPIO0 por 3 segundos
- **Via web:** Acesse `/config` → "Reset WiFi Settings"

### OTA (Atualização sem fio)
Após a primeira configuração, atualizações podem ser feitas via Arduino IDE:
- Tools → Port → selecione `rdm-proxy.local`
- Senha OTA: `rdmproxy2024`

---

## 📝 EEPROM — Layout de Memória

```
Byte 0    : Magic (0xA5) — indica dados válidos
Bytes 1+  : struct ProxyConfig (~80 bytes)
```

---

## ⚠️ Limitações Conhecidas

- O ESP8266 tem **apenas uma UART hardware**. Isso significa que debug via Serial
  e DMX físico compartilham o mesmo pino TX/RX. Para produção, remova os
  `Serial.print()` do código ou use SoftwareSerial para debug.
- O protocolo RDM completo (E1.20) é extenso. Esta implementação cobre os PIDs
  mais comuns usados na prática. PIDs não suportados recebem um NACK adequado.
- Art-Net suporta apenas o opcode `ArtDmx` (0x5000). `ArtPoll` não é implementado
  (o dispositivo não aparecerá em descoberta Art-Net, mas receberá os dados).

---

## 📄 Licença

MIT License — use e modifique livremente.
