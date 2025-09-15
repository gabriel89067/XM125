

# XM125 - Biblioteca de Comunicação (ESP32 + I²C)

Este código implementa funções para comunicação com o sensor **Acconeer XM125** via **I²C** em um ESP32 utilizando o framework **ESP-IDF**.
O XM125 é um módulo de radar que permite detectar presença e medir distâncias com alta precisão.

A biblioteca inclui:

* Funções de leitura e escrita em registradores do sensor.
* Configuração e inicialização dos modos de **presença** e **distância**.
* Algoritmos de detecção e filtragem (média móvel e linearização).

---

## Estrutura do código

* **Leitura e escrita I²C**

  * `read_register()` e `read_register1()` → lêem valores de registradores do XM125.
  * `write_register()` e `write_register1()` → escrevem valores nos registradores do XM125.

* **Configuração do sensor**

  * `configuration_ok_p()` e `configuration_ok()` → validam se a configuração de presença/distância foi aplicada com sucesso.
  * `wait_not_busy_p()` e `wait_not_busy()` → aguardam o radar terminar operações internas.

* **Exemplos de uso**

  * `example_setup_and_start()` → configura e inicia o modo de **detecção de presença**.
  * `example_setup_and_measure()` → configura e realiza medições de **distância**.

* **Processamento de dados**

  * `detectar_obj()` → detecta se há movimento ou objeto presente, baseado em variações sucessivas.
  * `media_dist()` → aplica uma média móvel sobre as últimas leituras de distância.
  * `media_peak()` → aplica uma média móvel sobre os valores de pico de RSS.
  * `lineariza()` e `lineariza_peak()` → reduzem o ruído nas medições, aplicando uma suavização de 5%.

---

##  Constantes importantes

* `NMAX_MED` → tamanho do buffer para médias móveis.
* `limite`, `margem`, `margem_peak` → parâmetros usados na suavização e filtragem.
* `I2C_ADDR_P` e `I2C_ADDR_D` → endereços I²C do radar para modo **presença** e **distância**.
* Registradores como `PRESENCE_REG_*` e `DISTANCE_REG_*` controlam funções específicas do sensor.

---

## Fluxo típico de operação

1. **Inicialização** → configurar os parâmetros de início/fim (em mm) para presença ou distância.
2. **Aplicar configuração** → enviar comando `APPLY_CONFIGURATION`.
3. **Esperar fim da configuração** → via `wait_not_busy()`.
4. **Validar configuração** → usando `configuration_ok()`.
5. **Rodar medições**:

   * No modo presença → usar `example_setup_and_start()`.
   * No modo distância → usar `example_setup_and_measure()`.
6. **Filtrar valores** → aplicar `media_*()` e `lineariza_*()` para estabilizar leituras.

---

## Saída esperada

* **Modo Presença**:

  ```
  Presence detected at distance: XXXX mm
  ```

  ou

  ```
  No presence detected
  ```

* **Modo Distância**:

  ```
  Peak distance: XXXX mm
  ```

  ou

  ```
  No peak detected
  ```

---
