"""
Controle de motores — robô 2: tração DC via ponte H (estilo BTS7960/IBT-2) + direção por servo

Vive em `robo2/` (em vez de sufixo no nome do arquivo) porque é um chassi,
uma placa e um Raspberry Pi diferentes do robô 1 (módulos `motores.py` /
`white_filter.py` na raiz do repo) — sem relação de código entre os dois além
de seguir a mesma convenção de API.

O chassi tem:
    • 1 (ou mais, em paralelo) motor(es) DC de tração → controlado(s) por uma
      ponte H com habilitação + PWM separado por sentido (R_EN/L_EN habilitam
      a ponte; RPWM acelera pra frente, LPWM acelera pra ré — só um dos dois
      tem duty > 0 de cada vez). Pinagem R_EN/L_EN/RPWM/LPWM é a nomenclatura
      padrão de módulos tipo BTS7960/IBT-2 de dupla ponte H com dois canais
      de PWM em vez de PWM único + pino de sentido (como o L298N do robô 1).
    • 1 servo de direção → controla o ângulo do eixo diretamente. Diferente
      do motor DC de direção do robô 1 (que não tem sensor de posição e
      precisa de dead-reckoning pra tentar centralizar), um servo de hobby
      guarda a própria posição internamente — direita()/esquerda() só mandam
      pra um ângulo fixo, não acumulam pulso.

Pinagem (BCM):

    Tração — ponte H  →  GPIO (BCM)
    ─────────────────────────────────
    R_EN      →  22   (habilita o lado direito da ponte — digital, fica ligado)
    L_EN      →  27   (habilita o lado esquerdo da ponte — digital, fica ligado)
    RPWM_PIN  →   6   (pino físico 31 — PWM, velocidade pra frente; movido de
                        GPIO12/pino físico 32 durante debug de fiação, ver nota abaixo)
    LPWM_PIN  →  13   (PWM — velocidade pra ré)

    Direção — servo  →  GPIO (BCM)
    ───────────────────────────────
    SERVO_PIN   →  12   (pino físico 32 — PWM a 50Hz, duty cycle 5.0%/7.5%/10.0% =
                          esquerda/centro/direita; movido de GPIO18/pino físico 12
                          durante debug de fiação, ver nota abaixo)

    De onde vem essa pinagem: veio de dois rascunhos de teste que já
    existiam soltos no Raspberry Pi (não neste repo) — `app_local.py` (mais
    recente, 30/ago 23:23) e `testeMotores.py` (mais antigo, 30/ago 20:31).
    Os dois divergiam: testeMotores.py testava o servo em GPIO12, mas
    app_local.py — escrito depois — já usa GPIO18 pro servo e GPIO12/13 pra
    tração. Este módulo seguiu app_local.py por ser a versão mais recente.

    RPWM_PIN movido de GPIO12 para GPIO6, e SERVO_PIN movido de GPIO18 para
    GPIO12: depois de testes sem nenhum movimento físico, suspeita de troca
    entre numeração BCM e numeração física do header (BCM12 = pino físico
    32, mas pino físico 12 = BCM18 — fácil de confundir contando os furos do
    conector). Ambos trocados como parte do debug de fiação — ajuste esses
    valores conforme o que for confirmado fisicamente.

    ATENÇÃO — movimento físico ainda não confirmado: uma versão anterior
    deste módulo (e o próprio app_local.py) rodaram sem erro de Python em
    testes via SSH, mas ninguém confirmou ter visto o robô se mexer de
    verdade. Antes de assumir que a lógica abaixo está certa, vale validar
    pino a pino com multímetro/LED (ou rodar teste_motores.py e observar de
    perto cada comando) em vez de confiar só em "rodou sem exceção".

Por que RPi.GPIO em vez de pigpio:
    O robô 1 (motores.py da raiz) usa pigpio por causa do PWM de hardware,
    mas o pacote do daemon (`pigpiod`) não está mais disponível nos
    repositórios do Raspberry Pi OS a partir do Bookworm/Trixie (projeto
    pigpio sem manutenção) — confirmado ausente neste Pi (Raspberry Pi 4,
    Debian 13 "trixie"). RPi.GPIO já está instalado nele. Isso significa PWM
    por SOFTWARE (thread) em vez de hardware — pode sofrer jitter se este
    processo rodar junto com OpenCV, diferente do robô 1.

    Padrão "mover e cortar o sinal" no servo: depois de mandar o servo pra
    uma posição, espera um tempinho pra ele chegar lá e corta o duty cycle
    (ChangeDutyCycle(0)) — evita o "tremor" de manter pulso contínuo depois
    que o servo já chegou. Servos comuns seguram a posição mecanicamente por
    um tempo mesmo sem pulso.

    ATENÇÃO — fase e neutro (rede elétrica AC):
    Este robô é alimentado a partir da rede elétrica AC (fase e neutro), não
    de bateria. Este módulo só gera sinais lógicos DE BAIXA TENSÃO (3.3V) nos
    pinos GPIO do Raspberry Pi — em nenhum momento ele chaveia ou entra em
    contato com a tensão AC da rede. A suposição é que a fase/neutro vai só
    até uma fonte/transformador externo (AC→DC) que alimenta o Raspberry Pi,
    a ponte H e o servo em baixa tensão — o código não liga/desliga nada na
    rede AC diretamente. Se em vez disso o GPIO precisar acionar um relé que
    chaveia a própria rede AC, isso precisa de um relé apropriado pra tensão
    de rede e isolamento adequado — o código abaixo NÃO cobre esse caso.

API pública:
    iniciar(velocidade_base)  — configura os pinos, centraliza o servo e inicia o PWM de tração
    frente()                  — centraliza a direção e segue reto
    re()                      — centraliza a direção e dá ré
    direita()                 — vira o servo para o ângulo de direita, mantendo o sentido vigente (frente/ré)
    esquerda()                — vira o servo para o ângulo de esquerda, mantendo o sentido vigente (frente/ré)
    parar()                   — para a tração (mantém o servo na última posição)
    encerrar()                — para o PWM, solta o servo e libera os pinos (GPIO.cleanup())

direita()/esquerda() só mexem no ângulo do servo — não decidem se o carro vai
pra frente ou de ré. Quem decide isso é a última chamada a frente()/re(); virar
durante uma ré continua em ré, virar durante uma ida continua indo. Se nenhum
frente()/re() foi chamado ainda, virar não movimenta a tração.
"""

from __future__ import annotations

import time

import RPi.GPIO as GPIO

# ── Pinos (BCM) ────────────────────────────────────────────────────────────────
# Tração — ponte H
R_EN = 22
L_EN = 27
RPWM_PIN = 6     # frente (pino físico 31)
LPWM_PIN = 13    # ré

# Direção — servo
SERVO_PIN = 12   # pino físico 32

# ── Parâmetros de tração ──────────────────────────────────────────────────────
FREQ_MOTOR_HZ = 1000
VEL_BASE      = 100    # duty cycle da tração em reta (0–100) — app_local.py usou
                        # 40 pra ser mais suave; ajuste via iniciar(velocidade_base=...)

# ── Parâmetros do servo de direção ────────────────────────────────────────────
FREQ_SERVO_HZ = 50
# Duty cycle a 50Hz. Os limites de curso mecânico podem precisar de ajuste
# fino por unidade — mexa aqui, não no resto do módulo.
SERVO_CENTRO   = 7.5
SERVO_DIREITA  = 10.0
SERVO_ESQUERDA = 5.0
SERVO_TEMPO_MOVIMENTO_S = 0.3  # tempo pro servo chegar na posição antes de cortar o sinal

# ── Objetos de PWM (criados em iniciar()) ─────────────────────────────────────
_pwm_rpwm: GPIO.PWM | None = None
_pwm_lpwm: GPIO.PWM | None = None
_pwm_servo: GPIO.PWM | None = None

# Último sentido de tração comandado (frente/ré) — direita()/esquerda() não
# mudam isso, só o ângulo do servo; a tração continua no sentido vigente.
_modo_tracao: str | None = None   # "frente" | "re" | None


def iniciar(velocidade_base: int = VEL_BASE) -> None:
    """Configura os pinos, centraliza o servo e inicia o PWM de tração (zerado)."""
    global _pwm_rpwm, _pwm_lpwm, _pwm_servo, VEL_BASE, _modo_tracao

    VEL_BASE = velocidade_base
    _modo_tracao = None

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup([R_EN, L_EN, RPWM_PIN, LPWM_PIN, SERVO_PIN], GPIO.OUT)

    # A ponte fica habilitada o tempo todo — o sentido e a velocidade são
    # controlados só pelo duty de RPWM/LPWM.
    GPIO.output(R_EN, True)
    GPIO.output(L_EN, True)

    _pwm_rpwm = GPIO.PWM(RPWM_PIN, FREQ_MOTOR_HZ)
    _pwm_lpwm = GPIO.PWM(LPWM_PIN, FREQ_MOTOR_HZ)
    _pwm_rpwm.start(0)
    _pwm_lpwm.start(0)

    _pwm_servo = GPIO.PWM(SERVO_PIN, FREQ_SERVO_HZ)
    _pwm_servo.start(SERVO_CENTRO)
    time.sleep(SERVO_TEMPO_MOVIMENTO_S)
    _pwm_servo.ChangeDutyCycle(0)


# ── Tração (ponte H → R_EN/L_EN/RPWM/LPWM) ────────────────────────────────────

def _tracao(duty: int) -> None:
    """Aciona a tração para frente na velocidade indicada."""
    _pwm_lpwm.ChangeDutyCycle(0)
    _pwm_rpwm.ChangeDutyCycle(duty)


def _tracao_re(duty: int) -> None:
    """Aciona a tração em marcha ré na velocidade indicada."""
    _pwm_rpwm.ChangeDutyCycle(0)
    _pwm_lpwm.ChangeDutyCycle(duty)


def _parar_tracao() -> None:
    _pwm_rpwm.ChangeDutyCycle(0)
    _pwm_lpwm.ChangeDutyCycle(0)


# ── Direção (servo → SERVO_PIN) ───────────────────────────────────────────────

def _definir_servo(duty: float) -> None:
    """Move o servo pra um duty cycle e corta o sinal depois (evita tremor)."""
    _pwm_servo.ChangeDutyCycle(duty)
    time.sleep(SERVO_TEMPO_MOVIMENTO_S)
    _pwm_servo.ChangeDutyCycle(0)


# Comandos principais

def _manter_tracao() -> None:
    """Continua a tração no último sentido comandado (frente/ré). Se ainda não
    houve frente()/re() nenhum, não mexe na tração (fica como estava)."""
    if _modo_tracao == "frente":
        _tracao(VEL_BASE)
    elif _modo_tracao == "re":
        _tracao_re(VEL_BASE)


def frente() -> None:
    """Centraliza o servo de direção e segue reto."""
    global _modo_tracao
    _modo_tracao = "frente"
    _definir_servo(SERVO_CENTRO)
    _tracao(VEL_BASE)


def re() -> None:
    """Centraliza o servo de direção e dá ré."""
    global _modo_tracao
    _modo_tracao = "re"
    _definir_servo(SERVO_CENTRO)
    _tracao_re(VEL_BASE)


def direita() -> None:
    """Vira o servo para o ângulo de direita e mantém a tração no sentido
    vigente (frente/ré) — não força ida."""
    _definir_servo(SERVO_DIREITA)
    _manter_tracao()


def esquerda() -> None:
    """Vira o servo para o ângulo de esquerda e mantém a tração no sentido
    vigente (frente/ré) — não força ida."""
    _definir_servo(SERVO_ESQUERDA)
    _manter_tracao()


def parar() -> None:
    """Para a tração (o servo mantém a última posição comandada)."""
    global _modo_tracao
    _modo_tracao = None
    _parar_tracao()


def encerrar() -> None:
    """Para tudo, desabilita a ponte, solta o servo e libera os pinos GPIO."""
    parar()
    if _pwm_servo:
        _pwm_servo.ChangeDutyCycle(0)
        _pwm_servo.stop()
    if _pwm_rpwm:
        _pwm_rpwm.stop()
    if _pwm_lpwm:
        _pwm_lpwm.stop()
    GPIO.output(R_EN, False)
    GPIO.output(L_EN, False)
    GPIO.cleanup()
