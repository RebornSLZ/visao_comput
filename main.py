"""
Loop principal do robô 2 (tração por ponte H + direção por servo, ver
motores.py): captura a câmera, usa lanedetecmod.getLaneCurve() como
tratamento de visão (detecção de pista + warp preenchido) e converte o valor
de curva retornado (-1 a 1) em comandos de motores.py.

direita()/esquerda() aqui movem o servo direto pro ângulo fixo de curva (não
acumulam pulso como no robô 1) — o efeito é "bang-bang" (centro/direita
travada/esquerda travada), não proporcional à magnitude da curva.

Controles:
  Q          — sair
  trackbars  — ajustar os 4 pontos do warp em tempo real
"""

import cv2

import lanedetecmod
import motores
import utlis

LARGURA, ALTURA = 420, 240
TRACK_VALS_INICIAIS = [101, 92, 50, 193]

CURVA_LIMIAR = 0.05   # |curva| abaixo disso é tratado como "reto"

# Índice 0 é a câmera USB ("HD Web Camera") — confirmado via
# `v4l2-ctl --list-devices`; os demais /dev/video* são nós internos de
# ISP/codec do Broadcom (bcm2835-isp, bcm2835-codec), não câmeras.
CAMERA_INDEX = 0


def main() -> None:
    utlis.initializeTrackbars(TRACK_VALS_INICIAIS)

    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Não foi possível abrir a câmera USB.")

    motores.iniciar(40)  # velocidade reduzida: chassi/fiação do robô 2 ainda não confirmados fisicamente
    print(" Q  para sair")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Falha ao capturar frame.")
                break

            frame = cv2.resize(frame, (LARGURA, ALTURA))
            curva = lanedetecmod.getLaneCurve(frame, display=2)

            if curva > CURVA_LIMIAR:
                motores.direita()
            elif curva < -CURVA_LIMIAR:
                motores.esquerda()
            else:
                motores.frente()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        motores.encerrar()


if __name__ == "__main__":
    main()
