"""
Controle manual do robô por teclado, em tempo real (sem precisar apertar Enter).

Teclas:
    w  — frente
    s  — ré
    a  — esquerda
    d  — direita
    espaço — parar
    q  — sair

w/s/espaço comandam a tração até a próxima tecla ser pressionada (o PWM fica
ligado no duty cycle atual até um novo comando chegar). Já a/d dão só um pulso
curto na direção a cada tecla (o eixo vira aos poucos, como um volante) — pra
virar mais, aperte de novo.

Requer um terminal interativo de verdade (rodar direto no Pi, ou via SSH numa
sessão onde você está digitando ao vivo — não funciona alimentando comandos
por pipe/redirecionamento, como o teste_motores.py aceita).
"""

from __future__ import annotations

import sys
import termios
import tty

import motores

TECLAS = {
    "w": ("frente", motores.frente),
    "s": ("ré", motores.re),
    "a": ("esquerda", motores.esquerda),
    "d": ("direita", motores.direita),
    " ": ("parar", motores.parar),
}


def ler_tecla() -> str:
    """Lê um único caractere do terminal sem esperar por Enter."""
    fd = sys.stdin.fileno()
    config_original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, config_original)


def main() -> None:
    motores.iniciar()
    print("Controle por teclado — w=frente  s=ré  a=esquerda  d=direita  espaço=parar  q=sair")

    try:
        while True:
            tecla = ler_tecla().lower()

            if tecla == "q":
                break

            acao = TECLAS.get(tecla)
            if acao is None:
                continue

            nome, funcao = acao
            print(nome)
            funcao()
    finally:
        motores.encerrar()
        print("Motores encerrados.")


if __name__ == "__main__":
    main()
