# Trabalho Final - Trainee AVANT UFMG (Eletrônica)

Este repositório contém o código-fonte e os arquivos de configuração do projeto final desenvolvido para o programa de trainee da [AVANT UFMG]

## Sobre o Projeto

O objetivo deste projeto foi desenvolver o sistema de software para um drone capaz de competir na missão "Hang the Hook"[cite: 29].

## Pacote Principal: `tpFinal`

A pasta `/tpFinal` é o principal pacote ROS (Robot Operating System) do projeto. Ele contém:

* **`cameraSubscriber', 'soltura_gancho'`**: Os nós que modelamos
* **`package.xml`, `setup.cfg`, `setup.py`**: Arquivos responsáveis pela configuração e compilação do pacote ROS.

## Ambientes e programas utilizados

* **ROS (Robot Operating System):** Framework principal para comunicação entre os nós.
* **Gazebo/SITL:** Ambiente de simulação.
