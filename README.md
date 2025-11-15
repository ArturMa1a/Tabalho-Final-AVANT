# Trabalho Final - Trainee AVANT UFMG (Eletrônica)

Este repositório contém o código-fonte e os arquivos de configuração do projeto final desenvolvido para o programa de trainee da [AVANT UFMG]

## Sobre o Projeto

[cite_start]O objetivo deste projeto foi desenvolver o sistema de software para um drone capaz de competir na missão "Hang the Hook"[cite: 29].

## Pacote Principal: `tpFinal`

A pasta `/tpFinal` é o principal pacote ROS (Robot Operating System) do projeto. Ele contém:

* **`nodes/`**: Os nós que modelamos, responsáveis por tarefas como:
    * [cite_start]Processamento de visão computacional (detecção da linha)[cite: 248].
    * Lógica de navegação e controle de voo.
    * [cite_start]Comunicação com o controlador de voo (FMU)[cite: 83].
    * [cite_start]Controle do sistema de liberação do gancho (eletroímã)[cite: 216].
* **`launch/`**: Arquivos de inicialização para executar os nós de forma coordenada.
* **`CMakeLists.txt` e `package.xml`**: Arquivos responsáveis pela configuração e compilação do pacote ROS.

## Ambientes e programas utilizados

* **ROS (Robot Operating System):** Framework principal para comunicação entre os nós.
* **Gazebo/SITL:** Ambiente de simulação.
