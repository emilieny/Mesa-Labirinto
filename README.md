Link do youtube do projeto em execução: https://youtube.com/shorts/zbnCw2gqATI


#  Mesa Labirinto Controlada por Joystick

Projeto desenvolvido para a disciplina **Sistemas Embarcados** do curso de **Engenharia de Computação – IFPB (Campus Campina Grande)**, semestre **2025.1**, sob orientação do **Professor Alexandre Sales**.

##  Integrantes do Grupo

- Eduardo Henrique Lima de Morais  
- Emilieny de Souza Silva  
- Victor José Cordeiro de Medeiros  


## Visão Geral

O projeto consiste no desenvolvimento de uma **mesa de labirinto com inclinação controlada por joystick**, utilizando um sistema embarcado baseado no **ESP32**.  
A inclinação da mesa nos eixos **X e Y** é realizada por dois **servomotores**, enquanto a orientação (**Pitch e Roll**) é monitorada por um **sensor inercial MPU6050**.

Os dados de orientação são enviados em tempo real para um computador, permitindo a criação de um **gêmeo digital**, visualizado por meio de dashboards no **Grafana**, sincronizados com o movimento físico da mesa.

##  Tecnologias Utilizadas

- ESP32-WROOM-32  
- FreeRTOS  
- C / ESP-IDF  
- Servomotores (PWM – LEDC)  
- Joystick Analógico (ADC)  
- MPU6050 (I²C)  
- Python  
- InfluxDB  
- Grafana  

