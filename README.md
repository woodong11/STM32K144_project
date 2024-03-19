# STM32K144_project
STM32K144를 이용한 기초 자동차 제어입니다.
STM보드와 외부 GPIO 입출력 장치를 이용해 구현했습니다.

스위치 1번: 브레이크

스위치 2번: 좌측 방향 지시등

스위치 3번: 우측 방향 지시등

스위치 4번: 비상등

스위치 5번: 후진기어 (모터 방향 변경)

## 다이어그램과 GPIO 설정 
![image](https://github.com/woodong11/STM32K144_project/assets/91379630/90459537-dd93-42b0-9330-12d0d1788dc2)

PTB 0, 1: DC motor In1, In2

PTC 11, 12, 13, 16, 17: switch

PTD 1, 2, 3, 4, 5, 6, 7: 7_segment FND_DATA

PTD 8, 9, 10, 11: 7_segment select control

PTD 15: red LED

PTE 1, 2, 3, 4, 5, 7, 8, 9: LED

