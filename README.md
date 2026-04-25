# 3sphere-wheel-receive

3-Sphere Omniwheel Mobility — Receive 펌웨어

nRF24L01 RF 수신 및 VESC 3개 제어 STM32F103 펌웨어입니다.

## Branch 구조

| Branch | 설명 |
|--------|------|
| `master` | 폴링 기반 RF 수신 + TIM1 PWM으로 VESC 3개 PPM 제어 |
| `interrupt-conversion` | nRF24 IRQ 인터럽트 기반 수신 구조로 전환 |
| `kiwiDrive_Modify` | PPM → CAN Bus 전환, TIM3 Watchdog, E-Stop 래치 추가. 저속 진동 원인을 역기구학으로 오판했던 코드 수정 시도 흔적 (실제 원인: 모터 제조 공차 + PID 설정 한계) |

## 사용 기술
STM32F103, CAN Bus, SPI, nRF24L01, TIM1 PWM, UART, VESC

## 전체 프로젝트
→ [3sphere-omniwheel-mobility](https://github.com/xodrb/3sphere-omniwheel-mobility)
