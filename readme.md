# COVA: 차량 인증 및 제어 시스템

**RFID 기반 사용자 인증과 센서 진단을 통해  
운전 가능 여부를 판단하고, 안전한 차량 제어를 실현하는 시스템입니다.**

---

## 📽️ 시연 영상  
[![COVA Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/0.jpg)](https://www.youtube.com/watch?v=YOUR_VIDEO_ID)

> 📌 *영상으로 시스템 전반 흐름을 빠르게 이해할 수 있습니다.*

---

## 🔑 핵심 기능 요약

- **RFID 인증**: 등록된 카드만 차량 제어 가능  
- **음주 감지**: 음주 시 차량 시동 차단  
- **차량 제어**: 전진 / 후진 / 좌우회전 / 정지  
- **센서 데이터 기록**: 충격, 온도 등 센서값 DB 저장  
- **LCD 출력**: 실시간 상태 표시  
- **UID 등록/조회/검증 시스템** 포함  

---

## 📑 목차

1. [Overview](#overview)  
2. [Key Features](#key-features)  
3. [Team Information](#team-information)  
4. [Development Environment](#development-environment)  
5. [System Design](#system-design)  
   - [User Requirements](#user-requirements)  
   - [System Requirements](#system-requirements)  
   - [System Architecture](#system-architecture)  
   - [Scenario](#scenario)  
   - [GUI](#gui)  
6. [Database Design](#database-design)  
   - [ER Diagram](#er-diagram)  
7. [Interface Specification](#interface-specification)  
8. [Test Cases](#test-cases)  
9. [Problems and Solutions](#problems-and-solutions)  
10. [Limitations](#limitations)  
11. [Conclusion and Future Work](#conclusion-and-future-work)  

---

## 1. Overview

COVA는 운전자 인증과 상태 확인을 바탕으로 차량의 **안전한 사용을 제어 및 기록**하는 시스템입니다.  
아두이노 기반 센서와 RFID 인증 장치로 구성되며, 사용자 행동 기록 및 안전 상태를 종합적으로 판단합니다.

---

## 2. Key Features

- RFID 기반 사용자 인증
- 음주 여부 확인 (MQ2)
- 온도/조도/충격 센서 데이터 수집
- 모터 기반 차량 이동 제어
- LCD 기반 실시간 시각 피드백
- DB 연동 센서 기록

---

## 3. Team Information

| 이름   | 역할 |
|--------|------|
| 홍길동 | 센서 통합 및 제어 로직 개발 |
| 김철수 | 인증 시스템 및 모터 제어 |
| 박영희 | UI/GUI 개발 및 DB 연동 |

---

## 4. Development Environment

- **하드웨어**: Arduino Uno, LCD, MQ2, 조도센서, 온도센서, 초음파센서  
- **소프트웨어**: C++ (Arduino), Python (서버/검증), MySQL  
- **툴**: PlatformIO, VSCode, Git, GitHub

---

## 5. System Design

### 5.1 User Requirements

| ID   | Description |
|------|-------------|
| UR_1 | 사용자 UID 등록 및 삭제 가능해야 함 |
| UR_2 | 차량 제어 전 사용자 인증 필수 |
| UR_3 | 음주 상태 판단 후 제어 허용/차단 |
| UR_4 | 주행 중 센서 정보 기록 가능 |

### 5.2 System Requirements

| ID   | 기능 | 설명 |
|------|------|------|
| SR_1 | RFID 인증 | UID 기반 사용자 인증 |
| SR_2 | 음주 측정 | MQ2 센서 기반 상태 판단 |
| SR_3 | 차량 제어 | W/A/S/D 입력 기반 이동 |
| SR_4 | 센서 기록 | Shock, Temp, Light 데이터 전송 및 저장 |

### 5.3 System Architecture

> 시스템 구성도 또는 이미지 삽입 예정  
> 예: RFID → 음주 판단 → 차량 제어 → DB 저장 흐름도

### 5.4 Scenario

- 등록된 UID 태깅 → 음주 측정  
- 통과 시 차량 제어 허용  
- 차량 이동 및 센서값 지속 수집  
- DB에 운전 기록 저장

### 5.5 GUI

- COVA Admin: 사용자 등록/조회/검증  
- COVA Jr Control: 차량 상태 표시 (LCD)

---

## 6. Database Design

### 6.1 ER Diagram

> ERD 이미지 삽입  
> 주요 테이블: `user`, `sensor_log`, `auth_log`, `vehicle_status`

---

## 7. Interface Specification

> 각 모듈 간 데이터 송수신 포맷 정의  
> 예:  
> - RFID → 제어부: UID 요청  
> - 제어부 → 센서부: 후진 시작 명령  
> - 센서부 → 제어부: 센서값 응답  
> - 제어부 → DB: 전체 기록 전송

(※ 상세 표는 [별도 링크 또는 부록]에서 참조)

---

## 8. Test Cases

| TC ID | 주체     | 동작               | 예상 결과            | 실제 결과 |
|--------|----------|--------------------|------------------------|------------|
| TC01   | 관리자   | RFID 등록           | 등록 성공               | PASS       |
| TC02   | 사용자   | 음주 상태 확인       | 상태에 따라 제어 허용/차단 | PASS       |
| TC03   | 사용자   | 차량 이동 (W 키)     | 전진 동작                | PASS       |

...

---

## 9. Problems and Solutions

- **문제**: 음주 센서 민감도 불안정  
  **해결**: 5초간 평균값 측정 및 기준값 설정  

- **문제**: StaleElement 예외  
  **해결**: WebDriverWait, 재탐색 방식으로 해결

---

## 10. Limitations

- 지역 내 시스템 사용에 국한됨 (서울 한정)  
- 모바일 제어 앱 미구현  
- 실내외 상황 자동 판별은 미완성

---

## 11. Conclusion and Future Work

- 인증 및 판단 기반 제어 흐름 검증 완료  
- 향후 웹 기반 관리 페이지, 실시간 주행 모니터링, 앱 연동 가능성 검토 중  
- 차량 사고 상황 자동 신고 기능도 기획 예정

---

