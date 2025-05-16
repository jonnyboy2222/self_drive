# COVA


## User Requirements
| ID   | Description                                                          |
|------|----------------------------------------------------------------------|
| UR_1 | 사용자 정보와 사용자 고유의 카드키 정보를 Main PC 데이터 베이스에 추가/삭제할 수 있다 |
| UR_2 | 등록된 사용자의 정보는 조회할 수 있다                                      |
| UR_3 | 등록된 사용자는 고유의 카드로 등록된 차량을 제어할 수 있다                      |
| UR_4 | 등록된 사용자가 차량을 제어하기 위해서는 음주 유무를 판단 후에 제어권을 제공 받아야 한다. |
| UR_5 | 등록된 사용자가 음주 후 차량 제어 시도 시 차량 제어권을 부여하지 않고 위치 정보를 Main PC에 보내줘야 한다. |
| UR_6 | 등록된 사용자가 등록된 차량을 제어할 때 생기는 정보들을 데이터 베이스에 저장할 수 있다 |
| UR_7 | 사용자가 차량 제어 중 생기는 이벤트를 Main PC에서 인지하고 후속 처리 서비스를 제공할 수 있다 |

## System Requirements
| ID   | Function    | Description                                              |
|------|-------------|----------------------------------------------------------|
| SR_1 | 사용자 관리      | 입력된 정보(신규 사용자 정보) 등록<br><br>등록된 사용자인 경우 등록된 정보 조회<br><br>등록된 사용자의 운전습관 데이터 조회<br><br>등록된 사용자 정보 삭제 |
| SR_2 | 차량 도어락      | 등록된 사용자인 경우 차량 도어락 해제                              |
| SR_3 | 음주운전 예방     | 사용자의 음주상태 확인<br><br>음주상태 아닌 경우에만 차량 시동 가능<br><br>음주상태 확인 시 GPS 통해 위치정보 받아 사용자 위치로 대리운전 연결 |
| SR_4 | 충격 감지       | 차량에 가해지는 충격 감지된 정보 받기<br><br>시간 당 충격 감지 횟수 계산해서 정보 받기 |
| SR_5 | 온도 감지       | 차량 내부에 사용되는 부품의 온도 감지해 임계값 이상일 시 정보 전송 및 경고          |
| SR_6 | 자동 헤드라이트    | 빛이 일정량 이하로 감지될 시 헤드라이트 ON                        |
| SR_7 | 주차 및 운행 보조  | 장애물과의 거리를 감지해 청각적/시각적 경고로 충돌 예방                |
| SR_8 | 사고 발생 처리 기능 | 충격, 가연, 온도 감지 센서에서 특정치 이상의 값이 감지되었을 경우 위치 데이터를 전송 |
| SR_9 | 위험 경고       | 각 센서에서 보내는 경고 메시지를 경고음으로 송출<br><br>각 센서에서 보내는 경고 메시지를 화면에 출력 |

## System Architecture
[HW_architecture](link) <br>
[SW_ardhictecture](link)

## Scenario
### RFID Register and Browse
[RFID_Register and Browse](link)
## Vehicle Authentication
[Vehicle Authentication](link)
### Shock and Temperature Management
[Shock and Temperature Management](link)
### Vehicle Control
[Vehicle Control](link)
### Headlight Control
[Headlight Control](link)
### Reverse Control Management
[Reverse Control Management](link)

## GUI
### COVA Admin
#### Default
[Default](link)
#### RFID Register and Browse
[RFID Register and Browse](link)
#### Register Initialize
[RFID Register Initialize](link)
#### Duplicate Register
[Duplicate Register](link)
### COVA Jr Control
#### Default
[Default](link)
#### Door Open
[Door Open](link)
#### Engine Start
[Engine Start](link)
#### Reverse
[Reverse](link)

## Interface Specification

### Command List

| Command | Full Name        |
|---------|------------------|
| DB      | DataBase         |
| VF      | VeriFication     |
| JS      | JSON             |
| MB      | Move Back        |
| ST      | Stop             |
| UR      | Ultrasonic       |
| LS      | Light Status     |

### COVA Jr Sensor_Control <->  COVA Jr Control

#### Local UID Verification Interface

| Interface No.     | Sender               | Receiver             | Header | Command | UID      | Status   | End   | Length(Bytes) | Description                  |
|-------------------|----------------------|----------------------|--------|---------|----------|----------|-------|----------------|------------------------------|
| -                 | -                    | -                    | 1 Byte | 2 Bytes | 4 Bytes | 1 Byte   | 1 Byte | -              | -                            |
| Rea_Ctrl_VF_01    | COVA Jr SensorControl | COVA Jr Control      | 0xAA   | VF      | Tag UID  | -        | '\\n'  | up to 8        | 등록된 uid인지 확인 요청     |
| Res_Ctrl_VF_01    | COVA Jr Control      | COVA Jr SensorControl | 0xAA   | VF      | -        | Status1* | '\\n'  | up to 8        | 등록된 uid인지 응답          |

##### Status1 Description

| Status1 | Description                      |
|---------|----------------------------------|
| 0x01    | Boolean `true` : 등록 카드       |
| 0x00    | Boolean `false` : 미등록 카드    |


#### Sensor Logging Interface

| Interface No. | Sender               | Receiver           | Header | Command | UID      | Shock   | Temp    | Length(Bytes) | Description             |
|---------------|----------------------|--------------------|--------|---------|----------|---------|---------|----------------|--------------------------|
| -             | -                    | -                  | 1 Byte | 2 Bytes | 4 Bytes | 4 Bytes | 4 Bytes | -              | -                        |
| Ctrl_DB_01    | COVA Jr SensorControl | COVA Jr Control    | 0xAA   | DB      | Tag UID  | value   | value   | 15 Bytes       | UID 기준 센서값 DB 저장 요청 |


#### Ultrasonic Command Interface

| Interface No.   | Sender           | Receiver             | Command | End   | Length(Bytes) | Description                             |
|-----------------|------------------|-----------------------|---------|-------|----------------|-----------------------------------------|
| -               | -                | -                     | 2 Bytes | 1 Byte | -              | -                                       |
| Ctrl_Ultra_01   | COVA Jr Control  | COVA Jr SensorControl | MB      | 0x00  | 3 Bytes        | \"후진 시작\" 전송 → 초음파 센서 시작      |
| Ctrl_Ultra_02   | COVA Jr Control  | COVA Jr SensorControl | ST      | 0x00  | 3 Bytes        | \"멈춤\" 전송 → 초음파 센서 멈춤          |

#### Driving Safefy Sensor Interface

| Interface No.  | Sender               | Receiver            | Header | Command | Value     | Length(Bytes) | Description                        |
|----------------|----------------------|---------------------|--------|---------|-----------|----------------|------------------------------------|
| -              | -                    | -                   | 1 Byte | 2 Bytes | -         | -              | -                                  |
| Ctrl_Alc01     | COVA Jr SensorControl | COVA Jr Control     | 0xAA   | VF      | Status2*  | 4 Bytes        | 음주 여부 전송 (운전 가능 판단용)       |
| Ctrl_Ultra03   | COVA Jr SensorControl | COVA Jr Control     | 0xAA   | UR      | value     | 7 Bytes        | 후방 장애물 거리 전송                  |
| Ctrl_Ls01      | COVA Jr SensorControl | COVA Jr Control     | 0xAA   | LS      | Status2*  | 4 Bytes        | 조도 상태 전송 (야간 여부 판단용 등)   |

##### Status2 Description

| Status2 | Description                     |
|---------|----------------------------------|
| 0x01    | Boolean `true` : pass / on       |
| 0x00    | Boolean `false` : nonpass / off  |

### COVA Jr Motor Control  <->  COVA Jr Control
#### Motor Control Interface

| Interface No.   | Sender          | Receiver            | Value       | Data Type   | Length (Bytes) | Description  |
|-----------------|------------------|----------------------|-------------|-------------|----------------|--------------|
| Ctrl_Motor_01   | COVA Jr Control  | COVA Jr Motor Control | how_move*   | Byte String | 3 Bytes        | 차량 이동 제어 |

##### how_move Description

| how_move   | Description     |
|------------|------------------|
| MF + '\\n' | Move Front       |
| MB + '\\n' | Move Back        |
| TL + '\\n' | Turn Left        |
| TR + '\\n' | Turn Right       |
| MS + '\\n' | Move Stop        |


### COVA Jr Control <->COVA Server
#### Remote UID Verification Interface

| Interface No.         | Sender           | Receiver         | Header | Command | UID      | Status    | End   | Length(Bytes) | Description                     |
|------------------------|------------------|------------------|--------|---------|----------|-----------|-------|----------------|---------------------------------|
| -                      | -                | -                | 1 Byte | 2 Bytes | 4 Bytes | 1 Byte    | 1 Byte | -              | -                               |
| Req_Ctrl_Sr_VF_01      | COVA Jr Control  | COVA Server      | 0xAA   | VF      | Tag UID  | -         | '\\n' | up to 8        | 등록된 UID인지 확인 요청         |
| Res_Ctrl_Sr_VF_01      | COVA Server      | COVA Jr Control  | 0xAA   | VF      | -        | Status1*  | '\\n' | up to 8        | 등록된 UID인지 응답              |

##### Status1 Description

| Status1 | Description                      |
|---------|----------------------------------|
| 0x01    | Boolean `true` : 등록 카드       |
| 0x00    | Boolean `false` : 미등록 카드    |


### Tag Reader ->COVA Admin
#### Admin Registration Interface

| Interface No.     | Sender     | Receiver   | Value    | Data Type | Length (Bytes) | Description               |
|-------------------|------------|------------|----------|-----------|----------------|---------------------------|
| Req_Admin_VF_01   | Tag Reader | COVA Admin | Tag UID  | String    | 4 Bytes        | 사용자의 UID 등록 요청     |

### COVA Admin ↔︎ COVA Server
#### User Data Transfer Interface

| Interface No.         | Sender      | Receiver    | Header | Command | data_length      | Purpose      | Data         | Data Type    | Length(Bytes) | Description            |
|------------------------|-------------|-------------|--------|---------|------------------|--------------|--------------|--------------|----------------|------------------------|
| -                      | -           | -           | 1 Byte | 2 Bytes | 2 Bytes          | -            | *            | -            | -              | -                      |
| Req_Admin_Sr_VF_01     | COVA Admin  | COVA Server | 0xAA   | JS      | -                | verification | Tag UID      | JSON String  | up to 60      | 태그된 UID 전송         |
| Admin_Sr_DB_01         | COVA Admin  | COVA Server | 0xAA   | JS      | len(json_bytes)  | db           | value_data*  | Bytes        | *              | 사용자 정보 등록         |

##### value_data Description

| Column       | Data Type (Bytes) | Description     |
|--------------|-------------------|------------------|
| uid          | JSON string (4)   | 사용자 UID       |
| user_name    | JSON string (10)  | 사용자 이름      |
| birth_date   | JSON string (10)  | 생년월일         |
| height       | JSON number (~6)  | 키               |
| weight       | JSON number (~6)  | 몸무게           |
| phone_num    | JSON string (11)  | 전화번호         |
| license_num  | JSON string (15)  | 면허번호         |

#### UID Verification Response Interface

| Interface No.       | Sender       | Receiver     | Header | Command | Purpose | Message     | Data Type    | Length(Bytes) | Description         |
|---------------------|--------------|--------------|--------|---------|---------|--------------|---------------|----------------|---------------------|
| Res_Admin_Sr_VF_01  | COVA Server  | COVA Admin   | 0xAA   | -       | -       | PASS / FAIL  | JSON String   | *              | 등록 여부 전송        |

## ER Diagram
### Admin DB
[Admin DB](link)
### Vehicle DB
[Vehicle DB](link)

## Test Case Table

| TC ID  | 주체     | 상호작용 대상            | 행동 (입력)                           | 예상 결과 (대상 포함)                                                              | 실제 결과 |
|--------|----------|--------------------------|----------------------------------------|--------------------------------------------------------------------------------------|------------|
| TC01   | 관리자   | COVA Admin               | 대기 상태                              | [COVA Admin GUI] 유저 정보 등록 폼/확인/등록 버튼 비활성, 초기화 버튼만 활성       | pass       |
| TC02   | 관리자   | TAG Reader               | 미등록 RFID 태깅                       | [COVA Admin GUI] RFID UID 출력, 확인/등록 버튼 활성화                               | pass       |
| TC03   | 관리자   | COVA Admin               | 유저 정보 입력 후 확인 클릭            | [COVA Admin GUI] 표에 유저 정보 출력                                                | pass       |
| TC04   | 관리자   | COVA Admin               | 등록 버튼 클릭                         | [COVA Admin GUI] 등록 성공 팝업 출력                                                | pass       |
| TC05   | 관리자   | COVA Admin               | 등록 성공 팝업 OK 클릭                 | [COVA Admin GUI] 팝업 닫힘, 유저 정보 및 버튼 초기화                                 | pass       |
| TC06   | 관리자   | COVA Admin               | 등록된 RFID 태깅                       | [COVA Admin GUI] 등록 오류 팝업 출력, 기존 사용자 정보 표에 출력                    | fail       |
| TC07   | 관리자   | COVA Admin               | 등록 오류 팝업 OK 클릭                 | [COVA Admin GUI] 팝업 닫힘, 유저 정보 및 표 및 버튼 초기화                          | pass       |
| TC08   | 사용자   | COVA Jr Sensor Control   | 미등록 RFID 태깅                       | [COVA Jr Control GUI] Unregistered 출력                                             | fail       |
| TC09   | 사용자   | COVA Jr Sensor Control   | 등록된 RFID 태깅                       | [COVA Jr Control GUI] Welcome 출력                                                  | pass       |
| TC10   | 사용자   | COVA Jr Sensor Control   | 음주측정 버튼 클릭                     | [COVA Jr Control GUI] 5초 후 Engine Start 출력                                      | pass       |
| TC11   | 사용자   | COVA Jr Control          | W 키 입력                              | [COVA Jr Motor Control] 차량 전진                                                   | pass       |
| TC12   | 사용자   | COVA Jr Control          | W 키 해제                              | [COVA Jr Motor Control] 차량 정지                                                   | pass       |
| TC13   | 사용자   | COVA Jr Control          | A 키 입력                              | [COVA Jr Motor Control] 차량 좌회전                                                 | pass       |
| TC14   | 사용자   | COVA Jr Control          | A 키 해제                              | [COVA Jr Motor Control] 차량 정지                                                   | pass       |
| TC15   | 사용자   | COVA Jr Control          | D 키 입력                              | [COVA Jr Motor Control] 차량 우회전                                                 | pass       |
| TC16   | 사용자   | COVA Jr Control          | D 키 해제                              | [COVA Jr Motor Control] 차량 정지                                                   | pass       |
| TC17   | 사용자   | COVA Jr Control          | S 키 입력 (후면 1m 이내 장애물)        | [COVA Jr Motor Control] 차량 후진, [COVA Jr Sensor Control] 경고음 시작 (거리 비례) | pass       |
| TC18   | 사용자   | COVA Jr Control          | S 키 해제                              | [COVA Jr Motor Control] 차량 정지, [COVA Jr Sensor Control] 경고음 해제             | pass       |

