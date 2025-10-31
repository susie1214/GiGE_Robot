# 🐕 Spot Micro Robot Controller - 프로젝트 개요

**작성일**: 2025-10-31
**플랫폼**: RDK X5
**서보 모터**: MG966R (12개)
**서보 컨트롤러**: PCA9685 (I2C)

---

## 📋 목차

1. [프로젝트 구조](#-프로젝트-구조)
2. [좌표계 시스템](#-좌표계-시스템)
3. [역기구학 (IK) 시스템](#-역기구학-ik-시스템)
4. [핵심 파일 설명](#-핵심-파일-설명)
5. [동작 기능](#-동작-기능)
6. [설정 파일](#️-설정-파일)
7. [사용 방법](#-사용-방법)
8. [좌표 기반 제어](#-좌표-기반-제어)
9. [개발 히스토리](#-개발-히스토리)

---

## 📁 프로젝트 구조

```
Dog_V3/
├── spot_micro_controller.py    # 메인 컨트롤러 (좌표 기반 IK 포함)
├── config.py                    # 설정 파일 (각도, 채널, 타이밍)
├── ik_calculator_3d.py          # IK 계산기 (테스트 및 검증용)
├── servo_calibration.py         # 서보 캘리브레이션 도구
├── servo_test.py                # 서보 개별 테스트
├── quick_start.py               # 빠른 시작 스크립트
├── deprecated/                  # 백업 및 이전 버전
│   ├── spot_micro_controller_backup2.py
│   └── spot_micro_controller_backup4.py
├── samples/                     # 참고용 샘플 코드
│   ├── motion_controller.py
│   └── config.py
└── PROJECT_OVERVIEW.md          # 이 문서
```

---

## 🎯 좌표계 시스템

### **기준점: 어깨 회전축**

로봇의 모든 좌표는 **어깨 회전축**을 원점(0,0,0)으로 합니다.

```
       Z (위+)
       ↑
       |
       |
       |________→ X (앞+)
      /
     /
    ↙ Y (오른쪽+)
```

### **좌표축 정의**

| 축 | 방향 | 양수 | 음수 |
|----|------|------|------|
| **X** | 전후 | 앞(+) | 뒤(-) |
| **Y** | 좌우 | 오른쪽(+) | 왼쪽(-) |
| **Z** | 상하 | 위(+) | 아래(-) |

### **예시 좌표**

```python
# 서있는 자세
(X=0, Y=0, Z=-20)    # 중립, 지면에서 20cm 위

# 엎드린 자세
(X=-2, Y=0, Z=-16)   # 약간 뒤, 지면에서 16cm 위

# 걷기 자세
(X=-6.5, Y=0, Z=-17) # 뒤쪽, 지면에서 17cm 위
```

---

## 🔧 역기구학 (IK) 시스템

### **IK 계산 흐름**

```
3D 좌표 (x, y, z)
    ↓
좌우 다리 판단 (is_left)
    ↓
앞뒤 다리 판단 (is_rear)
    ↓
어깨 각도 계산 (Y 좌표 기반)
    ↓
2D IK 계산 (X-Z 평면)
    ↓
모터 각도 (shoulder, upper, lower)
```

### **세그먼트 길이**

```python
UPPER_SEG_LENGTH = 11.0  # 상부 관절 길이 (cm)
LOWER_SEG_LENGTH = 13.5  # 하부 관절 길이 (cm)
IK_SHOULDER_OFFSET = 0.0 # 어깨 오프셋 (cm)

최대 도달 거리 = 11.0 + 13.5 = 24.5cm
최소 도달 거리 = |11.0 - 13.5| = 2.5cm
```

### **각도 정의 (오른쪽 다리 기준)**

| 관절 | 0° | 90° | 180° |
|------|-----|-----|------|
| **Upper** | 뒤 수평 | 수직 아래 | 앞 수평 |
| **Lower** | 뒤로 꺾임 | 앞하단 45° | 앞으로 접힘 |

### **어깨 각도 방향**

| 다리 | 각도 증가 시 | 각도 감소 시 |
|------|------------|------------|
| `front_right` | 바깥(벌림) ↗ | 안쪽 ↙ |
| `front_left` | 안쪽 ↙ | 바깥(벌림) ↗ |
| `rear_right` | 안쪽 ↙ | 바깥(벌림) ↗ |
| `rear_left` | 바깥(벌림) ↗ | 안쪽 ↙ |

---

## 📄 핵심 파일 설명

### 1️⃣ **spot_micro_controller.py**

**메인 컨트롤러 파일** - 모든 동작 제어 로직 포함

#### 주요 함수

**기본 동작**
- `lie_down()` - 엎드리기
- `stand_up()` - 서기
- `tilt_left()` - 왼쪽 기울이기
- `tilt_right()` - 오른쪽 기울이기

**이동 동작**
- `walk_forward(steps_count, step_duration)` - 전진 걷기
- `turn_left(steps_count, step_duration, turn_angle_offset)` - 왼쪽 회전
- `turn_right(steps_count, step_duration, turn_angle_offset)` - 오른쪽 회전

**몸체 제어** (신규 추가)
- `body_move_up_down(height_offset, duration)` - 몸체 상하 이동
- `body_shift_weight(shift_y, shift_x, duration)` - 무게중심 이동

**좌표 제어**
- `set_leg_position_xyz(leg_name, x, y, z, duration, steps)` - 개별 다리 좌표 제어
- `set_all_legs_position_xyz(positions_dict, duration, steps)` - 모든 다리 동시 제어

**역기구학 (IK)**
- `coord_to_angles_3d(x, y, z, is_left, is_rear)` - 좌표 → 각도 변환

**사용 모드**
- `demo_sequence()` - 자동 데모 실행
- `interactive_mode()` - 대화형 제어 모드

---

### 2️⃣ **config.py**

**설정 파일** - 하드웨어 및 동작 파라미터

#### 주요 설정

**하드웨어**
```python
I2C_BUS_NUM = 5
PCA9685_ADDRESS = 0x40
SERVO_FREQUENCY = 50  # Hz
```

**채널 맵핑**
```python
CHANNELS = {
    'front_left':  [14, 13, 12],  # 어깨, 상부, 하부
    'front_right': [10, 9, 8],
    'rear_left':   [6, 5, 4],
    'rear_right':  [2, 1, 0]
}
```

**자세별 각도**
```python
RIGHT_STAND_ANGLES = [90, 45, 130]
LEFT_STAND_ANGLES = [90, 135, 50]

RIGHT_LIE_DOWN_ANGLES = [90, 20, 160]
LEFT_LIE_DOWN_ANGLES = [90, 160, 20]
```

**서보 캘리브레이션 오프셋**
```python
SERVO_CALIBRATION_OFFSET = {
    'front_right': [+3, 0, -10],
    'rear_right': [0, 0, 0],
    'front_left': [+5, 0, 0],
    'rear_left': [+2, -5, 0]
}
```

---

### 3️⃣ **ik_calculator_3d.py**

**IK 계산기 및 검증 도구**

#### 기능

- **Forward Kinematics**: 각도 → 좌표
- **Inverse Kinematics**: 좌표 → 각도
- **정확도 검증**: FK → IK → FK 순환 테스트
- **모든 자세 테스트**: config.py의 모든 자세 각도 검증

#### 실행 방법

```bash
./Dog_venv/bin/python3 ik_calculator_3d.py
```

#### 출력 예시

```
================================================================================
자세: 서있는 자세 (STAND)
================================================================================

[오른쪽 앞 다리 (front_right)]
  입력 각도: 어깨=90°, 상부=45°, 하부=130°
  → Forward Kinematics 3D 좌표:
     X(앞+/뒤-)=  0.90cm, Y(오른쪽+/왼쪽-)=  0.00cm, Z(위+/아래-)=-18.12cm
  → Inverse Kinematics 각도:
     어깨= 90.00°, 상부= 45.00°, 하부=130.00°
  → 오차:
     어깨= 0.00°, 상부= 0.00°, 하부= 0.00°
  → 위치 오차: 0.0000cm
  ✓ 완벽! (모든 오차 < 0.1°)
```

---

### 4️⃣ **servo_calibration.py**

**서보 모터 캘리브레이션 도구**

#### 기능

- 개별 서보 각도 조정
- 실시간 PWM 값 확인
- 캘리브레이션 오프셋 설정
- config.py에 자동 저장

#### 실행 방법

```bash
./Dog_venv/bin/python3 servo_calibration.py
```

---

### 5️⃣ **servo_test.py**

**서보 모터 개별 테스트**

#### 기능

- 전체 서보 스윕 테스트
- 개별 채널 테스트
- 연결 상태 확인

---

## 🎮 동작 기능

### **기본 자세**

| 동작 | 함수 | 설명 |
|------|------|------|
| 엎드리기 | `lie_down()` | 안전한 휴식 자세 |
| 서기 | `stand_up()` | 기본 서있는 자세 |
| 왼쪽 기울이기 | `tilt_left()` | 왼쪽으로 몸체 기울임 |
| 오른쪽 기울이기 | `tilt_right()` | 오른쪽으로 몸체 기울임 |

### **이동 동작**

| 동작 | 함수 | 설명 |
|------|------|------|
| 전진 걷기 | `walk_forward(steps, duration)` | 트로트 보행 (대각선 다리 쌍 교대) |
| 왼쪽 회전 | `turn_left(steps, duration, offset)` | 제자리 왼쪽 회전 |
| 오른쪽 회전 | `turn_right(steps, duration, offset)` | 제자리 오른쪽 회전 |

### **몸체 제어** ⭐ 신규

| 동작 | 함수 | 차이점 |
|------|------|--------|
| 몸체 상하 이동 | `body_move_up_down(height_offset)` | 엎드리기/서기와 다르게 **높이만 조절** |
| 무게중심 이동 | `body_shift_weight(shift_y, shift_x)` | 기울이기와 다르게 **모든 다리 같은 높이 유지** |

### **걷기 시퀀스 (backup2 방식)**

8단계 세분화된 걷기:

```
Phase 1: 오른쪽 앞 + 왼쪽 뒤
  0. 걷기 준비 (STANBY)
  1. 다리 들기 (PUSH)
  2. 앞으로 뻗기 (LIFT)
  3. 착지 (STANBY)

Phase 2: 왼쪽 앞 + 오른쪽 뒤
  4. 다리 들기 (PUSH)
  5. 앞으로 뻗기 (LIFT)
  6. 착지 (STANBY)
```

**좌표 설정 위치**: `spot_micro_controller.py` 596-622줄

---

## ⚙️ 설정 파일

### **config.py 주요 섹션**

#### 1. 하드웨어 설정
```python
I2C_BUS_NUM = 5
PCA9685_ADDRESS = 0x40
SERVO_FREQUENCY = 50
```

#### 2. 채널 맵핑
```python
CHANNELS = {
    'front_left':  [14, 13, 12],
    'front_right': [10, 9, 8],
    'rear_left':   [6, 5, 4],
    'rear_right':  [2, 1, 0]
}
```

#### 3. 자세별 각도
```python
RIGHT_STAND_ANGLES = [90, 45, 130]
LEFT_STAND_ANGLES = [90, 135, 50]
```

#### 4. 서보 캘리브레이션 오프셋
```python
SERVO_CALIBRATION_OFFSET = {
    'front_right': [+3, 0, -10],
    'rear_right': [0, 0, 0],
    'front_left': [+5, 0, 0],
    'rear_left': [+2, -5, 0]
}
```

---

## 🚀 사용 방법

### **1. 기본 실행**

```bash
# 메인 컨트롤러 실행
./Dog_venv/bin/python3 spot_micro_controller.py

# 모드 선택
1. 데모 시퀀스 실행    # 자동 데모
2. 인터랙티브 모드     # 수동 제어
```

### **2. 테스트 모드 (시뮬레이션)**

```bash
# 실제 모터 없이 각도만 출력
./Dog_venv/bin/python3 spot_micro_controller.py --test
```

### **3. IK 계산기 실행**

```bash
# 모든 자세에 대해 IK 정확도 검증
./Dog_venv/bin/python3 ik_calculator_3d.py
```

### **4. 서보 캘리브레이션**

```bash
# 서보 모터 캘리브레이션 도구
./Dog_venv/bin/python3 servo_calibration.py
```

---

## 🎮 인터랙티브 모드 명령어

### **기본 명령어**

| 명령어 | 단축키 | 동작 |
|--------|--------|------|
| lie | 1 | 엎드리기 |
| stand | 2 | 서기 |
| tiltl | 3 | 왼쪽 기울이기 |
| tiltr | 4 | 오른쪽 기울이기 |
| walk | 5 | 걷기 (걸음 수 입력) |
| turnl | 6 | 왼쪽 회전 (스텝 수 입력) |
| turnr | 7 | 오른쪽 회전 (스텝 수 입력) |

### **고급 명령어** ⭐ 신규

| 명령어 | 단축키 | 동작 | 기본값 |
|--------|--------|------|--------|
| up | u | 몸체 위로 | +3cm |
| down | d | 몸체 아래로 | -3cm |
| left | l | 무게중심 왼쪽 | -2cm |
| right | r | 무게중심 오른쪽 | +2cm |

### **기타**

| 명령어 | 단축키 | 동작 |
|--------|--------|------|
| demo | 8 | 전체 데모 실행 |
| xyz | 9 | 개별 다리 좌표 제어 |
| quit | q | 종료 |

---

## 📐 좌표 기반 제어

### **개별 다리 제어**

```python
# 오른쪽 앞다리를 (x=2, y=1, z=-18)cm로 이동
set_leg_position_xyz('front_right', 2.0, 1.0, -18.0, duration=0.5, steps=20)

# 다리 이름: 'front_right', 'front_left', 'rear_right', 'rear_left'
```

### **모든 다리 동시 제어**

```python
# 모든 다리를 동시에 다른 위치로 이동
positions = {
    'front_right': (0, 0, -20),
    'rear_right': (0, 0, -20),
    'front_left': (0, 0, -20),
    'rear_left': (0, 0, -20)
}
set_all_legs_position_xyz(positions, duration=1.0, steps=30)
```

### **몸체 높이 조절**

```python
# 현재 높이에서 3cm 위로
body_move_up_down(3.0)

# 현재 높이에서 3cm 아래로
body_move_up_down(-3.0)
```

### **무게중심 이동**

```python
# 오른쪽으로 2cm 이동
body_shift_weight(2.0)

# 왼쪽으로 2cm 이동
body_shift_weight(-2.0)

# 오른쪽 2cm + 앞 1cm
body_shift_weight(2.0, shift_x=1.0)
```

---

## 📚 개발 히스토리

### **2025-10-31 - 최종 업데이트**

#### ✅ 완료된 작업

1. **좌표계 통일**
   - 모든 파일에서 동일한 좌표계 사용
   - X=앞(+)/뒤(-), Y=오른쪽(+)/왼쪽(-), Z=위(+)/아래(-)

2. **IK 시스템 개선**
   - `spot_micro_controller.py`의 IK 알고리즘을 `ik_calculator_3d.py`에 적용
   - 세그먼트 길이 업데이트: UPPER=11.0cm, LOWER=13.5cm
   - 완벽한 정확도 달성 (모든 오차 < 0.1°)

3. **걷기 시퀀스 개선**
   - `deprecated/spot_micro_controller_backup2.py`의 8단계 시퀀스 적용
   - 좌표 기반으로 변환 (사용자가 좌표 입력 가능)

4. **신규 기능 추가**
   - `body_move_up_down()`: 몸체 상하 이동
   - `body_shift_weight()`: 무게중심 이동 (좌우/전후)
   - 인터랙티브 모드 명령어 추가 (u/d/l/r)

5. **샘플 코드 분석**
   - `samples/motion_controller.py` 분석 완료
   - 유용한 기능 추출 및 좌표계 변환

#### 📝 주요 변경사항

**ik_calculator_3d.py**
- Forward/Inverse Kinematics 완전 재구현
- `is_rear` 파라미터 추가 (앞다리/뒷다리 구분)
- 위치 오차 검증 추가

**spot_micro_controller.py**
- `walk_forward()` 함수 시퀀스 변경 (backup2 방식)
- 좌표 입력 위치 명시 (596-622줄)
- 신규 몸체 제어 함수 추가 (878-961줄)

**config.py**
- 세그먼트 길이 업데이트
- 서보 캘리브레이션 오프셋 추가

---

## 🔍 참고 파일

### **Deprecated (백업)**

- `deprecated/spot_micro_controller_backup2.py` - 8단계 걷기 시퀀스 원본
- `deprecated/spot_micro_controller_backup4.py` - 이전 버전

### **Samples (참고용)**

- `samples/motion_controller.py` - 몸체 제어 참고 코드
- `samples/config.py` - 설정 시스템 참고

---

## 📊 테스트 결과

### **IK 정확도 테스트**

모든 자세에서 **완벽한 정확도** 달성:

```
✓ 서있는 자세 (STAND)      - 위치 오차: 0.0000cm
✓ 엎드린 자세 (LIE DOWN)   - 위치 오차: 0.0000cm
✓ 대기 자세 (STANDBY)      - 위치 오차: 0.0000cm
✓ 걷기 - 다리 앞으로 (LIFT) - 위치 오차: 0.0000cm
✓ 걷기 - 다리 뒤로 (PUSH)  - 위치 오차: 0.0000cm
✓ 회전 - 다리 들기 (TURN)  - 위치 오차: 0.0000cm
```

### **신규 기능 테스트**

```
✓ 몸체 상하 이동 (+3cm)   - 정상 작동
✓ 몸체 상하 이동 (-3cm)   - 정상 작동
✓ 무게중심 이동 (오른쪽)   - 정상 작동
✓ 무게중심 이동 (왼쪽)     - 정상 작동
✓ 무게중심 이동 (복합)     - 정상 작동
```

---

## 🎯 다음 단계 (TODO)

### **걷기 좌표 조정**

`spot_micro_controller.py` 596-622줄의 좌표값을 실제 로봇에 맞게 조정:

```python
# TODO: 아래 좌표값들을 실제 로봇에 맞게 조정하세요

# Phase 0: 걷기 준비 자세 (STANBY)
STANBY_FR = (0, 0, -20)  # front_right (X, Y, Z) cm
STANBY_RL = (0, 0, -20)  # rear_left
STANBY_FL = (0, 0, -20)  # front_left
STANBY_RR = (0, 0, -20)  # rear_right

# Phase 1-1: 다리 들기 (PUSH 위치 - 뒤쪽으로 들기)
PUSH_FR = (-2, 0, -18)   # front_right
PUSH_RL = (-2, 0, -18)   # rear_left

# ... (계속)
```

### **추가 기능 구현**

- [ ] 후진 걷기 (좌표 기반)
- [ ] 측면 걷기 (좌표 기반)
- [ ] 계단 오르기
- [ ] 장애물 회피

---

## 📞 문의 및 지원

- **프로젝트 경로**: `/home/sunrise/Desktop/Dog_V3/`
- **Python 환경**: `./Dog_venv/bin/python3`
- **플랫폼**: RDK X5
- **하드웨어**: MG966R 서보 × 12, PCA9685

---

**마지막 업데이트**: 2025-10-31
**버전**: 3.0 (좌표 기반 IK 시스템)

