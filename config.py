#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spot Micro Robot - Configuration File
각도 설정 및 하드웨어 파라미터를 쉽게 조정할 수 있습니다.
"""

# ============================================================================
# 하드웨어 설정
# ============================================================================

# I2C 설정
I2C_BUS_NUM = 5
PCA9685_ADDRESS = 0x40

# PWM 설정
SERVO_FREQUENCY = 50  # Hz
SERVO_MIN_TICK = 150  # 0도에 해당하는 PWM 값
SERVO_MAX_TICK = 600  # 180도에 해당하는 PWM 값

# PCA9685 채널 맵핑
# 각 다리: [어깨(좌우), 상부관절(상하), 하부관절(상하)]
CHANNELS = {
    'front_left':  [14, 13, 12],  # 모터 1, 2, 3
    'front_right': [10, 9, 8],     # 모터 4, 5, 6
    'rear_left':   [6, 5, 4],      # 모터 7, 8, 9
    'rear_right':  [2, 1, 0]       # 모터 10, 11, 12
}

# ============================================================================
# 각도 설정 (자세별)
# ============================================================================

# 서있는 자세 각도 [어깨, 상부관절, 하부관절]
# 팁: 상부와 하부 관절의 각도를 조정하여 로봇의 높이를 변경할 수 있습니다.
RIGHT_STAND_ANGLES = [90, 45, 130]
LEFT_STAND_ANGLES = [90, 135, 50]

# 엎드린 자세 각도
# 팁: 이 자세는 로봇의 안전한 시작/종료 자세입니다.
RIGHT_LIE_DOWN_ANGLES = [90, 20, 160]
LEFT_LIE_DOWN_ANGLES = [90, 160, 20]

# ============================================================================
# 걷기 동작 각도 (세분화된 단계)
# ============================================================================

RIGHT_STANDBY_ANGLES = [90, 30, 140]
LEFT_STANDBY_ANGLES = [90, 150, 40]

# (공중에서 앞으로)
# 팁: 이 각도가 너무 높으면 불안정해지고, 너무 낮으면 걸음이 작아집니다.
RIGHT_LIFT_ANGLES = [90, 40, 145]  # 앞으로 뻗기
LEFT_LIFT_ANGLES = [90, 140, 35]   # 앞으로 뻗기

# (추진력 생성)
RIGHT_PUSH_ANGLES = [90, 30, 130]  # 뒤로 밀기
LEFT_PUSH_ANGLES = [90, 150, 50]   # 뒤로 밀기

# 좌우 기울이기 각도 조정값
# 팁: 이 값이 클수록 더 많이 기울어집니다. (권장: 10-20)
TILT_ANGLE_OFFSET = 17

# ============================================================================
# 회전 동작 각도
# ============================================================================

# 회전 시 어깨 각도 오프셋 (좌우로 벌림)
# 팁: 값이 클수록 회전 반경이 작아집니다 (권장: 15-30)
TURN_SHOULDER_OFFSET = 20

# 회전 시 다리를 들어올린 높이 (서있는 자세보다 살짝만 들어올림)
RIGHT_TURN_LIFT_ANGLES = [90, 55, 125]  # 회전 중 다리 들기 (낮게)
LEFT_TURN_LIFT_ANGLES = [90, 125, 55]   # 회전 중 다리 들기 (낮게)

# ============================================================================
# 동작 타이밍 설정
# ============================================================================

# 각 동작의 기본 실행 시간 (초)
DEFAULT_LIE_DOWN_DURATION = 1.0
DEFAULT_STAND_UP_DURATION = 1.0
DEFAULT_TILT_DURATION = 0.5
DEFAULT_WALK_STEP_DURATION = 0.3

# 부드러운 이동을 위한 기본 스텝 수
# 팁: 값이 클수록 더 부드럽게 움직이지만 처리 시간이 늘어납니다.
DEFAULT_INTERPOLATION_STEPS = 20

# ============================================================================
# 보행 설정
# ============================================================================

# 한 번의 걷기 명령에서 실행할 기본 걸음 수
DEFAULT_WALK_STEPS = 4

# 트로트 보행 페이즈 타이밍 비율
# [다리 들기 시간 비율, 다리 내리기 시간 비율]
WALK_PHASE_RATIO = [0.5, 0.5]

# ============================================================================
# 안전 설정
# ============================================================================

# 서보 각도 범위 제한 (0-180도)
ANGLE_MIN_LIMIT = 20
ANGLE_MAX_LIMIT = 160

# 비상 정지 시 자세 (lie_down 사용)
EMERGENCY_POSE = 'lie_down'

# ============================================================================
# 디버그 설정
# ============================================================================

# 상세 로깅 활성화
VERBOSE_LOGGING = False

# 각 동작 후 대기 시간 (디버그용)
DEBUG_DELAY_AFTER_ACTION = 0.0

# ============================================================================
# 고급 설정 (신중하게 수정)
# ============================================================================

# 서보별 미세 조정 오프셋 (필요시 사용)
# 예: 특정 서보가 정확히 90도로 설정되지 않을 때
SERVO_CALIBRATION_OFFSET = {
    'front_left': [5, 0, 0],   # [어깨, 상부, 하부] 각도 오프셋
    'front_right': [-3, 0, -10],
    'rear_left': [-2, -5, 0],
    'rear_right': [0, 0, 0],
}

# 서보별 방향 반전 (필요시 사용)
# True: 각도 반전 (180 - angle), False: 정상
SERVO_REVERSE = {
    # 'front_left': [False, False, False],
    # 'front_right': [False, False, False],
    # 'rear_left': [False, False, False],
    # 'rear_right': [False, False, False],
}

# ============================================================================
# 프리셋 자세 (추가 자세를 정의할 수 있습니다)
# ============================================================================

PRESET_POSES = {
    'stand': {
        'front_right': RIGHT_STAND_ANGLES,
        'rear_right': RIGHT_STAND_ANGLES,
        'front_left': LEFT_STAND_ANGLES,
        'rear_left': LEFT_STAND_ANGLES
    },
    'lie_down': {
        'front_right': RIGHT_LIE_DOWN_ANGLES,
        'rear_right': RIGHT_LIE_DOWN_ANGLES,
        'front_left': LEFT_LIE_DOWN_ANGLES,
        'rear_left': LEFT_LIE_DOWN_ANGLES
    },
    # 여기에 커스텀 자세를 추가할 수 있습니다
    # 'sit': {
    #     'front_right': [90, 45, 135],
    #     'rear_right': [90, 90, 90],
    #     'front_left': [90, 135, 45],
    #     'rear_left': [90, 90, 90]
    # },
}

# ============================================================================
# 설정 검증 함수
# ============================================================================

def validate_config():
    """설정 값의 유효성을 검증합니다."""
    errors = []
    
    # 각도 범위 검증
    for pose_name, pose_angles in PRESET_POSES.items():
        for leg_name, angles in pose_angles.items():
            for i, angle in enumerate(angles):
                if not (ANGLE_MIN_LIMIT <= angle <= ANGLE_MAX_LIMIT):
                    errors.append(
                        f"{pose_name} 자세의 {leg_name} 다리, "
                        f"관절 {i}의 각도 {angle}가 범위를 벗어났습니다. "
                        f"(범위: {ANGLE_MIN_LIMIT}-{ANGLE_MAX_LIMIT})"
                    )
    
    # PWM 범위 검증
    if SERVO_MIN_TICK >= SERVO_MAX_TICK:
        errors.append(
            f"SERVO_MIN_TICK({SERVO_MIN_TICK})이 "
            f"SERVO_MAX_TICK({SERVO_MAX_TICK})보다 작아야 합니다."
        )
    
    # 채널 검증
    all_channels = []
    for leg_channels in CHANNELS.values():
        all_channels.extend(leg_channels)
    
    if len(all_channels) != len(set(all_channels)):
        errors.append("중복된 PCA9685 채널이 있습니다.")
    
    if errors:
        print("⚠️  설정 오류 발견:")
        for error in errors:
            print(f"  - {error}")
        return False
    
    print("✓ 설정 검증 완료")
    return True

if __name__ == "__main__":
    print("Spot Micro 로봇 설정 파일")
    print("="*60)
    validate_config()
    print("\n현재 설정:")
    print(f"  I2C 버스: {I2C_BUS_NUM}")
    print(f"  PCA9685 주소: 0x{PCA9685_ADDRESS:02X}")
    print(f"  PWM 주파수: {SERVO_FREQUENCY}Hz")
    print(f"  서있는 자세 (오른쪽): {RIGHT_STAND_ANGLES}")
    print(f"  서있는 자세 (왼쪽): {LEFT_STAND_ANGLES}")
    print(f"  기울이기 오프셋: {TILT_ANGLE_OFFSET}도")