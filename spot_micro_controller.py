#!/usr/bin/env python3
"""
Spot Micro Robot Controller for RDK X5
MG966R 서보 모터 사용, PCA9685 제어
"""

import time
import sys
import math
import config

# 테스트 모드 설정 (True: 각도만 출력, False: 실제 모터 제어)
TEST_MODE = False

if not TEST_MODE:
    try:
        import Adafruit_PCA9685
    except ImportError:
        print("경고: Adafruit_PCA9685 라이브러리를 찾을 수 없습니다. 테스트 모드로 전환합니다.")
        TEST_MODE = True

# ============================================================================
# 하드웨어 설정 (config.py에서 가져옴)
# ============================================================================
I2C_BUS_NUM = config.I2C_BUS_NUM
PCA9685_ADDRESS = config.PCA9685_ADDRESS
SERVO_FREQUENCY = config.SERVO_FREQUENCY
SERVO_MIN_TICK = config.SERVO_MIN_TICK
SERVO_MAX_TICK = config.SERVO_MAX_TICK

# PCA9685 채널 맵핑 (각 다리당 3개의 서보)
# [어깨(좌우), 상부관절(상하), 하부관절(상하)]
channels = config.CHANNELS

# ============================================================================
# 서보 캘리브레이션 오프셋 (config.py에서 가져옴)
# ============================================================================
# 각 서보의 캘리브레이션 오프셋값 (각도 단위)
SERVO_CALIBRATION_OFFSET = config.SERVO_CALIBRATION_OFFSET

# ============================================================================
# IK (Inverse Kinematics) 설정
# ============================================================================
UPPER_SEG_LENGTH = 11.0  # 상부 관절 길이 (cm)
LOWER_SEG_LENGTH = 13.5  # 하부 관절 길이 (cm)
IK_SHOULDER_OFFSET = 0.0  # 어깨 오프셋 (cm) - 오프셋 없음

# ============================================================================
# 동작 기본 설정 (모든 값은 사용자가 조정 가능)
# ============================================================================
# 엎드린 자세 좌표
LIE_X = 2.35     # 엎드린 자세 X (cm)
LIE_Y = 0.0     # 엎드린 자세 Y (cm)
LIE_Z = -8.4   # 엎드린 자세 Z (cm)

# 걷기/회전 기본 자세 좌표 => 기본 좌표로 변경
STANDBY_X = 0.82   # 걷기 시 앞뒤 중립 위치 (cm)
STANDBY_Y = 0.0    # 걷기 시 좌우 중립 위치 (cm)
STANDBY_Z = -14.18  # 걷기 시 높이 (cm)

# 동작 공통 파라미터
TURN_LIFT_HEIGHT = 2.0   # 회전 시 다리 들어올리는 높이 (cm)
TILT_HEIGHT = 4.0        # 기울이기 높이 차이 (cm)

# ============================================================================
# 전역 변수
# ============================================================================
pca = None
# 좌표 기반 초기 각도 (IK 함수가 정의된 후, init_pca9685()에서 초기화됨)
current_angles = None

# ============================================================================
# IK (Inverse Kinematics) 함수
# ============================================================================

def coord_to_angles_3d(x, y, z, is_left=False, is_rear=False):
    """
    3D 좌표를 3개의 관절 각도로 변환 (Inverse Kinematics)

    좌표계 (어깨 회전축 기준):
        X: 앞(+) / 뒤(-)
        Y: 오른쪽(+) / 왼쪽(-)
        Z: 위(+) / 아래(-)

    각도 정의 (오른쪽 다리 기준, 지면 수평선 기준):
        Upper: 0°=뒤 수평, 90°=수직 아래, 180°=앞 수평
        Lower: 0°=뒤로 꺾임, 90°=앞하단 45°, 180°=앞으로 접힘

    어깨 각도 방향:
        front_right: 각도↑ = 바깥(벌림)
        front_left: 각도↓ = 바깥(벌림)
        rear_right: 각도↓ = 바깥(벌림)
        rear_left: 각도↑ = 바깥(벌림)

    Args:
        x: 목표 x 좌표 (앞+/뒤-, cm)
        y: 목표 y 좌표 (오른쪽+/왼쪽-, cm)
        z: 목표 z 좌표 (위+/아래-, cm)
        is_left: 왼쪽 다리 여부
        is_rear: 뒷다리 여부

    Returns:
        (shoulder, upper, lower): 3개 관절 각도 (도)
        실패 시 None 반환
    """
    # 왼쪽 다리는 Y를 반전시켜 오른쪽처럼 계산
    if is_left:
        y = -y

    # 1. 어깨 각도 계산
    # 어깨는 좌우 회전만 담당 (Y 좌표로만 결정)
    # Y=0 → 90° (정면), Y>0 → 90°+ (바깥), Y<0 → 90°- (안쪽)
    if y != 0:
        shoulder_angle_offset = math.degrees(math.atan(y / (abs(x) + IK_SHOULDER_OFFSET)))
    else:
        shoulder_angle_offset = 0

    shoulder = 90 + shoulder_angle_offset  # 90도가 정면

    # 어깨 방향 조정
    # front_right: 그대로 (각도↑ = 벌림)
    # front_left: 반전 필요 (각도↓ = 벌림)
    # rear_right: 반전 필요 (각도↓ = 벌림)
    # rear_left: 그대로 (각도↑ = 벌림)
    # 패턴: front는 left만 반전, rear는 right만 반전
    if (not is_rear and is_left) or (is_rear and not is_left):
        # front_left 또는 rear_right
        shoulder = 180 - shoulder

    # 2. 어깨 오프셋 적용 (X 방향으로만)
    # 어깨 오프셋은 항상 다리 방향(X)으로 5cm 떨어져 있음
    effective_x = x - IK_SHOULDER_OFFSET if x >= 0 else x + IK_SHOULDER_OFFSET

    # 3. 2D IK (수직 평면, effective_x와 z만 사용)
    distance = math.sqrt(effective_x**2 + z**2)

    # 도달 가능 범위 체크
    max_reach = UPPER_SEG_LENGTH + LOWER_SEG_LENGTH
    min_reach = abs(UPPER_SEG_LENGTH - LOWER_SEG_LENGTH)

    if distance > max_reach or distance < min_reach:
        print(f"⚠ 좌표 ({x:.1f}, {y:.1f}, {z:.1f})은 도달 불가능")
        return None

    # 목표점의 각도 (수평선으로부터, Z가 위쪽 +)
    angle_to_target = math.atan2(z, effective_x)

    # 코사인 법칙
    cos_alpha = (UPPER_SEG_LENGTH**2 + distance**2 - LOWER_SEG_LENGTH**2) / \
                (2 * UPPER_SEG_LENGTH * distance)
    cos_alpha = max(-1.0, min(1.0, cos_alpha))
    alpha = math.acos(cos_alpha)

    # upper segment의 절대 각도 (수평선 기준)
    upper_abs_rad = angle_to_target - alpha

    # 코사인 법칙으로 관절 각도
    cos_beta = (UPPER_SEG_LENGTH**2 + LOWER_SEG_LENGTH**2 - distance**2) / \
               (2 * UPPER_SEG_LENGTH * LOWER_SEG_LENGTH)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    # lower segment의 절대 각도
    lower_abs_rad = upper_abs_rad + (math.pi - beta)

    # 절대 각도를 모터 각도로 변환
    # 절대 각도: 0°=앞 수평, -90°=아래, -180°=뒤 수평
    # upper 모터: 0°=뒤, 90°=아래, 180°=앞
    # lower 모터: 0°=뒤, 90°=앞하단45°, 180°=앞
    # 변환: 모터각도 = 180° - 절대각도
    upper = 180 + math.degrees(upper_abs_rad)
    lower = 180 + math.degrees(lower_abs_rad)

    # 왼쪽 다리는 180도 대칭
    if is_left:
        upper = 180 - upper
        lower = 180 - lower
        
    print(f"각도 : [{shoulder}, {upper}, {lower}]")

    return (shoulder, upper, lower)

def set_leg_position_xyz(leg_name, x, y, z, duration=0.5, steps=20):
    """
    개별 다리를 3D 좌표로 제어

    Args:
        leg_name: 다리 이름 ('front_left', 'front_right', 'rear_left', 'rear_right')
        x: 목표 x 좌표 (전후, cm)
        y: 목표 y 좌표 (좌우, cm)
        z: 목표 z 좌표 (상하, cm)
        duration: 이동 시간 (초)
        steps: 부드러운 이동을 위한 스텝 수
    """
    # 왼쪽 다리 여부 판단
    is_left = 'left' in leg_name
    # 뒷다리 여부 판단
    is_rear = 'rear' in leg_name

    # IK 계산
    result = coord_to_angles_3d(x, y, z, is_left, is_rear)

    if result is None:
        print(f"✗ {leg_name} 다리를 목표 위치로 이동할 수 없습니다")
        return False

    shoulder, upper, lower = result

    # 검증: FK로 다시 계산하여 정확도 확인
    # 모터 각도를 절대 각도로 변환
    if is_left:
        # 왼쪽 다리: 180도 대칭 원복
        upper_motor = 180 - upper
        lower_motor = 180 - lower
    else:
        upper_motor = upper
        lower_motor = lower

    # 모터 각도 → 절대 각도 (수평선 기준)
    # 모터 180° → 절대 0°(앞 수평), 모터 90° → 절대 -90°(아래), 모터 0° → 절대 -180°(뒤)
    # 절대각도 = 모터각도 - 180°
    upper_abs_rad = math.radians(upper_motor - 180)
    lower_abs_rad = math.radians(lower_motor - 180)

    # upper segment 끝점 (수평선 기준)
    upper_end_x = UPPER_SEG_LENGTH * math.cos(upper_abs_rad)
    upper_end_z = UPPER_SEG_LENGTH * math.sin(upper_abs_rad)

    # lower segment 끝점
    lower_end_x = upper_end_x + LOWER_SEG_LENGTH * math.cos(lower_abs_rad)
    lower_end_z = upper_end_z + LOWER_SEG_LENGTH * math.sin(lower_abs_rad)

    # 어깨 회전 적용 (방향 반전 고려)
    shoulder_motor = shoulder
    # front_left 또는 rear_right는 반전되어 있으므로 원복
    if (not is_rear and is_left) or (is_rear and not is_left):
        shoulder_motor = 180 - shoulder_motor

    # 어깨 각도로부터 Y 오프셋 계산
    shoulder_offset_angle = shoulder_motor - 90  # 90도가 정면
    shoulder_offset_rad = math.radians(shoulder_offset_angle)

    # 어깨 오프셋 복원 (X 방향으로만 적용)
    # lower_end_x에 어깨 오프셋을 더해서 원래 X 좌표 복원
    if lower_end_x >= 0:
        verify_x = lower_end_x + IK_SHOULDER_OFFSET
    else:
        verify_x = lower_end_x - IK_SHOULDER_OFFSET

    # Y는 어깨 각도로부터 계산
    # 어깨 회전에 의한 Y 오프셋
    verify_y = (abs(verify_x)) * math.tan(shoulder_offset_rad)
    verify_z = lower_end_z

    # 왼쪽 다리는 Y 반전 복구
    if is_left:
        verify_y = -verify_y

    error = math.sqrt((x - verify_x)**2 + (y - verify_y)**2 + (z - verify_z)**2)

    if TEST_MODE:
        print(f"[좌표 제어] {leg_name}")
        print(f"  목표 좌표: X={x:.1f}cm, Y={y:.1f}cm, Z={z:.1f}cm")
        print(f"  계산된 각도: 어깨={shoulder:.1f}°, 상부={upper:.1f}°, 하부={lower:.1f}°")
        print(f"  검증 좌표: X={verify_x:.2f}cm, Y={verify_y:.2f}cm, Z={verify_z:.2f}cm")
        print(f"  위치 오차: {error:.4f}cm")

    # 각도로 다리 이동
    angles = [shoulder, upper, lower]
    print(f"{leg_name} 이동 각도 : {angles}")
    
    set_leg_angles(leg_name, angles, duration, steps)

    return True


# ============================================================================
# 초기 각도 계산 함수
# ============================================================================
def _calculate_initial_angles():
    """
    엎드린 자세의 초기 각도 계산 (좌표 기반 IK 사용)

    Returns:
        dict: 각 다리의 초기 각도 {'leg_name': [shoulder, upper, lower]}
    """
    angles = {}
    legs_config = [
        ('front_right', False, False),
        ('rear_right', False, True),
        ('front_left', True, False),
        ('rear_left', True, True)
    ]

    for leg_name, is_left, is_rear in legs_config:
        result = coord_to_angles_3d(LIE_X, LIE_Y, LIE_Z, is_left, is_rear)
        if result:
            angles[leg_name] = list(result)
        else:
            # IK 실패 시 안전한 기본값 (90도 정면, 중립 자세)
            print(f"⚠ {leg_name}: IK 계산 실패, 기본값 사용")
            angles[leg_name] = [90.0, 90.0, 90.0]

    return angles

def set_all_legs_position_xyz(positions_dict, duration=0.5, steps=20):
    """
    모든 다리를 3D 좌표로 동시에 제어

    Args:
        positions_dict: {'front_left': (x, y, z), 'front_right': (x, y, z), ...}
        duration: 이동 시간 (초)
        steps: 부드러운 이동을 위한 스텝 수
    """
    # 각 다리의 좌표를 각도로 변환
    angles_dict = {}

    for leg_name, (x, y, z) in positions_dict.items():
        # 왼쪽 다리 여부 판단
        is_left = 'left' in leg_name
        # 뒷다리 여부 판단
        is_rear = 'rear' in leg_name

        # IK 계산
        result = coord_to_angles_3d(x, y, z, is_left, is_rear)

        if result is None:
            print(f"✗ {leg_name} 다리를 목표 위치로 이동할 수 없습니다")
            return False

        shoulder, upper, lower = result
        angles_dict[leg_name] = [shoulder, upper, lower]

        if TEST_MODE:
            print(f"[좌표 제어] {leg_name}: ({x:.1f}, {y:.1f}, {z:.1f})cm → [{shoulder:.1f}°, {upper:.1f}°, {lower:.1f}°]")

    # 모든 다리를 동시에 각도로 이동
    set_all_legs_angles(angles_dict, duration, steps)

    return True


# ============================================================================
# 초기화 함수
# ============================================================================
def init_pca9685():
    """PCA9685 및 초기 각도 초기화"""
    global pca, current_angles

    # 좌표 기반 초기 각도 계산
    if current_angles is None:
        current_angles = _calculate_initial_angles()
        print("✓ 초기 각도 계산 완료 (좌표 기반 IK)")

    if TEST_MODE:
        print("[테스트 모드] PCA9685 초기화 시뮬레이션")
        return True

    try:
        pca = Adafruit_PCA9685.PCA9685(address=PCA9685_ADDRESS, busnum=I2C_BUS_NUM)
        pca.set_pwm_freq(SERVO_FREQUENCY)
        print(f"✓ I2C 버스 {I2C_BUS_NUM}번에서 PCA9685가 성공적으로 초기화되었습니다.")
        return True
    except Exception as e:
        print(f"✗ I2C 초기화 오류: {e}")
        return False

# ============================================================================
# 저수준 서보 제어 함수
# ============================================================================
def _set_servo_pwm(channel, angle):
    """특정 채널의 서보를 지정된 각도로 이동"""
    # 각도 범위 제한 (서보 안전 보호)
    angle = max(0, min(180, angle))

    if TEST_MODE:
        # 테스트 모드: 각도만 출력
        return

    # PWM 값 계산 및 적용
    pwm_value = int(SERVO_MIN_TICK + (SERVO_MAX_TICK - SERVO_MIN_TICK) * angle / 180.0)
    pca.set_pwm(channel, 0, pwm_value)

def set_leg_angles(leg_name, angles, duration=0.5, steps=20):
    """
    특정 다리의 모든 관절을 즉시 이동 (보간 없음)

    Args:
        leg_name: 다리 이름 ('front_left', 'front_right', 'rear_left', 'rear_right')
        angles: [어깨, 상부관절, 하부관절] 각도 리스트
        duration: (사용 안 함, 호환성 유지)
        steps: (사용 안 함, 호환성 유지)
    """
    if leg_name not in channels:
        print(f"경고: 알 수 없는 다리 이름 '{leg_name}'")
        return

    leg_channels = channels[leg_name]
    start_angles = current_angles[leg_name].copy()

    # offset 적용 (config.py의 SERVO_CALIBRATION_OFFSET 사용)
    angles_with_offset = angles.copy()
    if leg_name in config.SERVO_CALIBRATION_OFFSET:
        offsets = config.SERVO_CALIBRATION_OFFSET[leg_name]
        for i in range(len(angles_with_offset)):
            angles_with_offset[i] += offsets[i]

    if TEST_MODE:
        if leg_name in config.SERVO_CALIBRATION_OFFSET:
            print(f"[테스트] {leg_name}: {start_angles} → {angles} (offset 적용 후: {angles_with_offset})")
        else:
            print(f"[테스트] {leg_name}: {start_angles} → {angles}")

    # 보간 없이 바로 이동 (빠르고 정확한 동작)
    for i, channel in enumerate(leg_channels):
        _set_servo_pwm(channel, angles_with_offset[i])

    # 현재 각도 업데이트 (offset이 적용된 각도로)
    current_angles[leg_name] = angles_with_offset.copy()
    
def set_all_legs_angles(angles_dict, duration=0.5, steps=20):
    """
    모든 다리를 동시에 즉시 이동 (보간 없음)

    Args:
        angles_dict: {'front_left': [...], 'front_right': [...], ...}
        duration: (사용 안 함, 호환성 유지)
        steps: (사용 안 함, 호환성 유지)
    """
    # offset 적용된 각도 딕셔너리 생성
    angles_with_offset_dict = {}
    for leg_name, target_angles in angles_dict.items():
        angles_with_offset = target_angles.copy()
        if leg_name in config.SERVO_CALIBRATION_OFFSET:
            offsets = config.SERVO_CALIBRATION_OFFSET[leg_name]
            for i in range(len(angles_with_offset)):
                angles_with_offset[i] += offsets[i]
        angles_with_offset_dict[leg_name] = angles_with_offset

    # 시작 각도 저장
    start_angles_dict = {leg: current_angles[leg].copy() for leg in angles_dict.keys()}

    if TEST_MODE:
        print(f"[테스트] 모든 다리 이동:")
        for leg_name, target_angles in angles_dict.items():
            if leg_name in config.SERVO_CALIBRATION_OFFSET:
                print(f"  {leg_name}: {start_angles_dict[leg_name]} → {target_angles} (offset 적용 후: {angles_with_offset_dict[leg_name]})")
            else:
                print(f"  {leg_name}: {start_angles_dict[leg_name]} → {target_angles}")

    # 보간 없이 바로 이동 (빠르고 정확한 동작)
    for leg_name in angles_dict.keys():
        leg_channels = channels[leg_name]
        target_angles_with_offset = angles_with_offset_dict[leg_name]

        for i, channel in enumerate(leg_channels):
            _set_servo_pwm(channel, target_angles_with_offset[i])

    # 현재 각도 업데이트 (offset이 적용된 각도로)
    for leg_name in angles_dict.keys():
        current_angles[leg_name] = angles_with_offset_dict[leg_name].copy()

# ============================================================================
# 고수준 동작 함수
# ============================================================================

def lie_down(duration=1.0):
    """
    엎드리기 동작 (좌표 기반, 네 발 동시 동작)
    """
    print("동작: 엎드리기")

    # 동작 단계 수
    steps = max(int(duration * 30), 10)  # 최소 10 스텝

    # 모든 다리를 동시에 엎드린 위치로 이동
    positions = {
        'front_right': (LIE_X, LIE_Y, LIE_Z),
        'rear_right': (LIE_X, LIE_Y, LIE_Z),
        'front_left': (LIE_X, LIE_Y, LIE_Z),
        'rear_left': (LIE_X, LIE_Y, LIE_Z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print("✓ 엎드리기 완료")

def stand_up(duration=1.0):
    """
    서기 동작 (좌표 기반, 네 발 동시 동작)
    """
    print("동작: 서기")

    # 동작 단계 수
    steps = max(int(duration * 30), 10)  # 최소 10 스텝

    # 모든 다리를 동시에 서있는 위치로 이동
    positions = {
        'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print("✓ 서기 완료")

def tilt_left(duration=0.5, tilt_height=0.2):
    """
    왼쪽으로 기울이기 (좌표 기반, 네 발 동시 동작)

    왼쪽 다리는 낮게, 오른쪽 다리는 높게
    """
    print(f"동작: 왼쪽 기울이기 (높이 차이: {tilt_height}cm)")

    # 동작 단계 수
    steps = max(int(duration * 30), 10)

    # 왼쪽 다리들은 더 낮게, 오른쪽 다리들은 더 높게
    positions = {
        'front_right': (STANDBY_X, STANDBY_Y + tilt_height, STANDBY_Z),
        'rear_right': (STANDBY_X, STANDBY_Y + tilt_height, STANDBY_Z),
        'front_left': (STANDBY_X, STANDBY_Y + tilt_height, STANDBY_Z),
        'rear_left': (STANDBY_X, STANDBY_Y + tilt_height, STANDBY_Z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print("✓ 왼쪽 기울이기 완료")

def tilt_right(duration=0.5, tilt_height=0.2):
    """
    오른쪽으로 기울이기 (좌표 기반, 네 발 동시 동작)

    오른쪽 다리는 낮게, 왼쪽 다리는 높게

    Args:
        duration: 동작 시간 (초)
        tilt_height: 기울임 높이 차이 (cm)
    """
    print(f"동작: 오른쪽 기울이기 (높이 차이: {tilt_height}cm)")

    # 동작 단계 수
    steps = max(int(duration * 30), 10)

    # 오른쪽 다리들은 더 낮게, 왼쪽 다리들은 더 높게
    positions = {
        'front_right': (STANDBY_X, STANDBY_Y - tilt_height, STANDBY_Z),
        'rear_right': (STANDBY_X, STANDBY_Y - tilt_height, STANDBY_Z),
        'front_left': (STANDBY_X, STANDBY_Y - tilt_height, STANDBY_Z),
        'rear_left': (STANDBY_X, STANDBY_Y - tilt_height, STANDBY_Z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print("✓ 오른쪽 기울이기 완료")

def walk_forward(steps_count=4, step_duration=0.3):
    """
    전진 걷기 동작 (좌표 기반, backup2 시퀀스 사용)
    대각선 다리 쌍을 교대로 움직이는 트로트 보행
    """
    print(f"동작: 전진 걷기 ({steps_count} 스텝)")

    # ============================================================
    # 걷기 좌표 설정 (사용자 입력 위치)
    # ============================================================
    # Phase 1-1: 다리 들기 (PUSH 위치 - 뒤쪽으로 밀기)
    PUSH_COORD = (-0.85, 0, -15.84)   

    # Phase 1-2: 앞으로 뻗기 (PUSH 유지, 지지다리는 LIFT로)
    LIFT_COORD = (2.63, 0, -14.81)    

    # ============================================================

    # 타이밍 설정 (각 단계별 시간 비율)
    push_time = step_duration * 0.3    # 밀기: 30%
    lift_time = step_duration * 0.3    # 들기: 30%
    land_time = step_duration * 0.4    # 착지: 40%
    
    for step in range(steps_count):
        print(f"  스텝 {step + 1}/{steps_count} - Phase 1: 오른쪽 앞/왼쪽 뒤 이동")

        # ===== Phase 1: 오른쪽 앞 + 왼쪽 뒤 이동 =====

        # # 0. 걷기 준비 (높이 들어올리기)
        # set_all_legs_position_xyz({
        #     'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        #     'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        #     'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
        #     'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        # }, lift_time, 3)
        # time.sleep(lift_time)

        # 1. 다리 들기 (지면에서 떼기)
        set_all_legs_position_xyz({
            'front_right': PUSH_COORD,
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': PUSH_COORD
        }, push_time, 3)
        time.sleep(push_time)

        # 2. 앞으로 뻗기 (공중에서 앞으로 이동)
        set_all_legs_position_xyz({
            'front_right': PUSH_COORD,
            'rear_right': LIFT_COORD,
            'front_left': LIFT_COORD,
            'rear_left': PUSH_COORD            
        }, lift_time, 3)
        time.sleep(lift_time)

        # 3. 착지하기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        time.sleep(land_time)

        # ===== Phase 2: 왼쪽 앞 + 오른쪽 뒤 이동 =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 2: 왼쪽 앞/오른쪽 뒤 이동")

        # 4. 다리 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': PUSH_COORD,    
            'front_left': PUSH_COORD,
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, push_time * 0.5, 5)
        time.sleep(push_time * 0.5)

        # 5. 앞으로 뻗기 (착지 전)
        set_all_legs_position_xyz({
            'front_right': LIFT_COORD,
            'rear_right': PUSH_COORD,
            'front_left': PUSH_COORD,
            'rear_left': LIFT_COORD
        }, lift_time * 0.5, 5)
        time.sleep(lift_time * 0.5)

        # 6. 착지하기 (중립 자세로 복귀)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)            
        }, land_time, 3)
        time.sleep(land_time)

    print("✓ 걷기 완료")

def walk_backward(steps_count=4, step_duration=0.3):
    """
    후진 걷기 동작 (좌표 기반, backup2 시퀀스 사용)
    대각선 다리 쌍을 교대로 움직이는 트로트 보행
    """
    print(f"동작: 전진 걷기 ({steps_count} 스텝)")
    
    # ============================================================
    # 걷기 좌표 설정 (사용자 입력 위치)
    # ============================================================
    # Phase 1-1: 다리 들기 (PUSH 위치 - 뒤쪽으로 밀기)
    BACK_PUSH_COORD = (2.49, 0, -15.84)
    # Phase 1-2: 앞으로 뻗기 (PUSH 유지, 지지다리는 LIFT로)
    BACK_LIFT_COORD = (-0.99, 0, -14.81)
    
    # ============================================================
    # 타이밍 설정 (각 단계별 시간 비율)
    push_time = step_duration * 0.3    # 밀기: 30%
    lift_time = step_duration * 0.3    # 들기: 30%
    land_time = step_duration * 0.4    # 착지: 40%
    
    for step in range(steps_count):
        print(f"  스텝 {step + 1}/{steps_count} - Phase 1: 오른쪽 앞/왼쪽 뒤 이동")
        # ===== Phase 1: 오른쪽 앞 + 왼쪽 뒤 이동 =====
        # 1. 다리 들기 (지면에서 떼기)
        set_all_legs_position_xyz({
            'front_right': BACK_PUSH_COORD,
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': BACK_PUSH_COORD
        }, push_time, 3)
        
        time.sleep(push_time)
        
        # 2. 앞으로 뻗기 (공중에서 앞으로 이동)
        set_all_legs_position_xyz({
            'front_right': BACK_PUSH_COORD,
            'rear_right': BACK_LIFT_COORD,
            'front_left': BACK_LIFT_COORD,
            'rear_left': BACK_PUSH_COORD
        }, lift_time, 3)
        
        time.sleep(lift_time)
        
        # 3. 착지하기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        
        time.sleep(land_time)
        
        # ===== Phase 2: 왼쪽 앞 + 오른쪽 뒤 이동 =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 2: 왼쪽 앞/오른쪽 뒤 이동")
        # 4. 다리 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': BACK_PUSH_COORD,
            'front_left': BACK_PUSH_COORD,
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, push_time * 0.5, 5)
        
        time.sleep(push_time * 0.5)
        
        # 5. 앞으로 뻗기 (착지 전)
        set_all_legs_position_xyz({
            'front_right': BACK_LIFT_COORD,
            'rear_right': BACK_PUSH_COORD,
            'front_left': BACK_PUSH_COORD,
            'rear_left': BACK_LIFT_COORD
        }, lift_time * 0.5, 5)
        
        time.sleep(lift_time * 0.5)
        
        # 6. 착지하기 (중립 자세로 복귀)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        
        time.sleep(land_time)
        
    print("✓ 후진 걷기 완료")

def strafe_left(steps_count=4, step_duration=0.4, turn_angle_offset=0.2):
    """
    왼쪽으로 제자리 회전 (좌표 기반)

    회전 원리:
    - 대각선 다리 쌍을 들어올리고 Y 좌표를 변경하여 회전
    - 오른쪽 다리들은 바깥쪽(+Y)으로, 왼쪽 다리들은 안쪽(-Y)으로
    """
    print(f"동작: 왼쪽 회전 ({steps_count} 스텝, Y 오프셋: {turn_angle_offset}cm)")

    # 타이밍 설정
    lift_time = step_duration * 0.3
    rotate_time = step_duration * 0.4
    land_time = step_duration * 0.3

    for step in range(steps_count):
        # ===== 첫 번째 다리 쌍: 오른쪽 앞 + 왼쪽 뒤 =====
        print(f"  스텝 {step + 1}/{steps_count} - 1단계: 오른쪽 앞/왼쪽 뒤 회전")

        # 1. 다리 들기 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT)                     
        }, lift_time, 3)
        time.sleep(lift_time)  # 서보가 움직일 시간 대기

        # 2. 회전 (Y 좌표 변경) - 두 다리 동시
        # 왼쪽 회전: 오른쪽 다리는 바깥쪽(+Y), 왼쪽 다리는 안쪽(-Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, rotate_time, 4)
        time.sleep(rotate_time)  # 서보가 움직일 시간 대기

        # 3. 착지 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z)
        }, land_time, 3)
        time.sleep(land_time)  # 서보가 움직일 시간 대기

        # ===== 두 번째 다리 쌍: 왼쪽 앞 + 오른쪽 뒤 =====
        print(f"  스텝 {step + 1}/{steps_count} - 2단계: 왼쪽 앞/오른쪽 뒤 회전")

        # 1. 다리 들기 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z)
        }, lift_time, 3)
        time.sleep(lift_time)  # 서보가 움직일 시간 대기

        # 2. 회전 (Y 좌표 변경) - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),            
            'rear_left': (STANDBY_X, STANDBY_Y + turn_angle_offset, STANDBY_Z)
        }, rotate_time, 4)
        time.sleep(rotate_time)  # 서보가 움직일 시간 대기

        # 3. 착지하고 중립 자세로 복귀 - 네 발 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),            
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        time.sleep(land_time)  # 서보가 움직일 시간 대기

    print("✓ 왼쪽 회전 완료")

def strafe_right(steps_count=4, step_duration=0.4, turn_angle_offset=0.2):
    """
    오른쪽으로 제자리 회전 (좌표 기반)

    회전 원리:
    - 대각선 다리 쌍을 들어올리고 Y 좌표를 변경하여 회전
    - 왼쪽 다리들은 바깥쪽(+Y)으로, 오른쪽 다리들은 안쪽(-Y)으로
    """
    print(f"동작: 오른쪽 회전 ({steps_count} 스텝, Y 오프셋: {turn_angle_offset}cm)")

    # 타이밍 설정
    lift_time = step_duration * 0.3
    rotate_time = step_duration * 0.4
    land_time = step_duration * 0.3

    for step in range(steps_count):
        # ===== 첫 번째 다리 쌍: 오른쪽 앞 + 왼쪽 뒤 =====
        print(f"  스텝 {step + 1}/{steps_count} - 1단계: 오른쪽 앞/왼쪽 뒤 회전")

        # 1. 다리 들기 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),            
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, lift_time, 3)
        time.sleep(lift_time)  # 서보가 움직일 시간 대기

        # 2. 회전 (Y 좌표 변경) - 두 다리 동시
        # 오른쪽 회전: 오른쪽 다리는 안쪽(-Y), 왼쪽 다리는 바깥쪽(+Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, rotate_time, 4)
        time.sleep(rotate_time)  # 서보가 움직일 시간 대기

        # 3. 착지 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z)
        }, land_time, 3)
        time.sleep(land_time)  # 서보가 움직일 시간 대기

        # ===== 두 번째 다리 쌍: 왼쪽 앞 + 오른쪽 뒤 =====
        print(f"  스텝 {step + 1}/{steps_count} - 2단계: 왼쪽 앞/오른쪽 뒤 회전")

        # 1. 다리 들기 - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z)
        }, lift_time, 3)
        time.sleep(lift_time)  # 서보가 움직일 시간 대기

        # 2. 회전 (Y 좌표 변경) - 두 다리 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y - turn_angle_offset, STANDBY_Z)
        }, rotate_time, 4)
        time.sleep(rotate_time)  # 서보가 움직일 시간 대기

        # 3. 착지하고 중립 자세로 복귀 - 네 발 동시
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        time.sleep(land_time)  # 서보가 움직일 시간 대기

    print("✓ 오른쪽 회전 완료")

def rotate_body_left(steps_count=4, step_duration=0.4, rotate_offset=0.2):
    """
    몸체 왼쪽 회전 (제자리 회전, 같은 쪽 다리 쌍 사용)

    turn_left()와 다르게 같은 쪽 다리들을 함께 움직여서 몸체를 회전시킵니다.
    - Phase 1: 오른쪽 다리들 (front_right + rear_right) 함께
    - Phase 2: 왼쪽 다리들 (front_left + rear_left) 함께

    회전 원리 (왼쪽 회전 = 반시계 방향):
    - 앞다리들: 왼쪽으로 (-Y)
    - 뒷다리들: 오른쪽으로 (+Y)
    → 몸체가 반시계 방향으로 회전

    좌표계:
        X: 앞(+) / 뒤(-)
        Y: 오른쪽(+) / 왼쪽(-)
        Z: 위(+) / 아래(-)
    """
    print(f"동작: 몸체 왼쪽 회전 ({steps_count} 스텝, Y 오프셋: ±{rotate_offset}cm)")

    # 타이밍 설정
    lift_time = step_duration * 0.25
    rotate_time = step_duration * 0.35
    land_time = step_duration * 0.25
    adjust_time = step_duration * 0.15

    for step in range(steps_count):
        # ===== Phase 1: 오른쪽 다리들 (front_right + rear_right) =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 1: 오른쪽 다리들 회전")

        # 1. 오른쪽 다리들 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, lift_time, 3)
        
        time.sleep(lift_time)

        # 2. 오른쪽 다리들 회전 위치로 이동
        # 왼쪽 회전: front_right는 왼쪽(-Y), rear_right는 오른쪽(+Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)

        # 3. 오른쪽 다리들 착지
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        
        time.sleep(land_time)
        
        # . 오른쪽 다리들 착지
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time, 3)
        
        time.sleep(land_time)

        # ===== Phase 2: 왼쪽 다리들 (front_left + rear_left) =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 2: 왼쪽 다리들 회전")

        # 4. 왼쪽 다리들 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, lift_time, 3)
        
        time.sleep(lift_time)

        # 5. 왼쪽 다리들 회전 위치로 이동 (오른쪽 다리들에 맞춤)
        # 왼쪽 회전: front_left는 왼쪽(-Y), rear_left는 오른쪽(+Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)
        
        # 왼쪽 회전: front_left는 왼쪽(-Y), rear_left는 오른쪽(+Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)

        # 6. 왼쪽 다리들 착지 및 중립 자세로 복귀
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time + adjust_time, 3)
        
        time.sleep(land_time + adjust_time)
        
    print("✓ 몸체 왼쪽 회전 완료")

def rotate_body_right(steps_count=4, step_duration=0.4, rotate_offset=0.2):
    """
    몸체 오른쪽 회전 (제자리 회전, 같은 쪽 다리 쌍 사용)
    turn_right()와 다르게 같은 쪽 다리들을 함께 움직여서 몸체를 회전시킵니다.
    - Phase 1: 오른쪽 다리들 (front_right + rear_right) 함께
    - Phase 2: 왼쪽 다리들 (front_left + rear_left) 함께
    회전 원리 (오른쪽 회전 = 시계 방향):
    - 앞다리들: 오른쪽으로 (+Y)
    - 뒷다리들: 왼쪽으로 (-Y)
    → 몸체가 시계 방향으로 회전
    좌표계:
        X: 앞(+) / 뒤(-)
        Y: 오른쪽(+) / 왼쪽(-)
        Z: 위(+) / 아래(-)
    Args:
        steps_count: 회전 스텝 수
        step_duration: 한 스텝 시간 (초)
        rotate_offset: 회전을 위한 Y 좌표 변화량 (cm)
    예시:
        rotate_body_right(4, 0.4, 3.0)  # 4스텝, 각 0.4초, Y±3cm
    """
    print(f"동작: 몸체 오른쪽 회전 ({steps_count} 스텝, Y 오프셋: ±{rotate_offset}cm)")
    
    # 타이밍 설정
    lift_time = step_duration * 0.25
    rotate_time = step_duration * 0.35
    land_time = step_duration * 0.25
    adjust_time = step_duration * 0.15
    
    for step in range(steps_count):
        # ===== Phase 1: 오른쪽 다리들 (front_right + rear_right) =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 1: 오른쪽 다리들 회전")
        # 1. 오른쪽 다리들 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, lift_time, 3)
        
        time.sleep(lift_time)
        
        # 2. 오른쪽 다리들 회전 위치로 이동
        # 오른쪽 회전: front_right는 오른쪽(+Y), rear_right는 왼쪽(-Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_left': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)
        
        # 3. 오른쪽 다리들 착지
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z)
        }, land_time, 3)
        
        time.sleep(land_time)
        
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, lift_time, 3)
        
        time.sleep(lift_time)
        
        # ===== Phase 2: 왼쪽 다리들 (front_left + rear_left) =====
        print(f"  스텝 {step + 1}/{steps_count} - Phase 2: 왼쪽 다리들 회전")
        
        # 4. 왼쪽 다리들 들기
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, lift_time, 3)
        
        time.sleep(lift_time)
        
        # 5. 왼쪽 다리들 회전 위치로 이동 (오른쪽 다리들에 맞춤)
        # 오른쪽 회전: front_left는 오른쪽(+Y), rear_left는 왼쪽(-Y)
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z + TURN_LIFT_HEIGHT),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z + TURN_LIFT_HEIGHT)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)
        
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y + rotate_offset, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y , STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y - rotate_offset, STANDBY_Z)
        }, rotate_time, 4)
        
        time.sleep(rotate_time)
        
        # 6. 왼쪽 다리들 착지 및 중립 자세로 복귀
        set_all_legs_position_xyz({
            'front_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_right': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'front_left': (STANDBY_X, STANDBY_Y, STANDBY_Z),
            'rear_left': (STANDBY_X, STANDBY_Y, STANDBY_Z)
        }, land_time + adjust_time, 3)
        
        time.sleep(land_time + adjust_time)
        
    print("✓ 몸체 오른쪽 회전 완료")

def body_move_up_down(height_offset, duration=0.5):
    """
    몸체 상하 이동 (좌표 기반)

    서있는 상태에서 높이를 자유롭게 조절합니다.
    엎드리기/서기와 다르게 현재 X, Y 좌표는 유지하고 Z축만 조정합니다.

    좌표계:
        X: 앞(+) / 뒤(-)
        Y: 오른쪽(+) / 왼쪽(-)
        Z: 위(+) / 아래(-)

    Args:
        height_offset: 높이 조정값 (cm)
            양수 = 위로 올림
            음수 = 아래로 내림
        duration: 동작 시간 (초)

    예시:
        body_move_up_down(3.0)   # 3cm 위로
        body_move_up_down(-3.0)  # 3cm 아래로
    """
    print(f"동작: 몸체 상하 이동 (높이 조정: {height_offset:+.1f}cm)")

    # 현재 서있는 높이에서 offset 적용
    target_z = STANDBY_Z + height_offset

    # 동작 단계 수
    steps = max(int(duration * 30), 10)

    # 모든 다리를 같은 높이로 이동
    positions = {
        'front_right': (STANDBY_X, STANDBY_Y, target_z),
        'rear_right': (STANDBY_X, STANDBY_Y, target_z),
        'front_left': (STANDBY_X, STANDBY_Y, target_z),
        'rear_left': (STANDBY_X, STANDBY_Y, target_z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print(f"✓ 몸체 상하 이동 완료 (목표 높이: {target_z:.1f}cm)")

def body_shift_weight(shift_y, shift_x=0.0, duration=0.5):
    """
    몸체 무게중심 이동 (좌표 기반)

    좌우 기울이기와 다르게, 모든 다리를 같은 높이로 유지하면서
    Y 좌표(좌우) 또는 X 좌표(전후)를 이동하여 무게중심을 이동합니다.

    좌표계:
        X: 앞(+) / 뒤(-)
        Y: 오른쪽(+) / 왼쪽(-)
        Z: 위(+) / 아래(-)

    Args:
        shift_y: 좌우 이동량 (cm)
            양수 = 오른쪽으로 이동
            음수 = 왼쪽으로 이동
        shift_x: 전후 이동량 (cm, 선택사항)
            양수 = 앞으로 이동
            음수 = 뒤로 이동
        duration: 동작 시간 (초)

    예시:
        body_shift_weight(2.0)        # 오른쪽으로 2cm
        body_shift_weight(-2.0)       # 왼쪽으로 2cm
        body_shift_weight(2.0, 1.0)   # 오른쪽 2cm + 앞 1cm
    """
    print(f"동작: 무게중심 이동 (X: {shift_x:+.1f}cm, Y: {shift_y:+.1f}cm)")

    # 동작 단계 수
    steps = max(int(duration * 30), 10)

    # 좌우 다리의 Y 좌표를 반대 방향으로 이동
    # 오른쪽 다리: +Y 방향으로 이동
    # 왼쪽 다리: -Y 방향으로 이동
    positions = {
        'front_right': (STANDBY_X + shift_x, STANDBY_Y + shift_y, STANDBY_Z),
        'rear_right': (STANDBY_X + shift_x, STANDBY_Y + shift_y, STANDBY_Z),
        'front_left': (STANDBY_X + shift_x, STANDBY_Y - shift_y, STANDBY_Z),
        'rear_left': (STANDBY_X + shift_x, STANDBY_Y - shift_y, STANDBY_Z)
    }
    set_all_legs_position_xyz(positions, duration, steps)

    print("✓ 무게중심 이동 완료")

# ============================================================================
# 데모 및 테스트 함수
# ============================================================================

def demo_sequence():
    """전체 동작 데모 시퀀스"""
    print("\n" + "="*60)
    print("Spot Micro 로봇 데모 시작")
    print("="*60 + "\n")
    
    try:
        # 1. 엎드린 상태에서 시작
        print("\n[1/6] 초기 자세: 엎드리기")
        lie_down(duration=2.0)
        time.sleep(1)
        
        # 2. 일어서기
        print("\n[2/6] 일어서기")
        stand_up(duration=2.0)
        time.sleep(1)
        
        # 3. 왼쪽으로 기울이기
        print("\n[3/6] 왼쪽 기울이기")
        tilt_left(duration=1.0)
        time.sleep(0.5)
        stand_up(duration=0.5)
        time.sleep(0.5)
        
        # 4. 오른쪽으로 기울이기
        print("\n[4/6] 오른쪽 기울이기")
        tilt_right(duration=1.0)
        time.sleep(0.5)
        stand_up(duration=0.5)
        time.sleep(1)
        
        # 5. 걷기
        print("\n[5/6] 걷기")
        walk_forward(steps_count=4, step_duration=0.4)
        time.sleep(1)
        
        # 6. 다시 엎드리기
        print("\n[6/6] 마무리: 엎드리기")
        lie_down(duration=2.0)
        
        print("\n" + "="*60)
        print("데모 완료!")
        print("="*60 + "\n")
        
    except KeyboardInterrupt:
        print("\n\n데모 중단됨")
        lie_down(duration=1.0)

def interactive_mode():
    """대화형 제어 모드"""
    print("\n" + "="*60)
    print("Spot Micro 인터랙티브 모드")
    print("="*60)
    print("\n기본 명령어:")
    print("  1 또는 lie      : 엎드리기")
    print("  2 또는 stand    : 서기")
    print("  3 또는 tiltl    : 왼쪽 기울이기")
    print("  4 또는 tiltr    : 오른쪽 기울이기")
    print("  5 또는 walk     : 걷기")
    print("  6 또는 turnl    : 왼쪽 회전 (대각선 쌍)")
    print("  7 또는 turnr    : 오른쪽 회전 (대각선 쌍)")
    print("\n몸체 제어:")
    print("  u 또는 up       : 몸체 위로")
    print("  d 또는 down     : 몸체 아래로")
    print("  l 또는 left     : 무게중심 왼쪽")
    print("  r 또는 right    : 무게중심 오른쪽")
    print("\n몸체 회전:")
    print("  rl 또는 rotl    : 몸체 왼쪽 회전 (같은 쪽 쌍)")
    print("  rr 또는 rotr    : 몸체 오른쪽 회전 (같은 쪽 쌍)")
    print("\n기타:")
    print("  8 또는 demo     : 전체 데모")
    print("  9 또는 xyz      : 개별 다리 좌표 제어 (X, Y, Z)")
    print("  q 또는 quit     : 종료")
    print("="*60 + "\n")

    try:
        while True:
            cmd = input("명령어 입력: ").strip().lower()

            if cmd in ['q', 'quit', 'exit']:
                print("종료합니다...")
                lie_down(duration=1.0)
                break
            elif cmd in ['1', 'lie']:
                lie_down()
            elif cmd in ['2', 'stand']:
                stand_up()
            elif cmd in ['3', 'tiltl']:
                tilt_left()
                time.sleep(0.5)
                stand_up(duration=0.5)
            elif cmd in ['4', 'tiltr']:
                tilt_right()
                time.sleep(0.5)
                stand_up(duration=0.5)
            elif cmd in ['5', 'walk']:
                steps = input("걸음 수 (기본값 4): ").strip()
                steps = int(steps) if steps.isdigit() else 4
                walk_forward(steps_count=steps, step_duration=0.4)
            elif cmd in ['6', 'turnl']:
                steps = input("회전 스텝 수 (기본값 4): ").strip()
                steps = int(steps) if steps.isdigit() else 4
                strafe_left(steps_count=steps)
            elif cmd in ['7', 'turnr']:
                steps = input("회전 스텝 수 (기본값 4): ").strip()
                steps = int(steps) if steps.isdigit() else 4
                strafe_right(steps_count=steps)
            elif cmd in ['u', 'up']:
                height = input("높이 조정 (cm, 기본값 +3): ").strip()
                try:
                    height = float(height) if height else -3.0
                    body_move_up_down(height)
                except ValueError:
                    print("✗ 잘못된 숫자 형식입니다.")
            elif cmd in ['d', 'down']:
                height = input("높이 조정 (cm, 기본값 -3): ").strip()
                try:
                    height = float(height) if height else 3.0
                    body_move_up_down(height)
                except ValueError:
                    print("✗ 잘못된 숫자 형식입니다.")
            elif cmd in ['l', 'left']:
                shift = input("이동량 (cm, 기본값 -2): ").strip()
                try:
                    shift = float(shift) if shift else -0.2
                    body_shift_weight(shift)
                    time.sleep(0.5)
                    stand_up(duration=0.5)  # 중립 자세로 복귀
                except ValueError:
                    print("✗ 잘못된 숫자 형식입니다.")
            elif cmd in ['r', 'right']:
                shift = input("이동량 (cm, 기본값 +2): ").strip()
                try:
                    shift = float(shift) if shift else 0.2
                    body_shift_weight(shift)
                    time.sleep(0.5)
                    stand_up(duration=0.5)  # 중립 자세로 복귀
                except ValueError:
                    print("✗ 잘못된 숫자 형식입니다.")
            elif cmd in ['rl', 'rotl']:
                steps = input("회전 스텝 수 (기본값 4): ").strip()
                steps = int(steps) if steps.isdigit() else 4
                rotate_body_left(steps_count=steps)
            elif cmd in ['rr', 'rotr']:
                steps = input("회전 스텝 수 (기본값 4): ").strip()
                steps = int(steps) if steps.isdigit() else 4
                rotate_body_right(steps_count=steps)
            elif cmd in ['8', 'demo']:
                demo_sequence()
            elif cmd in ['9', 'xyz']:
                print("\n다리 선택:")
                print("  1. front_right (오른쪽 앞)")
                print("  2. front_left (왼쪽 앞)")
                print("  3. rear_right (오른쪽 뒤)")
                print("  4. rear_left (왼쪽 뒤)")
                leg_choice = input("다리 번호 (1-4): ").strip()

                leg_map = {
                    '1': 'front_right',
                    '2': 'front_left',
                    '3': 'rear_right',
                    '4': 'rear_left'
                }

                if leg_choice in leg_map:
                    leg_name = leg_map[leg_choice]
                    print(f"\n{leg_name} 다리의 목표 좌표를 입력하세요 (cm 단위)")
                    print("좌표계 (어깨 기준): X=앞(+)/뒤(-), Y=오른쪽(+)/왼쪽(-), Z=위(+)/아래(-)")
                    print(f"도달 범위: 최대 {UPPER_SEG_LENGTH + LOWER_SEG_LENGTH}cm, 최소 {abs(UPPER_SEG_LENGTH - LOWER_SEG_LENGTH)}cm")

                    try:
                        x = float(input("X 좌표 (cm): ").strip())
                        y = float(input("Y 좌표 (cm): ").strip())
                        z = float(input("Z 좌표 (cm): ").strip())

                        print(f"\n→ {leg_name} 다리를 ({x:.2f}, {y:.2f}, {z:.2f})cm로 이동합니다...")
                        success = set_leg_position_xyz(leg_name, x, y, z, duration=0.5, steps=20)

                        if success:
                            print("✓ 이동 완료")
                        else:
                            print("✗ 이동 실패 (도달 불가능한 좌표)")
                    except ValueError:
                        print("✗ 잘못된 숫자 형식입니다.")
                else:
                    print("✗ 잘못된 다리 번호입니다.")
            else:
                print("알 수 없는 명령어입니다.")

            print()

    except KeyboardInterrupt:
        print("\n\n프로그램 종료")
        lie_down(duration=1.0)

# ============================================================================
# 메인 함수
# ============================================================================

def main():
    """메인 함수"""
    global TEST_MODE
    
    # 명령줄 인자 확인
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        TEST_MODE = True
        print("\n🧪 테스트 모드 활성화 (각도만 출력)\n")
    
    if TEST_MODE:
        print("="*60)
        print("테스트 모드: 실제 모터를 제어하지 않고 각도만 출력합니다")
        print("="*60 + "\n")
    
    # PCA9685 초기화
    if not init_pca9685():
        if not TEST_MODE:
            print("초기화 실패. 프로그램을 종료합니다.")
            return
    
    time.sleep(0.5)
    
    # 사용 모드 선택
    print("\n모드 선택:")
    # print("  1. 데모 시퀀스 실행")
    print("  2. 인터랙티브 모드")
    
    try:
        choice = input("\n선택 (1 또는 2): ").strip()
        
        if choice == '1':
            demo_sequence()
        elif choice == '2':
            interactive_mode()
        else:
            print("잘못된 선택입니다. 데모 시퀀스를 실행합니다.")
            demo_sequence()
            
    except KeyboardInterrupt:
        print("\n\n프로그램 중단")
    finally:
        if not TEST_MODE:
            print("\n로봇을 안전한 자세로 전환합니다...")
            lie_down(duration=1.0)

if __name__ == "__main__":
    main()