# pc_robot_safety_system.py
#
# [기능 통합]
# - ... (이전과 동일) ...
# - [ADD] PC 자체 TTS 기능 추가 (pyttsx3)
#

import socket
import threading
import time
import struct
import cv2
import numpy as np
import torch
import queue
from PIL import Image, ImageDraw, ImageFont
from transformers import AutoModelForCausalLM, AutoTokenizer
from ultralytics import YOLO
import collections
import traceback
import os
from typing import Tuple, Dict
import pyttsx3  # [ADD] PC 자체 TTS를 위한 import

# --- 전역 변수 및 잠금 ---
g_latest_frame = None
g_annotated_frame = None
g_robot_motion_command = "FORWARD"
g_robot_speech_command = ""
g_program_running = True

g_frame_lock = threading.Lock()
g_command_lock = threading.Lock()
# ------------------------------------

# _recv_all, video_receive_thread, command_send_thread 함수는 변경 사항 없음
# ... (이전 코드와 동일) ...

def _recv_all(sock, n):
    """소켓에서 n 바이트를 모두 수신하는 헬퍼 함수"""
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet: return None
        data.extend(packet)
    return data

def video_receive_thread(port):
    """'눈' 스레드: RDK X5의 비디오 연결을 수신"""
    global g_latest_frame, g_program_running, g_annotated_frame
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', port)); server_socket.listen(1)
    print(f"📹 비디오 수신 대기 중 (포트 {port})...")

    while g_program_running:
        conn = None
        try:
            conn, addr = server_socket.accept(); print(f"✓ 비디오 연결됨: {addr}")
            
            while g_program_running:
                size_data = _recv_all(conn, 4);
                if size_data is None: break
                msg_size = struct.unpack(">L", size_data)[0]
                frame_data = _recv_all(conn, msg_size)
                if frame_data is None: break
                frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                with g_frame_lock:
                    g_latest_frame = frame
            
            print(f"✗ 비디오 연결 끊김: {addr}")
            if conn: conn.close()
            with g_frame_lock: g_latest_frame = None; g_annotated_frame = None

        except Exception as e:
            if g_program_running: print(f"✗ 비디오 수신 스레드 오류: {e}")
            if conn: conn.close(); time.sleep(1)
            
    server_socket.close(); print("비디오 수신 스레드 종료.")

def command_send_thread(port):
    """'입' 스레드: RDK X5로 '동작|음성' 명령을 0.1초마다 전송"""
    global g_robot_motion_command, g_robot_speech_command, g_program_running

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', port)); server_socket.listen(1)
    print(f"🤖 명령 전송 대기 중 (포트 {port})...")

    while g_program_running:
        conn = None
        try:
            conn, addr = server_socket.accept(); print(f"✓ 명령 채널 연결됨: {addr}")
            while g_program_running:
                with g_command_lock:
                    motion = g_robot_motion_command
                    speech = g_robot_speech_command
                    g_robot_speech_command = "" # 음성 명령은 한 번만 보내고 초기화

                # '동작|음성' 포맷으로 전송
                combined_command = f"{motion}|{speech}\n"
                conn.sendall(combined_command.encode('utf-8'))
                time.sleep(0.1) # 1초에 10번 명령 전송

            print(f"✗ 명령 채널 연결 끊김: {addr}")
            if conn: conn.close()
        except (ConnectionResetError, BrokenPipeError):
            print(f"✗ 명령 채널 클라이언트 연결 끊김.")
            if conn: conn.close()
        except Exception as e:
            if g_program_running: print(f"✗ 명령 전송 스레드 오류: {e}")
            if conn: conn.close(); time.sleep(1)

    server_socket.close(); print("명령 전송 스레드 종료.")

# ===============================================
# [From safety_watch.py] UI 및 폰트 유틸리티
# ... (이전 코드와 동일) ...
# ===============================================
def get_korean_font(size=20):
    font_paths = [
        "C:\\Windows\\Fonts\\malgun.ttf",
        "/System/Library/Fonts/AppleSDGothicNeo.ttc",
        "/Library/Fonts/AppleSDGothicNeo.ttf",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansKR-Regular.otf",
    ]
    for font_path in font_paths:
        try:
            if os.path.exists(font_path):
                return ImageFont.truetype(font_path, size)
        except: continue
    print("⚠️  한글 폰트를 찾을 수 없습니다. 기본 폰트를 사용합니다.")
    return ImageFont.load_default()

def put_korean_text(cv_img, text, pos, font_size=20, color=(255, 255, 255)):
    try:
        pil_img = Image.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_img)
        font = get_korean_font(font_size)
        rgb_color = (color[2], color[1], color[0])
        draw.text(pos, text, font=font, fill=rgb_color)
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
    except Exception as e:
        print(f"한글 텍스트 렌더링 실패: {e}"); return cv_img

def draw_translucent_box(img, box, color=(0, 0, 255), alpha=0.25, thickness=2):
    x1, y1, x2, y2 = map(int, box)
    overlay = img.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

def draw_alert_banner(frame, en_text):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 80), (0, 0, 255), -1) # 빨간 배경
    cv2.putText(frame, en_text, (15, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3, cv2.LINE_AA) # 노란 글씨
    return frame

def draw_safe_banner(frame, ko_text):
    h, w = frame.shape[:2]
    cv2.rectangle(frame, (0, 0), (w, 80), (128, 128, 128), -1) # 회색 배경
    frame = put_korean_text(frame, ko_text, (15, 25), font_size=30, color=(0, 255, 0)) # 초록 글씨
    return frame

def crop_safe(img, box, pad=0):
    h, w = img.shape[:2]
    x1, y1, x2, y2 = box
    x1 = max(0, int(x1 - pad)); y1 = max(0, int(y1 - pad))
    x2 = min(w - 1, int(x2 + pad)); y2 = min(h - 1, int(y2 + pad))
    if x2 <= x1 or y2 <= y1: return None
    return img[y1:y2, x1:x2].copy()

# ===============================================
# [ADD] PC 자체 TTS 워커 클래스 (safety_watch.py에서 복원)
# ===============================================
class TTSWorker:
    """PC의 스피커로 소리를 재생하는 비차단 TTS 워커"""
    def __init__(self, rate=180, voice_idx=None):
        self.q = queue.Queue()
        try:
            self.eng = pyttsx3.init()
            self.eng.setProperty('rate', rate)
            if voice_idx is not None:
                voices = self.eng.getProperty('voices')
                if 0 <= voice_idx < len(voices):
                    self.eng.setProperty('voice', voices[voice_idx].id)
            self.th = threading.Thread(target=self._run, daemon=True)
            self._last = ""
            self._last_t = 0.0
            self.th.start()
            print("✓ PC TTS 엔진 초기화 성공")
        except Exception as e:
            print(f"✗ PC TTS 엔진 초기화 실패: {e}. PC 소리가 재생되지 않습니다.")
            self.eng = None

    def _run(self):
        while True:
            text = self.q.get()
            try:
                self.eng.say(text)
                self.eng.runAndWait()
            except Exception:
                pass

    def say(self, text: str, cooldown=3.0):
        if not self.eng: # TTS 초기화 실패 시
            return 
            
        now = time.time()
        # 쿨다운 로직은 analysis_thread에서 이미 처리하므로 여기서는 제거
        # if text != self._last or (now - self._last_t) > cooldown:
        # self._last, self._last_t = text, now
        self.q.put(text)


# ===============================================
# [From safety_watch.py] VQA 모델 및 워커 클래스
# ... (이전 코드와 동일) ...
# ===============================================
class MoonVQA:
    def __init__(self, model_id: str = "vikhyatk/moondream2", device: str = None, max_side: int = 448):
        default_device = (
            "cuda" if torch.cuda.is_available()
            else "mps" if torch.backends.mps.is_available()
            else "cpu"
        )
        self.device = device or default_device
        self.max_side = max_side
        self.tokenizer = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
        self.model = AutoModelForCausalLM.from_pretrained(
            model_id,
            dtype=(torch.float16 if self.device == "cuda" else torch.float32),
            trust_remote_code=True
        ).to(self.device).eval()

    @torch.inference_mode()
    def ask(self, image_bgr: np.ndarray, question: str, max_new_tokens: int = 16) -> str:
        if image_bgr is None or image_bgr.size == 0: return ""
        try:
            img_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            h, w = img_rgb.shape[:2]
            scale = self.max_side / max(h, w)
            if scale < 1.0:
                img_rgb = cv2.resize(img_rgb, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
            
            pil_img = Image.fromarray(img_rgb)
            enc = self.model.encode_image(pil_img)
            
            try: out = self.model.answer_question(enc, question, self.tokenizer, max_new_tokens=max_new_tokens)
            except (AttributeError, TypeError): out = self.model.query(pil_img, question); out = out.get("answer", "") if isinstance(out, dict) else out
            
            return out.strip().lower()
        except Exception as e:
            print(f"[VQA ask 오류] {type(e).__name__}: {e}"); return ""

    def is_yes(self, image_bgr: np.ndarray, question: str) -> bool:
        ans = self.ask(image_bgr, question)
        if "lying" in question.lower():
            positive_keywords = ["yes", "lying", "lying on", "laying", "fallen", "on the floor"]
        elif "helmet" in question.lower():
            positive_keywords = ["yes", "wearing", "wear", "착용"]
        else:
            positive_keywords = ["yes", "true"]

        negative_keywords = ["no", "not", "none", "isn't", "aren't", "아니", "없"]
        if any(k in ans for k in negative_keywords): return False
        
        result = any(k in ans for k in positive_keywords)
        print(f"[VQA is_yes] Q: {question[:40]}... | A: '{ans}' → {result}")
        return result

class VQAWorker:
    def __init__(self, vqa: MoonVQA, max_cache_size: int = 50):
        self.vqa = vqa
        self.q = queue.Queue(maxsize=8)
        self.out: collections.OrderedDict[int, Tuple[bool, bool]] = collections.OrderedDict()
        self.max_cache_size = max(10, max_cache_size)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            key, person_crop, head_crop = self.q.get()
            fallen_h, fallen_vqa, wearing = False, False, True
            try:
                if person_crop is not None and person_crop.size:
                    h, w = person_crop.shape[:2]; fallen_h = (w / (h + 1e-6)) > 1.8
                    if fallen_h: print(f"[VQA 비율 감지] Key: {key} (쓰러짐 의심)")
                    fallen_vqa = self.vqa.is_yes(person_crop, "Is the person lying on the floor? Answer yes or no.")

                if head_crop is not None and head_crop.size:
                    wearing = self.vqa.is_yes(head_crop, "Is the person wearing an industrial safety helmet? Answer yes or no.")
                
                is_fallen = fallen_h or fallen_vqa
                no_helmet = not wearing
                self.out[key] = (is_fallen, no_helmet)
                print(f"[VQA 최종] Key: {key} | 쓰러짐: {is_fallen}, 헬멧미착용: {no_helmet}")

                while len(self.out) > self.max_cache_size: self.out.popitem(last=False) 
            except Exception as e:
                print(f"[VQA 오류] Key: {key} | {e}"); self.out[key] = (fallen_h, not wearing)
                while len(self.out) > self.max_cache_size: self.out.popitem(last=False)

    def submit(self, key: int, person_crop: np.ndarray, head_crop: np.ndarray):
        if not self.q.full(): self.q.put((key, person_crop, head_crop))

    def fetch(self, key: int):
        if key in self.out: self.out.move_to_end(key); return self.out[key]
        return None

# ===============================================
# [병합] '뇌' 스레드 (YOLO + VQA + PC TTS 통합)
# ===============================================
def analysis_thread():
    """'뇌' 스레드: 화재(YOLO), 장애물/사람(YOLO), VQA(Moondream) 동시 처리 및 제어"""
    global g_latest_frame, g_annotated_frame, g_robot_motion_command, g_robot_speech_command, g_program_running

    print("🧠 통합 분석 스레드 시작. AI 모델 로드 중...")
    device = (
        "cuda" if torch.cuda.is_available()
        else "mps" if torch.backends.mps.is_available()
        else "cpu"
    )
    try:
        model_fire = YOLO('./weights/firedetect-11s.pt').to(device) 
        model_po = YOLO('yolov8n.pt').to(device)
        vqa = MoonVQA(device=device)
        worker = VQAWorker(vqa)
        
        # [ADD] PC TTS 객체 생성
        pc_tts = TTSWorker(rate=180) 
        
        print(f"✓ AI 모델 로드 완료 (장치: {device})")
    except Exception as e:
        print(f"✗ AI 모델 로드 실패: {e}"); g_program_running = False; return

    person_states: Dict[int, Tuple[bool, bool]] = {}
    frame_idx = 0
    vqa_every_n = 8
    vqa_topk = 3
    
    last_fire_alert_time = 0
    last_fallen_alert_time = 0
    last_helmet_alert_time = 0

    while g_program_running:
        frame_copy = None
        with g_frame_lock:
            if g_latest_frame is None: time.sleep(0.05); continue
            frame_copy = g_latest_frame.copy()

        frame_idx += 1
        out_frame = frame_copy.copy()

        # --- 1. AI 추론 (YOLO) ---
        results_fire = model_fire(frame_copy, verbose=False, conf=0.5)
        results_po = model_po(frame_copy, classes=[0, 56, 57, 60], verbose=False, conf=0.5)

        # --- 2. 감지 결과 추출 ---
        is_fire_detected = len(results_fire[0].boxes) > 0
        is_person_detected = False
        is_obstacle_detected = False
        person_boxes = []

        for b in results_po[0].boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            cls = int(b.cls[0])
            if cls == 0:
                is_person_detected = True
                area = max(1, (x2 - x1) * (y2 - y1))
                person_boxes.append(((x1, y1, x2, y2), area))
                draw_translucent_box(out_frame, (x1, y1, x2, y2), (0, 0, 255), alpha=0.1)
            elif cls in [56, 57, 60]:
                is_obstacle_detected = True
                draw_translucent_box(out_frame, (x1, y1, x2, y2), (255, 0, 0), alpha=0.1)
        
        if is_fire_detected:
            for b in results_fire[0].boxes:
                x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
                draw_translucent_box(out_frame, (x1, y1, x2, y2), (0, 165, 255), alpha=0.4, thickness=3)

        # --- 3. VQA 분석 (Script 1 로직) ---
        vqa_alerts = []
        detected_indices = set(range(min(len(person_boxes), vqa_topk)))
        current_indices = set(person_states.keys())
        indices_to_remove = current_indices - detected_indices
        for i in indices_to_remove: del person_states[i]

        person_boxes.sort(key=lambda t: t[1], reverse=True)
        
        for i, ((x1, y1, x2, y2), _) in enumerate(person_boxes[:vqa_topk]):
            key = i
            if frame_idx % vqa_every_n == 0:
                person_crop = crop_safe(frame_copy, (x1, y1, x2, y2))
                head_y2 = y1 + max(20, int(0.28 * (y2 - y1)))
                head_crop = crop_safe(frame_copy, (x1, y1, x2, head_y2), pad=4)
                # print(f"[Frame {frame_idx}] VQA 분석 요청: Person {key}") # 로그가 너무 많아 주석 처리
                worker.submit(key, person_crop, head_crop)

            res = worker.fetch(key)
            if res is not None: person_states[i] = res
            
            fallen, no_helmet = person_states.get(i, (False, False))
            
            if fallen: vqa_alerts.append("FALLEN") 
            if no_helmet: vqa_alerts.append("NO HELMET") 

        # --- 4. 우선순위 제어 시스템 ---
        alert_level = "SAFE"
        scene_desc = "순찰 중. 이상 없음."
        motion_cmd = "FORWARD"
        speech_cmd = "" # 로봇에게 보낼 명령
        en_banner_text = ""
        
        now = time.time()

        if is_fire_detected:
            alert_level = "FIRE"
            scene_desc = "화재 감지! 즉시 정지!"
            motion_cmd = "STOP"
            en_banner_text = "EMERGENCY: FIRE DETECTED!"
            if now - last_fire_alert_time > 5:
                speech_cmd = f"ALERT|{scene_desc}" # 로봇 전송용
                pc_tts.say(scene_desc)             # [MOD] PC 재생
                last_fire_alert_time = now

        elif "FALLEN" in vqa_alerts:
            alert_level = "FALLEN"
            scene_desc = "긴급 상황! 쓰러진 사람이 발견되었습니다!"
            motion_cmd = "STOP"
            en_banner_text = "EMERGENCY: PERSON FALLEN!"
            if now - last_fallen_alert_time > 5:
                speech_cmd = f"ALERT|{scene_desc}" # 로봇 전송용
                pc_tts.say(scene_desc)             # [MOD] PC 재생
                last_fallen_alert_time = now
            
        elif "NO HELMET" in vqa_alerts:
            alert_level = "NO_HELMET"
            scene_desc = "경고! 안전모 미착용이 감지되었습니다!"
            motion_cmd = "STOP"
            en_banner_text = "WARNING: NO HELMET!"
            if now - last_helmet_alert_time > 5:
                speech_cmd = f"WARN|{scene_desc}"  # 로봇 전송용
                pc_tts.say(scene_desc)             # [MOD] PC 재생
                last_helmet_alert_time = now

        elif is_person_detected:
            alert_level = "SAFE"
            scene_desc = "사람 확인 (안전). 로봇 정지합니다."
            motion_cmd = "STOP"
        
        elif is_obstacle_detected:
            alert_level = "SAFE"
            scene_desc = "장애물 감지. 회피 기동합니다."
            motion_cmd = "TURN_LEFT"
            
        if alert_level == "SAFE":
            last_fire_alert_time = 0
            last_fallen_alert_time = 0
            last_helmet_alert_time = 0

        # --- 5. UI 배너 적용 및 전역 변수 설정 ---
        with g_command_lock:
            g_robot_motion_command = motion_cmd
            if speech_cmd:
                g_robot_speech_command = speech_cmd
        
        if alert_level == "SAFE":
            out_frame = draw_safe_banner(out_frame, scene_desc)
        else:
            # 위험 배너의 한글 설명을 영문으로 교체
            out_frame = draw_alert_banner(out_frame, en_banner_text)
            
        with g_frame_lock:
            g_annotated_frame = out_frame

        time.sleep(0.01)

    print("통합 분석 스레드 종료.");

# --- 메인 스레드 (GUI 표시) ---
if __name__ == "__main__":
    print("="*50); print("💻 PC 로봇 안전 감시 시스템 시작..."); print("="*50)

    threads = [
        threading.Thread(target=video_receive_thread, args=(9999,), daemon=True),
        threading.Thread(target=command_send_thread, args=(9998,), daemon=True),
        threading.Thread(target=analysis_thread, daemon=True),
    ]
    for t in threads: t.start()
        
    print("\n🚀 모든 스레드 시작됨. 'q' 키를 눌러 종료.")
    
    try:
        while g_program_running:
            frame_to_show = None

            with g_frame_lock:
                if g_annotated_frame is not None:
                    frame_to_show = g_annotated_frame.copy() 
                elif g_latest_frame is not None:
                    frame_to_show = g_latest_frame.copy()

            if frame_to_show is not None:
                cv2.imshow("RDK X5 Robot Safety System", frame_to_show)
            else:
                loading_screen = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(loading_screen, "Waiting for RDK X5 connection...",
                            (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.imshow("RDK X5 Robot Safety System", loading_screen)

            if cv2.waitKey(33) & 0xFF == ord('q'):
                print("[PC] 'q' 키 입력. 시스템 종료."); break
                
    except KeyboardInterrupt:
        print("\n[PC] Ctrl+C 감지. 시스템 종료...")
    finally:
        g_program_running = False
        print("모든 스레드 종료 중...");
        for t in threads:
            if t.is_alive(): t.join(timeout=1)
        cv2.destroyAllWindows()
        print("="*50); print("💻 PC 제어 서버 종료."); print("="*50)