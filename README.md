# 🦾 오억이 프로젝트 (GiGE Controller Project)

> **폴리텍 졸업작품 | MVS 기반 디지털 카메라 제어 + 로봇 융합 시스템**

---

## 🎯 프로젝트 개요

**오억이(Oh-Eok-i)** 는  
🎥 **산업용 카메라 제어 시스템**과 🐕 **보행 로봇**을 결합한 프로젝트입니다.  

1️⃣ 1단계: **MV-CU050-30GC 카메라 제어 툴 개발 (Visual Studio / C++)**  
2️⃣ 2단계: **로봇 오억이와 통합 (라즈베리파이 + ROS + 센서 기반 행동 제어)**  

---

## 🧠 주요 기능

### 📸 디지털 카메라 제어 툴
- **ON/OFF 제어**  
- **밝기 및 명암 조정 (Exposure / Gain / Gamma)**  
- **Digital Zoom (줌 인/아웃)**  
- **Filtering**
  - HSV 색상 필터링 🎨  
  - Blurring / Gaussian 필터 💧  
- **해상도 설정**: Full HD (1920×1080 @30FPS)  
- **사진 캡처 & 녹화 기능** 📷🎞️  
- **OCR 문자 인식 기능** 🔤  
- **Object Detection (YOLO 기반)** 🤖  

---

### 🐾 로봇 오억이 (2단계 통합 목표)
- **보행 제어**: 앉기 / 걷기 / 계단 오르기 / 내리기  
- **상황 인식 AI** (카메라 기반)  
- **안내 및 상담 모드** 🗣️  
- **알림 및 비상 호출 기능** 🔔  

---

## ⚙️ 개발 환경

| 항목 | 내용 |
|------|------|
| 🧩 IDE | Visual Studio (C++) |
| 📷 카메라 | Hikrobot MV-CU050-30GC |
| 🧠 프레임워크 | MVS SDK / OpenCV / Tesseract OCR / YOLOv8 |
| 💻 OS | Windows 10 / Raspberry Pi OS |
| 🔌 연결 | GigE / PoE 기반 |

---

## 📁 디렉토리 구조
```
roller_project/
├─ src/ # C++ 소스 코드
├─ include/ # 헤더 파일
├─ assets/ # 아이콘, UI, 이미지
├─ data/ # 학습/테스트용 데이터
├─ build/ # 빌드 결과물
└─ docs/ # 설계서, 시연 영상, 보고서
```

---

## 🧩 향후 계획

| 단계 | 내용 | 상태 |
|------|------|------|
| 1️⃣ | 카메라 제어 툴 완성 | ✅ |
| 2️⃣ | OpenCV 필터링 + OCR 기능 추가 | 🔄 진행중 |
| 3️⃣ | YOLOv8 Object Detection 통합 | 🔜 예정 |
| 4️⃣ | 오억이 로봇 제어 모듈 연동 (앉기/걷기 등) | 🧩 계획중 |
| 5️⃣ | 라즈베리파이 기반 독립 구동 | 🛰️ 연구중 |

---

## 👥 팀 소개

| 이름 | 역할 |
|------|------|
| 조진경 | 프로젝트 리더 / 비전 알고리즘 개발 / UI 및 영상 처리 모듈 |
| 신현택 | 로봇 제어 / 하드웨어 설계 |

---

## 🚀 미리보기


<div align="center">
  <img width="154" height="174" alt="image" src="https://github.com/user-attachments/assets/c370b12f-9543-411a-82b9-3d71ab62c5f8" />
<br>
  <em>“오억이 – 시각과 움직임이 결합된 지능형 안내견 로봇”</em>
</div>

---

## 🧾 License

MIT License © 2025 

---

> 🦾 "오억이, 세상을 인식하고 움직이다."  
> – 폴리텍 융합기술교육원 졸업작품 2025


