# Single-Point LiDAR 3D Scanner

> Arduino + TFmini LiDAR 센서를 이용한 360° 3D 포인트 클라우드 스캐너  
> 하드웨어 레지스터 직접 제어 · 칼만 필터 · PID 제어 · 포인트 클라우드 후처리 · Poisson Surface Reconstruction · Three.js 웹 뷰어

DC 모터(Yaw 회전) + 28BYJ-48 스텝 모터(Pitch 틸트) + TFmini 거리 센서를 결합하여 물체의 3D 형상을 스캔하고,  
Z-Buffer 고스트 제거 → 적응형 SOR 필터 → 빈 공간 보간 → 재질 분류의 후처리 파이프라인을 거쳐 **Poisson Surface Mesh**로 복원한 뒤,  
웹 브라우저에서 실시간 3D 뷰잉이 가능한 **풀스택 프로젝트**

---
<img width="288" height="384" alt="Image" src="https://github.com/user-attachments/assets/35e6492e-49d0-4f64-9779-98235c4ac61e" />
<img width="653" height="423" alt="Image" src="https://github.com/user-attachments/assets/cf4039d2-19c0-4319-a054-f1789e0b5afa" />
<img width="651" height="438" alt="Image" src="https://github.com/user-attachments/assets/d6da8ea2-b7ce-4470-97e8-d175f30a3381" />
<img width="531" height="375" alt="Image" src="https://github.com/user-attachments/assets/22821e1f-b706-4a01-a6a3-8a7bf55bef1a" />
<img width="531" height="460" alt="Image" src="https://github.com/user-attachments/assets/0e111a58-817d-417e-acf6-b3e66730909b" />
<img width="431" height="405" alt="Image" src="https://github.com/user-attachments/assets/f85465eb-0a4c-4e7e-a9dc-21be06c1aba8" />
<img width="671" height="438" alt="Image" src="https://github.com/user-attachments/assets/82f5d62b-2a32-489e-b3bf-f0eb471542fd" />
<img width="950" height="691" alt="Image" src="https://github.com/user-attachments/assets/508d9cca-61f7-40f3-ae38-966e8280ded6" />


## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                      Hardware Layer                          │
│  Arduino Uno (ATmega328P, 16MHz)                            │
│  ├─ DC Motor + L298N Driver + Rotary Encoder (Yaw, 360°)   │
│  ├─ 28BYJ-48 Stepper + ULN2003 Driver (Pitch, 0~45°)      │
│  └─ TFmini LiDAR Sensor (SoftwareSerial, 115200bps)        │
├─────────────────────────────────────────────────────────────┤
│                      Software Layer                          │
│  arduino_3d.py ──→ postprocess.py ──→ export_mesh.py        │
│  (실시간 수집)      (필터링·분석)       (OBJ 메쉬 생성)       │
│  ├ Kalman Filter    ├ Z-Buffer Filter   ├ Normal Estimation  │
│  ├ Multi-threading  ├ Adaptive SOR      └ Poisson Recon     │
│  └ 3D Rendering     ├ Hole Filling                          │
│                      ├ Material Segm.   viewer.html          │
│                      └ 6-Panel Dash     (Three.js 웹 뷰어)   │
└─────────────────────────────────────────────────────────────┘
```

## 프로젝트 구조

```
project/
├── arduino_lidar_scanner_register/
│   └── arduino_lidar_scanner_register.ino   # Arduino 펌웨어 (레지스터 직접 제어)
├── arduino_3d.py           # 실시간 스캔 수집 + 칼만 필터 + 3D 시각화
├── postprocess.py          # 포인트 클라우드 후처리 파이프라인 (6패널 대시보드)
├── export_mesh.py          # Poisson Surface → OBJ 메쉬 변환
├── viewer.html             # Three.js 기반 웹 3D 뷰어
└── docs/                   # 기술 문서 (코드별 상세 해설)
    ├── EXPLANATION_ARDUINO.md       # 아두이노 펌웨어 완전 분석
    ├── EXPLANATION_PYTHON_3D.md     # 실시간 스캔 코드 완전 분석
    ├── EXPLANATION_POSTPROCESS.md   # 후처리 파이프라인 완전 분석
    └── PROJECT_SKILLS.md            # 프로젝트 핵심 기술 요약
```

---

## 1. 임베디드 하드웨어 제어 (Arduino / C++)

아두이노의 `Serial`, `analogWrite` 등의 기본 라이브러리를 사용하지 않고, ATmega328P의 **하드웨어 레지스터를 직접 조작**하여 16MHz 칩에서 로봇 공학 수준의 리얼타임 제어를 구현했습니다.

### 1-1. UART 레지스터 직접 제어

무거운 `Serial` 클래스를 완전히 대체하여 115200bps 고속 통신을 지연 없이 처리합니다.

```cpp
// 배속 모드 활성화 + Baud Rate Divisor 설정
void uart_init_115200() {
    UCSR0A = (1 << U2X0);          // U2X0: 배속 모드 ON
    UBRR0L = 16;                   // 16MHz / 8 / (16+1) ≈ 115200bps
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // RX, TX 활성화
}

// 1바이트 전송: 송신 버퍼가 빌 때까지 대기 후 직접 기록
void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0)));  // UDRE0 = 버퍼 비었음
    UDR0 = c;                           // Data Register에 직접 기록
}
```

- **`uart_puts_P()`**: 아두이노의 제한된 RAM(2KB)을 아끼기 위해 문자열을 Flash 메모리(PROGMEM)에 저장하고 `pgm_read_byte`로 한 글자씩 꺼내 전송하는 기법을 사용합니다.

### 1-2. 외부 인터럽트 기반 로터리 엔코더 (INT1)

엔코더 핀 상태를 하드웨어 인터럽트로 실시간 추적하여 660 카운트/회전의 정밀한 Yaw 각도를 계산합니다.

```cpp
// 인터럽트 설정: CHANGE 모드로 핀 상태 변화 감지
void setup_encoder_interrupt() {
    EICRA = (EICRA & ~((1 << ISC11) | (1 << ISC10))) | (1 << ISC10);
    EIMSK = (EIMSK & ~(1 << INT0)) | (1 << INT1);
}

ISR(INT1_vect) {
    uint8_t pd = PIND;                // PIND 레지스터 통째 읽기 (극도로 빠름)
    bool a = (pd >> ENC_A_BIT) & 1;
    bool b = (pd >> ENC_B_BIT) & 1;
    bool dir = (a == b);              // A==B → 정방향, A≠B → 역방향
    encoderCount += dir ? 1 : -1;

    // 목표 바퀴 수 도달 시 즉시 Flag 발동
    if (!targetRevsReached && encoderThreshold > 0) {
        long absNow = encoderCount < 0 ? -encoderCount : encoderCount;
        if (absNow >= encoderThreshold) {
            targetRevsReached = true;
            encoderCountAtTrigger = encoderCount;  // 정확한 트리거 시점 백업
        }
    }
}
```

- `volatile` 키워드를 사용하여 CPU 최적화로 인한 변수 무시를 방지합니다.
- `cli()` / `SREG` 복원 패턴을 사용하여 인터럽트-안전(Interrupt-Safe)하게 변수를 읽습니다.

### 1-3. 타이머 인터럽트 기반 스텝 모터 비동기 구동 (Timer1 CTC)

메인 `loop()`와 완전히 독립적으로 동작하는 스텝 모터 드라이버를 타이머 인터럽트로 구현했습니다.

```cpp
// Timer1을 CTC 모드, 1024 프리스케일러로 설정 → 초당 ~30회 인터럽트
void setup_stepper_timer() {
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);  // CTC + /1024
    OCR1A = 520;              // Compare Match 값
    TIMSK1 |= (1 << OCIE1A); // 인터럽트 활성화
}

ISR(TIMER1_COMPA_vect) {
    if (stepPosition < stepTarget)      stepIndex = (stepIndex + 1) & 0x07;
    else if (stepPosition > stepTarget) stepIndex = (stepIndex - 1) & 0x07;
    else return;  // 목표 도달 시 정지
    PORTB = (PORTB & ~STP_MASK) | HALF_STEP_SEQ[stepIndex];  // 4핀 동시 제어
}
```

- **하프 스텝 시퀀스**: 8단계 비트 배열(`HALF_STEP_SEQ[8]`)로 28BYJ-48을 풀스텝 대비 2배 부드럽게 구동합니다.
- 스텝 모터 4개 핀이 모두 PORTB에 속해 있어, `STP_MASK` 비트 마스크로 한 번의 레지스터 쓰기로 4핀을 동시 제어합니다.

### 1-4. DC 모터 하드웨어 PWM

`analogWrite()`를 완전히 대체하여 타이머0의 레지스터를 직접 조작합니다.

```cpp
void setDcMotorPwm_raw(uint8_t duty) {
    if (duty == 0) {
        TCCR0A &= ~(1 << COM0A1);      // OC0A 핀 연결 해제
        PORTD &= ~(1 << MTR_PWM_BIT);  // 완전 LOW
    } else {
        TCCR0A |= (1 << COM0A1);       // OC0A 핀 연결
        OCR0A = duty;                   // PWM 듀티 (0~255) 직접 설정
    }
}
```

### 1-5. PID 속도 제어 + Feedforward + EMA 필터

DC 모터의 회전 속도를 일정하게 유지하기 위한 고급 제어 기법이 적용되어 있습니다.

```
출력 PWM = Feedforward + Kp×오차 + Ki×적분 + Kd×(EMA 필터 미분)
```

| 기법 | 설명 |
|------|------|
| **Feedforward** | 목표 속도(200 cnt/s)에 대한 기본 PWM을 미리 더해 PID가 0부터 고생하지 않게 함 |
| **Anti-Windup** | 적분값이 `±INTEGRAL_MAX`를 넘지 않도록 클램핑하여 오버슈트 방지 |
| **Derivative Kick 방지** | 오차의 미분이 아닌 **측정 속도의 미분**을 사용하여 목표값 변경 시 미분 스파이크 제거 |
| **EMA 필터** | 미분값에 지수이동평균(`α=0.3`)을 적용하여 엔코더 노이즈로 인한 미분 진동 억제 |

### 1-6. 자동 스캔 상태 머신

`ScannerState` 열거형으로 스캐너의 전체 스캔 프로세스를 FSM(Finite State Machine)으로 관리합니다.

```
IDLE → MOVING_PITCH → SPINNING_YAW → MOVING_PITCH → ... → FINISHED
         (스텝 모터     (DC 모터 1회전    (다음 층으로         (최대 높이
          목표 각도로     + LiDAR 데이터    pitch 올림)          도달)
          틸트)          수집)
```

- **오버슛 보상**: 모터 정지 시 관성에 의한 오버슈트를 `encoderCountAtTrigger`로 정확히 계산하여, 다음 층 시작 시 보정합니다.
- **아크탄젠트 각도 계산**: `atan2(높이, 48.0cm)`으로 0.5cm 간격의 일정한 수직 해상도를 보장합니다.

---

## 2. 실시간 스캔 수집 소프트웨어 (Python)

### 2-1. 칼만 필터 (Kalman Filter) — 엔코더 노이즈 제거

아두이노 모터 기어의 백래시(헐거움)로 인해 떨리는 Yaw 각도를 소프트웨어적으로 완벽하게 평활화합니다.

```python
class YawKalmanFilter:
    def __init__(self):
        self.x = np.array([[yaw], [0.0]])  # 상태 벡터: [각도, 각속도]
        self.P = np.eye(2)                  # 공분산 행렬
        self.Q = np.array([[0.01, 0], [0, 0.5]])   # 시스템 노이즈
        self.R = np.array([[5.0]])                   # 측정 노이즈
```

| 단계 | 수식 | 설명 |
|------|------|------|
| **예측 (Predict)** | `x̂ = F·x` | 등속 모델로 현재 시점의 각도 예측 (`F = [[1, dt], [0, 1]]`) |
| **공분산 전파** | `P̂ = F·P·Fᵀ + Q` | 예측의 불확실성 전파 |
| **칼만 이득** | `K = P̂·Hᵀ·(H·P̂·Hᵀ + R)⁻¹` | 예측값과 측정값의 최적 가중 비율 계산 |
| **업데이트** | `x = x̂ + K·(z - H·x̂)` | 측정값으로 상태 보정 |
| **Wrap-around** | `diff > 180° → diff -= 360°` | 359°→0° 전환 시 착각 방지 |

### 2-2. 멀티스레드 시리얼 수신기 (Thread-Safe)

메인 스레드가 3D 렌더링에 바빠도 데이터를 한 바이트도 놓치지 않는 병렬 수신 구조입니다.

```
┌─ Main Thread ────────────────┐    ┌─ Reader Thread (Daemon) ────────┐
│  matplotlib 3D 렌더링         │    │  Serial.readline() 무한 루프     │
│  client.snapshot() 으로       │◀───│  SYNC 파싱 → 칼만 필터 적용     │
│  Thread-Safe하게 데이터 복사  │    │  with self._lock: 으로 안전하게  │
│                               │    │  self.points 리스트에 추가       │
├─ Input Thread (Daemon) ──────┤    └─────────────────────────────────┘
│  stdin 감시 → 명령어 전송     │
│  q 입력 시 stop_event.set()  │
└──────────────────────────────┘
```

- **`threading.Lock`**: 점 리스트 접근 시 뮤텍스로 Race Condition 방지
- **`queue.Queue`**: 상태 메시지를 스레드 간 안전하게 전달하는 FIFO 큐
- **Daemon Thread**: 메인 스레드 종료 시 자동으로 함께 종료

### 2-3. 구면좌표 → 데카르트 좌표 변환 (센서 오프셋 보정 포함)

LiDAR 센서가 회전축 정중앙이 아닌 48cm 앞에 위치하는 물리적 구조를 수학적으로 보정합니다.

```
                    센서 ──────── 물체 표면
                    │←── dist ──→│
    ────────────────┼─────────────────────
    회전축 (중심)    │← 48cm →│
                    │         │← actual_radius →│
```

```python
r_horizontal = distance * cos(pitch)        # 빗변의 수평 성분
actual_radius = 48.0 - r_horizontal         # 중심축에서 물체까지의 실제 거리
x = actual_radius * sin(yaw)
y = actual_radius * cos(yaw)
z = distance * sin(pitch)                   # 수직 성분
```

이 오프셋 보정이 없으면 스캔된 물체가 불룩하게 찌그러집니다.

### 2-4. 실시간 Matplotlib 3D 렌더링 루프

```python
plt.ion()  # 인터랙티브 모드: 마우스로 3D 카메라 시점 조작 가능

while not stop_event.is_set():
    points = client.snapshot()       # Thread-Safe 복사
    if len(points) != last_count:    # 점 개수 변경 시에만 렌더링 (CPU 절약)
        xyz = polar_to_xyz(points)
        ax.cla()
        ax.scatter(xyz[:,0], xyz[:,1], xyz[:,2], c=color_values, cmap='viridis', s=8)
    plt.pause(0.05)                  # GPU에게 프레임 렌더링 시간 부여
```

- `finally` 블록에서 프로그램 종료 시 모든 점을 CSV로 자동 저장합니다.

---

## 3. 포인트 클라우드 후처리 파이프라인

원시 스캔 데이터에서 노이즈를 제거하고, 빈 공간을 채우고, 재질을 분류하는 **다단계 필터링 파이프라인**입니다.

```
scan_points.csv
      │
      ▼
┌─────────────────────┐
│ 1. Z-Buffer Filter  │  유령 포인트(Ghosting) 제거
└──────────┬──────────┘
           ▼
┌─────────────────────┐
│ 2. Adaptive SOR     │  통계적 이상치 3단 제거 (Voxel → SOR → Radius)
└──────────┬──────────┘
           ▼
┌─────────────────────┐
│ 3. Hole Filling     │  빈 공간 보간 (Delaunay + Wrap-around)
└──────────┬──────────┘
           ▼
┌─────────────────────┐
│ 4. Material Segm.   │  동적 히스토그램 Peak Detection 재질 분류
└──────────┬──────────┘
           ▼
  6-Panel Dashboard + scan_points_filtered.csv
```

### 3-1. Z-Buffer 유령 포인트 제거

단일 포인트 LiDAR 회전 스캔 시 필연적으로 발생하는 "허공에 뜬 유령점"을 제거합니다.

```python
# 3D 구면 좌표를 (yaw, pitch) 격자로 이산화
yaw_idx = np.round(yaw / 1.0)
pitch_idx = np.round(pitch / 0.5)
cell_key = yaw_idx * 1,000,000 + pitch_idx   # 고유 격자 키 생성

# 동일 격자 내에서 센서와 가장 가까운 점(= 진짜 표면) 1개만 보존
for start, end in bins:
    local_min = np.argmin(dist[start:end])
    inlier_mask[order[start + local_min]] = True
```

**원리**: 같은 방향(yaw, pitch)에 여러 점이 찍혔다면, 센서에 가장 가까운 점이 물체의 진짜 표면이고 뒤에 찍힌 것들은 오류입니다.

### 3-2. 적응형 통계적 이상치 제거 (Adaptive SOR)

스캔 밀도에 따라 필터 파라미터를 **자동으로 조절**하는 3단 콤보 노이즈 제거 시스템입니다.

#### 자동 파라미터 계산
```python
estimated_res_cm = delta_pitch_deg × (π/180) × 48.0cm   # 실제 수직 해상도 추정
ratio = estimated_res_cm / 0.5cm                          # 기준 대비 비율
adaptive_k = max(4, round(8 / ratio))                     # 촘촘 → k↑, 듬성 → k↓
adaptive_voxel = clip(0.5 * ratio, 0.2, 5.0)              # 복셀 크기 비례 조절
```

#### 3단 필터링 파이프라인 (Open3D)

| 단계 | 알고리즘 | 설명 |
|------|----------|------|
| **1. Voxel Downsampling** | `voxel_down_sample(size)` | 공간을 정육면체 큐브로 쪼개 각 큐브 내의 점들을 좌표 평균으로 1개로 합침 → 밀도 균일화 |
| **2. Statistical Outlier Removal** | `remove_statistical_outlier(k, std)` | 각 점의 K-Nearest Neighbors 거리 분포를 정규분포로 피팅, 평균+std_ratio×σ 이상 떨어진 점 제거 |
| **3. Radius Outlier Removal** | `remove_radius_outlier(4, 3.0)` | 반경 3.0cm 내에 이웃 점이 4개 미만인 고립점 제거 |
| **4. 원본 매핑** | `scipy.spatial.cKDTree` | 복셀화로 이동된 좌표를 KD-Tree로 원본 CSV 좌표에 매칭하여 순정 좌표 복원 |

### 3-3. 빈 공간 무조건 채우기 (Spherical Hole Fill)

물리적으로 스캔이 누락된 "빵꾸"를 Convex Hull 내부에서 무조건 메워넣는 보간 알고리즘입니다.

```python
# 360° Wrap-around 처리: 0°와 360°의 데이터를 좌우로 복제
pts_left[:, 0] -= 360 / yaw_bin    # -360° 복제
pts_right[:, 0] += 360 / yaw_bin   # +360° 복제
all_pts = np.vstack([pts, pts_left, pts_right])

# Delaunay 삼각분할 기반 선형 보간으로 빈 격자 채움
interp_dist = griddata(all_pts, all_vals, target_pts, method='linear')
```

- **`scipy.interpolate.griddata`**: 들로네 삼각분할(Delaunay Triangulation)을 수행하여 주변 데이터 점으로 구성된 삼각형 내부를 선형 보간합니다.
- **Wrap-around**: 원통형 스캔의 0°↔360° 경계가 자연스럽게 연결되도록 데이터를 복제합니다.

### 3-4. 동적 재질 분류 (Dynamic Peak Detection)

물체 표면의 반사 강도(Reflectivity)를 분석하여 재질 종류를 자동으로 분류합니다.

```python
# 1. 거리에 따른 빛 감쇠 역보정 (Inverse-Square Law)
reflectivity = raw_strength × (distance_cm²)

# 2. 반사율 히스토그램에서 봉우리(Peak) 동적 탐지
hist, edges = np.histogram(reflectivity, bins=50)
smooth_hist = convolve(hist, window)  # 이동평균 스무딩
peaks, _ = find_peaks(smooth_hist, distance=4, prominence=N*0.01)

# 3. 봉우리 사이의 골짜기를 경계로 재질 그룹 분할
labels = np.digitize(reflectivity, thresholds)
```

K-Means 같은 클러스터링으로 그룹 개수를 고정하는 대신, **데이터 내 실제 봉우리 개수를 동적으로 파악**하여 2~5개의 재질 레이어를 스마트하게 분류합니다.

### 3-5. 6패널 분석 대시보드

`render_analysis` 함수가 Matplotlib `2×3` 서브플롯으로 전문적인 분석 대시보드를 렌더링합니다.

| 패널 | 내용 |
|------|------|
| **1. Original + Outliers** | 살아남은 점(고도 그라데이션) + 제거된 이상치(빨간 X 마커) |
| **2. Filtered + Bounding Box** | 반사 강도 보정 컬러맵(magma) + AABB 와이어프레임 + 가로×세로×높이 치수 |
| **3. Material Segmentation** | 동적 Peak Detection으로 분류된 재질 레이어별 컬러 + 범례 |
| **4. Voxel Occupancy Grid** | 1.5cm 단위 정육면체 격자로 공간 점유 시각화 (로봇 경로 탐색 기초) |
| **5. Poisson Surface Mesh** | 법선 추정 → 포아송 표면 재구성 → 삼각형 폴리곤 렌더링 |
| **6. Cross Section** | 특정 Z 높이에서 ±0.5cm 슬라이스한 2D 단면 프로파일 |

---

## 4. 3D 메쉬 생성 (Poisson Surface Reconstruction)

필터링된 포인트 클라우드를 매끄러운 3D 표면 메쉬로 변환합니다.

```python
pcd = o3d.geometry.PointCloud(points)

# 1. 법선 벡터(Normal) 추정 — KD-Tree 기반 Hybrid 검색
pcd.estimate_normals(search_param=KDTreeSearchParamHybrid(radius=3.0, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(100)

# 2. Poisson 표면 재구성 — 포아송 방정식을 풀어 연속적 표면 생성
mesh, densities = TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)

# 3. 저밀도 삼각형 제거 — 스캔 범위 밖 아티팩트 제거
density_threshold = np.percentile(densities, 5)
mesh.remove_vertices_by_mask(densities < density_threshold)
```

- 최종 결과물은 정점(Vertex) 색상이 포함된 **Wavefront OBJ 형식**으로 내보내집니다.

---

## 5. 웹 기반 3D 뷰어 (Three.js + WebGL)

`viewer.html`은 생성된 `mesh.obj`를 웹 브라우저에서 인터랙티브하게 볼 수 있는 Three.js 기반 웹앱입니다.

| 기능 | 구현 |
|------|------|
| **OBJ 파싱** | `fetch` + `ReadableStream`으로 Chunk 단위 스트리밍 로드 + 실시간 프로그레스 바 |
| **재질** | `MeshPhongMaterial` + Vertex Color 지원 + 양면 렌더링 |
| **조명** | Directional Light 3개 + Hemisphere Light + Ambient Light |
| **카메라** | `OrbitControls` — 마우스 드래그(회전), 스크롤(줌), 우클릭(팬) |
| **UI** | Auto-Rotate 토글, 와이어프레임 모드, 그리드 토글, 뷰 초기화 |
| **디자인** | 글래스모피즘 패널, 그라데이션 타이포그래피, Z축 컬러 레전드 |

---

## 사용 방법

### 1. 필요 패키지 설치

```bash
pip install numpy matplotlib pyserial scipy open3d
```

### 2. Arduino 펌웨어 업로드

`arduino_lidar_scanner_register/arduino_lidar_scanner_register.ino`를  
Arduino IDE로 열어 보드에 업로드합니다.

### 3. 스캔 수집

```bash
# 자동 포트 감지
python arduino_3d.py

# 포트 직접 지정
python arduino_3d.py --port /dev/ttyUSB0

# 시뮬레이션 모드 (하드웨어 없이 테스트)
python arduino_3d.py --simulate
```

**실시간 명령어**: `s` 스캔 시작 | `h` 정지 | `c` 초기화 | `q` 종료 및 저장  
**디버그 명령어**: `l`/`r` DC모터 수동 | `u`/`d` 스텝모터 수동 | `t` 진단 출력 | `v` LiDAR 단독 테스트

### 4. 후처리

```bash
# 기본 파이프라인 (Z-Buffer → Adaptive SOR → Hole Fill → 6패널 대시보드)
python postprocess.py scan_points.csv

# 필터링된 CSV 저장
python postprocess.py scan_points.csv --output scan_points_filtered.csv

# Z-Buffer 격자 크기 조절
python postprocess.py scan_points.csv --yaw-bin 2.0 --pitch-bin 0.6
```

### 5. 3D 메쉬 생성 & 웹 뷰어

```bash
# Poisson Surface Mesh → mesh.obj 생성
python export_mesh.py scan_points_filtered.csv

# 로컬 웹 서버 실행 후 viewer.html 열기
python -m http.server 8000
# 브라우저에서 http://localhost:8000/viewer.html 접속
```

---

## 주요 파라미터

### 스캔 수집 (`arduino_3d.py`)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--center-offset` | 48.0 | 회전 중심 → 센서 거리 (cm) |
| `--min-distance` | 10.0 | TFmini 사각지대 제거 (cm) |
| `--max-distance` | 120.0 | 최대 측정 거리 (cm) |
| `--min-strength` | 0 | 최소 반사 강도 (0 = 필터 없음) |
| `--invert-yaw` | false | Yaw 방향 반전 |
| `--color-by` | z | 색상 기준 (`z` / `strength` / `distance`) |
| `--time-offset` | 0.0 | 동기화 타이밍 캘리브레이션 오프셋 (ms) |

### 후처리 (`postprocess.py`)

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `--k` | (자동) | SOR k-NN 이웃 수 (미지정 시 밀도 기반 자동 결정) |
| `--std-ratio` | 1.5 | SOR 표준편차 비율 |
| `--yaw-bin` | 1.0 | Z-Buffer yaw 격자 크기 (°) |
| `--pitch-bin` | 0.5 | Z-Buffer pitch 격자 크기 (°) |
| `--slice-z` | (자동) | 단면도 높이 (미지정 시 중간값) |
| `--no-zbuffer` | - | Z-Buffer 필터 비활성화 |
| `--no-adaptive-sor` | - | 적응형 SOR 비활성화 (고정 k=8) |

---

## 하드웨어 구성

| 부품 | 모델 | 역할 | 연결 |
|------|------|------|------|
| MCU | Arduino Uno (ATmega328P) | 전체 제어 | USB UART |
| LiDAR | TFmini (SoftwareSerial) | 거리 측정 (12m, 100Hz) | D8(TX), D9(RX) |
| DC Motor | + L298N 드라이버 | Yaw 360° 회전 | D4(IN1), D5(IN2), D6(PWM) |
| Encoder | 로터리 엔코더 (660 CPR) | 회전 각도 피드백 | D3(INT1), D2 |
| Stepper | 28BYJ-48 + ULN2003 | Pitch 0~45° 틸트 | D10, D11, D12, D13 |

---

## 기술 스택 요약

```
┌──────────────┬──────────────────────────────────────────────────┐
│ 하드웨어 제어 │ ATmega328P 레지스터 직접 제어 (UART, GPIO, PWM)  │
│              │ 외부 인터럽트 (INT1) · 타이머 인터럽트 (Timer1)   │
│              │ PID + Feedforward + Anti-Windup + EMA 필터       │
│              │ FSM 상태 머신 · 하프스텝 스텝모터 구동             │
├──────────────┼──────────────────────────────────────────────────┤
│ 신호 처리    │ 칼만 필터 (2D 상태 모델, 예측-갱신 루프)          │
│              │ 구면좌표 → 데카르트 변환 (오프셋 기하학 보정)      │
│              │ 360° Wrap-around 처리                             │
├──────────────┼──────────────────────────────────────────────────┤
│ 포인트 클라우드│ Z-Buffer 유령 제거 · Voxel Downsampling          │
│              │ Adaptive SOR · Radius Outlier Removal             │
│              │ Delaunay 기반 Hole Filling · KD-Tree 매핑         │
├──────────────┼──────────────────────────────────────────────────┤
│ 3D 재구성    │ Poisson Surface Reconstruction (Open3D)           │
│              │ Normal Estimation · OBJ Export with Vertex Colors │
├──────────────┼──────────────────────────────────────────────────┤
│ 데이터 분석  │ 히스토그램 Peak Detection 재질 분류                │
│              │ AABB 바운딩 박스 · 단면 프로파일 분석               │
│              │ Voxel Occupancy Grid                               │
├──────────────┼──────────────────────────────────────────────────┤
│ 소프트웨어   │ 멀티스레딩 (Daemon Thread, Lock, Queue)            │
│ 엔지니어링   │ 실시간 Matplotlib 인터랙티브 렌더링                │
│              │ Argparse CLI · CSV I/O · Thread-Safe 설계          │
├──────────────┼──────────────────────────────────────────────────┤
│ 웹 시각화    │ Three.js (WebGL) · OrbitControls                   │
│              │ Streaming OBJ Loader · 글래스모피즘 UI              │
└──────────────┴──────────────────────────────────────────────────┘
```

---

## 라이선스

이 프로젝트는 학술·교육 목적으로 제작되었습니다.
