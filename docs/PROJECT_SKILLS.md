# LiDAR 3D Scanner Project - Key Skills & Concepts

이 문서는 본 3D 스캐너 프로젝트를 구현하는 데 사용된 핵심 기술, 알고리즘, 프로그래밍 스킬을 정리한 요약본입니다. 하드웨어 제어부터 3D 렌더링에 이르기까지 풀스택(Full-stack) 엔지니어링 역량이 집약되어 있습니다.

## 1. 하드웨어 및 임베디드 제어 (Arduino / C++)
* **레지스터 직접 제어 (Direct Hardware Control)**: 아두이노 기본 `Serial` 라이브러리의 병목을 피하기 위해 `UCSR0A`, `UDR0` 등의 레지스터를 직접 타격하여 115200bps의 고속 통신을 지연 없이 처리합니다.
* **비동기 인터럽트 처리 (ISR & Timers)**:
  * **외부 인터럽트 (INT1)**: 엔코더(Rotary Encoder) 핀 상태를 하드웨어 인터럽트로 실시간 추적하여 `encoderCount`를 업데이트합니다. 1회전 당 정확히 660 카운트를 잡아내어 정밀한 각도(Yaw)를 계산합니다.
  * **타이머 인터럽트 (Timer1 CTC)**: 스텝 모터의 펄스를 일정한 주기로 발생시켜 `loop()` 내의 딜레이(지연)와 무관하게 고개를 안정적으로 들도록(Pitch) 비동기 구동을 구현했습니다.
* **PID 모터 제어 (PID Control & EMA Filter)**:
  * DC 모터의 회전 속도를 일정하게 유지하기 위해 비례-적분-미분(PID) 제어기를 직접 구현했습니다. 
  * 목표 속도 달성을 위한 Feedforward 값 적용, 적분 누적 방지(Anti-Windup), 노이즈 제거를 위한 지수이동평균(EMA) 필터 등 실무적인 제어 공학 기법이 사용되었습니다.
* **하프 스텝핑 (Half-step Stepping)**: 스텝 모터를 풀스텝보다 2배 더 부드럽게 구동하기 위해 8단계 비트 시퀀스를 배열로 구성하여 직접 핀 레지스터를 제어합니다. (최대 45도까지 스캔)

## 2. 3D 수학 및 기하학 (Python / NumPy)
* **좌표계 변환 (Coordinate Transform)**:
  * LiDAR에서 얻은 구면 좌표계 `(yaw, pitch, distance)` 데이터를 데카르트 좌표계 `(x, y, z)`로 완벽히 변환합니다.
  * 단순히 삼각함수를 적용하는 것을 넘어, 센서가 터닝테이블 중심(48cm)에 위치한 물리적 오프셋 기하학(`actual_radius = 48.0 - dist * cos(pitch)`)을 수식화하여 왜곡을 바로잡았습니다.

## 3. 포인트 클라우드 후처리 파이프라인 (Open3D / SciPy)
* **Z-Buffer 고스팅 제거**: 
  * 센서 특성상 발생하는 유령점(허공에 찍히는 노이즈)을 제거하기 위해, 3D 공간을 (yaw, pitch) 격자로 나누고 동일 격자 내에서 센서와 가장 가까운 진짜 표면점 하나만 살리는 논리 필터를 설계했습니다.
* **적응형 통계 필터 (Adaptive SOR)**:
  * Open3D의 K-Nearest Neighbors(KNN) 기반 통계적 이상치 제거(SOR) 알고리즘을 사용합니다.
  * 스캔 밀도(해상도)에 따라 `k`값과 `voxel_size`를 자동으로 조절하는 적응형(Adaptive) 로직을 추가하여 필터링 성능을 극대화했습니다.
* **무조건적 빈 공간 보간 (Delaunay Triangulation / griddata)**:
  * 물리적으로 스캔이 누락된 "빵꾸"를 채우기 위해 `scipy.interpolate.griddata`를 사용했습니다.
  * 볼록 껍질(Convex Hull) 내부의 모든 빈 공간을 주변 픽셀(Delaunay 삼각형)을 기준으로 선형 보간(Linear Interpolation)하여 단 한 번에 무조건 메워버리는 강력한 토폴로지 복원 기술입니다. 360도 스캔의 Wrap-around 특성까지 고려되었습니다.

## 4. 데이터 분석 및 시각화 (NumPy / SciPy / Matplotlib)
* **히스토그램 봉우리 동적 탐지 기반 재질 분류 (Dynamic Peak Detection)**:
  * 빛의 거리에 따른 감쇠(Inverse-square law)를 역보정하여 순수한 물질 반사율(Reflectivity)을 추출합니다.
  * `SciPy`의 `find_peaks`를 활용하여 반사 강도의 분포(히스토그램)를 분석합니다. 억지로 개수를 정해놓고 쪼개는 대신, 데이터 내에 존재하는 실제 '봉우리(Peak)'의 개수를 스캔할 때마다 동적으로 파악하여 물체의 재질 개수(2개~5개 등)에 맞게 알아서 그룹을 나누는 스마트 분할 기법이 적용되어 있습니다.
* **바운딩 박스 & 단면도 분석 (AABB)**:
  * 3D 물체의 가로, 세로, 높이 최댓값/최솟값을 계산해 축 정렬 바운딩 박스(AABB)를 도출하고 총 부피를 추정합니다.
  * 특정 Z 고도에서 점들을 슬라이스(Slice)하여 단면 프로파일의 정밀도를 분석합니다.

## 5. 3D 메쉬 생성 및 웹 시각화 (Poisson / Three.js)
* **Poisson Surface Reconstruction**:
  * 점(Point) 데이터에 법선 벡터(Normals)를 추정한 뒤, 포아송 방정식을 풀어 부드러운 3D 표면(Mesh)을 가진 오브젝트 파일(`mesh.obj`)로 변환(Export)합니다.
* **웹 기반 렌더링 (HTML5 / WebGL)**:
  * 사용자가 웹 브라우저에서 스캔 결과물을 돌려볼 수 있도록 `Three.js` (WebGL 기반 3D 라이브러리)를 활용했습니다.
  * `OBJLoader`와 `OrbitControls`를 연동하여 카메라 시점 변환, 재질(MeshStandardMaterial), 조명(DirectionalLight) 세팅 등을 구현한 3D 웹앱 기술이 포함되어 있습니다.
