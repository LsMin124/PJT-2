# asset_probe — Isaac 6.0 로봇 에셋 정밀도 조사 도구 (2026-09-04)

두 번째 AMR 모델 후보를 고르기 위해 만든 도구. 에셋을 S3에서 내려받아(`s3get.py`)
USD를 정적 조사하고(`inspect_usd.py`, `kin.py`), Isaac 헤드리스로 스폰·주행해 실측한다(`amr_probe.py`, `nc_probe2.py`).

## 실행

```bash
# 1) 목록·다운로드 (공개 S3, Assets/Isaac/6.0/Isaac/Robots/ 기준)
python3 s3list.py dirs NVIDIA/ Fraunhofer/
python3 s3get.py ~/isaac_assets/Robots Fraunhofer/O3dyn/ NVIDIA/NovaCarter/      # --top 이면 최상위 파일만
# 2) 정적 조사 (Isaac 동봉 pxr 사용, Kit 기동 없음)
./pxr_py inspect_usd.py ~/isaac_assets/Robots/NVIDIA/NovaCarter/nova_carter.usd > out.json && python3 brief.py out.json
ALLJ=1 ./pxr_py kin.py ~/isaac_assets/Robots/NVIDIA/NovaCarter/nova_carter.usd     # 관절 앵커·트랙·바퀴 콜라이더
# 3) 동적 프로브 (Isaac 인스턴스 1개만, ~90 s 기동)
cd ~/isaacsim && AMR_ASSET_DIR=~/isaac_assets/Robots ./python.sh <이 폴더>/amr_probe.py --robot o3dyn
cd ~/isaacsim && AMR_ASSET_DIR=~/isaac_assets/Robots ./python.sh <이 폴더>/nc_probe2.py     # Nova Carter, NVIDIA 표준 레시피
```

## 결과 요약 (results/ 에 원본 JSON)

정적 조사 값은 USD에서 읽은 것이고, iw.hub의 바퀴 반경 0.08·트랙 0.579는 T3에서 실측한 0.08/0.58과 일치해 방법을 검증했다.

| 후보 | 크기 L×W×H (m) | 질량 (USD) | 구동 | 바퀴 반경 / 트랙 | 콜라이더 | 시뮬 센서 | ROS2 샘플 | 라이선스 |
|---|---|---|---|---|---|---|---|---|
| iw.hub (기준) | 1.43×0.66×0.23 | 49 kg | 차동 2륜 + 캐스터 2, 리프트 4 cm | 0.08 / 0.579 | 박스·실린더·구체, 런타임 수술 필요(기존 함정) | 카메라 2 | iw_hub_ROS (2D 라이다 2·Hawk·IMU·odom) | CC BY 4.0 |
| Nova Carter | 0.73×0.90×0.62 | 48.7 kg | 차동 2륜 + 캐스터 2 | 0.14 / 0.413 | convexHull + 실린더, 정상 | Hawk 4·Owl·RPLidar 2D ×2·Hesai XT32·IMU (Sensors 변형) | Nova_Carter_ROS, Joint_States 변형 | CC BY 4.0 |
| O3dyn (Fraunhofer) | 2.03×1.45×0.89 | 515 kg | 메카넘 4륜(롤러 7×4 물리), 서스펜션 4, 파레트 그리퍼 4 | 유효 0.122 (실측) / 1.11, 축거 1.53 | 바운딩 박스 17 + 구체 168, 정상 | 없음(센서 형상만: VLP-16·SICK TiM·IMU·D435·L515) | 없음(게임패드 그래프만) | Open Logistics Foundation 1.3 |
| Forklift B | 3.03×1.13×2.94 | 미저작 | 후륜 1 구동·조향 ±60°, 전방 롤러 4, 리프트 2 m | 0.16 (구동륜 구체) | convexDecomposition, 정상 | Hawk 3 + RPLidar 2D (sensor 변형) | forklift_b_ROS (Ackermann) | CC BY 4.0 |
| Forklift C | 3.73×1.36×2.17 | 미저작 | 전륜 2 구동 + 후륜 조향 ±45°, 리프트 | 0.325 / 1.195 | convexDecomposition, 정상 | 없음 | 없음 | CC BY 4.0 |
| Jackal / Dingo / limo | 0.5 m 급 | 19 / 1.2 / 4.2 kg | 스키드 4륜 / 차동 / 4륜 | 0.098 / 0.049 / 0.045 | 단순 | PhysX Lidar·카메라 | limo_ROS만 | — |

동적 프로브(평지, 0.5 m/s 지령 4 s 직진 후 정지, 일부는 2 s 제자리 회전):

| 로봇 | 정착 | 직진 거리 | 실측 속도(마지막 1 s) | 유효 반경 | 요 편차 | 진행 방향 편차 | 제자리 회전 2 s |
|---|---|---|---|---|---|---|---|
| O3dyn | 안정(요동 3 mm) | 1.42 m | 0.359 m/s | 0.122 m | −0.6° | 0.1° | 26.4° (0.23 rad/s) |
| Forklift B | 안정(3 mm) | 1.52 m | 0.397 m/s | 0.127 m | −0.4° | −0.3° | — |
| Nova Carter, NVIDIA 표준 레시피(World + WheeledRobot + GroundPlane) | z 0, 피치 −0.09° | 1.97 m | 0.501 m/s | 0.14 (USD 값 그대로) | −0.17° | — | — |
| Nova Carter, 실험 API Articulation(chassis_link 루트) + 자작 바닥 | 7.5 cm 주저앉음, 피치 8° | 0.05 m | 0 | 헛돎 | — | — | — |

Nova Carter는 스폰 레시피에 따라 결과가 갈렸다(원인은 분리하지 못함). 표준 레시피를 쓸 것.
정적 BBox 검사에서 Forklift B 롤러가 16 m로 나왔지만 월드 스케일은 0.16 m였고 주행도 정상이었다. BBox 경고는 참고용이고 판정은 동적 프로브로 한다.
