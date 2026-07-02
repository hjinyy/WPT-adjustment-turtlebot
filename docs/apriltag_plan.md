# AprilTag Station Map 계획

## 1. 기본 원칙

현재 물리 floor layout은 `apriltag_sheet.pdf`에 인쇄된 numeric marker ID를 그대로 사용합니다. `A02N`, `A02E` 같은 문자열은 사람이 읽는 station label이고, AprilTag detector가 반환하는 값은 숫자 ID입니다.

예:

| Label | 의미 | 실제 marker ID |
|---|---|---:|
| A02N | A02 station north marker | 5 |
| A02E | A02 station east marker | 6 |
| A02S | A02 station south marker | 7 |
| A02W | A02 station west marker | 8 |

기존 `111/112/113/114` 방식은 backwards compatibility 용도로만 남기고, 실제 실험의 기본값은 station map입니다.

## 2. Floor Node Markers

| Node | ID |
|---|---:|
| A01 | 1 |
| B01 | 2 |
| C01 | 3 |
| D01 | 4 |
| D02 | 17 |
| A03 | 18 |
| B03 | 19 |
| C03 | 20 |
| D04 | 37 |
| A05 | 38 |
| B05 | 39 |
| C05 | 40 |
| D05 | 41 |

## 3. Station Markers

| Station | North | East | South | West |
|---|---:|---:|---:|---:|
| A02 | 5 | 6 | 7 | 8 |
| B02 | 9 | 10 | 11 | 12 |
| C02 | 13 | 14 | 15 | 16 |
| D03 | 21 | 22 | 23 | 24 |
| A04 | 25 | 26 | 27 | 28 |
| B04 | 29 | 30 | 31 | 32 |
| C04 | 33 | 34 | 35 | 36 |

## 4. Pair 사용 규칙

최종 WPT 코일 정합에는 단일 tag가 아니라 두 tag pair를 사용합니다.

- 기본 최종 정합 pair: `west_east`
- 90도 회전 후 방향 검증 pair: `north_south`

예:

| Station | Pair | 실제 marker IDs |
|---|---|---:|
| A02 | west_east | 8, 6 |
| A02 | north_south | 5, 7 |
| B02 | west_east | 12, 10 |
| C04 | north_south | 33, 35 |

## 5. 설정

```yaml
layout_mode: station_map
target_station: A02

alignment:
  final_pair: west_east
  rotation_verify_pair: north_south
```
