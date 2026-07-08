# AprilTag 4-Coil Layout 계획

## 1. 기본 원칙

현재 물리 layout은 복도 주행 없이 4개 WPT 코일 사이에서 정합하는 구조입니다. 각 코일 주변에 north/east/south/west marker 4개를 배치하고, 전체적으로 16개 AprilTag marker만 사용합니다.

코드 기본값은 다음과 같습니다.

```yaml
layout_mode: four_coil_map
target_coil: coil_1
```

## 2. Marker ID Map

| Coil | Shelf 위치 | North | East | South | West |
|---|---|---:|---:|---:|---:|
| `coil_1` | (1,1) | 11 | 14 | 12 | 13 |
| `coil_2` | (1,2) | 21 | 24 | 22 | 23 |
| `coil_3` | (2,1) | 31 | 34 | 32 | 33 |
| `coil_4` | (2,2) | 41 | 44 | 42 | 43 |

## 3. Pair 사용 규칙

최종 WPT 코일 정합에는 단일 tag가 아니라 두 tag pair를 사용합니다.

- 기본 최종 정합 pair: `west_east`
- 90도 회전 후 방향 검증 pair: `north_south`

예:

| Coil | Pair | 실제 marker IDs |
|---|---|---:|
| `coil_1` | west_east | 13, 14 |
| `coil_1` | north_south | 11, 12 |
| `coil_2` | west_east | 23, 24 |
| `coil_3` | north_south | 31, 32 |
| `coil_4` | west_east | 43, 44 |

## 4. 설정 예시

```yaml
layout_mode: four_coil_map
target_coil: coil_1

alignment:
  final_pair: west_east
  rotation_verify_pair: north_south
```

`target_coil`을 `coil_1`, `coil_2`, `coil_3`, `coil_4` 중 하나로 바꾸면 같은 pair 정합 로직으로 다른 코일을 목표로 사용할 수 있습니다.
