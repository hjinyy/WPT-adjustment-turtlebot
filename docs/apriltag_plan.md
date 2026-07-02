# AprilTag 제작 및 ID 계획

## 1. 태그 종류

| 종류 | 역할 | 예시 ID |
|---|---|---|
| Head tag | 복도 주행 중 선반 진입 위치 판단 | 101, 102, ... |
| Coil alignment tag | 송신 코일 주변 최종 정합 | 111~114, 121~124, ... |

## 2. ID 규칙

Head tag:

```text
head_id = 100 + shelf_number
```

Coil alignment tag:

```text
coil_id = 100 + 10 * shelf_number + position_number
```

| 위치 | 번호 |
|---|---:|
| north/up | 1 |
| south/down | 2 |
| west/left | 3 |
| east/right | 4 |

예: 선반 1의 west/east pair는 `113`, `114`입니다.
예: 선반 6의 east tag는 `164`입니다.

## 3. 코일 주변 배치

```text
           Tag North
               □

Tag West □    ◎    □ Tag East
           Tx Coil

               □
           Tag South
```

최종 WPT 코일 정합에는 단일 tag가 아니라 두 tag pair를 사용합니다.

- 기본 최종 정합 pair: West/East, 예: 선반 1은 `113`, `114`
- 90도 회전 후 방향 검증 pair: North/South, 예: 선반 1은 `111`, `112`

## 4. 출력 권장

1. 기본 family는 `tag36h11`입니다.
2. 무광 출력이 유리합니다.
3. 태그 주변 흰색 여백을 충분히 둡니다.
4. 출력 후 실제 태그 크기를 측정해 기록합니다.

## 5. 태그 시트 생성

```bash
python3 scripts/generate_tag_sheet.py --shelves 1 2 3 4 5 6 7 8 --output-dir generated_tags
```

이 스크립트는 공식 AprilTag 이미지 저장소에서 PNG를 내려받아 A4 출력용 PNG/PDF를 생성합니다.
