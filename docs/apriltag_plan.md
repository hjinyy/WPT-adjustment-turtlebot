# AprilTag 제작 및 ID 계획

## 1. 태그 종류

선반 배치가 3x3(shelf 최대 8)에서 2x2(shelf 1~4)로 변경되면서, head tag는 더 이상 사용하지 않고 coil alignment tag 16개(shelf 1~4 × position 1~4)만 사용합니다.

| 종류 | 역할 | 사용 ID |
|---|---|---|
| Coil alignment tag | 송신 코일 주변 최종 정합 | 11~14, 21~24, 31~34, 41~44 (총 16개) |

기존 head tag(`100 + shelf_number`) 생성 함수는 `tag_layout.py`에 코드 그대로 남아 있지만, 상태 머신과 태그 시트 생성 스크립트에서는 더 이상 호출하지 않습니다.

## 2. ID 규칙

Coil alignment tag (변경 없음, 인쇄된 태그 모양 유지):

```text
coil_id = shelf_number * 10 + position_number
```

| 위치 | 번호 |
|---|---:|
| north/up | 1 |
| south/down | 2 |
| west/left | 3 |
| east/right | 4 |

예: 선반 4의 east tag는 `44`입니다.

## 3. 코일 주변 배치

```text
           Tag North
               □

Tag West □    ◎    □ Tag East
           Tx Coil

               □
           Tag South
```

로봇이 어느 방향(북/남/서/동)에서 선반에 진입할지 알 수 없으므로, 전방 카메라 1대 + 하부/측면 카메라 2대로 마주보는 두 태그를 한 쌍(pair)으로 인식해 정합하는 정밀 방식을 ROS2 노드(`wpt_alignment_node`)에 구현해 뒀습니다. 기본 페어는 `west_east`(`alignment.final_pair`, `config/wpt_alignment.yaml`)이며, 두 태그의 중점(midpoint)과 두 태그를 잇는 각도(pair angle)를 정합 기준으로 사용합니다. 자세한 알고리즘은 `docs/algorithm.md` 참고. 다만 현재 실제 실험은 이 노드 없이 3x3 그리드 방식으로 진행 중입니다 — README 참고.

## 4. 출력 권장

1. 기본 family는 `tag36h11`입니다.
2. 무광 출력이 유리합니다.
3. 태그 주변 흰색 여백을 충분히 둡니다.
4. 출력 후 실제 태그 크기를 측정해 기록합니다.

## 5. 태그 시트 생성

```bash
python3 scripts/generate_tag_sheet.py --shelves 1 2 3 4 --output-dir generated_tags
```

이 스크립트는 공식 AprilTag 이미지 저장소에서 PNG를 내려받아 A4 출력용 PNG/PDF를 생성합니다.
