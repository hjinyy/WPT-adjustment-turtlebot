# 카메라별 캘리브레이션 절차

## 1. 목적

실제 송수신 코일이 정합된 상태에서 각 카메라에 보이는 AprilTag의 기준 좌표를 저장합니다. 이 값을 `target_x`, `target_y`, `target_angle`로 사용합니다.

## 2. 절차

1. TurtleBot을 손으로 정확한 WPT 코일 정합 위치에 놓습니다.
2. 전방/하부/측면 카메라 영상을 확인합니다.
3. 각 카메라에서 보이는 tag의 `center_x`, `center_y`, `angle_deg`를 기록합니다.
4. `config/wpt_alignment.yaml`의 `shelves.<번호>.targets`에 기록합니다.
5. `dry_run: true`로 속도 명령 부호를 확인합니다.
6. 바퀴를 띄우거나 비상정지 가능한 환경에서 `dry_run: false`를 테스트합니다.

## 3. YAML 예시

```yaml
shelves:
  "1":
    targets:
      front:
        head: {x: 320, y: 240, angle_deg: 0}
      left_bottom:
        "11": {x: 318, y: 219, angle_deg: 0}
      right_bottom:
        "14": {x: 317, y: 229, angle_deg: 180}
```

## 4. 1 mm 정합 주의사항

카메라만으로 1 mm 정합을 보장하기는 어렵습니다. 렌즈 캘리브레이션, 무광 조명, 저속 접근, N프레임 조건, V자 가이드/스토퍼 같은 물리 구조를 같이 고려하는 것이 좋습니다.
