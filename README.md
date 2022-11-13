# 3D Keypoint를 이용한 다양한 게임 캐릭터 애니메이션 구현: 암벽등반 위주로<br>
Various Game Character Animation using 3D Keypoint Detection: Focusing on Rock Climbing<br>

## 목표
- 여러 Keypoint Detection을 Unreal Engine 5 환경에서 구현할 수 있다. <br>
- 구현한 Keypoint Detection을 정해진 시나리오에 알맞도록 평가할 수 있다. <br>
    - 시나리오: 게임 캐릭터의 암벽등반 <br>
        - 오픈월드 게임에서 자주 등장하는 시스템 <br>

## 진행 사항
- Keypoint Detection 구현 <br>
    - Harris 3D (완료)
    - Heat Kerner Signature (완료)
    - Intrinsic Shape Signature (완료)
    - Mesh Saliency (완료)
- 캡스톤디자인 프로젝트 수정 사항
    - Procedural Animation: 횡단하는 암벽등반 모션 수정
        - 절벽 횡단할 때의 구분 동작 수정 (예정)
        - Stylized에서 Realistic스럽게 (예정)
    - Keypoint Dection 구현 및 추가 (완료)
    - UV, Normal, Tangent 등의 불일치로 생기는 Vertex 중복 문제 해결 (완료)
- 기준 세우기
    - 도메인 변경은 잘 진행되었으나 여기서 찾을 수 있는 가치 판단
    - Keypoint Detection을 평가할 수 있는 기준 확립
    - 암벽등반센터 설치 가이드를 참고할 예정
    
## 앞으로 해야할 것
- Keypoint Detection을 평가할 수 있는 기준 세우기
- 실험진행 및 결과 도출
- 학부 졸업 논문 작성
    - 요약, 서론: 자료 조사 토대로 작성
    - 본론, 실험, 결과, 결론: 실험을 수행한 내용 토대로 작성
- Keypoint Detection을 수행할 때마다 발생하는 메모리 누수 문제 해결

### 출처
- https://github.com/jhonmgb/HarrisInterestPoints3d
- https://github.com/vamshikodipaka/Interest-Point-Detection-on-3D-Meshes
- https://github.com/rohan-sawhney/correspondence 
