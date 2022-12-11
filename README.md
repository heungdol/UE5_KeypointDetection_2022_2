# 3D Keypoint를 이용한 다양한 게임 캐릭터 애니메이션 구현: 암벽등반 위주로<br>
Various Game Character Animation using 3D Keypoint Detection: Focusing on Rock Climbing<br>

## 목표
- 여러 Keypoint Detection을 Unreal Engine 5 환경에서 구현할 수 있다. <br>
- 구현한 Keypoint Detection을 정해진 시나리오에 알맞도록 평가할 수 있다. <br>
    - 시나리오: 게임 캐릭터의 암벽등반 <br>
        - 오픈월드 게임에서 자주 등장하는 시스템 <br>

## 진행 사항 
- Keypoint Detection 구현 <br>
    - Harris 3D 
    - Heat Kerner Signature
    - Intrinsic Shape Signature 
    - Mesh Saliency
- 캡스톤디자인 프로젝트 수정 사항
    - Procedural Animation
        - 기존 2022_1 캡스톤디자인 컨트롤러 활용
        - https://github.com/heungdol/UE4_Harris3D_ProceduralAnimation 
## 실험  
  - 기준
    - 등반 벽 기울기
    - 홀드 형태
    - 양각 및 음각 홀드
  - 결과
    - Harris 3D / ISS가 비교적 적합함

### 출처
- https://github.com/jhonmgb/HarrisInterestPoints3d
- https://github.com/vamshikodipaka/Interest-Point-Detection-on-3D-Meshes
- https://github.com/rohan-sawhney/correspondence 
