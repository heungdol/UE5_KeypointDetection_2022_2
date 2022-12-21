<img src = "https://user-images.githubusercontent.com/30585313/208798542-811833b9-615d-4b99-9fc9-49942323150f.png" width="50%" height="50%">

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
        - 해당 Github의 컨트롤러를 이용하지 않음
        - 기존 2022_1 캡스톤디자인 컨트롤러 활용
            - 과정 중 UE4에서 UE5로 업그레이도 진행
            - https://github.com/heungdol/UE4_Harris3D_ProceduralAnimation 
## 실험  
  - 기준
    - 모델 기반
        - 등반 벽 기울기
        - 홀드 형태
        - 양각 및 음각 홀드
    - 컨트롤러 기반
        - Voxel Grid
            - 여럿 시작점으로부터 도달할 수 있는 최단 거리의 개수
## 결과
   - Harris 3D / ISS가 비교적 적합함
   - 그렇지만 사용되는 모델도 어떤 모델이냐에 따라서 결과가 달라짐

## 활용 Github 프로젝트
- https://github.com/jhonmgb/HarrisInterestPoints3d
- https://github.com/vamshikodipaka/Interest-Point-Detection-on-3D-Meshes
- https://github.com/rohan-sawhney/correspondence 

## 리포트
- https://daheung.blogspot.com/2022/12/unreal-3d-keypoint-detection.html 
