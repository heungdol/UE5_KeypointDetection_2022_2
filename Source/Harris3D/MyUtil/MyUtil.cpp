#include "MyUtil.h"

bool MyUtil::ReadMeshWithoutOverwrap (const  UStaticMeshComponent* sm, MeshData& meshData)
{
     // Static Mesh 정보 가져오기
    TArray <FVector> verts;
    TArray <int> tris;
    TArray <FVector> nors;
    TArray <FVector2D> uvs;
    TArray<FProcMeshTangent> tans;

    UKismetProceduralMeshLibrary::GetSectionFromStaticMesh(sm->GetStaticMesh(), 0, 0, verts, tris, nors, uvs, tans);
    
    // 버텍스 개수 판단
    if (verts.Num() <= 0 || verts.Num() > 50000)
        return false;

    std::vector<int> overlappingVert;
    std::vector<int> overlappingIndeices;

    // 중복 설정
    for(int i = 0; i < verts.Num(); i++)
    {
        overlappingVert.push_back(i);

        // 중복 버텍스 제거 위함
        // 처음부터 순회하면서 같은 위치에 있는 것을 중복으로 판단한다
        for (int j = 0; j < overlappingVert.size()-1; j++)
        {
            // 거리 판단
            float dist = FVector::Dist(verts[i], verts[j]);

            // 노멀 판단
            float dott = FVector::DotProduct(nors [i], nors[j]);
            if (dist < 0.01f && dott > 0.99f)
            {
                overlappingVert [i] = j;
                overlappingIndeices.push_back(i);
                break;
            }
        }
    }

    // 버텍스 설정
    // UV 설정
    // 노멀 설정
    for (int i = 0; i < verts.Num(); i++)
    {
        if (overlappingVert [i] != i)
            continue;
        
        meshData.positions.push_back(Eigen::Vector3d(verts[i].X, verts[i].Y, verts[i].Z));
        meshData.uvs.push_back(Eigen::Vector3d(uvs[i].X, uvs[i].Y,0));
        meshData.normals.push_back(Eigen::Vector3d(nors[i].X, nors[i].Y, nors[i].Z));

        // mesh.verts.push_back(verts[i]);
        // mesh.nors.push_back(nors[i]);
    }

    // data.positions;
    // data.uvs;
    // data.normals;

    // 페이스 인덱스 리스트 재설정
    for (int t = 0; t < tris.Num(); t++)
    {
        tris[t] = overlappingVert[tris[t]];
    }
    
    for (int t = 0; t < tris.Num(); t++)
    {
        int count = 0;

        for (int o = 0; o < overlappingIndeices.size(); o++)
        {
            if (tris[t] > overlappingIndeices[o])
                count += 1;
        }
        
        tris[t] -= count;
    }

    // 테스트
    /*
    int maxi = 0;
    for (int t = 0; t < tris.Num(); t++)
    {
        maxi = std::max (maxi, tris [t]);
    }
    */

    for (VertexNeighbor vn : meshData.neighbors)
    {
        vn.neighborIndices.clear();
    }
    meshData.neighbors.clear();
    meshData.neighbors = std::vector<VertexNeighbor> (meshData.positions.size());

    // 페이스 설정
    for (int i = 0; i < tris.Num(); i+=3)
    {
        if (i + 2 >= tris.Num())
            break;

        std::vector<Index> indexVec;
        indexVec.push_back(Index (tris [i], tris [i], tris [i]));
        indexVec.push_back(Index (tris [i+1], tris [i+1], tris [i+1]));
        indexVec.push_back(Index (tris [i+2], tris [i+2], tris [i+2]));
        
        meshData.indices.push_back(indexVec);

        meshData.neighbors [tris [i+0]].neighborIndices.push_back(tris [i+1]);
        meshData.neighbors [tris [i+0]].neighborIndices.push_back(tris [i+2]);

        meshData.neighbors [tris [i+1]].neighborIndices.push_back(tris [i+0]);
        meshData.neighbors [tris [i+1]].neighborIndices.push_back(tris [i+2]);

        meshData.neighbors [tris [i+2]].neighborIndices.push_back(tris [i+0]);
        meshData.neighbors [tris [i+2]].neighborIndices.push_back(tris [i+1]);
    }
    return true;
}
