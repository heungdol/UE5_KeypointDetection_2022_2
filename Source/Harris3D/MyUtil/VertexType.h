#pragma once

UENUM(BlueprintType)
enum class EVertexType : uint8
{
	NONE
	, VERTEX_BUMP
	, VERTEX_FLAT 
	, VERTEX_SINK 
};

UENUM(BlueprintType)
enum class EVertexNormalType : uint8
{
	NONE 
	, VERTEX_UP 
	, VERTEX_PARALLEL 
	, VERTEX_DOWN 
};


class VertexType
{
public:
	
};
