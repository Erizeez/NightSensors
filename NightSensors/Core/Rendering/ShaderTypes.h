#ifndef ShaderTypes_h
#define ShaderTypes_h

#include <simd/simd.h>

// 顶点数据结构
struct Vertex2D {
    vector_float2 position;
    vector_float2 texCoord;
};

// 传递给 Fragment Shader 的统一参数
struct UniformParams {
    float minDistance;
    float maxDistance;
    int displayMode; // 0: Thermal, 1: Cyberpunk, 2: EdgeGlow, 3: Infrared
    float time;
};

#endif /* ShaderTypes_h */
