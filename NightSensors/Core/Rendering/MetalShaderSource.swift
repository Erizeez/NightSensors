import Foundation

/// 内置 Metal 着色器源码 (支持 JIT 运行时编译，确保在任何开发环境下均可即开即用)
public enum MetalShaderSource {
    public static let code = """
    #include <metal_stdlib>
    using namespace metal;

    struct Vertex2D {
        vector_float2 position;
        vector_float2 texCoord;
    };

    struct UniformParams {
        float minDistance;
        float maxDistance;
        int displayMode; // 0: Thermal, 1: Cyberpunk Mesh, 2: Edge Glow, 3: Infrared NVG
        float time;
    };

    struct RasterizerData {
        float4 position [[position]];
        float2 texCoord;
    };

    vertex RasterizerData vertexShader(uint vertexID [[vertex_id]],
                                       constant Vertex2D *vertices [[buffer(0)]]) {
        RasterizerData out;
        out.position = float4(vertices[vertexID].position, 0.0, 1.0);
        out.texCoord = vertices[vertexID].texCoord;
        return out;
    }

    // 辅助高保真色谱: 军用热成像 Turbo LUT (0.0=近/白红炽热, 1.0=远/深蓝冷调)
    float3 turboColormap(float t) {
        t = clamp(t, 0.0, 1.0);
        float r = clamp(1.5 - abs(t * 4.0 - 1.0), 0.0, 1.0);
        float g = clamp(1.5 - abs(t * 4.0 - 2.0), 0.0, 1.0);
        float b = clamp(1.5 - abs(t * 4.0 - 3.0), 0.0, 1.0);
        
        // 极近距离 (<0.15) 高温过曝发白
        if (t < 0.15) {
            float whiteFactor = (0.15 - t) / 0.15;
            r = mix(r, 1.0, whiteFactor);
            g = mix(g, 1.0, whiteFactor);
            b = mix(b, 1.0, whiteFactor);
        }
        return float3(r, g, b);
    }

    // 1. 军用伪彩色热成像片段着色器
    fragment float4 fragmentThermal(RasterizerData in [[stage_in]],
                                    texture2d<float, access::sample> depthTexture [[texture(0)]],
                                    constant UniformParams &params [[buffer(0)]]) {
        constexpr sampler s(address::clamp_to_edge, filter::linear);
        float depth = depthTexture.sample(s, in.texCoord).r;
        
        // 背景微光网格底纹
        float2 bgGrid = abs(fract(in.texCoord * float2(24.0, 48.0) - 0.5) - 0.5);
        float bgLine = step(0.47, max(bgGrid.x, bgGrid.y)) * 0.12;
        float3 darkBase = float3(0.02, 0.03, 0.08) + float3(0.0, 0.2, 0.4) * bgLine;
        
        if (depth <= 0.10 || depth > params.maxDistance || isnan(depth)) {
            return float4(darkBase, 1.0);
        }
        
        // 归一化深度: 0.0 (最近) -> 1.0 (最远)
        float normDepth = (depth - params.minDistance) / max(0.1, params.maxDistance - params.minDistance);
        float3 color = turboColormap(normDepth);
        
        // 动态微光扫描线增强
        float scanline = sin(in.texCoord.y * 600.0) * 0.03;
        color += scanline;
        
        return float4(color, 1.0);
    }

    // 2. 赛博朋克 3D 几何线框网格 (Wireframe Mesh Grid)
    fragment float4 fragmentCyberpunk(RasterizerData in [[stage_in]],
                                      texture2d<float, access::sample> depthTexture [[texture(0)]],
                                      constant UniformParams &params [[buffer(0)]]) {
        constexpr sampler s(address::clamp_to_edge, filter::linear);
        float depth = depthTexture.sample(s, in.texCoord).r;
        
        // 赛博主色系: 霓虹青 (Cyan) 与 品红 (Magenta)
        float3 cyan = float3(0.0, 0.95, 1.0);
        float3 magenta = float3(1.0, 0.05, 0.65);
        
        // 全局空间透视网格计算
        float effectiveDepth = (depth > 0.1 && depth <= params.maxDistance && !isnan(depth)) ? depth : 3.0;
        float2 meshScale = float2(28.0, 52.0) * (1.0 / max(0.4, effectiveDepth * 0.35));
        float2 gridPos = in.texCoord * meshScale;
        float2 gridFrac = abs(fract(gridPos - 0.5) - 0.5);
        float gridLine = smoothstep(0.42, 0.48, max(gridFrac.x, gridFrac.y));
        
        // 动态脉冲扫描波 (Pulse Wave)
        float scanWave = abs(fract(effectiveDepth * 1.2 - params.time * 0.7) - 0.5);
        float waveGlow = smoothstep(0.40, 0.49, 0.5 - scanWave);
        
        float normDepth = (effectiveDepth - params.minDistance) / max(0.1, params.maxDistance - params.minDistance);
        float3 baseColor = mix(cyan, magenta, normDepth);
        
        float3 finalColor = baseColor * 0.3 + gridLine * cyan * 1.4 + waveGlow * float3(1.0, 1.0, 1.0) * 0.9;
        return float4(finalColor, 1.0);
    }

    // 3. 深度 Sobel 边缘提取发光着色器 (轮廓夜视)
    fragment float4 fragmentEdgeGlow(RasterizerData in [[stage_in]],
                                     texture2d<float, access::sample> depthTexture [[texture(0)]],
                                     constant UniformParams &params [[buffer(0)]]) {
        constexpr sampler s(address::clamp_to_edge, filter::linear);
        
        float2 texelSize = float2(1.0 / depthTexture.get_width(), 1.0 / depthTexture.get_height());
        
        // 3x3 邻域采样
        float d00 = depthTexture.sample(s, in.texCoord + float2(-texelSize.x, -texelSize.y)).r;
        float d01 = depthTexture.sample(s, in.texCoord + float2(0.0, -texelSize.y)).r;
        float d02 = depthTexture.sample(s, in.texCoord + float2(texelSize.x, -texelSize.y)).r;
        float d10 = depthTexture.sample(s, in.texCoord + float2(-texelSize.x, 0.0)).r;
        float d12 = depthTexture.sample(s, in.texCoord + float2(texelSize.x, 0.0)).r;
        float d20 = depthTexture.sample(s, in.texCoord + float2(-texelSize.x, texelSize.y)).r;
        float d21 = depthTexture.sample(s, in.texCoord + float2(0.0, texelSize.y)).r;
        float d22 = depthTexture.sample(s, in.texCoord + float2(texelSize.x, texelSize.y)).r;
        
        // Sobel 算子
        float gx = (d02 + 2.0 * d12 + d22) - (d00 + 2.0 * d10 + d20);
        float gy = (d20 + 2.0 * d21 + d22) - (d00 + 2.0 * d01 + d02);
        float grad = sqrt(gx * gx + gy * gy);
        
        // 轮廓线增强
        float edge = smoothstep(0.03, 0.15, grad);
        
        float3 edgeColor = float3(0.1, 1.0, 0.45); // 经典军用荧光绿
        float3 darkBg = float3(0.015, 0.03, 0.02);
        
        float3 result = mix(darkBg, edgeColor, edge);
        return float4(result, 1.0);
    }

    // 4. 前置红外 / 绿磷夜视仪模式 (NVG Phosphor)
    fragment float4 fragmentInfrared(RasterizerData in [[stage_in]],
                                     texture2d<float, access::sample> depthTexture [[texture(0)]],
                                     constant UniformParams &params [[buffer(0)]]) {
        constexpr sampler s(address::clamp_to_edge, filter::linear);
        float rawDepth = depthTexture.sample(s, in.texCoord).r;
        
        float intensity = 0.0;
        if (rawDepth > 0.08 && rawDepth <= params.maxDistance && !isnan(rawDepth)) {
            // 前置 3D 原深感结构光深度: 越近反光强度与浮雕越高 (0.12m ~ 1.0m)
            float norm = (rawDepth - params.minDistance) / max(0.1, params.maxDistance - params.minDistance);
            intensity = clamp(1.0 - norm, 0.0, 1.0);
            intensity = pow(intensity, 0.85); // 适度提升暗部对比度
        }
        
        // 经典军用 PVS-14 绿磷夜视镜色彩映射
        float3 nvgGreen = float3(0.12, 0.98, 0.32);
        float3 color = nvgGreen * max(0.06, intensity);
        
        // 动态微光噪点质感 (营造真实夜视镜颗粒感)
        float noise = fract(sin(dot(in.texCoord + params.time * 0.1, float2(12.9898, 78.233))) * 43758.5453);
        color += (noise - 0.5) * 0.03;
        
        // 边缘柔和暗角晕影 (提升沉浸感，无任何中间硬边椭圆)
        float2 uvDist = (in.texCoord - 0.5) * float2(1.0, 1.2);
        float distSq = dot(uvDist, uvDist);
        float vignette = 1.0 - smoothstep(0.35, 0.70, distSq);
        color *= vignette;
        
        return float4(color, 1.0);
    }
    """
}
