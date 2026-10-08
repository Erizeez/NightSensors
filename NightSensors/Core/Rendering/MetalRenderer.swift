import Foundation
import MetalKit
import ARKit
import CoreVideo

/// 画面填充与缩放模式
public enum ScalingMode: String, CaseIterable, Identifiable, Sendable {
    case aspectFill = "铺满全屏"
    case aspectFit = "保持比例"
    
    public var id: String { self.rawValue }
    public var iconName: String {
        switch self {
        case .aspectFill: return "arrow.up.left.and.arrow.down.right"
        case .aspectFit: return "aspectratio"
        }
    }
}

/// Metal 着色器传入参数结构体 (与 Shaders.metal 中内存对齐严格对应)
public struct UniformParams {
    public var minDistance: Float
    public var maxDistance: Float
    public var displayMode: Int32
    public var time: Float
    
    public init(minDistance: Float, maxDistance: Float, displayMode: Int32, time: Float) {
        self.minDistance = minDistance
        self.maxDistance = maxDistance
        self.displayMode = displayMode
        self.time = time
    }
}

/// Metal 高性能 GPU 实时渲染器 (MTKViewDelegate)
public final class MetalRenderer: NSObject, MTKViewDelegate, @unchecked Sendable {
    
    public var currentMode: VisualizationMode = .thermal
    public var maxDistance: Float = 4.5
    public var minDistance: Float = 0.2
    public var isFrontCamera: Bool = false {
        didSet {
            setupQuad()
        }
    }
    
    /// 画面缩放模式：铺满全屏 (Aspect Fill) vs 保持完整比例 (Aspect Fit)
    public var scalingMode: ScalingMode = .aspectFill {
        didSet {
            setupQuad()
        }
    }
    
    /// 当前屏幕界面朝向 (支持竖屏与横屏切换)
    public var currentOrientation: UIInterfaceOrientation = .portrait {
        didSet {
            setupQuad()
        }
    }
    
    private var textureWidth: Int = 640
    private var textureHeight: Int = 480
    private var viewportSize: CGSize = CGSize(width: 390, height: 844)
    
    public let device: MTLDevice
    private let commandQueue: MTLCommandQueue
    private var textureCache: CVMetalTextureCache?
    
    private var pipelineThermal: MTLRenderPipelineState?
    private var pipelineCyberpunk: MTLRenderPipelineState?
    private var pipelineEdgeGlow: MTLRenderPipelineState?
    private var pipelineInfrared: MTLRenderPipelineState?
    
    private var vertexBuffer: MTLBuffer?
    private var currentDepthTexture: MTLTexture?
    private var currentCVTexture: CVMetalTexture?
    private var fallbackTexture: MTLTexture?
    private var startTime: TimeInterval = Date().timeIntervalSince1970
    
    private weak var attachedView: MTKView?
    
    private struct QuadVertex {
        var position: simd_float2
        var texCoord: simd_float2
    }
    
    public init?(device: MTLDevice? = MTLCreateSystemDefaultDevice()) {
        guard let dev = device, let queue = dev.makeCommandQueue() else {
            return nil
        }
        self.device = dev
        self.commandQueue = queue
        super.init()
        
        CVMetalTextureCacheCreate(kCFAllocatorDefault, nil, dev, nil, &textureCache)
        setupPipelines()
        setupQuad()
        setupFallbackTexture()
    }
    
    /// 挂载到 MTKView
    public func attach(to metalView: MTKView) {
        self.attachedView = metalView
        metalView.device = self.device
        metalView.delegate = self
        metalView.colorPixelFormat = .bgra8Unorm
        metalView.framebufferOnly = true
        metalView.clearColor = MTLClearColor(red: 0.02, green: 0.04, blue: 0.08, alpha: 1.0)
        metalView.isPaused = false
        metalView.enableSetNeedsDisplay = false
        metalView.preferredFramesPerSecond = 60
    }
    
    private func setupPipelines() {
        var lib: MTLLibrary? = device.makeDefaultLibrary()
        if lib == nil {
            lib = try? device.makeLibrary(source: MetalShaderSource.code, options: nil)
        }
        
        guard let library = lib else {
            print("[MetalRenderer] 无法加载或编译 Metal 库")
            return
        }
        
        let vertexFunc = library.makeFunction(name: "vertexShader")
        let fragThermal = library.makeFunction(name: "fragmentThermal")
        let fragCyber = library.makeFunction(name: "fragmentCyberpunk")
        let fragEdge = library.makeFunction(name: "fragmentEdgeGlow")
        let fragInfrared = library.makeFunction(name: "fragmentInfrared")
        
        let desc = MTLRenderPipelineDescriptor()
        desc.vertexFunction = vertexFunc
        desc.colorAttachments[0].pixelFormat = .bgra8Unorm
        
        do {
            if let fragThermal = fragThermal {
                desc.fragmentFunction = fragThermal
                self.pipelineThermal = try device.makeRenderPipelineState(descriptor: desc)
            }
            if let fragCyber = fragCyber {
                desc.fragmentFunction = fragCyber
                self.pipelineCyberpunk = try device.makeRenderPipelineState(descriptor: desc)
            }
            if let fragEdge = fragEdge {
                desc.fragmentFunction = fragEdge
                self.pipelineEdgeGlow = try device.makeRenderPipelineState(descriptor: desc)
            }
            if let fragInfrared = fragInfrared {
                desc.fragmentFunction = fragInfrared
                self.pipelineInfrared = try device.makeRenderPipelineState(descriptor: desc)
            }
        } catch {
            print("[MetalRenderer] 创建管线状态失败: \(error)")
        }
    }
    
    public func updateOrientation(_ orientation: UIInterfaceOrientation) {
        guard orientation != self.currentOrientation else { return }
        self.currentOrientation = orientation
    }
    
    private func setupQuad() {
        let vw = max(1.0, Float(viewportSize.width))
        let vh = max(1.0, Float(viewportSize.height))
        
        // 传感器纹理原始像素宽高
        let tw = max(1.0, Float(textureWidth))
        let th = max(1.0, Float(textureHeight))
        
        // 当前设备朝向是否将纹理垂直旋转了 90 度
        let isRotated: Bool
        switch currentOrientation {
        case .landscapeLeft, .landscapeRight:
            isRotated = false
        default:
            isRotated = true
        }
        
        // 旋转后呈现给用户的内容视觉长宽比 (视觉宽 / 视觉高)
        let contentAspect: Float = isRotated ? (th / tw) : (tw / th)
        let viewportAspect: Float = vw / vh
        
        // 计算 NDC 缩放系数 (scale_x, scale_y) 彻底消解拉伸变形
        var sx: Float = 1.0
        var sy: Float = 1.0
        
        switch scalingMode {
        case .aspectFit:
            // 完整比例容纳模式：两边留微弱黑边，视场角 100% 完整可见，完全保持真实几何比例
            if contentAspect > viewportAspect {
                sx = 1.0
                sy = viewportAspect / contentAspect
            } else {
                sx = contentAspect / viewportAspect
                sy = 1.0
            }
        case .aspectFill:
            // 等比铺满全屏模式：按长边等比放大裁切填满整屏，绝不拉伸压扁任何物体
            if contentAspect > viewportAspect {
                sx = contentAspect / viewportAspect
                sy = 1.0
            } else {
                sx = 1.0
                sy = viewportAspect / contentAspect
            }
        }
        
        // 针对不同朝向与前后置镜头的 UV 空间映射
        let uv0: simd_float2 // 左下 (-sx, -sy)
        let uv1: simd_float2 // 右下 ( sx, -sy)
        let uv2: simd_float2 // 左上 (-sx,  sy)
        let uv3: simd_float2 // 右上 ( sx,  sy)
        
        if isFrontCamera {
            // 前置 TrueDepth 镜像模式
            switch currentOrientation {
            case .landscapeRight:
                uv0 = simd_float2(1, 1); uv1 = simd_float2(0, 1); uv2 = simd_float2(1, 0); uv3 = simd_float2(0, 0)
            case .landscapeLeft:
                uv0 = simd_float2(0, 0); uv1 = simd_float2(1, 0); uv2 = simd_float2(0, 1); uv3 = simd_float2(1, 1)
            case .portraitUpsideDown:
                uv0 = simd_float2(0, 1); uv1 = simd_float2(0, 0); uv2 = simd_float2(1, 1); uv3 = simd_float2(1, 0)
            default: // .portrait
                uv0 = simd_float2(1, 0); uv1 = simd_float2(1, 1); uv2 = simd_float2(0, 0); uv3 = simd_float2(0, 1)
            }
        } else {
            // 后置 LiDAR 物理对齐
            switch currentOrientation {
            case .landscapeRight:
                uv0 = simd_float2(0, 1); uv1 = simd_float2(1, 1); uv2 = simd_float2(0, 0); uv3 = simd_float2(1, 0)
            case .landscapeLeft:
                uv0 = simd_float2(1, 0); uv1 = simd_float2(0, 0); uv2 = simd_float2(1, 1); uv3 = simd_float2(0, 1)
            case .portraitUpsideDown:
                uv0 = simd_float2(0, 0); uv1 = simd_float2(0, 1); uv2 = simd_float2(1, 0); uv3 = simd_float2(1, 1)
            default: // .portrait
                uv0 = simd_float2(1, 1); uv1 = simd_float2(1, 0); uv2 = simd_float2(0, 1); uv3 = simd_float2(0, 0)
            }
        }
        
        let vertices = [
            QuadVertex(position: simd_float2(-sx, -sy), texCoord: uv0),
            QuadVertex(position: simd_float2( sx, -sy), texCoord: uv1),
            QuadVertex(position: simd_float2(-sx,  sy), texCoord: uv2),
            
            QuadVertex(position: simd_float2(-sx,  sy), texCoord: uv2),
            QuadVertex(position: simd_float2( sx, -sy), texCoord: uv1),
            QuadVertex(position: simd_float2( sx,  sy), texCoord: uv3),
        ]
        
        self.vertexBuffer = device.makeBuffer(
            bytes: vertices,
            length: vertices.count * MemoryLayout<QuadVertex>.stride,
            options: .storageModeShared
        )
    }
    
    /// 初始化默认空纹理 (启动时保持清屏，绝不产生虚假同心圆/椭圆干扰用户)
    private func setupFallbackTexture() {
        self.fallbackTexture = nil
    }
    
    /// 清除当前纹理 (切换镜头时重置画面，避免残影)
    public func resetTexture() {
        self.currentDepthTexture = nil
        self.currentCVTexture = nil
    }
    
    /// 将每一帧中的深度或红外缓冲区绑定为 Metal 纹理
    public func updateDepthBuffer(_ pixelBuffer: CVPixelBuffer) {
        guard let cache = textureCache else { return }
        
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        
        if width != self.textureWidth || height != self.textureHeight {
            self.textureWidth = width
            self.textureHeight = height
            setupQuad()
        }
        
        let formatType = CVPixelBufferGetPixelFormatType(pixelBuffer)
        
        var mtlPixelFormat: MTLPixelFormat = .r32Float
        var planeIndex: Int = 0
        
        switch formatType {
        case kCVPixelFormatType_DepthFloat32, kCVPixelFormatType_32AlphaGray:
            mtlPixelFormat = .r32Float
        case kCVPixelFormatType_DepthFloat16:
            mtlPixelFormat = .r16Float
        case kCVPixelFormatType_OneComponent8:
            mtlPixelFormat = .r8Unorm
        case kCVPixelFormatType_32BGRA:
            mtlPixelFormat = .bgra8Unorm
        case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange, kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
            mtlPixelFormat = .r8Unorm
            planeIndex = 0 // 取 Y 平面
        default:
            mtlPixelFormat = .r32Float
        }
        
        var cvMetalTexture: CVMetalTexture?
        let status = CVMetalTextureCacheCreateTextureFromImage(
            kCFAllocatorDefault,
            cache,
            pixelBuffer,
            nil,
            mtlPixelFormat,
            width,
            height,
            planeIndex,
            &cvMetalTexture
        )
        
        if status == kCVReturnSuccess, let cvTex = cvMetalTexture {
            self.currentCVTexture = cvTex // 保持强引用
            self.currentDepthTexture = CVMetalTextureGetTexture(cvTex)
        } else {
            // 尝试以单通道格式重试
            var retryTex: CVMetalTexture?
            let retryStatus = CVMetalTextureCacheCreateTextureFromImage(
                kCFAllocatorDefault,
                cache,
                pixelBuffer,
                nil,
                .r32Float,
                width,
                height,
                0,
                &retryTex
            )
            if retryStatus == kCVReturnSuccess, let rTex = retryTex {
                self.currentCVTexture = rTex
                self.currentDepthTexture = CVMetalTextureGetTexture(rTex)
            }
        }
    }
    
    // MARK: - MTKViewDelegate
    
    public func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        if size.width > 0 && size.height > 0 && (size.width != viewportSize.width || size.height != viewportSize.height) {
            self.viewportSize = size
            setupQuad()
        }
    }
    
    public func draw(in view: MTKView) {
        guard let drawable = view.currentDrawable,
              let renderPassDesc = view.currentRenderPassDescriptor,
              let vertexBuf = self.vertexBuffer,
              let commandBuffer = commandQueue.makeCommandBuffer() else {
            return
        }
        
        // 仅使用当前实测深度纹理；若尚无深度数据则保持清屏暗底，绝不绘制假椭圆
        guard let activeTex = self.currentDepthTexture else {
            if let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDesc) {
                encoder.endEncoding()
                commandBuffer.present(drawable)
                commandBuffer.commit()
            }
            return
        }
        
        let targetPipeline: MTLRenderPipelineState?
        var modeIndex: Int32 = 0
        switch currentMode {
        case .thermal:
            targetPipeline = pipelineThermal
            modeIndex = 0
        case .cyberpunk:
            targetPipeline = pipelineCyberpunk
            modeIndex = 1
        case .edgeGlow:
            targetPipeline = pipelineEdgeGlow
            modeIndex = 2
        case .infrared:
            targetPipeline = pipelineInfrared ?? pipelineThermal
            modeIndex = 3
        }
        
        guard let pipeline = targetPipeline,
              let encoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDesc) else {
            return
        }
        
        encoder.setRenderPipelineState(pipeline)
        encoder.setVertexBuffer(vertexBuf, offset: 0, index: 0)
        encoder.setFragmentTexture(activeTex, index: 0)
        
        var params = UniformParams(
            minDistance: minDistance,
            maxDistance: maxDistance,
            displayMode: modeIndex,
            time: Float(Date().timeIntervalSince1970 - startTime)
        )
        encoder.setFragmentBytes(&params, length: MemoryLayout<UniformParams>.stride, index: 0)
        
        encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 6)
        encoder.endEncoding()
        
        commandBuffer.present(drawable)
        commandBuffer.commit()
    }
}
