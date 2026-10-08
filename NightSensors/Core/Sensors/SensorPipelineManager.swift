import Foundation
import ARKit
import CoreMotion
import Combine

/// 硬件摄像头工作模式
public enum CameraMode: String, CaseIterable, Identifiable, Sendable {
    case rearLiDAR = "后置 LiDAR"
    case frontTrueDepth = "前置 原深感"
    
    public var id: String { self.rawValue }
    public var iconName: String {
        switch self {
        case .rearLiDAR: return "camera.macro"
        case .frontTrueDepth: return "person.crop.square.fill"
        }
    }
}

/// 综合传感器调度与硬件管线管理器
public final class SensorPipelineManager: NSObject, ObservableObject, ARSessionDelegate, @unchecked Sendable {
    
    // MARK: - 发布给 UI 的实时响应式状态
    
    @Published public var currentCenterDistance: Float = 0.0
    @Published public var isLiDARAvailable: Bool = false
    @Published public var activeCamera: CameraMode = .rearLiDAR
    @Published public var diagnostics: DriftDiagnostics = DriftDiagnostics(
        estimatedGyroBias: simd_float3(0, 0, 0),
        gyroBiasMagnitudeDegPerSec: 0,
        isGravityLocked: false,
        icpConfidence: 0,
        detectedGroundHeight: nil,
        totalCompensatedYawDeg: 0
    )
    @Published public var currentFPS: Double = 60.0
    @Published public var trackedPointsCount: Int = 0
    @Published public var scalingMode: ScalingMode = .aspectFill {
        didSet {
            renderer.scalingMode = scalingMode
        }
    }
    @Published public var currentOrientation: UIInterfaceOrientation = .portrait {
        didSet {
            renderer.updateOrientation(currentOrientation)
        }
    }
    
    /// 切换画面缩放模式：铺满全屏 vs 保持比例完整视野
    public func toggleScalingMode() {
        DispatchQueue.main.async {
            self.scalingMode = (self.scalingMode == .aspectFill) ? .aspectFit : .aspectFill
        }
    }
    
    // 渲染回调闭包 (将实时深度或红外视频帧直通 Metal GPU)
    public var onDepthPixelBuffer: (@Sendable (CVPixelBuffer) -> Void)?
    
    // MARK: - 内部组件
    
    public let arSession = ARSession()
    public let motionManager = CMMotionManager()
    public let driftEngine = DriftCompensationEngine()
    public let hapticEngine = HapticSonarEngine()
    public let trueDepthStream = TrueDepthInfraredStream()
    public let renderer: MetalRenderer = MetalRenderer()!
    
    private let processingQueue = DispatchQueue(label: "com.nightsensors.pipeline.queue", qos: .userInteractive)
    private var lastFrameTime: TimeInterval = 0
    private var isRunning = false
    
    public override init() {
        super.init()
        self.isLiDARAvailable = ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
        arSession.delegate = self
        setupFrontStreamForwarding()
    }
    
    private func setupFrontStreamForwarding() {
        // 前置原深感结构光深度直接灌入 Metal GPU 渲染器 (绝对黑暗中完全主动红外工作)
        trueDepthStream.onDepthReceived = { [weak self] frontDepth in
            guard let self = self else { return }
            self.renderer.updateDepthBuffer(frontDepth)
            self.onDepthPixelBuffer?(frontDepth)
            self.readCenterDistance(from: frontDepth)
            
            // 前置高频帧率统计
            let now = CACurrentMediaTime()
            if self.lastFrameTime > 0 {
                let delta = now - self.lastFrameTime
                if delta > 0 && delta < 1.0 {
                    let instantFPS = 1.0 / delta
                    DispatchQueue.main.async {
                        self.currentFPS = self.currentFPS * 0.85 + instantFPS * 0.15
                    }
                }
            }
            self.lastFrameTime = now
        }
        trueDepthStream.onVideoReceived = { [weak self] videoBuffer in
            guard let self = self else { return }
            self.renderer.updateDepthBuffer(videoBuffer)
            self.onDepthPixelBuffer?(videoBuffer)
        }
    }
    
    // MARK: - 生命周期与前后置摄像头切换控制
    
    public func startPipeline() {
        guard !isRunning else { return }
        isRunning = true
        startIMU()
        applyCameraMode(activeCamera)
    }
    
    public func stopPipeline() {
        guard isRunning else { return }
        isRunning = false
        arSession.pause()
        trueDepthStream.stopStream()
        motionManager.stopDeviceMotionUpdates()
        hapticEngine.stop()
    }
    
    /// 切换前后置摄像头并自适应近场/远场深度范围
    public func switchCamera(to mode: CameraMode) {
        guard activeCamera != mode else { return }
        DispatchQueue.main.async {
            self.activeCamera = mode
            self.renderer.isFrontCamera = (mode == .frontTrueDepth)
            if mode == .frontTrueDepth {
                // 前置原深感人脸近场自适应深度区间 (0.12m ~ 1.0m)
                self.renderer.minDistance = 0.12
                self.renderer.maxDistance = 1.0
            } else {
                // 后置 LiDAR 空间探测深度区间 (0.2m ~ 4.5m)
                self.renderer.minDistance = 0.20
                self.renderer.maxDistance = 4.5
            }
        }
        applyCameraMode(mode)
    }
    
    private func applyCameraMode(_ mode: CameraMode) {
        // 关键：切换镜头时重置渲染器纹理，消除前一个镜头的残留画面与伪影
        self.renderer.resetTexture()
        switch mode {
        case .rearLiDAR:
            trueDepthStream.stopStream()
            startRearLiDARSession()
        case .frontTrueDepth:
            arSession.pause()
            trueDepthStream.startStream()
        }
    }
    
    private func startRearLiDARSession() {
        let config = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            config.frameSemantics.insert(.sceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.smoothedSceneDepth) {
            config.frameSemantics.insert(.smoothedSceneDepth)
        }
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            config.sceneReconstruction = .meshWithClassification
        } else if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
            config.sceneReconstruction = .mesh
        }
        
        config.worldAlignment = .gravity
        arSession.run(config, options: [.resetTracking, .removeExistingAnchors])
    }
    
    private func startIMU() {
        if motionManager.isDeviceMotionAvailable {
            motionManager.deviceMotionUpdateInterval = 0.01 // 100Hz
            motionManager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: .main) { [weak self] motion, error in
                guard let self = self, let motion = motion else { return }
                
                let gyro = simd_float3(
                    Float(motion.rotationRate.x),
                    Float(motion.rotationRate.y),
                    Float(motion.rotationRate.z)
                )
                
                let accel = simd_float3(
                    Float(motion.userAcceleration.x + motion.gravity.x * 9.80665),
                    Float(motion.userAcceleration.y + motion.gravity.y * 9.80665),
                    Float(motion.userAcceleration.z + motion.gravity.z * 9.80665)
                )
                
                self.driftEngine.processIMU(
                    angularVelocity: gyro,
                    linearAcceleration: accel,
                    timestamp: motion.timestamp
                )
            }
        }
    }
    
    // MARK: - ARSessionDelegate 深度与几何更新 (后置 LiDAR)
    
    public func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard activeCamera == .rearLiDAR else { return }
        
        let now = frame.timestamp
        if lastFrameTime > 0 {
            let delta = now - lastFrameTime
            if delta > 0 {
                let instantFPS = 1.0 / delta
                DispatchQueue.main.async {
                    self.currentFPS = self.currentFPS * 0.9 + instantFPS * 0.1
                }
            }
        }
        lastFrameTime = now
        
        guard let depthMap = frame.sceneDepth?.depthMap ?? frame.smoothedSceneDepth?.depthMap else {
            return
        }
        
        // 核心：立即直通派发给 Metal 渲染器实时上屏！
        self.renderer.updateDepthBuffer(depthMap)
        onDepthPixelBuffer?(depthMap)
        
        // 1. 获取屏幕中心距离并驱动触觉雷达
        readCenterDistance(from: depthMap)
        
        // 2. 异步处理点云配准与漂移消除
        processingQueue.async { [weak self] in
            guard let self = self else { return }
            
            let cameraIntrinsics = frame.camera.intrinsics
            let rawPoints = self.extractPointCloud(from: depthMap, intrinsics: cameraIntrinsics, step: 8)
            
            let diag = self.driftEngine.processLiDARPointCloud(rawPoints: rawPoints, timestamp: now)
            
            DispatchQueue.main.async {
                self.diagnostics = diag
                self.trackedPointsCount = rawPoints.count
            }
        }
    }
    
    private func readCenterDistance(from depthMap: CVPixelBuffer) {
        let centerDist = readCenterDepth(depthMap: depthMap)
        DispatchQueue.main.async {
            self.currentCenterDistance = centerDist
        }
        hapticEngine.updateProximity(distance: centerDist)
    }
    
    // MARK: - 深度图快速取样与点云反投影
    
    private func readCenterDepth(depthMap: CVPixelBuffer) -> Float {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return 0 }
        
        let stride = bytesPerRow / MemoryLayout<Float32>.size
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        
        // 中心 3x3 邻域中值滤波采样，杜绝单个像素散斑空洞导致 NaN
        let cx = width / 2
        let cy = height / 2
        var validDepths: [Float] = []
        
        for dy in -1...1 {
            for dx in -1...1 {
                let x = cx + dx
                let y = cy + dy
                if x >= 0 && x < width && y >= 0 && y < height {
                    let d = floatBuffer[y * stride + x]
                    if !d.isNaN && d > 0.05 && d < 10.0 {
                        validDepths.append(d)
                    }
                }
            }
        }
        
        if validDepths.isEmpty { return 0 }
        validDepths.sort()
        return validDepths[validDepths.count / 2]
    }
    
    private func extractPointCloud(
        from depthMap: CVPixelBuffer,
        intrinsics: simd_float3x3,
        step: Int = 8
    ) -> [simd_float3] {
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else { return [] }
        
        let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
        
        let fx = intrinsics.columns.0.x
        let fy = intrinsics.columns.1.y
        let cx = intrinsics.columns.2.x
        let cy = intrinsics.columns.2.y
        
        var points: [simd_float3] = []
        points.reserveCapacity((width / step) * (height / step))
        
        for y in stride(from: 0, to: height, by: step) {
            let rowOffset = y * width
            for x in stride(from: 0, to: width, by: step) {
                let depth = floatBuffer[rowOffset + x]
                if depth > 0.2 && depth < 5.0 && !depth.isNaN {
                    // 相机坐标系反投影: X = (u - cx)*Z / fx,  Y = (v - cy)*Z / fy,  Z = depth
                    let px = (Float(x) - cx) * depth / fx
                    let py = (Float(y) - cy) * depth / fy
                    points.append(simd_float3(px, py, depth))
                }
            }
        }
        
        return points
    }
}
