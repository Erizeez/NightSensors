import Foundation
import AVFoundation
import CoreImage

/// 前置原深感数据输出类型
public enum FrontCaptureType: String, Sendable {
    case activeDepthNightVision // 主动红外结构光近场深度 (完全无光下运行)
    case lowLightVideo          // 可见光微光画面 (可选对比)
}

/// 前置 TrueDepth 红外与原深感数据流管理器
public final class TrueDepthInfraredStream: NSObject, @unchecked Sendable, AVCaptureDepthDataOutputDelegate, AVCaptureVideoDataOutputSampleBufferDelegate {
    
    public var onDepthReceived: (@Sendable (CVPixelBuffer) -> Void)?
    public var onVideoReceived: (@Sendable (CVPixelBuffer) -> Void)?
    
    /// 当前前置捕获模式：默认严格使用“主动红外结构光深度”，杜绝可见光冲刷
    public var captureType: FrontCaptureType = .activeDepthNightVision
    
    private let captureSession = AVCaptureSession()
    private let sessionQueue = DispatchQueue(label: "com.nightsensors.truedepth.queue", qos: .userInteractive)
    private var isRunning = false
    
    public override init() {
        super.init()
    }
    
    public func startStream() {
        // 确保具备相机访问权限
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            self.startSessionInternal()
        case .notDetermined:
            AVCaptureDevice.requestAccess(for: .video) { [weak self] granted in
                if granted {
                    self?.startSessionInternal()
                } else {
                    print("[TrueDepth] 用户未授权相机权限")
                }
            }
        case .denied, .restricted:
            print("[TrueDepth] 相机权限被拒绝或受限")
        @unknown default:
            break
        }
    }
    
    private func startSessionInternal() {
        sessionQueue.async { [weak self] in
            guard let self = self, !self.isRunning else { return }
            self.configureSession()
            self.captureSession.startRunning()
            self.isRunning = true
            print("[TrueDepth] 前置原深感会话已启动 running=\(self.captureSession.isRunning)")
        }
    }
    
    public func stopStream() {
        sessionQueue.async { [weak self] in
            guard let self = self, self.isRunning else { return }
            self.captureSession.stopRunning()
            self.isRunning = false
            print("[TrueDepth] 前置原深感会话已停止")
        }
    }
    
    private func configureSession() {
        guard !captureSession.isRunning else { return }
        captureSession.beginConfiguration()
        
        // 关键 1: 使用 .inputPriority 以允许手动绑定高精度深度格式与 activeDepthDataFormat
        captureSession.sessionPreset = .inputPriority
        
        // 优先获取前置 TrueDepth 模组 (配备 940nm 点阵投影器与红外深度传感器)
        let device = AVCaptureDevice.default(.builtInTrueDepthCamera, for: .video, position: .front)
            ?? AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .front)
        
        guard let validDevice = device else {
            print("[TrueDepth] 错误: 未检测到前置摄像头硬件")
            captureSession.commitConfiguration()
            return
        }
        
        // 清理已有输入输出
        for input in captureSession.inputs { captureSession.removeInput(input) }
        for output in captureSession.outputs { captureSession.removeOutput(output) }
        
        do {
            let input = try AVCaptureDeviceInput(device: validDevice)
            if captureSession.canAddInput(input) {
                captureSession.addInput(input)
            }
            
            // 关键 2: 遍历设备支持的格式，选取支持深度输出的模式
            let depthFormats = validDevice.formats.filter { !$0.supportedDepthDataFormats.isEmpty }
            
            // 优先选择 VGA 640x480 或 1280x720 中支持深度的格式
            let selectedFormat = depthFormats.first(where: {
                let dims = CMVideoFormatDescriptionGetDimensions($0.formatDescription)
                return dims.width == 640 && dims.height == 480
            }) ?? depthFormats.first
            
            if let targetFormat = selectedFormat {
                try validDevice.lockForConfiguration()
                validDevice.activeFormat = targetFormat
                
                // 关键 3: 显式设置 activeDepthDataFormat 为 32 位浮点深度 (避免系统静默关闭深度输出)
                let depthDataFormats = targetFormat.supportedDepthDataFormats
                let float32DepthFormat = depthDataFormats.first(where: {
                    CMFormatDescriptionGetMediaSubType($0.formatDescription) == kCVPixelFormatType_DepthFloat32
                }) ?? depthDataFormats.first
                
                if let depthFmt = float32DepthFormat {
                    validDevice.activeDepthDataFormat = depthFmt
                    print("[TrueDepth] 成功锁定前置主动红外深度格式: \(depthFmt)")
                }
                validDevice.unlockForConfiguration()
            }
            
            // 关键 4: 挂载原深感结构光深度输出
            let depthOutput = AVCaptureDepthDataOutput()
            depthOutput.setDelegate(self, callbackQueue: sessionQueue)
            depthOutput.isFilteringEnabled = true // 开启亚像素硬件平滑与空洞填补
            depthOutput.alwaysDiscardsLateDepthData = true
            if captureSession.canAddOutput(depthOutput) {
                captureSession.addOutput(depthOutput)
            }
            
            // 关键 5: 挂载视频数据输出作为帧时钟同步源
            let videoOutput = AVCaptureVideoDataOutput()
            videoOutput.setSampleBufferDelegate(self, queue: sessionQueue)
            videoOutput.alwaysDiscardsLateVideoFrames = true
            videoOutput.videoSettings = [
                kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32BGRA)
            ]
            if captureSession.canAddOutput(videoOutput) {
                captureSession.addOutput(videoOutput)
            }
            
            // 确保深度连接处于激活状态
            if let depthConnection = depthOutput.connection(with: .depthData) {
                depthConnection.isEnabled = true
            }
            
        } catch {
            print("[TrueDepth] 配置前置原深感会话异常: \(error)")
        }
        
        captureSession.commitConfiguration()
    }
    
    // MARK: - AVCaptureDepthDataOutputDelegate (核心主动红外结构光深度)
    
    public func depthDataOutput(_ output: AVCaptureDepthDataOutput, didOutput depthData: AVDepthData, timestamp: CMTime, connection: AVCaptureConnection) {
        // 关键 6: 确保转换为真实的米制单精度浮点深度 (kCVPixelFormatType_DepthFloat32)
        // 解决 TrueDepth 默认视差 (Disparity) 导致着色器数据解析异常的问题
        var metricDepth = depthData
        if depthData.depthDataType != kCVPixelFormatType_DepthFloat32 {
            metricDepth = depthData.converting(toDepthDataType: kCVPixelFormatType_DepthFloat32)
        }
        let pixelBuffer = metricDepth.depthDataMap
        onDepthReceived?(pixelBuffer)
    }
    
    public func depthDataOutput(_ output: AVCaptureDepthDataOutput, didDrop depthData: AVDepthData, timestamp: CMTime, connection: AVCaptureConnection, reason: AVCaptureOutput.DataDroppedReason) {
        // 记录掉帧诊断
    }
    
    // MARK: - AVCaptureVideoDataOutputSampleBufferDelegate
    
    public func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        // 仅当用户显式选择低光可见光模式时才发送，默认绝对黑暗下全靠 940nm 点阵深度
        guard captureType == .lowLightVideo else { return }
        guard let imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        onVideoReceived?(imageBuffer)
    }
}
