import Foundation
import CoreHaptics

/// 基于 CoreHaptics 的空间距离变频触觉雷达引擎
public final class HapticSonarEngine: @unchecked Sendable {
    
    private var engine: CHHapticEngine?
    private var isSupported: Bool = false
    private var lastTriggerTime: TimeInterval = 0
    
    public init() {
        self.isSupported = CHHapticEngine.capabilitiesForHardware().supportsHaptics
        guard isSupported else { return }
        
        do {
            self.engine = try CHHapticEngine()
            try self.engine?.start()
            
            // 引擎意外停止时自动重启
            self.engine?.stoppedHandler = { [weak self] reason in
                try? self?.engine?.start()
            }
            self.engine?.resetHandler = { [weak self] in
                try? self?.engine?.start()
            }
        } catch {
            print("[HapticSonar] 初始化触觉引擎失败: \(error)")
        }
    }
    
    /// 根据前方障碍物距离 (0.2m ~ 5.0m) 动态触发变频蜂鸣触觉
    /// - Parameter distance: 距离（米）
    public func updateProximity(distance: Float) {
        guard isSupported, let engine = self.engine else { return }
        
        // 超出 4.0 米或无效距离不震动
        guard distance >= 0.2 && distance <= 4.0 else { return }
        
        let now = ProcessInfo.processInfo.systemUptime
        
        // 距离越近，脉冲间隔越短 (0.2m -> 0.08s 快速急促蜂鸣; 4.0m -> 0.8s 缓慢探测音)
        let normalizedDist = (distance - 0.2) / (4.0 - 0.2) // 0.0 ~ 1.0
        let interval = Double(0.08 + normalizedDist * 0.72)
        
        guard (now - lastTriggerTime) >= interval else { return }
        lastTriggerTime = now
        
        // 距离越近，震动强度与锐利度越大
        let intensityValue = max(0.2, 1.0 - normalizedDist)
        let sharpnessValue = max(0.3, 1.0 - normalizedDist * 0.7)
        
        let intensity = CHHapticEventParameter(parameterID: .hapticIntensity, value: intensityValue)
        let sharpness = CHHapticEventParameter(parameterID: .hapticSharpness, value: sharpnessValue)
        
        let event = CHHapticEvent(
            eventType: .hapticTransient,
            parameters: [intensity, sharpness],
            relativeTime: 0
        )
        
        do {
            let pattern = try CHHapticPattern(events: [event], parameters: [])
            let player = try engine.makePlayer(with: pattern)
            try player.start(atTime: CHHapticTimeImmediate)
        } catch {
            // 忽略瞬间播放错误
        }
    }
    
    public func stop() {
        engine?.stop(completionHandler: nil)
    }
}
