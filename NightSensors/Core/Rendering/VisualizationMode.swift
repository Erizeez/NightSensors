import Foundation

/// 实时夜视渲染模式
public enum VisualizationMode: String, CaseIterable, Identifiable, Sendable {
    case thermal = "热成像"
    case cyberpunk = "赛博网格"
    case edgeGlow = "轮廓夜视"
    case infrared = "前置红外"
    
    public var id: String { self.rawValue }
    
    public var iconName: String {
        switch self {
        case .thermal: return "flame.fill"
        case .cyberpunk: return "waveform.path.ecg"
        case .edgeGlow: return "viewfinder"
        case .infrared: return "camera.metering.spot"
        }
    }
}
