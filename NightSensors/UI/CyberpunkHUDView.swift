import SwiftUI

/// 极富科技感的暗夜感知 HUD 交互界面
public struct CyberpunkHUDView: View {
    @Environment(\.verticalSizeClass) private var verticalSizeClass
    @Binding public var currentMode: VisualizationMode
    @Binding public var maxDistance: Float
    public let activeCamera: CameraMode
    public let centerDistance: Float
    public let fps: Double
    public let isLiDARAvailable: Bool
    public let scalingMode: ScalingMode
    public let onSwitchCamera: () -> Void
    public let onToggleScaling: () -> Void
    public let onOpenDiagnostics: () -> Void
    
    // 距离颜色判断
    private var distanceColor: Color {
        if centerDistance <= 0.05 { return .gray }
        if centerDistance < 0.8 { return .red }
        if centerDistance < 2.0 { return .yellow }
        return .cyan
    }
    
    public var body: some View {
        VStack {
            // MARK: - 顶部状态栏与相机翻转切换
            HStack(spacing: 8) {
                // 1. 传感器状态标识
                HStack(spacing: 6) {
                    Circle()
                        .fill(isLiDARAvailable ? Color.green : Color.red)
                        .frame(width: 8, height: 8)
                    Text(activeCamera == .rearLiDAR ? "LiDAR ACTIVE" : "TRUEDEPTH IR")
                        .font(.system(size: 10, weight: .bold, design: .monospaced))
                        .foregroundStyle(.white)
                }
                .padding(.horizontal, 9)
                .padding(.vertical, 6)
                .background(Color.black.opacity(0.65))
                .clipShape(Capsule())
                
                Spacer()
                
                // 2. 画面比例模式切换 (铺满全屏 vs 完整比例)
                Button(action: onToggleScaling) {
                    HStack(spacing: 4) {
                        Image(systemName: scalingMode == .aspectFill ? "arrow.up.left.and.arrow.down.right" : "aspectratio")
                            .font(.system(size: 11, weight: .bold))
                        Text(scalingMode == .aspectFill ? "铺满全屏" : "保持比例")
                            .font(.system(size: 10, weight: .bold))
                    }
                    .foregroundStyle(.white)
                    .padding(.horizontal, 9)
                    .padding(.vertical, 6)
                    .background(Color.purple.opacity(0.35))
                    .clipShape(Capsule())
                    .overlay(
                        Capsule().stroke(Color.purple.opacity(0.8), lineWidth: 1.2)
                    )
                }
                
                // 3. 前后置摄像头切换按钮
                Button(action: onSwitchCamera) {
                    HStack(spacing: 4) {
                        Image(systemName: "arrow.triangle.2.circlepath.camera.fill")
                            .font(.system(size: 12, weight: .semibold))
                        Text(activeCamera == .rearLiDAR ? "切前置" : "切后置")
                            .font(.system(size: 10, weight: .bold))
                    }
                    .foregroundStyle(.white)
                    .padding(.horizontal, 9)
                    .padding(.vertical, 6)
                    .background(Color.cyan.opacity(0.35))
                    .clipShape(Capsule())
                    .overlay(
                        Capsule().stroke(Color.cyan, lineWidth: 1.2)
                    )
                }
                
                // 4. FPS 与诊断入口
                Button(action: onOpenDiagnostics) {
                    HStack(spacing: 3) {
                        Image(systemName: "waveform.path.ecg")
                            .font(.system(size: 10))
                        Text(String(format: "%.0f", fps))
                            .font(.system(size: 10, weight: .bold, design: .monospaced))
                    }
                    .foregroundStyle(.cyan)
                    .padding(.horizontal, 8)
                    .padding(.vertical, 6)
                    .background(Color.black.opacity(0.65))
                    .clipShape(Capsule())
                    .overlay(
                        Capsule().stroke(Color.cyan.opacity(0.5), lineWidth: 1)
                    )
                }
            }
            .padding(.horizontal, 16)
            .padding(.top, 12)
            
            Spacer()
            
            // MARK: - 中央极简无遮挡准星与动态测距 (无任何椭圆/圆环)
            VStack(spacing: 10) {
                // 极简微型十字准星 (无圆环，仅微小发光线标，绝不遮挡视野)
                ZStack {
                    // 水平准线
                    Rectangle()
                        .fill(distanceColor)
                        .frame(width: 14, height: 1.5)
                    // 垂直准线
                    Rectangle()
                        .fill(distanceColor)
                        .frame(width: 1.5, height: 14)
                }
                .shadow(color: distanceColor.opacity(0.8), radius: 3)
                .animation(.easeInOut(duration: 0.2), value: centerDistance)
                
                // 极简悬浮测距胶囊
                if centerDistance > 0.05 {
                    Text(String(format: "%.2f M", centerDistance))
                        .font(.system(size: 16, weight: .heavy, design: .monospaced))
                        .foregroundStyle(distanceColor)
                        .padding(.horizontal, 10)
                        .padding(.vertical, 3)
                        .background(Color.black.opacity(0.75))
                        .clipShape(Capsule())
                        .overlay(
                            Capsule().stroke(distanceColor.opacity(0.5), lineWidth: 1)
                        )
                } else {
                    Text("STANDBY")
                        .font(.system(size: 11, weight: .bold, design: .monospaced))
                        .foregroundStyle(.white.opacity(0.4))
                        .padding(.horizontal, 8)
                        .padding(.vertical, 3)
                        .background(Color.black.opacity(0.6))
                        .clipShape(Capsule())
                }
            }
            
            Spacer()
            
            // MARK: - 底部控制面板
            VStack(spacing: verticalSizeClass == .compact ? 6 : 12) {
                // 距离阈值滑块 (后置模式可调)
                if activeCamera == .rearLiDAR {
                    HStack {
                        Text("探测深度: \(String(format: "%.1f", maxDistance))m")
                            .font(.system(size: 11, weight: .medium, design: .monospaced))
                            .foregroundStyle(.white.opacity(0.85))
                        
                        Slider(value: $maxDistance, in: 1.0...5.0, step: 0.5)
                            .tint(.cyan)
                    }
                    .padding(.horizontal, verticalSizeClass == .compact ? 40 : 20)
                }
                
                // 模式选择胶囊
                HStack(spacing: verticalSizeClass == .compact ? 6 : 10) {
                    ForEach(VisualizationMode.allCases) { mode in
                        Button(action: {
                            currentMode = mode
                        }) {
                            VStack(spacing: 2) {
                                Image(systemName: mode.iconName)
                                    .font(.system(size: verticalSizeClass == .compact ? 13 : 15))
                                Text(mode.rawValue)
                                    .font(.system(size: verticalSizeClass == .compact ? 9 : 11, weight: .medium))
                            }
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, verticalSizeClass == .compact ? 5 : 8)
                            .background(currentMode == mode ? Color.cyan.opacity(0.3) : Color.black.opacity(0.55))
                            .foregroundStyle(currentMode == mode ? Color.cyan : Color.white.opacity(0.75))
                            .clipShape(RoundedRectangle(cornerRadius: 8))
                            .overlay(
                                RoundedRectangle(cornerRadius: 8)
                                    .stroke(currentMode == mode ? Color.cyan : Color.clear, lineWidth: 1.2)
                            )
                        }
                    }
                }
                .padding(.horizontal, verticalSizeClass == .compact ? 40 : 14)
            }
            .padding(.bottom, verticalSizeClass == .compact ? 8 : 20)
            .background(
                LinearGradient(
                    colors: [Color.clear, Color.black.opacity(0.85)],
                    startPoint: .top,
                    endPoint: .bottom
                )
            )
        }
    }
}
