import SwiftUI

/// 空间传感器健康度与陀螺仪零漂补偿实时诊断视图
public struct SensorDiagnosticsView: View {
    public let diagnostics: DriftDiagnostics
    public let fps: Double
    public let pointsCount: Int
    
    @Environment(\.dismiss) private var dismiss
    
    public var body: some View {
        NavigationStack {
            List {
                Section("陀螺仪零点漂移在线消除 (Gyro Drift)") {
                    HStack {
                        Label("估计偏置 (Bias)", systemImage: "gyroscope")
                        Spacer()
                        Text(String(format: "%.3f °/s", diagnostics.gyroBiasMagnitudeDegPerSec))
                            .font(.system(.body, design: .monospaced))
                            .foregroundStyle(diagnostics.gyroBiasMagnitudeDegPerSec < 0.5 ? .green : .orange)
                    }
                    
                    VStack(alignment: .leading, spacing: 4) {
                        Text("3 轴角速度物理补偿向量 (rad/s):")
                            .font(.caption)
                            .foregroundStyle(.secondary)
                        Text(String(
                            format: "X: %.4f | Y: %.4f | Z: %.4f",
                            diagnostics.estimatedGyroBias.x,
                            diagnostics.estimatedGyroBias.y,
                            diagnostics.estimatedGyroBias.z
                        ))
                        .font(.system(.caption, design: .monospaced))
                        .foregroundStyle(.cyan)
                    }
                    
                    HStack {
                        Label("累计几何纠偏 (Yaw)", systemImage: "arrow.triangle.2.circlepath")
                        Spacer()
                        Text(String(format: "%.1f°", diagnostics.totalCompensatedYawDeg))
                            .font(.system(.body, design: .monospaced))
                            .foregroundStyle(.cyan)
                    }
                }
                
                Section("空间几何绝对约束 (Spatial Grounding)") {
                    HStack {
                        Label("重力矢量绝对对齐 (Pitch/Roll)", systemImage: "arrow.down.to.line.compact")
                        Spacer()
                        Image(systemName: diagnostics.isGravityLocked ? "checkmark.seal.fill" : "exclamationmark.triangle.fill")
                            .foregroundStyle(diagnostics.isGravityLocked ? .green : .red)
                    }
                    
                    HStack {
                        Label("LiDAR 几何匹配置信度", systemImage: "point.3.filled.connected.triangle.path")
                        Spacer()
                        Text(String(format: "%.0f%%", diagnostics.icpConfidence * 100))
                            .font(.system(.body, design: .monospaced))
                            .foregroundStyle(diagnostics.icpConfidence > 0.5 ? .green : .yellow)
                    }
                    
                    HStack {
                        Label("地面绝对基准标高", systemImage: "square.split.bottomhalf.horizontal.fill")
                        Spacer()
                        if let h = diagnostics.detectedGroundHeight {
                            Text(String(format: "%.2f m", h))
                                .font(.system(.body, design: .monospaced))
                                .foregroundStyle(.green)
                        } else {
                            Text("未锁定")
                                .foregroundStyle(.secondary)
                        }
                    }
                }
                
                Section("硬件性能与采样率 (Telemetry)") {
                    HStack {
                        Label("实时渲染帧率", systemImage: "speedometer")
                        Spacer()
                        Text(String(format: "%.0f FPS", fps))
                            .font(.system(.body, design: .monospaced))
                            .foregroundStyle(fps > 50 ? .green : .orange)
                    }
                    
                    HStack {
                        Label("活跃几何特征点数", systemImage: "circle.grid.3x3.fill")
                        Spacer()
                        Text("\(pointsCount) 点")
                            .font(.system(.body, design: .monospaced))
                    }
                }
            }
            .navigationTitle("空间传感器与算法诊断")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    Button("关闭") { dismiss() }
                }
            }
        }
    }
}
