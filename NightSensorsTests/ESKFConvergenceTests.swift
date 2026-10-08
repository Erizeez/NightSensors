import XCTest
import simd
@testable import NightSensors

final class ESKFConvergenceTests: XCTestCase {
    
    func testOnlineGyroBiasEstimationConvergence() {
        let eskf = ESKFStateEstimator()
        
        // 模拟陀螺仪硬件存在未校准的物理零漂 Bias: 0.03 rad/s (约 1.72°/s) 沿 Z 轴
        let trueGyroBias = simd_float3(0.0, 0.0, 0.03)
        let dt: Float = 0.02 // 50Hz
        
        // 模拟 5 秒钟的静止状态
        for step in 0..<250 {
            // 真实角速度为 0，但传感器由于偏置输出 0 + trueGyroBias
            let measuredOmega = trueGyroBias
            let measuredAccel = simd_float3(0, -9.80665, 0) // 受到重力
            
            // 1. 惯导积分预测
            eskf.predict(angularVelocity: measuredOmega, linearAcceleration: measuredAccel, dt: dt)
            
            // 2. 加速度计重力矢量倾角约束
            eskf.updateGravityConstraint(measuredAccel: measuredAccel, weight: 0.8)
            
            // 3. 模拟 LiDAR ICP 保持环境静止检测 (真实旋转量为 0)
            if step % 2 == 0 {
                eskf.updateLiDAROdometry(
                    observedRotation: matrix_identity_float3x3,
                    observedTranslation: simd_float3(0, 0, 0),
                    confidence: 0.95
                )
            }
        }
        
        // 检验 ESKF 估计出的陀螺仪偏置是否成功收敛靠近 trueGyroBias
        let estimatedBias = eskf.state.gyroBias
        XCTAssertEqual(estimatedBias.z, trueGyroBias.z, accuracy: 0.015, "陀螺仪偏置在线收敛不符合预期")
        
        // 检验姿态是否稳定没有严重翻转
        let euler = eskf.state.orientation.eulerAngles
        XCTAssertLessThan(abs(euler.x), 0.1, "Roll 发生发散")
        XCTAssertLessThan(abs(euler.y), 0.1, "Pitch 发生发散")
    }
}
