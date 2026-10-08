import XCTest
import simd
@testable import NightSensors

final class PointToPlaneICPTests: XCTestCase {
    
    func testPointToPlaneICPConvergence() {
        let icp = PointToPlaneICP(maxIterations: 10, maxCorrespondenceDistance: 0.5)
        
        // 构造一个包含水平地面 (y = 0) 和垂直墙面 (z = 2) 的目标几何结构
        var targetPoints: [OrientedPoint] = []
        for x in stride(from: Float(-1.0), through: 1.0, by: 0.2) {
            for z in stride(from: Float(0.5), through: 2.0, by: 0.2) {
                // 地面点 (法向量朝上 0, 1, 0)
                targetPoints.append(OrientedPoint(
                    position: simd_float3(x, 0.0, z),
                    normal: simd_float3(0, 1, 0)
                ))
                // 正面墙点 (法向量朝前 0, 0, -1)
                targetPoints.append(OrientedPoint(
                    position: simd_float3(x, z, 2.0),
                    normal: simd_float3(0, 0, -1)
                ))
                // 侧面墙点 (法向量朝侧面 1, 0, 0) 提供 X 轴几何约束
                targetPoints.append(OrientedPoint(
                    position: simd_float3(1.0, x, z),
                    normal: simd_float3(1, 0, 0)
                ))
            }
        }
        
        // 真实微小变换: 平移 [0.03, 0.01, -0.02], 旋转绕 Y 轴 2 度 (0.035 rad)
        let trueRot = simd_float3(0, 0.035, 0).expSO3()
        let trueTrans = simd_float3(0.03, 0.01, -0.02)
        
        // 生成源点云: P_src = R^T * (P_tgt - t)
        let invRot = trueRot.transpose
        var sourcePoints: [simd_float3] = []
        for tp in targetPoints {
            let pSrc = invRot * (tp.position - trueTrans)
            sourcePoints.append(pSrc)
        }
        
        // 运行 ICP 配准
        let result = icp.align(sourcePoints: sourcePoints, targetPoints: targetPoints)
        
        XCTAssertTrue(result.converged, "ICP 应收敛")
        XCTAssertLessThan(result.fitnessScore, 0.05, "残差应在 5cm 内")
        
        // 验证平移与旋转恢复精度
        let recoveredEuler = simd_quatf(result.deltaRotation).eulerAngles
        XCTAssertEqual(recoveredEuler.y, 0.035, accuracy: 0.02, "恢复偏航角偏差超标")
        XCTAssertEqual(result.deltaTranslation.x, trueTrans.x, accuracy: 0.02)
        XCTAssertEqual(result.deltaTranslation.y, trueTrans.y, accuracy: 0.02)
        XCTAssertEqual(result.deltaTranslation.z, trueTrans.z, accuracy: 0.02)
    }
}
