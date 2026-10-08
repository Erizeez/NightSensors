import XCTest
import simd
@testable import NightSensors

final class MatrixAndLieAlgebraTests: XCTestCase {
    
    func testLieAlgebraExpLogConsistency() {
        // 测试小角度旋转
        let theta = simd_float3(0.05, -0.03, 0.08)
        let R = theta.expSO3()
        let recoveredTheta = R.logSO3()
        
        XCTAssertEqual(recoveredTheta.x, theta.x, accuracy: 1e-4)
        XCTAssertEqual(recoveredTheta.y, theta.y, accuracy: 1e-4)
        XCTAssertEqual(recoveredTheta.z, theta.z, accuracy: 1e-4)
    }
    
    func testSkewSymmetricProperties() {
        let v = simd_float3(1.0, 2.0, 3.0)
        let u = simd_float3(4.0, 5.0, 6.0)
        
        let skewCross = v.skewSymmetric * u
        let standardCross = simd_cross(v, u)
        
        XCTAssertEqual(skewCross.x, standardCross.x, accuracy: 1e-5)
        XCTAssertEqual(skewCross.y, standardCross.y, accuracy: 1e-5)
        XCTAssertEqual(skewCross.z, standardCross.z, accuracy: 1e-5)
    }
    
    func testMatrix6x6CholeskySolver() {
        var H = Matrix6x6(diagonal: 2.0)
        // 构造一个正定矩阵
        H[0, 1] = 0.5; H[1, 0] = 0.5
        H[2, 3] = 0.3; H[3, 2] = 0.3
        
        let b: [Float] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        guard let x = H.solveCholesky(b: b, damping: 1e-6) else {
            XCTFail("Cholesky 解算失败")
            return
        }
        
        // 验证 H * x ≈ b
        for r in 0..<6 {
            var sum: Float = 0
            for c in 0..<6 {
                sum += H[r, c] * x[c]
            }
            XCTAssertEqual(sum, b[r], accuracy: 1e-3, "行 \(r) 残差超标")
        }
    }
}
