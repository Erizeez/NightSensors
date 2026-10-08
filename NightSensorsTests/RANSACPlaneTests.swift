import XCTest
import simd
@testable import NightSensors

final class RANSACPlaneTests: XCTestCase {
    
    func testGroundPlaneDetection() {
        let detector = RANSACPlaneDetector(maxIterations: 50, distanceThreshold: 0.04, minInlierRatio: 0.4)
        
        // 构造一个位于 y = -1.25m 的水平地面平面，外加少许随机杂散点
        var points: [simd_float3] = []
        for x in stride(from: Float(-1.5), through: 1.5, by: 0.3) {
            for z in stride(from: Float(0.5), through: 3.5, by: 0.3) {
                // 地面点 (y ≈ -1.25m，微小扰动)
                let noiseY = Float.random(in: -0.01...0.01)
                points.append(simd_float3(x, -1.25 + noiseY, z))
            }
        }
        
        // 添加 15 个空中杂散离群点
        for _ in 0..<15 {
            points.append(simd_float3(
                Float.random(in: -2...2),
                Float.random(in: 0...2),
                Float.random(in: 0.5...4)
            ))
        }
        
        let gravityDir = simd_float3(0, -9.81, 0)
        let plane = detector.detectGroundPlane(points: points, gravityDirection: gravityDir)
        
        XCTAssertNotNil(plane, "应成功提取出地面")
        guard let detected = plane else { return }
        
        XCTAssertTrue(detected.isFloorPlane)
        // 验证法向量垂直向上 (0, 1, 0)
        XCTAssertEqual(detected.normal.y, 1.0, accuracy: 0.05)
        // 验证地面距离 |d| ≈ 1.25m
        XCTAssertEqual(abs(detected.distance), 1.25, accuracy: 0.05)
    }
}
