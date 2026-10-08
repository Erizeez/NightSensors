import Foundation
import simd

public extension simd_float3 {
    /// 反对称矩阵 [v]_x (Skew-symmetric matrix)，满足 [v]_x * u = v × u
    var skewSymmetric: simd_float3x3 {
        return simd_float3x3(
            simd_float3(0, self.z, -self.y),
            simd_float3(-self.z, 0, self.x),
            simd_float3(self.y, -self.x, 0)
        )
    }

    /// 李代数 so(3) 指数映射到 SO(3) 旋转矩阵 (Rodrigues 公式)
    func expSO3() -> simd_float3x3 {
        let theta = simd_length(self)
        if theta < 1e-7 {
            // 一阶泰勒展开避免除零：I + [v]_x
            return matrix_identity_float3x3 + self.skewSymmetric
        }
        
        let axis = self / theta
        let K = axis.skewSymmetric
        let I = matrix_identity_float3x3
        let sinTheta = sin(theta)
        let cosTheta = cos(theta)
        
        // R = I + sin(θ)*K + (1 - cos(θ))*K^2
        return I + sinTheta * K + (1.0 - cosTheta) * (K * K)
    }

    /// 指数映射到单位四元数
    func expQuaternion() -> simd_quatf {
        let theta = simd_length(self)
        if theta < 1e-7 {
            return simd_quatf(angle: 0, axis: simd_float3(0, 0, 1))
        }
        let axis = self / theta
        return simd_quatf(angle: theta, axis: axis)
    }
}

public extension simd_float3x3 {
    /// 提取李代数 so(3) 向量 (对数映射)
    func logSO3() -> simd_float3 {
        let trace = self.columns.0.x + self.columns.1.y + self.columns.2.z
        let cosTheta = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
        let theta = acos(cosTheta)
        
        if theta < 1e-7 {
            return simd_float3(0, 0, 0)
        }
        
        if abs(theta - Float.pi) < 1e-4 {
            // 奇异点处理
            let x = sqrt(max(0, (self.columns.0.x + 1.0) * 0.5))
            let y = sqrt(max(0, (self.columns.1.y + 1.0) * 0.5))
            let z = sqrt(max(0, (self.columns.2.z + 1.0) * 0.5))
            return simd_float3(x, y, z) * theta
        }
        
        let factor = theta / (2.0 * sin(theta))
        let v = simd_float3(
            self.columns.1.z - self.columns.2.y,
            self.columns.2.x - self.columns.0.z,
            self.columns.0.y - self.columns.1.x
        )
        return v * factor
    }
}

public extension simd_quatf {
    /// 与微小旋转向量进行右扰动乘法 q ⊗ exp(0.5 * δθ)
    func perturbed(by deltaTheta: simd_float3) -> simd_quatf {
        let deltaQ = deltaTheta.expQuaternion()
        return simd_mul(self, deltaQ).normalized
    }
    
    /// 获取四元数旋转矩阵 (3x3)
    var rotationMatrix3x3: simd_float3x3 {
        return simd_float3x3(self)
    }

    /// 欧拉角 (Roll, Pitch, Yaw)
    var eulerAngles: simd_float3 {
        let q = self.vector
        let ysqr = q.y * q.y

        // Roll (x-axis rotation)
        let t0 = +2.0 * (q.w * q.x + q.y * q.z)
        let t1 = +1.0 - 2.0 * (q.x * q.x + ysqr)
        let roll = atan2(t0, t1)

        // Pitch (y-axis rotation)
        var t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = max(-1.0, min(1.0, t2))
        let pitch = asin(t2)

        // Yaw (z-axis rotation)
        let t3 = +2.0 * (q.w * q.z + q.x * q.y)
        let t4 = +1.0 - 2.0 * (ysqr + q.z * q.z)
        let yaw = atan2(t3, t4)

        return simd_float3(roll, pitch, yaw)
    }
}

public extension simd_float4x4 {
    /// 构造刚体变换矩阵 T = [R, t; 0, 1]
    init(rotation: simd_float3x3, translation: simd_float3) {
        self.init(
            simd_float4(rotation.columns.0, 0),
            simd_float4(rotation.columns.1, 0),
            simd_float4(rotation.columns.2, 0),
            simd_float4(translation, 1)
        )
    }
    
    var rotation3x3: simd_float3x3 {
        return simd_float3x3(
            simd_float3(columns.0.x, columns.0.y, columns.0.z),
            simd_float3(columns.1.x, columns.1.y, columns.1.z),
            simd_float3(columns.2.x, columns.2.y, columns.2.z)
        )
    }
    
    var translation3: simd_float3 {
        return simd_float3(columns.3.x, columns.3.y, columns.3.z)
    }
}
