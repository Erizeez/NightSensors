import Foundation
import simd

/// 专门针对 6-DoF 刚体运动学优化的高性能 6x6 对称矩阵求解器
public struct Matrix6x6 {
    public var elements: [Float] // 36 elements in row-major
    
    public init() {
        self.elements = Array(repeating: 0, count: 36)
    }
    
    public init(diagonal: Float) {
        self.elements = Array(repeating: 0, count: 36)
        for i in 0..<6 {
            self.elements[i * 6 + i] = diagonal
        }
    }
    
    @inlinable
    public subscript(row: Int, col: Int) -> Float {
        get { elements[row * 6 + col] }
        set { elements[row * 6 + col] = newValue }
    }
    
    /// 将 6x1 雅可比向量的自外积累加至海森矩阵: H += J^T * J
    @inlinable
    public mutating func accumulateOuterProduct(_ J: (Float, Float, Float, Float, Float, Float)) {
        let j = [J.0, J.1, J.2, J.3, J.4, J.5]
        for r in 0..<6 {
            let jr = j[r]
            let rOffset = r * 6
            for c in 0..<6 {
                elements[rOffset + c] += jr * j[c]
            }
        }
    }
    
    /// Cholesky 分解 (LL^T) 求解 H * x = b，其中 H 须为对称半正定/正定矩阵
    /// 支持 Levenberg-Marquardt 对角线阻尼 (damping)
    public func solveCholesky(b: [Float], damping: Float = 1e-4) -> [Float]? {
        guard b.count == 6 else { return nil }
        var L = Array(repeating: Float(0), count: 36)
        
        // 1. LL^T 分解
        for i in 0..<6 {
            for j in 0...i {
                var sum: Float = 0
                for k in 0..<j {
                    sum += L[i * 6 + k] * L[j * 6 + k]
                }
                
                if i == j {
                    let val = elements[i * 6 + i] + damping - sum
                    if val <= 0 {
                        return nil // 非正定，奇异
                    }
                    L[i * 6 + j] = sqrt(val)
                } else {
                    let denom = L[j * 6 + j]
                    if abs(denom) < 1e-9 { return nil }
                    L[i * 6 + j] = (elements[i * 6 + j] - sum) / denom
                }
            }
        }
        
        // 2. 前向替换解 L * y = b
        var y = Array(repeating: Float(0), count: 6)
        for i in 0..<6 {
            var sum: Float = 0
            for j in 0..<i {
                sum += L[i * 6 + j] * y[j]
            }
            let denom = L[i * 6 + i]
            if abs(denom) < 1e-9 { return nil }
            y[i] = (b[i] - sum) / denom
        }
        
        // 3. 后向替换解 L^T * x = y
        var x = Array(repeating: Float(0), count: 6)
        for i in (0..<6).reversed() {
            var sum: Float = 0
            for j in (i + 1)..<6 {
                sum += L[j * 6 + i] * x[j]
            }
            let denom = L[i * 6 + i]
            if abs(denom) < 1e-9 { return nil }
            x[i] = (y[i] - sum) / denom
        }
        
        return x
    }
}
