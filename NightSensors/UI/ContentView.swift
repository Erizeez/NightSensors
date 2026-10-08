import SwiftUI
import MetalKit
import ARKit

public struct ContentView: View {
    @StateObject private var pipeline = SensorPipelineManager()
    @State private var currentMode: VisualizationMode = .thermal
    @State private var maxDistance: Float = 4.5
    @State private var showingDiagnostics = false
    
    public init() {}
    
    public var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            
            // 1. Metal 深度 GPU 实时渲染层 (直接绑定 pipeline.renderer)
            MetalViewContainer(renderer: pipeline.renderer)
                .ignoresSafeArea()
            
            // 2. 赛博朋克 HUD 覆盖层与交互控制
            CyberpunkHUDView(
                currentMode: $currentMode,
                maxDistance: $maxDistance,
                activeCamera: pipeline.activeCamera,
                centerDistance: pipeline.currentCenterDistance,
                fps: pipeline.currentFPS,
                isLiDARAvailable: pipeline.isLiDARAvailable,
                scalingMode: pipeline.scalingMode,
                onSwitchCamera: {
                    let nextCamera: CameraMode = pipeline.activeCamera == .rearLiDAR ? .frontTrueDepth : .rearLiDAR
                    pipeline.switchCamera(to: nextCamera)
                    if nextCamera == .frontTrueDepth {
                        currentMode = .infrared
                    } else if currentMode == .infrared {
                        currentMode = .thermal
                    }
                },
                onToggleScaling: {
                    pipeline.toggleScalingMode()
                },
                onOpenDiagnostics: { showingDiagnostics = true }
            )
        }
        .onAppear {
            pipeline.renderer.currentMode = currentMode
            pipeline.renderer.maxDistance = maxDistance
            pipeline.renderer.isFrontCamera = (pipeline.activeCamera == .frontTrueDepth)
            updateInterfaceOrientation()
            pipeline.startPipeline()
        }
        .onDisappear {
            pipeline.stopPipeline()
        }
        .onReceive(NotificationCenter.default.publisher(for: UIDevice.orientationDidChangeNotification)) { _ in
            updateInterfaceOrientation()
        }
        .onChange(of: currentMode) { _, newMode in
            pipeline.renderer.currentMode = newMode
            if newMode == .infrared {
                if pipeline.activeCamera != .frontTrueDepth {
                    pipeline.switchCamera(to: .frontTrueDepth)
                }
            } else {
                if pipeline.activeCamera != .rearLiDAR {
                    pipeline.switchCamera(to: .rearLiDAR)
                }
            }
        }
        .onChange(of: pipeline.activeCamera) { _, newCam in
            pipeline.renderer.isFrontCamera = (newCam == .frontTrueDepth)
        }
        .onChange(of: maxDistance) { _, newMax in
            pipeline.renderer.maxDistance = newMax
        }
        .sheet(isPresented: $showingDiagnostics) {
            SensorDiagnosticsView(
                diagnostics: pipeline.diagnostics,
                fps: pipeline.currentFPS,
                pointsCount: pipeline.trackedPointsCount
            )
        }
    }
    
    private func updateInterfaceOrientation() {
        if let windowScene = UIApplication.shared.connectedScenes.first(where: { $0.activationState == .foregroundActive }) as? UIWindowScene {
            pipeline.currentOrientation = windowScene.interfaceOrientation
        } else if let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene {
            pipeline.currentOrientation = windowScene.interfaceOrientation
        }
    }
}

/// SwiftUI 对 MTKView 的桥接封装
struct MetalViewContainer: UIViewRepresentable {
    let renderer: MetalRenderer
    
    func makeUIView(context: Context) -> MTKView {
        let mtkView = MTKView()
        mtkView.backgroundColor = .black
        renderer.attach(to: mtkView)
        return mtkView
    }
    
    func updateUIView(_ uiView: MTKView, context: Context) {}
}
