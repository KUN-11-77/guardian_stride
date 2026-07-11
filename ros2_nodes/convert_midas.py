"""将 MiDaS ONNX 转为 OpenVINO IR → NPU 编译"""
import openvino as ov
import numpy as np
import sys, os

ONNX_PATH = sys.argv[1] if len(sys.argv) > 1 else "/home/guardian/midas_small.onnx"
IR_PATH = ONNX_PATH.replace(".onnx", "_ir.xml")

core = ov.Core()
print(f"OpenVINO {ov.__version__}")
print(f"可用设备: {core.available_devices}")

# 1. 读取 ONNX
print(f"\n读取 {ONNX_PATH} ({os.path.getsize(ONNX_PATH)/1024/1024:.1f}MB)...")
model = core.read_model(ONNX_PATH)

# 2. 检查原始 shape
inp = model.input(0)
print(f"  原始输入: {inp.any_name} shape={inp.partial_shape}")

# 3. 静态 shape (NPU 必需)
model.reshape({inp.any_name: [1, 3, 256, 256] if inp.partial_shape[2].is_dynamic else inp.partial_shape})
print(f"  静态shape: {model.input(0).partial_shape}")
print(f"  输出: {model.output(0).partial_shape}")

# 4. 保存 IR
ov.serialize(model, IR_PATH)
print(f"\nIR 已保存: {IR_PATH} ({os.path.getsize(IR_PATH)/1024:.0f}KB)")

# 5. NPU 编译测试
print(f"\n编译 NPU ...")
try:
    compiled = core.compile_model(model, "NPU")
    print("  ✅ NPU 编译成功!")
    
    # 推理测试
    test_in = np.random.randn(1, 3, 256, 256).astype(np.float32)
    infer = compiled.create_infer_request()
    infer.infer({model.input(0).any_name: test_in})
    out = infer.get_output_tensor(0).data
    print(f"  ✅ NPU 推理成功! 输出: {out.shape}")
    print(f"  深度范围: [{out.min():.3f}, {out.max():.3f}]")
    print(f"\n🎉 真实 MiDaS 权重 NPU 部署完成!")
except Exception as e:
    print(f"  ❌ NPU 失败: {str(e)[:300]}")
    print("  尝试 CPU 编译...")
    compiled = core.compile_model(model, "CPU")
    print("  ✅ CPU 编译成功 (可回退)")
