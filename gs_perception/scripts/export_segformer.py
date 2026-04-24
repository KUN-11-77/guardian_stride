#!/usr/bin/env python3
"""
将 HuggingFace SegFormer-B0 导出为 OpenVINO INT8 IR
使用方法：
  cd ~/ros2_ws/src/guardian_stride/gs_perception
  python3 scripts/export_segformer.py
输出：
  models/segformer_b0_int8.xml
  models/segformer_b0_int8.bin
"""

import os
import numpy as np
from optimum.intel import OVModelForSemanticSegmentation
import nncf
import openvino as ov

MODEL_ID = "nvidia/segformer-b0-finetuned-cityscapes-1024-1024"
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "..", "models")

def main():
    print("Step 1: 导出 FP32 OpenVINO IR...")
    model = OVModelForSemanticSegmentation.from_pretrained(MODEL_ID, export=True)
    fp32_dir = os.path.join(OUTPUT_DIR, "segformer_b0_fp32")
    model.save_pretrained(fp32_dir)
    print(f"  FP32 模型保存至 {fp32_dir}")

    print("Step 2: INT8 PTQ 量化...")
    core = ov.Core()
    ov_model = core.read_model(os.path.join(fp32_dir, "openvino_model.xml"))

    def calibration_dataset():
        for _ in range(100):
            yield {"pixel_values": np.random.rand(1, 3, 480, 640).astype(np.float32)}

    quantized_model = nncf.quantize(
        ov_model,
        calibration_dataset=nncf.Dataset(calibration_dataset()),
        preset=nncf.QuantizationPreset.PERFORMANCE,
    )

    int8_path = os.path.join(OUTPUT_DIR, "segformer_b0_int8.xml")
    ov.save_model(quantized_model, int8_path)
    print(f"  INT8 模型保存至 {int8_path}")
    print("完成！")

if __name__ == "__main__":
    main()
