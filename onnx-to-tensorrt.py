#!/usr/bin/env python3
import os
import sys
import argparse
import logging
import time
import tensorrt as trt
import numpy as np
# from cuda import cudart

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def build_engine_from_onnx(onnx_file_path, engine_file_path, precision="fp16", workspace_size=1, 
                           verbose=False, dynamic_batch=False, max_batch_size=1):
    """
    Build a TensorRT engine from an ONNX model
    
    Args:
        onnx_file_path: Path to the ONNX model
        engine_file_path: Path where the TensorRT engine will be saved
        precision: Precision mode ("fp32", "fp16", or "int8")
        workspace_size: Maximum workspace size in GB
        verbose: Enable verbose logging
        dynamic_batch: Enable dynamic batch size
        max_batch_size: Maximum batch size if dynamic_batch is True
    """
    logger.info(f"Converting ONNX model: {onnx_file_path}")
    logger.info(f"Target engine file: {engine_file_path}")
    logger.info(f"Precision: {precision}")
    
    # Initialize TensorRT builder and configs
    TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE if verbose else trt.Logger.INFO)
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_size * (1 << 30))  # Convert to bytes
    
    # Set up precision
    if precision.lower() == "fp16" and builder.platform_has_fast_fp16:
        logger.info("Using FP16 precision")
        config.set_flag(trt.BuilderFlag.FP16)
    elif precision.lower() == "int8" and builder.platform_has_fast_int8:
        logger.info("Using INT8 precision")
        config.set_flag(trt.BuilderFlag.INT8)
        # Note: INT8 requires calibration, which is not implemented in this script
        logger.warning("INT8 mode requires calibration - using empty calibrator")
        # You would need to implement a calibrator here for proper INT8 quantization
    else:
        logger.info("Using FP32 precision")
    
    # Parse ONNX model
    parser = trt.OnnxParser(network, TRT_LOGGER)
    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            for error in range(parser.num_errors):
                logger.error(f"ONNX parse error: {parser.get_error(error)}")
            raise RuntimeError("Failed to parse ONNX model")
    
    # Configure dynamic batch size if requested
    if dynamic_batch:
        logger.info(f"Using dynamic batch size with max batch size: {max_batch_size}")
        profile = builder.create_optimization_profile()
        input_name = network.get_input(0).name
        input_shape = network.get_input(0).shape
        
        # Set dynamic batch dimension
        min_shape = input_shape.copy()
        min_shape[0] = 1
        opt_shape = input_shape.copy()
        opt_shape[0] = max(1, max_batch_size // 2)  # Optimal is half of max by default
        max_shape = input_shape.copy()
        max_shape[0] = max_batch_size
        
        profile.set_shape(input_name, min_shape, opt_shape, max_shape)
        config.add_optimization_profile(profile)
    
    # Build and save engine
    logger.info("Building TensorRT engine... this may take a while")
    start_time = time.time()
    engine = builder.build_engine(network, config)
    build_time = time.time() - start_time
    
    if engine is None:
        raise RuntimeError("Failed to build TensorRT engine")
    
    logger.info(f"Engine built successfully in {build_time:.2f} seconds")
    
    # Serialize engine to file
    with open(engine_file_path, 'wb') as f:
        f.write(engine.serialize())
    
    logger.info(f"Engine saved to {engine_file_path}")
    return True

def main():
    parser = argparse.ArgumentParser(description="Convert ONNX model to TensorRT engine on Jetson Orin Nano")
    parser.add_argument("--onnx", required=True, help="Path to ONNX model file")
    parser.add_argument("--output", required=True, help="Path to output TensorRT engine file")
    parser.add_argument("--precision", choices=["fp32", "fp16", "int8"], default="fp16", 
                        help="Precision mode (default: fp16)")
    parser.add_argument("--workspace", type=int, default=1, help="Workspace size in GB (default: 1)")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    parser.add_argument("--dynamic_batch", action="store_true", help="Enable dynamic batch size")
    parser.add_argument("--max_batch_size", type=int, default=1, help="Maximum batch size for dynamic batching")
    
    args = parser.parse_args()
    
    # Check if ONNX file exists
    if not os.path.isfile(args.onnx):
        logger.error(f"ONNX file not found: {args.onnx}")
        return 1
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    try:
        build_engine_from_onnx(
            args.onnx, 
            args.output, 
            args.precision, 
            args.workspace, 
            args.verbose,
            args.dynamic_batch,
            args.max_batch_size
        )
        logger.info("Conversion completed successfully")
        return 0
    except Exception as e:
        logger.error(f"Error during conversion: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
