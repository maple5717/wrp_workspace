import pyrealsense2 as rs

def get_depth_scale():
    # Create a context object
    ctx = rs.context()

    # Get the first connected device
    device = ctx.query_devices()[0]

    # Create a pipeline
    pipeline = rs.pipeline(ctx)

    # Start streaming with a basic configuration
    config = rs.config()
    config.enable_stream(rs.stream.depth)
    pipeline.start(config)

    try:
        # Get the depth sensor
        depth_sensor = device.first_depth_sensor()

        # Retrieve the depth scale
        depth_scale = depth_sensor.get_depth_scale()

        return depth_scale
    finally:
        # Stop the pipeline
        pipeline.stop()

if __name__ == "__main__":
    scale = get_depth_scale()
    print(f"Depth scale: {scale} meters per unit")

