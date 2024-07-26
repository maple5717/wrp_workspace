import pyrealsense2 as rs
 
 
def get_intrinsics():
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    config = rs.config()
 
    # Configure the pipeline to stream the depth stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # Start streaming
    pipeline.start(config)
 
    # Get frames from the camera to get the intrinsic parameters
    profile = pipeline.get_active_profile()
    depth_stream = profile.get_stream(rs.stream.color)
    intr = depth_stream.as_video_stream_profile().get_intrinsics()
    print(type(intr))
    # Stop the pipeline
    pipeline.stop()
 
    # Intrinsics
    intrinsics_matrix = [
        [intr.fx, 0, intr.ppx],
        [0, intr.fy, intr.ppy],
        [0, 0, 1]
    ]
 
    return intrinsics_matrix
 
 
if __name__ == "__main__":
    intrinsics_matrix = get_intrinsics()
    print("Intrinsic matrix for RealSense D435 depth camera:")
    for row in intrinsics_matrix:
        print(row)