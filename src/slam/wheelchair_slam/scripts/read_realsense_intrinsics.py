import pyrealsense2 as rs
 
 
def get_intrinsics():
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    config = rs.config()
    matrices = []
    # Configure the pipeline to stream the depth stream
    # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # names = ["color", "depth", "infra"]
    stream_type = [rs.stream.color, rs.stream.depth, rs.stream.infrared]
    stream_type = [rs.stream.depth]

    for t in stream_type:
        print(t)
        config.enable_stream(t, 640, 480, rs.format.z16, 30)
        # Start streaming
        pipeline.start(config)
    
        # Get frames from the camera to get the intrinsic parameters
        profile = pipeline.get_active_profile()
        depth_stream = profile.get_stream(t)
        intr = depth_stream.as_video_stream_profile().get_intrinsics()
        # print(type(intr))
        # Stop the pipeline
        pipeline.stop()
 
        # Intrinsics
        intrinsics_matrix = [
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ]
        matrices.append(intrinsics_matrix)
    return matrices
 
 
if __name__ == "__main__":
    matrices = get_intrinsics()
    print("Intrinsic matrix for RealSense D435 depth camera:")
    for intrinsics_matrix in matrices:
        for row in intrinsics_matrix:
            print(row)