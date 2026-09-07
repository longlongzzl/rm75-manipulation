#!/usr/bin/env python3
"""Bounded camera-only RGB-D recording for RRTrack offline validation.

No robot SDK, motion client, perception model, or guessed extrinsic is used.
"""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--frames', type=int, default=30)
    args = parser.parse_args()
    if not 1 <= args.frames <= 300:
        parser.error('--frames must be 1..300')
    import cv2
    import numpy as np
    import pyrealsense2 as rs

    args.output.mkdir(parents=True, exist_ok=False)
    for name in ('rgb', 'depth'):
        (args.output / name).mkdir()
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    profile = pipeline.start(config)
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        for _ in range(30):
            pipeline.wait_for_frames(5000)
        with (args.output / 'capture.jsonl').open('x') as log:
            for index in range(args.frames):
                frames = align.process(pipeline.wait_for_frames(5000))
                rgb, depth = frames.get_color_frame(), frames.get_depth_frame()
                if not rgb or not depth:
                    raise RuntimeError('missing aligned RGB-D frame')
                intr = rgb.profile.as_video_stream_profile().intrinsics
                K = [intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1]
                camera = dict(cam_K=K, serial=args.serial, depth_scale_m=scale,
                              width=intr.width, height=intr.height,
                              distortion_model=str(intr.model), distortion=intr.coeffs)
                bgr = np.asanyarray(rgb.get_data())
                depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * scale
                if not cv2.imwrite(str(args.output / 'rgb' / f'{index:06d}.png'), bgr):
                    raise RuntimeError('RGB write failed')
                np.save(args.output / 'depth' / f'{index:06d}.npy', depth_m)
                if index == 0:
                    (args.output / 'camera.json').write_text(json.dumps(camera, indent=2))
                log.write(json.dumps(dict(index=index, timestamp_s=time.time(),
                    device_timestamp_ms=rgb.get_timestamp(), frame_number=rgb.get_frame_number(),
                    valid_depth_pixels=int(np.count_nonzero(depth_m)))) + '\n')
                log.flush()
                time.sleep(0.2)
    finally:
        pipeline.stop()
    print(json.dumps(dict(status='captured', frames=args.frames, serial=args.serial,
                          output=str(args.output), robot_connected=False)))


if __name__ == '__main__':
    main()
