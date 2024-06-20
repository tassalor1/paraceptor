"""Compute depth maps for images in the input folder.
"""
import os
import glob
import torch
import utils_midas as utils_midas
import cv2
import argparse
import time

import numpy as np

from imutils.video import VideoStream
from midas.model_loader import default_models, load_model
from midas_mo.cv_processor import CVProcessor
from utils_midas import read_pfm



first_execution = True
def process(device, model, model_type, image, input_size, target_size, optimize, use_camera):
    """
    Run the inference and interpolate.

    Args:
        device (torch.device): the torch device used
        model: the model used for inference
        model_type: the type of the model
        image: the image fed into the neural network
        input_size: the size (width, height) of the neural network input (for OpenVINO)
        target_size: the size (width, height) the neural network output is interpolated to
        optimize: optimize the model to half-floats on CUDA?
        use_camera: is the camera used?

    Returns:
        the prediction
    """
    global first_execution

    if "openvino" in model_type:
        if first_execution or not use_camera:
            print(f"    Input resized to {input_size[0]}x{input_size[1]} before entering the encoder")
            first_execution = False

        sample = [np.reshape(image, (1, 3, *input_size))]
        prediction = model(sample)[model.output(0)][0]
        prediction = cv2.resize(prediction, dsize=target_size,
                                interpolation=cv2.INTER_CUBIC)
    else:
        sample = torch.from_numpy(image).to(device).unsqueeze(0)

        if optimize and device == torch.device("cuda"):
            if first_execution:
                print("  Optimization to half-floats activated. Use with caution, because models like Swin require\n"
                      "  float precision to work properly and may yield non-finite depth values to some extent for\n"
                      "  half-floats.")
            sample = sample.to(memory_format=torch.channels_last)
            sample = sample.half()

        if first_execution or not use_camera:
            height, width = sample.shape[2:]
            print(f"    Input resized to {width}x{height} before entering the encoder")
            first_execution = False

        prediction = model.forward(sample)
        prediction = (
            torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=target_size[::-1],
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )

    return prediction


def create_side_by_side(image, depth, grayscale):
    """
    Take an RGB image and depth map and place them side by side. This includes a proper normalization of the depth map
    for better visibility.

    Args:
        image: the RGB image
        depth: the depth map
        grayscale: use a grayscale colormap?

    Returns:
        the image and depth map place side by side
    """
    depth_min = depth.min()
    depth_max = depth.max()
    normalized_depth = 255 * (depth - depth_min) / (depth_max - depth_min)
    normalized_depth *= 3

    right_side = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
    if not grayscale:
        right_side = cv2.applyColorMap(np.uint8(right_side), cv2.COLORMAP_INFERNO)

    if image is None:
        return right_side
    else:
        return np.concatenate((image, right_side), axis=1)


def run(image, output_path, model_path, model_type="dpt_swin2_tiny_256", optimize=False, side=False, height=None,
        square=False, grayscale=False):
    """Run MonoDepthNN to compute depth maps.

    Args:
        image (str): image
        output_path (str): path to output folder
        model_path (str): path to saved model
        model_type (str): the model type
        optimize (bool): optimize the model to half-floats on CUDA?
        side (bool): RGB and depth side by side in output images?
        height (int): inference encoder image height
        square (bool): resize to a square resolution?
        grayscale (bool): use a grayscale colormap?
    """
    print("Initialize")

    # select device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Device: %s" % device)

    model, transform, net_w, net_h = load_model(device, model_path, model_type, optimize, height, square)


    if image is not None:
        if output_path is None:
            print("Warning: No output path specified. Images will be processed but not shown or stored anywhere.")

        # input
        original_image_rgb = utils_midas.read_image(image)  # in [0, 1]
        image_t = transform({"image": original_image_rgb})["image"]

        # compute
        with torch.no_grad():
            prediction = process(device, model, model_type, image_t, (net_w, net_h), original_image_rgb.shape[1::-1],
                                 optimize, False)

        if not os.path.exists(output_path):
            os.makedirs(output_path)

        t

    else:
        with torch.no_grad():
            fps = 1
            video = VideoStream(0).start()
            time_start = time.time()
            frame_index = 0
            while True:
                frame = video.read()
                if frame is not None:
                    original_image_rgb = np.flip(frame, 2)  # in [0, 255] (flip required to get RGB)
                    image = transform({"image": original_image_rgb/255})["image"]

                    prediction = process(device, model, model_type, image, (net_w, net_h),
                                         original_image_rgb.shape[1::-1], optimize, True)

                    original_image_bgr = np.flip(original_image_rgb, 2) if side else None
                    content = create_side_by_side(original_image_bgr, prediction, grayscale)
                    cv2.imshow('MiDaS Depth Estimation - Press Escape to close window ', content/255)

                    if output_path is not None:
                        filename = os.path.join(output_path, 'Camera' + '-' + model_type + '_' + str(frame_index))
                        cv2.imwrite(filename + ".png", content)

                    alpha = 0.1
                    if time.time()-time_start > 0:
                        fps = (1 - alpha) * fps + alpha * 1 / (time.time()-time_start)  # exponential moving average
                        time_start = time.time()
                    print(f"\rFPS: {round(fps,2)}", end="")

                    if cv2.waitKey(1) == 27:  # Escape key
                        break

                    frame_index += 1
        print()

    print("Finished")


if __name__ == "__main__":

    cv = CVProcessor()
    current_frame = cv2.imread("/home/connor/cv_drone/MiDaS/input/960.jpg")
    img, highest_conf, best_bbox = cv.process_image(current_frame=current_frame)

    best_bbox1 = list(map(int, best_bbox))
    height, width = img.shape[:2]
    best_bbox2 = [
        max(0, min(best_bbox1[0], width)),
        max(0, min(best_bbox1[1], height)),
        max(0, min(best_bbox1[2], width)),
        max(0, min(best_bbox1[3], height))
    ]
    img_sliced = img[best_bbox2[1]:best_bbox2[3], best_bbox2[0]:best_bbox2[2]]
    # if img_sliced.size > 0:
    #     cv2.imshow('Detected Frame', img_sliced)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    # else:
    #     print("Resulting image is empty. Cannot display.")

    default_models = {
        'dpt_swin2_tiny_256': '/home/connor/cv_drone/MiDaS/weights/dpt_swin2_tiny_256.pt',
    }

    # Set torch options
    torch.backends.cudnn.enabled = True
    torch.backends.cudnn.benchmark = True

    run(image=img_sliced,
        output_path='/home/connor/paraceptor/MiDaS/output/',
        model_path=default_models['dpt_swin2_tiny_256'],
        model_type='dpt_swin2_tiny_256',
        optimize=True,
        side=False,
        height=256,
        square=True,
        grayscale=False)
