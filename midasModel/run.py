"""Compute depth maps for images in the input folder.
"""

import torch
import cv2
import numpy as np
from midasModel.midas.model_loader import default_models, load_model
# from cv_processor import CVProcessor



def read_image(image):
    """Read image and output RGB image (0-1).

    Args:
        path (str): path to file

    Returns:
        array: RGB image (0-1)
    """
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0
    return img

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

def run(image, model_path, model_type="dpt_swin2_tiny_256", optimize=False, height=None,
        square=False):
    """Run MonoDepthNN to compute depth maps.

    Args:
        image (str): image
        model_path (str): path to saved model
        model_type (str): the model type
        optimize (bool): optimize the model to half-floats on CUDA?
        height (int): inference encoder image height
        square (bool): resize to a square resolution?
    """
    colour = None
    width = None
    height = None

    # select device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("Device: %s" % device)
    model, transform, net_w, net_h = load_model(device, model_path, model_type, optimize, height, square)

    if image is not None:
        # input
        original_image_rgb = read_image(image=image)  # in [0, 1]
        image_t = transform({"image": original_image_rgb})["image"]

        # compute
        with torch.no_grad():
            prediction = process(device, model, model_type, image_t, (net_w, net_h), original_image_rgb.shape[1::-1],
                                 optimize, False)

        # predict distance
        if prediction is not None:
            height, width = original_image_rgb.shape[1::-1]
            shape = (height, width, 3) if colour else (height, width)
            data = np.reshape(prediction, shape)
            depth_map = np.flipud(data)
            valid_depth_values = depth_map[(depth_map > 0) & np.isfinite(depth_map)]
            if len(valid_depth_values) == 0:
                print("No valid depth values found in the depth map")
            else:
                print(f"Depth Map Statistics:")
                print(f"Min Depth: {np.min(valid_depth_values)}")
                print(f"Max Depth: {np.max(valid_depth_values)}")
                print(f"Mean Depth: {np.mean(valid_depth_values)}")
                print(f"Median Depth: {np.median(valid_depth_values)}")
                # Compute the median depth from the valid values
                median_depth = np.median(valid_depth_values)
                print(f"The distance to the detected drone is approximately {median_depth:.2f} meters")
                return median_depth


# if __name__ == "__main__":

#     cv = CVProcessor()
#     current_frame = cv2.imread("/home/connor/cv_drone/MiDaS/input/960.jpg")
#     img, highest_conf, best_bbox = cv.process_image(current_frame=current_frame)

#     best_bbox1 = list(map(int, best_bbox))
#     height, width = img.shape[:2]
#     best_bbox2 = [
#         max(0, min(best_bbox1[0], width)),
#         max(0, min(best_bbox1[1], height)),
#         max(0, min(best_bbox1[2], width)),
#         max(0, min(best_bbox1[3], height))
#     ]
#     img_sliced = img[best_bbox2[1]:best_bbox2[3], best_bbox2[0]:best_bbox2[2]]
#     # if img_sliced.size > 0:
#     #     cv2.imshow('Detected Frame', img_sliced)
#     #     cv2.waitKey(0)
#     #     cv2.destroyAllWindows()
#     # else:
#     #     print("Resulting image is empty. Cannot display.")

#     default_models = {
#         'dpt_swin2_tiny_256': '/home/connor/cv_drone/MiDaS/weights/dpt_swin2_tiny_256.pt',
#     }

#     # Set torch options
#     torch.backends.cudnn.enabled = True
#     torch.backends.cudnn.benchmark = True

#     run(image=img_sliced,
#         output_path='/home/connor/paraceptor/MiDaS/output/',
#         model_path=default_models['dpt_swin2_tiny_256'],
#         model_type='dpt_swin2_tiny_256',
#         optimize=True,
#         side=False,
#         height=256,
#         square=True,
#         grayscale=False)
