#!/usr/bin/env python3
import os
import sys
import math
import torch
import torch.nn.functional as F
import numpy as np
import cv2


# # Path manipulation for ROS compatibility
# pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# if pkg_path not in sys.path:
#     sys.path.insert(0, pkg_path)
#
# # Import config directly from package
# from config.bisenetv1_SUPS import cfg  # Direct import without lib

# # Add package root to path
# pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# if pkg_root not in sys.path:
#     sys.path.insert(0, pkg_root)

# Add both package root and lib directory
pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
lib_path = os.path.join(pkg_root, 'lib')

for path in [pkg_root, lib_path]:
    if path not in sys.path:
        sys.path.insert(0, path)

import lib.data.transform_cv2 as T
from lib.models import model_factory
from config import set_cfg_from_file

# # Now use ABSOLUTE imports
# from config.bisenetv1_SUPS import cfg  # Direct import

class SegmentationProcessor:
    def __init__(self, config_path, weight_path):
        """Initialize segmentation model with config and weights"""
        self.cfg = set_cfg_from_file(config_path)
        self.palette = np.random.randint(0, 256, (256, 3), dtype=np.uint8)

        # Device configuration
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # Model setup
        self.net = model_factory[self.cfg.model_type](self.cfg.n_cats, aux_mode='eval')
        self.net.load_state_dict(
            torch.load(weight_path, map_location=self.device),
            strict=False
        )
        self.net.eval()
        self.net.to(self.device)

        # Image transformation
        self.to_tensor = T.ToTensor(
            mean=(0.3257, 0.3690, 0.3223),
            std=(0.2112, 0.2148, 0.2115),
        )

    def process(self, input_image_np):
        """Process BGR numpy array and return segmentation result"""
        try:
            # Convert and preprocess
            im = cv2.cvtColor(input_image_np, cv2.COLOR_BGR2RGB)
            im = self.to_tensor(dict(im=im, lb=None))['im'].unsqueeze(0).to(self.device)

            # Handle dimensions
            org_size = im.size()[2:]
            new_size = [math.ceil(el / 32) * 32 for el in im.size()[2:]]

            # Inference
            im = F.interpolate(im, size=new_size, align_corners=False, mode='bilinear')
            out = self.net(im)[0]
            out = F.interpolate(out, size=org_size, align_corners=False, mode='bilinear')
            out = out.argmax(dim=1)

            # Convert to output
            out = out.squeeze().detach().cpu().numpy()
            return self.palette[out]

        except Exception as e:
            print(f"[Segmentation] Processing error: {str(e)}")
            raise