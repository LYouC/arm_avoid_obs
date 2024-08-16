import numpy as np
from PIL import Image
import numpy as np
import cv2
import onnxruntime as rt
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import pycocotools.mask as mask_util
import cv2

import math

CLASSES = []

class MaskRCNNModel:
    def __init__(self, model_path):
        self.session = rt.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def get_label(self,idx):
        return CLASSES[idx]

    def pre_process(self, image):
        # Resize
        ratio = 800.0 / min(image.shape[0], image.shape[1])
        image = cv2.resize(image, (int(ratio * image.shape[1]), int(ratio * image.shape[0])))

        # Convert to BGR
        image = np.array(image)[:, :, [2, 1, 0]].astype('float32')

        # HWC -> CHW
        image = np.transpose(image, [2, 0, 1])

        # Normalize
        mean_vec = np.array([102.9801, 115.9465, 122.7717])
        for i in range(image.shape[0]):
            image[i, :, :] = image[i, :, :] - mean_vec[i]

        # Pad to be divisible of 32
        padded_h = int(math.ceil(image.shape[1] / 32) * 32)
        padded_w = int(math.ceil(image.shape[2] / 32) * 32)

        padded_image = np.zeros((3, padded_h, padded_w), dtype=np.float32)
        padded_image[:, :image.shape[1], :image.shape[2]] = image
        image = padded_image

        return image

    def post_process(self, image,boxes, scores, masks,labels,  score_threshold=0.7):
        idx = np.where(scores > score_threshold)
        boxes = boxes[idx]
        labels = labels[idx]
        scores = scores[idx]
        masks = masks[idx]
        
        # Resize boxes
        ratio = 800.0/480.0
        boxes /= ratio
        
        center = []
        
        for mask, box, label, score in zip(masks, boxes, labels, scores):

            # Finding contour based on mask
            mask = mask[0, :, :, None]
            int_box = [int(i) for i in box]
            mask = cv2.resize(mask, (int_box[2]-int_box[0]+1, int_box[3]-int_box[1]+1))
            mask = mask > 0.5
            im_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
            x_0 = max(int_box[0], 0)
            x_1 = min(int_box[2] + 1, image.shape[1])
            y_0 = max(int_box[1], 0)
            y_1 = min(int_box[3] + 1, image.shape[0])
            mask_y_0 = max(y_0 - box[1], 0)
            mask_y_1 = mask_y_0 + y_1 - y_0
            mask_x_0 = max(x_0 - box[0], 0)
            mask_x_1 = mask_x_0 + x_1 - x_0
            im_mask[y_0:y_1, x_0:x_1] = mask[
                mask_y_0 : mask_y_1, mask_x_0 : mask_x_1
            ]
            im_mask = im_mask[:, :, None]
            
            bottom_x = 800
            bottom_y = 0
            top_x = 0
            top_y = 0
            count = 0
            for i in range(y_0, int((y_1+y_0)/2)):
                for j in range(x_0+(x_1-x_0)//5, x_1-(x_1-x_0)//5):
                    if im_mask[i,j] == 1:
                        top_x += j
                        top_y += i
                        count += 1
            top_x = top_x/(count)
            top_y = top_y/(count)
            top_x = int(top_x)
            top_y = int(top_y)
            
            count = 0
            for i in range(int((y_1+y_0)/2),y_1):
                for j in range(x_0+(x_1-x_0)//5, x_1-(x_1-x_0)//5):
                    if im_mask[i,j] == 1:
                        bottom_x += j
                        bottom_y += i
                        count += 1
            bottom_x = bottom_x/(count)
            bottom_y = bottom_y/(count)
            bottom_x = int(bottom_x)
            bottom_y = int(bottom_y)
            
            h = y_1 - y_0
            w = x_1 - x_0
            
            
            center.append([
                (x_1+x_0)//2,(y_1+y_0)//2,
                # top_x,top_y,
                # bottom_x,bottom_y,
                w,h,
                label,score
            ])
            
        return center

    def predict(self, image:np.ndarray):
        # must be bgr8 format
        input_image = self.pre_process(image.copy())
        result = self.session.run(self.output_names, {self.input_name: input_image})
        boxes = result[0]
        labels = result[1]
        scores = result[2]
        masks = result[3]
        return self.post_process(image, boxes, scores, masks,labels)
    
    

if __name__ == '__main__':
    model = MaskRCNNModel('/home/lyouc/ros2_project/arm_avoid_obs/src/obs_avoid/obs_avoid/MaskRCNN-12-int8/MaskRCNN-12-int8.onnx')
    res = model.predict(cv2.imread('/home/lyouc/ros2_project/arm_avoid_obs/src/obs_avoid/obs_avoid/MaskRCNN-12-int8/abc.jpg'))
    print(res)

text = '''
background
person
bicycle
car
motorcycle
airplane
bus
train
truck
boat
traffic light
fire hydrant
stop sign
parking meter
bench
bird
cat
dog
horse
sheep
cow
elephant
bear
zebra
giraffe
backpack
umbrella
handbag
tie
suitcase
frisbee
skis
snowboard
sports ball
kite
baseball bat
baseball glove
skateboard
surfboard
tennis racket
bottle
wine glass
cup
fork
knife
spoon
bowl
banana
apple
sandwich
orange
broccoli
carrot
hot dog
pizza
donut
cake
chair
couch
potted plant
bed
dining table
toilet
tv
laptop
mouse
remote
keyboard
cell phone
microwave
oven
toaster
sink
refrigerator
book
clock
vase
scissors
teddy bear
hair drier
toothbrush
'''
CLASSES = [t.strip() for t in text.split('\n') if len(t.strip())]

