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

classes = [line.rstrip('\n') for line in open('./test_data_set_0/coco_classes.txt')]


def other_preprocess(image):
    # Resize
    ratio = 800.0 / min(image.size[0], image.size[1])
    image = image.resize((int(ratio * image.size[0]), int(ratio * image.size[1])), Image.BILINEAR)

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

def simple_preprocess(image):
    """simple preprocess the image for MaskRCNN model.for the image size is h=480,w=640

    Usages:
        image = cv2.imread('image.jpg') # 640*480*3,uint8,BGR
        input = simple_preprocess(image)

    Returns:
        np.ndarray: the preprocessed image (3,800,1088) float32 BGR
    """
    
    image = cv2.resize(image, (1088, 800), interpolation=cv2.INTER_LINEAR)
    
    # normalize
    image = image.astype(np.float32)
    mean_vec = np.array([102.9801, 115.9465, 122.7717])
    
    # HWC -> CHW
    image = np.transpose(image, [2, 0, 1]).astype(np.float32)

    # temp = image.astype(np.uint8)


    return image

def preprocess(image):
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

def display_objdetect_image_cv2(image, boxes, labels, scores, masks, score_threshold=0.7):
    
    colors = plt.get_cmap("tab20")(np.linspace(0, 1, 20))
    colors = colors[:, :3]
    
    idx = np.where(scores > score_threshold)
    boxes = boxes[idx]
    labels = labels[idx]
    scores = scores[idx]
    masks = masks[idx]
    
    # Resize boxes
    ratio = 800.0 / min(image.shape[0], image.shape[1])
    boxes /= ratio
    
    image = np.array(image)

    for mask, box, label, score in zip(masks, boxes, labels, scores):

        # Finding contour based on mask
        #mask:(1,28,28)
        mask = mask[0, :, :, None]  #(28,28,1)
        int_box = [int(i) for i in box]
        mask = cv2.resize(mask, (int_box[2]-int_box[0]+1,int_box[3]-int_box[1]+1)) #box区域大小
        mask = mask > 0.5
        im_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8) #image大小的全0mask
        x_0 = max(int_box[0], 0)
        x_1 = min(int_box[2] + 1, image.shape[1])
        y_0 = max(int_box[1], 0)
        y_1 = min(int_box[3] + 1, image.shape[0])
        mask_y_0 = max(y_0 - int_box[1], 0)
        mask_y_1 = max(0, mask_y_0 + y_1 - y_0)
        mask_x_0 = max(x_0 - int_box[0], 0)
        mask_x_1 = max(0, mask_x_0 + x_1 - x_0)

        im_mask[y_0:y_1, x_0:x_1] = mask[  #box区域填上mask，（480,640,1)
            mask_y_0 : mask_y_1, mask_x_0 : mask_x_1
        ]
        
        center_x = x_0 + (x_1 - x_0) // 2
        center_y = y_0 + (y_1 - y_0) // 2
        
        bottom_x = 800
        bottom_y = 0
        top_x = 0
        top_y = 0
        count = 0
        for i in range(y_0, int((y_1+y_0)/2)):
            for j in range(x_0+(x_1-x_0)//6, x_1-(x_1-x_0)//6):
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
            for j in range(x_0+(x_1-x_0)//6, x_1-(x_1-x_0)//6):
                if im_mask[i,j] == 1:
                    bottom_x += j
                    bottom_y += i
                    count += 1
        bottom_x = bottom_x/(count)
        bottom_y = bottom_y/(count)
        bottom_x = int(bottom_x)
        bottom_y = int(bottom_y)
        

        im_mask = im_mask[:, :, None] #(480,640,1)
        mask3 = image.copy()  #填上颜色的mask

        color = (colors[label%len(colors)]*255).astype(np.uint8).tolist()
        mask3[im_mask[:,:,0]>0] = color
        image = cv2.addWeighted(image, 0.5, mask3, 0.5, 0)
		
        text = "{}:{:.1f}%".format(classes[label], score * 100)
        txt_color = (0, 0, 0) if np.mean(colors[label%len(colors)]) > 0.5 else (255, 255, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        txt_size = cv2.getTextSize(text, font, 0.5, 2)[0]
        #画box框
        cv2.rectangle(image, (x_0, y_0), (x_1, y_1), color, 1)
        cv2.circle(image,(bottom_x,bottom_y),5,(255,0,0),3)
        cv2.circle(image,(top_x,top_y),5,(0,255,0),3)
        #标签区域框
        cv2.rectangle(
            image,
            (x_0, y_0 - txt_size[1] - 1),
            (x_0 + txt_size[0] + txt_size[1], y_0 - 1),
            color,
            -1,
        )
        #填上标签
        cv2.putText(image, text, (x_0, y_0 - 1), font, 0.5, txt_color, thickness=1)
    cv2.imshow("image", image)
    cv2.waitKey(0)

def display_objdetect_image(image, boxes, labels, scores, masks, score_threshold=0.7):
    
    idx = np.where(scores > score_threshold)
    boxes = boxes[idx]
    labels = labels[idx]
    scores = scores[idx]
    masks = masks[idx]
    
    # Resize boxes
    ratio = 800.0 / min(image.shape[1], image.shape[0])
    boxes /= ratio

    _, ax = plt.subplots(1, figsize=(12,9))
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = np.array(image)
    
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

        # OpenCV version 4.x
        contours, hierarchy = cv2.findContours(
            im_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        image = cv2.drawContours(image, contours, -1, 25, 3)

        rect = patches.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], linewidth=1, edgecolor='b', facecolor='none')
        ax.annotate(classes[label] + ':' + str(np.round(score, 2)), (box[0], box[1]), color='w', fontsize=12)
        ax.add_patch(rect)
    ax.imshow(image)
    plt.show()

def post_process(image,boxes, scores, masks, image_shape, score_threshold=0.7):
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
        
        w = y_1 - y_0
        h = x_1 - x_0
        
        
        center.append([top_x,top_y,bottom_x,bottom_y,w,h.label,score])
        
    return center
        
    

def test_onnx():
    image_file = './abc.jpg'
    img = cv2.imread(image_file)
    input = preprocess(img)
    # img = Image.open(image_file)
    # input = other_preprocess(img)
    
    model = rt.InferenceSession('./MaskRCNN-12-int8.onnx')
    input_name = model.get_inputs()[0].name
    output_name = [o.name for o in model.get_outputs()]
    result = model.run(output_name, {input_name: input})
    
    # 处理结果
    boxes = result[0]
    labels = result[1]
    scores = result[2]
    masks = result[3]
    
    
    display_objdetect_image_cv2(img, boxes, labels, scores, masks)
    

if __name__ == '__main__':
    test_onnx()