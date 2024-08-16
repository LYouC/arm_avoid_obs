#!/usr/bin/env python3
# 发布yolo检测结果

import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.timer
from sensor_msgs.msg import Image 
import numpy as np
import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from obs_avoid.model_utils import MaskRCNNModel
from std_msgs.msg import Float32MultiArray,Bool
import time
import math


class TestNode(Node):
    def __init__(self):
        super().__init__('model_node')
        
        img_sub = Subscriber(self,Image, '/camera/image_raw')
        depth_sub = Subscriber(self,Image, '/camera/depth/image_raw')
        self.enable_sub = self.create_subscription(Bool, '/enable_model', self.enable_model_callback, 1)
        self.ats = ApproximateTimeSynchronizer([img_sub, depth_sub], 2, 0.1)
        self.ats.registerCallback(self.image_callback)
        self.depth_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.depth_info_callback, 1)
        self.pub = self.create_publisher(Float32MultiArray, '/detect_result', 1)
        
        self.dist_coeffs = None
        self.P_mat = None
        self.K_mat = None
        
        self.cv_bridge = CvBridge()
        self.img = None
        self.depth = None
        
        self.model = MaskRCNNModel('./src/obs_avoid/obs_avoid/MaskRCNN-12-int8/MaskRCNN-12-int8.onnx')
        self.pre_res = None
        self.pre_time = 0
        
        self.can_info = False
        
        self.need_model = True
    
    def enable_model_callback(self, msg:Bool):
        self.need_model = msg.data
        self.get_logger().info(f'need_model: {self.need_model}')
    
    def info(self,msg):
        if self.can_info:
            self.get_logger().info(msg)
    
    def image_callback(self, img_msg:Image, depth_msg:Image):
        self.img = self.cv_bridge.imgmsg_to_cv2(img_msg,'bgr8')
        self.depth = self.cv_bridge.imgmsg_to_cv2(depth_msg,'32FC1')
        
        
        # cv2.imwrite('abc.jpg', self.img)
        # self.get_logger().info('Image saved as abc.jpg')
        
        #depth_img = np.clip(self.depth, 0, 1)
        #depth_array_norm = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX)
        # self.get_logger().info('Depth image saved as depth_image.jpg')
        # cv2.imwrite('depth_image.jpg', depth_array_norm)
        
        res = self.model.predict(self.img)
        if not self.need_model or res is None or self.K_mat is None:
            return
        new_res = sorted(res, key=lambda x: x[0])
        
        # 判断是否需要重新发布
        out_time = time.time() - self.pre_time > 8.0
        new_res = [item for item in new_res if item[4] == 40]
        need_repub = False
        if self.pre_res is None or len(self.pre_res)!= len(new_res):
            need_repub = True
        else:
            for i in range(len(new_res)):
                new_item = new_res[i]
                pre_item = self.pre_res[i]
                tolerance = 10
                for i in range(4):
                    if abs(new_item[i]-pre_item[i]) > tolerance:
                        need_repub = True
                        break
                if new_item[5]!= pre_item[5]:
                    need_repub = True
                if need_repub:
                    break
        self.pre_res = new_res  
        if not need_repub and not out_time:
            self.info('No need to republish')
            return
        
        # 将新的结果进行发布
        self.pre_time = time.time()
        msg = Float32MultiArray()   # 第一位代表数据是否有更新
        if need_repub:
            self.get_logger().warn('data change,Republish')
            msg.data.append(1)
        else:
            self.get_logger().warn('time out,Republish')
            msg.data.append(0)

        for item in new_res:
            px,py,w,h,label,score = item
            if label == 0 and w*h < 1600 and label != 40:
                continue
            # 获取对应位置深度信息
            pos0 = self.get_pos(int(px),int(py))
            left_pos = self.get_pos(int(px-w/3),int(py))
            right_pos = self.get_pos(int(px+w/3),int(py))
            up_pos = self.get_pos(int(px),int(py-h/3))
            down_pos = self.get_pos(int(px),int(py+h/3))
            if pos0 is None or left_pos is None or right_pos is None or up_pos is None or down_pos is None:
                img = self.img.copy()
                cv2.circle(img, (int(px),int(py)), 5, (0,0,255), -1)
                cv2.circle(img, (int(px-w/3),int(py)), 5, (0,255,255), -1)
                cv2.circle(img, (int(px),int(py-h/3)), 5, (255,0,255), -1)
                cv2.circle(img, (int(px+w/3),int(py)), 5, (0,255,255), -1)
                cv2.circle(img, (int(px),int(py+h/3)), 5, (255,0,255), -1)
                cv2.imwrite(f'get_pos_failed_{w:.2f}_{h:.2f}.jpg', img)
                self.get_logger().warn('get pos failed')
                continue
            
            radius = math.fabs(right_pos[0]-left_pos[0])*0.75
            height = math.fabs(up_pos[1]-down_pos[1])*1.5
            # 圆柱体的中心点
            z = pos0[2] + radius/2
            
            msg.data.extend([pos0[0],pos0[1],z,radius,height,label,score])
        self.pub.publish(msg)
        
        # wait 1 second
        time.sleep(1)
    
    def depth_info_callback(self,msg:CameraInfo):
        # self.depth_sub.
        self.K_mat = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.P_mat = np.array(msg.p).reshape(3, 4)
        self.get_logger().info('Received camera info')
        self.destroy_subscription(self.depth_info_sub)
        pass
    
    def get_pos(self,px,py):
        if self.img is None or self.depth is None or self.K_mat is None:
            return None
        depth = self.depth[py,px]
        if depth < 0.1 or depth > 5:
            return None
        ppg = np.array([[px,py]],dtype=np.float32)
        undistorted_coords = cv2.undistortPoints(ppg, self.K_mat, self.dist_coeffs, P=self.P_mat)[0]
        # 相机坐标
        x = (undistorted_coords[0, 0]-self.K_mat[0,2])*depth/self.K_mat[0,0]
        y = (undistorted_coords[0, 1]-self.K_mat[1,2])*depth/self.K_mat[1,1]
        z = depth
        return x,y,z
    
    
def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()