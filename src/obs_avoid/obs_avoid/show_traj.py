#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

def read_file_to_np(file_path):
    """
    读取文件并返回数组，文件中的每一行表示一个数组，元素之间以空格分割。

    参数:
    file_path (str): 文件路径

    返回:
    list: 包含文件中所有数组的列表
    """
    try:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            # 将每行按空格分割，并将每个元素转换为浮点数
            array = [[float(num) for num in line.split()] for line in lines]
            return np.array(array,dtype=np.float32)
    except FileNotFoundError:
        print(f"文件 {file_path} 未找到。")
        return np.array([],dtype=np.float32)
    except Exception as e:
        print(f"读取文件时发生错误: {e}")
        return np.array([],dtype=np.float32)
    
def display_np(obs_path,traj_path):
    
    obs = read_file_to_np(obs_path)
    
    traj = read_file_to_np(traj_path)
    
    # 将obs的前两维作为x和y坐标，第四维作为半径，绘制成散点图
    plt.scatter(obs[:,0], obs[:,1], s=obs[:,3]*2000, c='r')
    
    # 将traj的前两维作为x和y坐标，绘制成折线图
    plt.plot(traj[:,0], traj[:,1], c='b')
    
    plt.show()
    
if __name__ == '__main__':
    
    obs_path = get_package_share_directory('obs_avoid')+'/config/obs.txt'
    traj_path = get_package_share_directory('obs_avoid')+'/config/traj.txt'

    display_np(obs_path,traj_path)