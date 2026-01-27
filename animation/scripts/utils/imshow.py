"""
车辆图像显示工具
用于在matplotlib图上显示旋转和平移后的车辆图像
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D
import matplotlib.image as mpimg


def imshow(image_data, vehicle_state, vehicle_para):
    """
    在matplotlib图上显示车辆图像
    
    参数:
        image_data: numpy数组，形状为 (rows, cols, colors)，类型为 float32
                    图像数据，值范围通常在 [0, 1] 或 [0, 255]
        vehicle_state: numpy数组，包含 [x, y, yaw]
                      车辆位置和航向角
        vehicle_para: numpy数组，包含 [length, width] 或类似参数
                      车辆尺寸参数
    """
    try:
        # 确保 image_data 是正确的形状
        if len(image_data.shape) != 3:
            return
        
        rows, cols, colors = image_data.shape
        
        # 提取车辆状态
        x = float(vehicle_state[0])
        y = float(vehicle_state[1])
        yaw = float(vehicle_state[2])
        
        # 提取车辆参数（通常是长度和宽度）
        if len(vehicle_para) >= 2:
            length = float(vehicle_para[0])
            width = float(vehicle_para[1])
        else:
            # 默认值
            length = 4.5
            width = 2.0
        
        # 确保图像数据在正确的范围内
        img = image_data.copy()
        if img.max() > 1.0:
            img = img / 255.0
        
        # 获取当前axes
        ax = plt.gca()
        
        # 计算图像显示范围（以车辆中心为原点）
        # 假设图像已经是以车辆为中心的
        extent_x = [-length/2, length/2]
        extent_y = [-width/2, width/2]
        
        # 创建仿射变换：先旋转再平移
        transform = Affine2D().rotate(yaw).translate(x, y) + ax.transData
        
        # 显示图像
        ax.imshow(img, 
                 extent=[extent_x[0], extent_x[1], extent_y[0], extent_y[1]],
                 transform=transform,
                 origin='upper',
                 interpolation='bilinear',
                 alpha=1.0,
                 zorder=10)
        
    except Exception as e:
        # 如果出错，静默失败（C++代码会处理错误）
        import sys
        print(f"Error in imshow: {e}", file=sys.stderr)
        pass
