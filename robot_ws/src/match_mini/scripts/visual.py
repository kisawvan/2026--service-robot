import matplotlib.pyplot as plt

def plot_region_3_path():
    # 设定参数：与 auto_slam.py 中的 idex==3 保持一致
    start_x = 0.0
    start_y = 0.0
    # 注意：你在 execute_special_path 中调用区域3时传了 0.8 的步长
    forward_length = 0.8  

    first_side_length3 = 0.15
    second_side_length3 = 0.12
    third_side_length3 = 0.12
    fourth_side_length3 = 0.15
    addition_side_length = 0.10

    # 预计算 Y 方向的累加偏移量 (由于全部是减法，这里都按正数累加，作图时取负数)
    y1 = first_side_length3 + addition_side_length                     # 0.25
    y2 = y1 + second_side_length3                                      # 0.37
    y3 = y2 + third_side_length3                                       # 0.49
    y4 = y3 + fourth_side_length3                                      # 0.64

    # 按照代码逻辑构建每个路径点（加入索引0作为最初的起点）
    points = [
        (start_x, start_y),                                            # 起点 P0
        (start_x - forward_length, start_y),                           # P1: 第一步 X减小
        (start_x - forward_length, start_y - y1),                      # P2: 第二步 Y减小
        (start_x, start_y - y1),                                       # P3: 第三步 X增加 (回到原X)
        (start_x, start_y - y2),                                       # P4: 第四步 Y减小
        (start_x - forward_length, start_y - y2),                      # P5: 第五步 X减小
        (start_x - forward_length, start_y - y3),                      # P6: 第六步 Y减小
        (start_x, start_y - y3),                                       # P7: 第七步 X增加
        (start_x, start_y - y4),                                       # P8: 第八步 Y减小
        (start_x - forward_length, start_y - y4),                      # P9: 第九步 X减小
    ]

    # 分离 X 和 Y 坐标以便作图
    x_coords, y_coords = zip(*points)

    # 开始绘图
    plt.figure(figsize=(8, 5))
    
    # 绘制线路与航点
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='#1f77b4', linewidth=2)
    
    # 重点标出起点和终点
    plt.plot(x_coords[0], y_coords[0], marker='o', color='green', markersize=10, label='Start (P0)')
    plt.plot(x_coords[-1], y_coords[-1], marker='X', color='red', markersize=10, label='End (P9)')

    # 添加相对坐标注释
    for i, (px, py) in enumerate(points):
        # 微调注释文本的位置，避免重叠
        offset = (5, 5) if py == 0 or px == start_x else (-25, -15)
        plt.annotate(
            f"P{i}\n({px:.2f}, {py:.2f})",
            (px, py), 
            textcoords="offset points", 
            xytext=offset,
            ha='left' if px == start_x else 'right',
            fontsize=9
        )

    plt.title("Region 3 Zig-Zag Path (Relative to Start Point)", fontsize=14)
    plt.xlabel("X Coordinate (m)", fontsize=12)
    plt.ylabel("Y Coordinate (m)", fontsize=12)
    plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    plt.axis('equal')  # 强制X轴和Y轴比例一致，这样能真实反映轨迹形状
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_region_3_path()