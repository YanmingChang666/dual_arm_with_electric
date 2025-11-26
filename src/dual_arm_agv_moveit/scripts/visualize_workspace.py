#!/usr/bin/env python3
"""
工作空间可视化工具
可视化机械臂 Z 轴最低点扫描结果
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import re
import sys
import os

def parse_summary_file(filename):
    """解析扫描结果摘要文件"""
    positions = []
    z_values = []
    ranges = []
    orientations = []
    
    with open(filename, 'r', encoding='utf-8') as f:
        for line in f:
            # 匹配格式: [X=0.25, Y=-0.80] [vertical_down] → Z_min=1.234 m (范围: 0.456 m)
            match = re.search(r'\[X=([\d.]+), Y=([-\d.]+)\].*Z_min=([\d.]+).*范围: ([\d.]+)', line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                z_min = float(match.group(3))
                z_range = float(match.group(4))
                
                positions.append([x, y])
                z_values.append(z_min)
                ranges.append(z_range)
                
                # 提取姿态名称
                orient_match = re.search(r'\[([\w_]+)\]', line)
                if orient_match:
                    orientations.append(orient_match.group(1))
                else:
                    orientations.append('unknown')
    
    return np.array(positions), np.array(z_values), np.array(ranges), orientations

def plot_workspace_2d(positions, z_values, ranges, output_file='workspace_2d.png'):
    """绘制2D工作空间热图"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # 子图1: Z最低点热图
    scatter1 = ax1.scatter(positions[:, 0], positions[:, 1], 
                          c=z_values, s=200, cmap='RdYlGn_r', 
                          edgecolors='black', linewidth=1.5)
    ax1.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax1.set_title('Z 轴最低可达位置 (m)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # 添加数值标注
    for i, (pos, z) in enumerate(zip(positions, z_values)):
        ax1.annotate(f'{z:.3f}', 
                    xy=(pos[0], pos[1]), 
                    xytext=(5, 5),
                    textcoords='offset points',
                    fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
    
    cbar1 = plt.colorbar(scatter1, ax=ax1)
    cbar1.set_label('Z_min (m)', fontsize=11, fontweight='bold')
    
    # 子图2: 可达范围热图
    scatter2 = ax2.scatter(positions[:, 0], positions[:, 1], 
                          c=ranges, s=200, cmap='viridis', 
                          edgecolors='black', linewidth=1.5)
    ax2.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax2.set_title('Z 轴可达范围 (m)', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # 添加数值标注
    for i, (pos, r) in enumerate(zip(positions, ranges)):
        ax2.annotate(f'{r:.3f}', 
                    xy=(pos[0], pos[1]), 
                    xytext=(5, 5),
                    textcoords='offset points',
                    fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
    
    cbar2 = plt.colorbar(scatter2, ax=ax2)
    cbar2.set_label('范围 (m)', fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ 2D 可视化已保存: {output_file}")
    plt.close()

def plot_workspace_3d(positions, z_values, output_file='workspace_3d.png'):
    """绘制3D工作空间表面"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # 创建网格
    x_unique = np.unique(positions[:, 0])
    y_unique = np.unique(positions[:, 1])
    
    if len(x_unique) > 1 and len(y_unique) > 1:
        # 如果有足够的点,创建插值表面
        from scipy.interpolate import griddata
        
        xi = np.linspace(positions[:, 0].min(), positions[:, 0].max(), 50)
        yi = np.linspace(positions[:, 1].min(), positions[:, 1].max(), 50)
        xi, yi = np.meshgrid(xi, yi)
        
        zi = griddata(positions, z_values, (xi, yi), method='cubic')
        
        # 绘制表面
        surf = ax.plot_surface(xi, yi, zi, cmap='coolwarm', 
                              alpha=0.7, edgecolor='none')
        
        # 添加colorbar
        fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5, 
                    label='Z_min (m)')
    
    # 绘制实际测试点
    scatter = ax.scatter(positions[:, 0], positions[:, 1], z_values, 
                        c=z_values, s=100, cmap='coolwarm', 
                        edgecolors='black', linewidth=1.5, 
                        depthshade=True)
    
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z_min (m)', fontsize=12, fontweight='bold')
    ax.set_title('机械臂工作空间 - Z 轴最低可达位置', 
                fontsize=14, fontweight='bold', pad=20)
    
    # 设置视角
    ax.view_init(elev=25, azim=45)
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ 3D 可视化已保存: {output_file}")
    plt.close()

def plot_orientation_comparison(positions, z_values, orientations, 
                                output_file='orientation_comparison.png'):
    """比较不同姿态下的结果"""
    unique_orientations = list(set(orientations))
    
    if len(unique_orientations) <= 1:
        print("⚠️  只有一种姿态,跳过姿态比较图")
        return
    
    fig, axes = plt.subplots(1, len(unique_orientations), 
                            figsize=(6*len(unique_orientations), 5))
    
    if len(unique_orientations) == 1:
        axes = [axes]
    
    for i, orient in enumerate(unique_orientations):
        # 筛选该姿态的数据
        mask = [o == orient for o in orientations]
        pos_subset = positions[mask]
        z_subset = z_values[mask]
        
        if len(pos_subset) == 0:
            continue
        
        scatter = axes[i].scatter(pos_subset[:, 0], pos_subset[:, 1], 
                                 c=z_subset, s=200, cmap='RdYlGn_r',
                                 edgecolors='black', linewidth=1.5,
                                 vmin=z_values.min(), vmax=z_values.max())
        
        axes[i].set_xlabel('X (m)', fontsize=11, fontweight='bold')
        axes[i].set_ylabel('Y (m)', fontsize=11, fontweight='bold')
        axes[i].set_title(f'{orient}\n(n={len(pos_subset)})', 
                         fontsize=12, fontweight='bold')
        axes[i].grid(True, alpha=0.3)
        axes[i].set_aspect('equal')
        
        # 添加数值标注
        for pos, z in zip(pos_subset, z_subset):
            axes[i].annotate(f'{z:.3f}', 
                           xy=(pos[0], pos[1]), 
                           xytext=(3, 3),
                           textcoords='offset points',
                           fontsize=7,
                           bbox=dict(boxstyle='round,pad=0.2', 
                                   facecolor='white', alpha=0.7))
    
    # 添加共享colorbar
    fig.colorbar(scatter, ax=axes, orientation='horizontal', 
                pad=0.1, label='Z_min (m)', aspect=40)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✅ 姿态比较图已保存: {output_file}")
    plt.close()

def generate_report(positions, z_values, ranges, orientations, output_file='report.txt'):
    """生成统计报告"""
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("工作空间分析报告\n")
        f.write("=" * 60 + "\n\n")
        
        f.write(f"总测试点数: {len(positions)}\n")
        f.write(f"成功点数: {len(z_values)}\n\n")
        
        f.write("--- Z 轴统计 ---\n")
        f.write(f"最低 Z 值: {z_values.min():.4f} m\n")
        f.write(f"最高 Z 值: {z_values.max():.4f} m\n")
        f.write(f"平均 Z 值: {z_values.mean():.4f} m\n")
        f.write(f"标准差: {z_values.std():.4f} m\n\n")
        
        f.write("--- 可达范围统计 ---\n")
        f.write(f"最小范围: {ranges.min():.4f} m\n")
        f.write(f"最大范围: {ranges.max():.4f} m\n")
        f.write(f"平均范围: {ranges.mean():.4f} m\n\n")
        
        f.write("--- 位置边界 ---\n")
        f.write(f"X 范围: [{positions[:, 0].min():.4f}, {positions[:, 0].max():.4f}] m\n")
        f.write(f"Y 范围: [{positions[:, 1].min():.4f}, {positions[:, 1].max():.4f}] m\n\n")
        
        if len(set(orientations)) > 1:
            f.write("--- 各姿态统计 ---\n")
            for orient in set(orientations):
                mask = [o == orient for o in orientations]
                z_subset = z_values[mask]
                if len(z_subset) > 0:
                    f.write(f"\n{orient}:\n")
                    f.write(f"  测试点数: {len(z_subset)}\n")
                    f.write(f"  最低 Z: {z_subset.min():.4f} m\n")
                    f.write(f"  平均 Z: {z_subset.mean():.4f} m\n")
        
        f.write("\n" + "=" * 60 + "\n")
    
    print(f"✅ 统计报告已保存: {output_file}")

def main():
    if len(sys.argv) < 2:
        print("使用方法: python3 visualize_workspace.py <summary_file>")
        print("示例: python3 visualize_workspace.py workspace_scan_results_*/summary.txt")
        sys.exit(1)
    
    summary_file = sys.argv[1]
    
    if not os.path.exists(summary_file):
        print(f"❌ 文件不存在: {summary_file}")
        sys.exit(1)
    
    print(f"📊 正在分析文件: {summary_file}")
    
    # 解析数据
    positions, z_values, ranges, orientations = parse_summary_file(summary_file)
    
    if len(positions) == 0:
        print("❌ 未找到有效数据")
        sys.exit(1)
    
    print(f"✅ 找到 {len(positions)} 个有效测试点")
    
    # 获取输出目录
    output_dir = os.path.dirname(summary_file)
    
    # 生成可视化
    print("\n📈 生成可视化图表...")
    plot_workspace_2d(positions, z_values, ranges, 
                     os.path.join(output_dir, 'workspace_2d.png'))
    
    if len(positions) >= 4:  # 需要足够的点才能绘制3D表面
        plot_workspace_3d(positions, z_values, 
                         os.path.join(output_dir, 'workspace_3d.png'))
    
    plot_orientation_comparison(positions, z_values, orientations,
                               os.path.join(output_dir, 'orientation_comparison.png'))
    
    # 生成报告
    generate_report(positions, z_values, ranges, orientations,
                   os.path.join(output_dir, 'analysis_report.txt'))
    
    print("\n✅ 完成! 所有结果已保存到:", output_dir)

if __name__ == '__main__':
    main()