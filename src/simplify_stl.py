import bpy
import os

# 配置路径
input_dir = "/home/iad/ROS/humble/dual_arm_with_electric/src/dual_arm_agv/meshes"
output_dir = "/home/iad/ROS/humble/dual_arm_with_electric/src/dual_arm_agv/meshes/simplified"

# 创建输出目录
os.makedirs(output_dir, exist_ok=True)

# 简化比例设置（可根据需要调整）
simplify_ratios = {
    "base_link.STL": 0.05,    # base_link保留5%（更激进）
    "default": 0.2             # 其他文件保留20%
}

def simplify_stl(input_path, output_path, ratio):
    """简化单个STL文件"""
    # 清空场景
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # 导入STL
    print(f"\n处理: {os.path.basename(input_path)}")
    bpy.ops.import_mesh.stl(filepath=input_path)
    
    # 获取导入的对象
    obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = obj
    
    original_faces = len(obj.data.polygons)
    original_verts = len(obj.data.vertices)
    print(f"  原始: {original_faces:,} 面片, {original_verts:,} 顶点")
    
    # 添加Decimate修改器
    decimate = obj.modifiers.new(name="Decimate", type='DECIMATE')
    decimate.ratio = ratio
    decimate.use_collapse_triangulate = True
    
    # 应用修改器
    bpy.ops.object.modifier_apply(modifier="Decimate")
    
    # 清理网格
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.0001)  # 合并重复顶点
    bpy.ops.mesh.normals_make_consistent(inside=False)  # 统一法线
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # 平滑着色
    bpy.ops.object.shade_smooth()
    
    simplified_faces = len(obj.data.polygons)
    simplified_verts = len(obj.data.vertices)
    reduction = 100 * (1 - simplified_faces / original_faces)
    
    print(f"  简化后: {simplified_faces:,} 面片, {simplified_verts:,} 顶点")
    print(f"  减少: {reduction:.1f}%")
    
    # 导出为二进制STL
    bpy.ops.export_mesh.stl(
        filepath=output_path,
        use_selection=False,
        ascii=False  # 使用二进制格式（更小）
    )
    
    # 显示文件大小
    input_size = os.path.getsize(input_path) / (1024 * 1024)
    output_size = os.path.getsize(output_path) / (1024 * 1024)
    size_reduction = 100 * (1 - output_size / input_size)
    
    print(f"  文件大小: {input_size:.2f} MB → {output_size:.2f} MB (减少 {size_reduction:.1f}%)")
    
    return simplified_faces, original_faces

# 处理所有STL文件
total_original = 0
total_simplified = 0

for filename in sorted(os.listdir(input_dir)):
    if filename.upper().endswith(".STL"):
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename)
        
        # 获取简化比例
        ratio = simplify_ratios.get(filename, simplify_ratios["default"])
        
        simplified, original = simplify_stl(input_path, output_path, ratio)
        total_original += original
        total_simplified += simplified

print("\n" + "="*60)
print(f"总计:")
print(f"  原始总面片数: {total_original:,}")
print(f"  简化后总面片数: {total_simplified:,}")
print(f"  总减少: {100*(1-total_simplified/total_original):.1f}%")
print("="*60)
