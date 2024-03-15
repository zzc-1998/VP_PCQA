import open3d as o3d
import math,os,time
import numpy as np
import argparse
from scipy.stats import entropy
import pandas as pd

def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=50, fill='█'):
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end='')

    if iteration == total:
        print()

def calc_distance(cord1, cord2):
    return math.sqrt((cord1[0]-cord2[0])**2 + (cord1[1]-cord2[1])**2 + (cord1[2]-cord2[2])**2)

def calc_point_xyz(extrinsic_matrix):
    x, y, z = 0.5, 0.5, 1.0
    point_cam = np.array([[x], [y], [z], [1]])
    point_world = np.linalg.inv(extrinsic_matrix) @ point_cam
    xyz = point_world[:3, 0]
    return xyz

def find_center_radius(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pcd)
    ctrl = vis.get_view_control()
    param = ctrl.convert_to_pinhole_camera_parameters()
    # get the original coordinates of the camera
    res = calc_point_xyz(param.extrinsic)

    # rotate 180 degree and get the rotated coordinates
    ctrl.rotate(0,math.pi/0.003)
    param2 = ctrl.convert_to_pinhole_camera_parameters()
    res2 = calc_point_xyz(param2.extrinsic)
    radius = calc_distance(res, res2) / 2

    # print(pcd.get_center())
    # print(res, res2, (res+res2)/2, radius)
    vis.destroy_window()
    del ctrl
    del vis

    # the middle point is the rotation center
    return (res+res2)/2, radius

def get_visible_ratio(pcd,pt_map):
    # get the visible points ratio
    num_points = len(pcd.points)
    num_visible_points = len(pt_map)
    point_visible_ratio = num_visible_points/num_points

    # get the visible hull ratio 
    pcd_visible = pcd.select_by_index(pt_map) 
    hull,_ = pcd.compute_convex_hull()
    # get the convex_hull of visible points
    hull_visible,_ = pcd_visible.compute_convex_hull()
    # Get the volume of the convex hull
    volume = hull.get_volume()
    volume_visible = hull_visible.get_volume()
    hull_visible_ratio = volume_visible/volume
    
    # get the color entropy of the visible points
    colors = np.asarray(pcd.colors)
    colors_entropy = entropy(colors)
    colors_visible = np.asarray(pcd_visible.colors)
    colors_entropy_visible = entropy(colors_visible)
    color_entropy_visible_ratio = colors_entropy_visible/colors_entropy

    return point_visible_ratio,hull_visible_ratio,color_entropy_visible_ratio




if __name__ == '__main__':
    # capture the projections of the 3d model
    parser = argparse.ArgumentParser()

    parser.add_argument('--pcd_path', type=str, default = 'dataset_zzc/dis')
    parser.add_argument('--json_path', type=str, default = 'dataset_zzc/dis_vp_json')  
    parser.add_argument('--vp_num', type=int, default = 42)  

    config = parser.parse_args()
    for pcd_name in os.listdir(config.pcd_path)[107:]:
        print(pcd_name)
        pcd = o3d.io.read_point_cloud(os.path.join(config.pcd_path,pcd_name))
        _,radius = find_center_radius(pcd)
        data = {'vp_index':[],'pvr':[],'hvr':[],'revr':[],'gevr':[],'bevr':[]}
        start = time.time()
        for i in range(config.vp_num):
            json_path = os.path.join(config.json_path, pcd_name, str(i)+ '.json')
            # get the camera location
            param = o3d.io.read_pinhole_camera_parameters(json_path)
            camera = calc_point_xyz(param.extrinsic)
            # get the visbile points index
            _, pt_map = pcd.hidden_point_removal(camera, radius*1000)
            point, hull, rgb = get_visible_ratio(pcd,pt_map)
            data['vp_index'].append(str(i))
            data['pvr'].append(point)
            data['hvr'].append(hull)
            data['revr'].append(rgb[0])
            data['gevr'].append(rgb[1])
            data['bevr'].append(rgb[2])
            print_progress_bar(i+1, config.vp_num, prefix='Progress:', suffix='Complete', length=30)
        end = time.time()
        print('Rendering time comsumption: ' + str(end-start) + ' s.')
        data = pd.DataFrame(data)
        data.to_csv(os.path.join(config.json_path,pcd_name,'vp_attribute.csv'),index = None)
        
