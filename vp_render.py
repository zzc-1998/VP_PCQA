import math
import numpy as np
import open3d as o3d
from PIL import Image
import math
import cv2,os,time
from vp_utils import point_division,calc_distance,vis_parameter_setting,find_extrinsic_by_xyz
from vp_utils import background_crop,generate_dir,pre_adjust
import argparse


def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=50, fill='█'):
    
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end='')

    if iteration == total:
        print()


# caculate the xyz coordinates from the extrinsic parameters
def calc_point_xyz(extrinsic_matrix):
    x, y, z = 0.5, 0.5, 1.0
    point_cam = np.array([[x], [y], [z], [1]])
    point_world = np.linalg.inv(extrinsic_matrix) @ point_cam
    xyz = point_world[:3, 0]
    return xyz

# get the rotation center coordinates and radius
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


# translate the division points to real xyz coordinates
def get_point_division(pcd, define_points):
    center, radius = find_center_radius(pcd)
    for i in range(len(define_points)):
        define_points[i] = [define_points[i][0]*radius+center[0], 
                            define_points[i][1]*radius+center[1], define_points[i][2]*radius+center[2]]
    # print(define_points)
    # draw_vp(pcd, define_points)
    return define_points, radius


#render the point clouds from the division viewpoints
def geoesdic_sphere_vp_render(pcd, pcd_name,xyz,img_path,json_path):
    # prepare rendering
    center,radius = find_center_radius(pcd)
        
    # get the camera parameter for xyz
    param = find_extrinsic_by_xyz(pcd,pcd_name, xyz,radius) 
    # save the viewpoint parameter to json 
    o3d.io.write_pinhole_camera_parameters(json_path, param)
    param = o3d.io.read_pinhole_camera_parameters(json_path)
    # control the camera to the viewpoint
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pcd)
    vis_parameter_setting(vis, pcd_name)
    ctrl = vis.get_view_control()
    ctrl.convert_from_pinhole_camera_parameters(param)
    vis.poll_events()
    vis.update_renderer()  

    #render from the viewpoint  
    img = vis.capture_screen_float_buffer(True)
    img = Image.fromarray((np.asarray(img)* 255).astype(np.uint8))
    img = cv2.cvtColor(np.asarray(img),cv2.COLOR_RGB2BGR) 
    # crop the main object out of the white background
    img = background_crop(img)
    cv2.imwrite(img_path,img)
        
    # print("time consuming: ",end-start)
    vis.destroy_window()
    del ctrl
    del vis

    
    
   




if __name__ == '__main__':
    # capture the projections of the 3d model
    parser = argparse.ArgumentParser()

    # parser.add_argument('--pcd_path', type=str, default = 'dataset_zzc/dis')
    # parser.add_argument('--img_path', type=str, default = 'dataset_zzc/dis_vp_image') 
    # parser.add_argument('--json_path', type=str, default = 'dataset_zzc/dis_vp_json') 
    # parser.add_argument('--subdiv', type=int, default = 2) 
    parser.add_argument('--pcd_path', type=str, default = 'dataset_zzc/ref_ply')
    parser.add_argument('--img_path', type=str, default = 'dataset_zzc/ref_vp_image') 
    parser.add_argument('--json_path', type=str, default = 'dataset_zzc/ref_vp_json') 
    parser.add_argument('--subdiv', type=int, default = 1) 

    config = parser.parse_args()
    #print(define_points)
    for pcd_name in os.listdir(config.pcd_path):
        # generate the dirs for saving vps and images
        pcd_vp_image_path = generate_dir(os.path.join(config.img_path,pcd_name))
        #print(pcd_vp_image_path)
        pcd_vp_json_path = generate_dir(os.path.join(config.json_path,pcd_name))
        # load the point cloud
        pcd_name = os.path.join(config.pcd_path,pcd_name)
        print(pcd_name)
        pcd = o3d.io.read_point_cloud(pcd_name)
        # sample the viewpoints
        define_points = point_division(subdiv=config.subdiv)
        for i in range(len(define_points)):
            print(i)
            print(define_points[i])
        #print(define_points)
        #break
        xyz_list = get_point_division(pcd, define_points)[0]
        start = time.time()
        for i in range(len(xyz_list)):
            #print(xyz_list[i])
            geoesdic_sphere_vp_render(pcd, pcd_name, xyz_list[i], os.path.join(pcd_vp_image_path,str(i) + '.png'), os.path.join(pcd_vp_json_path,str(i) + '.json'))
            print_progress_bar(i+1, len(xyz_list), prefix='Progress:', suffix='Complete', length=30)
        end = time.time()
        print('Rendering time comsumption: ' + str(end-start) + ' s.')
    
    