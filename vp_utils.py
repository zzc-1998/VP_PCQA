import math 
from math import sqrt
import cv2,os
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np


def background_crop(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #print(gray_img.shape)
    col = np.mean(gray_img,axis=0)
    row = np.mean(gray_img,axis=1)
    for i in range(len(col)):
        if col[i] != 255:
            col_a = i
            break
    for i in range(len(col)):
        if col[-i] != 255:
            col_b = len(col)-i
            break  
    for i in range(len(row)):
        if row[i] != 255:
            row_a = i
            break
    for i in range(len(row)):
        if row[-i] != 255:
            row_b = len(row)-i
            break       
    img = img[row_a:row_b,col_a:col_b,:]
    return img

def generate_dir(path):
    os.makedirs(path, exist_ok=True)
    return path

def pre_adjust(pcd_name,ctrl):
    if 'coffee' in pcd_name or 'house' in pcd_name:
        ctrl.rotate(1050, 0)
        # print("cof ho")
    elif 'Matis' in pcd_name:
        ctrl.rotate(-525, 0)
        ctrl.rotate(0, 525)
        ctrl.rotate(-525, 0)
        # print("mat")
    elif 'AxeGuy' in pcd_name:
        param = o3d.io.read_pinhole_camera_parameters('AxeGuy.json')
        ctrl.convert_from_pinhole_camera_parameters(param)
    return ctrl


def calc_distance(cord1, cord2):
    return math.sqrt((cord1[0]-cord2[0])**2 + (cord1[1]-cord2[1])**2 + (cord1[2]-cord2[2])**2)

def vis_parameter_setting(vis, pcd_name):
    render_opt = vis.get_render_option()
    pt5_list = ['coffee', 'fruits', 'house', 'pen', 'statue', 'wooden']
    # to avoid the quality disturbance caused by the point size
    if pcd_name in pt5_list:
        render_opt.point_size = 5
    else:
        render_opt.point_size = 3
    render_opt.light_on = False
    render_opt.line_width = 0
    #render_opt.background_color = np.array([128/255,128/255,128/255])





# get the geodesic sphere division points
# https://sinestesia.co/blog/tutorials/python-icospheres/
def point_division(subdiv=2):
    scale = 1
    def vertex(x, y, z):
        """ Return vertex coordinates fixed to the unit sphere """
        length = sqrt(x**2 + y**2 + z**2)
        return [(i * scale) / length for i in (x,y,z)]
    # --------------------------------------------------------------
    # Make the base icosahedron
    # Golden ratio
    PHI = (1 + sqrt(5)) / 2
    verts = [
            vertex(-1,  PHI, 0),vertex( 1,  PHI, 0),vertex(-1, -PHI, 0),vertex( 1, -PHI, 0),
            vertex(0, -1, PHI),vertex(0,  1, PHI),vertex(0, -1, -PHI),vertex(0,  1, -PHI),
            vertex( PHI, 0, -1),vertex( PHI, 0,  1),vertex(-PHI, 0, -1),vertex(-PHI, 0,  1),
            ]
    faces = [
            [0, 11, 5],[0, 5, 1],[0, 1, 7],[0, 7, 10],[0, 10, 11],# 5 faces around point 0
            [1, 5, 9],[5, 11, 4],[11, 10, 2],[10, 7, 6],[7, 1, 8],# Adjacent faces
            [3, 9, 4],[3, 4, 2],[3, 2, 6],[3, 6, 8],[3, 8, 9],# 5 faces around 3
            [4, 9, 5],[2, 4, 11],[6, 2, 10],[8, 6, 7],[9, 8, 1],# Adjacent faces
    ]
    # 画八面体
    # verts = [(1,0,0), (0,1,0), (0,0,1), (-1,0,0), (0,0,-1), (0,-1,0)]
    # faces = [[0,1,2], [0,4,1], [0,2,5], [0,5,4], [3,2,1], [3,1,4], [3,5,2], [3,4,5]]

    middle_point_cache = {}

    def middle_point(point_1, point_2):
        """ Find a middle point and project to the unit sphere """

        # We check if we have already cut this edge first to avoid duplicated verts
        smaller_index = min(point_1, point_2)
        greater_index = max(point_1, point_2)
        key = '{0}-{1}'.format(smaller_index, greater_index)
        if key in middle_point_cache:
            return middle_point_cache[key]

        # If it's not in cache, then we can cut it
        vert_1 = verts[point_1]
        vert_2 = verts[point_2]
        middle = [sum(i)/2 for i in zip(vert_1, vert_2)]
        verts.append(vertex(*middle))
        index = len(verts) - 1
        middle_point_cache[key] = index

        return index

    def visualization():
        # Create a point cloud from the vertices
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(verts)

        # Create a line set representing the edges
        lines = []
        for triangle in faces:
            [v1, v2, v3] = triangle
            lines.append([v1, v2])
            lines.append([v2, v3])
            lines.append([v3, v1])

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(verts)
        line_set.lines = o3d.utility.Vector2iVector(lines)

        # Visualize the point cloud and line set
        o3d.visualization.draw_geometries([point_cloud, line_set])

    # Subdivisions
    for i in range(subdiv):
        faces_subdiv = []
        for tri in faces:
            v1 = middle_point(tri[0], tri[1])
            v2 = middle_point(tri[1], tri[2])
            v3 = middle_point(tri[2], tri[0])
            faces_subdiv.append([tri[0], v1, v3])
            faces_subdiv.append([tri[1], v2, v1])
            faces_subdiv.append([tri[2], v3, v2])
            faces_subdiv.append([v1, v2, v3])
        faces = faces_subdiv
    # print(verts)
    #visualization()
    return verts,faces

# get the extrinsic location from xyz coordinates
def find_extrinsic_by_xyz(pcd, pcd_name, cord, radius):
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    vis.add_geometry(pcd)
    ctrl = vis.get_view_control()
    ctrl = pre_adjust(pcd_name,ctrl)
    param = ctrl.convert_to_pinhole_camera_parameters()
    directions = ['east', 'north', 'west', 'south', 'northeast', 'northwest', 'southeast', 'southwest',
                  'northeast1', 'northeast2', 'northwest1', 'northwest2', 'southeast1', 'southeast2', 'southwest1', 'southwest2']

    count = 0
    while count <= 500:
        count += 1
        param = ctrl.convert_to_pinhole_camera_parameters()
        point_cam = np.array([[0.5], [0.5], [1.0], [1]])
        point_world = np.linalg.inv(param.extrinsic) @ point_cam
        point_world = point_world[:3, 0]
        distance = calc_distance(point_world, cord)

        circle = 2 * math.pi / (0.003)  # Rotation angle for one circle
        one_degree = 2 * math.pi * radius / circle  # Distance corresponding to one degree rotation
        #degree = distance / (one_degree * 40)  # Rotation angle
        degree = distance / (one_degree * 5)  # Rotation angle

        min_distance = 100000
        min_idx = -1

        for (i, direction) in enumerate(directions):
            ctrl.convert_from_pinhole_camera_parameters(param)
            angle = degree / math.sqrt(2) if 'sqrt' in direction else degree

            if 'east' in direction:
                ctrl.rotate(angle, 0)
            elif 'north' in direction:
                ctrl.rotate(0, angle)
            elif 'west' in direction:
                ctrl.rotate(-angle, 0)
            elif 'south' in direction:
                ctrl.rotate(0, -angle)
            else:
                x, y = angle, angle
                if '1' in direction:
                    x, y = angle / 2, math.sqrt(3) * angle / 2
                elif '2' in direction:
                    x, y = math.sqrt(3) * angle / 2, angle / 2
                ctrl.rotate(x, y)

            new_param = ctrl.convert_to_pinhole_camera_parameters()
            new_point_world = (np.linalg.inv(new_param.extrinsic) @ point_cam)[:3, 0]
            new_distance = calc_distance(new_point_world, cord)

            if new_distance < min_distance:
                min_distance = new_distance
                min_idx = i

        if min_distance > distance:
            #print("min dis ", distance, "error", distance / (2 * math.pi * radius))
            break
        else:
            ctrl.convert_from_pinhole_camera_parameters(param)
            angle = degree / math.sqrt(2) if 'sqrt' in directions[min_idx] else degree

            if 'east' in directions[min_idx]:
                ctrl.rotate(angle, 0)
            elif 'north' in directions[min_idx]:
                ctrl.rotate(0, angle)
            elif 'west' in directions[min_idx]:
                ctrl.rotate(-angle, 0)
            elif 'south' in directions[min_idx]:
                ctrl.rotate(0, -angle)
            else:
                x, y = angle, angle
                if '1' in directions[min_idx]:
                    x, y = angle / 2, math.sqrt(3) * angle / 2
                elif '2' in directions[min_idx]:
                    x, y = math.sqrt(3) * angle / 2, angle / 2
                ctrl.rotate(x, y)
    vis.destroy_window()
    del ctrl
    del vis
    return param

def get_lines_from_faces(faces):
    """Generate lines (edges) from triangle faces."""
    lines = []
    for triangle in faces:
        [v1, v2, v3] = triangle
        lines.append([v1, v2])
        lines.append([v2, v3])
        lines.append([v3, v1])
    return lines


if __name__ == '__main__':
    # verts = point_division(subdiv = 1)
    # print(verts)
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(verts)
    # # Visualize the geodesic sphere
    # o3d.visualization.draw_geometries([point_cloud])
    verts, faces = point_division(subdiv=2)
    print(len(verts))
    # exit()
    lines = get_lines_from_faces(faces)

    # 创建一个点云
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(verts)

    # 创建一个线集表示边缘
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(verts)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    colors = [[0, 0, 0] for i in range(len(lines))]  # RGB for light red color
    line_set.colors = o3d.utility.Vector3dVector(colors)

    # 创建一个Visualizer对象
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.add_geometry(line_set)

    # 添加点的编号
    # for idx, vert in enumerate(verts):
    #     vis.add_geometry(o3d.geometry.Text3D(str(idx), vert + [0.05, 0.05, 0], text_size=0.05, font_path=None))

    # 调整点的大小
    render_option = vis.get_render_option()
    render_option.point_size = 20  # 你可以根据自己的需要更改这个值

    vis.run()
    vis.destroy_window()
