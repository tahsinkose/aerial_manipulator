import sys
import bpy
import numpy as np
import re
import json
from pathlib import Path
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--model_dir', dest='model_dir',
                    default='generated/apple_tree')
parser.add_argument('-s', '--seed', dest='seed', type=int, default=np.random.randint(10000))


if '--' in sys.argv:
    argv = sys.argv[sys.argv.index('--') + 1:]
else:
    argv = []
args = parser.parse_known_args(argv)[0]

np.random.seed(args.seed)

bpy.ops.object.select_all(action='DESELECT')

model_dir = Path(args.model_dir)
model_dir.mkdir(parents=True, exist_ok=True)

primitives_count = dict()
def reset_primitives():
    primitives_count['Apple'] = 0
    primitives_count['Blossom'] = 0
    primitives_count['Leaf'] = 0
    primitives_count['Branch'] = 0

def decide_prefix(name):
    global primitives_count
    for prefix in primitives_count:
        if name.startswith(prefix):
            primitives_count[prefix] += 1
            return prefix + str(primitives_count[prefix])

def assign_indices(objects, keep_materials):
    reset_primitives()
    bpy.ops.object.select_all(action='DESELECT')

    for ob in objects:
        # name the mesh
        ob.name = decide_prefix(ob.material_slots[0].material.name)
        if ob.name.startswith('Apple'):
            uv_layer_to_delete = ob.data.uv_layers.get("UVMap")
            if uv_layer_to_delete != None:
                ob.data.uv_layers.remove(uv_layer_to_delete)
        # remove the material (it will be added in the SDF file)
        if not keep_materials:
            ob.data.materials.clear()
        # select for the export
        ob.select_set(True)
class Quaternion:
    def __init__(self, x, y, z ,w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __mul__(self, other):
        v1 = np.array([self.x, self.y, self.z])
        v2 = np.array([other.x, other.y, other.z])
        w = self.w * other.w - np.dot(v1, v2)
        v = self.w * v2 + other.w * v1 + np.cross(v1, v2)
        result = Quaternion(v[0], v[1], v[2], w)
        return result
    def __str__(self):
        return str(self.w) + " <" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ">"
    def rotate(self, vec):
        conjugate = Quaternion(-self.x, -self.y, -self.z, self.w)
        pure = Quaternion(vec[0], vec[1], vec[2], 0)
        result = self * pure * conjugate
        return np.array([result.x, result.y, result.z])


class Pose:
    def __init__(self, t, q):
        self.t = t
        self.q = q
    def __mul__(self, other):
        t = self.t + self.q.rotate(other.t)
        q = self.q * other.q
        result = Pose(t, q)
        return result
    def __str__(self):
        return "translation: " + str(self.t) + "\nquaternion: " + str(self.q)


def euler_to_quaternion(roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return Quaternion(qx, qy, qz, qw)

# Generate the main structure

class Node:
    def __init__(self, x, y, z, yaw, r):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.r = r

    def __str__(self):
        return "x: " + str(self.x) + ", y: "+ str(self.y) + ", z: " + str(self.z) + ", yaw: " \
            + str(self.yaw) + ", r: " + str(self.r)


def gen_node_heights(height: float, node_count: int):
    sample_count = 8
    samples = np.array([np.concatenate([np.sort(np.random.uniform(
        0.005, height, node_count)), [height]]) for _ in range(sample_count)])
    max_distances = np.max(samples[:, 1:] - samples[:, :-1], axis=1)
    return samples[np.argmin(max_distances)]


def gen_main_stem(collection):
    verts = []
    faces = []

    DIV = 23
    height = 2.5 + np.random.normal(0, 0.1)
    r_start = .009 + np.random.normal(0, 0.0001)
    r_end = r_start * 0.45
    node_count = 18 + np.random.randint(6)

    # create the nodes from bottom-up
    nodes = [Node(0, 0, 0, 0, r_start)]
    node_heights = gen_node_heights(height, node_count)
    for z in node_heights:
        prev = nodes[-1]
        yaw = prev.yaw + (np.pi * (2.0/3.0) + np.random.normal(0, 0.4))
        step_up = z - prev.z
        y = np.sin(yaw) * (step_up/15.0)
        x = np.cos(yaw) * (step_up/15.0)
        r = r_start - (r_start-r_end) * (z / height)
        nodes.append(Node(x, y, z, yaw, r))

    def create_ring_verts(node):
        verts = []
        for i in range(DIV):
            a = (np.pi*2) * (i/DIV)
            x = node.x + np.cos(a) * node.r
            y = node.y + np.sin(a) * node.r
            verts.append((x, y, node.z))
        return verts

    prev_ring = create_ring_verts(nodes[0])
    verts = verts + prev_ring
    for node in nodes[1:]:
        vert_start = len(verts) - DIV
        ring = create_ring_verts(node)
        verts = verts + ring
        prev_ring = ring

        for i in range(DIV):
            vi = vert_start + i
            vip = vert_start + ((i+1) % DIV)
            faces.append((vi, vip, vip+DIV, vi+DIV))

    #close main stem
    last_ring_start = len(verts) - DIV
    verts.append((nodes[-1].x, nodes[-1].y, nodes[-1].z + 0.01))
    for i in range(DIV):
        vi = last_ring_start + i
        vip = last_ring_start + ((i+1) % DIV)
        faces.append((vi, vip, len(verts)-1))

    #Define mesh and object
    mesh = bpy.data.meshes.new("Branch")

    #Create mesh
    mesh.from_pydata(verts, [], faces)
    mesh.update(calc_edges=True)

    object = bpy.data.objects.new("Branch", mesh)

    collection.objects.link(object)

    mat = bpy.data.materials.get("Branch")
    object.data.materials.append(mat)

    return object, nodes



# Generate leaf, fruit, and flower formations

def filter_prefix(objects, prefix):
    return list(
        filter(lambda ob: ob.name.startswith(prefix), objects))

def gen_end_stem(collection, location: (float, float, float), yaw: float, growth: float):
    primitives_collection = bpy.data.collections['Primitives']
    stems = filter_prefix(primitives_collection.all_objects, 'b_')
    growths = np.array(list(map(lambda ob: ob.location[2], stems)))
    growths = growths / np.max(growths)
    idx_options = np.argsort(np.abs(growths - growth))[:5]
    ob_stem = stems[np.random.choice(idx_options)]
    ob_fruits = filter_prefix(ob_stem.children, 'pose_fruit')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.duplicate({
        'selected_objects': [ob_stem] + ob_fruits,
        'active_object': ob_stem
    })
    # Choose the duplicated stem and fruits.
    ob_stem = filter_prefix(bpy.context.selected_objects, 'b_')[0]
    ob_fruits = filter_prefix(ob_stem.children, 'pose_fruit')
    ob_stem.select_set(False)
    if location is not None:
        ob_stem.location = location
    if yaw is not None:
        ob_stem.rotation_euler[2] += yaw
    base_T_stem = Pose(np.asarray(ob_stem.location), euler_to_quaternion(
        ob_stem.rotation_euler[0], ob_stem.rotation_euler[1], ob_stem.rotation_euler[2]))
    for ob in ob_fruits:
        stem_T_fruit = Pose(np.asarray(ob.location), euler_to_quaternion(
            ob.rotation_euler[0], ob.rotation_euler[1], ob.rotation_euler[2]))
        base_T_fruit = base_T_stem * stem_T_fruit
        ob.location = base_T_fruit.t
    collection.objects.link(ob_stem)
    primitives_collection.objects.unlink(ob_stem)
    for ob in ob_fruits:
        collection.objects.link(ob)
        primitives_collection.objects.unlink(ob)

    return ob_stem, ob_fruits




def gen_plant(write_results=False, keep_materials=False):
    apple_tree_collection = bpy.data.collections.new('apple_tree')
    bpy.context.scene.collection.children.link(apple_tree_collection)

    ob_main_stem, nodes = gen_main_stem(apple_tree_collection)

    ob_meshes = [ob_main_stem]
    fruits = []
    for node in nodes[1:]:
        ob_m, ob_f = gen_end_stem(
            apple_tree_collection,
            location = (node.x, node.y, node.z),
            yaw = node.yaw,
            growth = node.z / nodes[-1].z)
        ob_meshes.append(ob_m)
        fruits += ob_f
    markers = []


    def add_marker(location, type='FRUIT'):
        marker = {
            'marker_type': type,
            'translation': [location[0], location[1], location[2]],
        }
        markers.append(marker)


    for ob_f in fruits:
        add_marker(ob_f.location)
        bpy.ops.object.delete({'selected_objects': [ob_f]}, use_global=True)

    bpy.ops.object.join({
        'selected_objects': ob_meshes,
        'selected_editable_objects': ob_meshes,
        'active_object': ob_meshes[0]
    })

    ob_merged = apple_tree_collection.objects[0]
    bpy.ops.mesh.separate({
        'selected_editable_objects': [ob_merged]
    }, type='MATERIAL')

    assign_indices(apple_tree_collection.all_objects, keep_materials)

    bpy.ops.object.select_all(action='DESELECT')
    for i in range(0, len(apple_tree_collection.all_objects)):
        ob = apple_tree_collection.all_objects[i]
        if ob.name.startswith('Apple'):
            ob.select_set(True)
            bpy.ops.mesh.separate(type='LOOSE')
            ob.select_set(False)
    apple_tree_collection = bpy.data.collections['apple_tree.001']
    objects_to_delete = []
    for ob in apple_tree_collection.all_objects:
        if ob.dimensions[0] <= 1e-2 or ob.dimensions[1] <= 1e-2 or ob.dimensions[2] <= 1e-2:
            objects_to_delete.append(ob)
    bpy.ops.object.delete({'selected_objects': objects_to_delete}, use_global=True)
    apple_tree_collection = bpy.data.collections['apple_tree.001']
    assign_indices(apple_tree_collection.all_objects, keep_materials)
    if write_results:
        bpy.ops.wm.collada_export(filepath=str(
            model_dir / 'meshes/apple_tree.dae'), check_existing=True, selected=True)

        with open(model_dir / 'markers.json', 'w') as outfile:
            json.dump(markers, outfile, indent=4)

    return apple_tree_collection

if __name__ == '__main__':
    gen_plant(write_results=True)
