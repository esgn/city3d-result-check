import os
import shutil

def recreate_dir(output_dir):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.mkdir(output_dir)

def read_obj_file(filepath):
    vertices, normals, faces = [],[],[]
    with open(filepath) as src:
        while True:
            line = src.readline()
            if not line: 
                break
            else:
                line = line.rstrip()
            if line.startswith("v "):
                c=line.split(' ')[1:]
                vertices.append([float(x) for x in c])
            if line.startswith("vn "):
                c=line.split(' ')[1:]
                normals.append([int(x) for x in c])
            if line.startswith("f "):
                c=line.split(' ')[1:]
                faces.append([str(x) for x in c])
    return vertices, normals, faces

def write_obj_file(filepath, vertices, normals, faces):
    with open(filepath,'w') as dst:
        for vertex in vertices:
            vx = ["v"] + list(str(v) for v in vertex)
            dst.write(' '.join(vx)+"\n")
        for normal in normals:
            nl = ["vn"] + list(str(n) for n in normal)
            dst.write( ' '.join(nl)+"\n")
        for face in faces:
            fc = ['f'] + list(str(f) for f in face)
            dst.write( ' '.join(fc)+"\n")

def read_origin(filepath):
        fline=open(filepath).readline().rstrip()       
        center = [float(x) for x in fline.split(' ')]
        return center