#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import argparse
from utils import *
from shapely import Polygon, Point, LineString
from skspatial.objects import Line as skLine
from skspatial.objects import Point as skPoint

def parse_args():
    parser = argparse.ArgumentParser("Try to fix results by adding necessary vertices to face edges")
    parser.add_argument("--input_file", "-f",
                        help="Input obj city3d result file", default="23_result.obj")
    parser.add_argument("--output_file", "-o",
                        help="output file", default="23_result_cleaned.obj")
    parser.add_argument("--delta_point_in_line", "-p", 
                        help="Maximum distance for considering that a point belongs to a line", default="1e-6")
    parser.add_argument("--delta_coordinates", "-d", 
                        help="If the distance between two coordinates are under this threshold they are considered as the same", default="1e-6")
    return parser.parse_args()

# distance 3d point to 3d line
def get_distance(vertex,edge):
    is_in_segment = 1
    vertex = skPoint(list(vertex.coords[0]))
    line_start = skPoint(list(edge.coords[0]))
    line_end = skPoint(list(edge.coords[1]))
    # Get distance between 3d point and 3 line
    line = skLine.from_points(line_start, line_end)
    vertex_edge_distance = line.distance_point(vertex)
    # Project point along line and get position of projected point along line
    point_projected = line.project_point(vertex)
    distance_start_projected = line_start.distance_point(point_projected)
    distance_end_projected = line_end.distance_point(point_projected)
    edge_length = line_start.distance_point(line_end)
    # Test if point is between origin and stard of the edge (naive approach)
    if (distance_start_projected + distance_end_projected > edge_length+1e-6):
        is_in_segment = 0
    distance_projected_along_edge = float(distance_start_projected/edge_length)
    return vertex_edge_distance, distance_projected_along_edge, is_in_segment

# return face vertices as list of coordinates
def get_face_vertices(face, vertices):
    face_vertices = []
    vertices_index = [int(i) for i in face]
    face_vertices = [vertices[i-1] for i in vertices_index] + [vertices[vertices_index[0]-1]]
    return face_vertices

# return face edges as list of shapely linestring
def get_face_edges(face_vertices):
    footprint = Polygon(face_vertices)
    b = footprint.boundary.coords
    edges = [LineString(b[k:k+2]) for k in range(len(b) - 1)]
    return edges

# return candidate vertices for a face 
def get_candidate_vertices(face,vertices,min_distance):

    candidate_vertices = {}
    candidate_count = 0
    # get face vertices and edges
    face_vertices = get_face_vertices(face, vertices)
    edges = get_face_edges(face_vertices)

    for e_idx,edge in enumerate(edges):
        added_coords = []

        xmin = min(edge.coords[0][0],edge.coords[1][0]) - min_distance
        ymin = min(edge.coords[0][1],edge.coords[1][1]) - min_distance
        zmin = min(edge.coords[0][2],edge.coords[1][2]) - min_distance
        xmax = max(edge.coords[0][0],edge.coords[1][0]) + min_distance
        ymax = max(edge.coords[0][1],edge.coords[1][1]) + min_distance
        zmax = max(edge.coords[0][2],edge.coords[1][2]) + min_distance

        for v_idx,vertex_coords in enumerate(vertices):
            
            # ignore vertices already present in face
            if vertex_coords in face_vertices:
                continue

            # Continue if vertex is not situated in 3D bbox surrounding edge
            if (vertex_coords[0] < xmin or vertex_coords[0] > xmax 
                or vertex_coords[1] < ymin or vertex_coords[1] > ymax 
                or vertex_coords[2] < zmin or vertex_coords[2] > zmax):
                continue

            # Get distance between line and vertex
            vertex = Point(vertex_coords)
            vertex_edge_distance, distance_projected_along_edge, is_in_segment = get_distance(vertex,edge)

            # Continue if vertex projection on edge is not between edge start and end point
            if not is_in_segment:
                # print("is_in_segment removing " + str(v_idx+1))
                continue

            # Continue if vertex is at the start or at the end of edge
            # TODO: Is O or 1 ok or should we use something like a delta 1e-N
            if distance_projected_along_edge == 0 or distance_projected_along_edge == 1:
                # print("distance_projected_along_edge removing " + str(v_idx+1))
                continue

            # If vertex distance to edge is small enough and the new vertex has not already been added to the list of candidates
            if vertex_edge_distance < min_distance and vertex_coords not in added_coords:
                if e_idx not in candidate_vertices:
                    candidate_vertices[e_idx] = {}
                vertex_info = {}
                vertex_info["distance_along_edge"] = distance_projected_along_edge
                vertex_info["coordinates"] = vertex_coords
                vertex_info["distance_to_edge"] = vertex_edge_distance
                candidate_vertices[e_idx][v_idx+1] = vertex_info
                added_coords.append(vertex_coords)
                candidate_count+=1

    return candidate_vertices, candidate_count

def add_candidates_to_face(face, candidate_vertices):
    # order by edge index reversed
    candidate_vertices = dict(sorted(candidate_vertices.items(), reverse=True))
    vertices_added = 0
    new_face = face.copy()
    for edge_idx,vertex_to_add in candidate_vertices.items():
        # order vertex to add by distance along edge in reverse order
        vertex_entry = dict(sorted(vertex_to_add.items(), key=lambda item: item[1]["distance_along_edge"], reverse=True))
        for vertex_idx, vertex_info in vertex_entry.items():
            new_face.insert(edge_idx+1, vertex_idx)
            vertices_added += 1
    return new_face, vertices_added

def cleanup_coordinates_on_axis(vertices, delta, axis):
    z = [v[axis] for v in vertices]
    unique_z = sorted(set(z))
    replacements = {}
    for i in range(len(unique_z)-1):
        if abs(unique_z[i+1] - unique_z[i]) < delta:
            if unique_z[i] not in replacements:
                replacements[unique_z[i+1]] = unique_z[i]
            else:
                replacements[unique_z[i+1]] = replacements[unique_z[i]]
    for z_to_replace,z_replacement in replacements.items():
        for index, vz in enumerate(z):
            if vz == z_to_replace:
                z[index] = z_replacement
    for i in range(len(z)):
        vertices[i][axis] = z[i]
    return vertices

def main():

    args = parse_args()
    filename = args.input_file        
    vertices, normals, faces = read_obj_file(filename)

    # Merge very close coordinates values on all axis
    vertices = cleanup_coordinates_on_axis(vertices, float(args.delta_point_in_line), 0)
    vertices = cleanup_coordinates_on_axis(vertices, float(args.delta_point_in_line), 1)
    vertices = cleanup_coordinates_on_axis(vertices, float(args.delta_point_in_line), 2)

    # Add vertices to edges when these vertices are very close to the edge
    new_faces = []
    for face in faces:
        candidate_vertices, candidate_count = get_candidate_vertices(face, vertices, float(args.delta_point_in_line))
        if(len(candidate_vertices)>0):
            new_face, vertices_added = add_candidates_to_face(face,candidate_vertices)
            new_faces.append(new_face)
        else:
            new_faces.append(face)

    write_obj_file(args.output_file, vertices, normals, new_faces)

if __name__ == "__main__":
    main()
