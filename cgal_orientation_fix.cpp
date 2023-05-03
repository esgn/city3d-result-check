#include <iostream>
#include <string>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Surface_mesh;
typedef boost::graph_traits<Surface_mesh>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor halfedge_descriptor;
typedef boost::graph_traits<Surface_mesh>::edge_descriptor edge_descriptor;
typedef boost::graph_traits<Surface_mesh>::face_descriptor face_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

int main(int argc, char *argv[])
{
  // input parameters
  const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("23_result.obj");
  const char *outfilename = (argc > 2) ? argv[2] : "23_result_fixed.obj";
  const int output_interesecting_faces = (argc > 3) ? std::stoi(argv[3]) : 1;

  std::vector<Point> points;
  std::vector<std::vector<std::size_t>> polygons;

  // read city3D result as polygon soup
  if (!CGAL::IO::read_polygon_soup(filename, points, polygons) || points.empty())
  {
    std::cerr << "Cannot open file " << std::endl;
    return 1;
  }
  std::cout << "Reading city3d result as polygon soup" << std::endl;
  std::cout << "\t " << points.size() << " points imported" << std::endl;
  std::cout << "\t " << polygons.size() << " polygons imported" << std::endl;

  // Try repairing polygon soup
  PMP::repair_polygon_soup(points, polygons);
  std::cout << "Trying repair_polygon_soup()" << std::endl;
  std::cout << "\t " << points.size() << " points after repair_polygon_soup" << std::endl;
  std::cout << "\t " << polygons.size() << " polygons after repair_polygon_soup" << std::endl;
  std::cout << "\t Is polygon soup a polygon mesh ? " << PMP::is_polygon_soup_a_polygon_mesh(polygons) << std::endl;

  // orient polygon soup with visitor to get details
  bool success = PMP::orient_polygon_soup(points, polygons);
  std::cout << "Trying orient_polygon_soup()" << std::endl;
  std::cout << "\t Is polygon soup a polygon mesh ? " << PMP::is_polygon_soup_a_polygon_mesh(polygons) << std::endl;

  // Polygon soup to polygon mesh
  Surface_mesh mesh;
  if (PMP::is_polygon_soup_a_polygon_mesh(polygons))
  {
    std::cout << "Converting polygon_soup to polygon_mesh" << std::endl;
    PMP::polygon_soup_to_polygon_mesh(points, polygons, mesh);
  }
  else
  {
    std::cout << "Cannot convert polygon soup to polygon mesh" << std::endl;
    return 2;
  }
  
  std::cout << "\t Is the mesh closed ? " << CGAL::is_closed(mesh) << std::endl;
  std::cout << "\t Is the mesh valid ? " << CGAL::is_valid(mesh) << std::endl;

  // Check for self intersections in the mesh
  bool intersecting = PMP::does_self_intersect<CGAL::Parallel_if_available_tag>(mesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, mesh)));
  std::cout << (intersecting ? "There are self-intersections." : "There is no self-intersection.") << std::endl;
  std::vector<std::pair<face_descriptor, face_descriptor>> intersected_faces;
  PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(mesh), mesh, std::back_inserter(intersected_faces));
  std::cout << "\t " << intersected_faces.size() << " pairs of faces intersect." << std::endl;
  std::cout << "" << std::endl;

  if (output_interesecting_faces)
  {
    // Export self interecting faces
    std::vector<Surface_mesh::Vertex_index> verts;
    std::vector<Surface_mesh::Vertex_index> vd;

    int count = 0;

    for (auto face : intersected_faces)
    {
      Surface_mesh test;

      verts.clear();
      vd.clear();

      for (auto vh : CGAL::vertices_around_face(mesh.halfedge(face.first), mesh))
      {
        verts.push_back(vh);
      }

      for (auto v : verts)
      {
        vd.push_back(test.add_vertex(mesh.point(v))) ;
      }

      test.add_face(vd);

      verts.clear();
      vd.clear();

      for (auto vh : CGAL::vertices_around_face(mesh.halfedge(face.second), mesh))
      {
        verts.push_back(vh);
      }

      for (auto v : verts)
      {
        vd.push_back(test.add_vertex(mesh.point(v))) ;
      }

      test.add_face(vd);

      std::string fname = "pair_" + std::to_string(count) + ".obj";
      CGAL::IO::write_polygon_mesh(fname, test, CGAL::parameters::stream_precision(17));
      count += 1;
    }
  }

  if (CGAL::is_closed(mesh) && CGAL::is_valid(mesh))
  {
    PMP::orient_to_bound_a_volume(mesh);
    CGAL::IO::write_polygon_mesh(outfilename, mesh, CGAL::parameters::stream_precision(17));
    return 0;
  }
  else
  {
    std::cout << "Could not clean the 3D object" << std::endl;
    return 3;
  }

}
