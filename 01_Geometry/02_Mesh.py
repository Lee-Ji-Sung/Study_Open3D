#####################################################################
#   02_Mesh.py
#       - This shows how to use Mesh in Open3D
#           . mesh
#           . visualize a 3D mesh
#           . Surface normal estimation
#           . crop mesh
#           . paint mesh
#           . mesh properties
#           . mesh filtering
#               - average filter
#               - laplacian
#               - taubin filter
#           . sampling
#           . mesh subdivision
#           . mesh simplification
#               - vertex clustering
#               - mesh decimation
#           . connected components
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################
import copy

import numpy as np

import open3d as o3d





# mesh properties
def check_properties(name, mesh):
    mesh.compute_vertex_normals()

    edge_manifold = mesh.is_edge_manifold(allow_boundary_edges=True)
    edge_manifold_boundary = mesh.is_edge_manifold(allow_boundary_edges=False)
    vertex_manifold = mesh.is_vertex_manifold()
    self_intersection = mesh.is_vertex_manifold()
    watertight = mesh.is_watertight()
    orientable = mesh.is_orientable()

    print(name)
    print(f' edge_manifold :             {edge_manifold}')
    print(f' edge_manifold_boundary :    {edge_manifold_boundary}')
    print(f' vertex_manifold :           {vertex_manifold}')
    print(f' self_intersection :         {self_intersection}')
    print(f' watertight :                {watertight}')
    print(f'orientable :                 {orientable}')

    geoms = [mesh]
    if not edge_manifold:
        edges = mesh.get_non_manifold_edges(allow_boundary_edges=True)
        geoms.append(o3dtut.edges_to_lineset(mesh, edges, (1, 0, 0)))
    if not edge_manifold_boundary:
        edges = mesh.get_non_manifold_edges(allow_boundary_edges=False)
        geoms.append(o3dtut.edges_to_lineset(mesh, edges, (0, 1, 0)))
    if not vertex_manifold:
        verts = np.asarray(mesh.get_non_manifold_vertices())
        pcl = o3d.geometry.PointCloud(points=o3d.utility.Vector3dVector(np.asarray(mesh.vertices)[verts]))
        pcl.paint_uniform_color((0, 0, 1))
        geoms.append(pcl)
    if self_intersection:
        intersection_triangles = np.asarray(mesh.get_self_intersecting_triangles())
        intersection_triangles = intersection_triangles[0:1]
        intersection_triangles = np.unique(intersection_triangles)
        print(f' # visualize self-intersecting triangles')
        triangles = np.asarray(mesh.triangles)[intersection_triangles]
        edges = [
            np.vstack((triangles[:, i], triangles[:, j]))
            for i, j in [(0, 1), (1, 2), (2, 0)]
        ]
        edges = np.hstack(edges).T
        edges = o3d.utility.Vector2iVector(edges)
        geoms.append(o3dtut.edges_to_lineset(mesh, edges, (1, 0, 1)))

    o3d.visualization.draw_geometries(geoms, mesh_show_back_face=True)





if __name__ == '__main__':



    ######################################################################################
    # mesh
    ######################################################################################
    print(f'Testing mesh in Open3D...')
    armadillo_mesh = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(armadillo_mesh.path)
    print(mesh)

    knot_mesh = o3d.data.KnotMesh()
    mesh = o3d.io.read_triangle_mesh(knot_mesh.path)
    print(mesh)
    print(f'Vertices :')
    print(np.asarray(mesh.vertices))
    print(f'Trinangles : ')
    print(np.asarray(mesh.triangles))
    ######################################################################################



    ######################################################################################
    # visualize a 3D mesh
    ######################################################################################
    print("Try to render a mesh with normals (exist : " +
          str(mesh.has_vertex_normals()) + " and colors (exist : " +
          str(mesh.has_vertex_colors()) + ")")

    o3d.visualization.draw_geometries([mesh])
    print(f'A mesh with no normals and no colors does not look good.')
    ######################################################################################



    ######################################################################################
    # surface normal estimation
    ######################################################################################
    print(f'computing normal and rendering it.')
    mesh.compute_vertex_normals()
    print(np.asarray(mesh.triangle_normals))
    o3d.visualization.draw_geometries([mesh])
    ######################################################################################



    ######################################################################################
    # crop mesh
    ######################################################################################
    print(f'We make a partial mesh of only the first half trinangles.')
    mesh1 = copy.deepcopy(mesh)
    mesh1.triangles = o3d.utility.Vector3iVector(np.asarray(mesh1.triangles)[:len(mesh1.triangles) // 2, :])
    mesh1.triangle_normals = o3d.utility.Vector3dVector(np.asarray(mesh1.triangle_normals)[:len(mesh1.triangle_normals) // 2, :])
    print(mesh1.triangles)
    o3d.visualization.draw_geometries([mesh1])
    ######################################################################################



    ######################################################################################
    # paint mesh
    ######################################################################################
    print(f'Painting the mesh')
    mesh1.paint_uniform_color([1, 0.7, 0])
    o3d.visualization.draw_geometries([mesh1])
    ######################################################################################



    ######################################################################################
    # mesh properties
    ######################################################################################
    """
    knot_mesh_data = o3d.data.KnotMesh()
    knot_mesh = o3d.io.read_triangle_mesh(knot_mesh_data.path)
    check_properties('KnotMesh', knot_mesh_data)
    check_properties('Mobius', o3d.geometry.TriangleMesh.create_moebius(twists=1))
    check_properties('non-manifold edge', o3dtut.get_non_manifold_edge_mesh())
    check_properties('not-manifold vertex', o3dtut.get_non_manifold_vertex_mesh())
    check_properties('open box', o3dtut.get_open_box_mesh())
    check_properties('intersection_boxes', o3dtut.get_intersecting_boxes_mesh())
    """
    ######################################################################################



    ######################################################################################
    # mesh filtering
    #   . average filter
    #   . Laplacian
    #   . Taubin filter
    ######################################################################################
    # average filter
    print(f'create noisy mesh')
    knot_mesh = o3d.data.KnotMesh()
    mesh_in = o3d.io.read_triangle_mesh(knot_mesh.path)
    vertices = np.asarray(mesh_in.vertices)
    noise = 5
    vertices += np.random.uniform(0, noise, size=vertices.shape)
    mesh_in.vertices = o3d.utility.Vector3dVector(vertices)
    mesh_in.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_in])

    print(f'filter with average with 1 iteration')
    mesh_out = mesh_in.filter_smooth_simple(number_of_iterations=1)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])

    print(f'filter with average with 5 iterations')
    mesh_out = mesh_in.filter_smooth_simple(number_of_iterations=5)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])



    # Laplacian
    print(f'filter with Laplacian with 10 iterations')
    mesh_out = mesh_in.filter_smooth_laplacian(number_of_iterations=10)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])

    print(f'filter with Laplacian with 50 iterations')
    mesh_out = mesh_in.filter_smooth_laplacian(number_of_iterations=50)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])


    # Taubin filter
    print(f'filter with Taubin with 10 iterations')
    mesh_out = mesh_in.filter_smooth_taubin(number_of_iterations=10)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])

    print(f'filter with Taubin with 100 iterations')
    mesh_out = mesh_in.filter_smooth_taubin(number_of_iterations=100)
    mesh_out.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh_out])
    ######################################################################################



    ######################################################################################
    # Sampling
    ######################################################################################
    # sphere
    mesh = o3d.geometry.TriangleMesh.create_sphere()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])
    pcd = mesh.sample_points_uniformly(number_of_points=500)
    o3d.visualization.draw_geometries([pcd])



    # bunny
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()

    o3d.visualization.draw_geometries([mesh])
    pcd = mesh.sample_points_uniformly(number_of_points=500)
    o3d.visualization.draw_geometries([pcd])


    # uniform sampling - sphere
    mesh = o3d.geometry.TriangleMesh.create_sphere()
    pcd = mesh.sample_points_poisson_disk(number_of_points=500, init_factor=5)
    o3d.visualization.draw_geometries([mesh])

    pcd = mesh.sample_points_uniformly(number_of_points=2500)
    pcd = mesh.sample_points_poisson_disk(number_of_points=500, pcl=pcd)
    o3d.visualization.draw_geometries([pcd])

    # uniform sampling - bunny
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)

    pcd = mesh.sample_points_poisson_disk(number_of_points=500, init_factor=5)
    o3d.visualization.draw_geometries([pcd])

    pcd = mesh.sample_points_uniformly(number_of_points=2500)
    pcd = mesh.sample_points_poisson_disk(number_of_points=500, pcl=pcd)
    o3d.visualization.draw_geometries([pcd])
    ######################################################################################



    ######################################################################################
    # mesh subdivision --> o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True) 이부분 error
    ######################################################################################
    """
    mesh = o3d.geometry.TriangleMesh.create_box()
    mesh.compute_vertex_normals()
    print(f'The mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)

    mesh = mesh.subdivide_midpoint(number_of_iterations=1)
    print(f'After subdivision it has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)


    # sphere exam
    mesh = o3d.geometry.TriangleMesh.create_sphere()
    mesh.compute_vertex_normals()
    print(f'The mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)

    mesh = mesh.subdivide_loop(number_of_iterations=2)
    print(f'After subdivision it has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)



    # knot - exam
    knot_mesh = o3d.data.KnotMesh()
    mesh = o3d.io.read_triangle_mesh(knot_mesh.path)
    mesh.compute_vertex_normals()
    print(f'The mesh has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)
    mesh = mesh.subdivide_loop(number_of_iterations=1)
    print(f'After subdivision it has {len(mesh.vertices)} vertices and {len(mesh.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh], zoom=0.8, mesh_show_wireframe=True)
    """
    ######################################################################################



    ######################################################################################
    # mesh simplification
    #   . vertex clustering
    #   . mesh decimation
    ######################################################################################
    # vertex clustering
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()
    print(f'Input mesh has {len(mesh_in.vertices)} vertices and {len(mesh_in.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh_in])

    voxel_size = max(mesh_in.get_max_bound() - mesh_in.get_min_bound()) / 32
    print(f'voxel_size = {voxel_size:e}')
    mesh_smp = mesh_in.simplify_vertex_clustering(voxel_size=voxel_size, contraction=o3d.geometry.SimplificationContraction.Average)
    print(f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh_smp])

    voxel_size = max(mesh_in.get_max_bound() - mesh_in.get_min_bound()) / 16
    print(f'voxel_size = {voxel_size:e}')
    mesh_smp = mesh_in.simplify_vertex_clustering(voxel_size=voxel_size, contraction=o3d.geometry.SimplificationContraction.Average)
    print(f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh_smp])



    # mesh decimation
    mesh_smp = mesh_in.simplify_quadric_decimation(target_number_of_triangles=6500)
    print(f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh_smp])

    mesh_smp = mesh_in.simplify_quadric_decimation(target_number_of_triangles=1700)
    print(f'Simplified mesh has {len(mesh_smp.vertices)} vertices and {len(mesh_smp.triangles)} triangles')
    o3d.visualization.draw_geometries([mesh_smp])
    ######################################################################################




    ######################################################################################
    # connected components
    ######################################################################################
    print(f'Generate data')
    bunny = o3d.data.BunnyMesh()
    mesh = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()

    mesh = mesh.subdivide_midpoint(number_of_iterations=2)
    vert = np.asarray(mesh.vertices)
    min_vert, max_vert = vert.min(axis=0), vert.max(axis=0)
    for _ in range(30):
        cube = o3d.geometry.TriangleMesh.create_box()
        cube.scale(0.005, center=cube.get_center())
        cube.translate(
            (
                np.random.uniform(min_vert[0], max_vert[0]),
                np.random.uniform(min_vert[1], max_vert[1]),
                np.random.uniform(min_vert[2], max_vert[2]),
            ),
            relative=False,
        )
        mesh += cube

    mesh.compute_vertex_normals()
    print(f'Show input mesh')
    o3d.visualization.draw_geometries([mesh])



    print(f'Cluster connected triangles')
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        triangle_clusters, cluster_n_triangles, cluster_area = (mesh.cluster_connected_triangles())
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    cluster_area = np.asarray(cluster_area)


    print(f'Show mesh with samll clusters removed')
    mesh_0 = copy.deepcopy(mesh)
    triangle_to_remove = cluster_n_triangles[triangle_clusters] < 100
    mesh_0.remove_triangles_by_mask(triangle_to_remove)
    o3d.visualization.draw_geometries([mesh_0])



    print(f'Show lagest cluster')
    mesh_1 = copy.deepcopy(mesh)
    lagest_cluster_idx = cluster_n_triangles.argmax()
    triangle_to_remove = triangle_clusters != lagest_cluster_idx
    mesh_1.remove_triangles_by_mask(triangle_to_remove)
    o3d.visualization.draw_geometries([mesh_1])
    ######################################################################################





    print('Done')