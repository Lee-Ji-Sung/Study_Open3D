#####################################################################
#   14_Distance_Queries.py
#       - This shows how to use Distance_Queries in Open3D
#           . converting a mesh to an implicit representation
#           . computing distance with closet point queries
#       - open3d version : 0.15.1
#   written by jslee
#   date : 2000.00.00
#####################################################################




import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d



def compute_signed_distance_and_closest_goemetry(query_points: np.ndarray):
    closest_points = scene.compute_closest_points(query_points)
    distance = np.linalg.norm(query_points - closest_points['points'].numpy(),
                              axis=-1)
    rays = np.concatenate([query_points, np.ones_like(query_points)], axis=-1)
    intersection_counts = scene.count_intersections(rays).numpy()
    is_inside = intersection_counts % 2 == 1
    distance[is_inside] *= -1
    return distance, closest_points['geometry_ids'].numpy()



if __name__ == '__main__':


    ######################################################################################
    # converting a mesh to an implicit representation
    ######################################################################################
    ### Initialization
    # Load mesh and convert to open3d.t.geometry.TriangleMesh
    armadillo_data = o3d.data.ArmadilloMesh()
    mesh = o3d.io.read_triangle_mesh(armadillo_data.path)
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)  # we do not need the geometry ID for mesh




    ### Computing distance and occupancy for a single point
    query_point = o3d.core.Tensor([[10, 10, 10]], dtype=o3d.core.Dtype.Float32)

    # Compute distance of the query point from the surface
    unsigned_distance = scene.compute_distance(query_point)
    signed_distance = scene.compute_signed_distance(query_point)
    occupancy = scene.compute_occupancy(query_point)

    print("unsigned distance", unsigned_distance.numpy())
    print("signed_distance", signed_distance.numpy())
    print("occupancy", occupancy.numpy())


    ### Computing distance for multiple points and grids
    min_bound = mesh.vertex['positions'].min(0).numpy()
    max_bound = mesh.vertex['positions'].max(0).numpy()

    N = 256
    query_points = np.random.uniform(low=min_bound, high=max_bound,
                                     size=[N, 3]).astype(np.float32)

    # Compute the signed distance for N random points
    signed_distance = scene.compute_signed_distance(query_points)




    xyz_range = np.linspace(min_bound, max_bound, num=32)

    # query_points is a [32,32,32,3] array ..
    query_points = np.stack(np.meshgrid(*xyz_range.T), axis=-1).astype(np.float32)

    # signed distance is a [32,32,32] array
    signed_distance = scene.compute_signed_distance(query_points)

    # We can visualize a slice of the distance field directly with matplotlib
    plt.imshow(signed_distance.numpy()[:, :, 15])
    ######################################################################################




    ######################################################################################
    # computing distance with closet point queries
    ######################################################################################
    ### Initialization
    cube = o3d.t.geometry.TriangleMesh.from_legacy(
        o3d.geometry.TriangleMesh.create_box().translate([-1.2, -1.2, 0]))
    sphere = o3d.t.geometry.TriangleMesh.from_legacy(
        o3d.geometry.TriangleMesh.create_sphere(0.5).translate([0.7, 0.8, 0]))

    scene = o3d.t.geometry.RaycastingScene()
    # Add triangle meshes and remember ids
    mesh_ids = {}
    mesh_ids[scene.add_triangles(cube)] = 'cube'
    mesh_ids[scene.add_triangles(sphere)] = 'sphere'


    ### Computing the closest points on the surface
    query_point = o3d.core.Tensor([[0, 0, 0]], dtype=o3d.core.Dtype.Float32)

    # We compute the closest point on the surface for the point at position [0,0,0].
    ans = scene.compute_closest_points(query_point)

    # Compute_closest_points provides the point on the surface, the geometry id,
    # and the primitive id.
    # The dictionary keys are
    # .    points
    # .    geometry_ids
    # .    primitive_ids
    print('The closest point on the surface is', ans['points'].numpy())
    print('The closest point is on the surface of the',
          mesh_ids[ans['geometry_ids'][0].item()])
    print('The closest point belongs to triangle', ans['primitive_ids'][0].item())



    rays = np.concatenate(
        [query_point.numpy(),
         np.ones(query_point.shape, dtype=np.float32)],
        axis=-1)
    intersection_counts = scene.count_intersections(rays).numpy()
    # A point is inside if the number of intersections with the scene is even
    # This sssumes that inside and outside is we ll defined for the scene.
    is_inside = intersection_counts % 2 == 1



    # compute range
    xyz_range = np.linspace([-2, -2, -2], [2, 2, 2], num=32)
    # query_points is a [32,32,32,3] array ..
    query_points = np.stack(np.meshgrid(*xyz_range.T), axis=-1).astype(np.float32)

    sdf, closest_geom = compute_signed_distance_and_closest_goemetry(query_points)

    # We can visualize a slice of the grids directly with matplotlib
    fig, axes = plt.subplots(1, 2)
    axes[0].imshow(sdf[:, :, 16])
    axes[1].imshow(closest_geom[:, :, 16])
    ######################################################################################

    print(f'Done')