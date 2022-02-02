import open3d as o3d

print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("./rs_pc_test.pcd")  # format='xyzrgb'
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0, 0, 0])

o3d.visualization.draw_geometries(
    [pcd, mesh_frame],
    zoom=1,
    front=[0.4257, -0.2125, -0.8795],
    lookat=[2.6172, 2.0475, 1.532],
    up=[-0.0694, -0.9768, 0.2024],
)
