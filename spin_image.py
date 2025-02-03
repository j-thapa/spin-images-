import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA

def compute_spin_image_with_support_angle(oriented_point, chunk, support_radius, bin_size, resolution, support_angle):
    """
    Compute a spin image for the given oriented point and chunk, considering support angle.
    
    Args:
        oriented_point (tuple): A tuple (point, normal) where:
            - point is the 3D location of the oriented point.
            - normal is the unit normal vector at the oriented point.
        chunk (o3d.geometry.PointCloud): Section of the surface mesh or point cloud.
        support_radius (float): Neighborhood radius for considering points.
        bin_size (float): Size of each bin in the spin image.
        resolution (int): Resolution of the spin image (number of bins per axis).
        support_angle (float): Maximum allowable angle (in radians) for point contribution.
        
    Returns:
        np.ndarray: Spin image as a 2D histogram.
    """
    O = np.asarray(oriented_point[0])  # Reference point
    n_A = np.asarray(oriented_point[1])  # Normal at the reference point

   

    # Initialize spin image
    SI = np.zeros((resolution, resolution))

    # Process points in the chunk
    points = np.asarray(chunk.points)
    normals = np.asarray(chunk.normals)
    cos_threshold = np.cos(support_angle)  # Precompute cosine of the support angle

    for x, n_B in zip(points, normals):
        # Check support angle constraint
        if np.dot(n_A, n_B) < cos_threshold:
            continue  # Skip points outside the support angle

        # Compute α and β
        d = x - O
        alpha = np.sqrt(np.linalg.norm(d) ** 2 - (np.dot(n_A, d)) ** 2)  # α
        beta = np.dot(n_A, d)                                           # β

        # Map α, β to (i, j) bins
        i = int(np.floor((resolution / 2 - beta / bin_size)))
        j = int(np.floor(alpha / bin_size))

        if 0 <= i < resolution - 1 and 0 <= j < resolution - 1:
            # Calculate bilinear weights
            a = (alpha / bin_size) - j
            b = (resolution / 2 - beta / bin_size) - i

            # Update spin image using bilinear interpolation
            SI[i, j] += (1 - a) * (1 - b)
            SI[i + 1, j] += a * (1 - b)
            SI[i, j + 1] += (1 - a) * b
            SI[i + 1, j + 1] += a * b

    return SI



def visualize_chunks_and_spin_images(mesh, chunks, support_radius, bin_size, resolution, support_angle):
    """
    Visualize chunks of a surface mesh and their corresponding spin images.

    Args:
        mesh (o3d.geometry.PointCloud): The full mesh or point cloud.
        chunks (list of list of int): List of index lists defining the chunks.
        support_radius (float): Neighborhood radius for spin image computation.
        bin_size (float): Bin size for spin image histogram.
        resolution (int): Resolution of the spin image (number of bins per axis).
    """
    fig, axs = plt.subplots(len(chunks), 3, figsize=(12, 4 * len(chunks)))
    if len(chunks) == 1:
        axs = [axs]

    for idx, (chunk_indices, ax_row) in enumerate(zip(chunks, axs)):
        # Extract chunk
        chunk = mesh.select_by_index(chunk_indices)
        chunk.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.1, max_nn=30))

        # Select an oriented point from the chunk
        oriented_point = (chunk.points[500], chunk.normals[500])

        # Compute spin image for the chunk
       
        spin_image = compute_spin_image_with_support_angle(oriented_point, chunk, support_radius, bin_size, resolution, support_angle)

        # Visualize
        # 1. 3D chunk
        points = np.asarray(chunk.points)
        ax_row[0].scatter(points[:, 0], points[:, 1], c=points[:, 2], cmap='viridis')
        ax_row[0].set_title(f"3D Chunk {idx + 1}")
        ax_row[0].set_xlabel("X")
        ax_row[0].set_ylabel("Y")

        # 2. 2D projection (using PCA for visualization)
        pca = PCA(n_components=2)
        points_2d = pca.fit_transform(points)
        ax_row[1].scatter(points_2d[:, 0], points_2d[:, 1], c='black', s=1)
        ax_row[1].set_title(f"2D Projection {idx + 1}")
        ax_row[1].set_xlabel("α")
        ax_row[1].set_ylabel("β")

        # 3. Spin Image
        ax_row[2].imshow(spin_image, cmap='jet', interpolation='nearest')
        ax_row[2].set_title(f"Spin Image {idx + 1}")
        ax_row[2].axis('off')

    plt.tight_layout()
    plt.show()


def main():
    # Load the surface mesh or point cloud
    mesh = o3d.io.read_point_cloud("file.pcd")  # Replace with file name
    mesh.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))
    mesh.orient_normals_towards_camera_location(camera_location=np.array([0.0,1.0,0.0]))

    # Define chunks using lists of indices
    chunks = [list(range(0, 2500)),
        list(range(2500, 5000)),           # Example indices for Chunk 2
        list(range(5000, 8000))            # Example indices for Chunk 3
    ]

    # Spin image parameters
    support_radius = 0.05 # Define based on object scale; radius to consider to get spin image of a point
    bin_size = 0.007   # Bin size in 3D space; how to round off bins if bigger size more spin maps will fall into one bin
    resolution = 80 # Spin image resolution (64x64 bins), determine numer of bins\
    support_angle = 140 #angle condition; honly points within a certain angular range relative to the oriented point's normal contribute to the spin image; reduce the effects of self-occlusion and clutter.

    # Visualize chunks and their spin images
    visualize_chunks_and_spin_images(mesh, chunks, support_radius, bin_size, resolution,support_angle)


if __name__ == "__main__":
    main()




