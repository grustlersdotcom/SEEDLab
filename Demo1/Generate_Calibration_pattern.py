# This scrip generates the chessboard patterns that opencv uses to calibrate itself and get the camera matrix
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def generate_chessboard(square_size=1, grid_size=(9, 6), dpi=100):
    """
    Generate a chessboard calibration pattern.

    Args:
        square_size (int): Size of each square in the chessboard.
        grid_size (tuple): Number of inner corners per row and column.
        dpi (int): Resolution of the generated image.
    """
    # Create the figure and axis
    fig, ax = plt.subplots(figsize=(grid_size[0], grid_size[1]), dpi=dpi)

    # Loop to create the chessboard pattern
    for i in range(grid_size[0] + 1):
        for j in range(grid_size[1] + 1):
            if (i + j) % 2 == 0:
                # Add a black square
                ax.add_patch(patches.Rectangle((i * square_size, j * square_size),
                                               square_size, square_size,
                                               edgecolor='none', facecolor='black'))

    # Set limits and remove axes
    ax.set_xlim(0, (grid_size[0] + 1) * square_size)
    ax.set_ylim(0, (grid_size[1] + 1) * square_size)
    ax.set_aspect('equal')
    ax.axis('off')

    # Save as an image file
    plt.savefig('chessboard_calibration_pattern.png', bbox_inches='tight', pad_inches=0)
    print("Chessboard calibration pattern saved as 'chessboard_calibration_pattern.png'.")

# Generate a chessboard pattern with default settings
generate_chessboard(square_size=1, grid_size=(9, 6), dpi=300)
