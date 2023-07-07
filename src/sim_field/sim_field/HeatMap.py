import numpy as np

def createHeatmap(rows, cols, hotspot_coords, sigma=10):
        """
        Create a 2D heatmap with a hotspot at a given row and column.

        :param rows: Number of rows in the heatmap
        :param cols: Number of columns in the heatmap
        :param hotspot_row: The row index for the hotspot
        :param hotspot_col: The column index for the hotspot
        :param sigma: The standard deviation of the Gaussian, controlling the "spread" of the hotspot
        """
        # Create a grid of coordinates
        y = np.arange(rows)
        x = np.arange(cols)
        X, Y = np.meshgrid(x, y)
        hotspot_col = hotspot_coords[1]
        hotspot_row = hotspot_coords[0]


        # Create the Gaussian function centered at the hotspot
        heatmap = np.exp(-((X - hotspot_col)**2 + (Y - hotspot_row)**2) / (2 * sigma**2))

        return heatmap