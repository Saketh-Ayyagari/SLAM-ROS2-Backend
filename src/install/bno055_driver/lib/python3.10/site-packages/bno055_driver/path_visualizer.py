import matplotlib.pyplot as plt

class PathVisualizer: # used for visualizing odometry data
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # Lock the aspect ratio and set equal scaling for x and y
        self.ax.set_aspect('equal', adjustable='box')

        self.path_time, self.path_x = [], []

    def visualize(self, time, x):
        # Store position history
        self.path_time.append(time)
        self.path_x.append(x)

        # Plot path
        self.ax.clear()
        self.ax.plot(self.path_time, self.path_x, 'b-', label='Path')
        self.ax.plot(time, x, 'ro', label='Current Position')  # Mark current position
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('X Position (m)')
        self.ax.set_title('Debugging Plot')
        self.ax.legend()
        plt.draw()
        plt.pause(0.001)  # Short pause to update plot
