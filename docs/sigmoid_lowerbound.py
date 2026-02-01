import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np


def sigmoid(x):
    return 1 / (1 + np.exp(-x))


def lam_xi(xi):
    t0 = sigmoid(xi)
    lam = (0.5 - t0) / (2 * xi)
    lam[xi == 0] = -0.125  # handle the case when xi = 0
    return t0, lam


def sigmoid_lower(x, xi):
    t1 = (x - xi) / 2
    t0, lam = lam_xi(xi)
    return lam, t0 * np.exp(t1 + lam * (x**2 - xi**2))


class App:
    def __init__(self):
        self.x = 0
        self.xi = np.linspace(-10, 10, 400)

        self.fig, self.ax = plt.subplots()
        plt.subplots_adjust(bottom=0.2)
        self.vline_x = self.ax.vlines(self.x, -0.1, 1.1, colors="r", linestyles="dashed")
        self.hline_y = self.ax.hlines(sigmoid(self.x), -5, 5, colors="r", linestyles="dashed")
        # self.sigmoid_line = self.ax.plot(self.xi, sigmoid(self.xi), label="Sigmoid", color="blue")[0]
        y1, y2 = sigmoid_lower(self.x, self.xi)
        self.lam_line = self.ax.plot(self.xi, y1, label="Lambda", color="green")[0]
        self.lowerbound_line = self.ax.plot(self.xi, y2, label="Lower Bound", color="orange")[0]
        y_min = min(y1.min(), y2.min())
        y_max = max(y1.max(), y2.max())
        self.ax.set_ylim(y_min - 0.01, y_max + 0.01)
        self.ax.legend()
        self.slider_ax = self.fig.add_axes([0.2, 0.05, 0.65, 0.03])
        self.slider = Slider(self.slider_ax, "x", -5.0, 5.0, valinit=self.x)

        self.slider.on_changed(self.update)

    def update(self, val):
        self.x = val
        self.vline_x.set_segments([[[self.x, -0.1], [self.x, 1.1]]])
        y = sigmoid(self.x)
        self.hline_y.set_segments([[[-5, y], [5, y]]])
        y1, y2 = sigmoid_lower(self.x, self.xi)
        self.lam_line.set_ydata(y1)
        self.lowerbound_line.set_ydata(y2)
        y_min = min(y1.min(), y2.min())
        y_max = max(y1.max(), y2.max())
        self.ax.set_ylim(y_min - 0.01, y_max + 0.01)
        self.fig.canvas.draw_idle()

    def run(self):
        plt.show()


def main():
    app = App()
    app.run()


if __name__ == "__main__":
    main()
