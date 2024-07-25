import cv2
import numpy as np
import matplotlib.pyplot as plt


def main():

    image = np.zeros((10, 10), dtype=np.uint8)
    plt.subplot(2, 2, 1)
    plt.imshow(image)

    image = cv2.drawContours(image, [np.array([[0, 0], [2, 3], [0, 0]])], 0, 255, -1)
    plt.subplot(2, 2, 2)
    plt.imshow(image)

    image = cv2.circle(image, (5, 5), 1, 255, -1)
    plt.subplot(2, 2, 3)
    plt.imshow(image)

    image = cv2.rectangle(image, (8, 8), (8, 8), 255, -1)
    plt.subplot(2, 2, 4)
    plt.imshow(image)

    plt.show()


if __name__ == "__main__":
    main()
