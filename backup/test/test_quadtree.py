from erl_geometry.geometry import IncrementalQuadtree
import unittest


class TestQuadtree(unittest.TestCase):
    def test_ray_tracing(self):
        # Create a quadtree
        quadtree = IncrementalQuadtree(0, 0, 10, 10)


if __name__ == "__main__":
    unittest.main()
