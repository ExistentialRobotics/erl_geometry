from erl_geometry import PyObjectOccupancyQuadtree, PyObjectOccupancyQuadtreeNode


quadtree = PyObjectOccupancyQuadtree()
node: PyObjectOccupancyQuadtreeNode = quadtree.insert_node(1, 1, depth=0)  # 0 means to the deepest level

print(node.py_object)  # None initially

node.py_object = "Some object"
print(node.py_object)  # Should print "Some object"

node.py_object = [1, 2, 3]  # Assigning a list
print(node.py_object)  # Should print [1, 2, 3]

node2 = quadtree.search(1, 1)  # Searching for the node we just inserted
print(node2.py_object)  # Should print [1, 2, 3]
