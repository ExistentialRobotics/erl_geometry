from erl_geometry import PyObjectOccupancyOctree, PyObjectOccupancyOctreeNode


octree = PyObjectOccupancyOctree()
node: PyObjectOccupancyOctreeNode = octree.insert_node(1, 1, 1, depth=0)  # 0 means to the deepest level

print(node.py_object)  # None initially

node.py_object = "Some object"
print(node.py_object)  # Should print "Some object"

node.py_object = [1, 2, 3]  # Assigning a list
print(node.py_object)  # Should print [1, 2, 3]

node2 = octree.search(1, 1, 1)  # Searching for the node we just inserted
print(node2.py_object)  # Should print [1, 2, 3]

octree.write_raw("test_octree.ot")  # Writing the octree to a file

octree2 = PyObjectOccupancyOctree()  # Creating a new octree
octree2.read_raw("test_octree.ot")  # Reading the octree from a file
node3 = octree2.search(1, 1, 1)  # Searching for the node we just inserted
print(node3.py_object)  # Should print [1, 2, 3]
