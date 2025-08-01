/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * https://octomap.github.io/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCTOMAP_MAP_NODE_H
#define OCTOMAP_MAP_NODE_H

#include <string>
#include <octomap/OcTree.h>

namespace octomap {

    template<class TREETYPE>
    class MapNode {

    public:
        MapNode();
        MapNode(TREETYPE* node_map, pose6d origin);
        MapNode(std::string filename, pose6d origin);
        MapNode(const Pointcloud& cloud, pose6d origin);
        ~MapNode();

        typedef TREETYPE TreeType;

        TREETYPE*
        getMap() {
            return node_map;
        }

        void
        updateMap(const Pointcloud& cloud, point3d sensor_origin);

        inline std::string
        getId() {
            return id;
        }

        inline void
        setId(std::string newid) {
            id = newid;
        }

        inline pose6d
        getOrigin() {
            return origin;
        }

        // returns cloud of voxel centers in global reference frame
        Pointcloud
        generatePointcloud();
        bool
        writeMap(std::string filename);

    protected:
        TREETYPE* node_map;  // occupancy grid map
        pose6d origin;       // origin and orientation relative to parent
        std::string id;

        void
        clear();
        bool
        readMap(std::string filename);
    };

}  // namespace octomap

#include "octomap/MapNode.hxx"

#endif
