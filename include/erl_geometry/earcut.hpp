/**
 * This file is adapted from https://github.com/mapbox/earcut.hpp
 *
 * Example:
 * // The number type to use for tessellation
 * using Coord = double;
 *
 * // The index type. Defaults to uint32_t, but you can also pass uint16_t if you know that your
 * // data won't have more than 65536 vertices.
 * using N = uint32_t;
 *
 * // Create array
 * using Point = std::array<Coord, 2>;
 * std::vector<std::vector<Point>> polygon;
 *
 * // Fill polygon structure with actual data. Any winding order works.
 * // The first polyline defines the main polygon.
 * polygon.push_back({{100, 0}, {100, 100}, {0, 100}, {0, 0}});
 * // Following polylines define holes.
 * polygon.push_back({{75, 25}, {75, 75}, {25, 75}, {25, 25}});
 *
 * // Run tessellation
 * // Returns array of indices that refer to the vertices of the input polygon.
 * // e.g: the index 6 would refer to {25, 75} in this example.
 * // Three subsequent indices form a triangle. Output triangles are clockwise.
 * std::vector<N> triangles_vertex_indices = erl::geometry::earcut<N>(polygon);
 */

#pragma once
#include "erl_common/logging.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace erl::geometry {

    namespace earcut_util {

        template<std::size_t I, typename T>
        struct Nth {
            static std::tuple_element_t<I, T>
            Get(const T& t) {
                return std::get<I>(t);
            }
        };

        // Eigen support
        template<>
        struct Nth<0, Eigen::Vector2d> {
            static auto
            Get(const Eigen::Vector2d& t) {
                return t[0];
            }
        };

        template<>
        struct Nth<1, Eigen::Vector2d> {
            static auto
            Get(const Eigen::Vector2d& t) {
                return t[1];
            }
        };

    }  // namespace earcut_util

    template<typename N = uint32_t>
    class Earcut {
        bool m_hashing_{};
        double m_min_x_{};
        double m_max_x_{};
        double m_min_y_{};
        double m_max_y_{};
        double m_inv_size_ = 0;

        template<typename T, typename Alloc = std::allocator<T>>
        class ObjectPool {
        public:
            ObjectPool() = default;

            explicit ObjectPool(const std::size_t block_size) { Reset(block_size); }

            ~ObjectPool() { Clear(); }

            template<typename... Args>
            T*
            Construct(Args&&... args) {
                if (m_current_index_ >= m_block_size_) {
                    m_current_block_ = AllocTraits::allocate(m_alloc_, m_block_size_);
                    m_allocations_.emplace_back(m_current_block_);
                    m_current_index_ = 0;
                }
                T* object = &m_current_block_[m_current_index_++];
                AllocTraits::construct(m_alloc_, object, std::forward<Args>(args)...);
                return object;
            }

            void
            Reset(const std::size_t new_block_size) {
                for (auto allocation: m_allocations_) { AllocTraits::deallocate(m_alloc_, allocation, m_block_size_); }
                m_allocations_.clear();
                m_block_size_ = std::max<std::size_t>(1, new_block_size);
                m_current_block_ = nullptr;
                m_current_index_ = m_block_size_;
            }

            void
            Clear() {
                Reset(m_block_size_);
            }

        private:
            T* m_current_block_ = nullptr;
            std::size_t m_current_index_ = 1;
            std::size_t m_block_size_ = 1;
            std::vector<T*> m_allocations_;
            Alloc m_alloc_;
            using AllocTraits = std::allocator_traits<Alloc>;
        };

        struct Node {
            const N i;
            const double x;
            const double y;

            // previous and next vertice nodes in a polygon ring
            Node* prev = nullptr;
            Node* next = nullptr;

            // z-order curve value
            int32_t z = 0;

            // previous and next nodes in z-order
            Node* prev_z = nullptr;
            Node* next_z = nullptr;

            // indicates whether this is a steiner point
            bool steiner = false;

            Node(N index, const double x, const double y)
                : i(index),
                  x(x),
                  y(y) {}

            Node(const Node&) = delete;
            Node&
            operator=(const Node&) = delete;
            Node(Node&&) = delete;
            Node&
            operator=(Node&&) = delete;
        };

        ObjectPool<Node> m_nodes_;

    public:
        std::vector<N> indices;
        std::size_t vertices = 0;

        template<typename Polygon>
        void
        operator()(const Polygon& points);

    private:
        // create a circular doubly linked list from polygon points in the specified winding order
        template<typename Ring>
        Node*
        LinkedList(const Ring& points, const bool clockwise) {
            using Point = typename Ring::value_type;
            double sum = 0;
            const std::size_t len = points.size();
            std::size_t i, j;
            Node* last = nullptr;

            // calculate original winding order of a polygon ring
            for (i = 0, j = len > 0 ? len - 1 : 0; i < len; j = i++) {
                const auto& p1 = points[i];
                const auto& p2 = points[j];
                const double p20 = earcut_util::Nth<0, Point>::Get(p2);
                const double p10 = earcut_util::Nth<0, Point>::Get(p1);
                const double p11 = earcut_util::Nth<1, Point>::Get(p1);
                const double p21 = earcut_util::Nth<1, Point>::Get(p2);
                sum += (p20 - p10) * (p11 + p21);
            }

            // link points into circular doubly-linked list in the specified winding order
            if (clockwise == (sum > 0)) {
                for (i = 0; i < len; i++) last = InsertNode(vertices + i, points[i], last);
            } else {
                for (i = len; i-- > 0;) last = InsertNode(vertices + i, points[i], last);
            }

            if (last && Equals(last, last->next)) {
                RemoveNode(last);
                last = last->next;
            }

            vertices += len;

            return last;
        }

        // eliminate colinear or duplicate points
        Node*
        FilterPoints(Node* start, Node* end = nullptr) {
            if (end == nullptr) { end = start; }

            Node* p = start;
            bool again;
            do {
                again = false;

                if (!p->steiner && (Equals(p, p->next) || Area(p->prev, p, p->next) == 0)) {
                    RemoveNode(p);
                    p = end = p->prev;

                    if (p == p->next) { break; }
                    again = true;

                } else {
                    p = p->next;
                }
            } while (again || p != end);

            return end;
        }

        // main ear slicing loop which triangulates a polygon (given as a linked list)
        void
        EarcutLinked(Node* ear, const int pass = 0) {
            if (ear == nullptr) { return; }

            // interlink polygon nodes in z-order
            if (pass == 0 && m_hashing_) { IndexCurve(ear); }

            Node* stop = ear;
            // iterate through ears, slicing them one by one
            while (ear->prev != ear->next) {
                Node* prev = ear->prev;
                Node* next = ear->next;

                if (m_hashing_ ? IsEarHashed(ear) : IsEar(ear)) {
                    // cut off the triangle
                    indices.emplace_back(prev->i);
                    indices.emplace_back(ear->i);
                    indices.emplace_back(next->i);

                    RemoveNode(ear);

                    // skipping the next vertice leads to less sliver triangles
                    ear = next->next;
                    stop = next->next;

                    continue;
                }

                ear = next;

                // if we looped through the whole remaining polygon and can't find any more ears
                if (ear == stop) {
                    // try filtering points and slicing again
                    if (pass == 0) {
                        EarcutLinked(FilterPoints(ear), 1);
                    }

                    // if this didn't work, try curing all small self-intersections locally
                    else if (pass == 1) {
                        ear = CureLocalIntersections(FilterPoints(ear));
                        EarcutLinked(ear, 2);  // as a last resort, try splitting the remaining polygon into two
                    } else if (pass == 2) {
                        SplitEarcut(ear);
                    }

                    break;
                }
            }
        }

        // check whether a polygon node forms a valid ear with adjacent nodes
        bool
        IsEar(Node* ear) {
            const Node* a = ear->prev;
            const Node* b = ear;
            const Node* c = ear->next;

            if (Area(a, b, c) >= 0) { return false; }  // reflex, can't be an ear

            // now make sure we don't have other points inside the potential ear
            Node* p = ear->next->next;

            while (p != ear->prev) {
                if (PointInTriangle(a->x, a->y, b->x, b->y, c->x, c->y, p->x, p->y) && Area(p->prev, p, p->next) >= 0) { return false; }
                p = p->next;
            }

            return true;
        }

        bool
        IsEarHashed(Node* ear) {
            const Node* a = ear->prev;
            const Node* b = ear;
            const Node* c = ear->next;

            if (Area(a, b, c) >= 0) { return false; }  // reflex, can't be an ear

            // triangle bbox; min & max are calculated like this for speed
            const double min_tx = std::min<double>(a->x, std::min<double>(b->x, c->x));
            const double min_ty = std::min<double>(a->y, std::min<double>(b->y, c->y));
            const double max_tx = std::max<double>(a->x, std::max<double>(b->x, c->x));
            const double max_ty = std::max<double>(a->y, std::max<double>(b->y, c->y));

            // z-order range for the current triangle bbox;
            const int32_t min_z = ZOrder(min_tx, min_ty);
            const int32_t max_z = ZOrder(max_tx, max_ty);

            // first look for points inside the triangle in increasing z-order
            Node* p = ear->next_z;

            while (p && p->z <= max_z) {
                if (p != ear->prev && p != ear->next && PointInTriangle(a->x, a->y, b->x, b->y, c->x, c->y, p->x, p->y) && Area(p->prev, p, p->next) >= 0) {
                    return false;
                }
                p = p->next_z;
            }

            // then look for points in decreasing z-order
            p = ear->prev_z;

            while (p && p->z >= min_z) {
                if (p != ear->prev && p != ear->next && PointInTriangle(a->x, a->y, b->x, b->y, c->x, c->y, p->x, p->y) && Area(p->prev, p, p->next) >= 0) {
                    return false;
                }
                p = p->prev_z;
            }

            return true;
        }

        // go through all polygon nodes and cure small local self-intersections
        Node*
        CureLocalIntersections(Node* start) {
            Node* p = start;
            do {
                Node* a = p->prev;
                Node* b = p->next->next;

                // a self-intersection where edge (v[i-1],v[i]) intersects (v[i+1],v[i+2])
                if (!Equals(a, b) && Intersects(a, p, p->next, b) && LocallyInside(a, b) && LocallyInside(b, a)) {
                    indices.emplace_back(a->i);
                    indices.emplace_back(p->i);
                    indices.emplace_back(b->i);

                    // remove two nodes involved
                    RemoveNode(p);
                    RemoveNode(p->next);

                    p = start = b;
                }
                p = p->next;
            } while (p != start);

            return FilterPoints(p);
        }

        // try splitting polygon into two and triangulate them independently
        void
        SplitEarcut(Node* start) {
            // look for a valid diagonal that divides the polygon into two
            Node* a = start;
            do {
                Node* b = a->next->next;
                while (b != a->prev) {
                    if (a->i != b->i && IsValidDiagonal(a, b)) {
                        // split the polygon in two by the diagonal
                        Node* c = SplitPolygon(a, b);

                        // filter colinear points around the cuts
                        a = FilterPoints(a, a->next);
                        c = FilterPoints(c, c->next);

                        // run earcut on each half
                        EarcutLinked(a);
                        EarcutLinked(c);
                        return;
                    }
                    b = b->next;
                }
                a = a->next;
            } while (a != start);
        }

        // link every hole into the outer loop, producing a single-ring polygon without holes
        template<typename Polygon>
        Node*
        EliminateHoles(const Polygon& points, Node* outer_node) {
            const size_t len = points.size();

            std::vector<Node*> queue;
            for (size_t i = 1; i < len; i++) {
                Node* list = LinkedList(points[i], false);
                if (list) {
                    if (list == list->next) list->steiner = true;
                    queue.push_back(GetLeftmost(list));
                }
            }
            std::sort(queue.begin(), queue.end(), [](const Node* a, const Node* b) { return a->x < b->x; });

            // process holes from left to right
            for (size_t i = 0; i < queue.size(); i++) { outer_node = EliminateHole(queue[i], outer_node); }

            return outer_node;
        }

        // find a bridge between vertices that connects hole with an outer ring and and link it
        Node*
        EliminateHole(Node* hole, Node* outer_node) {
            Node* bridge = FindHoleBridge(hole, outer_node);
            if (!bridge) { return outer_node; }

            Node* bridge_reverse = SplitPolygon(bridge, hole);

            // filter collinear points around the cuts
            FilterPoints(bridge_reverse, bridge_reverse->next);

            // Check if input node was removed by the filtering
            return FilterPoints(bridge, bridge->next);
        }

        // David Eberly's algorithm for finding a bridge between hole and outer polygon
        Node*
        FindHoleBridge(const Node* hole, Node* outer_node) {
            Node* p = outer_node;
            double hx = hole->x;
            double hy = hole->y;
            double qx = -std::numeric_limits<double>::infinity();
            Node* m = nullptr;

            // find a segment intersected by a ray from the hole's leftmost Vertex to the left;
            // segment's endpoint with lesser x will be potential connection Vertex
            do {
                if (hy <= p->y && hy >= p->next->y && p->next->y != p->y) {
                    if (const double x = p->x + (hy - p->y) * (p->next->x - p->x) / (p->next->y - p->y); x <= hx && x > qx) {
                        qx = x;
                        m = p->x < p->next->x ? p : p->next;
                        if (x == hx) return m;  // hole touches outer segment; pick leftmost endpoint
                    }
                }
                p = p->next;
            } while (p != outer_node);

            if (!m) { return nullptr; }

            // look for points inside the triangle of hole Vertex, segment intersection and endpoint;
            // if there are no points found, we have a valid connection;
            // otherwise choose the Vertex of the minimum angle with the ray as connection Vertex

            const Node* stop = m;
            double tan_min = std::numeric_limits<double>::infinity();

            p = m;
            const double mx = m->x;
            const double my = m->y;

            do {
                if (hx >= p->x && p->x >= mx && hx != p->x && PointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, p->x, p->y)) {

                    if (const double tan_cur = std::abs(hy - p->y) / (hx - p->x);
                        LocallyInside(p, hole) && (tan_cur < tan_min || (tan_cur == tan_min && (p->x > m->x || SectorContainsSector(m, p))))) {
                        m = p;
                        tan_min = tan_cur;
                    }
                }

                p = p->next;
            } while (p != stop);

            return m;
        }

        // whether sector in vertex m contains sector in vertex p in the same coordinates
        bool
        SectorContainsSector(const Node* m, const Node* p) {
            return Area(m->prev, m, p->prev) < 0 && Area(p->next, m, m->next) < 0;
        }

        // interlink polygon nodes in z-order
        void
        IndexCurve(Node* start) {
            assert(start);
            Node* p = start;

            do {
                p->z = p->z ? p->z : ZOrder(p->x, p->y);
                p->prev_z = p->prev;
                p->next_z = p->next;
                p = p->next;
            } while (p != start);

            p->prev_z->next_z = nullptr;
            p->prev_z = nullptr;

            SortLinked(p);
        }

        // Simon Tatham's linked list merge sort algorithm
        // http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
        Node*
        SortLinked(Node* list) {
            ERL_DEBUG_ASSERT(list != nullptr, "list should not be nullptr.");
            Node* e;
            uint32_t in_size = 1;

            while (true) {
                Node* p = list;
                list = nullptr;
                Node* tail = nullptr;
                int num_merges = 0;

                while (p) {
                    ++num_merges;
                    // step `insize` places along from p
                    Node* q = p;
                    uint32_t p_size = 0;
                    for (uint32_t i = 0; i < in_size; ++i) {
                        ++p_size;
                        q = q->next_z;
                        if (q == nullptr) { break; }  // early stop
                    }

                    uint32_t q_size = in_size;

                    while (p_size > 0 || (q_size > 0 && q != nullptr)) {

                        if (q_size == 0 || q == nullptr) {  // q is empty, e must come from p
                            e = p;
                            p = p->next_z;
                            p_size--;
                        } else {                // q is not empty
                            if (p_size == 0) {  // p is empty, e must come from q
                                e = q;
                                q = q->next_z;
                                q_size--;
                            } else {  // both p and q are not empty
                                if (p->z <= q->z) {
                                    e = p;
                                    p = p->next_z;
                                    p_size--;
                                } else {
                                    e = q;
                                    q = q->next_z;
                                    q_size--;
                                }
                            }
                        }

                        if (tail != nullptr) {
                            tail->next_z = e;
                        } else {
                            list = e;
                        }

                        e->prev_z = tail;
                        tail = e;
                    }

                    p = q;
                }

                if (tail != nullptr) { tail->next_z = nullptr; }
                if (num_merges <= 1) return list;
                in_size *= 2;
            }
        }

        // z-order of a Vertex given coords and size of the data bounding box
        [[nodiscard]] int32_t
        ZOrder(const double x, const double y) const {
            // coords are transformed into non-negative 15-bit integer range
            auto x_int = static_cast<int32_t>((x - m_min_x_) * m_inv_size_);
            auto y_int = static_cast<int32_t>((y - m_min_y_) * m_inv_size_);

            x_int = (x_int | x_int << 8) & 0x00FF00FF;
            x_int = (x_int | x_int << 4) & 0x0F0F0F0F;
            x_int = (x_int | x_int << 2) & 0x33333333;
            x_int = (x_int | x_int << 1) & 0x55555555;

            y_int = (y_int | y_int << 8) & 0x00FF00FF;
            y_int = (y_int | y_int << 4) & 0x0F0F0F0F;
            y_int = (y_int | y_int << 2) & 0x33333333;
            y_int = (y_int | y_int << 1) & 0x55555555;

            return x_int | y_int << 1;
        }

        // find the leftmost node of a polygon ring
        Node*
        GetLeftmost(Node* start) {
            Node* p = start;
            Node* leftmost = start;
            do {
                if (p->x < leftmost->x || (p->x == leftmost->x && p->y < leftmost->y)) leftmost = p;
                p = p->next;
            } while (p != start);

            return leftmost;
        }

        // check if a point lies within a convex triangle
        [[nodiscard]] static bool
        PointInTriangle(
            const double ax,
            const double ay,
            const double bx,
            const double by,
            const double cx,
            const double cy,
            const double px,
            const double py) {
            return (cx - px) * (ay - py) >= (ax - px) * (cy - py) && (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
                   (bx - px) * (cy - py) >= (cx - px) * (by - py);
        }

        // check if a diagonal between two polygon nodes is valid (lies in polygon interior)
        bool
        IsValidDiagonal(const Node* a, const Node* b) {
            return a->next->i != b->i && a->prev->i != b->i && !IntersectsPolygon(a, b) &&              // dones't intersect other edges
                   ((LocallyInside(a, b) && LocallyInside(b, a) && MiddleInside(a, b) &&                // locally visible
                     (Area(a->prev, a, b->prev) != 0.0 || Area(a, b->prev, b) != 0.0)) ||               // does not create opposite-facing sectors
                    (Equals(a, b) && Area(a->prev, a, a->next) > 0 && Area(b->prev, b, b->next) > 0));  // special zero-length case
        }

        // signed area of a triangle
        static double
        Area(const Node* p, const Node* q, const Node* r) {
            return (q->y - p->y) * (r->x - q->x) - (q->x - p->x) * (r->y - q->y);
        }

        // check if two points are equal
        static bool
        Equals(const Node* p1, const Node* p2) {
            return p1->x == p2->x && p1->y == p2->y;
        }

        // check if two segments intersect
        bool
        Intersects(const Node* p1, const Node* q1, const Node* p2, const Node* q2) {
            const int o1 = Sign(Area(p1, q1, p2));
            const int o2 = Sign(Area(p1, q1, q2));
            const int o3 = Sign(Area(p2, q2, p1));
            const int o4 = Sign(Area(p2, q2, q1));

            if (o1 != o2 && o3 != o4) return true;  // general case

            if (o1 == 0 && OnSegment(p1, p2, q1)) return true;  // p1, q1 and p2 are collinear and p2 lies on p1q1
            if (o2 == 0 && OnSegment(p1, q2, q1)) return true;  // p1, q1 and q2 are collinear and q2 lies on p1q1
            if (o3 == 0 && OnSegment(p2, p1, q2)) return true;  // p2, q2 and p1 are collinear and p1 lies on p2q2
            if (o4 == 0 && OnSegment(p2, q1, q2)) return true;  // p2, q2 and q1 are collinear and q1 lies on p2q2

            return false;
        }

        // for collinear points p, q, r, check if point q lies on segment pr
        static bool
        OnSegment(const Node* p, const Node* q, const Node* r) {
            return q->x <= std::max<double>(p->x, r->x) && q->x >= std::min<double>(p->x, r->x) && q->y <= std::max<double>(p->y, r->y) &&
                   q->y >= std::min<double>(p->y, r->y);
        }

        static int
        Sign(const double val) {
            return (0.0 < val) - (val < 0.0);
        }

        // check if a polygon diagonal intersects any polygon segments
        bool
        IntersectsPolygon(const Node* a, const Node* b) {
            const Node* p = a;
            do {
                if (p->i != a->i && p->next->i != a->i && p->i != b->i && p->next->i != b->i && Intersects(p, p->next, a, b)) return true;
                p = p->next;
            } while (p != a);

            return false;
        }

        // check if a polygon diagonal is locally inside the polygon
        bool
        LocallyInside(const Node* a, const Node* b) {
            return Area(a->prev, a, a->next) < 0 ? Area(a, b, a->next) >= 0 && Area(a, a->prev, b) >= 0 : Area(a, b, a->prev) < 0 || Area(a, a->next, b) < 0;
        }

        // check if the middle Vertex of a polygon diagonal is inside the polygon
        bool
        MiddleInside(const Node* a, const Node* b) {
            const Node* p = a;
            bool inside = false;
            double px = (a->x + b->x) / 2;
            double py = (a->y + b->y) / 2;
            do {
                if (((p->y > py) != (p->next->y > py)) && (p->next->y != p->y) && (px < (p->next->x - p->x) * (py - p->y) / (p->next->y - p->y) + p->x)) {
                    inside = !inside;
                }
                p = p->next;
            } while (p != a);

            return inside;
        }

        // link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits
        // polygon into two; if one belongs to the outer ring and another to a hole, it merges it into a
        // single ring
        Node*
        SplitPolygon(Node* a, Node* b) {
            Node* a2 = m_nodes_.Construct(a->i, a->x, a->y);
            Node* b2 = m_nodes_.Construct(b->i, b->x, b->y);
            Node* an = a->next;
            Node* bp = b->prev;

            a->next = b;
            b->prev = a;

            a2->next = an;
            an->prev = a2;

            b2->next = a2;
            a2->prev = b2;

            bp->next = b2;
            b2->prev = bp;

            return b2;
        }

        // create a node and util::optionally link it with previous one (in a circular doubly linked list)
        template<typename Point>
        Node*
        InsertNode(std::size_t i, const Point& pt, Node* last) {
            Node* p = m_nodes_.Construct(static_cast<N>(i), earcut_util::Nth<0, Point>::Get(pt), earcut_util::Nth<1, Point>::Get(pt));

            if (!last) {
                p->prev = p;
                p->next = p;

            } else {
                assert(last);
                p->next = last->next;
                p->prev = last;
                last->next->prev = p;
                last->next = p;
            }
            return p;
        }

        static void
        RemoveNode(Node* p) {
            p->next->prev = p->prev;
            p->prev->next = p->next;

            if (p->prev_z) p->prev_z->next_z = p->next_z;
            if (p->next_z) p->next_z->prev_z = p->prev_z;
        }
    };

    template<typename N>
    template<typename Polygon>
    void
    Earcut<N>::operator()(const Polygon& points) {
        // reset
        indices.clear();
        vertices = 0;

        if (points.empty()) return;

        int threshold = 80;
        std::size_t len = 0;

        for (size_t i = 0; threshold >= 0 && i < points.size(); i++) {
            threshold -= static_cast<int>(points[i].size());
            len += points[i].size();
        }

        // estimate size of nodes and indices
        m_nodes_.Reset(len * 3 / 2);
        indices.reserve(len + points[0].size());

        Node* outer_node = LinkedList(points[0], true);
        if (!outer_node || outer_node->prev == outer_node->next) return;

        if (points.size() > 1) outer_node = EliminateHoles(points, outer_node);

        // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
        m_hashing_ = threshold < 0;
        if (m_hashing_) {
            Node* p = outer_node->next;
            m_min_x_ = m_max_x_ = outer_node->x;
            m_min_y_ = m_max_y_ = outer_node->y;
            do {
                double x = p->x;
                double y = p->y;
                m_min_x_ = std::min<double>(m_min_x_, x);
                m_min_y_ = std::min<double>(m_min_y_, y);
                m_max_x_ = std::max<double>(m_max_x_, x);
                m_max_y_ = std::max<double>(m_max_y_, y);
                p = p->next;
            } while (p != outer_node);

            // minX, minY and inv_size are later used to transform coords into integers for z-order calculation
            m_inv_size_ = std::max<double>(m_max_x_ - m_min_x_, m_max_y_ - m_min_y_);
            m_inv_size_ = m_inv_size_ != .0 ? 32767. / m_inv_size_ : .0;
        }

        EarcutLinked(outer_node);

        m_nodes_.Clear();
    }

    template<typename N = uint32_t, typename Polygon>
    std::vector<N>
    RunEarcut(const Polygon& poly) {
        Earcut<N> earcut;
        earcut(poly);
        return std::move(earcut.indices);
    }
}  // namespace erl::geometry
