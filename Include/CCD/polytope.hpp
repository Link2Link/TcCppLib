//
// Created by Chenfei Wang on 7/7/2023.
//

#ifndef TCCPPLIB_POLYTOPE_HPP
#define TCCPPLIB_POLYTOPE_HPP


#include <cstdlib>
#include "support.hpp"
#include "list.hpp"
#include <cfloat>

namespace CCD {
#define CCD_PT_VERTEX 1
#define CCD_PT_EDGE   2
#define CCD_PT_FACE   3


#define __CCD_PT_EL \
    int type;           /*! type of element */ \
    ccd_real_t dist;        /*! distance from origin */ \
    ccd_vec3_t witness; /*! witness point of projection of origin */ \
    ccd_list_t list;    /*! list of elements of same type */

/**
 * General polytope element.
 * Could be vertex, edge or triangle.
 */
    struct _ccd_pt_el_t {
        __CCD_PT_EL
    };
    typedef struct _ccd_pt_el_t ccd_pt_el_t;

    struct _ccd_pt_edge_t;
    struct _ccd_pt_face_t;

/**
 * Polytope's vertex.
 */
    struct _ccd_pt_vertex_t {
        __CCD_PT_EL

        int id;
        ccd_support_t v;
        ccd_list_t edges; //!< List of edges
    };
    typedef struct _ccd_pt_vertex_t ccd_pt_vertex_t;
    using ccd_pt_vertex_t_ptr = ccd_pt_vertex_t *;

/**
 * Polytope's edge.
 */
    struct _ccd_pt_edge_t {
        __CCD_PT_EL

        ccd_pt_vertex_t *vertex[2]; //!< Reference to vertices
        struct _ccd_pt_face_t *faces[2]; //!< Reference to faces

        ccd_list_t vertex_list[2]; //!< List items in vertices' lists
    };
    typedef struct _ccd_pt_edge_t ccd_pt_edge_t;

/**
 * Polytope's triangle faces.
 */
    struct _ccd_pt_face_t {
        __CCD_PT_EL

        ccd_pt_edge_t *edge[3]; //!< Reference to surrounding edges
    };
    typedef struct _ccd_pt_face_t ccd_pt_face_t;


/**
 * Struct containing polytope.
 */
    struct _ccd_pt_t {
        ccd_list_t vertices; //!< List of vertices
        ccd_list_t edges; //!< List of edges
        ccd_list_t faces; //!< List of faces

        ccd_pt_el_t *nearest;
        ccd_real_t nearest_dist;
        int nearest_type;
    };
    typedef struct _ccd_pt_t ccd_pt_t;


    CCD_EXPORT void ccdPtInit(ccd_pt_t *pt);

    CCD_EXPORT void ccdPtDestroy(ccd_pt_t *pt);

/**
 * Returns vertices surrounding given triangle face.
 */
    _ccd_inline void ccdPtFaceVec3(const ccd_pt_face_t *face,
                                   ccd_vec3_t **a,
                                   ccd_vec3_t **b,
                                   ccd_vec3_t **c);

    _ccd_inline void ccdPtFaceVertices(const ccd_pt_face_t *face,
                                       ccd_pt_vertex_t **a,
                                       ccd_pt_vertex_t **b,
                                       ccd_pt_vertex_t **c);

    _ccd_inline void ccdPtFaceEdges(const ccd_pt_face_t *f,
                                    ccd_pt_edge_t **a,
                                    ccd_pt_edge_t **b,
                                    ccd_pt_edge_t **c);

    _ccd_inline void ccdPtEdgeVec3(const ccd_pt_edge_t *e,
                                   ccd_vec3_t **a,
                                   ccd_vec3_t **b);

    _ccd_inline void ccdPtEdgeVertices(const ccd_pt_edge_t *e,
                                       ccd_pt_vertex_t **a,
                                       ccd_pt_vertex_t **b);

    _ccd_inline void ccdPtEdgeFaces(const ccd_pt_edge_t *e,
                                    ccd_pt_face_t **f1,
                                    ccd_pt_face_t **f2);


/**
 * Adds vertex to polytope and returns pointer to newly created vertex.
 */
    CCD_EXPORT ccd_pt_vertex_t *ccdPtAddVertex(ccd_pt_t *pt, const ccd_support_t *v);

    _ccd_inline ccd_pt_vertex_t *ccdPtAddVertexCoords(ccd_pt_t *pt,
                                                      ccd_real_t x, ccd_real_t y, ccd_real_t z);

/**
 * Adds edge to polytope.
 */
    CCD_EXPORT ccd_pt_edge_t *ccdPtAddEdge(ccd_pt_t *pt, ccd_pt_vertex_t *v1,
                                           ccd_pt_vertex_t *v2);

/**
 * Adds face to polytope.
 */
    CCD_EXPORT ccd_pt_face_t *ccdPtAddFace(ccd_pt_t *pt, ccd_pt_edge_t *e1,
                                           ccd_pt_edge_t *e2,
                                           ccd_pt_edge_t *e3);

/**
 * Deletes vertex from polytope.
 * Returns 0 on success, -1 otherwise.
 */
    _ccd_inline int ccdPtDelVertex(ccd_pt_t *pt, ccd_pt_vertex_t *);

    _ccd_inline int ccdPtDelEdge(ccd_pt_t *pt, ccd_pt_edge_t *);

    _ccd_inline int ccdPtDelFace(ccd_pt_t *pt, ccd_pt_face_t *);


/**
 * Recompute distances from origin for all elements in pt.
 */
    CCD_EXPORT void ccdPtRecomputeDistances(ccd_pt_t *pt);

/**
 * Returns nearest element to origin.
 */
    CCD_EXPORT ccd_pt_el_t *ccdPtNearest(ccd_pt_t *pt);


//    CCD_EXPORT void ccdPtDumpSVT(ccd_pt_t *pt, const char *fn);

//    CCD_EXPORT void ccdPtDumpSVT2(ccd_pt_t *pt, FILE *);


/**** INLINES ****/
    _ccd_inline ccd_pt_vertex_t *ccdPtAddVertexCoords(ccd_pt_t *pt,
                                                      ccd_real_t x, ccd_real_t y, ccd_real_t z) {
        ccd_support_t s;
        ccdVec3Set(&s.v, x, y, z);
        return ccdPtAddVertex(pt, &s);
    }

    _ccd_inline int ccdPtDelVertex(ccd_pt_t *pt, ccd_pt_vertex_t *v) {
        // test if any edge is connected to this vertex
        if (!ccdListEmpty(&v->edges))
            return -1;

        // delete vertex from main list
        ccdListDel(&v->list);

        if ((void *) pt->nearest == (void *) v) {
            pt->nearest = NULL;
        }

        delete(v);
        return 0;
    }

    _ccd_inline int ccdPtDelEdge(ccd_pt_t *pt, ccd_pt_edge_t *e) {
        // text if any face is connected to this edge (faces[] is always
        // aligned to lower indices)
        if (e->faces[0] != NULL)
            return -1;

        // disconnect edge from lists of edges in vertex struct
        ccdListDel(&e->vertex_list[0]);
        ccdListDel(&e->vertex_list[1]);

        // disconnect edge from main list
        ccdListDel(&e->list);

        if ((void *) pt->nearest == (void *) e) {
            pt->nearest = NULL;
        }

        delete(e);
        return 0;
    }

    _ccd_inline int ccdPtDelFace(ccd_pt_t *pt, ccd_pt_face_t *f) {
        ccd_pt_edge_t *e;
        size_t i;

        // remove face from edges' recerence lists
        for (i = 0; i < 3; i++) {
            e = f->edge[i];
            if (e->faces[0] == f) {
                e->faces[0] = e->faces[1];
            }
            e->faces[1] = NULL;
        }

        // remove face from list of all faces
        ccdListDel(&f->list);

        if ((void *) pt->nearest == (void *) f) {
            pt->nearest = NULL;
        }

        delete(f);
        return 0;
    }

    _ccd_inline void ccdPtFaceVec3(const ccd_pt_face_t *face,
                                   ccd_vec3_t **a,
                                   ccd_vec3_t **b,
                                   ccd_vec3_t **c) {
        *a = &face->edge[0]->vertex[0]->v.v;
        *b = &face->edge[0]->vertex[1]->v.v;

        if (face->edge[1]->vertex[0] != face->edge[0]->vertex[0]
            && face->edge[1]->vertex[0] != face->edge[0]->vertex[1]) {
            *c = &face->edge[1]->vertex[0]->v.v;
        } else {
            *c = &face->edge[1]->vertex[1]->v.v;
        }
    }

    _ccd_inline void ccdPtFaceVertices(const ccd_pt_face_t *face,
                                       ccd_pt_vertex_t **a,
                                       ccd_pt_vertex_t **b,
                                       ccd_pt_vertex_t **c) {
        *a = face->edge[0]->vertex[0];
        *b = face->edge[0]->vertex[1];

        if (face->edge[1]->vertex[0] != face->edge[0]->vertex[0]
            && face->edge[1]->vertex[0] != face->edge[0]->vertex[1]) {
            *c = face->edge[1]->vertex[0];
        } else {
            *c = face->edge[1]->vertex[1];
        }
    }

    _ccd_inline void ccdPtFaceEdges(const ccd_pt_face_t *f,
                                    ccd_pt_edge_t **a,
                                    ccd_pt_edge_t **b,
                                    ccd_pt_edge_t **c) {
        *a = f->edge[0];
        *b = f->edge[1];
        *c = f->edge[2];
    }

    _ccd_inline void ccdPtEdgeVec3(const ccd_pt_edge_t *e,
                                   ccd_vec3_t **a,
                                   ccd_vec3_t **b) {
        *a = &e->vertex[0]->v.v;
        *b = &e->vertex[1]->v.v;
    }

    _ccd_inline void ccdPtEdgeVertices(const ccd_pt_edge_t *e,
                                       ccd_pt_vertex_t **a,
                                       ccd_pt_vertex_t **b) {
        *a = e->vertex[0];
        *b = e->vertex[1];
    }

    _ccd_inline void ccdPtEdgeFaces(const ccd_pt_edge_t *e,
                                    ccd_pt_face_t **f1,
                                    ccd_pt_face_t **f2) {
        *f1 = e->faces[0];
        *f2 = e->faces[1];
    }


    _ccd_inline void _ccdPtNearestUpdate(ccd_pt_t *pt, ccd_pt_el_t *el) {
        if (ccdEq(pt->nearest_dist, el->dist)) {
            if (el->type < pt->nearest_type) {
                pt->nearest = el;
                pt->nearest_dist = el->dist;
                pt->nearest_type = el->type;
            }
        } else if (el->dist < pt->nearest_dist) {
            pt->nearest = el;
            pt->nearest_dist = el->dist;
            pt->nearest_type = el->type;
        }
    }

    static void _ccdPtNearestRenew(ccd_pt_t *pt) {
        ccd_pt_vertex_t *v;
        ccd_pt_edge_t *e;
        ccd_pt_face_t *f;

        pt->nearest_dist = CCD_REAL_MAX;
        pt->nearest_type = 3;
        pt->nearest = NULL;

        ccdListForEachEntry(&pt->vertices, v, ccd_pt_vertex_t, list) {
            _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) v);
        }

        ccdListForEachEntry(&pt->edges, e, ccd_pt_edge_t, list) {
            _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) e);
        }

        ccdListForEachEntry(&pt->faces, f, ccd_pt_face_t, list) {
            _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) f);
        }
    }


    void ccdPtInit(ccd_pt_t *pt) {
        ccdListInit(&pt->vertices);
        ccdListInit(&pt->edges);
        ccdListInit(&pt->faces);

        pt->nearest = NULL;
        pt->nearest_dist = CCD_REAL_MAX;
        pt->nearest_type = 3;
    }

    void ccdPtDestroy(ccd_pt_t *pt) {
        ccd_pt_face_t *f, *f2;
        ccd_pt_edge_t *e, *e2;
        ccd_pt_vertex_t *v, *v2;

        // first delete all faces
        ccdListForEachEntrySafe(&pt->faces, f, ccd_pt_face_t, f2, ccd_pt_face_t, list) {
            ccdPtDelFace(pt, f);
        }

        // delete all edges
        ccdListForEachEntrySafe(&pt->edges, e, ccd_pt_edge_t, e2, ccd_pt_edge_t, list) {
            ccdPtDelEdge(pt, e);
        }

        // delete all vertices
        ccdListForEachEntrySafe(&pt->vertices, v, ccd_pt_vertex_t, v2, ccd_pt_vertex_t, list) {
            ccdPtDelVertex(pt, v);
        }
    }


    ccd_pt_vertex_t *ccdPtAddVertex(ccd_pt_t *pt, const ccd_support_t *v) {
        ccd_pt_vertex_t *vert;

        vert = new(ccd_pt_vertex_t);
        if (vert == NULL)
            return NULL;

        vert->type = CCD_PT_VERTEX;
        ccdSupportCopy(&vert->v, v);

        vert->dist = ccdVec3Len2(&vert->v.v);
        ccdVec3Copy(&vert->witness, &vert->v.v);

        ccdListInit(&vert->edges);

        // add vertex to list
        ccdListAppend(&pt->vertices, &vert->list);

        // update position in .nearest array
        _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) vert);

        return vert;
    }

    ccd_pt_edge_t *ccdPtAddEdge(ccd_pt_t *pt, ccd_pt_vertex_t *v1,
                                ccd_pt_vertex_t *v2) {
        const ccd_vec3_t *a, *b;
        ccd_pt_edge_t *edge;

        if (v1 == NULL || v2 == NULL)
            return NULL;

        edge = new(ccd_pt_edge_t);
        if (edge == NULL)
            return NULL;

        edge->type = CCD_PT_EDGE;
        edge->vertex[0] = v1;
        edge->vertex[1] = v2;
        edge->faces[0] = edge->faces[1] = NULL;

        a = &edge->vertex[0]->v.v;
        b = &edge->vertex[1]->v.v;
        edge->dist = ccdVec3PointSegmentDist2(ccd_vec3_origin, a, b, &edge->witness);

        ccdListAppend(&edge->vertex[0]->edges, &edge->vertex_list[0]);
        ccdListAppend(&edge->vertex[1]->edges, &edge->vertex_list[1]);

        ccdListAppend(&pt->edges, &edge->list);

        // update position in .nearest array
        _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) edge);

        return edge;
    }

    ccd_pt_face_t *ccdPtAddFace(ccd_pt_t *pt, ccd_pt_edge_t *e1,
                                ccd_pt_edge_t *e2,
                                ccd_pt_edge_t *e3) {
        const ccd_vec3_t *a, *b, *c;
        ccd_pt_face_t *face;
        ccd_pt_edge_t *e;
        size_t i;

        if (e1 == NULL || e2 == NULL || e3 == NULL)
            return NULL;

        face = new(ccd_pt_face_t);
        if (face == NULL)
            return NULL;

        face->type = CCD_PT_FACE;
        face->edge[0] = e1;
        face->edge[1] = e2;
        face->edge[2] = e3;

        // obtain triplet of vertices
        a = &face->edge[0]->vertex[0]->v.v;
        b = &face->edge[0]->vertex[1]->v.v;
        e = face->edge[1];
        if (e->vertex[0] != face->edge[0]->vertex[0]
            && e->vertex[0] != face->edge[0]->vertex[1]) {
            c = &e->vertex[0]->v.v;
        } else {
            c = &e->vertex[1]->v.v;
        }
        face->dist = ccdVec3PointTriDist2(ccd_vec3_origin, a, b, c, &face->witness);


        for (i = 0; i < 3; i++) {
            if (face->edge[i]->faces[0] == NULL) {
                face->edge[i]->faces[0] = face;
            } else {
                face->edge[i]->faces[1] = face;
            }
        }

        ccdListAppend(&pt->faces, &face->list);

        // update position in .nearest array
        _ccdPtNearestUpdate(pt, (ccd_pt_el_t *) face);

        return face;
    }


    void ccdPtRecomputeDistances(ccd_pt_t *pt) {
        ccd_pt_vertex_t *v;
        ccd_pt_edge_t *e;
        ccd_pt_face_t *f;
        const ccd_vec3_t *a, *b, *c;
        ccd_real_t dist;

        ccdListForEachEntry(&pt->vertices, v, ccd_pt_vertex_t, list) {
            dist = ccdVec3Len2(&v->v.v);
            v->dist = dist;
            ccdVec3Copy(&v->witness, &v->v.v);
        }

        ccdListForEachEntry(&pt->edges, e, ccd_pt_edge_t, list) {
            a = &e->vertex[0]->v.v;
            b = &e->vertex[1]->v.v;
            dist = ccdVec3PointSegmentDist2(ccd_vec3_origin, a, b, &e->witness);
            e->dist = dist;
        }

        ccdListForEachEntry(&pt->faces, f, ccd_pt_face_t, list) {
            // obtain triplet of vertices
            a = &f->edge[0]->vertex[0]->v.v;
            b = &f->edge[0]->vertex[1]->v.v;
            e = f->edge[1];
            if (e->vertex[0] != f->edge[0]->vertex[0]
                && e->vertex[0] != f->edge[0]->vertex[1]) {
                c = &e->vertex[0]->v.v;
            } else {
                c = &e->vertex[1]->v.v;
            }

            dist = ccdVec3PointTriDist2(ccd_vec3_origin, a, b, c, &f->witness);
            f->dist = dist;
        }
    }

    ccd_pt_el_t *ccdPtNearest(ccd_pt_t *pt) {
        if (!pt->nearest) {
            _ccdPtNearestRenew(pt);
        }
        return pt->nearest;
    }


//    void ccdPtDumpSVT(ccd_pt_t *pt, const char *fn) {
//        FILE *fout;
//
//        fout = fopen(fn, "a");
//        if (fout == NULL)
//            return;
//
//        ccdPtDumpSVT2(pt, fout);
//
//        fclose(fout);
//    }
//
//    void ccdPtDumpSVT2(ccd_pt_t *pt, FILE *fout) {
//        ccd_pt_vertex_t *v, *a, *b, *c;
//        ccd_pt_edge_t *e;
//        ccd_pt_face_t *f;
//        size_t i;
//
//        fprintf(fout, "-----\n");
//
//        fprintf(fout, "Points:\n");
//        i = 0;
//        ccdListForEachEntry(&pt->vertices, v, ccd_pt_vertex_t, list) {
//            v->id = i++;
//            fprintf(fout, "%lf %lf %lf\n",
//                    ccdVec3X(&v->v.v), ccdVec3Y(&v->v.v), ccdVec3Z(&v->v.v));
//        }
//
//        fprintf(fout, "Edges:\n");
//        ccdListForEachEntry(&pt->edges, e, ccd_pt_edge_t, list) {
//            fprintf(fout, "%d %d\n", e->vertex[0]->id, e->vertex[1]->id);
//        }
//
//        fprintf(fout, "Faces:\n");
//        ccdListForEachEntry(&pt->faces, f, ccd_pt_face_t, list) {
//            a = f->edge[0]->vertex[0];
//            b = f->edge[0]->vertex[1];
//            c = f->edge[1]->vertex[0];
//            if (c == a || c == b) {
//                c = f->edge[1]->vertex[1];
//            }
//            fprintf(fout, "%d %d %d\n", a->id, b->id, c->id);
//        }
//    }


}


#endif //TCCPPLIB_POLYTOPE_HPP
