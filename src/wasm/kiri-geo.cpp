//#define use_int32

#include <emscripten.h>
#include "clipper.hpp"
#include <math.h>

typedef unsigned char Uint8;
typedef unsigned short Uint16;
typedef unsigned int Uint32;
typedef int int32;

using namespace ClipperLib;

Uint8 *mem = 0;

extern "C" {
    extern void debug_string(Uint32 len, char *str);
}

struct length16 {
    Uint16 length;
};

struct point32 {
    int32 x;
    int32 y;
};

__attribute__ ((export_name("mem_get")))
Uint32 mem_get(Uint32 size) {
    return (Uint32)malloc(size);
}

__attribute__ ((export_name("mem_clr")))
void mem_clr(Uint32 loc) {
    free((void *)loc);
}

void send_string(const char *format, ...) {
    char buffer[100];
    va_list args;
    va_start(args, format);
    Uint32 len =  vsprintf(buffer, format, args);
    va_end(args);
    debug_string(len, buffer);
}

Uint32 readPoly(Path &path, Uint32 pos) {
    struct length16 *ls = (struct length16 *)(mem + pos);
    Uint16 points = ls->length;
    pos += 2;
    while (points > 0) {
        struct point32 *ip = (struct point32 *)(mem + pos);
        pos += 8;
        path << IntPoint(ip->x, ip->y);
        points--;
    }
    return pos;
}

Uint32 readPolys(Paths &paths, Uint32 pos, Uint32 count) {
    Uint32 poly = 0;
    while (count > 0) {
        pos = readPoly(paths[poly++], pos);
        count--;
    }
    return pos;
}

Uint32 writePolys(Paths &outs, Uint32 pos) {
    for (Path po : outs) {
        struct length16 *ls = (struct length16 *)(mem + pos);
        ls->length = po.size();
        pos += 2;
        for (IntPoint pt : po) {
            struct point32 *ip = (struct point32 *)(mem + pos);
            ip->x = (int)pt.X;
            ip->y = (int)pt.Y;
            pos += 8;
        }
    }
    // null terminate
    struct length16 *ls = (struct length16 *)(mem + pos);
    ls->length = 0;
    return pos + 2;
}

__attribute__ ((export_name("poly_offset")))
Uint32 poly_offset(Uint32 memat, Uint32 polys, float offset, float clean, Uint8 simple) {
    Paths ins(polys);
    Paths outs;
    Uint32 pos = memat;
    Uint16 poly = 0;

    pos = readPolys(ins, pos, polys);

    if (clean > 0) {
        Paths cleans;
        CleanPolygons(ins, cleans, clean);
        ins = cleans;
    }

    if (simple > 0) {
        Paths simples;
        SimplifyPolygons(ins, simples);
        ins = simples;
    }

    ClipperOffset co;
    co.AddPaths(ins, jtMiter, etClosedPolygon);
    co.Execute(outs, offset);

    Uint32 resat = pos;

    pos = writePolys(outs, pos);

    co.Clear();

    return resat;
}

__attribute__ ((export_name("poly_union")))
Uint32 poly_union(Uint32 memat, Uint32 polys, float offset) {

    Paths ins(polys);
    Paths outs;
    Uint32 pos = memat;
    Uint16 poly = 0;

    pos = readPolys(ins, pos, polys);

    Clipper clip;
    clip.AddPaths(ins, ptSubject, true);
    clip.Execute(ctUnion, outs);

    Uint32 resat = pos;

    pos = writePolys(outs, pos);

    clip.Clear();

    return resat;
}

__attribute__ ((export_name("poly_diff")))
Uint32 poly_diff(Uint32 memat, Uint32 polysA, Uint32 polysB, Uint8 AB, Uint8 BA, float clean) {

    Paths inA(polysA);
    Paths inB(polysB);
    Uint32 pos = memat;

    pos = readPolys(inA, pos, polysA);
    pos = readPolys(inB, pos, polysB);

    Uint32 resat = pos;

    if (AB > 0) {
        Paths outs;
        Clipper clip;
        clip.AddPaths(inA, ptSubject, true);
        clip.AddPaths(inB, ptClip, true);
        clip.Execute(ctDifference, outs, pftEvenOdd, pftEvenOdd);
        if (clean > 0) {
            // CleanPolygons(outs, clean);
            for (Path po : outs) {
                CleanPolygon(po, clean);
            }
        }
        pos = writePolys(outs, pos);
        clip.Clear();
    }

    if (BA > 0) {
        Paths outs;
        Clipper clip;
        clip.AddPaths(inB, ptSubject, true);
        clip.AddPaths(inA, ptClip, true);
        clip.Execute(ctDifference, outs, pftEvenOdd, pftEvenOdd);
        if (clean > 0) {
            // CleanPolygons(outs, clean);
            for (Path po : outs) {
                CleanPolygon(po, clean);
            }
        }
        pos = writePolys(outs, pos);
        clip.Clear();
    }

    return resat;
}

// --- grid raycast ---

__attribute__ ((export_name("grid_raycast")))
bool grid_raycast(
    Uint8* grid_buffer,
    float ray_ox, float ray_oy,
    float ray_dx, float ray_dy,
    float max_dist
) {
    float* header_f = (float*)grid_buffer;
    Uint32* header_u = (Uint32*)(grid_buffer + 3 * sizeof(float));

    float offsetX = header_f[0];
    float offsetY = header_f[1];
    float cellSize = header_f[2];
    Uint32 cols = header_u[0];
    Uint32 rows = header_u[1];
    Uint32 segments_count = header_u[2];
    Uint32 cell_segment_map_count = header_u[3];

    Uint32 segments_offset = (3 * sizeof(float)) + (4 * sizeof(Uint32));
    Uint32 cell_index_offset = segments_offset + segments_count * 4 * sizeof(float);
    Uint32 cell_segment_map_offset = cell_index_offset + rows * cols * 2 * sizeof(Uint32);

    float* segments = (float*)(grid_buffer + segments_offset);
    Uint32* cell_indices = (Uint32*)(grid_buffer + cell_index_offset);
    Uint32* cell_segment_map = (Uint32*)(grid_buffer + cell_segment_map_offset);

    const float gridOriginX = ray_ox - offsetX;
    const float gridOriginY = ray_oy - offsetY;

    int x = floor(gridOriginX / cellSize);
    int y = floor(gridOriginY / cellSize);

    const int stepX = ray_dx > 0 ? 1 : -1;
    const int stepY = ray_dy > 0 ? 1 : -1;

    const float tDeltaX = fabs(cellSize / ray_dx);
    const float tDeltaY = fabs(cellSize / ray_dy);

    float tMaxX = ray_dx > 0
        ? ((x + 1) * cellSize - gridOriginX) / ray_dx
        : (gridOriginX - x * cellSize) / fabs(ray_dx);

    float tMaxY = ray_dy > 0
        ? ((y + 1) * cellSize - gridOriginY) / ray_dy
        : (gridOriginY - y * cellSize) / fabs(ray_dy);

    float closest_dist = max_dist;
    bool hit = false;

    while (true) {
        if (x < 0 || x >= cols || y < 0 || y >= rows) break;

        Uint32 cell_idx_addr = (y * cols + x) * 2;
        Uint32 cell_offset = cell_indices[cell_idx_addr];
        Uint32 cell_count = cell_indices[cell_idx_addr + 1];

        if (cell_count > 0) {
            for (Uint32 i = 0; i < cell_count; i++) {
                Uint32 seg_idx = cell_segment_map[cell_offset + i] * 4;
                float p1x = segments[seg_idx];
                float p1y = segments[seg_idx + 1];
                float p2x = segments[seg_idx + 2];
                float p2y = segments[seg_idx + 3];

                const float v1x = ray_ox - p1x;
                const float v1y = ray_oy - p1y;
                const float v2x = p2x - p1x;
                const float v2y = p2y - p1y;
                const float v3x = -ray_dx;
                const float v3y = -ray_dy;

                const float dot = v2x * v3y - v2y * v3x;

                if (fabs(dot) < 0.000001) continue;

                const float t1 = (v2x * v1y - v2y * v1x) / dot;
                const float t2 = (v1x * v3y - v1y * v3x) / dot;

                if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f) {
                    if (t1 < closest_dist) {
                        closest_dist = t1;
                        hit = true;
                    }
                }
            }
        }

        if (hit && closest_dist < fmin(tMaxX, tMaxY)) {
            return true;
        }

        if (tMaxX < tMaxY) {
            tMaxX += tDeltaX;
            x += stepX;
        } else {
            tMaxY += tDeltaY;
            y += stepY;
        }

        if (tMaxX > closest_dist && tMaxY > closest_dist) break;
    }

    return hit;
}
