#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                auto length = neighbors.size();
                if (length==0) {
                    curr_mesh.Positions.push_back(prev_mesh.Positions[i]);
                    continue;
                }
                else if (v->OnBoundary()) {
                    curr_mesh.Positions.push_back(0.75f * prev_mesh.Positions[i] + 0.125f * (prev_mesh.Positions[neighbors[0]] + prev_mesh.Positions[neighbors[1]]));
                    continue;
                }
                else{
                    float beta;
                    if (length > 3) beta = 3.0f / (8.0f * length);
                    else beta = 3.0f / 16.0f;
                    glm::vec3 n(0.0f);
                    for (auto idx : neighbors) n += prev_mesh.Positions[idx];
                    curr_mesh.Positions.push_back((1 - length * beta) * prev_mesh.Positions[i] + beta * n);
                }
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 newVertex = 0.5f * (prev_mesh.Positions[e->From()] + prev_mesh.Positions[e->To()]);
                    curr_mesh.Positions.push_back(newVertex);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 newVertex = 0.375f * (prev_mesh.Positions[e->From()] + prev_mesh.Positions[e->To()]) + 0.125f * (prev_mesh.Positions[e->NextEdge()->To()] + prev_mesh.Positions[eTwin->NextEdge()->To()]);
                    curr_mesh.Positions.push_back(newVertex);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0, m2, m1},
                    {v1, m0, m2},
                    {v2, m1, m0},
                    {m0, m1, m2}
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::vector<DCEL::VertexIdx> boundary_vertices;
        for (int i = 0; i < G.NumOfVertices(); ++i) {
            if (G.Vertex(i)->OnBoundary()) {boundary_vertices.push_back(i);}
        }
        std::vector<DCEL::VertexIdx> ordered_boundary;
        if (boundary_vertices.size()>0) {
            auto cur = boundary_vertices[0];
            auto prev=-1;
            ordered_boundary.push_back(cur);
            while(ordered_boundary.size()<boundary_vertices.size()) {
                auto v = G.Vertex(cur);
                if(!v)break;
                auto neighbors = v->BoundaryNeighbors();
                auto next= neighbors.first==prev?neighbors.second:neighbors.first;
                ordered_boundary.push_back(next);
                prev=cur;
                cur=next;
            }
        }
        int L=ordered_boundary.size();
        for(int i=0;i<L;i++){
            float angle = 2.0f * glm::pi<float>() * (float)i / (float)L;
            output.TexCoords[ordered_boundary[i]] = {cos(angle),sin(angle)};
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            for(int i=0;i<G.NumOfVertices();i++){
                auto v = G.Vertex(i);
                if(!v)continue;
                if(v->OnBoundary())continue;
                auto neighbors = v->Neighbors();
                glm::vec2 uv(0.0f);
                for (auto idx : neighbors) uv += output.TexCoords[idx];
                uv /= (float)neighbors.size();
                output.TexCoords[i] = uv;
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                // your code here:
                auto v0 = output.Positions[f->VertexIndex(0)];
                auto v1 = output.Positions[f->VertexIndex(1)];
                auto v2 = output.Positions[f->VertexIndex(2)];
                auto normal = (glm::cross(v1 - v0, v2 - v0));
                normal = glm::normalize(normal);
                float d = -glm::dot(normal, v0);
                glm::vec4 p(normal, d);
                Kp = glm::outerProduct(p, p);
                return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                ContractionPair pair;
                pair.edge = edge;
                if(!edge) return {};
                glm::mat4 Qbar = Q;
                Qbar[0][3] = 0.0f;
                Qbar[1][3] = 0.0f;
                Qbar[2][3] = 0.0f;
                Qbar[3][3] = 1.0f;
                auto targetPosition = glm::vec4(0.0f);
                if (fabs(glm::determinant(Qbar)) > 1e-3) {
                    glm::vec4 v = glm::inverse(Qbar) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                    targetPosition = v;
                } else {
                    glm::vec4 v1(p1, 1.0f);
                    glm::vec4 v2(p2, 1.0f);
                    glm::vec4 mid = 0.5f * (v1 + v2);
                    targetPosition = mid;
                }
                auto cost = glm::dot(targetPosition, Q * targetPosition);
                if(cost<0) cost=0;
                //spdlog::info("Edge: v{}->v{}, cost={}", edge->From(), edge->To(), cost);
                return {edge, targetPosition, cost};
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();
            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Kf$.
                DCEL::Triangle const * face = e->Face();
                if(!face||G.IsFaceRemoved(face)) continue;
                auto newKp = UpdateQ(face);
                auto oldKp = Kf[G.IndexOf(face)];
                auto diff = newKp - oldKp;
                auto v_idx1=e->To();
                auto v_idx2=e->From();
                Qv[v_idx1] += diff;
                Qv[v_idx2] += diff;
                Qv[v1]     += newKp;
                Kf[G.IndexOf(face)] = newKp;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
        for (auto e : G.Vertex(v1)->Ring()) {
                auto v1   = e->From();
                auto ring = G.Vertex(v1)->Ring();
                for (auto e1 : ring) {
                    auto e2 = e1->NextEdge();
                    if (! G.IsContractable(e2)) {
                        pairs[pair_map[G.IndexOf(e2)]].edge = nullptr;
                    } else {
                        auto v2                                                  = e1->To();
                        auto pair                                                = MakePair(e2, output.Positions[v1], output.Positions[v2], Qv[v1] + Qv[v2]);
                        pairs[pair_map[G.IndexOf(e2)]].targetPosition = pair.targetPosition;
                        pairs[pair_map[G.IndexOf(e2)]].cost           = pair.cost;
                    }
                }
            }
         
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                glm::vec3 h1=(v1-vAngle);
                glm::vec3 h2=(v2-vAngle);
                float d=glm::dot(h1,h2);
                float c=glm::length(glm::cross(h1,h2));
                float cot=d/(c+1e-6);
                cot=glm::clamp(cot,0.1f,100.0f);
                cot=fabs(cot);
                return cot;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                glm::vec3 la=glm::vec3(0.0f);
                if(useUniformWeight){
                    int q=0;
                    for(auto f: G.Vertex(i)->Neighbors()){
                        q++;
                        la+=curr_mesh.Positions[f];
                    }
                    la/=q;
                }
                else{
                    float w=0.0f;
                    for(auto e: G.Vertex(i)->Ring()){
                        auto f=e->From();
                        auto pe=e->PrevEdge();
                        auto fp=pe->From();
                        auto alpha=GetCotangent(curr_mesh.Positions[fp],curr_mesh.Positions[i],curr_mesh.Positions[f]);
                        auto twin_e = e->TwinEdge();
                        float beta;
                        if (twin_e) {  // 若不是边界边，计算另一侧余切
                            auto tpe = twin_e->PrevEdge();  // 孪生边的前序半边
                            auto l = tpe->From();           // 另一侧三角形的第三顶点l
                            glm::vec3 pos_l = curr_mesh.Positions[l];
            // 计算另一侧三角形中角i的余切（角i由j-i-l形成）
                            beta = GetCotangent(curr_mesh.Positions[l],curr_mesh.Positions[i],curr_mesh.Positions[f]);
                            }
                        w+=(alpha+beta);
                        la+=(alpha+beta)*curr_mesh.Positions[f];
                    }
                    la/=w;
                }

                curr_mesh.Positions[i] = (1-lambda)*curr_mesh.Positions[i]+lambda*la;
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
         std::vector<glm::vec3> unit={{1.0f,0,0},{0,1.0f,0},{0,0,1.0f}};
        for(int x=0;x<n;x++){
            for(int y=0;y<n;y++){
                for(int z=0;z<n;z++){
                    glm::vec3 v0=grid_min+glm::vec3((float)x*dx,(float)y*dx,(float)z*dx);
                    int ans=0;
                    for (int i=0;i<8;i++){
                        float x0=v0.x;
                        float y0=v0.y;
                        float z0=v0.z;
                        glm::vec3 v=glm::vec3(x0 + (float)(i & 1) * dx, y0 + (float)((i >> 1) & 1) * dx, z0 + (float)(i >> 2) * dx);
                        if(sdf(v)>0){
                            ans+=(1<<i);
                        }
                    }
                    for(int i=0;i<16;i+=3){
                        if(c_EdgeOrdsTable[ans][i]==-1){
                            break;
                        }
                        glm::vec3 p;
                        for(int j=0;j<3;j++){
                            int e_id=c_EdgeOrdsTable[ans][i+j];
                            glm::vec3 p1;
                            p1=v0+dx * (float)(e_id & 1) * unit[((e_id >> 2) + 1) % 3] + dx * (float)((e_id >> 1) & 1) * unit[((e_id >> 2) + 2) % 3];
                            glm::vec3 p2;
                            p2=p1+dx*unit[e_id>>2];
                            float s1 = sdf(p1);
                            float s2 = sdf(p2);
                            float t = s1 / (s1 - s2);  
                            glm::vec3 p = p1 + t * (p2 - p1);  
                            output.Positions.push_back(p);
                        }
                        output.Indices.push_back(output.Positions.size() - 1);
                        output.Indices.push_back(output.Positions.size() - 2);
                        output.Indices.push_back(output.Positions.size() - 3);
                    }
                }
            }
        }
    }
} // namespace VCX::Labs::GeometryProcessing
