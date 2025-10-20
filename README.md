# 三维网格几何处理算法的实现与分析报告


## 一、摘要  
本文针对三维网格的四大核心几何处理算法——网格细分、网格参数化、网格简化与网格平滑进行了详细阐述。




## 二、网格细分算法（SubdivisionMesh）  
网格细分的核心目标是通过迭代细化网格，增加三角形数量，使模型表面更平滑。本文实现的是基于三角形网格的Loop细分思想。  


### 2.1 算法原理  
Loop细分的核心思想是：每次迭代中，对原始网格的每个顶点进行位置更新，并在每条边上生成新顶点，最终将每个原始三角形分割为4个新三角形。具体规则如下：  
1. **顶点更新**：根据顶点是否为边界顶点，采用不同的加权平均策略。  
   - 边界顶点：由自身位置（75%权重）和两个相邻边界顶点（各12.5%权重）加权平均得到。  
   - 内部顶点：由自身位置和所有相邻顶点加权平均得到，权重与相邻顶点数量相关（β值）。  
2. **新顶点生成**：每条边会生成一个新顶点，边界边与内部边的生成规则不同。  
   - 边界边：新顶点为边的两个端点的平均值。  
   - 内部边：新顶点由边的两个端点（各37.5%权重）和边所在两个面的另外两个顶点（各12.5%权重）加权平均得到。  
3. **面重构**：每个原始三角形被分割为4个新三角形，由原始顶点和新生成的边顶点组合而成。  


### 2.2 代码实现解析  
代码中`SubdivisionMesh`函数通过多轮迭代实现细分，核心步骤如下：  

#### 2.2.1 初始化与拓扑结构构建  
每次迭代前，将当前网格复制到`prev_mesh`，并基于`prev_mesh`构建DCEL结构`G`，用于快速访问网格的拓扑信息（如顶点的相邻顶点、边是否为边界边等）。  

```cpp
Engine::SurfaceMesh curr_mesh = input;
for (std::uint32_t it = 0; it < numIterations; ++it) {
    Engine::SurfaceMesh prev_mesh;
    prev_mesh.Swap(curr_mesh);
    DCEL G(prev_mesh); // 构建DCEL以管理拓扑关系
    // ... 检查网格是否为流形
}
```


#### 2.2.2 原始顶点位置更新  
遍历`prev_mesh`的所有顶点，根据顶点类型（边界/内部）计算新位置，并存储到`curr_mesh`的顶点列表中。  

- **边界顶点处理**：通过`v->OnBoundary()`判断顶点是否为边界顶点，使用75%-12.5%-12.5%的权重计算新位置：  
  ```cpp
  if (v->OnBoundary()) {
      curr_mesh.Positions.push_back(0.75f * prev_mesh.Positions[i] + 0.125f * (prev_mesh.Positions[neighbors[0]] + prev_mesh.Positions[neighbors[1]]));
  }
  ```  

- **内部顶点处理**：根据相邻顶点数量`length`计算β值（当`length>3`时，β=3/(8*length)；否则β=3/16），再通过加权平均更新位置：  
  ```cpp
  float beta = (length > 3) ? 3.0f / (8.0f * length) : 3.0f / 16.0f;
  glm::vec3 n(0.0f);
  for (auto idx : neighbors) n += prev_mesh.Positions[idx];
  curr_mesh.Positions.push_back((1 - length * beta) * prev_mesh.Positions[i] + beta * n);
  ```  


#### 2.2.3 新顶点生成  
遍历所有边，根据边是否为边界边（通过`e->TwinEdgeOr(nullptr)`判断，无对偶边则为边界边）生成新顶点，并记录新顶点在`newIndices`中的位置（用于后续面重构）。  

- **边界边新顶点**：直接取两端点的平均值：  
  ```cpp
  if (!eTwin) { // 边界边
      glm::vec3 newVertex = 0.5f * (prev_mesh.Positions[e->From()] + prev_mesh.Positions[e->To()]);
      curr_mesh.Positions.push_back(newVertex);
  }
  ```  

- **内部边新顶点**：结合两端点及相邻面的另外两个顶点计算：  
  ```cpp
  else { // 内部边
      glm::vec3 newVertex = 0.375f * (prev_mesh.Positions[e->From()] + prev_mesh.Positions[e->To()]) + 0.125f * (prev_mesh.Positions[e->NextEdge()->To()] + prev_mesh.Positions[eTwin->NextEdge()->To()]);
      curr_mesh.Positions.push_back(newVertex);
  }
  ```  


#### 2.2.4 面重构  
每个原始三角形（由顶点`v0, v1, v2`组成）被分割为4个新三角形，顶点分别为：  
- 原始顶点`v0`与边`v0-v1`、`v0-v2`上的新顶点`m2, m1`；  
- 原始顶点`v1`与边`v1-v2`、`v1-v0`上的新顶点`m0, m2`；  
- 原始顶点`v2`与边`v2-v0`、`v2-v1`上的新顶点`m1, m0`；  
- 三条边上的新顶点`m0, m1, m2`。  

代码中通过`toInsert`数组定义新三角形的顶点索引，并插入到`curr_mesh`的索引列表中：  
```cpp
std::uint32_t toInsert[4][3] = {
    {v0, m2, m1},
    {v1, m0, m2},
    {v2, m1, m0},
    {m0, m1, m2}
};
curr_mesh.Indices.insert(..., toInsert, toInsert + 12U);
```  




## 三、网格参数化算法（Parameterization）  
网格参数化的目标是将三维网格拓扑等价地映射到二维平面（如单位圆或矩形），保留网格的邻接关系，为纹理映射、网格编辑等提供基础。本文实现的是基于边界约束的参数化方法，采用高斯-赛德尔迭代求解内部顶点坐标。  


### 3.1 算法原理  
参数化的核心是在保持网格拓扑结构的前提下，求解二维纹理坐标（UV坐标）。步骤如下：  
1. **边界处理**：提取网格的边界顶点，按顺序排列后映射到单位圆上（极坐标分布），确保边界在二维平面上的连续性。  
2. **内部顶点求解**：内部顶点的UV坐标由其所有相邻顶点的UV坐标平均值确定，通过高斯-赛德尔迭代优化，直至收敛。  


### 3.2 代码实现解析  
`Parameterization`函数的核心步骤包括边界顶点排序与映射、内部顶点迭代更新。  


#### 3.2.1 边界顶点提取与排序  
边界顶点是指仅属于一个面的顶点（通过`v->OnBoundary()`判断）。为确保边界在二维平面上的连续性，需要按顺时针或逆时针顺序排列边界顶点：  

```cpp
std::vector<DCEL::VertexIdx> boundary_vertices;
for (int i = 0; i < G.NumOfVertices(); ++i) {
    if (G.Vertex(i)->OnBoundary()) {
        boundary_vertices.push_back(i);
    }
}
// 按顺序排列边界顶点（通过边界邻接关系遍历）
std::vector<DCEL::VertexIdx> ordered_boundary;
if (!boundary_vertices.empty()) {
    auto cur = boundary_vertices[0];
    auto prev = -1;
    ordered_boundary.push_back(cur);
    while (ordered_boundary.size() < boundary_vertices.size()) {
        auto v = G.Vertex(cur);
        auto neighbors = v->BoundaryNeighbors(); // 获取边界邻接顶点
        auto next = (neighbors.first == prev) ? neighbors.second : neighbors.first;
        ordered_boundary.push_back(next);
        prev = cur;
        cur = next;
    }
}
```  


#### 3.2.2 边界顶点映射到单位圆  
将排序后的边界顶点均匀映射到单位圆上，第`i`个顶点的UV坐标为`(cosθ, sinθ)`，其中`θ=2πi/L`（`L`为边界顶点数量）：  

```cpp
int L = ordered_boundary.size();
for (int i = 0; i < L; ++i) {
    float angle = 2.0f * glm::pi<float>() * (float)i / (float)L;
    output.TexCoords[ordered_boundary[i]] = {cos(angle), sin(angle)};
}
```  


#### 3.2.3 内部顶点高斯-赛德尔迭代  
内部顶点（非边界顶点）的UV坐标设为其所有相邻顶点UV坐标的平均值，通过多轮迭代优化：  

```cpp
for (int k = 0; k < numIterations; ++k) { // 迭代numIterations次
    for (int i = 0; i < G.NumOfVertices(); i++) {
        auto v = G.Vertex(i);
        if (v->OnBoundary()) continue; // 跳过边界顶点
        auto neighbors = v->Neighbors(); // 获取所有相邻顶点
        glm::vec2 uv(0.0f);
        for (auto idx : neighbors) uv += output.TexCoords[idx];
        output.TexCoords[i] = uv / (float)neighbors.size(); // 平均值更新
    }
}
```  
 


## 四、网格简化算法（SimplifyMesh）  
网格简化的目标是在保留模型主要形状特征的前提下，减少顶点和面的数量，降低存储与渲染成本。本文实现的是基于边收缩的简化算法，通过Q矩阵计算收缩误差，迭代收缩成本最小的边。  


### 4.1 算法原理  
边收缩简化的核心是：将一条边的两个端点合并为一个新顶点，删除被收缩的边及相关面，并更新周围的拓扑关系。为确保简化后模型的形状保真，需通过Q矩阵计算收缩误差（成本），优先收缩成本最小的边。  

1. **Q矩阵计算**：每个三角形面的Q矩阵（`Kp`）由其平面方程系数确定，顶点的Q矩阵为其所属所有面的`Kp`之和。  
2. **边收缩成本**：对于边`(v1, v2)`，收缩后的目标顶点`v`的最优位置由`(Q1+Q2)v=0`求解（`Q1, Q2`为`v1, v2`的Q矩阵），成本为`v^T(Q1+Q2)v`。  
3. **迭代收缩**：每次选择成本最小的可收缩边进行收缩，更新相关顶点的Q矩阵及拓扑关系，直至达到目标简化比例。  


### 4.2 代码实现解析  
`SimplifyMesh`函数的核心步骤包括Q矩阵初始化、边收缩成本计算、迭代收缩与拓扑更新。  


#### 4.2.1 Q矩阵初始化  
- **面的Q矩阵（Kp）**：由三角形平面方程`ax + by + cz + d = 0`的系数`(a,b,c,d)`构建，`Kp = (a,b,c,d)^T (a,b,c,d)`（外积）：  

```cpp
auto UpdateQ = [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
    glm::vec3 v0 = output.Positions[f->VertexIndex(0)];
    glm::vec3 v1 = output.Positions[f->VertexIndex(1)];
    glm::vec3 v2 = output.Positions[f->VertexIndex(2)];
    auto normal = glm::normalize(glm::cross(v1 - v0, v2 - v0)); // 平面法向量(a,b,c)
    float d = -glm::dot(normal, v0); // 平面方程常数项d
    glm::vec4 p(normal, d);
    return glm::outerProduct(p, p); // Kp = p * p^T
};
```  

- **顶点的Q矩阵（Qv）**：顶点所属所有面的`Kp`之和：  

```cpp
std::vector<glm::mat4> Qv(G.NumOfVertices(), glm::mat4(0));
for (auto f : G.Faces()) {
    auto Q = UpdateQ(f);
    Qv[f->VertexIndex(0)] += Q;
    Qv[f->VertexIndex(1)] += Q;
    Qv[f->VertexIndex(2)] += Q;
}
```  


#### 4.2.2 边收缩成本与目标位置计算  
对于边`(v1, v2)`，收缩目标位置`v`通过求解`(Q1+Q2)v = 0`确定。若`Q1+Q2`可逆，则`v = (Q1+Q2)^{-1} * (0,0,0,1)^T`；否则取`v1, v2`的中点。成本为`v^T(Q1+Q2)v`：  

```cpp
static constexpr auto MakePair = [] (DCEL::HalfEdge const * edge,
    glm::vec3 const & p1, glm::vec3 const & p2, glm::mat4 const & Q) -> ContractionPair {
    glm::mat4 Qbar = Q;
    Qbar[3] = glm::vec4(0, 0, 0, 1); // 调整Q矩阵以求解v
    glm::vec4 targetPosition;
    if (fabs(glm::determinant(Qbar)) > 1e-3) { // 可逆
        targetPosition = glm::inverse(Qbar) * glm::vec4(0, 0, 0, 1);
    } else { // 不可逆，取中点
        targetPosition = 0.5f * glm::vec4(p1 + p2, 2.0f);
    }
    float cost = glm::dot(targetPosition, Q * targetPosition); // 计算成本
    return {edge, targetPosition, cost};
};
```  


#### 4.2.3 迭代收缩与拓扑更新  
- **选择成本最小的边**：遍历所有可收缩边，筛选出成本最小的边进行收缩。  
- **收缩后更新**：收缩边`(v1, v2)`后，`v2`被删除，`v1`移动到目标位置；更新相关面的`Kp`矩阵及顶点的`Qv`矩阵，并重新计算相邻边的收缩成本：  

```cpp
while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
    // 查找成本最小的可收缩边
    std::size_t min_idx = ~0;
    for (std::size_t i = 0; i < pairs.size(); ++i) {
        if (!pairs[i].edge || !G.IsContractable(pairs[i].edge)) {
            pairs[i].edge = nullptr;
            continue;
        }
        if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
            min_idx = i;
        }
    }
    if (!~min_idx) break;

    // 收缩边并更新拓扑与Q矩阵
    ContractionPair & top = pairs[min_idx];
    auto v1 = top.edge->From();
    auto v2 = top.edge->To();
    auto result = G.Contract(top.edge); // 执行收缩
    output.Positions[v1] = top.targetPosition; // 更新v1位置

    // 更新相关顶点的Q矩阵
    Qv[v1] = glm::mat4(0);
    for (auto e : G.Vertex(v1)->Ring()) { // 遍历v1的邻边
        auto face = e->Face();
        auto newKp = UpdateQ(face);
        auto oldKp = Kf[G.IndexOf(face)];
        Qv[e->To()] += newKp - oldKp; // 更新邻接顶点Q矩阵
        Qv[v1] += newKp; // 更新v1的Q矩阵
        Kf[G.IndexOf(face)] = newKp;
    }

    // 重新计算相邻边的收缩成本
    for (auto e : G.Vertex(v1)->Ring()) {
        // ... 调用MakePair更新边的成本与目标位置
    }
}
```  

  


## 五、网格平滑算法（SmoothMesh）  
网格平滑的目标是通过调整顶点位置减少噪声（如扫描数据中的毛刺），使模型表面更光滑，同时尽量保留原始形状特征。本文实现了两种平滑策略：均匀权重平滑和余切权重平滑。  


### 5.1 算法原理  
平滑算法通过迭代调整顶点位置实现，核心是定义顶点的更新规则：  
1. **均匀权重平滑**：顶点新位置为其所有相邻顶点的平均值，适用于快速去除噪声，但可能导致模型收缩。  
2. **余切权重平滑**：顶点新位置由相邻顶点的加权平均确定，权重为相邻边夹角的余切值，可减少收缩并保留特征。  


### 5.2 代码实现解析  
`SmoothMesh`函数的核心是余切计算与顶点位置迭代更新。  


#### 5.2.1 余切计算  
对于顶点`v`的相邻边`(v, v1)`和`(v, v2)`，夹角的余切值通过向量点积与叉积计算：  

```cpp
static constexpr auto GetCotangent = [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
    glm::vec3 h1 = v1 - vAngle; // 向量v->v1
    glm::vec3 h2 = v2 - vAngle; // 向量v->v2
    float dot = glm::dot(h1, h2); // 点积
    float cross_len = glm::length(glm::cross(h1, h2)); // 叉积长度
    return dot / (cross_len + 1e-6); // 余切 = 点积 / 叉积长度
};
```  


#### 5.2.2 顶点位置迭代更新  
根据权重类型（均匀/余切）计算新位置，每次迭代的更新幅度由`lambda`控制（`lambda`越小，平滑越平缓）：  

```cpp
Engine::SurfaceMesh prev_mesh;
prev_mesh.Positions = input.Positions;
for (std::uint32_t it = 0; it < numIterations; ++it) {
    Engine::SurfaceMesh curr_mesh = prev_mesh; // 复制当前位置
    for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
        auto v = G.Vertex(i);
        if (v->OnBoundary()) continue; // 边界顶点不更新（避免收缩）

        auto neighbors = v->Neighbors();
        glm::vec3 new_pos(0.0f);
        float weight_sum = 0.0f;

        if (useUniformWeight) { // 均匀权重
            for (auto idx : neighbors) {
                new_pos += prev_mesh.Positions[idx];
            }
            new_pos /= (float)neighbors.size();
        } else { // 余切权重
            for (auto idx : neighbors) {
                // 找到与idx相邻的两个顶点（形成夹角）
                auto edges = v->EdgesTo(idx);
                auto e1 = edges.first;
                auto e2 = edges.second;
                auto v1 = prev_mesh.Positions[e1->To()];
                auto v2 = prev_mesh.Positions[e2->To()];
                float cot = GetCotangent(prev_mesh.Positions[i], v1, v2);
                new_pos += cot * prev_mesh.Positions[idx];
                weight_sum += cot;
            }
            new_pos /= weight_sum;
        }

        // 按lambda更新位置（原始位置 + lambda*(新位置-原始位置)）
        curr_mesh.Positions[i] = prev_mesh.Positions[i] + lambda * (new_pos - prev_mesh.Positions[i]);
    }
    prev_mesh = curr_mesh;
}
output = prev_mesh;
```  
## 六、Marching Cubes算法（等值面生成）  
Marching Cubes算法从三维标量场（如符号距离函数SDF）中提取等值面，生成三维网格，广泛应用于医学影像（如CT重建）、流体模拟等领域。  


### 6.1 算法原理  
标量场中，等值面是所有满足`SDF(v)=0`的点`v`的集合（本文以0为阈值）。算法通过以下步骤生成等值面：  
1. **体素采样**：在三维网格中划分立方体体素，计算体素8个顶点的SDF值。  
2. **体素分类**：根据顶点SDF值的符号（正/负）确定体素与等值面的相交状态，通过预定义的查找表（`c_EdgeOrdsTable`）获取边交点组合。  
3. **交点计算**：对每个与等值面相交的边，通过线性插值计算交点坐标。  
4. **三角形生成**：按查找表连接交点，形成三角形面片，拼接为完整网格。  


### 6.2 代码实现解析  
`MarchingCubes`函数的核心步骤如下：  


#### 6.2.1 体素网格定义  
通过三重循环遍历三维网格中的每个体素，体素起点为`v0 = grid_min + (x*dx, y*dx, z*dx)`，其中`grid_min`为网格起点，`dx`为体素边长，`n`为每维采样数。  

```cpp
std::vector<glm::vec3> unit = {{1.0f,0,0}, {0,1.0f,0}, {0,0,1.0f}}; // 单位向量（x,y,z轴）
for(int x=0; x<n; x++){
    for(int y=0; y<n; y++){
        for(int z=0; z<n; z++){
            glm::vec3 v0 = grid_min + glm::vec3((float)x*dx, (float)y*dx, (float)z*dx); // 体素起点
            // ... 体素处理
        }
    }
}
```  


#### 6.2.2 体素顶点SDF计算与分类  
计算体素8个顶点的SDF值，根据符号（`sdf(v) > 0`）生成8位二进制编码（`ans`），用于查找表索引。  

```cpp
int ans = 0; // 8位编码，每一位表示一个顶点是否在等值面外侧（SDF>0）
for (int i=0; i<8; i++){
    // 计算第i个顶点坐标（体素8个顶点的索引规则：i的二进制位对应x,y,z方向偏移）
    glm::vec3 v = glm::vec3(
        v0.x + (float)(i & 1) * dx,       // x偏移：i的第0位
        v0.y + (float)((i >> 1) & 1) * dx, // y偏移：i的第1位
        v0.z + (float)(i >> 2) * dx        // z偏移：i的第2位
    );
    if(sdf(v) > 0) ans += (1 << i); // 外侧顶点对应位设为1
}
```  


#### 6.2.3 边交点计算与三角形生成  
通过`c_EdgeOrdsTable[ans]`获取体素内需要生成的三角形边索引，对每条边计算交点，并连接为三角形。  

- **边索引解析**：`e_id`为边的编码，包含边的起点、方向信息。例如，`e_id >> 2`表示边的方向（0=x轴，1=y轴，2=z轴），`(e_id & 1)`和`((e_id >> 1) & 1)`表示起点在体素中的偏移。  
- **交点计算**：对边的两个端点`p1`（内侧，SDF≤0）和`p2`（外侧，SDF>0），通过线性插值`p = p1 + t*(p2-p1)`计算交点，其中`t = s1/(s1-s2)`（`s1, s2`为两端点SDF值）。  

```cpp
for(int i=0; i<16; i+=3){ // 查找表每行最多3个三角形（9个顶点索引，每3个一组）
    if(c_EdgeOrdsTable[ans][i] == -1) break; // 无更多三角形

    for(int j=0; j<3; j++){ // 每个三角形的3条边
        int e_id = c_EdgeOrdsTable[ans][i+j]; // 边索引
        // 计算边的起点p1和终点p2
        glm::vec3 p1 = v0 + 
            dx * (float)(e_id & 1) * unit[((e_id >> 2) + 1) % 3] + 
            dx * (float)((e_id >> 1) & 1) * unit[((e_id >> 2) + 2) % 3];
        glm::vec3 p2 = p1 + dx * unit[e_id >> 2]; // 沿方向轴偏移dx

        // 线性插值计算交点
        float s1 = sdf(p1);
        float s2 = sdf(p2);
        float t = s1 / (s1 - s2); // 确保t在[0,1]内
        glm::vec3 p = p1 + t * (p2 - p1);
        output.Positions.push_back(p); // 添加交点到顶点列表
    }

    // 添加三角形索引（最后3个顶点）
    output.Indices.push_back(output.Positions.size() - 1);
    output.Indices.push_back(output.Positions.size() - 2);
    output.Indices.push_back(output.Positions.size() - 3);
}
```  
