#include "QEM.h"

int* Previous; // Previous index
int* New; // New index
int* Order; //Sort order index
bool* Boundary; // Boundary edge

std::vector<Vertex> vertices;
std::vector<Face> faces;
std::vector<std::set<int> > edges, orig_edges;

std::priority_queue<Edge> Edge_error; // Edges for simplification in a priority order based on their error values.
std::map<std::pair<int, int>, int> timer; // Time count

int face_num, vertex_num; // Original size
double error_limit = 1e5;
double angle_limit = 0.25;
double out_cost = 0;
double last_cost = 0;

void clearData() { // Clear data for next simplification

	vertices.clear();

	faces.clear();

	edges.clear();
	orig_edges.clear();

	while (!Edge_error.empty()) {
		Edge_error.pop();
	}

	timer.clear();

	face_num = 0;
	vertex_num = 0;
}

inline bool operator <(const Edge& a, const Edge& b) { // Redefine < operator
	return a.error > b.error;
}

inline double distance(int u, int v) { // Calculate distance
	glm::dvec3 positionU = vertices[u].position;
	glm::dvec3 positionV = vertices[v].position;

	return glm::distance(positionU, positionV);
}

inline bool compare(const int& u, const int& v) { // Compare function for sorting
	for (int i = 0; i < 3; ++i) {
		if (vertices[u].position[i] != vertices[v].position[i]) {
			return vertices[u].position[i] < vertices[v].position[i];
		}
	}
	return false;  // If u(x,y,z) = v(x,y,z), return false
}

void computeNormal(Face& face) {

	glm::dvec3 p1 = vertices[face.v1].position;
	glm::dvec3 p2 = vertices[face.v2].position;
	glm::dvec3 p3 = vertices[face.v3].position;

	// Compute edge vectors for the face
	glm::dvec3 edge1 = p2 - p1;
	glm::dvec3 edge2 = p3 - p1;

	// Compute the cross product of the two edge vectors to obtain the face normal
	glm::dvec3 n = glm::cross(edge1, edge2);
	face.normal = glm::normalize(n);

	// Assign computed normal to the normal of the face
	face.d = -glm::dot(face.normal, p1);

}

void computeQuadric(Vertex& vertex) {

	// Initialize Quadric 
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			vertex.Quadric[i][j] = 0.0;
		}
	}

	// Accumulate to the four dimensional matrix
	for (int i = 0; i < vertex.faceList.size(); i++) {
		int faceIndex = *std::next(vertex.faceList.begin(), i);
		const Face& f = faces[faceIndex];
		double p[4] = { f.normal.x, f.normal.y, f.normal.z, f.d }; // Normal and distance
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				vertex.Quadric[i][j] += p[i] * p[j];
			}
		}
	}
}

void computeMatrixCost(int u, int v, glm::dvec4& x, double& cost) {
	for (auto it = vertices[v].faceList.begin(); it != vertices[v].faceList.end(); it++) {
		if (faces[*it].v1 == u || faces[*it].v2 == u || faces[*it].v3 == u) continue;
		int v1 = faces[*it].v1, v2 = faces[*it].v2, v3 = faces[*it].v3;

		glm::dvec3 temp = glm::dvec3(x.x, x.y, x.z);

		glm::dvec3 vec1, vec2, n, nn;
		if (faces[*it].v1 == v) {
			vec1 = vertices[v2].position - vertices[v1].position;
			vec2 = vertices[v3].position - vertices[v1].position;
			n = glm::cross(vec1, vec2);
			vec1 = vertices[v2].position - temp;
			vec2 = vertices[v3].position - temp;
			nn = glm::cross(vec1, vec2);
		}
		else if (faces[*it].v2 == v) {
			vec1 = vertices[v1].position - vertices[v2].position;
			vec2 = vertices[v3].position - vertices[v2].position;
			n = glm::cross(vec1, vec2);
			vec1 = vertices[v1].position - temp;
			vec2 = vertices[v3].position - temp;
			nn = glm::cross(vec1, vec2);
		}
		else {
			assert(faces[*it].v3 == v);
			vec1 = vertices[v1].position - vertices[v3].position;
			vec2 = vertices[v2].position - vertices[v3].position;
			n = glm::cross(vec1, vec2);
			vec1 = vertices[v1].position - temp;
			vec2 = vertices[v2].position - temp;
			nn = glm::cross(vec1, vec2);
		}

		// Normalize the computed normals
		n = glm::normalize(n);
		nn = glm::normalize(nn);

		// If the dot product of the original and new normals is less than or equal to angle_limit
		// assign a cost value of error_limit and terminate the loop
		if (glm::dot(n, nn) <= angle_limit) {
			cost = error_limit; // The error_limit is the cost to stop the iteration.
			break;
		}

	}
}

double computeCost(int u, int v, double* x, double* y, double* z) {
	// Create a 4x4 matrix, summing up the quadrics of vertices u and v

	glm::dmat4 quadricMatrix(0.0);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			quadricMatrix[i][j] = vertices[u].Quadric[i][j] + vertices[v].Quadric[i][j];
		}
	}

	quadricMatrix[3][0] = quadricMatrix[3][1] = quadricMatrix[3][2] = 0; // Set the last row of matrix A to [0, 0, 0, 1]
	quadricMatrix[3][3] = 1;

	// Define vector b as [0, 0, 0, 1] and vector x as [0, 0, 0, 0]
	glm::dvec4 v1(0.0f, 0.0f, 0.0f, 1.0f);
	glm::dvec4 v2(0.0f);

	bool judge = true;
	for (int i = 0; i < 4; i++) {
		int maxLine = i;
		// Find the max absolute value in current column
		for (int j = i + 1; j < 4; j++)
			if (fabs(quadricMatrix[j][i]) > fabs(quadricMatrix[maxLine][i])) {
				maxLine = j;
			}

		// Swap the ith row and maxLine row in A
		for (int j = i; j < 4; j++) {
			std::swap(quadricMatrix[i][j], quadricMatrix[maxLine][j]);
		}
		std::swap(v1[i], v1[maxLine]); // Swap the ith element and maxLine element in b

		double t = quadricMatrix[i][i];
		// If the diagonal element is too close to 0, break and flag as false
		if (fabs(t) < 1e-10) {
			v2 = glm::dvec4(glm::dvec3(vertices[u].position + vertices[v].position) * 0.5, 1.0);
			judge = false;
			break;
		}
		// Normalize the ith row of A and the ith element of b
		for (int j = i; j < 4; j++) {
			quadricMatrix[i][j] /= t;
		}
		v1[i] /= t;
		// Subtract multiples of the ith row from all rows below i in A and b
		for (int j = i + 1; j < 4; j++) {
			if (fabs(quadricMatrix[j][i]) > 1e-8) {
				t = quadricMatrix[j][i];
				for (int k = i; k < 4; k++) {
					quadricMatrix[j][k] -= quadricMatrix[i][k] * t;
				}
				v1[j] -= v1[i] * t;
			}
		}
	}

	if (judge) { // If the flag is true, back substitute to find the solution vector x
		for (int i = 3; i >= 0; i--) {
			v2[i] = v1[i];
			for (int k = i + 1; k < 4; k++) {
				v2[i] -= quadricMatrix[i][k] * v2[k];
			}
		}
	}

	*x = v2[0];
	*y = v2[1];
	*z = v2[2];

	double cost = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			cost += v2[i] * v2[j] * (vertices[u].Quadric[i][j] + vertices[v].Quadric[i][j]);
		}
	}

	computeMatrixCost(u, v, v2, cost);
	computeMatrixCost(v, u, v2, cost);
	return cost;
}

void updateFace(int u, int v) {
	// Facelist
	for (auto it = vertices[u].faceList.begin(); it != vertices[u].faceList.end();) {
		if (faces[*it].v1 == v || faces[*it].v2 == v || faces[*it].v3 == v) {
			int f = *it; // Current face index
			it++;
			// Remove face f from the faceList of all vertices associated with face f
			vertices[faces[f].v1].faceList.erase(f);
			vertices[faces[f].v2].faceList.erase(f);
			vertices[faces[f].v3].faceList.erase(f);
		}
		else {
			computeNormal(faces[*it]); // Recompute normal and d
			it++; // Move to the next face
		}
	}
}

void updateVertexAndEdge(int u, int v) {
	// Adjust vertex, face, and edge data after edge contraction.
	for (auto it = vertices[v].faceList.begin(); it != vertices[v].faceList.end(); it++) {
		if (faces[*it].v1 != u && faces[*it].v2 != u && faces[*it].v3 != u) {
			if (faces[*it].v1 == v) { // Check if v is the first vertex of the face and adjust references
				faces[*it].v1 = u; // Convert vertex v to u
				// Update the edges and original edges data structures for the vertices of the current face.
				// This involves removing references to vertex v and adding references to vertex u.
				edges[u].insert(faces[*it].v2); orig_edges[u].insert(faces[*it].v2);
				edges[faces[*it].v2].erase(v); orig_edges[faces[*it].v2].erase(v);
				edges[faces[*it].v2].insert(u); orig_edges[faces[*it].v2].insert(u);
				edges[u].insert(faces[*it].v3); orig_edges[u].insert(faces[*it].v3);
				edges[faces[*it].v3].erase(v); orig_edges[faces[*it].v3].erase(v);
				edges[faces[*it].v3].insert(u); orig_edges[faces[*it].v3].insert(u);
			}
			else if (faces[*it].v2 == v) { // Similar adjustment
				faces[*it].v2 = u;
				edges[u].insert(faces[*it].v1); orig_edges[u].insert(faces[*it].v1);
				edges[faces[*it].v1].erase(v); orig_edges[faces[*it].v1].erase(v);
				edges[faces[*it].v1].insert(u); orig_edges[faces[*it].v1].insert(u);
				edges[u].insert(faces[*it].v3); orig_edges[u].insert(faces[*it].v3);
				edges[faces[*it].v3].erase(v); orig_edges[faces[*it].v3].erase(v);
				edges[faces[*it].v3].insert(u); orig_edges[faces[*it].v3].insert(u);
			}
			else if (faces[*it].v3 == v) { // Similar adjustment
				faces[*it].v3 = u;
				edges[u].insert(faces[*it].v1); orig_edges[u].insert(faces[*it].v1);
				edges[faces[*it].v1].erase(v); orig_edges[faces[*it].v1].erase(v);
				edges[faces[*it].v1].insert(u); orig_edges[faces[*it].v1].insert(u);
				edges[u].insert(faces[*it].v2); orig_edges[u].insert(faces[*it].v2);
				edges[faces[*it].v2].erase(v); orig_edges[faces[*it].v2].erase(v);
				edges[faces[*it].v2].insert(u); orig_edges[faces[*it].v2].insert(u);
			}
			computeNormal(faces[*it]); // Recompute the normal
			vertices[u].faceList.insert(*it);
		}
	}
	computeQuadric(vertices[u]); // Recompute the quadric
}

void updateQueue(int u, int v, double dist_eps) {
	// Re-insert the edge into the priority queue with updated error.
	for (auto it = edges[u].begin(); it != edges[u].end(); it++) {
		v = *it;
		// If vertex v has been contracted before, skip it.
		if (Previous[v] != v) {
			edges[u].erase(v);
			continue;
		}
		computeQuadric(vertices[v]);
		assert(u != v);
		int new_time = ++timer[(u < v) ? std::make_pair(u, v) : std::make_pair(v, u)];
		if ((orig_edges[u].find(v) != orig_edges[u].end()) || (distance(u, v) < dist_eps)) {
			double x, y, z;
			double error = computeCost(u, v, &x, &y, &z);
			Edge_error.push((u < v) ? Edge(u, v, error, x, y, z, new_time) : Edge(v, u, error, x, y, z, new_time));
		}
	}
}

void simplify(float simp_rate, float dist_eps) {
	int u, v;
	// Initialize a priority queue with all edges, computing the edge cost based on quadric error
	for (u = 0; u < vertex_num; u++) {
		for (int v : edges[u]) {
			if (u > v) continue;  // Skip if vertices are not ordered
			// Set timer
			timer[std::make_pair(u, v)] = 1;
			double x, y, z;
			double error = computeCost(u, v, &x, &y, &z); // Compute minimum cost
			// Add edge to priority queue
			Edge_error.push(Edge(u, v, error, x, y, z, 1));
		}
	}

	int target = (int)((1 - simp_rate) * face_num) >> 1;
	double cost = 0;

	// Perform edge contractions until reaching the target number of faces
	for (int i = 0; i < target; i++) {
		// Remove edges from the queue that can no longer be contracted

		// Judge conditions
		while (!Edge_error.empty()) {
			Edge top = Edge_error.top();

			// The first vertex of the edge has been changed in the past
			if (top.e1 != Previous[top.e1]) {
				Edge_error.pop();
				continue;
			}

			// The second vertex of the edge has been changed in the past
			if (top.e2 != Previous[top.e2]) {
				Edge_error.pop();
				continue;
			}

			// Maintain boundary
			if (Boundary[top.e1] || Boundary[top.e2]) {
				Edge_error.pop();
				continue;
			}

			// error limit
			if (top.error > error_limit) {
				Edge_error.pop();
				continue;
			}

			// If the timestamp, remove the edge and move to the next iteration
			if (top.time != timer[std::make_pair(top.e1, top.e2)]) {
				Edge_error.pop();
				continue;
			}
			break;
		}

		if (Edge_error.empty()) {
			std::cout << "Stop after " << i << " iterations.";
			std::cout << "Termination condition: All edges that can be contracted have been removed." << std::endl;
			break;
		}

		Edge edge = Edge_error.top(); Edge_error.pop(); // Pop the edge with the smallest error from the queue and contract it
		cost += edge.error;
		last_cost = edge.error;
		u = edge.e1; v = edge.e2;

		Previous[v] = u; // Contract the edge with the smallest error.
		vertices[u].position = edge.position;
		edges[u].erase(v); orig_edges[u].erase(v);

		updateFace(u, v);
		updateVertexAndEdge(u, v);
		updateQueue(u, v, dist_eps);

	}
	out_cost = cost;
}

void find_close_pairs(int left, int right, int dimension, double dist_eps) {
	if (right - left <= 1) return; // Less than two vertices

	const int mid = (left + right) >> 1; // Calculate middle index.

	std::sort(Order + left, Order + right, compare);

	// Get the position of the middle vertex in the current dimension.
	const double mid_position = (dimension == 0) ? vertices[Order[mid]].position.x :
		(dimension == 1) ? vertices[Order[mid]].position.y :
		vertices[Order[mid]].position.z;

	// Start from the middle and search towards the left.
	for (int i = mid; i >= left; i--) {
		const double current_position = (dimension == 0) ? vertices[Order[i]].position.x :
			(dimension == 1) ? vertices[Order[i]].position.y :
			vertices[Order[i]].position.z;

		if (current_position + dist_eps < mid_position) break; //break the loop.

		// For each vertex on the left, start from the middle and search towards the right.
		for (int j = mid + 1; j < right; j++) {
			const double compare_position = (dimension == 0) ? vertices[Order[j]].position.x :
				(dimension == 1) ? vertices[Order[j]].position.y :
				vertices[Order[j]].position.z;

			if (compare_position - dist_eps > mid_position) break; //break the loop.

			if (distance(Order[i], Order[j]) < dist_eps) {
				edges[Order[i]].insert(Order[j]);
				edges[Order[j]].insert(Order[i]);
			}
		}
	}
	// Recursively search in the next dimension.
	const int next_dim = (dimension + 1) % 3;
	find_close_pairs(left, mid, next_dim, dist_eps);
	find_close_pairs(mid, right, next_dim, dist_eps);
}

void simplification(std::string inputModel, std::string outputModel, float simp_rate, float dist_eps) {
	clock_t start = clock();

	std::ifstream inputFile(inputModel);
	std::ofstream outputFile(outputModel);

	std::ios::sync_with_stdio(false);

	std::string line;

	// Start a loop that iterates over each line of the file
	while (std::getline(inputFile, line)) {
		std::istringstream lineStream(line);
		std::string input;
		lineStream >> input; // Extract the string from the line

		// If the input is "v", define a new vertex
		if (input == "v") {
			glm::dvec3 position;
			Vertex temp;
			lineStream >> position.x >> position.y >> position.z;

			temp.position = position;
			vertices.push_back(temp);
			vertex_num++;
		}

		// If the input is "f", define a new face
		else if (input == "f") {
			int v1, v2, v3;
			Face temp;

			// Extract the vertex indices from the line and convert to 0-based
			lineStream >> v1 >> v2 >> v3;
			v1--; v2--; v3--;

			temp.v1 = v1, temp.v2 = v2, temp.v3 = v3;

			faces.push_back(temp);
			vertices[v1].faceList.insert(face_num);
			vertices[v2].faceList.insert(face_num);
			vertices[v3].faceList.insert(face_num);
			face_num++;
		}
	}

	if (vertices.size() == 0 || faces.size() == 0) {
		std::cerr << "Error: The model has no vertices or faces." << std::endl;
		return; // Return from the function
	}

	// Output vertex and face counts from the original file
	std::cout << "==========================================================" << std::endl;
	std::cout << "The numbers of vertices from original file: " << vertices.size() << std::endl;
	std::cout << "The numbers of faces from original file: " << faces.size() << std::endl;

	edges.resize(vertex_num);
	orig_edges.resize(vertex_num);
	for (int i = 0; i < face_num; i++) {
		int u = faces[i].v1, v = faces[i].v2, w = faces[i].v3;
		assert((u != v) && (v != w) && (w != u));
		edges[u].insert(v); edges[u].insert(w);
		edges[v].insert(u); edges[v].insert(w);
		edges[w].insert(u); edges[u].insert(v);
		orig_edges[u].insert(v); orig_edges[u].insert(w);
		orig_edges[v].insert(u); orig_edges[v].insert(w);
		orig_edges[w].insert(u); orig_edges[u].insert(v);
	}

	Previous = new int[vertex_num];
	for (int i = 0; i < vertex_num; i++) {
		Previous[i] = i;
	}

	for (int i = 0; i < face_num; i++) {
		computeNormal(faces[i]);
	}
	for (int i = 0; i < vertex_num; i++) {
		computeQuadric(vertices[i]);
	}

	// Handle boundary
	Boundary = new bool[vertex_num];
	for (int i = 0; i < vertex_num; i++) Boundary[i] = false;
	for (int u = 0; u < vertex_num; u++) if (!Boundary[u])
		for (auto it = edges[u].begin(); it != edges[u].end(); it++) {
			int v = *it;
			int face_cnt = 0;
			for (auto it = vertices[u].faceList.begin(); it != vertices[u].faceList.end(); it++)
				if (faces[*it].v1 == v || faces[*it].v2 == v || faces[*it].v3 == v) face_cnt++;
			if (face_cnt < 2)
				Boundary[u] = Boundary[v] = true;
		}

	Order = new int[vertex_num];
	for (int i = 0; i < vertex_num; i++) Order[i] = i;
	find_close_pairs(0, vertex_num, 0, dist_eps);

	simplify(simp_rate, dist_eps);

	New = new int[vertex_num];
	int cnt = 0;
	int n1 = 0, n2 = 0;

	// Process vertices for the output
	for (int i = 0; i < vertex_num; i++) {
		if (Previous[i] == i) {
			New[i] = cnt++;
			//std::cout << vertices[i].position.x << vertices[i].position.y << vertices[i].position.z << std::endl;
			outputFile << "v " << std::fixed << std::setprecision(4) << vertices[i].position.x << " " << vertices[i].position.y << " " << vertices[i].position.z << "\n";
			n1++;
		}
		else {
			New[i] = -1;
		}
	}
	// Process faces for the output
	for (int i = 0; i < face_num; i++) {
		int v1 = faces[i].v1;
		int v2 = faces[i].v2;
		int v3 = faces[i].v3;

		if (Previous[v1] == v1 && Previous[v2] == v2 && Previous[v3] == v3) {
			outputFile << "f " << New[v1] + 1 << " " << New[v2] + 1 << " " << New[v3] + 1 << "\n";
			n2++;
		}
	}

	// Output results
	std::cout << "The numbers of vertices from simplified file: " << n1 << std::endl;
	std::cout << "The numbers of faces from simplified file: " << n2 << std::endl;
	std::cout << "The last cost: " << last_cost << std::endl;
	std::cout << "The total cost: " << out_cost << std::endl;

	clock_t finish = clock();

	std::cout << "The simplification is done in " << std::fixed << (double)(finish - start) / CLOCKS_PER_SEC << " seconds" << std::endl;

	delete[] Order;
	delete[] Previous;
	delete[] New;
	delete[] Boundary;
}