#include "btReducedSoftBodyHelpers.h"
#include "../btSoftBodyHelpers.h"
#include <iostream>
#include <string>
#include <sstream>

btReducedSoftBody* btReducedSoftBodyHelpers::CreateFromVtkFile(btSoftBodyWorldInfo& worldInfo, const char* vtk_file)
{
	std::ifstream fs;
	fs.open(vtk_file);
	btAssert(fs);

	typedef btAlignedObjectArray<int> Index;
	std::string line;
	btAlignedObjectArray<btVector3> X;
	btVector3 position;
	btAlignedObjectArray<Index> indices;
	bool reading_points = false;
	bool reading_tets = false;
	size_t n_points = 0;
	size_t n_tets = 0;
	size_t x_count = 0;
	size_t indices_count = 0;
	while (std::getline(fs, line))
	{
		std::stringstream ss(line);
		if (line.size() == (size_t)(0))
		{
		}
		else if (line.substr(0, 6) == "POINTS")
		{
			reading_points = true;
			reading_tets = false;
			ss.ignore(128, ' ');  // ignore "POINTS"
			ss >> n_points;
			X.resize(n_points);
		}
		else if (line.substr(0, 5) == "CELLS")
		{
			reading_points = false;
			reading_tets = true;
			ss.ignore(128, ' ');  // ignore "CELLS"
			ss >> n_tets;
			indices.resize(n_tets);
		}
		else if (line.substr(0, 10) == "CELL_TYPES")
		{
			reading_points = false;
			reading_tets = false;
		}
		else if (reading_points)
		{
			btScalar p;
			ss >> p;
			position.setX(p);
			ss >> p;
			position.setY(p);
			ss >> p;
			position.setZ(p);
			//printf("v %f %f %f\n", position.getX(), position.getY(), position.getZ());
			X[x_count++] = position;
		}
		else if (reading_tets)
		{
			int d;
			ss >> d;
			if (d != 4)
			{
				printf("Load deformable failed: Only Tetrahedra are supported in VTK file.\n");
				fs.close();
				return 0;
			}
			ss.ignore(128, ' ');  // ignore "4"
			Index tet;
			tet.resize(4);
			for (size_t i = 0; i < 4; i++)
			{
				ss >> tet[i];
				//printf("%d ", tet[i]);
			}
			//printf("\n");
			indices[indices_count++] = tet;
		}
	}
	btReducedSoftBody* rsb = new btReducedSoftBody(&worldInfo, n_points, &X[0], 0);

	for (int i = 0; i < n_tets; ++i)
	{
		const Index& ni = indices[i];
		rsb->appendTetra(ni[0], ni[1], ni[2], ni[3]);
		{
			rsb->appendLink(ni[0], ni[1], 0, true);
			rsb->appendLink(ni[1], ni[2], 0, true);
			rsb->appendLink(ni[2], ni[0], 0, true);
			rsb->appendLink(ni[0], ni[3], 0, true);
			rsb->appendLink(ni[1], ni[3], 0, true);
			rsb->appendLink(ni[2], ni[3], 0, true);
		}
	}

	btSoftBodyHelpers::generateBoundaryFaces(rsb);
	rsb->initializeDmInverse();
	rsb->m_tetraScratches.resize(rsb->m_tetras.size());
	rsb->m_tetraScratchesTn.resize(rsb->m_tetras.size());
	printf("Nodes:  %u\r\n", rsb->m_nodes.size());
	printf("Links:  %u\r\n", rsb->m_links.size());
	printf("Faces:  %u\r\n", rsb->m_faces.size());
	printf("Tetras: %u\r\n", rsb->m_tetras.size());

	fs.close();
	return rsb;
}

// read in binary files
void btReducedSoftBodyHelpers::readBinary(btAlignedObjectArray<btScalar>& vec, 
																	 const unsigned int n_start, 				// starting index
																	 const unsigned int n_modes, 				// #entries read
																	 const unsigned int n_full,					// array size
																	 const char* file)
{
	std::ifstream f_in(file, std::ios::in | std::ios::binary);
	// first get size
	unsigned int size;
	f_in.read((char*)&size, sizeof(uint32_t));
	btAssert(size == n_full);

	// read data
	vec.resize(n_modes);
	double temp;
	for (unsigned int i = 0; i < n_start + n_modes; ++i)
	{
		f_in.read((char*)&temp, sizeof(double));
		if (i >= n_start) 
			vec[i - n_start] = btScalar(temp);
	}
  f_in.close();
}

void btReducedSoftBodyHelpers::readBinaryMat(btSoftBody::tDenseMatrix& mat, 
																			const unsigned int n_start, 		// starting mode index
																			const unsigned int n_modes, 		// #modes, outer array size
																			const unsigned int n_full, 			// inner array size
																			const char* file)
{
	std::ifstream f_in(file, std::ios::in | std::ios::binary);
	// first get size
	unsigned int v_size;
	f_in.read((char*)&v_size, sizeof(uint32_t));
	btAssert(v_size == n_full * n_full);

	// read data
	mat.resize(n_modes);
	for (int i = 0; i < n_start + n_modes; ++i) 
	{
		for (int j = 0; j < n_full; ++j)
		{
			double temp;
			f_in.read((char*)&temp, sizeof(double));

			if (i >= n_start && j >= n_start && i < n_start + n_modes && j < n_start + n_modes)
			{
				if (mat[i - n_start].size() != n_modes)
					mat[i - n_start].resize(n_modes);
				mat[i - n_start][j - n_start] = btScalar(temp);
			}
		}
	}
  f_in.close();
}

void btReducedSoftBodyHelpers::readBinaryModes(btSoftBody::tDenseMatrix& mat, 
																				const unsigned int n_start, 		// starting mode index
																				const unsigned int n_modes, 		// #modes, outer array size
																				const unsigned int n_full, 			// inner array size
																				const char* file)
{
	std::ifstream f_in(file, std::ios::in | std::ios::binary);
	// first get size
	unsigned int v_size;
	f_in.read((char*)&v_size, sizeof(uint32_t));
	btAssert(v_size == n_full * n_full);

	// read data
	mat.resize(n_modes);
	for (int i = 0; i < n_start + n_modes; ++i) 
	{
		for (int j = 0; j < n_full; ++j)
		{
			double temp;
			f_in.read((char*)&temp, sizeof(double));

			if (i >= n_start)
			{
				if (mat[i - n_start].size() != n_full)
					mat[i - n_start].resize(n_full);
				mat[i - n_start][j] = btScalar(temp);
			}
		}
	}
  f_in.close();
}