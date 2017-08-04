#ifndef CICP_CONTROLLATTICE
#define CICP_CONTROLLATTICE

static const int lattice_width = 20;
static const int lattice_height = 15;

template <typename T>
bool NonRigidAdjust(T& x, T& y, const T* const cl, int width, int height)
{
  T lattice_x = x;
  T lattice_y = y;
  lattice_x = lattice_x / ((double)width) * ((double)lattice_width);
  lattice_y = lattice_y / ((double)height) * ((double)lattice_height);
  int px = *(double*)&lattice_x;
  int py = *(double*)&lattice_y;
  // test boundary
  if (px < 0 || px >= lattice_width || py < 0 || py >= lattice_height)
	return false;
  T wx = lattice_x - (double)px;
  T wy = lattice_y - (double)py;
  T w[4];
  w[0] = ((T)1. - wx) * ((T)1. - wy);
  w[1] = wx * ((T)1. - wy);
  w[2] = ((T)1. - wx) * wy;
  w[3] = wx * wy;
  int rx = px + 1;
  int ry = py + 1;
  T c[8];
  c[0] = cl[2 * (py * lattice_width + px)];
  c[1] = cl[2 * (py * lattice_width + px) + 1];
  c[2] = cl[2 * (py * lattice_width + rx)];
  c[3] = cl[2 * (py * lattice_width + rx) + 1];
  c[4] = cl[2 * (ry * lattice_width + px)];
  c[5] = cl[2 * (ry * lattice_width + px) + 1];
  c[6] = cl[2 * (ry * lattice_width + rx)];
  c[7] = cl[2 * (ry * lattice_width + rx) + 1];

  x = x + w[0] * c[0] + w[1] * c[2] + w[2] * c[4] + w[3] * c[6];
  y = y + w[0] * c[1] + w[1] * c[3] + w[2] * c[5] + w[3] * c[7];

  return true;
}

#endif