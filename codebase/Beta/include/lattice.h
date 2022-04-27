#ifndef LATTICE_IS_INCLUDED
#define LATTICE_IS_INCLUDED
/* { */

#include <vector>
#include "ysclass.h"

template <class T>
class Lattice3d
{
protected:
    int nx,ny,nz;
    std::vector <T> elem;
    YsVec3 min,max;
public:
    /*! Create a lattice with nx*ny*nz blocks.  To store nodal information, the length of the
        storage actually allocated will be (nx+1)*(ny+1)*(nz+1).  */
    void Create(int nx,int ny,int nz,const YsVec3 &min,const YsVec3 &max)
    {
		this->nx=nx;
		this->ny=ny;
		this->nz=nz;
		elem.resize((nx+1)*(ny+1)*(nz+1));
		this->min=min;
		this->max=max;
	}
    /*! Returns number of blocks in X,Y, and Z direction. */
    YSSIZE_T Nx(void) const
    {
		return nx;
	}
    YSSIZE_T Ny(void) const
    {
		return ny;
	}
    YSSIZE_T Nz(void) const
    {
		return nz;
	}
    /*! Returns the dimension of one block. */
    YsVec3 GetBlockDimension(void) const
    {
		if(0<nx && 0<ny && 0<nz)
		{
			YsVec3 diagonal=max-min;
			diagonal.DivX((double)nx);
			diagonal.DivY((double)ny);
			diagonal.DivZ((double)nz);
			return diagonal;
		}
		else
		{
			return YsVec3::Origin();
		}
	}
    /*! Returns the minimum (x,y,z) of the block at (bx,by,bz). */
    YsVec3 GetBlockPosition(int ix,int iy,int iz) const
    {
		auto dim=GetBlockDimension();
		dim.MulX((double)ix);
		dim.MulY((double)iy);
		dim.MulZ((double)iz);
		return min+dim;
	}
    /*! Returns the index of the block that encloses the given position. */
    YsVec3i GetBlockIndex(const YsVec3 &pos) const
    {
		auto d=pos-min;
		auto blockDim=GetBlockDimension();
		d.DivX(blockDim.x());
		d.DivY(blockDim.y());
		d.DivZ(blockDim.z());
		return YsVec3i((int)d.x(),(int)d.y(),(int)d.z());
	}
    /*! Returns if the block index is within the valid range.
        The lattice elements can be nodal value or cell value.  To support the nodal values,
        this class allocates (nx+1)*(ny+1)*(nz+1) elems.  Therefore, the index is valid
        and this function returns true, if:
           0<=idx.x() && idx.x()<=nx && 0<=idx.y() && idx.y()<=ny && 0<=idx.z() && idx.z()<=nz. */
    bool IsInRange(const YsVec3i idx) const
    {
		return (0<=idx.x() && idx.x()<=nx && 0<=idx.y() && idx.y()<=ny && 0<=idx.z() && idx.z()<=nz);
	}
    /*! Returns a reference to the lattice element at the index. */
    T &operator[](const YsVec3i idx)
    {
		return elem[idx.z()*(nx+1)*(ny+1)+idx.y()*(nx+1)+idx.x()];
	}
    const T &operator[](const YsVec3i idx) const
    {
		return elem[idx.z()*(nx+1)*(ny+1)+idx.y()*(nx+1)+idx.x()];
	}
};



/* } */
#endif
