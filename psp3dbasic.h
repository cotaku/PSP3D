#ifndef __PSP3DBASIC_H__
#define __PSP3DBASIC_H__

#include <string.h>
#include "math.h"
#include "def.h"
#include "debug.h"

class Point3
{  
public:  
    Point3() { x = y = z = 0; w = 1.0; } 

    Point3(float xx, float yy, float zz) { x = xx; y = yy; z = zz; w = 1.0; }

    Point3(float xx, float yy, float zz, Color& c) { x = xx; y = yy; z = zz; w = 1.0; color = c; } 

    Point3(float *p) { x = p[0]; y = p[1]; z = p[2]; w = p[3]; }

    Point3(const Point3& p) { x = p.x; y = p.y; z = p.z; w = p.w; color = p.color; }

    ~Point3() {}

    inline bool operator==(const Point3& p) const
    {
        return fabs(x - p.x) <= EPSILON && fabs(y - p.y) <= EPSILON && fabs(z - p.z) <= EPSILON;
    }

    inline bool operator!=(const Point3& p) const
    {
        return fabs(x - p.x)> EPSILON || fabs(y - p.y) > EPSILON || fabs(z - p.z) > EPSILON;
    }

    inline Point3& operator=(const Point3& p)
    {
        x = p.x;
        y = p.y;
        z = p.z;
        w = p.w;
        color = p.color;
        return *this;		
    }

    inline float operator[](const int i) const 
    {
        if (i == 0) return x;
        else if (i == 1) return y;
        else if (i == 2) return z;
        else if (i == 3) return w;
        else
        {
            DebugPrint("Point3 index out of bounds for [] operator.\n");
            return 1.0;
        }
    }

    inline float& operator[](const int i) 
    {
        if (i == 0) return x;
        else if (i == 1) return y;
        else if (i == 2) return z;
        else if (i == 3) return w;
        else
        {
            DebugPrint("Point3 index out of bounds for [] operator.\n");
            //return ???;
        }
    }

    inline void Print() const{ DebugQuickPrint("(x=%f, y=%f, z=%f, w=%f)\n", x, y, z, w); }

    float x,y,z;
    float w;
    Color color;
};

class Vector3
{  
public:  
    Vector3() { x = y = z = 0; }

    Vector3(float xx, float yy, float zz) { x = xx; y = yy; z = zz; } 

    Vector3(float *v) { x = v[0]; y = v[1]; z = v[2]; }

    Vector3(const Vector3& v) { x = v.x; y = v.y; z = v.z; } 

    ~Vector3() {}

    inline bool operator==(const Vector3& v) const
    {		
        return fabs(x - v.x) <= EPSILON && fabs(y - v.y) <= EPSILON && fabs(z - v.z) <= EPSILON;
    }

    inline bool operator!=(const Vector3& v) const
    {		
        return fabs(x - v.x) > EPSILON || fabs(y - v.y) > EPSILON || fabs(z - v.z) > EPSILON;
    }

    inline Vector3& operator=(const Vector3& v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    inline float operator[](const int i) const
    { 
        if (i == 0) return x;
        else if (i == 1) return y;
        else if (i == 2) return z;
        else if (i == 3) return 0.0;
        else
        {			
            DebugPrint("Vector3 index out of bounds for [] operator.\n");
            return 0.0;
        }
    }

    inline float& operator[](const int i) 
    { 
        if (i == 0) return x;
        else if (i == 1) return y;
        else if (i == 2) return z;
        else
        {			
            DebugPrint("Vector3 index out of bounds for [] operator.\n");
            //return ???;
        }
    }

    inline void Print() const { DebugPrint("<%g,%g,%g>\n", x, y, z); }

    float x, y, z; 
};

class Matrix4 
{
public: 
    Matrix4()
    {		
        m[0] = m[5]  = m[10] = m[15] = 1.0; 
        m[1] = m[2]  = m[3]  = m[4]  = 0.0; 
        m[6] = m[7]  = m[8]  = m[9]  = 0.0; 
        m[11]= m[12] = m[13] = m[14] = 0.0; 
    } 

    Matrix4(
        const float a,  const float b, const float c, const float d, 
        const float e,  const float f, const float g, const float h, 
        const float i,  const float j, const float k, const float l, 
        const float mm, const float n, const float o, const float p)
    {				
        m[0]=a; m[4]=b; m[8]=c; m[12]=d;
        m[1]=e; m[5]=f; m[9]=g; m[13]=h;
        m[2]=i; m[6]=j; m[10]=k; m[14]=l;
        m[3]=mm; m[7]=n; m[11]=o; m[15]=p;
    }

    Matrix4(const Matrix4& m2) { memcpy(m, m2.m, 16*sizeof(float)); }

    virtual ~Matrix4() {}

    inline bool operator==(const Matrix4& m2) const
    {		
        for (int i = 0; i < 16; i++)
            if (fabs(m[i] - m2.m[i]) > EPSILON)
                return false;
        return true;
    }  

    inline bool operator!=(const Matrix4& m2) const
    {		
        for (int i = 0; i < 16; i++)
            if (fabs(m[i] - m2.m[i]) > EPSILON)
                return true;
        return false;
    }

    inline Matrix4& operator=(const Matrix4& m2)
    {		
        memcpy(m, m2.m, 16*sizeof(float));
        return *this;
    }

    inline float operator()(const int r, const int c) const { return m[c*4+r]; }

    inline float& operator()(const int r, const int c) { return m[c*4+r]; }

    inline void Print () const
    {		
        for (int r=0; r<4; r++)
        {			
            for (int c=0; c<4; c++)
                DebugPrint("%f ", m[c*4 + r]);
            DebugPrint("\n");
        }
    } 

    //! holds a 4 by 4 matrix in column major order
    float m[16];
};


inline Vector3 operator/(const Vector3& v, const float s)
{	
    return Vector3(v.x/s, v.y/s, v.z/s);
}

inline Vector3 operator*(const float s, const Vector3& v)
{	
    return Vector3(v.x*s, v.y*s, v.z*s);
}

inline Vector3 operator*(const Vector3& v, const float s)
{	
    return Vector3(v.x*s, v.y*s, v.z*s);
}

inline Vector3 operator-(const Vector3& v)
{	
    return Vector3(-v.x, -v.y, -v.z);
}

inline Point3 operator+(const Vector3& v, const Point3& p)
{	
    return Point3(p.x + v.x, p.y + v.y, p.z + v.z);
};

inline Point3 operator+(const Point3& p, const Vector3& v)
{	
    return Point3(p.x + v.x, p.y + v.y, p.z + v.z);
}

inline Vector3 operator+(const Vector3& v1, const Vector3& v2)
{	
    return Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

inline Point3 operator-(const Point3& p, const Vector3& v)
{	
    return Point3(p.x - v.x, p.y - v.y, p.z - v.z);
}

inline Vector3 operator-(const Vector3& v1, const Vector3& v2)
{	
    return Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

inline Vector3 operator-(const Point3& p1, const Point3& p2)
{	
    return Vector3(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

//! Multiply matrix and scalar, returns the new matrix
inline Matrix4 operator*(const Matrix4& m, const float& s)
{	
    return Matrix4(m(0,0) * s, m(0,1) * s, m(0,2) * s, m(0,3) * s,
        m(1,0) * s, m(1,1) * s, m(1,2) * s, m(1,3) * s,
        m(2,0) * s, m(2,1) * s, m(2,2) * s, m(2,3) * s,
        m(3,0) * s, m(3,1) * s, m(3,2) * s, m(3,3) * s);
}

//! Multiply matrix and scalar, returns the new matrix (same as above, different order)
inline Matrix4 operator*(const float& s, const Matrix4& m)
{	
    return Matrix4(m(0,0) * s, m(0,1) * s, m(0,2) * s, m(0,3) * s,
        m(1,0) * s, m(1,1) * s, m(1,2) * s, m(1,3) * s,
        m(2,0) * s, m(2,1) * s, m(2,2) * s, m(2,3) * s,
        m(3,0) * s, m(3,1) * s, m(3,2) * s, m(3,3) * s);
}

inline Point3 operator*(const Matrix4& m, const Point3& p)
{	
    float point[4];
    point[3] = m(3,0) * p[0] + m(3,1) * p[1] + m(3,2) * p[2] + m(3,3);
    point[0] = (m(0,0) * p[0] + m(0,1) * p[1] + m(0,2) * p[2] + m(0,3)) / point[3];
    point[1] = (m(1,0) * p[0] + m(1,1) * p[1] + m(1,2) * p[2] + m(1,3)) / point[3];
    point[2] = (m(2,0) * p[0] + m(2,1) * p[1] + m(2,2) * p[2] + m(2,3)) / point[3];

    return Point3(point[0], point[1], point[2]);    // Hint: Remember to homogenize the point!!!!!
}

inline void mat_times_point(const Matrix4& m, Point3* p, float* w)
{
    const Point3 newp(m.m[0]*p->x + m.m[4]*p->y + m.m[8]*p->z + m.m[12]*(*w), m.m[1]*p->x + m.m[5]*p->y + m.m[9]*p->z + m.m[13]*(*w), m.m[2]*p->x + m.m[6]*p->y + m.m[10]*p->z + m.m[14]*(*w), p->color);
    const float neww = m.m[3]*p->x + m.m[7]*p->y + m.m[11]*p->z + m.m[15]*(*w);
    (*p) = newp;
    (*w) = neww;
}

inline Vector3 operator*(const Matrix4& m, const Vector3& v)
{	
    return Vector3(m(0,0) * v[0] + m(0,1) * v[1] + m(0,2) * v[2],
        m(1,0) * v[0] + m(1,1) * v[1] + m(1,2) * v[2],
        m(2,0) * v[0] + m(2,1) * v[1] + m(2,2) * v[2]);
}

inline Matrix4 operator*(const Matrix4& m1, const Matrix4& m2)
{	
    return Matrix4(
        m1(0,0)*m2(0,0) + m1(0,1)*m2(1,0) + m1(0,2)*m2(2,0) + m1(0,3)*m2(3,0),   // m[0][0]
        m1(0,0)*m2(0,1) + m1(0,1)*m2(1,1) + m1(0,2)*m2(2,1) + m1(0,3)*m2(3,1),   // m[0][1]
        m1(0,0)*m2(0,2) + m1(0,1)*m2(1,2) + m1(0,2)*m2(2,2) + m1(0,3)*m2(3,2),   // m[0][2]
        m1(0,0)*m2(0,3) + m1(0,1)*m2(1,3) + m1(0,2)*m2(2,3) + m1(0,3)*m2(3,3),   // m[0][3]                   
        m1(1,0)*m2(0,0) + m1(1,1)*m2(1,0) + m1(1,2)*m2(2,0) + m1(1,3)*m2(3,0),   // m[1][0]
        m1(1,0)*m2(0,1) + m1(1,1)*m2(1,1) + m1(1,2)*m2(2,1) + m1(1,3)*m2(3,1),   // m[1][1]
        m1(1,0)*m2(0,2) + m1(1,1)*m2(1,2) + m1(1,2)*m2(2,2) + m1(1,3)*m2(3,2),   // m[1][2]
        m1(1,0)*m2(0,3) + m1(1,1)*m2(1,3) + m1(1,2)*m2(2,3) + m1(1,3)*m2(3,3),   // m[1][3]
        m1(2,0)*m2(0,0) + m1(2,1)*m2(1,0) + m1(2,2)*m2(2,0) + m1(2,3)*m2(3,0),   // m[2][0]
        m1(2,0)*m2(0,1) + m1(2,1)*m2(1,1) + m1(2,2)*m2(2,1) + m1(2,3)*m2(3,1),   // m[2][1]
        m1(2,0)*m2(0,2) + m1(2,1)*m2(1,2) + m1(2,2)*m2(2,2) + m1(2,3)*m2(3,2),   // m[2][2]
        m1(2,0)*m2(0,3) + m1(2,1)*m2(1,3) + m1(2,2)*m2(2,3) + m1(2,3)*m2(3,3),   // m[2][3]
        m1(3,0)*m2(0,0) + m1(3,1)*m2(1,0) + m1(3,2)*m2(2,0) + m1(3,3)*m2(3,0),   // m[3][0]
        m1(3,0)*m2(0,1) + m1(3,1)*m2(1,1) + m1(3,2)*m2(2,1) + m1(3,3)*m2(3,1),   // m[3][1]               
        m1(3,0)*m2(0,2) + m1(3,1)*m2(1,2) + m1(3,2)*m2(2,2) + m1(3,3)*m2(3,2),   // m[3][2]
        m1(3,0)*m2(0,3) + m1(3,1)*m2(1,3) + m1(3,2)*m2(2,3) + m1(3,3)*m2(3,3));  // m[3][3]
}


inline float dot(const Vector3& u, const Vector3& v)
{	
    return u.x*v.x + u.y*v.y + u.z*v.z;
}

inline Vector3 cross(const Vector3& u, const Vector3& v)
{	
    return Vector3(u.y*v.z - u.z*v.y, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x);
}

inline float length(const Vector3& v)
{	
    return 1 / rsqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

inline Vector3 normalize(Vector3 v)
{
    float rlength = rsqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    return Vector3(v.x*rlength, v.y*rlength, v.z*rlength);
}


inline Matrix4 transpose(const Matrix4& m)
{	
    return Matrix4(m(0,0), m(1,0), m(2,0), m(3,0),
        m(0,1), m(1,1), m(2,1), m(3,1),
        m(0,2), m(1,2), m(2,2), m(3,2),
        m(0,3), m(1,3), m(2,3), m(3,3));
}

inline float subDeterminant(const Matrix4 &m, int excludeRow, int excludeCol) 
{
    // Compute non-excluded row and column indices
    int row[3];
    int col[3];

    int r = 0;
    int c = 0;
    for (int i = 0; i < 4; i++)
    {		
        if (i != excludeRow)
        {			
            row[r] = i;
            r++;
        }
        if (i != excludeCol)
        {			
            col[c] = i;
            c++;
        }
    }

    // Compute the cofactors of each element in the first row
    float cofactor00 =    m(row[1],col[1]) * m(row[2],col[2])  -  m(row[1],col[2]) * m(row[2],col[1]);
    float cofactor01 = - (m(row[1],col[0]) * m(row[2],col[2])  -  m(row[1],col[2]) * m(row[2],col[0]));  
    float cofactor02 =    m(row[1],col[0]) * m(row[2],col[1])  -  m(row[1],col[1]) * m(row[2],col[0]);

    // The determinant is then the dot product of the first row and the cofactors of the first row
    return m(row[0],col[0])*cofactor00 + m(row[0],col[1])*cofactor01 + m(row[0],col[2])*cofactor02;
}

inline Matrix4 cofactor(const Matrix4 &m) 
{
    Matrix4 out;
    // We'll use i to incrementally compute -1^(r+c)
    int i = 1;
    for (int r = 0; r < 4; ++r)
    {		
        for (int c = 0; c < 4; ++c)
        {			
            // Compute the determinant of the 3x3 submatrix
            float det = subDeterminant(m, r, c);
            out(r,c) = i * det;
            i = -i;
        }
        i = -i;
    }
    return out;
}

inline float determinant(const Matrix4 &m)
{
    return m(0,0)*subDeterminant(m,0,0) - m(0,1)*subDeterminant(m,0,1) + m(0,2)*subDeterminant(m,0,2) - m(0,3)*subDeterminant(m,0,3);
}

inline Matrix4 inverse(const Matrix4 &m)
{
    if (determinant(m) != 0)
        return transpose(cofactor(m)) * (1 / determinant(m));
    return Matrix4();
}

inline Matrix4 scale_mat(const Vector3& v)
{	
    return Matrix4(
        v.x, 0,   0,   0,
        0,   v.y, 0,   0,
        0,   0,   v.z, 0,
        0,   0,   0,   1);
}

inline Matrix4 inv_scale_mat(const Vector3& v)
{	
    return inverse(scale_mat(v));
}

inline Matrix4 trans_mat(const Vector3& v)
{	
    return Matrix4(
        1, 0, 0, v.x,
        0, 1, 0, v.y,
        0, 0, 1, v.z,
        0, 0, 0, 1);
}

inline Matrix4 inv_trans_mat(const Vector3& v)
{	
    return inverse(trans_mat(v));
}

inline Matrix4 rotX_mat(const float radians)
{	
    return Matrix4(
        1, 0,            0,             0,
        0, cos(radians), -sin(radians), 0,
        0, sin(radians), cos(radians),  0,
        0, 0,            0,             1);
}

inline Matrix4 inv_rotX_mat(const float radians)
{	
    return inverse(rotX_mat(radians));
}

inline Matrix4 rotY_mat(const float radians)
{	
    return Matrix4(
        cos(radians),  0, sin(radians), 0,
        0,             1, 0,            0,
        -sin(radians), 0, cos(radians), 0,
        0,             0, 0,            1);
}

inline Matrix4 inv_rotY_mat(const float radians)
{	
    return inverse(rotY_mat(radians));
}

inline Matrix4 rotZ_mat(const float radians)
{	
    return Matrix4(
        cos(radians), -sin(radians), 0, 0,
        sin(radians), cos(radians),  0, 0,
        0,            0,             1, 0,
        0,            0,             0, 1);
}

inline Matrix4 inv_rotZ_mat(const float radians)
{	
    return inverse(rotZ_mat(radians));
}

inline Matrix4 rot_mat(const Point3& p, const Vector3& vv, const float t)
{	
    Vector3 vector = normalize(vv);
    float u = vector.x;
    float v = vector.y;
    float w = vector.z;
    float a = p.x;
    float b = p.y;
    float c = p.z;

    return Matrix4(
        u*u+(v*v+w*w)*cos(t),    u*v*(1-cos(t))-w*sin(t), u*w*(1-cos(t))+v*sin(t), (a*(v*v+w*w)-u*(b*v+c*w))*(1-cos(t))+(b*w-c*v)*sin(t),
        u*v*(1-cos(t))+w*sin(t), v*v+(u*u+w*w)*cos(t),    v*w*(1-cos(t))-u*sin(t), (b*(u*u+w*w)-v*(a*u+c*w))*(1-cos(t))+(c*u-a*w)*sin(t),
        u*w*(1-cos(t))-v*sin(t), v*w*(1-cos(t))+u*sin(t), w*w+(u*u+v*v)*cos(t),    (c*(u*u+v*v)-w*(a*u+b*v))*(1-cos(t))+(a*v-b*u)*sin(t),
        0,                       0,                       0,                       1);
}

inline Matrix4 inv_rot_mat(const Point3& p, const Vector3& v, const float a)
{	
    return inverse(rot_mat(p, v, a));
}

#endif
