#ifndef __CUBE_H__
#define __CUBE_H__

#include "def.h"

class Cube
{
public:
    Cube(float edgelen) : _edgelen(edgelen) { GeneratePoints(); }

    inline void GeneratePoints()
    {
        float d = 0.5 * _edgelen;
        _points[0] = Point3(-d, d, d);
        _points[1] = Point3(-d, -d, d);
        _points[2] = Point3(d, d, d);
        _points[3] = Point3(d, -d, d);
        _points[4] = Point3(d, d, -d);
        _points[5] = Point3(d, -d, -d);
        _points[6] = Point3(-d, d, -d);
        _points[7] = Point3(-d, -d, -d);

        _points[0].color = ColorRed;
        _points[1].color = ColorGreen;
        _points[2].color = ColorBlue;
        _points[3].color = ColorYellow;
        _points[4].color = ColorGray;
        _points[5].color = ColorPurple;
        _points[6].color = ColorOrange;
        _points[7].color = ColorWhite;

        _indices[0] = 0;
        _indices[1] = 1;
        _indices[2] = 2;
        _indices[3] = 2;
        _indices[4] = 1;
        _indices[5] = 3;

        _indices[6] = 2;
        _indices[7] = 3;
        _indices[8] = 4;
        _indices[9] = 4;
        _indices[10] = 3;
        _indices[11] = 5;

        _indices[12] = 4;
        _indices[13] = 5;
        _indices[14] = 7;
        _indices[15] = 4;
        _indices[16] = 7;
        _indices[17] = 6;

        _indices[18] = 0;
        _indices[19] = 6;
        _indices[20] = 7;
        _indices[21] = 0;
        _indices[22] = 7;
        _indices[23] = 1;

        _indices[24] = 0;
        _indices[25] = 2;
        _indices[26] = 6;
        _indices[27] = 2;
        _indices[28] = 4;
        _indices[29] = 6;

        _indices[30] = 1;
        _indices[31] = 7;
        _indices[32] = 3;
        _indices[33] = 7;
        _indices[34] = 5;
        _indices[35] = 3;
    }

    inline void AddTransformation(const Transformation transformation, const Vector3& vector, const float radian)
    {
        switch (transformation)
        {
        case TRANSFORM_SCALE:
            {
                _mat = _mat * scale_mat(vector);
                break;
            }
        case TRANSFORM_TRANSLATE:
            {
                _mat = _mat * trans_mat(vector);
                break;
            }
        case TRANSFORM_ROTATE:
            {
                _mat = _mat * rot_mat(Point3(), vector, radian);
                break;
            }
        }
    }

    inline void SetEdgelen(float edgelen) { _edgelen = edgelen; }
    inline float GetEdgelen() const { return _edgelen; }

    inline Matrix4* GetMatrix() { return &_mat; }

    inline void CopyPoints(vector<Point3>* ptr) const
    {
        ptr->clear();
        for (int i = 0; i < nPoints; i++)
            ptr->push_back(_points[i]);
    }
    inline void CopyIndices(vector<unsigned int>* ptr) const
    {
        ptr->clear();
        for (int i = 0; i < nIndices; i++)
            ptr->push_back(_indices[i]);
    }

    const static int nPoints = 8;
    const static int nIndices = 36;

private:
    float _edgelen;
    Matrix4 _mat;
    Point3 _points[nPoints];
    int _indices[nIndices];
};

#endif
